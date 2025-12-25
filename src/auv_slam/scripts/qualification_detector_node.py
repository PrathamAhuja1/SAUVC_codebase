#!/usr/bin/env python3
"""
GEOMETRIC INFERENCE Qualification Gate Detector
Removes center locking, uses continuous geometric inference
Works reliably even when very close to gate
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class GeometricGateDetector(Node):
    def __init__(self):
        super().__init__('qualification_gate_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        # Orange HSV range for gate posts
        self.orange_lower = np.array([0, 20, 40])
        self.orange_upper = np.array([35, 255, 255])
        
        self.min_area = 300
        self.gate_detection_history = deque(maxlen=3)
        self.reverse_mode = False
        
        # Gate geometry (SAUVC specs)
        self.expected_gate_width_meters = 1.5
        self.expected_post_pixel_width = None  # Calculated from distance
        
        # Position tracking
        self.gate_x_position = 0.0
        self.current_position = None
        
        # Smoothing
        self.center_history = deque(maxlen=5)
        self.smoothing_alpha = 0.4
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1, 
            durability=DurabilityPolicy.VOLATILE
        )
        
        self.image_sub = self.create_subscription(
            Image, '/camera_forward/image_raw', self.image_callback, qos_sensor)
        
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera_forward/camera_info', self.cam_info_callback, 10)
        
        self.reverse_mode_sub = self.create_subscription(
            Bool, '/mission/reverse_mode', self.reverse_mode_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Publishers
        self.gate_detected_pub = self.create_publisher(Bool, '/qualification/gate_detected', 10)
        self.alignment_pub = self.create_publisher(Float32, '/qualification/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/qualification/estimated_distance', 10)
        self.confidence_pub = self.create_publisher(Float32, '/qualification/confidence', 10) 
        self.partial_gate_pub = self.create_publisher(Bool, '/qualification/partial_detection', 10)
        self.gate_center_pub = self.create_publisher(Point, '/qualification/center_point', 10)
        self.debug_pub = self.create_publisher(Image, '/qualification/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/qualification/status', 10)
        self.frame_position_pub = self.create_publisher(Float32, '/qualification/frame_position', 10)
        
    
    def reverse_mode_callback(self, msg: Bool):
        self.reverse_mode = msg.data
        if msg.data:
            self.center_history.clear()
            self.get_logger().info('🔄 REVERSE MODE - History cleared')
    
    def odom_callback(self, msg: Odometry):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
    
    def cam_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.image_width = msg.width
            self.image_height = msg.height
            self.fx = self.camera_matrix[0, 0]
            self.get_logger().info(f'Camera: {self.image_width}x{self.image_height}, fx={self.fx:.1f}')

    def image_callback(self, msg: Image):
        if self.camera_matrix is None or self.current_position is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError:
            return
        
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        # Detect orange posts
        orange_mask = cv2.inRange(hsv_image, self.orange_lower, self.orange_upper)
        kernel = np.ones((5, 5), np.uint8)
        orange_mask_clean = cv2.dilate(orange_mask, kernel, iterations=3)
        
        orange_contours, _ = cv2.findContours(
            orange_mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        # Extract post information
        posts = []
        for cnt in orange_contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            
            posts.append({
                'center': (cx, cy),
                'area': area,
                'bbox': (x, y, w_box, h_box),
                'x_pos': cx,
                'width_pixels': w_box
            })
        
        # Estimate distance from odometry
        current_x = self.current_position[0]
        if not self.reverse_mode:
            estimated_distance = abs(self.gate_x_position - current_x)
        else:
            estimated_distance = abs(current_x - self.gate_x_position)
        estimated_distance = max(0.5, min(estimated_distance, 15.0))
        
        # GEOMETRIC INFERENCE - Core logic
        gate_detected = False
        partial_gate = False
        alignment_error = 0.0
        gate_center_x = w // 2
        gate_center_y = h // 2
        frame_position = 0.0
        confidence = 0.0
        detection_method = "NONE"
        
        if len(posts) >= 2:
            # === CASE 1: TWO POSTS VISIBLE ===
            posts_sorted = sorted(posts, key=lambda p: p['x_pos'])
            left_post = posts_sorted[0]
            right_post = posts_sorted[1]
            
            # Direct midpoint calculation
            detected_center_x = (left_post['center'][0] + right_post['center'][0]) // 2
            detected_center_y = (left_post['center'][1] + right_post['center'][1]) // 2
            
            gate_detected = True
            partial_gate = False
            confidence = 1.0
            detection_method = "BOTH_POSTS"
            
            # Smooth center
            if len(self.center_history) > 0:
                prev_center = self.center_history[-1]
                gate_center_x = int(self.smoothing_alpha * detected_center_x + 
                                   (1 - self.smoothing_alpha) * prev_center[0])
                gate_center_y = int(self.smoothing_alpha * detected_center_y + 
                                   (1 - self.smoothing_alpha) * prev_center[1])
            else:
                gate_center_x = detected_center_x
                gate_center_y = detected_center_y
            
            self.center_history.append((gate_center_x, gate_center_y))
            
            frame_position = (gate_center_x - w/2) / (w/2)
            alignment_error = frame_position
            
            # Draw visualization
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 25, (0, 255, 255), -1)
            cv2.line(debug_img, left_post['center'], right_post['center'], (0, 255, 0), 3)
            
            for post in [left_post, right_post]:
                x, y, w_b, h_b = post['bbox']
                cv2.rectangle(debug_img, (x, y), (x+w_b, y+h_b), (255, 0, 255), 3)
            
            cv2.putText(debug_img, "BOTH POSTS", 
                       (gate_center_x - 80, gate_center_y - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
        
        elif len(posts) == 1:
            # === CASE 2: ONE POST VISIBLE - GEOMETRIC INFERENCE ===
            post = posts[0]
            post_x = post['center'][0]
            post_y = post['center'][1]
            
            gate_detected = True
            partial_gate = True
            detection_method = "ONE_POST_GEOMETRIC"
            
            # Calculate expected gate width in pixels at current distance
            if estimated_distance > 0.5:
                expected_gate_width_pixels = (self.expected_gate_width_meters * self.fx) / estimated_distance
            else:
                expected_gate_width_pixels = w * 0.6  # Fallback
            
            # Determine which post (left or right) and infer center
            frame_center = w / 2
            
            if post_x < frame_center:
                # LEFT POST visible - center should be to the right
                inferred_center_x = int(post_x + expected_gate_width_pixels / 2)
                post_label = "LEFT POST"
                confidence = 0.7
            else:
                # RIGHT POST visible - center should be to the left
                inferred_center_x = int(post_x - expected_gate_width_pixels / 2)
                post_label = "RIGHT POST"
                confidence = 0.7
            
            # Clamp inferred center to frame
            inferred_center_x = max(50, min(inferred_center_x, w - 50))
            
            # Smooth with history
            if len(self.center_history) > 0:
                prev_center = self.center_history[-1]
                gate_center_x = int(self.smoothing_alpha * inferred_center_x + 
                                   (1 - self.smoothing_alpha) * prev_center[0])
                gate_center_y = int(self.smoothing_alpha * post_y + 
                                   (1 - self.smoothing_alpha) * prev_center[1])
            else:
                gate_center_x = inferred_center_x
                gate_center_y = post_y
            
            self.center_history.append((gate_center_x, gate_center_y))
            
            frame_position = (gate_center_x - w/2) / (w/2)
            alignment_error = frame_position
            
            # Visualization
            x, y, w_b, h_b = post['bbox']
            cv2.rectangle(debug_img, (x, y), (x+w_b, y+h_b), (0, 165, 255), 3)
            cv2.circle(debug_img, (post_x, post_y), 15, (0, 165, 255), -1)
            
            # Draw inferred center
            cv2.circle(debug_img, (gate_center_x, gate_center_y), 30, (255, 165, 0), 3)
            cv2.line(debug_img, (post_x, post_y), (gate_center_x, gate_center_y), 
                    (255, 165, 0), 2, cv2.LINE_AA)
            
            cv2.putText(debug_img, f"{post_label} -> INFER", 
                       (gate_center_x - 100, gate_center_y - 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)
            cv2.putText(debug_img, f"Conf: {confidence:.2f}", 
                       (gate_center_x - 60, gate_center_y + 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)
        
        else:
            # === CASE 3: NO POSTS VISIBLE ===
            gate_detected = False
            detection_method = "NO_POSTS"
            
            # Use last known center if recent
            if len(self.center_history) > 0:
                last_center = self.center_history[-1]
                gate_center_x = last_center[0]
                gate_center_y = last_center[1]
                
                cv2.circle(debug_img, (gate_center_x, gate_center_y), 25, (100, 100, 100), 3)
                cv2.putText(debug_img, "LAST KNOWN", 
                           (gate_center_x - 80, gate_center_y - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)
        
        # Draw center line and target line
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 2)
        if gate_detected:
            cv2.line(debug_img, (gate_center_x, 0), (gate_center_x, h), (0, 255, 0), 3)
        
        # Status overlay
        status_y = 40
        cv2.putText(debug_img, f"Dist: {estimated_distance:.2f}m | {detection_method}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        status_y += 35
        
        cv2.putText(debug_img, f"Posts: {len(posts)} | Conf: {confidence:.2f}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        status_y += 35
        
        if gate_detected:
            status_color = (0, 255, 0) if not partial_gate else (255, 165, 0)
            cv2.putText(debug_img, f"Align: {alignment_error:+.3f}", 
                       (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        if self.reverse_mode:
            cv2.putText(debug_img, "REVERSE MODE", 
                       (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        
        # Publish data
        self.gate_detection_history.append(gate_detected)
        confirmed = sum(self.gate_detection_history) >= 1
        
        self.gate_detected_pub.publish(Bool(data=confirmed))
        self.partial_gate_pub.publish(Bool(data=partial_gate))
        self.confidence_pub.publish(Float32(data=confidence))
        
        if confirmed:
            self.alignment_pub.publish(Float32(data=float(alignment_error)))
            self.distance_pub.publish(Float32(data=float(estimated_distance)))
            self.frame_position_pub.publish(Float32(data=float(frame_position)))
            
            center_msg = Point()
            center_msg.x = float(gate_center_x)
            center_msg.y = float(gate_center_y)
            center_msg.z = float(estimated_distance)
            self.gate_center_pub.publish(center_msg)
            
            status_text = f"{detection_method} | Conf:{confidence:.2f} | Align:{alignment_error:+.3f}"
            self.status_pub.publish(String(data=status_text))
        
        # Debug image
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = GeometricGateDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()