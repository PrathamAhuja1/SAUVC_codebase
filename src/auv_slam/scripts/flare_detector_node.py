#!/usr/bin/env python3
"""
Enhanced Flare Detector - Gate-Filtered Version
Location: src/auv_slam/scripts/flare_detector_node.py
Detects red, yellow, and blue flares while filtering out gate posts
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


class EnhancedFlareDetector(Node):
    def __init__(self):
        super().__init__('flare_detector_node')
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        # HSV ranges for flares (80cm tall, ~1.6cm diameter)
        self.flare_hsv_ranges = {
            'red': {
                'lower1': np.array([0, 120, 70]),
                'upper1': np.array([8, 255, 255]),
                'lower2': np.array([172, 120, 70]),
                'upper2': np.array([180, 255, 255])
            },
            'yellow': {
                'lower': np.array([20, 100, 100]),
                'upper': np.array([30, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 150, 50]),
                'upper': np.array([130, 255, 255])
            }
        }
        
        # Detection parameters - TUNED FOR FLARES ONLY
        self.min_flare_area = 200          # Smaller than gate posts
        self.max_flare_area = 3000         # Upper limit to reject gate
        self.min_aspect_ratio = 8.0        # Flares are very tall/thin
        self.max_aspect_ratio = 50.0       # Upper limit
        self.min_circularity = 0.3         # Flares are somewhat circular
        
        # Detection history
        self.detection_history = {
            'red': deque(maxlen=5),
            'yellow': deque(maxlen=5),
            'blue': deque(maxlen=5)
        }
        self.min_confirmations = 3
        
        self.frame_count = 0
        self.current_depth = 0.0
        self.current_position = None
        
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera_forward/image_raw', self.image_callback, qos_sensor)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera_forward/camera_info', self.cam_info_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Publishers - Detection flags
        self.red_detected_pub = self.create_publisher(Bool, '/flare/red/detected', 10)
        self.yellow_detected_pub = self.create_publisher(Bool, '/flare/yellow/detected', 10)
        self.blue_detected_pub = self.create_publisher(Bool, '/flare/blue/detected', 10)
        
        # Publishers - Positions
        self.red_position_pub = self.create_publisher(Point, '/flare/red/position', 10)
        self.yellow_position_pub = self.create_publisher(Point, '/flare/yellow/position', 10)
        self.blue_position_pub = self.create_publisher(Point, '/flare/blue/position', 10)
        
        # Publishers - Distances
        self.red_distance_pub = self.create_publisher(Float32, '/flare/red/distance', 10)
        self.yellow_distance_pub = self.create_publisher(Float32, '/flare/yellow/distance', 10)
        self.blue_distance_pub = self.create_publisher(Float32, '/flare/blue/distance', 10)
        
        # Publishers - Alignment errors
        self.red_alignment_pub = self.create_publisher(Float32, '/flare/red/alignment_error', 10)
        self.yellow_alignment_pub = self.create_publisher(Float32, '/flare/yellow/alignment_error', 10)
        self.blue_alignment_pub = self.create_publisher(Float32, '/flare/blue/alignment_error', 10)
        
        # Debug publishers
        self.debug_pub = self.create_publisher(Image, '/flare/debug_image', 10)
        self.status_pub = self.create_publisher(String, '/flare/detection_status', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ Enhanced Flare Detector Initialized')
        self.get_logger().info('   - Filters OUT gate posts (size & aspect ratio)')
        self.get_logger().info('   - Detects flares: 80cm tall, 1.6cm diameter')
        self.get_logger().info('   - Colors: RED, YELLOW, BLUE')
        self.get_logger().info('='*70)
    
    def cam_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.image_width = msg.width
            self.image_height = msg.height
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            self.get_logger().info(f'Camera: {self.image_width}x{self.image_height}, fx={self.fx:.1f}')
    
    def odom_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
    
    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return
        
        self.frame_count += 1
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        # Detect each flare color
        flare_detections = {}
        
        for color in ['red', 'yellow', 'blue']:
            detected, position, distance, alignment = self.detect_flare_color(
                hsv_image, debug_img, color, w, h
            )
            flare_detections[color] = (detected, position, distance, alignment)
        
        # Publish detections
        self.publish_detections(flare_detections, msg.header)
        
        # Status overlay
        status_text = f"Frame {self.frame_count} | Depth: {self.current_depth:.2f}m"
        cv2.putText(debug_img, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        detection_text = []
        for color in ['red', 'yellow', 'blue']:
            if flare_detections[color][0]:
                dist = flare_detections[color][2]
                detection_text.append(f"{color.upper()}: {dist:.2f}m")
        
        if detection_text:
            status = "FLARES: " + " | ".join(detection_text)
            cv2.putText(debug_img, status, (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(debug_img, "NO FLARES DETECTED", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 2)
        
        # Draw center line
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 2)
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except CvBridgeError:
            pass
    
    def detect_flare_color(self, hsv_image, debug_img, color, width, height):
        """
        Detect a specific flare color with gate filtering
        Returns: (detected, position, distance, alignment_error)
        """
        # Create mask
        if color == 'red':
            mask1 = cv2.inRange(hsv_image, self.flare_hsv_ranges['red']['lower1'],
                               self.flare_hsv_ranges['red']['upper1'])
            mask2 = cv2.inRange(hsv_image, self.flare_hsv_ranges['red']['lower2'],
                               self.flare_hsv_ranges['red']['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv_image, self.flare_hsv_ranges[color]['lower'],
                              self.flare_hsv_ranges[color]['upper'])
        
        # Morphological operations
        kernel = np.ones((3, 3), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return False, None, 999.0, 0.0
        
        # Find best flare candidate (NOT gate post)
        best_flare = None
        best_score = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            
            # CRITICAL: Size filtering to reject gate posts
            if area < self.min_flare_area or area > self.max_flare_area:
                continue
            
            # Get bounding box
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            if w_box == 0 or h_box == 0:
                continue
            
            # CRITICAL: Aspect ratio filtering (flares are very tall/thin)
            aspect_ratio = float(h_box) / w_box
            if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
                continue
            
            # Calculate circularity
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            
            if circularity < self.min_circularity:
                continue
            
            # Score based on aspect ratio and area
            score = aspect_ratio * np.sqrt(area)
            
            if score > best_score:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    best_flare = {
                        'center': (cx, cy),
                        'bbox': (x, y, w_box, h_box),
                        'area': area,
                        'aspect': aspect_ratio,
                        'circularity': circularity,
                        'score': score
                    }
                    best_score = score
        
        if best_flare is None:
            return False, None, 999.0, 0.0
        
        # Extract flare information
        cx, cy = best_flare['center']
        x, y, w_box, h_box = best_flare['bbox']
        
        # Estimate distance using known flare height (0.8m)
        flare_height_meters = 0.8
        if h_box > 10:
            distance = (flare_height_meters * self.fy) / h_box
            distance = max(0.5, min(distance, 20.0))
        else:
            distance = 999.0
        
        # Calculate alignment error
        image_center_x = width / 2
        pixel_error = cx - image_center_x
        alignment_error = pixel_error / image_center_x
        
        # Visualization
        color_bgr = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'blue': (255, 0, 0)
        }[color]
        
        cv2.rectangle(debug_img, (x, y), (x+w_box, y+h_box), color_bgr, 3)
        cv2.circle(debug_img, (cx, cy), 10, color_bgr, -1)
        cv2.circle(debug_img, (cx, cy), 12, (255, 255, 255), 2)
        
        # Draw line to center
        cv2.line(debug_img, (cx, cy), (width//2, cy), color_bgr, 2)
        
        # Label
        label = f"{color.upper()} FLARE"
        label_y = y - 10 if y > 30 else y + h_box + 25
        cv2.putText(debug_img, label, (x, label_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
        
        # Distance and alignment
        cv2.putText(debug_img, f"Dist: {distance:.2f}m", (x, label_y + 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(debug_img, f"Align: {alignment_error:+.3f}", (x, label_y + 45),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        position = (cx, cy)
        return True, position, distance, alignment_error
    
    def publish_detections(self, detections, header):
        """Publish all detection data with temporal filtering"""
        for color in ['red', 'yellow', 'blue']:
            detected, position, distance, alignment = detections[color]
            
            # Add to history
            self.detection_history[color].append(detected)
            
            # Confirm detection (temporal filtering)
            confirmed = sum(self.detection_history[color]) >= self.min_confirmations
            
            # Publish detection flag
            if color == 'red':
                self.red_detected_pub.publish(Bool(data=confirmed))
                if confirmed and position:
                    pos_msg = Point()
                    pos_msg.x = float(position[0])
                    pos_msg.y = float(position[1])
                    pos_msg.z = 0.0
                    self.red_position_pub.publish(pos_msg)
                    self.red_distance_pub.publish(Float32(data=float(distance)))
                    self.red_alignment_pub.publish(Float32(data=float(alignment)))
            
            elif color == 'yellow':
                self.yellow_detected_pub.publish(Bool(data=confirmed))
                if confirmed and position:
                    pos_msg = Point()
                    pos_msg.x = float(position[0])
                    pos_msg.y = float(position[1])
                    pos_msg.z = 0.0
                    self.yellow_position_pub.publish(pos_msg)
                    self.yellow_distance_pub.publish(Float32(data=float(distance)))
                    self.yellow_alignment_pub.publish(Float32(data=float(alignment)))
            
            elif color == 'blue':
                self.blue_detected_pub.publish(Bool(data=confirmed))
                if confirmed and position:
                    pos_msg = Point()
                    pos_msg.x = float(position[0])
                    pos_msg.y = float(position[1])
                    pos_msg.z = 0.0
                    self.blue_position_pub.publish(pos_msg)
                    self.blue_distance_pub.publish(Float32(data=float(distance)))
                    self.blue_alignment_pub.publish(Float32(data=float(alignment)))
        
        # Status message
        detected_flares = []
        for color in ['red', 'yellow', 'blue']:
            if sum(self.detection_history[color]) >= self.min_confirmations:
                detected_flares.append(color.upper())
        
        if detected_flares:
            status = f"Detected: {', '.join(detected_flares)}"
        else:
            status = "No flares detected"
        
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedFlareDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()