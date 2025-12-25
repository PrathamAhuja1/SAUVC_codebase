#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque


class SimpleGateDetector(Node):
    def __init__(self):
        super().__init__('simple_gate_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None
        
        self.gate_width_meters = 1.5
        self.gate_height_meters = 1.5
        
        self.min_contour_area = 1000
        self.detection_history = deque(maxlen=3)
        
        self.image_sub = self.create_subscription(
            Image, '/camera_forward/image_raw', self.image_callback, 1)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera_forward/camera_info', self.cam_info_callback, 10)
        
        self.gate_detected_pub = self.create_publisher(Bool, '/simple_gate/detected', 10)
        self.alignment_pub = self.create_publisher(Float32, '/simple_gate/alignment_error', 10)
        self.distance_pub = self.create_publisher(Float32, '/simple_gate/distance', 10)
        self.center_pub = self.create_publisher(Point, '/simple_gate/center', 10)
        self.debug_pub = self.create_publisher(Image, '/simple_gate/debug_image', 10)
        
        self.get_logger().info('Simple Gate Detector initialized')
    
    def cam_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.image_width = msg.width
            self.image_height = msg.height
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1]
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
    
    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        debug_img = cv_image.copy()
        h, w = cv_image.shape[:2]
        
        _, binary = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        gate_detected = False
        alignment_error = 0.0
        estimated_distance = 999.0
        gate_center_x = w // 2
        gate_center_y = h // 2
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_contour_area:
                x, y, w_box, h_box = cv2.boundingRect(largest_contour)
                
                aspect_ratio = float(h_box) / w_box if w_box > 0 else 0
                
                if 0.7 < aspect_ratio < 1.4:
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        gate_center_x = int(M["m10"] / M["m00"])
                        gate_center_y = int(M["m01"] / M["m00"])
                        
                        gate_detected = True
                        
                        pixel_error = gate_center_x - (w / 2)
                        alignment_error = pixel_error / (w / 2)
                        
                        if w_box > 20 and h_box > 20:
                            distance_from_width = (self.gate_width_meters * self.fx) / w_box
                            distance_from_height = (self.gate_height_meters * self.fy) / h_box
                            estimated_distance = (distance_from_width + distance_from_height) / 2
                            estimated_distance = max(0.3, min(estimated_distance, 20.0))
                        
                        cv2.rectangle(debug_img, (x, y), (x+w_box, y+h_box), (0, 255, 0), 3)
                        cv2.circle(debug_img, (gate_center_x, gate_center_y), 15, (0, 255, 0), -1)
                        cv2.line(debug_img, (gate_center_x, gate_center_y), 
                                (w//2, gate_center_y), (255, 0, 0), 2)
                        
                        cv2.putText(debug_img, f"Dist: {estimated_distance:.2f}m", 
                                   (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(debug_img, f"Align: {alignment_error:+.3f}", 
                                   (x, y+h_box+25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        cv2.line(debug_img, (w//2, 0), (w//2, h), (255, 255, 0), 2)
        
        status = f"{'DETECTED' if gate_detected else 'SEARCHING'} | {estimated_distance:.2f}m"
        cv2.putText(debug_img, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 
                   (0, 255, 0) if gate_detected else (100, 100, 100), 2)
        
        self.detection_history.append(gate_detected)
        confirmed = sum(self.detection_history) >= 2
        
        self.gate_detected_pub.publish(Bool(data=confirmed))
        
        if confirmed:
            self.alignment_pub.publish(Float32(data=float(alignment_error)))
            self.distance_pub.publish(Float32(data=float(estimated_distance)))
            
            center_msg = Point()
            center_msg.x = float(gate_center_x)
            center_msg.y = float(gate_center_y)
            center_msg.z = float(estimated_distance)
            self.center_pub.publish(center_msg)
        
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGateDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()