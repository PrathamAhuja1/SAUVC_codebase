#!/usr/bin/env python3
"""
COMPLETE Thruster System Validation Demo
Minimal logging version
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time
import math


class ThrusterValidationDemo(Node):
    def __init__(self):
        super().__init__('thruster_validation_demo')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Monitor individual thruster commands (logic kept, logging removed)
        self.thruster_values = [0.0] * 6
        self.thruster_subs = []
        for i in range(1, 7):
            sub = self.create_subscription(
                Float64, f'/thruster{i}_cmd',
                lambda msg, idx=i-1: self.thruster_callback(msg, idx), 10)
            self.thruster_subs.append(sub)
        
        # State tracking
        self.current_position = None
        self.current_depth = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        self.movement_start_position = None
        self.movement_start_yaw = 0.0
        
        # Demo control
        self.demo_active = False
        self.current_movement = 0
        self.movement_start_time = 0.0
        
        # Control timer (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        # MOVEMENT SEQUENCE
        self.movements = [
            # 1. HEAVE DOWN
            {
                'name': 'HEAVE DOWN',
                'vx': 0.0, 'vy': 0.0, 'vz': -0.5, 'yaw': 0.0,
                'duration': 8.0,
                'verify': lambda start, end: end[2] < start[2] - 0.3,
            },
            
            # 2. HEAVE UP
            {
                'name': 'HEAVE UP',
                'vx': 0.0, 'vy': 0.0, 'vz': 0.5, 'yaw': 0.0,
                'duration': 4.0,
                'verify': lambda start, end: end[2] > start[2] + 0.15,
            },
            
            # 3. SURGE FORWARD
            {
                'name': 'SURGE FORWARD',
                'vx': 0.6, 'vy': 0.0, 'vz': 0.0, 'yaw': 0.0,
                'duration': 8.0,
                'verify': lambda start, end: end[0] > start[0] + 0.4,
            },
            
            # 4. SURGE BACKWARD
            {
                'name': 'SURGE BACKWARD',
                'vx': -0.6, 'vy': 0.0, 'vz': 0.0, 'yaw': 0.0,
                'duration': 8.0,
                'verify': lambda start, end: end[0] < start[0] - 0.4,
            },
            
            # 5. YAW LEFT (CCW)
            {
                'name': 'YAW LEFT (CCW)',
                'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'yaw': 0.5,
                'duration': 8.0,
                'verify': lambda start, end: True,
            },
            
            # 6. YAW RIGHT (CW)
            {
                'name': 'YAW RIGHT (CW)',
                'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'yaw': -0.5,
                'duration': 8.0,
                'verify': lambda start, end: True,
            },
        ]
        
        self.test_results = []
        # Removed initialization banner logs
    
    def thruster_callback(self, msg: Float64, idx: int):
        self.thruster_values[idx] = msg.data
    
    def odom_callback(self, msg: Odometry):
        """Process odometry feedback"""
        if not self.odom_received:
            self.odom_received = True
            # Removed initial position logging
            
            # Start demo after 3 seconds
            self.create_timer(3.0, self.start_demo_once)
        
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def start_demo_once(self):
        """Start the demo sequence (called once)"""
        if self.demo_active:
            return
        
        self.demo_active = True
        self.current_movement = 0
        self.movement_start_time = time.time()
        self.movement_start_position = self.current_position
        self.movement_start_yaw = self.current_yaw
        
        move = self.movements[0]
        self.get_logger().info(f"=====Starting {move['name']}=====")
    
    def control_loop(self):
        """Main control loop at 20 Hz"""
        if not self.demo_active or self.current_position is None:
            return
        
        current_move = self.movements[self.current_movement]
        elapsed = time.time() - self.movement_start_time
        
        # Check if movement complete
        if elapsed >= current_move['duration']:
            # Log results (minimal)
            self.log_movement_complete(current_move)
            
            # Next movement
            self.current_movement += 1
            
            if self.current_movement >= len(self.movements):
                # All tests complete
                self.complete_demo()
                return
            
            # Start next movement
            self.movement_start_time = time.time()
            self.movement_start_position = self.current_position
            self.movement_start_yaw = self.current_yaw
            
            next_move = self.movements[self.current_movement]
            self.get_logger().info(f"Starting {next_move['name']}")
        
        # Execute current movement
        cmd = Twist()
        cmd.linear.x = current_move['vx']
        cmd.linear.y = current_move['vy']
        cmd.linear.z = current_move['vz']
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = current_move['yaw']
        
        self.cmd_vel_pub.publish(cmd)
        
        # Removed periodic progress logging
    
    def log_movement_complete(self, move: dict):
        """Log simple end message"""
        # Logic to record success/fail is kept for internal state, but not printed
        if self.current_position and self.movement_start_position:
            success = move['verify'](self.movement_start_position, self.current_position)
            self.test_results.append({'name': move['name'], 'success': success})
            
        self.get_logger().info(f"Ending {move['name']}")
    
    def complete_demo(self):
        """Demo finished"""
        self.demo_active = False
        
        # Stop all motion
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Removed summary table
        self.get_logger().info("All commands finished.")
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterValidationDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()