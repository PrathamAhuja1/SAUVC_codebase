#!/usr/bin/env python3
"""
FIXED White Gate Navigator
Fixes depth control and adds proper state management
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class FixedGateNavigator(Node):
    def __init__(self):
        super().__init__('simple_gate_navigator')
        
        # State Definitions
        self.SUBMERGING = 0
        self.SEARCHING = 1
        self.APPROACHING = 2
        self.ALIGNING = 3
        self.PASSING = 4
        self.COMPLETED = 5
        
        # Start with submerging
        self.state = self.SUBMERGING
        self.mission_active = True
        
  
        self.TARGET_DEPTH = -0.2  
        
        # Navigation Variables
        self.gate_detected = False
        self.alignment_error = 0.0
        self.distance = 999.0
        self.current_position = None
        self.current_depth = 0.0
        self.passing_start_x = None
        
        # Thresholds
        self.alignment_threshold = 0.10
        self.approach_distance = 2.0
        self.passing_distance = 0.8
        
        # Timing
        self.state_start_time = time.time()
        
        # Subscriptions
        self.create_subscription(Bool, '/simple_gate/detected', self.gate_cb, 10)
        self.create_subscription(Float32, '/simple_gate/alignment_error', self.align_cb, 10)
        self.create_subscription(Float32, '/simple_gate/distance', self.dist_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_cb, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Control Loop (20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*60)
        self.get_logger().info('✅ FIXED Gate Navigator Initialized')
        self.get_logger().info(f'   Target Depth: {self.TARGET_DEPTH:.2f}m')
        self.get_logger().info('='*60)
    
    def gate_cb(self, msg):
        self.gate_detected = msg.data
    
    def align_cb(self, msg):
        self.alignment_error = msg.data
    
    def dist_cb(self, msg):
        self.distance = msg.data
    
    def odom_cb(self, msg):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    def get_depth_velocity(self):
        """
        FIXED: Proper depth control with correct sign
        Negative z_velocity = move DOWN
        Positive z_velocity = move UP
        """
        if self.current_position is None:
            return 0.0

        current_depth = self.current_depth
        depth_error = self.TARGET_DEPTH - current_depth
        
        # Deadband to prevent jitter
        if abs(depth_error) < 0.08:
            return 0.0
        
        # P-control with clamping
        # If current is -0.2 and target is -0.5, error is -0.3 (need to go DOWN)
        # Negative error -> negative velocity -> DOWN ✓
        control_signal = depth_error * 1.8
        
        # Safety clamp
        return max(-0.6, min(control_signal, 0.6))

    def control_loop(self):
        """Main control loop"""
        
        if not self.mission_active or self.current_position is None:
            self.cmd_vel_pub.publish(Twist())
            return

        # Always apply depth control
        depth_cmd = self.get_depth_velocity()
        
        msg = Twist()
        msg.linear.z = float(depth_cmd)

        # State Machine
        if self.state == self.SUBMERGING:
            # Get to target depth before starting mission
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            depth_error = abs(self.TARGET_DEPTH - self.current_depth)
            
            if depth_error < 0.15:
                elapsed = time.time() - self.state_start_time
                if elapsed > 2.0:  # Stable for 2 seconds
                    self.get_logger().info(f'✅ Depth achieved: {self.current_depth:.2f}m - Starting search')
                    self.transition_to(self.SEARCHING)
            
            # Log progress
            if int(time.time() - self.state_start_time) % 2 == 0:
                self.get_logger().info(
                    f'⬇️ Submerging: Current={self.current_depth:.2f}m, Target={self.TARGET_DEPTH:.2f}m',
                    throttle_duration_sec=1.9
                )

        elif self.state == self.SEARCHING:
            msg.linear.x = 0.3
            msg.angular.z = 0.0
            
            if self.gate_detected:
                self.get_logger().info(f"🎯 Gate Detected at {self.distance:.2f}m -> APPROACHING")
                self.transition_to(self.APPROACHING)

        elif self.state == self.APPROACHING:
            if not self.gate_detected:
                self.transition_to(self.SEARCHING)
            else:
                msg.linear.x = 0.4
                msg.angular.z = -1.5 * self.alignment_error
                
                if self.distance < self.approach_distance:
                    self.get_logger().info("📍 Close to Gate -> ALIGNING")
                    self.transition_to(self.ALIGNING)

        elif self.state == self.ALIGNING:
            if not self.gate_detected:
                self.transition_to(self.SEARCHING)
            else:
                msg.linear.x = 0.2
                msg.angular.z = -2.0 * self.alignment_error
                
                if self.distance < 1.2 and abs(self.alignment_error) < self.alignment_threshold:
                    self.get_logger().info("✅ Aligned & Close -> PASSING")
                    self.passing_start_x = self.current_position[0]
                    self.transition_to(self.PASSING)

        elif self.state == self.PASSING:
            # Pure forward surge
            msg.linear.x = 0.8
            msg.angular.z = 0.0
            
            if self.passing_start_x is None:
                self.passing_start_x = self.current_position[0]
            
            distance_traveled = abs(self.current_position[0] - self.passing_start_x)
            
            if distance_traveled > 3.0:
                self.get_logger().info('='*60)
                self.get_logger().info('🎉 GATE PASSED SUCCESSFULLY!')
                self.get_logger().info('='*60)
                self.transition_to(self.COMPLETED)

        elif self.state == self.COMPLETED:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            msg.linear.z = 0.0
            self.mission_active = False

        # Publish command
        self.cmd_vel_pub.publish(msg)
    
    def transition_to(self, new_state: int):
        """Transition to new state"""
        state_names = {
            self.SUBMERGING: 'SUBMERGING',
            self.SEARCHING: 'SEARCHING',
            self.APPROACHING: 'APPROACHING',
            self.ALIGNING: 'ALIGNING',
            self.PASSING: 'PASSING',
            self.COMPLETED: 'COMPLETED'
        }
        
        old_name = state_names.get(self.state, 'UNKNOWN')
        self.state = new_state
        self.state_start_time = time.time()
        new_name = state_names.get(new_state, 'UNKNOWN')
        
        self.get_logger().info(f'🔄 STATE: {old_name} → {new_name}')


def main(args=None):
    rclpy.init(args=args)
    node = FixedGateNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()