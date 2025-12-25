#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class SimpleGateNavigator(Node):
    def __init__(self):
        super().__init__('simple_gate_navigator')
        
        # --- State Definitions ---
        self.SEARCHING = 0
        self.APPROACHING = 1
        self.ALIGNING = 2
        self.PASSING = 3
        self.COMPLETED = 4
        
        # --- Config ---
        self.state = self.SEARCHING
        self.mission_active = True
        self.TARGET_DEPTH = -0.65  # Target depth in meters
        
        # --- Navigation Variables ---
        self.gate_detected = False
        self.alignment_error = 0.0  # Range: -1.0 (Left) to 1.0 (Right)
        self.distance = 999.0       # Estimated distance to gate
        self.current_position = None
        self.passing_start_x = None
        
        # --- Thresholds & Speeds ---
        self.alignment_threshold = 0.10
        self.approach_distance = 2.0
        self.passing_distance = 0.8
        
        # --- Subscriptions ---
        # Logic inputs from detector
        self.create_subscription(Bool, '/simple_gate/detected', self.gate_cb, 10)
        self.create_subscription(Float32, '/simple_gate/alignment_error', self.align_cb, 10)
        self.create_subscription(Float32, '/simple_gate/distance', self.dist_cb, 10)
        
        # Odometry (Use /odometry/filtered or /dvl/odom for real robot)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_cb, 10)
        
        # --- Publishers ---
        # Velocity command (Twist)
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Control Loop (20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('✅ Simple Gate Navigator Initialized')
    
    # --- Callbacks ---
    def gate_cb(self, msg):
        self.gate_detected = msg.data
    
    def align_cb(self, msg):
        self.alignment_error = msg.data
    
    def dist_cb(self, msg):
        self.distance = msg.data
    
    def odom_cb(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )

    # --- Helper Functions ---
    def get_depth_velocity(self):
        """
        Calculates vertical velocity (z) to reach TARGET_DEPTH.
        Returns float clamped between -0.5 and 0.5 m/s.
        """
        if self.current_position is None:
            return 0.0

        current_depth = self.current_position[2]
        err = self.TARGET_DEPTH - current_depth
        
        # Deadband to prevent jitter
        if abs(err) < 0.05:
            return 0.0
            
        # P-gain of 1.5, Clamped to max 0.5 m/s safety limit
        control_signal = err * 1.5
        return max(-0.5, min(control_signal, 0.5))

    # --- Main Control Loop ---
    def control_loop(self):
        """
        Main control loop: Calculates Twist messages based on state
        Publishes to /rp2040/cmd_vel
        """
        # Safety check: Stop if mission inactive or no odometry yet
        if not self.mission_active or self.current_position is None:
            self.cmd_vel_pub.publish(Twist())
            return

        # 1. Calculate Depth Hold (Always Active)
        depth_cmd = self.get_depth_velocity()
        
        msg = Twist()
        msg.linear.z = float(depth_cmd)

        # 2. State Machine Logic
        if self.state == self.SEARCHING:
            # BEHAVIOR: Move forward slowly to find the gate
            msg.linear.x = 0.2  
            msg.angular.z = 0.0 
            
            if self.gate_detected:
                self.get_logger().info("Gate Detected! -> APPROACHING")
                self.state = self.APPROACHING

        elif self.state == self.APPROACHING:
            # BEHAVIOR: Move forward and keep the gate centered
            if not self.gate_detected:
                # If lost, go back to search
                self.state = self.SEARCHING
            else:
                msg.linear.x = 0.3
                
                # Yaw Control: Turn opposite to error to center image
                # alignment_error is -1.0 (Left) to 1.0 (Right)
                # To correct +Error (Right), turn Right (-Yaw)
                msg.angular.z = -1.0 * self.alignment_error
                
                # Transition to ALIGNING if we get close
                if self.distance < self.approach_distance:
                    self.get_logger().info("Close to Gate -> ALIGNING")
                    self.state = self.ALIGNING

        elif self.state == self.ALIGNING:
            # BEHAVIOR: Slower speed, precision alignment
            if not self.gate_detected:
                self.state = self.SEARCHING
            else:
                msg.linear.x = 0.15
                msg.angular.z = -1.5 * self.alignment_error
                
                # Transition to PASSING if very close and well aligned
                if self.distance < 1.2 and abs(self.alignment_error) < self.alignment_threshold:
                    self.get_logger().info("Aligned & Close -> PASSING")
                    self.passing_start_x = self.current_position[0]
                    self.state = self.PASSING

        elif self.state == self.PASSING:
            # BEHAVIOR: Blind forward surge to clear the gate
            if self.passing_start_x is None:
                self.passing_start_x = self.current_position[0]
            
            distance_traveled = abs(self.current_position[0] - self.passing_start_x)
            
            # Surge forward fast, lock heading
            msg.linear.x = 0.8
            msg.angular.z = 0.0 
            
            # Drive 3 meters past the start of the pass
            if distance_traveled > 3.0: 
                self.get_logger().info('✅ Gate Passed! Stopping.')
                self.state = self.COMPLETED
                self.mission_active = False

        elif self.state == self.COMPLETED:
            # Stop everything
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            msg.linear.z = 0.0

        # 3. Publish Command
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGateNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on exit
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()