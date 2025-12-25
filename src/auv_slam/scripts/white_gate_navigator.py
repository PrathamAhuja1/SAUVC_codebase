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
        
        self.SEARCHING = 0
        self.APPROACHING = 1
        self.ALIGNING = 2
        self.PASSING = 3
        self.COMPLETED = 4
        
        self.state = self.SEARCHING
        
        self.gate_detected = False
        self.alignment_error = 0.0
        self.distance = 999.0
        self.current_position = None
        self.passing_start_x = None
        
        self.alignment_threshold = 0.10
        self.approach_distance = 2.0
        self.passing_distance = 0.8
        
        self.search_speed = 0.3
        self.approach_speed = 0.4
        self.align_speed = 0.2
        self.passing_speed = 0.6
        
        self.state_start_time = time.time()
        
        self.create_subscription(Bool, '/simple_gate/detected', self.gate_cb, 10)
        self.create_subscription(Float32, '/simple_gate/alignment_error', self.align_cb, 10)
        self.create_subscription(Float32, '/simple_gate/distance', self.dist_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_cb, 10)
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Simple Gate Navigator initialized')
    
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
    
    def control_loop(self):
        cmd = Twist()
        
        cmd.linear.z = self.depth_control(-1.5)
        
        if self.state == self.SEARCHING:
            cmd = self.search_behavior(cmd)
        elif self.state == self.APPROACHING:
            cmd = self.approach_behavior(cmd)
        elif self.state == self.ALIGNING:
            cmd = self.align_behavior(cmd)
        elif self.state == self.PASSING:
            cmd = self.pass_behavior(cmd)
        elif self.state == self.COMPLETED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)
    
    def depth_control(self, target_depth):
        if not self.current_position:
            return 0.0
        
        current_depth = self.current_position[2]
        depth_error = target_depth - current_depth
        
        if abs(depth_error) < 0.2:
            return 0.0
        
        z_cmd = depth_error * 1.0
        return max(-0.5, min(z_cmd, 0.5))
    
    def search_behavior(self, cmd):
        if self.gate_detected and self.distance < 999:
            self.get_logger().info(f'Gate detected at {self.distance:.2f}m')
            self.transition_to(self.APPROACHING)
            return cmd
        
        cmd.linear.x = self.search_speed
        cmd.angular.z = 0.2
        
        return cmd
    
    def approach_behavior(self, cmd):
        if not self.gate_detected:
            self.get_logger().warn('Gate lost')
            self.transition_to(self.SEARCHING)
            return cmd
        
        if self.distance <= self.approach_distance:
            self.get_logger().info('Close enough, aligning')
            self.transition_to(self.ALIGNING)
            return cmd
        
        cmd.linear.x = self.approach_speed
        cmd.angular.z = -self.alignment_error * 1.5
        
        return cmd
    
    def align_behavior(self, cmd):
        if not self.gate_detected:
            self.get_logger().warn('Gate lost during alignment')
            self.transition_to(self.SEARCHING)
            return cmd
        
        elapsed = time.time() - self.state_start_time
        
        if abs(self.alignment_error) < self.alignment_threshold:
            if self.distance <= self.passing_distance:
                self.get_logger().info('Aligned, passing through')
                self.passing_start_x = self.current_position[0] if self.current_position else 0
                self.transition_to(self.PASSING)
                return cmd
        
        if elapsed > 15.0:
            self.get_logger().warn('Alignment timeout, proceeding anyway')
            self.passing_start_x = self.current_position[0] if self.current_position else 0
            self.transition_to(self.PASSING)
            return cmd
        
        if abs(self.alignment_error) > 0.15:
            cmd.linear.x = 0.0
            cmd.angular.z = -self.alignment_error * 3.0
        else:
            cmd.linear.x = self.align_speed
            cmd.angular.z = -self.alignment_error * 2.0
        
        return cmd
    
    def pass_behavior(self, cmd):
        if self.current_position and self.passing_start_x is not None:
            distance_traveled = abs(self.current_position[0] - self.passing_start_x)
            
            if distance_traveled > 2.5:
                self.get_logger().info('Gate passed successfully')
                self.transition_to(self.COMPLETED)
                return cmd
        
        cmd.linear.x = self.passing_speed
        cmd.angular.z = 0.0
        
        return cmd
    
    def transition_to(self, new_state):
        state_names = {
            self.SEARCHING: 'SEARCHING',
            self.APPROACHING: 'APPROACHING',
            self.ALIGNING: 'ALIGNING',
            self.PASSING: 'PASSING',
            self.COMPLETED: 'COMPLETED'
        }
        
        old_name = state_names[self.state]
        self.state = new_state
        self.state_start_time = time.time()
        new_name = state_names[self.state]
        
        self.get_logger().info(f'{old_name} -> {new_name}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleGateNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()