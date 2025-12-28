#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray
import numpy as np
import math


class PWM6Mapper(Node):
    def __init__(self):
        super().__init__('pwm6_mapper')
        
        # PWM Configuration
        self.PWM_MIN = 1300
        self.PWM_MAX = 1700
        self.PWM_NEUTRAL = 1500
        self.PWM_RANGE = self.PWM_MAX - self.PWM_MIN  # 400
        

        self.VELOCITY_SCALE = 200.0
        
        # Control gains (fine-tune these)
        self.SURGE_GAIN = 1.0
        self.SWAY_GAIN = 1.0
        self.HEAVE_GAIN = 1.0
        self.YAW_GAIN = 0.8
        

        self.COS_45 = math.cos(math.radians(45))  # 0.7071
        
        # Thruster indices
        self.T1_FRONT_RIGHT = 0
        self.T2_FRONT_LEFT = 1
        self.T3_BACK_RIGHT = 2
        self.T4_BACK_LEFT = 3
        self.T5_VERTICAL_FRONT = 4
        self.T6_VERTICAL_BACK = 5
        
        # Subscriptions
        self.twist_sub = self.create_subscription(
            Twist, '/rp2040/cmd_vel', self.twist_callback, 10)
        
        # Publishers
        self.pwm_pub = self.create_publisher(UInt16MultiArray, '/PWM6', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('6-Thruster PWM Mapper Initialized')
        self.get_logger().info('  Configuration: 4 Vectored + 2 Vertical')
        self.get_logger().info('  Subscribing to: /rp2040/cmd_vel')
        self.get_logger().info('  Publishing to: /PWM6')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):

        # Extract and scale velocity commands
        surge = msg.linear.x * self.SURGE_GAIN
        sway = msg.linear.y * self.SWAY_GAIN
        heave = msg.linear.z * self.HEAVE_GAIN
        yaw = msg.angular.z * self.YAW_GAIN
        
        # Initialize PWM array
        pwm = np.zeros(6)
        

        # VECTORED THRUSTER MIXING (4 thrusters)
        # Thruster orientation vectors (normalized):
        # T1 (Front Right): [-0.707, -0.707] → Southeast
        # T2 (Front Left):  [-0.707, +0.707] → Southwest  
        # T3 (Back Right):  [+0.707, -0.707] → Northeast
        # T4 (Back Left):   [+0.707, +0.707] → Northwest

        
        # T1 (Front Right) - Southeast orientation
        pwm[self.T1_FRONT_RIGHT] = (
            self.COS_45 * (surge + sway) - yaw * 2.0
        ) * self.VELOCITY_SCALE
        
        # T2 (Front Left) - Southwest orientation  
        pwm[self.T2_FRONT_LEFT] = (
            self.COS_45 * (surge - sway) + yaw * 2.0
        ) * self.VELOCITY_SCALE
        
        # T3 (Back Right) - Northeast orientation
        pwm[self.T3_BACK_RIGHT] = (
            self.COS_45 * (-surge + sway) + yaw * 2.0
        ) * self.VELOCITY_SCALE
        
        # T4 (Back Left) - Northwest orientation
        pwm[self.T4_BACK_LEFT] = (
            self.COS_45 * (-surge - sway) - yaw * 2.0
        ) * self.VELOCITY_SCALE
        

        # VERTICAL THRUSTER CONTROL
        heave_pwm = -heave * self.VELOCITY_SCALE * 2.5 
        
        pwm[self.T5_VERTICAL_FRONT] = heave_pwm
        pwm[self.T6_VERTICAL_BACK] = heave_pwm
        

        # ADD NEUTRAL VALUES AND CLAMP
        pwm = pwm + self.PWM_NEUTRAL
        pwm = np.clip(pwm, self.PWM_MIN, self.PWM_MAX)
        pwm = pwm.astype(np.uint16)
        

        # PUBLISH PWM VALUES
        msg_out = UInt16MultiArray()
        msg_out.data = pwm.tolist()
        self.pwm_pub.publish(msg_out)
        
        # Debug logging (throttled)
        self.get_logger().info(
            f'PWM6: T1={pwm[0]}, T2={pwm[1]}, T3={pwm[2]}, '
            f'T4={pwm[3]}, T5={pwm[4]}, T6={pwm[5]}',
            throttle_duration_sec=1.0
        )
    
    def deadband(self, value: float, threshold: float = 0.02) -> float:
        """Apply deadband to prevent drift from small inputs"""
        return 0.0 if abs(value) < threshold else value


def main(args=None):
    rclpy.init(args=args)
    node = PWM6Mapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        neutral_msg = UInt16MultiArray()
        neutral_msg.data = [node.PWM_NEUTRAL] * 6
        node.pwm_pub.publish(neutral_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()