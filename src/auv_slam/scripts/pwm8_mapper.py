#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16MultiArray
import numpy as np


class PWM8Mapper(Node):
    def __init__(self):
        super().__init__('pwm8_mapper')
        
        # PWM Configuration
        self.PWM_MIN = 1300
        self.PWM_MAX = 1700
        self.PWM_RANGE = self.PWM_MAX - self.PWM_MIN
        

        # Order: [Back-Left, Front-Right, Surge-Left, Surge-Right, Back-Right, Front-Left, Sway-Left, Sway-Right]
        self.NEUTRAL = np.array([1500, 1500, 1530, 1500, 1500, 1480, 1500, 1500])
        

        self.BACK_LEFT = 0
        self.FRONT_RIGHT = 1
        self.SURGE_LEFT = 2
        self.SURGE_RIGHT = 3
        self.BACK_RIGHT = 4
        self.FRONT_LEFT = 5
        self.SWAY_LEFT = 6
        self.SWAY_RIGHT = 7
        

        self.VELOCITY_SCALE = 200.0
        
        # Control gains (fine-tune these)
        self.HEAVE_GAIN = 1.0
        self.SURGE_GAIN = 1.0
        self.SWAY_GAIN = 1.0
        self.YAW_GAIN = 0.8
        self.PITCH_GAIN = 0.6
        self.ROLL_GAIN = 0.6
        
        # Subscriptions
        self.twist_sub = self.create_subscription(
            Twist, '/rp2040/cmd_vel', self.twist_callback, 10)
        
        # Publishers
        self.pwm_pub = self.create_publisher(UInt16MultiArray, '/PWM8', 10)
        
        self.get_logger().info('='*70)
        self.get_logger().info('8-Thruster PWM Mapper Initialized')
        self.get_logger().info('  Subscribing to: /rp2040/cmd_vel')
        self.get_logger().info('  Publishing to: /PWM8')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):

        
        # Extract velocity commands
        surge = msg.linear.x * self.SURGE_GAIN
        sway = msg.linear.y * self.SWAY_GAIN
        heave = msg.linear.z * self.HEAVE_GAIN
        roll = msg.angular.x * self.ROLL_GAIN
        pitch = msg.angular.y * self.PITCH_GAIN
        yaw = msg.angular.z * self.YAW_GAIN

        pwm = self.NEUTRAL.copy().astype(float)
        

        # HEAVE CONTROL
        heave_pwm = heave * self.VELOCITY_SCALE
        
        pwm[self.BACK_LEFT] += heave_pwm
        pwm[self.FRONT_RIGHT] += heave_pwm
        pwm[self.BACK_RIGHT] += heave_pwm
        pwm[self.FRONT_LEFT] += heave_pwm
        

        # PITCH CONTROL
        # Positive pitch = nose up
        pitch_pwm = pitch * self.VELOCITY_SCALE
        
        pwm[self.FRONT_RIGHT] -= pitch_pwm
        pwm[self.FRONT_LEFT] -= pitch_pwm
        pwm[self.BACK_RIGHT] += pitch_pwm
        pwm[self.BACK_LEFT] += pitch_pwm
        

        # ROLL CONTROL
        roll_pwm = roll * self.VELOCITY_SCALE
        
        pwm[self.FRONT_LEFT] -= roll_pwm
        pwm[self.BACK_LEFT] -= roll_pwm
        pwm[self.FRONT_RIGHT] += roll_pwm
        pwm[self.BACK_RIGHT] += roll_pwm
        

        # SURGE CONTROL
        surge_pwm = surge * self.VELOCITY_SCALE
        

        pwm[self.SURGE_LEFT] += surge_pwm
        pwm[self.SURGE_RIGHT] += surge_pwm
        

        # SWAY CONTROL
        sway_pwm = sway * self.VELOCITY_SCALE
        
        pwm[self.SWAY_LEFT] -= sway_pwm
        pwm[self.SWAY_RIGHT] += sway_pwm
        

        # YAW CONTROL
        yaw_pwm = yaw * self.VELOCITY_SCALE
        
        # Surge differential for yaw
        pwm[self.SURGE_LEFT] -= yaw_pwm
        pwm[self.SURGE_RIGHT] += yaw_pwm
        pwm[self.SWAY_LEFT] += yaw_pwm
        pwm[self.SWAY_RIGHT] += yaw_pwm
        

        # CLAMP AND CONVERT TO INT
        pwm = np.clip(pwm, self.PWM_MIN, self.PWM_MAX)
        pwm = pwm.astype(np.uint16)
        

        # PUBLISH PWM VALUES
        msg_out = UInt16MultiArray()
        msg_out.data = pwm.tolist()
        self.pwm_pub.publish(msg_out)
        
        # Debug logging (throttled)
        self.get_logger().info(
            f'PWM8: [{pwm[0]}, {pwm[1]}, {pwm[2]}, {pwm[3]}, '
            f'{pwm[4]}, {pwm[5]}, {pwm[6]}, {pwm[7]}]',
            throttle_duration_sec=1.0
        )
    
    def clamp_pwm(self, value: float) -> int:
        """Clamp PWM value to valid range"""
        return int(np.clip(value, self.PWM_MIN, self.PWM_MAX))


def main(args=None):
    rclpy.init(args=args)
    node = PWM8Mapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()