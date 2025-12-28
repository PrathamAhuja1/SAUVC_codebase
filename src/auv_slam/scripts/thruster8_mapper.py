#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class NewThrusterMapper(Node):
    def __init__(self):
        super().__init__('new_thruster_mapper')
        
        # Parameters
        self.declare_parameter('max_thrust', 10.0)
        self.declare_parameter('vertical_gain', 2.5)
        self.declare_parameter('pitch_gain', 3.0)
        self.declare_parameter('roll_gain', 3.0)
        self.declare_parameter('yaw_surge_gain', 2.0)
        self.declare_parameter('yaw_sway_gain', 1.0)
        
        self.max_thrust = self.get_parameter('max_thrust').value
        self.vertical_gain = self.get_parameter('vertical_gain').value
        self.pitch_gain = self.get_parameter('pitch_gain').value
        self.roll_gain = self.get_parameter('roll_gain').value
        self.yaw_surge_gain = self.get_parameter('yaw_surge_gain').value
        self.yaw_sway_gain = self.get_parameter('yaw_sway_gain').value
        

        self.Lx_vert = 0.23  # fore/aft distance
        self.Ly_vert = 0.16  # lateral distance
        
        # Surge thrusters - lateral separation
        self.Ly_surge = 0.15
        
        # Sway thrusters - fore/aft separation
        self.Lx_sway_front = 0.13
        self.Lx_sway_back = 0.14
        
        # Subscriptions
        self.twist_sub = self.create_subscription(
            Twist, '/rp2040/cmd_vel', self.twist_callback, 10)
        

        self.pubs = {
            'front_right': self.create_publisher(Float64, '/front_right_thrust_cmd', 10),
            'front_left': self.create_publisher(Float64, '/front_left_thrust_cmd', 10),
            'back_right': self.create_publisher(Float64, '/back_right_thrust_cmd', 10),
            'back_left': self.create_publisher(Float64, '/back_left_thrust_cmd', 10),
            'surge_right': self.create_publisher(Float64, '/surge_right_thrust_cmd', 10),
            'surge_left': self.create_publisher(Float64, '/surge_left_thrust_cmd', 10),
            'sway_front': self.create_publisher(Float64, '/sway_front_thrust_cmd', 10),
            'sway_back': self.create_publisher(Float64, '/sway_back_thrust_cmd', 10),
        }
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ New 8-Thruster Mapper Initialized')
        self.get_logger().info('='*70)
        self.get_logger().info('Configuration:')
        self.get_logger().info('  • 4 Vertical Thrusters (corners)')
        self.get_logger().info('  • 2 Surge Thrusters (fore/aft)')
        self.get_logger().info('  • 2 Sway Thrusters (lateral)')
        self.get_logger().info('='*70)
    
    def twist_callback(self, msg: Twist):

        surge = msg.linear.x
        sway = msg.linear.y
        heave = msg.linear.z
        roll = msg.angular.x
        pitch = msg.angular.y
        yaw = msg.angular.z
        
        # ========================================================================
        # VERTICAL THRUSTERS
        # ========================================================================
        
        heave_base = heave * self.vertical_gain
        
        pitch_force = (pitch * self.pitch_gain) / self.Lx_vert

        roll_force = (roll * self.roll_gain) / self.Ly_vert

        
        thrust_front_right = heave_base - pitch_force - roll_force
        thrust_front_left = heave_base - pitch_force + roll_force
        thrust_back_right = heave_base + pitch_force - roll_force
        thrust_back_left = heave_base + pitch_force + roll_force
        
        # ========================================================================
        # SURGE THRUSTERS
        # ========================================================================
        
        yaw_surge_force = (yaw * self.yaw_surge_gain) / self.Ly_surge
        
        thrust_surge_right = surge + yaw_surge_force
        thrust_surge_left = surge - yaw_surge_force
        
        # ========================================================================
        # SWAY THRUSTERS
        # ========================================================================

        # Average moment arm for sway thrusters
        Lx_sway_avg = (self.Lx_sway_front + self.Lx_sway_back) / 2
        yaw_sway_force = (yaw * self.yaw_sway_gain) / Lx_sway_avg

        thrust_sway_front = -sway + yaw_sway_force
        thrust_sway_back = -sway - yaw_sway_force
        
        # ========================================================================
        # LIMIT AND PUBLISH ALL THRUSTERS
        # ========================================================================
        thrusts = {
            'front_right': thrust_front_right,
            'front_left': thrust_front_left,
            'back_right': thrust_back_right,
            'back_left': thrust_back_left,
            'surge_right': thrust_surge_right,
            'surge_left': thrust_surge_left,
            'sway_front': thrust_sway_front,
            'sway_back': thrust_sway_back,
        }
        
        # Apply limits and publish
        for name, thrust in thrusts.items():
            # Clamp to max thrust
            thrust_limited = max(-self.max_thrust, min(thrust, self.max_thrust))
            
            # Create and publish message
            msg_out = Float64()
            msg_out.data = float(thrust_limited)
            self.pubs[name].publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = NewThrusterMapper()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop on shutdown
        zero_msg = Float64()
        zero_msg.data = 0.0
        for pub in node.pubs.values():
            pub.publish(zero_msg)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()