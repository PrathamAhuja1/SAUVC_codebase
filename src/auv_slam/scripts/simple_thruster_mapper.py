#!/usr/bin/env python3
"""
FIXED Simple Thruster Mapper - SURGE INVERSION CORRECTED
Direct control with proper surge direction (Silent Version)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class SimpleThrusterMapper(Node):
    def __init__(self):
        super().__init__('simple_thruster_mapper')
        
        self.declare_parameter('max_thrust', 10.0)
        self.max_thrust = self.get_parameter('max_thrust').value
        
        # Subscriptions
        self.twist_sub = self.create_subscription(
            Twist, '/rp2040/cmd_vel', self.twist_callback, 10)
        
        # Publishers for each thruster
        self.thruster_pubs = []
        for i in range(1, 7):
            pub = self.create_publisher(Float64, f'/thruster{i}_cmd', 10)
            self.thruster_pubs.append(pub)
            
        # Removed diagnostics timer and startup logging
    
    def twist_callback(self, msg: Twist):
        """
        Direct mapping of Twist commands to thrusters
        
        BlueROV2 Configuration:
              FRONT
           T1      T2
             \  /
              \/
              /\
             /  \
           T3      T4
              BACK
        
        T5 (front vertical), T6 (back vertical) - point DOWN

        """
        surge = msg.linear.x
        sway = msg.linear.y 
        heave = msg.linear.z
        yaw = msg.angular.z

        surge = -surge
        cos45 = 0.7071

        t1 = cos45 * (surge + sway) - yaw * 2.0

        t2 = cos45 * (surge - sway) + yaw * 2.0

        t3 = cos45 * (-surge + sway) + yaw * 2.0

        t4 = cos45 * (-surge - sway) - yaw * 2.0
        

        t5 = -heave * 2.5  # Front vertical
        t6 = -heave * 2.5  # Back vertical
        
        # Apply limits and publish
        thrusts = [t1, t2, t3, t4, t5, t6]
        
        for i, thrust in enumerate(thrusts):
            # Clamp to max thrust
            thrust = max(-self.max_thrust, min(thrust, self.max_thrust))
            
            # Publish
            msg = Float64()
            msg.data = float(thrust)
            self.thruster_pubs[i].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleThrusterMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()