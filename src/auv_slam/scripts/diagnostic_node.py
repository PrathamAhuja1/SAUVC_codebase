#!/usr/bin/env python3
"""
Thruster Direction Diagnostic
Tests each movement direction individually to verify correct thruster mapping
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import time


class ThrusterDiagnostic(Node):
    def __init__(self):
        super().__init__('thruster_diagnostic')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Monitor thruster commands
        self.thruster_values = [0.0] * 6
        for i in range(1, 7):
            self.create_subscription(
                Float64, f'/thruster{i}_cmd',
                lambda msg, idx=i-1: self.thruster_callback(msg, idx), 10)
        
        self.current_depth = 0.0
        self.start_depth = None
        self.current_position = None
        self.start_position = None
        
        self.get_logger().info('='*70)
        self.get_logger().info('🔧 THRUSTER DIAGNOSTIC TOOL')
        self.get_logger().info('='*70)
        self.get_logger().info('This will test each movement for 3 seconds')
        self.get_logger().info('Watch the robot and thruster commands')
        self.get_logger().info('='*70)
    
    def odom_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
    
    def thruster_callback(self, msg: Float64, idx: int):
        self.thruster_values[idx] = msg.data
    
    def test_movement(self, name: str, vx=0.0, vy=0.0, vz=0.0, yaw=0.0):
        """Test a movement for 3 seconds"""
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info(f'🧪 TESTING: {name}')
        self.get_logger().info(f'   Command: vx={vx}, vy={vy}, vz={vz}, yaw={yaw}')
        self.get_logger().info('='*70)
        
        # Record start state
        self.start_depth = self.current_depth
        self.start_position = self.current_position
        
        # Execute movement
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = yaw
        
        start_time = time.time()
        while (time.time() - start_time) < 3.0:
            self.cmd_vel_pub.publish(cmd)
            
            # Log thruster values once
            if (time.time() - start_time) < 0.5:
                self.get_logger().info(
                    f'Thrusters: T1={self.thruster_values[0]:.1f}, '
                    f'T2={self.thruster_values[1]:.1f}, '
                    f'T3={self.thruster_values[2]:.1f}, '
                    f'T4={self.thruster_values[3]:.1f}, '
                    f'T5={self.thruster_values[4]:.1f}, '
                    f'T6={self.thruster_values[5]:.1f}',
                    throttle_duration_sec=10.0
                )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        # Stop
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Report results
        if self.start_position and self.current_position:
            dx = self.current_position[0] - self.start_position[0]
            dy = self.current_position[1] - self.start_position[1]
            dz = self.current_position[2] - self.start_position[2]
            
            self.get_logger().info('📊 RESULT:')
            self.get_logger().info(f'   ΔX = {dx:+.3f}m')
            self.get_logger().info(f'   ΔY = {dy:+.3f}m')
            self.get_logger().info(f'   ΔZ = {dz:+.3f}m (depth change)')
        
        time.sleep(2)  # Pause between tests
    
    def run_diagnostics(self):
        """Run all diagnostic tests"""
        # Wait for odometry
        while self.current_position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        time.sleep(2)
        
        # Test each direction
        self.test_movement('HEAVE DOWN (should descend)', vz=-0.4)
        self.test_movement('HEAVE UP (should ascend)', vz=0.4)
        self.test_movement('SURGE FORWARD', vx=0.5)
        self.test_movement('SURGE BACKWARD', vx=-0.5)
        self.test_movement('YAW LEFT (CCW)', yaw=0.4)
        self.test_movement('YAW RIGHT (CW)', yaw=-0.4)
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ DIAGNOSTIC COMPLETE')
        self.get_logger().info('='*70)
        self.get_logger().info('Check if movements matched expected directions')
        self.get_logger().info('If not, thruster signs need adjustment')
        self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterDiagnostic()
    
    try:
        node.run_diagnostics()
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node.cmd_vel_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()