#!/usr/bin/env python3
"""
FIXED White Gate Navigator - Proper Depth and State Management
All issues resolved for reliable gate passage
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
        
        # CRITICAL FIX 1: Match spawn depth (-0.5m in video_world)
        self.TARGET_DEPTH = -0.5  # Changed from -0.2 to -0.5
        
        # Navigation Variables
        self.gate_detected = False
        self.alignment_error = 0.0
        self.distance = 999.0
        self.current_position = None
        self.current_depth = 0.0
        self.passing_start_x = None
        
        # CRITICAL FIX 2: Better thresholds
        self.alignment_threshold = 0.15  # Slightly relaxed
        self.approach_distance = 2.5     # Start approaching earlier
        self.passing_distance = 1.0      # Start passing closer
        
        # Timing
        self.state_start_time = time.time()
        self.submerge_stable_time = 0.0
        
        # Subscriptions
        self.create_subscription(Bool, '/simple_gate/detected', self.gate_cb, 10)
        self.create_subscription(Float32, '/simple_gate/alignment_error', self.align_cb, 10)
        self.create_subscription(Float32, '/simple_gate/distance', self.dist_cb, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_cb, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Control Loop (20Hz)
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ FIXED Gate Navigator Initialized')
        self.get_logger().info(f'   Target Depth: {self.TARGET_DEPTH:.2f}m')
        self.get_logger().info(f'   Approach Distance: {self.approach_distance:.2f}m')
        self.get_logger().info(f'   Passing Distance: {self.passing_distance:.2f}m')
        self.get_logger().info('='*70)
    
    def gate_cb(self, msg):
        was_detected = self.gate_detected
        self.gate_detected = msg.data
        
        # Log detection changes
        if not was_detected and self.gate_detected:
            self.get_logger().info('🎯 GATE DETECTED!')
        elif was_detected and not self.gate_detected:
            self.get_logger().warn('⚠️ Gate lost from view')
    
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
        FIXED: Proper depth control with correct direction
        """
        if self.current_position is None:
            return 0.0

        depth_error = self.TARGET_DEPTH - self.current_depth
        
        # CRITICAL FIX 3: Wider deadband to prevent jitter
        if abs(depth_error) < 0.12:
            return 0.0
        
        # P-control with clamping
        control_signal = depth_error * -1.5  # Slightly reduced gain
        
        # Safety clamp
        return max(-0.5, min(control_signal, 0.5))

    def control_loop(self):
        """Main control loop"""
        
        if not self.mission_active or self.current_position is None:
            self.cmd_vel_pub.publish(Twist())
            return

        # Always apply depth control
        depth_cmd = self.get_depth_velocity()
        
        msg = Twist()
        msg.linear.z = float(depth_cmd)

        # CRITICAL FIX 4: State Machine with proper transitions
        if self.state == self.SUBMERGING:
            # Get to target depth before starting mission
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            depth_error = abs(self.TARGET_DEPTH - self.current_depth)
            
            # Check if depth is stable
            if depth_error < 0.15:
                # Start timer for stability check
                if self.submerge_stable_time == 0.0:
                    self.submerge_stable_time = time.time()
                    self.get_logger().info(f'📍 Near target depth ({self.current_depth:.2f}m), stabilizing...')
                
                # Wait for 2 seconds of stability
                stable_duration = time.time() - self.submerge_stable_time
                if stable_duration > 2.0:
                    self.get_logger().info('='*70)
                    self.get_logger().info(f'✅ Depth achieved: {self.current_depth:.2f}m')
                    self.get_logger().info('🔍 Starting gate search...')
                    self.get_logger().info('='*70)
                    self.transition_to(self.SEARCHING)
            else:
                # Reset stability timer if depth changes
                self.submerge_stable_time = 0.0
            
            # Log progress every 2 seconds
            elapsed = time.time() - self.state_start_time
            if int(elapsed) % 2 == 0:
                self.get_logger().info(
                    f'⬇️ Submerging: Current={self.current_depth:.2f}m, '
                    f'Target={self.TARGET_DEPTH:.2f}m, Error={depth_error:.2f}m',
                    throttle_duration_sec=1.9
                )

        elif self.state == self.SEARCHING:
            # CRITICAL FIX 5: Active search pattern
            msg.linear.x = 0.4   # Move forward while searching
            msg.angular.z = 0.0  # Keep straight initially
            
            if self.gate_detected:
                self.get_logger().info('='*70)
                self.get_logger().info(f"🎯 Gate Detected at {self.distance:.2f}m")
                self.get_logger().info('   → Transitioning to APPROACHING')
                self.get_logger().info('='*70)
                self.transition_to(self.APPROACHING)
            else:
                # Gentle search pattern after 5 seconds
                elapsed = time.time() - self.state_start_time
                if elapsed > 5.0:
                    # Gentle sweep
                    msg.angular.z = 0.2 * math.sin(elapsed * 0.3)
                
                # Log search status
                if int(elapsed) % 3 == 0:
                    self.get_logger().info(
                        f'🔍 Searching for gate... ({elapsed:.0f}s)',
                        throttle_duration_sec=2.9
                    )

        elif self.state == self.APPROACHING:
            if not self.gate_detected:
                self.get_logger().warn('⚠️ Gate lost - returning to search')
                self.transition_to(self.SEARCHING)
            else:
                # CRITICAL FIX 6: Speed based on distance
                if self.distance > 3.0:
                    msg.linear.x = 0.5
                elif self.distance > 2.0:
                    msg.linear.x = 0.4
                else:
                    msg.linear.x = 0.3
                
                # Proportional yaw correction
                msg.angular.z = -1.5 * self.alignment_error
                
                # Check if close enough to align
                if self.distance < self.approach_distance:
                    self.get_logger().info('='*70)
                    self.get_logger().info(f'📍 Close to Gate ({self.distance:.2f}m)')
                    self.get_logger().info('   → Starting ALIGNMENT')
                    self.get_logger().info('='*70)
                    self.transition_to(self.ALIGNING)
                else:
                    # Log approach progress
                    elapsed = time.time() - self.state_start_time
                    if int(elapsed * 2) % 2 == 0:  # Every 1 second
                        self.get_logger().info(
                            f'➡️ Approaching: {self.distance:.2f}m, '
                            f'Align: {self.alignment_error:+.3f}',
                            throttle_duration_sec=0.9
                        )

        elif self.state == self.ALIGNING:
            if not self.gate_detected:
                self.get_logger().warn('⚠️ Gate lost - returning to search')
                self.transition_to(self.SEARCHING)
            else:
                # CRITICAL FIX 7: Slow alignment with strong correction
                msg.linear.x = 0.2
                msg.angular.z = -2.5 * self.alignment_error
                
                # Check if aligned and close enough
                is_aligned = abs(self.alignment_error) < self.alignment_threshold
                is_close = self.distance < self.passing_distance
                
                if is_close and is_aligned:
                    self.get_logger().info('='*70)
                    self.get_logger().info('✅ ALIGNED & POSITIONED!')
                    self.get_logger().info(f'   Distance: {self.distance:.2f}m')
                    self.get_logger().info(f'   Alignment: {self.alignment_error:+.3f}')
                    self.get_logger().info('   → Starting PASSAGE')
                    self.get_logger().info('='*70)
                    
                    self.passing_start_x = self.current_position[0]
                    self.transition_to(self.PASSING)
                else:
                    # Log alignment status
                    elapsed = time.time() - self.state_start_time
                    if int(elapsed * 4) % 2 == 0:  # Every 0.5 seconds
                        align_status = "✓" if is_aligned else "✗"
                        dist_status = "✓" if is_close else "✗"
                        self.get_logger().info(
                            f'🎯 Aligning: {align_status} Align={self.alignment_error:+.3f}, '
                            f'{dist_status} Dist={self.distance:.2f}m',
                            throttle_duration_sec=0.4
                        )

        elif self.state == self.PASSING:
            # CRITICAL FIX 8: Full speed straight passage
            msg.linear.x = 1.0  # Maximum speed
            msg.angular.z = 0.0  # No turning during passage
            
            if self.passing_start_x is None:
                self.passing_start_x = self.current_position[0]
                self.get_logger().info('🚀 PASSAGE STARTED - FULL SPEED!')
            
            # Calculate distance traveled
            distance_traveled = abs(self.current_position[0] - self.passing_start_x)
            
            # Need to travel at least 3m to clear gate
            if distance_traveled > 3.0:
                self.get_logger().info('='*70)
                self.get_logger().info('🎉 GATE PASSED SUCCESSFULLY!')
                self.get_logger().info(f'   Distance traveled: {distance_traveled:.2f}m')
                self.get_logger().info('='*70)
                self.transition_to(self.COMPLETED)
            else:
                # Log passage progress
                if int(distance_traveled * 4) % 2 == 0:
                    progress_pct = (distance_traveled / 3.0) * 100
                    self.get_logger().info(
                        f'🚀 PASSING: {distance_traveled:.2f}m / 3.0m ({progress_pct:.0f}%)',
                        throttle_duration_sec=0.4
                    )

        elif self.state == self.COMPLETED:
            # Stop all motion
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            
            # Keep depth
            depth_error = self.TARGET_DEPTH - self.current_depth
            if abs(depth_error) > 0.15:
                msg.linear.z = depth_error * -0.5
                msg.linear.z = max(-0.4, min(msg.linear.z, 0.4))
            else:
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