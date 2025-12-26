#!/usr/bin/env python3
"""
ROBUST White Gate Navigator - Reliable Gate Passage
Properly handles depth, alignment, and ensures complete passage through gate

Key Features:
- Proper depth control matching spawn conditions
- Progressive approach with alignment checkpoint
- Position-based passage completion verification
- Handles partial gate views gracefully
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class ImprovedWhiteGateNavigator(Node):
    def __init__(self):
        super().__init__('white_gate_navigator')
        
        # ============================================================
        # STATE MACHINE DEFINITION
        # ============================================================
        self.SUBMERGING = 0      # Get to target depth
        self.SEARCHING = 1        # Find gate
        self.APPROACHING = 2      # Move toward gate (far)
        self.ALIGNING = 3         # Align at optimal distance
        self.FINAL_APPROACH = 4   # Careful approach to commit point
        self.PASSING = 5          # Full speed through gate
        self.COMPLETED = 6        # Mission complete
        
        self.state = self.SUBMERGING
        self.mission_active = True
        
        # ============================================================
        # DEPTH CONFIGURATION
        # ============================================================
        # Critical: Match the spawn depth from video_world.sdf
        # Robot spawns at z=0.2, then descends to -0.5m
        self.TARGET_DEPTH = -0.5
        self.DEPTH_DEADBAND = 0.1      # Acceptable depth error
        self.DEPTH_STABLE_TIME = 2.0   # Time to hold depth before starting
        
        # ============================================================
        # NAVIGATION THRESHOLDS
        # ============================================================
        # Distance-based state transitions
        self.APPROACH_START_DISTANCE = 5.0    # Start approaching when detected
        self.ALIGNMENT_DISTANCE = 2.0         # Stop to align at 2m
        self.FINAL_APPROACH_DISTANCE = 1.2    # Start final approach at 1.2m
        self.PASSING_DISTANCE = 0.8           # Commit to passage at 0.8m
        
        # Alignment requirements
        self.COARSE_ALIGNMENT_THRESHOLD = 0.20   # For initial approach
        self.FINE_ALIGNMENT_THRESHOLD = 0.12     # For final approach
        self.ALIGNMENT_MAX_TIME = 15.0           # Max time spent aligning
        
        # Speed configuration
        self.SEARCH_FORWARD_SPEED = 0.3
        self.SEARCH_ROTATION_SPEED = 0.2
        self.APPROACH_SPEED = 0.5
        self.FINAL_APPROACH_SPEED = 0.4
        self.PASSING_SPEED = 1.0
        
        # Control gains
        self.YAW_GAIN_COARSE = 1.5
        self.YAW_GAIN_FINE = 2.5
        self.DEPTH_GAIN = 1.2
        
        # ============================================================
        # STATE VARIABLES
        # ============================================================
        # Detection state
        self.gate_detected = False
        self.alignment_error = 0.0
        self.distance = 999.0
        
        # Position tracking
        self.current_position = None
        self.current_depth = 0.0
        self.current_yaw = 0.0
        
        # Mission tracking
        self.passing_start_position = None
        self.passing_start_x = None
        self.state_start_time = time.time()
        self.submerge_stable_time = 0.0
        self.alignment_start_time = 0.0
        self.mission_start_time = time.time()
        
        # Gate position (from video_world.sdf)
        self.GATE_X_POSITION = -0.5
        self.GATE_CLEARANCE = 2.5  # Distance past gate to confirm passage
        
        # ============================================================
        # ROS INTERFACE
        # ============================================================
        # Subscriptions
        self.create_subscription(Bool, '/simple_gate/detected', 
                                self.gate_detected_callback, 10)
        self.create_subscription(Float32, '/simple_gate/alignment_error', 
                                self.alignment_callback, 10)
        self.create_subscription(Float32, '/simple_gate/distance', 
                                self.distance_callback, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', 
                                self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Control loop at 20Hz
        self.create_timer(0.05, self.control_loop)
        
        # ============================================================
        # INITIALIZATION LOG
        # ============================================================
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 IMPROVED WHITE GATE NAVIGATOR')
        self.get_logger().info('='*70)
        self.get_logger().info(f'   Target Depth: {self.TARGET_DEPTH:.2f}m')
        self.get_logger().info(f'   Gate Position: X={self.GATE_X_POSITION:.2f}m')
        self.get_logger().info(f'   Alignment Distance: {self.ALIGNMENT_DISTANCE:.2f}m')
        self.get_logger().info(f'   Passing Speed: {self.PASSING_SPEED:.2f}m/s')
        self.get_logger().info('='*70)
    
    # ============================================================
    # CALLBACK FUNCTIONS
    # ============================================================
    
    def gate_detected_callback(self, msg: Bool):
        """Gate detection status"""
        was_detected = self.gate_detected
        self.gate_detected = msg.data
        
        if not was_detected and self.gate_detected and self.state == self.SEARCHING:
            self.get_logger().info(f'🎯 GATE DETECTED at {self.distance:.2f}m!')
    
    def alignment_callback(self, msg: Float32):
        """Alignment error from detector"""
        self.alignment_error = msg.data
    
    def distance_callback(self, msg: Float32):
        """Distance to gate from detector"""
        self.distance = msg.data
    
    def odom_callback(self, msg: Odometry):
        """Odometry data"""
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
    
    # ============================================================
    # MAIN CONTROL LOOP
    # ============================================================
    
    def control_loop(self):
        """Main control loop - executed at 20Hz"""
        
        if not self.mission_active or self.current_position is None:
            self.cmd_vel_pub.publish(Twist())
            return
        
        # Initialize command
        cmd = Twist()
        
        # Depth control (always active except during passing)
        if self.state != self.PASSING:
            cmd.linear.z = self.compute_depth_control()
        
        # State machine
        if self.state == self.SUBMERGING:
            cmd = self.handle_submerging(cmd)
        elif self.state == self.SEARCHING:
            cmd = self.handle_searching(cmd)
        elif self.state == self.APPROACHING:
            cmd = self.handle_approaching(cmd)
        elif self.state == self.ALIGNING:
            cmd = self.handle_aligning(cmd)
        elif self.state == self.FINAL_APPROACH:
            cmd = self.handle_final_approach(cmd)
        elif self.state == self.PASSING:
            cmd = self.handle_passing(cmd)
        elif self.state == self.COMPLETED:
            cmd = self.handle_completed(cmd)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
    
    # ============================================================
    # DEPTH CONTROL
    # ============================================================
    
    def compute_depth_control(self) -> float:
        """
        Compute depth control command with proper deadband
        Returns: vertical velocity command
        """
        depth_error = self.TARGET_DEPTH - self.current_depth
        
        # Deadband to prevent oscillation
        if abs(depth_error) < self.DEPTH_DEADBAND:
            return 0.0
        
        # Proportional control with saturation
        z_cmd = depth_error * self.DEPTH_GAIN
        
        # Clamp to safe limits
        return max(-0.6, min(z_cmd, 0.6))
    
    # ============================================================
    # STATE HANDLERS
    # ============================================================
    
    def handle_submerging(self, cmd: Twist) -> Twist:
        """
        Submerge to target depth and stabilize before starting mission
        """
        # Stop horizontal motion
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        depth_error = abs(self.TARGET_DEPTH - self.current_depth)
        
        if depth_error < 0.15:
            # Near target depth - check for stability
            if self.submerge_stable_time == 0.0:
                self.submerge_stable_time = time.time()
                self.get_logger().info(
                    f'📍 Approaching target depth: {self.current_depth:.2f}m'
                )
            
            # Check if stable for required duration
            stable_duration = time.time() - self.submerge_stable_time
            if stable_duration >= self.DEPTH_STABLE_TIME:
                self.get_logger().info('='*70)
                self.get_logger().info(f'✅ Depth stabilized at {self.current_depth:.2f}m')
                self.get_logger().info('🔍 Starting gate search...')
                self.get_logger().info('='*70)
                self.transition_to(self.SEARCHING)
        else:
            # Reset stability timer if depth changes
            self.submerge_stable_time = 0.0
            
            # Log progress periodically
            elapsed = time.time() - self.state_start_time
            if int(elapsed) % 2 == 0:
                self.get_logger().info(
                    f'⬇️  Submerging: {self.current_depth:.2f}m → '
                    f'{self.TARGET_DEPTH:.2f}m (error: {depth_error:.2f}m)',
                    throttle_duration_sec=1.9
                )
        
        return cmd
    
    def handle_searching(self, cmd: Twist) -> Twist:
        """
        Active search pattern - move forward with gentle rotation
        """
        if self.gate_detected and self.distance < self.APPROACH_START_DISTANCE:
            self.get_logger().info('='*70)
            self.get_logger().info(f'🎯 Gate Found at {self.distance:.2f}m')
            self.get_logger().info('   → Starting approach')
            self.get_logger().info('='*70)
            self.transition_to(self.APPROACHING)
            return cmd
        
        # Move forward while searching
        cmd.linear.x = self.SEARCH_FORWARD_SPEED
        
        # Gentle sweep pattern
        elapsed = time.time() - self.state_start_time
        sweep_phase = (elapsed % 10.0) / 10.0  # 10 second cycle
        
        if sweep_phase < 0.5:
            cmd.angular.z = self.SEARCH_ROTATION_SPEED
        else:
            cmd.angular.z = -self.SEARCH_ROTATION_SPEED
        
        # Log search status
        if int(elapsed) % 3 == 0:
            direction = "LEFT" if sweep_phase < 0.5 else "RIGHT"
            self.get_logger().info(
                f'🔍 Searching ({direction})... {elapsed:.0f}s',
                throttle_duration_sec=2.9
            )
        
        return cmd
    
    def handle_approaching(self, cmd: Twist) -> Twist:
        """
        Approach gate with light corrections until alignment distance
        """
        if not self.gate_detected:
            self.get_logger().warn('⚠️  Gate lost - returning to search')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Check if reached alignment distance
        if self.distance <= self.ALIGNMENT_DISTANCE:
            self.get_logger().info('='*70)
            self.get_logger().info(f'📍 Reached alignment point ({self.distance:.2f}m)')
            self.get_logger().info('   → Starting alignment')
            self.get_logger().info('='*70)
            self.transition_to(self.ALIGNING)
            return cmd
        
        # Approach with speed based on distance
        if self.distance > 3.5:
            cmd.linear.x = self.APPROACH_SPEED
        else:
            cmd.linear.x = self.APPROACH_SPEED * 0.8
        
        # Light yaw correction to keep gate in view
        cmd.angular.z = -self.alignment_error * self.YAW_GAIN_COARSE
        
        # Log approach progress
        if int((time.time() - self.state_start_time) * 2) % 2 == 0:
            self.get_logger().info(
                f'➡️  Approaching: {self.distance:.2f}m, '
                f'align: {self.alignment_error:+.3f}',
                throttle_duration_sec=0.9
            )
        
        return cmd
    
    def handle_aligning(self, cmd: Twist) -> Twist:
        """
        Precise alignment at optimal distance before final approach
        """
        if not self.gate_detected:
            self.get_logger().warn('⚠️  Gate lost during alignment')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Initialize alignment timer
        if self.alignment_start_time == 0.0:
            self.alignment_start_time = time.time()
            self.get_logger().info('🎯 Starting precision alignment...')
        
        alignment_elapsed = time.time() - self.alignment_start_time
        
        # Check for timeout
        if alignment_elapsed > self.ALIGNMENT_MAX_TIME:
            self.get_logger().warn('⏰ Alignment timeout - proceeding anyway')
            self.alignment_start_time = 0.0
            self.transition_to(self.FINAL_APPROACH)
            return cmd
        
        # Check if well aligned
        is_aligned = abs(self.alignment_error) < self.COARSE_ALIGNMENT_THRESHOLD
        
        if is_aligned:
            self.get_logger().info('='*70)
            self.get_logger().info(
                f'✅ ALIGNED! (error: {self.alignment_error:+.3f}, '
                f'time: {alignment_elapsed:.1f}s)'
            )
            self.get_logger().info('   → Starting final approach')
            self.get_logger().info('='*70)
            self.alignment_start_time = 0.0
            self.transition_to(self.FINAL_APPROACH)
            return cmd
        
        # Alignment strategy: stronger rotation, minimal forward
        alignment_quality = abs(self.alignment_error)
        
        if alignment_quality > 0.15:
            # Far from aligned - strong rotation
            cmd.linear.x = 0.1
            cmd.angular.z = -self.alignment_error * self.YAW_GAIN_FINE
            status = "COARSE"
        else:
            # Close to aligned - fine tuning
            cmd.linear.x = 0.2
            cmd.angular.z = -self.alignment_error * (self.YAW_GAIN_FINE * 0.8)
            status = "FINE"
        
        # Log alignment progress
        if int(alignment_elapsed * 3) % 2 == 0:
            self.get_logger().info(
                f'🔄 Aligning ({status}): error={self.alignment_error:+.3f}, '
                f't={alignment_elapsed:.1f}s',
                throttle_duration_sec=0.6
            )
        
        return cmd
    
    def handle_final_approach(self, cmd: Twist) -> Twist:
        """
        Careful final approach from alignment point to commit distance
        """
        if not self.gate_detected:
            self.get_logger().warn('⚠️  Gate lost - returning to search')
            self.transition_to(self.SEARCHING)
            return cmd
        
        # Check if reached passing distance
        if self.distance <= self.PASSING_DISTANCE:
            # Final alignment check before committing
            if abs(self.alignment_error) < self.FINE_ALIGNMENT_THRESHOLD:
                self.get_logger().info('='*70)
                self.get_logger().info('🚀 COMMITTING TO PASSAGE!')
                self.get_logger().info(f'   Distance: {self.distance:.2f}m')
                self.get_logger().info(f'   Alignment: {self.alignment_error:+.3f}')
                self.get_logger().info('='*70)
                self.passing_start_position = self.current_position
                self.passing_start_x = self.current_position[0]
                self.transition_to(self.PASSING)
                return cmd
            else:
                # Too misaligned - emergency correction
                self.get_logger().error(
                    f'🚨 At commit point but misaligned! '
                    f'(error: {self.alignment_error:+.3f})'
                )
                cmd.linear.x = 0.1
                cmd.angular.z = -self.alignment_error * self.YAW_GAIN_FINE * 1.5
                return cmd
        
        # Slow, controlled approach with alignment maintenance
        cmd.linear.x = self.FINAL_APPROACH_SPEED
        
        # Correct drift during approach
        if abs(self.alignment_error) > 0.08:
            cmd.linear.x *= 0.7  # Slow down if drifting
            cmd.angular.z = -self.alignment_error * self.YAW_GAIN_FINE
        else:
            cmd.angular.z = -self.alignment_error * (self.YAW_GAIN_FINE * 0.5)
        
        # Log final approach progress
        if int((time.time() - self.state_start_time) * 2) % 2 == 0:
            self.get_logger().info(
                f'🎯 Final Approach: {self.distance:.2f}m, '
                f'align: {self.alignment_error:+.3f}',
                throttle_duration_sec=0.9
            )
        
        return cmd
    
    def handle_passing(self, cmd: Twist) -> Twist:
        """
        Full speed passage through gate with position-based completion
        """
        if self.passing_start_x is None:
            self.passing_start_x = self.current_position[0]
        
        # Calculate how far past the gate we are
        current_x = self.current_position[0]
        distance_past_gate = current_x - self.GATE_X_POSITION
        
        # Check if cleared the gate with sufficient margin
        if distance_past_gate >= self.GATE_CLEARANCE:
            distance_traveled = current_x - self.passing_start_x
            
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 GATE SUCCESSFULLY PASSED!')
            self.get_logger().info(f'   Current X: {current_x:.2f}m')
            self.get_logger().info(f'   Gate X: {self.GATE_X_POSITION:.2f}m')
            self.get_logger().info(f'   Clearance: {distance_past_gate:.2f}m')
            self.get_logger().info(f'   Distance traveled: {distance_traveled:.2f}m')
            self.get_logger().info('='*70)
            
            self.transition_to(self.COMPLETED)
            return cmd
        
        # Full speed straight through
        cmd.linear.x = self.PASSING_SPEED
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0
        
        # Log passage progress
        if distance_past_gate < 0:
            status = "APPROACHING"
            progress = 0
        else:
            status = "CLEARING"
            progress = (distance_past_gate / self.GATE_CLEARANCE) * 100
        
        if int((time.time() - self.state_start_time) * 3) % 2 == 0:
            self.get_logger().info(
                f'🚀 PASSING ({status}): X={current_x:.2f}m, '
                f'{abs(distance_past_gate):.2f}m '
                f'{"past" if distance_past_gate > 0 else "before"} gate '
                f'({progress:.0f}% cleared)',
                throttle_duration_sec=0.6
            )
        
        return cmd
    
    def handle_completed(self, cmd: Twist) -> Twist:
        """
        Mission complete - stop and hold position
        """
        if not hasattr(self, '_completion_logged'):
            total_time = time.time() - self.mission_start_time
            
            self.get_logger().info('='*70)
            self.get_logger().info('🏆 MISSION COMPLETE!')
            self.get_logger().info(f'   Total mission time: {total_time:.2f}s')
            self.get_logger().info(f'   Final position: X={self.current_position[0]:.2f}m')
            self.get_logger().info('='*70)
            
            self._completion_logged = True
            self.mission_active = False
        
        # Stop all motion
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        # Maintain depth
        depth_error = self.TARGET_DEPTH - self.current_depth
        if abs(depth_error) > 0.15:
            cmd.linear.z = depth_error * 0.5
            cmd.linear.z = max(-0.4, min(cmd.linear.z, 0.4))
        else:
            cmd.linear.z = 0.0
        
        return cmd
    
    # ============================================================
    # UTILITY FUNCTIONS
    # ============================================================
    
    def transition_to(self, new_state: int):
        """Transition to new state with logging"""
        state_names = {
            self.SUBMERGING: 'SUBMERGING',
            self.SEARCHING: 'SEARCHING',
            self.APPROACHING: 'APPROACHING',
            self.ALIGNING: 'ALIGNING',
            self.FINAL_APPROACH: 'FINAL_APPROACH',
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
    node = ImprovedWhiteGateNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()