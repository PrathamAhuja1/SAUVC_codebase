#!/usr/bin/env python3
"""
ROBUST White Gate Navigator - WITH CENTER LOCK FOR GATE PASSAGE
Key Feature: When close to gate, locks last known center and drives forward for 10s
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import time
import math


class ImprovedWhiteGateNavigator(Node):
    def __init__(self):
        super().__init__('white_gate_navigator')
        
        # ============================================================
        # STATE MACHINE DEFINITION
        # ============================================================
        self.SUBMERGING = 0
        self.SEARCHING = 1
        self.APPROACHING = 2
        self.ALIGNING = 3
        self.FINAL_APPROACH = 4
        self.PASSING = 5          # NEW: Locks center and drives for 10s
        self.COMPLETED = 6
        
        self.state = self.SUBMERGING
        self.mission_active = True
        
        # ============================================================
        # NAVIGATION PARAMETERS
        # ============================================================
        self.TARGET_DEPTH = -0.5
        self.DEPTH_DEADBAND = 0.1
        self.DEPTH_STABLE_TIME = 2.0
        
        # Distance thresholds
        self.APPROACH_START_DISTANCE = 5.0
        self.ALIGNMENT_DISTANCE = 2.0
        self.FINAL_APPROACH_DISTANCE = 1.2
        self.PASSING_DISTANCE = 0.8  # When to enter PASSING mode
        
        # Alignment
        self.COARSE_ALIGNMENT_THRESHOLD = 0.20
        self.FINE_ALIGNMENT_THRESHOLD = 0.12
        self.ALIGNMENT_MAX_TIME = 15.0
        
        # Speeds
        self.SEARCH_FORWARD_SPEED = 0.3
        self.SEARCH_ROTATION_SPEED = 0.2
        self.APPROACH_SPEED = 0.5
        self.FINAL_APPROACH_SPEED = 0.4
        self.PASSING_SPEED = 1.0
        
        # NEW: Passing duration - guaranteed 10 seconds
        self.PASSING_DURATION = 10.0
        
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
        self.gate_center_x = 640  # Default center
        self.gate_center_y = 360
        
        # NEW: Last known gate position (for center lock)
        self.last_known_gate_center_x = None
        self.last_known_gate_center_y = None
        self.last_gate_detection_time = 0.0
        
        # Position tracking
        self.current_position = None
        self.current_depth = 0.0
        self.current_yaw = 0.0
        
        # Timing
        self.passing_start_time = 0.0
        self.state_start_time = time.time()
        self.submerge_stable_time = 0.0
        self.alignment_start_time = 0.0
        self.mission_start_time = time.time()
        
        # NEW: Locked heading for passage
        self.locked_yaw = None
        
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
        self.create_subscription(Point, '/simple_gate/center',
                                self.gate_center_callback, 10)
        self.create_subscription(Odometry, '/ground_truth/odom', 
                                self.odom_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Control loop at 20Hz
        self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🚀 GATE NAVIGATOR WITH CENTER LOCK')
        self.get_logger().info('='*70)
        self.get_logger().info(f'   Passing Duration: {self.PASSING_DURATION}s (guaranteed)')
        self.get_logger().info('   Center Lock: Enabled when close to gate')
        self.get_logger().info('='*70)
    
    # ============================================================
    # CALLBACK FUNCTIONS
    # ============================================================
    
    def gate_detected_callback(self, msg: Bool):
        """Gate detection status"""
        was_detected = self.gate_detected
        self.gate_detected = msg.data
        
        if self.gate_detected:
            self.last_gate_detection_time = time.time()
        
        if not was_detected and self.gate_detected and self.state == self.SEARCHING:
            self.get_logger().info(f'🎯 GATE DETECTED at {self.distance:.2f}m!')
    
    def alignment_callback(self, msg: Float32):
        """Alignment error from detector"""
        self.alignment_error = msg.data
    
    def distance_callback(self, msg: Float32):
        """Distance to gate from detector"""
        self.distance = msg.data
    
    def gate_center_callback(self, msg: Point):
        """Gate center position - ALWAYS UPDATE when detected"""
        if self.gate_detected:
            self.gate_center_x = int(msg.x)
            self.gate_center_y = int(msg.y)
            # Store as last known position
            self.last_known_gate_center_x = self.gate_center_x
            self.last_known_gate_center_y = self.gate_center_y
    
    def odom_callback(self, msg: Odometry):
        """Odometry data"""
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        
        # Extract yaw
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
        
        self.cmd_vel_pub.publish(cmd)
    
    # ============================================================
    # DEPTH CONTROL
    # ============================================================
    
    def compute_depth_control(self) -> float:
        """Compute depth control with deadband"""
        depth_error = self.TARGET_DEPTH - self.current_depth
        
        if abs(depth_error) < self.DEPTH_DEADBAND:
            return 0.0
        
        z_cmd = depth_error * self.DEPTH_GAIN
        return max(-0.6, min(z_cmd, 0.6))
    
    # ============================================================
    # STATE HANDLERS
    # ============================================================
    
    def handle_submerging(self, cmd: Twist) -> Twist:
        """Submerge to target depth"""
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        
        depth_error = abs(self.TARGET_DEPTH - self.current_depth)
        
        if depth_error < 0.15:
            if self.submerge_stable_time == 0.0:
                self.submerge_stable_time = time.time()
            
            stable_duration = time.time() - self.submerge_stable_time
            if stable_duration >= self.DEPTH_STABLE_TIME:
                self.get_logger().info('='*70)
                self.get_logger().info(f'✅ Depth stabilized at {self.current_depth:.2f}m')
                self.get_logger().info('🔍 Starting gate search...')
                self.get_logger().info('='*70)
                self.transition_to(self.SEARCHING)
        else:
            self.submerge_stable_time = 0.0
            
            elapsed = time.time() - self.state_start_time
            if int(elapsed) % 2 == 0:
                self.get_logger().info(
                    f'⬇️  Submerging: {self.current_depth:.2f}m → {self.TARGET_DEPTH:.2f}m',
                    throttle_duration_sec=1.9
                )
        
        return cmd
    
    def handle_searching(self, cmd: Twist) -> Twist:
        """Search pattern"""
        if self.gate_detected and self.distance < self.APPROACH_START_DISTANCE:
            self.get_logger().info('='*70)
            self.get_logger().info(f'🎯 Gate Found at {self.distance:.2f}m')
            self.get_logger().info('='*70)
            self.transition_to(self.APPROACHING)
            return cmd
        
        cmd.linear.x = self.SEARCH_FORWARD_SPEED
        
        elapsed = time.time() - self.state_start_time
        sweep_phase = (elapsed % 10.0) / 10.0
        
        if sweep_phase < 0.5:
            cmd.angular.z = self.SEARCH_ROTATION_SPEED
        else:
            cmd.angular.z = -self.SEARCH_ROTATION_SPEED
        
        if int(elapsed) % 3 == 0:
            direction = "LEFT" if sweep_phase < 0.5 else "RIGHT"
            self.get_logger().info(
                f'🔍 Searching ({direction})... {elapsed:.0f}s',
                throttle_duration_sec=2.9
            )
        
        return cmd
    
    def handle_approaching(self, cmd: Twist) -> Twist:
        """Approach gate"""
        if not self.gate_detected:
            self.get_logger().warn('⚠️  Gate lost - returning to search')
            self.transition_to(self.SEARCHING)
            return cmd
        
        if self.distance <= self.ALIGNMENT_DISTANCE:
            self.get_logger().info('='*70)
            self.get_logger().info(f'📍 Reached alignment point ({self.distance:.2f}m)')
            self.get_logger().info('='*70)
            self.transition_to(self.ALIGNING)
            return cmd
        
        if self.distance > 3.5:
            cmd.linear.x = self.APPROACH_SPEED
        else:
            cmd.linear.x = self.APPROACH_SPEED * 0.8
        
        cmd.angular.z = -self.alignment_error * self.YAW_GAIN_COARSE
        
        if int((time.time() - self.state_start_time) * 2) % 2 == 0:
            self.get_logger().info(
                f'➡️  Approaching: {self.distance:.2f}m',
                throttle_duration_sec=0.9
            )
        
        return cmd
    
    def handle_aligning(self, cmd: Twist) -> Twist:
        """Precise alignment"""
        if not self.gate_detected:
            self.get_logger().warn('⚠️  Gate lost during alignment')
            self.transition_to(self.SEARCHING)
            return cmd
        
        if self.alignment_start_time == 0.0:
            self.alignment_start_time = time.time()
            self.get_logger().info('🎯 Starting precision alignment...')
        
        alignment_elapsed = time.time() - self.alignment_start_time
        
        if alignment_elapsed > self.ALIGNMENT_MAX_TIME:
            self.get_logger().warn('⏰ Alignment timeout - proceeding')
            self.alignment_start_time = 0.0
            self.transition_to(self.FINAL_APPROACH)
            return cmd
        
        is_aligned = abs(self.alignment_error) < self.COARSE_ALIGNMENT_THRESHOLD
        
        if is_aligned:
            self.get_logger().info('='*70)
            self.get_logger().info(f'✅ ALIGNED! (time: {alignment_elapsed:.1f}s)')
            self.get_logger().info('='*70)
            self.alignment_start_time = 0.0
            self.transition_to(self.FINAL_APPROACH)
            return cmd
        
        alignment_quality = abs(self.alignment_error)
        
        if alignment_quality > 0.15:
            cmd.linear.x = 0.1
            cmd.angular.z = -self.alignment_error * self.YAW_GAIN_FINE
            status = "COARSE"
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = -self.alignment_error * (self.YAW_GAIN_FINE * 0.8)
            status = "FINE"
        
        if int(alignment_elapsed * 3) % 2 == 0:
            self.get_logger().info(
                f'🔄 Aligning ({status}): error={self.alignment_error:+.3f}',
                throttle_duration_sec=0.6
            )
        
        return cmd
    
    def handle_final_approach(self, cmd: Twist) -> Twist:
        """
        Final approach to commit point
        NEW: If gate lost here, don't return to search - lock and commit!
        """
        
        # Check if reached passing distance
        if self.distance <= self.PASSING_DISTANCE or not self.gate_detected:
            # CRITICAL: Lock current heading and center position
            if self.last_known_gate_center_x is not None:
                self.get_logger().info('='*70)
                self.get_logger().info('🔒 CENTER LOCKED - COMMITTING TO PASSAGE!')
                self.get_logger().info(f'   Locked Center: X={self.last_known_gate_center_x}')
                self.get_logger().info(f'   Will drive forward for {self.PASSING_DURATION}s')
                self.get_logger().info('='*70)
                
                # Lock current yaw
                self.locked_yaw = self.current_yaw
                self.passing_start_time = time.time()
                self.transition_to(self.PASSING)
                return cmd
            else:
                # Fallback: continue forward if no center known
                self.get_logger().warn('⚠️  No center lock available - continuing forward')
                cmd.linear.x = self.FINAL_APPROACH_SPEED
                return cmd
        
        # Normal final approach with gate visible
        cmd.linear.x = self.FINAL_APPROACH_SPEED
        
        if abs(self.alignment_error) > 0.08:
            cmd.linear.x *= 0.7
            cmd.angular.z = -self.alignment_error * self.YAW_GAIN_FINE
        else:
            cmd.angular.z = -self.alignment_error * (self.YAW_GAIN_FINE * 0.5)
        
        if int((time.time() - self.state_start_time) * 2) % 2 == 0:
            self.get_logger().info(
                f'🎯 Final Approach: {self.distance:.2f}m',
                throttle_duration_sec=0.9
            )
        
        return cmd
    
    def handle_passing(self, cmd: Twist) -> Twist:
        """
        PASSING MODE - NEW IMPLEMENTATION
        Locks onto last known center and drives forward for 10 seconds
        NO DETECTION REQUIRED - guaranteed passage
        """
        
        # Initialize timer if first entry
        if self.passing_start_time == 0.0:
            self.passing_start_time = time.time()
            self.locked_yaw = self.current_yaw
            self.get_logger().info('🚀 PASSING MODE: 10 second locked drive initiated')
        
        # Calculate elapsed time
        elapsed = time.time() - self.passing_start_time
        remaining = self.PASSING_DURATION - elapsed
        
        # Check if 10 seconds completed
        if elapsed >= self.PASSING_DURATION:
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 10 SECOND PASSAGE COMPLETE!')
            self.get_logger().info(f'   Total passage time: {elapsed:.2f}s')
            self.get_logger().info('   Gate successfully cleared!')
            self.get_logger().info('='*70)
            self.transition_to(self.COMPLETED)
            return cmd
        
        # LOCKED FORWARD MOVEMENT
        cmd.linear.x = self.PASSING_SPEED
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0  # Maintain current depth
        
        # Maintain locked heading (gentle correction to prevent drift)
        if self.locked_yaw is not None:
            yaw_error = self.normalize_angle(self.locked_yaw - self.current_yaw)
            # Very gentle yaw correction to maintain straight line
            cmd.angular.z = yaw_error * 0.5
        else:
            cmd.angular.z = 0.0
        
        # Progress logging
        if int(elapsed * 2) % 2 == 0:
            progress = (elapsed / self.PASSING_DURATION) * 100
            self.get_logger().info(
                f'🚀 PASSING: {elapsed:.1f}s / {self.PASSING_DURATION}s '
                f'({progress:.0f}%) - {remaining:.1f}s remaining',
                throttle_duration_sec=0.9
            )
        
        return cmd
    
    def handle_completed(self, cmd: Twist) -> Twist:
        """Mission complete"""
        if not hasattr(self, '_completion_logged'):
            total_time = time.time() - self.mission_start_time
            
            self.get_logger().info('='*70)
            self.get_logger().info('🏆 MISSION COMPLETE!')
            self.get_logger().info(f'   Total mission time: {total_time:.2f}s')
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
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = ImprovedWhiteGateNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()