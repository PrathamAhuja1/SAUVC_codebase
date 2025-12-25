#!/usr/bin/env python3
"""
Flare Navigator - Sequential Flare Bumping Task
FIXED: Starts DISABLED and stabilizes until order is received
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


class FlareNavigator(Node):
    def __init__(self):
        super().__init__('flare_navigator')
        
        # State machine
        self.WAITING_FOR_ORDER = 0
        self.SUBMERGING = 1
        self.SEARCHING = 2
        self.APPROACHING = 3
        self.ALIGNING = 4
        self.BUMPING = 5
        self.VERIFYING_BUMP = 6
        self.NEXT_TARGET = 7
        self.COMPLETED = 8
        
        self.state = self.WAITING_FOR_ORDER
        
        # Parameters
        self.declare_parameter('target_depth', -0.8)  # Safe shallow depth for waiting
        self.declare_parameter('mission_depth', -2.3)  # Deeper mission depth
        self.declare_parameter('search_speed', 0.4)
        self.declare_parameter('approach_speed', 0.5)
        self.declare_parameter('bump_speed', 0.6)
        self.declare_parameter('bump_distance_threshold', 0.8)
        self.declare_parameter('alignment_threshold', 0.15)
        self.declare_parameter('bump_duration', 4.0)
        self.declare_parameter('verification_time', 2.0)
        
        self.target_depth = self.get_parameter('target_depth').value
        self.mission_depth = self.get_parameter('mission_depth').value
        self.search_speed = self.get_parameter('search_speed').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.bump_speed = self.get_parameter('bump_speed').value
        self.bump_distance = self.get_parameter('bump_distance_threshold').value
        self.alignment_threshold = self.get_parameter('alignment_threshold').value
        self.bump_duration = self.get_parameter('bump_duration').value
        self.verification_time = self.get_parameter('verification_time').value
        
        # Mission state
        self.flare_order = []
        self.current_target_index = 0
        self.current_target_color = None
        self.bumped_flares = []
        self.order_received = False
        
        # Detection state
        self.flare_detected = {
            'red': False,
            'yellow': False,
            'blue': False
        }
        self.flare_distance = {
            'red': 999.0,
            'yellow': 999.0,
            'blue': 999.0
        }
        self.flare_alignment = {
            'red': 0.0,
            'yellow': 0.0,
            'blue': 0.0
        }
        
        # Position tracking
        self.current_depth = 0.0
        self.current_position = None
        self.bump_start_position = None
        self.bump_start_time = 0.0
        self.verification_start_time = 0.0
        self.state_start_time = time.time()
        self.mission_start_time = time.time()
        
        # Subscriptions - Red flare
        self.create_subscription(Bool, '/flare/red/detected', 
                                lambda msg: self.flare_detection_callback(msg, 'red'), 10)
        self.create_subscription(Float32, '/flare/red/distance',
                                lambda msg: self.flare_distance_callback(msg, 'red'), 10)
        self.create_subscription(Float32, '/flare/red/alignment_error',
                                lambda msg: self.flare_alignment_callback(msg, 'red'), 10)
        
        # Subscriptions - Yellow flare
        self.create_subscription(Bool, '/flare/yellow/detected',
                                lambda msg: self.flare_detection_callback(msg, 'yellow'), 10)
        self.create_subscription(Float32, '/flare/yellow/distance',
                                lambda msg: self.flare_distance_callback(msg, 'yellow'), 10)
        self.create_subscription(Float32, '/flare/yellow/alignment_error',
                                lambda msg: self.flare_alignment_callback(msg, 'yellow'), 10)
        
        # Subscriptions - Blue flare
        self.create_subscription(Bool, '/flare/blue/detected',
                                lambda msg: self.flare_detection_callback(msg, 'blue'), 10)
        self.create_subscription(Float32, '/flare/blue/distance',
                                lambda msg: self.flare_distance_callback(msg, 'blue'), 10)
        self.create_subscription(Float32, '/flare/blue/alignment_error',
                                lambda msg: self.flare_alignment_callback(msg, 'blue'), 10)
        
        # Subscriptions - Odometry
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_callback, 10)
        
        # Subscriptions - Order input
        self.create_subscription(String, '/flare/mission_order', self.order_callback, 10)
        
        # CRITICAL: Start DISABLED until order is received
        self.task_enabled = False
        self.create_subscription(Bool, '/flare/task_enable', self.task_enable_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/flare/navigation_state', 10)
        self.mission_complete_pub = self.create_publisher(Bool, '/flare/mission_complete', 10)
        
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('🟡 Flare Navigator WAITING FOR ORDER')
        self.get_logger().info('   Status: DISABLED - Stabilizing at -0.8m depth')
        self.get_logger().info('   Enter flare order in the prompt terminal...')
        self.get_logger().info('='*70)
    
    def flare_detection_callback(self, msg: Bool, color: str):
        self.flare_detected[color] = msg.data
    
    def flare_distance_callback(self, msg: Float32, color: str):
        self.flare_distance[color] = msg.data
    
    def flare_alignment_callback(self, msg: Float32, color: str):
        self.flare_alignment[color] = msg.data
    
    def odom_callback(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
    
    def order_callback(self, msg: String):
        """Receive flare bumping order"""
        if self.order_received:
            self.get_logger().warn('Order already received! Ignoring new order.')
            return
        
        order_str = msg.data.strip().lower()
        flares_raw = order_str.split('-')
        
        # Convert short names to full names
        color_map = {
            'r': 'red', 'red': 'red',
            'y': 'yellow', 'yellow': 'yellow',
            'b': 'blue', 'blue': 'blue'
        }
        
        # Validate and convert
        if len(flares_raw) != 3:
            self.get_logger().error(f'Invalid order: must have 3 flares, got {len(flares_raw)}')
            return
        
        flares = []
        for flare in flares_raw:
            if flare not in color_map:
                self.get_logger().error(f'Invalid flare color: {flare} (use r/red, y/yellow, b/blue)')
                return
            flares.append(color_map[flare])
        
        if len(set(flares)) != 3:
            self.get_logger().error('Order must contain each flare exactly once')
            return
        
        # Set order
        self.flare_order = flares
        self.current_target_index = 0
        self.current_target_color = self.flare_order[0]
        self.order_received = True
        
        self.get_logger().info('='*70)
        self.get_logger().info('📋 FLARE ORDER RECEIVED!')
        self.get_logger().info(f'   Order: {" → ".join([f.upper() for f in self.flare_order])}')
        self.get_logger().info('   🚀 Starting mission in 3 seconds...')
        self.get_logger().info('='*70)
        
        # Enable task after short delay
        def enable_task():
            self.task_enabled = True
            self.target_depth = self.mission_depth  # Switch to mission depth
            self.transition_to(self.SUBMERGING)
            self.get_logger().info('✅ Flare task ENABLED - Mission starting!')
        
        self.create_timer(3.0, enable_task, oneshot=True)
    
    def task_enable_callback(self, msg: Bool):
        """Enable/disable this task"""
        if msg.data and not self.order_received:
            self.get_logger().warn('Cannot enable task - no order received yet!')
            return
        self.task_enabled = msg.data
    
    def control_loop(self):
        """Main control loop"""
        cmd = Twist()
        
        if not self.order_received:
            # STABILIZE: Maintain shallow depth and zero velocity
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = self.depth_control(self.target_depth)  # -0.8m
            cmd.angular.z = 0.0
            
            # Periodic reminder
            elapsed = time.time() - self.state_start_time
            if int(elapsed) % 10 == 0:
                self.get_logger().info(
                    f'⏳ Waiting for order... (depth: {self.current_depth:.2f}m, '
                    f'target: {self.target_depth:.2f}m)',
                    throttle_duration_sec=9.5
                )
        
        elif not self.task_enabled:
            # Order received but task not enabled yet (during 3s delay)
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = self.depth_control(self.target_depth)
            cmd.angular.z = 0.0
        
        else:
            # Task enabled - normal operation
            # Depth control (except during bumping)
            if self.state != self.BUMPING and self.state != self.VERIFYING_BUMP:
                cmd.linear.z = self.depth_control(self.target_depth)
            
            # State machine
            if self.state == self.SUBMERGING:
                cmd = self.submerging(cmd)
            elif self.state == self.SEARCHING:
                cmd = self.searching(cmd)
            elif self.state == self.APPROACHING:
                cmd = self.approaching(cmd)
            elif self.state == self.ALIGNING:
                cmd = self.aligning(cmd)
            elif self.state == self.BUMPING:
                cmd = self.bumping(cmd)
            elif self.state == self.VERIFYING_BUMP:
                cmd = self.verifying_bump(cmd)
            elif self.state == self.NEXT_TARGET:
                cmd = self.next_target(cmd)
            elif self.state == self.COMPLETED:
                cmd = self.completed(cmd)
        
        self.cmd_vel_pub.publish(cmd)
        self.state_pub.publish(String(data=self.get_state_name()))
    
    def depth_control(self, target_depth: float) -> float:
        """Depth control"""
        depth_error = target_depth - self.current_depth
        deadband = 0.2
        
        if abs(depth_error) < deadband:
            return 0.0
        
        z_cmd = depth_error * 1.0
        return max(-0.8, min(z_cmd, 0.8))
    
    def submerging(self, cmd: Twist) -> Twist:
        """Submerge to mission depth"""
        if abs(self.target_depth - self.current_depth) < 0.3:
            elapsed = time.time() - self.state_start_time
            if elapsed > 3.0:
                self.get_logger().info(f'✅ Submerged - searching for {self.current_target_color.upper()} flare')
                self.transition_to(self.SEARCHING)
        return cmd
    
    def searching(self, cmd: Twist) -> Twist:
        """Search for current target flare"""
        target_color = self.current_target_color
        
        if self.flare_detected[target_color]:
            distance = self.flare_distance[target_color]
            if distance < 999:
                self.get_logger().info(
                    f'🎯 {target_color.upper()} flare found at {distance:.2f}m - approaching'
                )
                self.transition_to(self.APPROACHING)
                return cmd
        
        # Search pattern
        elapsed = time.time() - self.state_start_time
        cmd.linear.x = self.search_speed
        cmd.angular.z = 0.3
        
        if int(elapsed) % 5 == 0:
            self.get_logger().info(
                f'🔍 Searching for {target_color.upper()} flare... ({elapsed:.0f}s)',
                throttle_duration_sec=4.9
            )
        
        return cmd
    
    def approaching(self, cmd: Twist) -> Twist:
        """Approach target flare"""
        target_color = self.current_target_color
        
        if not self.flare_detected[target_color]:
            self.get_logger().warn(f'Lost {target_color.upper()} flare - returning to search')
            self.transition_to(self.SEARCHING)
            return cmd
        
        distance = self.flare_distance[target_color]
        
        if distance < 2.0:
            self.get_logger().info(f'📍 Close to {target_color.upper()} flare - aligning')
            self.transition_to(self.ALIGNING)
            return cmd
        
        cmd.linear.x = self.approach_speed
        cmd.angular.z = -self.flare_alignment[target_color] * 1.5
        
        self.get_logger().info(
            f'➡️ Approaching {target_color.upper()}: {distance:.2f}m',
            throttle_duration_sec=0.5
        )
        
        return cmd
    
    def aligning(self, cmd: Twist) -> Twist:
        """Align with target flare before bumping"""
        target_color = self.current_target_color
        
        if not self.flare_detected[target_color]:
            self.get_logger().warn(f'Lost {target_color.upper()} flare - returning to search')
            self.transition_to(self.SEARCHING)
            return cmd
        
        distance = self.flare_distance[target_color]
        alignment = self.flare_alignment[target_color]
        
        if distance <= self.bump_distance and abs(alignment) < self.alignment_threshold:
            self.get_logger().info(
                f'🎯 ALIGNED with {target_color.upper()} flare - BUMPING!'
            )
            self.bump_start_position = self.current_position
            self.bump_start_time = time.time()
            self.transition_to(self.BUMPING)
            return cmd
        
        if abs(alignment) > 0.2:
            cmd.linear.x = 0.1
            cmd.angular.z = -alignment * 3.0
        elif abs(alignment) > 0.1:
            cmd.linear.x = 0.2
            cmd.angular.z = -alignment * 2.0
        else:
            cmd.linear.x = 0.3
            cmd.angular.z = -alignment * 1.5
        
        self.get_logger().info(
            f'🔄 Aligning {target_color.upper()}: dist={distance:.2f}m, align={alignment:+.3f}',
            throttle_duration_sec=0.3
        )
        
        return cmd
    
    def bumping(self, cmd: Twist) -> Twist:
        """Bump the flare"""
        elapsed = time.time() - self.bump_start_time
        
        if elapsed >= self.bump_duration:
            self.get_logger().info(
                f'✅ Bump complete for {self.current_target_color.upper()} - verifying'
            )
            self.verification_start_time = time.time()
            self.transition_to(self.VERIFYING_BUMP)
            return cmd
        
        cmd.linear.x = self.bump_speed
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.z = 0.0
        
        self.get_logger().info(
            f'💥 BUMPING {self.current_target_color.upper()}... ({elapsed:.1f}/{self.bump_duration:.1f}s)',
            throttle_duration_sec=0.5
        )
        
        return cmd
    
    def verifying_bump(self, cmd: Twist) -> Twist:
        """Stop and verify bump success"""
        elapsed = time.time() - self.verification_start_time
        
        if elapsed >= self.verification_time:
            self.bumped_flares.append(self.current_target_color)
            self.get_logger().info(
                f'✅ {self.current_target_color.upper()} flare BUMPED successfully!'
            )
            self.transition_to(self.NEXT_TARGET)
            return cmd
        
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        return cmd
    
    def next_target(self, cmd: Twist) -> Twist:
        """Move to next target"""
        if len(self.bumped_flares) >= len(self.flare_order):
            self.get_logger().info('🎉 ALL FLARES BUMPED - MISSION COMPLETE!')
            self.transition_to(self.COMPLETED)
            return cmd
        
        self.current_target_index += 1
        self.current_target_color = self.flare_order[self.current_target_index]
        
        remaining = len(self.flare_order) - len(self.bumped_flares)
        self.get_logger().info('='*70)
        self.get_logger().info(f'📋 Next target: {self.current_target_color.upper()} flare')
        self.get_logger().info(f'   Bumped: {" ✓ ".join([f.upper() for f in self.bumped_flares])}')
        self.get_logger().info(f'   Remaining: {remaining}')
        self.get_logger().info('='*70)
        
        self.transition_to(self.SEARCHING)
        return cmd
    
    def completed(self, cmd: Twist) -> Twist:
        """Mission complete"""
        if not hasattr(self, '_completion_reported'):
            total_time = time.time() - self.mission_start_time
            
            self.get_logger().info('='*70)
            self.get_logger().info('🏆 FLARE MISSION COMPLETE!')
            self.get_logger().info(f'   Order: {" → ".join([f.upper() for f in self.flare_order])}')
            self.get_logger().info(f'   Total time: {total_time:.1f}s')
            self.get_logger().info('='*70)
            
            self.mission_complete_pub.publish(Bool(data=True))
            self._completion_reported = True
        
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        return cmd
    
    def transition_to(self, new_state: int):
        """Transition to new state"""
        old_name = self.get_state_name()
        self.state = new_state
        self.state_start_time = time.time()
        new_name = self.get_state_name()
        
        if old_name != new_name:
            self.get_logger().info(f'🔄 STATE: {old_name} → {new_name}')
    
    def get_state_name(self) -> str:
        """Get state name"""
        names = {
            self.WAITING_FOR_ORDER: 'WAITING_FOR_ORDER',
            self.SUBMERGING: 'SUBMERGING',
            self.SEARCHING: 'SEARCHING',
            self.APPROACHING: 'APPROACHING',
            self.ALIGNING: 'ALIGNING',
            self.BUMPING: 'BUMPING',
            self.VERIFYING_BUMP: 'VERIFYING_BUMP',
            self.NEXT_TARGET: 'NEXT_TARGET',
            self.COMPLETED: 'COMPLETED'
        }
        return names.get(self.state, 'UNKNOWN')


def main(args=None):
    rclpy.init(args=args)
    node = FlareNavigator()
    
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