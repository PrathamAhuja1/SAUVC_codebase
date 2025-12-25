#!/usr/bin/env python3
"""
Task Coordinator - PROPER SAUVC SEQUENCE
Gate → Flares → Surface

NO DRUMS (hardware not ready)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time


class SAUVCTaskCoordinator(Node):
    def __init__(self):
        super().__init__('sauvc_task_coordinator')
        
        # Task states - PROPER SEQUENCE
        self.WAITING_FOR_FLARE_ORDER = 0
        self.GATE_TASK = 1
        self.TRANSITION_TO_FLARES = 2
        self.FLARE_TASK = 3
        self.MISSION_COMPLETE = 4
        
        self.current_task = self.WAITING_FOR_FLARE_ORDER
        
        # Completion flags
        self.flare_order_received = False
        self.gate_mission_complete = False
        self.flare_mission_complete = False
        
        # Transition timing
        self.transition_start_time = 0.0
        self.transition_delay = 3.0  # 3 seconds between tasks
        
        # Subscriptions
        self.gate_complete_sub = self.create_subscription(
            Bool, '/gate/mission_complete', self.gate_complete_callback, 10)
        self.flare_complete_sub = self.create_subscription(
            Bool, '/flare/mission_complete', self.flare_complete_callback, 10)
        self.flare_order_sub = self.create_subscription(
            String, '/flare/mission_order', self.flare_order_callback, 10)
        
        # Publishers
        self.gate_enable_pub = self.create_publisher(Bool, '/gate/task_enable', 10)
        self.flare_enable_pub = self.create_publisher(Bool, '/flare/task_enable', 10)
        self.mission_status_pub = self.create_publisher(String, '/mission/status', 10)
        
        # Timer
        self.timer = self.create_timer(0.5, self.update_state_machine)
        
        # Initial state - EVERYTHING DISABLED
        self.disable_all_tasks()
        
        self.get_logger().info('='*70)
        self.get_logger().info('✅ SAUVC Task Coordinator Initialized')
        self.get_logger().info('='*70)
        self.get_logger().info('📋 MISSION SEQUENCE:')
        self.get_logger().info('   0. Wait for Flare Order (from prompt terminal)')
        self.get_logger().info('   1. Gate Navigation Task')
        self.get_logger().info('   2. Flare Bumping Task')
        self.get_logger().info('   3. Surface & Complete')
        self.get_logger().info('='*70)
        self.get_logger().info('⏳ WAITING FOR FLARE ORDER...')
        self.get_logger().info('   → Open the prompt terminal and enter order (e.g., r-y-b)')
        self.get_logger().info('='*70)
    
    def flare_order_callback(self, msg: String):
        """Flare order received from prompt"""
        if not self.flare_order_received:
            self.flare_order_received = True
            self.get_logger().info('='*70)
            self.get_logger().info(f'📋 FLARE ORDER RECEIVED: {msg.data.upper()}')
            self.get_logger().info('='*70)
    
    def gate_complete_callback(self, msg: Bool):
        """Gate task completion callback"""
        if msg.data and not self.gate_mission_complete:
            self.gate_mission_complete = True
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 GATE TASK COMPLETE!')
            self.get_logger().info('='*70)
    
    def flare_complete_callback(self, msg: Bool):
        """Flare task completion callback"""
        if msg.data and not self.flare_mission_complete:
            self.flare_mission_complete = True
            self.get_logger().info('='*70)
            self.get_logger().info('🎉 FLARE TASK COMPLETE!')
            self.get_logger().info('='*70)
    
    def update_state_machine(self):
        """Main state machine"""
        
        if self.current_task == self.WAITING_FOR_FLARE_ORDER:
            # Wait for user to input flare order
            if self.flare_order_received:
                self.get_logger().info('='*70)
                self.get_logger().info('🚀 STARTING GATE TASK')
                self.get_logger().info('='*70)
                self.enable_gate_task()
                self.current_task = self.GATE_TASK
        
        elif self.current_task == self.GATE_TASK:
            # Wait for gate task to complete
            if self.gate_mission_complete:
                self.get_logger().info('🔄 Transitioning from Gate to Flares...')
                self.disable_gate_task()
                self.transition_start_time = time.time()
                self.current_task = self.TRANSITION_TO_FLARES
        
        elif self.current_task == self.TRANSITION_TO_FLARES:
            # Wait for transition delay
            elapsed = time.time() - self.transition_start_time
            if elapsed >= self.transition_delay:
                self.get_logger().info('='*70)
                self.get_logger().info('🚀 STARTING FLARE TASK')
                self.get_logger().info('='*70)
                self.enable_flare_task()
                self.current_task = self.FLARE_TASK
        
        elif self.current_task == self.FLARE_TASK:
            # Wait for flare task to complete
            if self.flare_mission_complete:
                self.get_logger().info('='*70)
                self.get_logger().info('🏆 ALL MISSION TASKS COMPLETE!')
                self.get_logger().info('   Gate: ✅ | Flares: ✅')
                self.get_logger().info('   🎉 SURFACING NOW!')
                self.get_logger().info('='*70)
                self.disable_flare_task()
                self.current_task = self.MISSION_COMPLETE
        
        elif self.current_task == self.MISSION_COMPLETE:
            # Mission finished - do nothing
            pass
        
        # Publish mission status
        status = self.get_mission_status()
        self.mission_status_pub.publish(String(data=status))
    
    def enable_gate_task(self):
        """Enable gate navigation"""
        self.gate_enable_pub.publish(Bool(data=True))
        self.get_logger().info('✅ Gate Task ENABLED')
    
    def disable_gate_task(self):
        """Disable gate navigation"""
        self.gate_enable_pub.publish(Bool(data=False))
        self.get_logger().info('⏸️ Gate Task DISABLED')
    
    def enable_flare_task(self):
        """Enable flare detection and navigation"""
        self.flare_enable_pub.publish(Bool(data=True))
        self.get_logger().info('✅ Flare Task ENABLED')
    
    def disable_flare_task(self):
        """Disable flare detection and navigation"""
        self.flare_enable_pub.publish(Bool(data=False))
        self.get_logger().info('⏸️ Flare Task DISABLED')
    
    def disable_all_tasks(self):
        """Disable all tasks"""
        self.gate_enable_pub.publish(Bool(data=False))
        self.flare_enable_pub.publish(Bool(data=False))
        self.get_logger().info('🔒 All tasks DISABLED - Waiting for flare order')
    
    def get_mission_status(self) -> str:
        """Get current mission status string"""
        if self.current_task == self.WAITING_FOR_FLARE_ORDER:
            return "WAITING_FOR_FLARE_ORDER"
        elif self.current_task == self.GATE_TASK:
            return "GATE_TASK_ACTIVE"
        elif self.current_task == self.TRANSITION_TO_FLARES:
            return "TRANSITIONING_TO_FLARES"
        elif self.current_task == self.FLARE_TASK:
            return "FLARE_TASK_ACTIVE"
        elif self.current_task == self.MISSION_COMPLETE:
            return "MISSION_COMPLETE"
        else:
            return "UNKNOWN"


def main(args=None):
    rclpy.init(args=args)
    node = SAUVCTaskCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()