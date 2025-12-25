#!/usr/bin/env python3
"""
Enhanced Flare Order Prompt - SAUVC Mission
User enters flare bumping order after bot stabilizes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


def print_banner():
    """Display colorful banner"""
    banner = """
╔══════════════════════════════════════════════════════════════════╗
║                                                                  ║
║        🎯  SAUVC FLARE BUMPING TASK - ORDER INPUT  🎯           ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝

📍 CURRENT STATUS:
   → AUV is stabilizing at safe depth (-0.8m)
   → Gate and Flare tasks are DISABLED
   → Waiting for YOUR input to begin mission

═══════════════════════════════════════════════════════════════════

🎨 FLARE COLORS & POSITIONS (from pool map):
   🔴 RED Flare    - Position: (1m, -2m)  - Left side
   🟡 YELLOW Flare - Position: (3m, 0m)   - Center
   🔵 BLUE Flare   - Position: (5m, 2m)   - Right side

═══════════════════════════════════════════════════════════════════

📝 VALID FLARE ORDERS (SHORT FORM):

   ┌─────────┬────────────────────────────────┐
   │ r-y-b   │ Red → Yellow → Blue            │
   │ r-b-y   │ Red → Blue → Yellow            │
   │ y-r-b   │ Yellow → Red → Blue            │
   │ y-b-r   │ Yellow → Blue → Red            │
   │ b-r-y   │ Blue → Red → Yellow            │
   │ b-y-r   │ Blue → Yellow → Red            │
   └─────────┴────────────────────────────────┘

═══════════════════════════════════════════════════════════════════
"""
    print(banner)


def main(args=None):
    print_banner()
    
    valid_orders = ['r-y-b', 'r-b-y', 'y-r-b', 'y-b-r', 'b-r-y', 'b-y-r']
    
    order = None
    while order not in valid_orders:
        try:
            print("\n📝 Enter flare order (e.g., r-y-b): ", end='', flush=True)
            order = input().lower().strip()
            
            if order not in valid_orders:
                print(f"\n❌ Invalid order '{order}'!")
                print("   Please use one of: r-y-b, r-b-y, y-r-b, y-b-r, b-r-y, b-y-r")
                print("   Try again...")
            else:
                break
        except (KeyboardInterrupt, EOFError):
            print("\n\n❌ Order input cancelled. Mission aborted.")
            sys.exit(1)
    
    # Display confirmation
    order_map = {'r': 'RED', 'y': 'YELLOW', 'b': 'BLUE'}
    colors = order.split('-')
    order_description = ' → '.join([order_map[c] for c in colors])
    
    print("\n" + "="*70)
    print("✅ ORDER CONFIRMED!")
    print("="*70)
    print(f"   Sequence: {order_description}")
    print(f"   Notation: {order.upper()}")
    print("="*70)
    print("\n📡 Sending order to AUV...", flush=True)
    
    # Initialize ROS and send order
    rclpy.init(args=args)
    node = Node('flare_order_prompt')
    publisher = node.create_publisher(String, '/flare/mission_order', 10)
    
    # Wait for publisher to be ready
    time.sleep(0.5)
    
    msg = String()
    msg.data = order
    
    # Send order multiple times to ensure delivery
    print("   Transmitting", end='', flush=True)
    for i in range(20):
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        if i % 4 == 0:
            print(".", end='', flush=True)
        time.sleep(0.1)
    
    print(" Done!")
    print("\n✅ Order transmitted successfully!")
    print("\n" + "="*70)
    print("🚀 MISSION STARTING SEQUENCE:")
    print("   1. Gate Task will activate first")
    print("   2. After gate completion, Flare Task will activate")
    print("   3. AUV will hit flares in order:", order_description)
    print("   4. Mission complete & surface")
    print("="*70)
    print("\n" + "="*70)
    print("📺 Monitor progress in the MAIN terminal window")
    print("🎮 This window will stay open for reference")
    print("="*70 + "\n")
    
    # Keep window open
    try:
        print("💡 TIP: Keep this window open to see the order")
        print("    Press Ctrl+C to close this window...\n")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        print("\n👋 Closing order prompt window...")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()