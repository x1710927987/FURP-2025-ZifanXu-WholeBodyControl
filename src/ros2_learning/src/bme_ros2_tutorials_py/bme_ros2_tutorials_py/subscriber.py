#!/usr/bin/env python3
"""ROS2 POP subscriber practicing node."""

import rclpy
from std_msgs.msg import String

def main(args = None):
    """Initialize and spin the ROS2 subscriber node."""
    rclpy.init(args = args)
    node = rclpy.create_node("python_subscriber")
    
    def subscriber_callback(msg):
        node.get_logger().info(f"I heard{msg.data}")
    
    node.create_subscription(String, "topic", subscriber_callback, 10)
    node.get_logger().info("Subscriber started.")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt! shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
