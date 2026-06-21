#!/usr/bin/env python3
"""ROS2 OOP subscriber practicing node."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MySubscriberNode(Node):
    """A ROS2 node that subscribe String messages periodically."""
    def __init__(self):
        super().__init__("python_subscriber_oop")
        self.create_subscription(String, "oop_topic", self.subscriber_callback, 10)
        self.get_logger().info("OOP subscriber started.")
        
    def subscriber_callback(self, msg: String):
        """Callback function for OOP subscriber"""
        self.get_logger().info(f"I heard: {msg.data}")
        
def main(args = None):
    """Initialize and spin the ROS2 subscriber node."""
    rclpy.init(args = args)
    node = MySubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt! Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()
