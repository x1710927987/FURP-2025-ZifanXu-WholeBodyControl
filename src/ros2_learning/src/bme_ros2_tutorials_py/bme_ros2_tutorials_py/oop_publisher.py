#!/usr/bin/env python3
"""ROS2 OOP publisher practicing node."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisherNode(Node):
    """A ROS2 node that publishes String messages periodically."""
    def __init__(self):
        super().__init__("oop_python_publisher")
        self.i = 0
        self.msg = String()
        self.publisher_ = self.create_publisher(String, "oop_topic", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Publisher OOP has been started.")

    def timer_callback(self):
        """Publish a message and increment the counter."""
        self.msg.data = f"Hello World: {self.i}"
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Publishing: \"{self.msg.data}\"")
        self.i += 1

def main(args = None):
    """Initialize and spin the ROS2 publisher node."""
    rclpy.init(args = args)
    node = MyPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt! Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
