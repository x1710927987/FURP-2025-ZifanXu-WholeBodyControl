#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyPublisherNode(Node):
    def __init__(self):
        super().__init__("oop_python_publisher")
        self.i = 0
        self.msg = String()
        self.publisher_ = self.create_publisher(String, "oop_topic", 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("Publisher OOP has been started.")

    def timer_callback(self):
        self.msg.data = f"Hello World: {self.i}"
        self.publisher_.publish(self.msg)
        self.get_logger().info(f"Publishing: \"{self.msg.data}\"")
        self.i += 1

def main(args = None):
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