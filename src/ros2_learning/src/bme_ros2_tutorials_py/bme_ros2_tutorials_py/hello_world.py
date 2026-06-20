#!/usr/bin/env python3

import rclpy

def main(args = None):
    rclpy.init(args = args)
    node = rclpy.create_node("python_hello_world")
    node.get_logger().info("Hello, ROS2!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()