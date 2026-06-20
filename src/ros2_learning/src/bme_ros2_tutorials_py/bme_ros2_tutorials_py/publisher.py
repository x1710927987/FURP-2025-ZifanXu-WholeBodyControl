#!/usr/bin/env python3

import time
import rclpy
from std_msgs.msg import String

def main(args = None):
    rclpy.init(args = args)
    node = rclpy.create_node("python_publisher")
    publisher = node.create_publisher(String, "topic", 10)
    msg = String()
    i = 0

    try:
        while rclpy.ok():
            msg.data = f"Hello World: {i}"
            publisher.publish(msg)
            node.get_logger().info(f"Publishing: \"{msg.data}\"")
            i += 1
            time.sleep(0.5)
    except KeyboardInterrupt:
        node.get_logger.info("KeyboardInterrupt! Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()