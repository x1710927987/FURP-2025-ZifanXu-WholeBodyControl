#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from bme_ros2_tutorials_interfaces.srv import CustomCalc

class MyServiceClientAsync(Node):
    def __init__(self):
        super().__init__('my_service_client_async')
        self.cli = self.create_client(CustomCalc, 'custom_calc')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = CustomCalc.Request()
        
    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)
    
def main(args=None):
    rclpy.init(args=args)
    my_service_client = MyServiceClientAsync()
    try:
        if len(sys.argv) != 3:
            my_service_client.get_logger().error('Usage: py_service_client a b')
            return
        future = my_service_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
        rclpy.spin_until_future_complete(my_service_client, future)
        response = future.result()
        if response is not None:
            my_service_client.get_logger().info(f'Result of custom_calc: {int(sys.argv[1])} + {int(sys.argv[2])} = {response.sum}')
        else:
            my_service_client.get_logger().error('Service call failed.')
    except KeyboardInterrupt:
        my_service_client.get_logger().info('KeyboardInterrupt detected! Stop spinning.')
    finally:
        my_service_client.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
