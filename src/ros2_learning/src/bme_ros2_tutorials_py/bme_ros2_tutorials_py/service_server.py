#!/usr/bin/env python3

from bme_ros2_tutorials_interfaces.srv import CustomCalc

import rclpy
from rclpy.node import Node

class MyService(Node):
    def __init__(self):
        super().__init__('my_service')
        self.create_service(CustomCalc, 'custom_calc', self.custom_calc_callback)
        
    def custom_calc_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: \na: {request.a}, b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    my_service_server = MyService()
    try:
        rclpy.spin(my_service_server)
    except KeyboardInterrupt:
        my_service_server.get_logger().info('KeyboardInterrupt! Shutting down.')
    finally:
        my_service_server.destroy_node()
        rclpy.shutdown()
   
if __name__ == '__main__':
    main()
