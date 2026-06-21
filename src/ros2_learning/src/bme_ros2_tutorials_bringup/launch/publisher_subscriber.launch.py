#!/usr/bin/env python3
"""ROS2 launch practicing"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch oop_publisher_py and oop_subscriber_py"""
    ld = LaunchDescription()

    publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="oop_publisher_py",
        name="oop_publisher_py"
    )

    subscriber_node = Node(
        package="bme_ros2_tutorials_py",
        executable="oop_subscriber_py",
        name="oop_subscriber_py",
    )

    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    return ld
