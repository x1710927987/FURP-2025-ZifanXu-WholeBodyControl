#!/usr/bin/env python3
"""ROS2 launch practicing."""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch oop_publisher_py and oop_subscriber_py"""
    ld = LaunchDescription()

    oop_publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="oop_publisher_py",
        name="oop_python_publisher",
        remappings=[("oop_topic", "remapped_oop_topic")],
        parameters=[{"published_text": "Parameter_from_launch"}]
    )

    oop_subscriber_node = Node(
        package="bme_ros2_tutorials_py",
        executable="oop_subscriber_py",
        name="oop_python_subscriber",
        remappings=[("oop_topic", "remapped_oop_topic")]
    )

    publisher_node = Node(
        package="bme_ros2_tutorials_py",
        executable="publisher_py",
        name="python_publisher",
        remappings=[("topic", "remapped_topic")]
    )

    subscriber_node = Node(
        package="bme_ros2_tutorials_py",
        executable="subscriber_py",
        name="python_subscriber",
        remappings=[("topic", "remapped_topic")]
    )

    ld.add_action(oop_publisher_node)
    ld.add_action(oop_subscriber_node)
    ld.add_action(publisher_node)
    ld.add_action(subscriber_node)
    return ld
