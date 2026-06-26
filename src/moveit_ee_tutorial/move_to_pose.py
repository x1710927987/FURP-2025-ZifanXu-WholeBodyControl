#!/usr/bin/env python3
"""
Hello MoveIt — end-effector pose planning with pymoveit2 (Panda proxy).

Prerequisites (WSL):
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/local_setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Terminal 1:
  ros2 launch panda_moveit_config ex_fake_control.launch.py

Terminal 2:
  python3 move_to_pose.py --x 0.25 --y 0.0 --z 1.0
"""

from __future__ import annotations

import argparse
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda as robot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plan and execute a Panda EE pose.")
    parser.add_argument("--x", type=float, default=0.25)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=1.0)
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, default=0.0)
    parser.add_argument("--qw", type=float, default=1.0)
    parser.add_argument("--cartesian", action="store_true")
    parser.add_argument("--velocity-scale", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()

    node = Node("move_to_pose")
    callback_group = ReentrantCallbackGroup()

    moveit2 = MoveIt2(
        node=node,
        joint_names=robot.joint_names(),
        base_link_name=robot.base_link_name(),
        end_effector_name=robot.end_effector_name(),
        group_name=robot.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )
    moveit2.planner_id = "RRTConnectkConfigDefault"
    moveit2.max_velocity = args.velocity_scale
    moveit2.max_acceleration = args.velocity_scale

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    node.create_rate(1.0).sleep()

    position = [args.x, args.y, args.z]
    quat_xyzw = [args.qx, args.qy, args.qz, args.qw]

    node.get_logger().info(
        f"Planning to position={position}, quat_xyzw={quat_xyzw}, cartesian={args.cartesian}"
    )

    moveit2.move_to_pose(
        position=position,
        quat_xyzw=quat_xyzw,
        cartesian=args.cartesian,
        cartesian_max_step=0.0025,
        cartesian_fraction_threshold=0.0,
        cartesian_jump_threshold=0.0,
        cartesian_avoid_collisions=False,
    )
    moveit2.wait_until_executed()

    node.get_logger().info("Motion finished.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
