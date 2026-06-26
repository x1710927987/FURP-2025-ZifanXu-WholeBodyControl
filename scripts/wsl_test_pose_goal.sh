#!/usr/bin/env bash
# Single end-effector pose goal (run inside WSL while fake control is up)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run pymoveit2 ex_pose_goal.py --ros-args \
  -p position:="[0.25, 0.0, 1.0]" \
  -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" \
  -p cartesian:=False
