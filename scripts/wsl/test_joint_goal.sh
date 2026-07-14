#!/usr/bin/env bash
# Run pymoveit2 joint goal (use inside WSL, not from PowerShell directly)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run pymoveit2 ex_joint_goal.py --ros-args \
  -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"
