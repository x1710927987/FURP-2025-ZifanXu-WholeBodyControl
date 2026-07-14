#!/usr/bin/env bash
# End-effector planning tests for pymoveit2 (run inside WSL while fake control is up)
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "==> Test 1: pose goal (joint-space planning)"
ros2 run pymoveit2 ex_pose_goal.py --ros-args \
  -p position:="[0.25, 0.0, 1.0]" \
  -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" \
  -p cartesian:=False

sleep 3

echo "==> Test 2: pose goal (alternate target)"
ros2 run pymoveit2 ex_pose_goal.py --ros-args \
  -p position:="[0.35, 0.15, 0.9]" \
  -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" \
  -p cartesian:=False

sleep 3

echo "==> Test 3: gripper toggle"
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"

sleep 2

echo "==> Test 4: add collision sphere"
ros2 run pymoveit2 ex_collision_primitive.py --ros-args \
  -p shape:="sphere" \
  -p position:="[0.5, 0.0, 0.5]" \
  -p dimensions:="[0.04]"

sleep 2

echo "==> Test 5: pose goal with obstacle present"
ros2 run pymoveit2 ex_pose_goal.py --ros-args \
  -p position:="[0.25, 0.0, 1.0]" \
  -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" \
  -p cartesian:=False

echo "==> All EE tests completed."
