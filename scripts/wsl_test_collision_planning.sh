#!/usr/bin/env bash
# Collision-aware planning: add obstacle sphere, then plan EE pose (WSL only)
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "==> Step 1: Add collision sphere to planning scene"
ros2 run pymoveit2 ex_collision_primitive.py --ros-args \
  -p shape:="sphere" \
  -p position:="[0.45, 0.0, 0.35]" \
  -p dimensions:="[0.06]"

sleep 2

echo "==> Step 2: Plan EE pose (should avoid obstacle)"
ros2 run pymoveit2 ex_pose_goal.py --ros-args \
  -p position:="[0.28, 0.12, 0.95]" \
  -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" \
  -p cartesian:=False

sleep 2

echo "==> Step 3: Clear planning scene (optional; needs: pip3 install trimesh)"
timeout 15 ros2 run pymoveit2 ex_clear_planning_scene.py || echo "WARN: clear scene skipped or timed out"

echo "==> Collision planning test finished."
