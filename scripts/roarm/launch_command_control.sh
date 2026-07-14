#!/usr/bin/env bash
# Launch command-control stack for RoArm-M3 (RoArm_M3 terminal 2).
# Prerequisite: roarm_launch_driver.sh running. End other control launches first.
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

echo "==> Launching command_control (keep this terminal running)..."
echo "==> Then in terminal 3 run: roarm_run_command_examples.sh [all|pose|joint|line|gripper|led|circle]"
exec ros2 launch roarm_moveit_cmd command_control.launch.py
