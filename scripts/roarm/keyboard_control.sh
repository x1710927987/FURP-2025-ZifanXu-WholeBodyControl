#!/usr/bin/env bash
# Keyboard teleop for RoArm-M3 via MoveIt Servo (RoArm_M3 terminal 3).
# Prerequisites:
#   - roarm_launch_driver.sh  (terminal 1)
#   - roarm_launch_servo.sh   (terminal 2, still running)
# Keep THIS terminal focused to receive key presses. Press Q to quit.
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash

echo "==> Starting keyboard control (focus this window; Q to quit)..."
exec ros2 run roarm_moveit_servo keyboardcontrol
