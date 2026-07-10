#!/usr/bin/env bash
# MoveIt2 Servo stack for RoArm-M3 keyboard control (RoArm_M3 terminal 2).
# Prerequisite: roarm_launch_driver.sh running in another terminal.
# End any display.launch.py or roarm_moveit.launch.py before starting this.
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

echo "==> Launching MoveIt Servo (keep this terminal running)..."
echo "==> Then open terminal 3 and run: roarm_keyboard_control.sh"
exec ros2 launch roarm_moveit_servo servo_control.launch.py
