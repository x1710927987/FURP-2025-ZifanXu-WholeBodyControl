#!/usr/bin/env bash
# Run RoArm-M3 command-control service examples (RoArm_M3 terminal 3).
# Prerequisites: roarm_launch_driver.sh + roarm_launch_command_control.sh running.
# Usage: roarm_run_command_examples.sh [all|pose|joint|line|gripper|led|circle]
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash

MODE="${1:-all}"
REPO_HINT="roarm_launch_command_control.sh"

if ! ros2 service list 2>/dev/null | grep -q '/get_pose_cmd'; then
  echo "ERROR: /get_pose_cmd not found. Start ${REPO_HINT} in terminal 2 first."
  exit 1
fi

run_pose() {
  echo "==> Get current pose (read only)"
  ros2 service call /get_pose_cmd roarm_msgs/srv/GetPoseCmd
}

run_joint() {
  echo "==> Move end-effector to pose (MoveJointCmd)"
  ros2 service call /move_joint_cmd roarm_msgs/srv/MoveJointCmd \
    "{x: 0.3, y: 0.0, z: 0.1, roll: 0.2, pitch: 0.2, yaw: 0.0}"
}

run_line() {
  echo "==> Move end-effector in a straight line (MoveLineCmd)"
  ros2 service call /move_line_cmd roarm_msgs/srv/MoveLineCmd \
    "{x: 0.2, y: 0.2, z: 0.1}"
}

run_gripper() {
  echo "==> Set gripper target (rad, range ~0.0-1.5)"
  ros2 topic pub /gripper_cmd std_msgs/msg/Float32 "{data: 0.5}" -1
}

run_led() {
  echo "==> Set gripper LED brightness (0-255)"
  ros2 topic pub /led_ctrl std_msgs/msg/Float32 "{data: 0.0}" -1
}

run_circle() {
  echo "==> Move along arc (MoveCircleCmd)"
  ros2 service call /move_circle_cmd roarm_msgs/srv/MoveCircleCmd \
    "{x0: 0.2, y0: 0.1, z0: 0.2, x1: 0.2, y1: 0.2, z1: 0.2}"
}

case "$MODE" in
  all)
    run_pose
    sleep 3
    run_joint
    sleep 5
    run_line
    sleep 5
    run_gripper
    sleep 2
    run_led
    sleep 2
    run_circle
    ;;
  pose)   run_pose ;;
  joint)  run_joint ;;
  line)   run_line ;;
  gripper) run_gripper ;;
  led)    run_led ;;
  circle) run_circle ;;
  *)
    echo "Usage: $0 [all|pose|joint|line|gripper|led|circle]"
    exit 1
    ;;
esac

echo "==> Done."
