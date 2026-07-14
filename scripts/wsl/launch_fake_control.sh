#!/usr/bin/env bash
# Launch Panda fake MoveIt control with delayed controller spawners (fixes race on Jazzy).
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

echo "==> Stopping stale ROS nodes (if any)..."
killall ros2_control_node move_group rviz2 2>/dev/null || true
pkill -f "ros2 launch panda_moveit_config" 2>/dev/null || true
sleep 2

echo "==> Starting ex_fake_control.launch.py in background..."
nohup ros2 launch panda_moveit_config ex_fake_control.launch.py > /tmp/panda_fake_control.log 2>&1 &
LAUNCH_PID=$!

echo "==> Waiting 15s for ros2_control_node to initialize..."
sleep 15

echo "==> Spawning controllers (retry-friendly)..."
ros2 run controller_manager spawner joint_state_broadcaster --controller-manager-timeout 30 || true
ros2 run controller_manager spawner joint_trajectory_controller --controller-manager-timeout 30 || true
ros2 run controller_manager spawner gripper_trajectory_controller --controller-manager-timeout 30 || true

echo "==> Stack ready (launch PID: $LAUNCH_PID, log: /tmp/panda_fake_control.log)"
echo "==> Run EE tests in another WSL terminal: bash scripts/wsl_run_ee_tests.sh"
