#!/usr/bin/env bash
# Launch MoveIt Task Constructor demo UI for RoArm-M3 (RoArm_M3 terminal 2).
# Prerequisite: roarm_launch_driver.sh running. End other control launches first.
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

echo "==> Launching MTC demo UI (keep this terminal running)..."
echo "==> In terminal 3 run ONE of:"
echo "    roarm_run_mtc.sh cartesian"
echo "    roarm_run_mtc.sh cartesian_modular"
echo "    roarm_run_mtc.sh pick_place"
echo "==> Then select the path in the UI and click Exec."
exec ros2 launch roarm_moveit_mtc_demo demo.launch.py
