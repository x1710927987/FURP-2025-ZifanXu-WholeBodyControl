#!/usr/bin/env bash
# RViz joint sliders for RoArm-M3 (run inside RoArm_M3 WSL; roarm_driver must be running).
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

echo "==> Launching display.launch.py (arm follows RViz sliders)..."
exec ros2 launch roarm_description display.launch.py
