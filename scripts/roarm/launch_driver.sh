#!/usr/bin/env bash
# Start roarm_driver for RoArm-M3 (run inside RoArm_M3 WSL; keep terminal open).
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash

SERIAL="${1:-/dev/ttyUSB0}"
if [[ ! -e "$SERIAL" ]]; then
  echo "ERROR: $SERIAL not found. Run roarm_attach_usb.ps1 from Admin PowerShell first."
  exit 1
fi

echo "==> Starting roarm_driver on $SERIAL (arm will move on connect)..."
if [[ "$SERIAL" == "/dev/ttyUSB0" ]]; then
  exec ros2 run roarm_driver roarm_driver
else
  exec ros2 run roarm_driver roarm_driver --ros-args -p "serial_port:=$SERIAL"
fi
