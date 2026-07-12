#!/usr/bin/env bash
# Briefly release serial from roarm_driver, send torque OFF, optionally restart driver.
set -eo pipefail

RESTART_DRIVER="${1:-1}"
SERIAL="${2:-/dev/ttyUSB0}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash

if [[ ! -e "$SERIAL" ]]; then
  echo "ERROR: $SERIAL not found."
  exit 1
fi

echo "==> Stopping roarm_driver (serial must be free)..."
pkill -f "roarm_driver/roarm_driver" 2>/dev/null || true
pkill -f "ros2 run roarm_driver roarm_driver" 2>/dev/null || true
sleep 1

echo "==> Sending torque OFF..."
python3 "$SCRIPT_DIR/roarm_torque_off.py" "$SERIAL"

if [[ "$RESTART_DRIVER" == "1" ]]; then
  echo "==> Restarting roarm_driver in background..."
  nohup ros2 run roarm_driver roarm_driver > /tmp/roarm_driver.log 2>&1 &
  sleep 2
  echo "==> Driver restarted. Log: /tmp/roarm_driver.log"
else
  echo "==> Driver not restarted (pass 0 as first arg to skip restart)."
fi

echo "==> Done. Joints should now move by hand (power still ON)."
