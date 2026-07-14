#!/bin/bash
# Move RoArm TCP to a point in the camera optical frame (Ubuntu-24.04).
# Example (10 cm along camera +X):
#   bash move_to_cam_point.sh --cam-xyz 10 0 0
#   bash move_to_cam_point.sh --cam-xyz 10 0 0 --dry-run
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" \
  python3 "$SCRIPT_DIR/move_to_cam_point.py" "$@"
