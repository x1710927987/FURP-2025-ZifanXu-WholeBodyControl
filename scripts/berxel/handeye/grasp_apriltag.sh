#!/bin/bash
# Two-stage AprilTag approach (hover then descend). Run in Ubuntu-24.04.
# Examples:
#   bash grasp_apriltag.sh --tag-size 0.06 --dry-run --once
#   bash grasp_apriltag.sh --tag-size 0.06 --safe-orientation --once
#   bash grasp_apriltag.sh --tag-size 0.06 --safe-orientation --redetect
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" \
  python3 "$SCRIPT_DIR/grasp_apriltag.py" "$@"
