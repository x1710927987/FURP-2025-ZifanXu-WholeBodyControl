#!/bin/bash
# Detect AprilTag and print MTC pick_place object_pose / run_mtc.sh command.
# Run in Ubuntu-24.04 (camera + extrinsics). Then paste the printed command
# into a RoArm_M3 terminal that has driver + launch_mtc_demo running.
#
# Example:
#   bash tag_to_mtc_pose.sh --tag-size 0.06
#   bash tag_to_mtc_pose.sh --tag-size 0.06 --tag-is top --object-height 0.05
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" \
  python3 "$SCRIPT_DIR/tag_to_mtc_pose.py" "$@"
