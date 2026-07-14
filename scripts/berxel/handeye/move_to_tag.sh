#!/bin/bash
# Detect AprilTag and move RoArm TCP to it (eye-to-hand). Run in Ubuntu-24.04.
# Examples:
#   bash move_to_tag.sh --tag-size 0.06 --dry-run --once
#   bash move_to_tag.sh --tag-size 0.06 --safe-orientation --once
#   bash move_to_tag.sh --tag-size 0.06 --offset 0 0 0.08 --offset-frame base
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" \
  python3 "$SCRIPT_DIR/move_to_tag.py" "$@"
