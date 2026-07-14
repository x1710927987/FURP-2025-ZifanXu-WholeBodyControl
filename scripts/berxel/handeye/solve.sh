#!/bin/bash
# Solve AprilTag eye-to-hand calibration from collected samples.
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"

python3 -c "import yaml" 2>/dev/null || pip3 install --user pyyaml

PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" python3 "$SCRIPT_DIR/solve.py" \
  --samples "$REPO/calibration/hand_eye/samples" \
  --output "$REPO/calibration/hand_eye/extrinsics.yaml" \
  "$@"

echo ""
echo "Results:"
echo "  $REPO/calibration/hand_eye/extrinsics.yaml"
echo "  $REPO/calibration/hand_eye/hand_eye_report.txt"
