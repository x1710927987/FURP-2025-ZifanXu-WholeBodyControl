#!/bin/bash
# Collect AprilTag eye-to-hand calibration samples.
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ ! -e /dev/video0 ]; then
    echo "Error: /dev/video0 not found. Attach camera first:"
    echo "  powershell -ExecutionPolicy Bypass -File scripts/berxel_attach_camera.ps1"
    exit 1
fi

mkdir -p "$REPO/calibration/hand_eye/samples"
PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}" python3 "$SCRIPT_DIR/berxel_handeye_collect.py" "$@"
