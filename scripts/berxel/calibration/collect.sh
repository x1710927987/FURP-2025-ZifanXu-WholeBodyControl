#!/bin/bash
# Collect checkerboard calibration images for Berxel iHawk100.
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BERXEL_DATA="${BERXEL_DATA_DIR:-$REPO/calibration/berxel}"

mkdir -p "$BERXEL_DATA/calib_images"

if [ ! -e /dev/video0 ]; then
    echo "Error: /dev/video0 not found. Attach camera first:"
    echo "  powershell -ExecutionPolicy Bypass -File scripts/berxel/camera/attach_camera.ps1"
    exit 1
fi

python3 "$SCRIPT_DIR/collect.py" \
    --output "$BERXEL_DATA/calib_images" \
    --cols 8 --rows 6 \
    "$@"
