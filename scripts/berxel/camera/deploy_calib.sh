#!/bin/bash
# Ensure Berxel calibration data directory and Python deps exist.
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BERXEL_DATA="$REPO/calibration/berxel"

mkdir -p "$BERXEL_DATA/calib_images"
python3 -c "import yaml" 2>/dev/null || pip3 install --user pyyaml

echo "Berxel calibration data directory ready:"
ls -la "$BERXEL_DATA"
