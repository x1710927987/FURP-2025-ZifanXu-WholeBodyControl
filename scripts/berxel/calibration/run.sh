#!/bin/bash
# Run checkerboard calibration on collected images.
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
BERXEL_DATA="${BERXEL_DATA_DIR:-$REPO/calibration/berxel}"

python3 -c "import yaml" 2>/dev/null || pip3 install --user pyyaml

python3 "$SCRIPT_DIR/berxel_calib_run.py" \
    --images "$BERXEL_DATA/calib_images" \
    --output "$BERXEL_DATA/intrinsics.yaml" \
    --cols 8 --rows 6 --square-size 0.025 \
    "$@"

echo ""
echo "Results:"
echo "  $BERXEL_DATA/intrinsics.yaml"
echo "  $BERXEL_DATA/calib_report.txt"
echo "  $BERXEL_DATA/undistort_sample.jpg"
