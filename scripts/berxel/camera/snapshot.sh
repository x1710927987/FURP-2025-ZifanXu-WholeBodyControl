#!/bin/bash
# Capture a single test frame from Berxel iHawk100 (run in Ubuntu-24.04 WSL).
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT="$REPO/calibration/berxel/snapshot.jpg"

mkdir -p "$(dirname "$OUTPUT")"

if [ ! -e /dev/video0 ]; then
    echo "Error: /dev/video0 not found. Attach camera first."
    exit 1
fi

python3 << PYEOF
import cv2
import time

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)
for _ in range(30):
    cap.read()
    time.sleep(0.05)
ret, frame = cap.read()
if ret:
    cv2.imwrite("$OUTPUT", frame)
    print(f"Saved: $OUTPUT ({frame.shape[1]}x{frame.shape[0]})")
else:
    raise SystemExit("Capture failed")
cap.release()
PYEOF
