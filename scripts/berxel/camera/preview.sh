#!/bin/bash
# Berxel iHawk100Q live RGB preview (run in Ubuntu-24.04 WSL).
set -eu

CAMERA_DEV="/dev/video0"
RESOLUTION="640x400"

echo "=== Berxel iHawk100 preview ==="

if [ ! -e "$CAMERA_DEV" ]; then
    echo "Error: $CAMERA_DEV not found. Attach camera first:"
    echo "  powershell -ExecutionPolicy Bypass -File scripts/berxel_attach_camera.ps1"
    exit 1
fi

echo "Device info:"
v4l2-ctl --list-devices 2>/dev/null || true
echo ""

if command -v ffplay &>/dev/null; then
    echo "Starting ffplay preview (press q to quit)..."
    ffplay -f v4l2 -input_format mjpeg -video_size "$RESOLUTION" -framerate 30 "$CAMERA_DEV"
    exit 0
fi

if python3 -c "import cv2" 2>/dev/null; then
    echo "Starting OpenCV preview (press q to quit)..."
    python3 << 'PYEOF'
import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

if not cap.isOpened():
    cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("Berxel iHawk100 - Press Q to quit", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
PYEOF
    exit 0
fi

echo "Error: install ffmpeg or python3-opencv"
