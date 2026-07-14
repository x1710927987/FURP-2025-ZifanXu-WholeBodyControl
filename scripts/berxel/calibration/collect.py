#!/usr/bin/env python3
"""Interactive checkerboard image collection for Berxel iHawk100 RGB calibration."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

DEFAULT_DEVICE = "/dev/video0"
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 400
DEFAULT_COLS = 8
DEFAULT_ROWS = 6


def default_berxel_data_dir() -> Path:
    return Path(__file__).resolve().parents[3] / "calibration" / "berxel"


def open_camera(device: str, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera device: {device}")
    return cap


def detect_corners(gray: np.ndarray, pattern_size: tuple[int, int]):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def next_image_path(output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    existing = sorted(output_dir.glob("img_*.jpg"))
    index = len(existing) + 1
    return output_dir / f"img_{index:03d}.jpg"


def draw_overlay(frame: np.ndarray, found: bool, corners, saved_count: int, pattern_size):
    display = frame.copy()
    if found and corners is not None:
        cv2.drawChessboardCorners(display, pattern_size, corners, found)
    status = "DETECTED - press S to save" if found else "No board detected"
    color = (0, 200, 0) if found else (0, 0, 255)
    cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(
        display,
        f"Saved: {saved_count} | Q=quit",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
    )
    return display


def run_interactive(args) -> int:
    pattern_size = (args.cols, args.rows)
    output_dir = Path(args.output).expanduser()

    cap = open_camera(args.device, args.width, args.height)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera opened: {actual_w}x{actual_h}")
    print(f"Pattern inner corners: {args.cols}x{args.rows}")
    print(f"Output directory: {output_dir}")
    print("Controls: S=save (only when detected), Q=quit")
    if args.auto:
        print("Auto mode: also saves when board is stable for 10 frames (min 1.5s apart)")

    saved_count = 0
    stable_frames = 0
    last_auto_save = 0.0
    window = "Berxel Calibration Collect"

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = detect_corners(gray, pattern_size)
            display = draw_overlay(frame, found, corners, saved_count, pattern_size)
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q"), 27):
                break

            save_now = False
            if key in (ord("s"), ord("S")) and found:
                save_now = True
            elif args.auto and found:
                stable_frames += 1
                now = time.time()
                if (
                    stable_frames >= 10
                    and saved_count < args.max_images
                    and now - last_auto_save >= 1.5
                ):
                    save_now = True
                    last_auto_save = now
                    stable_frames = 0
            else:
                stable_frames = 0

            if save_now:
                path = next_image_path(output_dir)
                cv2.imwrite(str(path), frame)
                saved_count += 1
                print(f"Saved {path.name} ({saved_count} total)")

            if saved_count >= args.max_images:
                print(f"Reached max images ({args.max_images}), exiting.")
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    print(f"Collection finished. {saved_count} images in {output_dir}")
    return 0 if saved_count > 0 else 1


def parse_args():
    parser = argparse.ArgumentParser(description="Collect checkerboard images for camera calibration")
    parser.add_argument("--device", default=DEFAULT_DEVICE, help="V4L2 device path")
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--cols", type=int, default=DEFAULT_COLS, help="Inner corners along width")
    parser.add_argument("--rows", type=int, default=DEFAULT_ROWS, help="Inner corners along height")
    parser.add_argument(
        "--output",
        default=str(default_berxel_data_dir() / "calib_images"),
        help="Directory for captured images",
    )
    parser.add_argument("--auto", action="store_true", help="Auto-save on stable detection")
    parser.add_argument("--max-images", type=int, default=25, help="Maximum images to collect")
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(run_interactive(parse_args()))
