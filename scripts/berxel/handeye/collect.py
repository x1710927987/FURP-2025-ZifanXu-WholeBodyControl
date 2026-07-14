#!/usr/bin/env python3
"""Collect eye-to-hand samples: RoArm pose + AprilTag pose in camera."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from common import (
    default_hand_eye_dir,
    default_intrinsics_path,
    detect_apriltag,
    estimate_tag_pose,
    get_aruco_dictionary,
    load_intrinsics,
    make_aruco_detector,
    pose_xyz_rpy_to_matrix,
    read_roarm_pose,
    rvec_tvec_to_matrix,
    save_sample_json,
)


def open_camera(device: str, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {device}")
    return cap


def warmup_camera(cap: cv2.VideoCapture, frames: int = 30) -> None:
    for _ in range(frames):
        cap.read()
        time.sleep(0.03)


def next_sample_index(samples_dir: Path) -> int:
    existing = sorted(samples_dir.glob("sample_*.json"))
    return len(existing) + 1


def draw_axes(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_len: float):
    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, axis_len)


def run(args) -> int:
    intrinsics_path = Path(args.intrinsics).expanduser()
    samples_dir = Path(args.output).expanduser()
    samples_dir.mkdir(parents=True, exist_ok=True)

    camera_matrix, dist_coeffs, intr_meta = load_intrinsics(intrinsics_path)
    width = int(args.width or intr_meta.get("image_width", 640))
    height = int(args.height or intr_meta.get("image_height", 400))

    dictionary = get_aruco_dictionary(args.dictionary)
    detector, api_mode = make_aruco_detector(dictionary)

    if not Path(args.device).exists():
        print(f"Error: {args.device} not found. Attach camera first.")
        return 1

    cap = open_camera(args.device, width, height)
    warmup_camera(cap)

    print("=== AprilTag eye-to-hand sample collection ===")
    print(f"Dictionary: {args.dictionary}, tag_id={args.tag_id}, tag_size={args.tag_size} m")
    print(f"Intrinsics: {intrinsics_path}")
    print(f"Samples: {samples_dir}")
    print(f"RoArm WSL: {args.roarm_wsl}")
    print("Controls: S=save sample, Q=quit")
    if args.tag_size < 0.02:
        print("WARNING: tag_size < 20 mm is very small; keep robot close to camera.")

    saved = 0
    window = "Hand-Eye Collect (S=save, Q=quit)"

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners, _marker_id = detect_apriltag(
                gray, detector, api_mode, dictionary, args.tag_id
            )

            display = frame.copy()
            status = "Tag detected - press S" if found else "No target tag detected"
            color = (0, 200, 0) if found else (0, 0, 255)
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(
                display,
                f"Saved: {saved} | tag_size={args.tag_size * 1000:.1f} mm",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            if found and corners is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, np.array([[args.tag_id]]))
                rvec, tvec = estimate_tag_pose(corners, args.tag_size, camera_matrix, dist_coeffs)
                draw_axes(display, camera_matrix, dist_coeffs, rvec, tvec, args.tag_size * 0.5)

            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break

            if key in (ord("s"), ord("S")):
                if not found or corners is None:
                    print("Skip save: target tag not detected")
                    continue
                try:
                    pose = read_roarm_pose(args.roarm_wsl)
                except RuntimeError as exc:
                    print(exc)
                    continue

                rvec, tvec = estimate_tag_pose(corners, args.tag_size, camera_matrix, dist_coeffs)
                T_base_gripper = pose_xyz_rpy_to_matrix(
                    pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"], pose["yaw"]
                )
                T_cam_tag = rvec_tvec_to_matrix(rvec, tvec)

                index = next_sample_index(samples_dir)
                stem = f"sample_{index:03d}"
                image_path = samples_dir / f"{stem}.jpg"
                json_path = samples_dir / f"{stem}.json"

                cv2.imwrite(str(image_path), frame)
                payload = {
                    "index": index,
                    "image": image_path.name,
                    "tag_id": args.tag_id,
                    "tag_size_m": args.tag_size,
                    "dictionary": args.dictionary,
                    "calibration_type": "eye_to_hand",
                    "robot_pose": pose,
                    "T_base_gripper": T_base_gripper.tolist(),
                    "T_cam_tag": T_cam_tag.tolist(),
                }
                save_sample_json(json_path, payload)
                saved += 1
                print(f"Saved {stem} (total {saved})")

                if saved >= args.max_samples:
                    print(f"Reached max samples ({args.max_samples})")
                    break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    print(f"Collection finished. {saved} samples in {samples_dir}")
    return 0 if saved > 0 else 1


def parse_args():
    parser = argparse.ArgumentParser(description="Collect eye-to-hand calibration samples")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--intrinsics", default=str(default_intrinsics_path()))
    parser.add_argument("--output", default=str(default_hand_eye_dir() / "samples"))
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.006, help="Tag outer edge length in meters")
    parser.add_argument("--roarm-wsl", default="RoArm_M3")
    parser.add_argument("--max-samples", type=int, default=20)
    return parser.parse_args()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(run(parse_args()))
