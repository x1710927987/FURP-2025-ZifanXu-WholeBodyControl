#!/usr/bin/env python3
"""Compute camera intrinsics from checkerboard calibration images."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml


def default_berxel_data_dir() -> Path:
    return Path(__file__).resolve().parent.parent / "calibration" / "berxel"


def build_object_points(pattern_size: tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = pattern_size
    objp = np.zeros((rows * cols, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp[:, :2] = grid * square_size
    return objp


def detect_corners(gray: np.ndarray, pattern_size: tuple[int, int]):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def collect_points(image_paths: list[Path], pattern_size: tuple[int, int], square_size: float):
    objp = build_object_points(pattern_size, square_size)
    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []
    image_size = None
    report_lines: list[str] = []

    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            report_lines.append(f"FAIL read: {path.name}")
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        found, corners = detect_corners(gray, pattern_size)
        if found and corners is not None:
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            report_lines.append(f"OK   {path.name}")
        else:
            report_lines.append(f"SKIP {path.name} (corners not found)")

    return objpoints, imgpoints, image_size, report_lines


def save_undistort_sample(
    sample_path: Path,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    output_path: Path,
    pattern_size: tuple[int, int],
):
    image = cv2.imread(str(sample_path))
    if image is None:
        return
    undistorted = cv2.undistort(image, camera_matrix, dist_coeffs)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = detect_corners(gray, pattern_size)
    if found and corners is not None:
        cv2.drawChessboardCorners(image, pattern_size, corners, found)
        cv2.drawChessboardCorners(undistorted, pattern_size, corners, found)
    combined = np.hstack([image, undistorted])
    cv2.putText(combined, "original", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.putText(
        combined,
        "undistorted",
        (image.shape[1] + 10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
    )
    cv2.imwrite(str(output_path), combined)


def run_calibration(args) -> int:
    pattern_size = (args.cols, args.rows)
    images_dir = Path(args.images).expanduser()
    output_path = Path(args.output).expanduser()
    report_path = Path(args.report).expanduser() if args.report else output_path.with_name("calib_report.txt")
    undistort_path = Path(args.undistort).expanduser() if args.undistort else output_path.with_name(
        "undistort_sample.jpg"
    )

    image_paths = sorted(
        p for p in images_dir.iterdir() if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}
    )
    if not image_paths:
        print(f"No images found in {images_dir}")
        return 1

    objpoints, imgpoints, image_size, report_lines = collect_points(
        image_paths, pattern_size, args.square_size
    )
    if image_size is None:
        print("Could not read any image")
        return 1
    if len(objpoints) < 3:
        print(f"Need at least 3 valid images, got {len(objpoints)}")
        return 1

    rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    result = {
        "image_width": int(image_size[0]),
        "image_height": int(image_size[1]),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.reshape(-1).tolist(),
        "reprojection_error": float(rms),
        "valid_image_count": len(objpoints),
        "checkerboard": {
            "cols": args.cols,
            "rows": args.rows,
            "square_size_m": args.square_size,
        },
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(result, f, sort_keys=False, allow_unicode=True)

    summary = [
        f"Images scanned: {len(image_paths)}",
        f"Valid images: {len(objpoints)}",
        f"RMS reprojection error: {rms:.4f} px",
        f"Output: {output_path}",
        "",
        "Per-image results:",
        *report_lines,
    ]
    report_path.write_text("\n".join(summary) + "\n", encoding="utf-8")
    print("\n".join(summary[:5]))

    first_ok = None
    for line, path in zip(report_lines, image_paths):
        if line.startswith("OK"):
            first_ok = path
            break
    if first_ok is not None:
        save_undistort_sample(first_ok, camera_matrix, dist_coeffs, undistort_path, pattern_size)
        print(f"Undistort sample: {undistort_path}")

    if rms > 1.0:
        print("WARNING: RMS > 1.0 px. Consider re-collecting with more poses.")
        return 2
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description="Run checkerboard camera calibration")
    parser.add_argument("--images", default=str(default_berxel_data_dir() / "calib_images"))
    parser.add_argument("--output", default=str(default_berxel_data_dir() / "intrinsics.yaml"))
    parser.add_argument("--report", default=None)
    parser.add_argument("--undistort", default=None)
    parser.add_argument("--cols", type=int, default=8)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--square-size", type=float, default=0.025, dest="square_size")
    return parser.parse_args()


if __name__ == "__main__":
    sys.exit(run_calibration(parse_args()))
