#!/usr/bin/env python3
"""Detect AprilTag in the fixed camera and move RoArm TCP to it (eye-to-hand)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

from common import (
    call_move_joint_cmd,
    check_roarm_pose_only,
    default_hand_eye_dir,
    default_intrinsics_path,
    detect_apriltag,
    estimate_tag_pose,
    get_aruco_dictionary,
    load_intrinsics,
    make_aruco_detector,
    read_roarm_pose,
    rvec_tvec_to_matrix,
)


def load_T_base_cam(path: Path) -> np.ndarray:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if "T_base_cam" not in data:
        raise KeyError(f"T_base_cam missing in {path}")
    return np.array(data["T_base_cam"], dtype=np.float64)


def open_camera(device: str, width: int, height: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {device}")
    return cap


def warmup_camera(cap: cv2.VideoCapture, frames: int = 30) -> None:
    for _ in range(frames):
        cap.read()
        time.sleep(0.03)


def flush_camera(cap: cv2.VideoCapture, frames: int = 8) -> None:
    for _ in range(frames):
        cap.read()


def cam_to_base_transform(T_base_cam: np.ndarray, T_cam_tag: np.ndarray) -> np.ndarray:
    """Eye-to-hand: fixed camera in base → T_base_tag = T_base_cam @ T_cam_tag."""
    return T_base_cam @ T_cam_tag


def apply_offset(
    T_base_tag: np.ndarray,
    offset_xyz: np.ndarray,
    offset_frame: str,
) -> np.ndarray:
    """Return target XYZ in base frame after applying offset in base or tag frame."""
    tag_xyz = T_base_tag[:3, 3].copy()
    if offset_frame == "base":
        return tag_xyz + offset_xyz
    if offset_frame == "tag":
        return tag_xyz + (T_base_tag[:3, :3] @ offset_xyz)
    raise ValueError(f"Unknown offset frame: {offset_frame}")


def warn_reachability(p_base: np.ndarray) -> None:
    if p_base[0] < 0.0 or p_base[2] < 0.0:
        print("WARNING: target has x<0 or z<0 (outside usual calibration workspace).")
    horiz = float(np.linalg.norm(p_base[:2]))
    if horiz > 0.45:
        print(
            f"WARNING: horizontal reach {horiz:.3f} m is large for RoArm-M3; "
            "IK/planning may fail."
        )


def resolve_orientation(
    args: argparse.Namespace,
    current: dict[str, float],
) -> tuple[float, float, float]:
    if args.safe_orientation:
        return 0.0, 0.0, 0.0
    if args.keep_orientation:
        return current["roll"], current["pitch"], current.get("yaw", 0.0)
    return args.roll, args.pitch, args.yaw


def detect_once(
    cap: cv2.VideoCapture,
    detector,
    api_mode: str,
    dictionary,
    tag_id: int,
    tag_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None, np.ndarray]:
    """Return (T_cam_tag, rvec, tvec, frame). T_cam_tag is None if not found."""
    flush_camera(cap)
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError("Failed to read camera frame")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners, _ = detect_apriltag(gray, detector, api_mode, dictionary, tag_id)
    if not found or corners is None:
        return None, None, None, frame
    rvec, tvec = estimate_tag_pose(corners, tag_size, camera_matrix, dist_coeffs)
    T_cam_tag = rvec_tvec_to_matrix(rvec, tvec)
    return T_cam_tag, rvec, tvec, frame


def move_tcp_to(
    p_base: np.ndarray,
    roll: float,
    pitch: float,
    yaw: float,
    wsl_distro: str,
    dry_run: bool,
) -> int:
    warn_reachability(p_base)
    print(
        f"TCP target (base, m): xyz=({p_base[0]:.4f}, {p_base[1]:.4f}, {p_base[2]:.4f}) "
        f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f})"
    )
    if dry_run:
        print("Dry-run only - not moving the arm.")
        return 0

    check_roarm_pose_only(wsl_distro=wsl_distro)
    ok, message = call_move_joint_cmd(
        float(p_base[0]),
        float(p_base[1]),
        float(p_base[2]),
        float(roll),
        float(pitch),
        float(yaw),
        wsl_distro=wsl_distro,
    )
    if not ok:
        print(f"Move failed: {message}")
        return 1

    final = read_roarm_pose(wsl_distro=wsl_distro, timeout_s=60)
    err = float(
        np.linalg.norm(np.array([final["x"], final["y"], final["z"]]) - p_base)
    )
    print(
        "Done. Reached: "
        f"x={final['x']:.4f} y={final['y']:.4f} z={final['z']:.4f} "
        f"(position error {err * 1000:.1f} mm vs target)"
    )
    return 0


def compute_target_from_detection(
    T_base_cam: np.ndarray,
    T_cam_tag: np.ndarray,
    offset_xyz: np.ndarray,
    offset_frame: str,
) -> tuple[np.ndarray, np.ndarray]:
    T_base_tag = cam_to_base_transform(T_base_cam, T_cam_tag)
    p_base = apply_offset(T_base_tag, offset_xyz, offset_frame)
    return T_base_tag, p_base


def run(args: argparse.Namespace) -> int:
    extrinsics = Path(args.extrinsics).expanduser()
    intrinsics_path = Path(args.intrinsics).expanduser()
    if not extrinsics.is_file():
        print(f"Missing extrinsics: {extrinsics}")
        return 1
    if not Path(args.device).exists():
        print(f"Error: {args.device} not found. Attach camera first.")
        return 1

    T_base_cam = load_T_base_cam(extrinsics)
    camera_matrix, dist_coeffs, intr_meta = load_intrinsics(intrinsics_path)
    width = int(args.width or intr_meta.get("image_width", 640))
    height = int(args.height or intr_meta.get("image_height", 400))
    dictionary = get_aruco_dictionary(args.dictionary)
    detector, api_mode = make_aruco_detector(dictionary)
    offset_xyz = np.array(args.offset, dtype=np.float64)

    cap = open_camera(args.device, width, height)
    warmup_camera(cap)

    print("=== Move TCP to AprilTag (eye-to-hand) ===")
    print(f"Tag: id={args.tag_id}, size={args.tag_size} m, dict={args.dictionary}")
    print(f"Offset ({args.offset_frame}): {offset_xyz.tolist()} m")
    print("Controls: G=go (detect+move), Q=quit" if not args.once else "Mode: --once")

    window = "Move to Tag (G=go, Q=quit)"
    try:
        if args.once:
            time.sleep(args.settle_s)
            T_cam_tag, rvec, tvec, frame = detect_once(
                cap,
                detector,
                api_mode,
                dictionary,
                args.tag_id,
                args.tag_size,
                camera_matrix,
                dist_coeffs,
            )
            if T_cam_tag is None:
                print("AprilTag not detected.")
                return 1
            tag_t = T_cam_tag[:3, 3]
            print(
                f"Detected tag in camera (m): "
                f"t=({tag_t[0]:.4f}, {tag_t[1]:.4f}, {tag_t[2]:.4f})"
            )
            T_base_tag, p_base = compute_target_from_detection(
                T_base_cam, T_cam_tag, offset_xyz, args.offset_frame
            )
            tag_b = T_base_tag[:3, 3]
            print(
                f"Tag in base (m): "
                f"({tag_b[0]:.4f}, {tag_b[1]:.4f}, {tag_b[2]:.4f})"
            )
            if not args.dry_run:
                check_roarm_pose_only(wsl_distro=args.roarm_wsl)
                current = read_roarm_pose(wsl_distro=args.roarm_wsl, timeout_s=60)
            else:
                current = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            roll, pitch, yaw = resolve_orientation(args, current)
            return move_tcp_to(
                p_base, roll, pitch, yaw, args.roarm_wsl, args.dry_run
            )

        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame")
                return 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners, _ = detect_apriltag(
                gray, detector, api_mode, dictionary, args.tag_id
            )
            display = frame.copy()
            status = "Tag OK - press G" if found else "No tag"
            color = (0, 200, 0) if found else (0, 0, 255)
            cv2.putText(display, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            if found and corners is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, np.array([[args.tag_id]]))
                rvec, tvec = estimate_tag_pose(
                    corners, args.tag_size, camera_matrix, dist_coeffs
                )
                cv2.drawFrameAxes(
                    display, camera_matrix, dist_coeffs, rvec, tvec, args.tag_size * 0.5
                )
            cv2.imshow(window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                return 0
            if key not in (ord("g"), ord("G")):
                continue
            if not found or corners is None:
                print("Skip: tag not detected")
                continue

            time.sleep(args.settle_s)
            T_cam_tag, _r, _t, _f = detect_once(
                cap,
                detector,
                api_mode,
                dictionary,
                args.tag_id,
                args.tag_size,
                camera_matrix,
                dist_coeffs,
            )
            if T_cam_tag is None:
                print("Skip: tag lost after settle")
                continue

            T_base_tag, p_base = compute_target_from_detection(
                T_base_cam, T_cam_tag, offset_xyz, args.offset_frame
            )
            tag_b = T_base_tag[:3, 3]
            tag_c = T_cam_tag[:3, 3]
            print(
                f"Cam tag t=({tag_c[0]:.4f}, {tag_c[1]:.4f}, {tag_c[2]:.4f}) | "
                f"Base tag=({tag_b[0]:.4f}, {tag_b[1]:.4f}, {tag_b[2]:.4f})"
            )
            if not args.dry_run:
                check_roarm_pose_only(wsl_distro=args.roarm_wsl)
                current = read_roarm_pose(wsl_distro=args.roarm_wsl, timeout_s=60)
            else:
                current = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
            roll, pitch, yaw = resolve_orientation(args, current)
            rc = move_tcp_to(p_base, roll, pitch, yaw, args.roarm_wsl, args.dry_run)
            if rc != 0:
                return rc
            if args.once_after_go:
                return 0
    finally:
        cap.release()
        cv2.destroyAllWindows()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Detect AprilTag with fixed camera and move TCP to tag "
            "(eye-to-hand: T_base_tag = T_base_cam @ T_cam_tag)."
        )
    )
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--intrinsics", default=str(default_intrinsics_path()))
    parser.add_argument(
        "--extrinsics",
        default=str(default_hand_eye_dir() / "extrinsics.yaml"),
    )
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument(
        "--tag-size",
        type=float,
        default=0.06,
        help="Tag outer edge length in meters (default 0.06 = 60 mm)",
    )
    parser.add_argument(
        "--offset",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.05],
        metavar=("DX", "DY", "DZ"),
        help="TCP offset from tag origin (meters). Default: 5 cm above in offset-frame",
    )
    parser.add_argument(
        "--offset-frame",
        choices=("base", "tag"),
        default="base",
        help="Interpret --offset in base (+Z up) or tag frame (default: base)",
    )
    parser.add_argument("--roarm-wsl", default="RoArm_M3")
    parser.add_argument(
        "--keep-orientation",
        action="store_true",
        default=True,
        help="Keep current TCP rpy (default)",
    )
    parser.add_argument(
        "--set-orientation",
        action="store_false",
        dest="keep_orientation",
        help="Use --roll/--pitch/--yaw instead of current orientation",
    )
    parser.add_argument(
        "--safe-orientation",
        action="store_true",
        help="Use roll=pitch=yaw=0 (often more reachable)",
    )
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument(
        "--once",
        action="store_true",
        help="Detect once and move (no preview loop)",
    )
    parser.add_argument(
        "--once-after-go",
        action="store_true",
        help="In preview mode, exit after one successful G",
    )
    parser.add_argument("--settle-s", type=float, default=0.3)
    parser.add_argument("--dry-run", action="store_true")
    return parser


def parse_args() -> argparse.Namespace:
    return build_parser().parse_args()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(run(parse_args()))
