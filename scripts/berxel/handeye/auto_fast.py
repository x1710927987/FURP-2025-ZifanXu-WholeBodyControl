#!/usr/bin/env python3
"""Fast automated eye-to-hand calibration: move → settle → sync sample (pose-only)."""

from __future__ import annotations

import argparse
import json
import shutil
import signal
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from common import (
    call_move_joint_cmd,
    check_roarm_pose_only,
    default_hand_eye_dir,
    default_intrinsics_path,
    detect_apriltag,
    estimate_tag_pose,
    get_aruco_dictionary,
    go_home_pose,
    load_intrinsics,
    make_aruco_detector,
    pose_xyz_rpy_to_matrix,
    read_roarm_pose,
    rvec_tvec_to_matrix,
    save_sample_json,
)
from waypoints import (
    Waypoint,
    filter_waypoints,
    pose_is_diverse,
    pose_translation_m,
    validate_runtime_constraints,
)


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
    """Drop buffered frames so the next read matches current arm pose."""
    for _ in range(frames):
        cap.read()


def backup_samples_dir(samples_dir: Path) -> Path | None:
    existing = list(samples_dir.glob("sample_*.json"))
    if not existing:
        return None
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = samples_dir.parent / f"samples_backup_{stamp}"
    shutil.move(str(samples_dir), str(backup_dir))
    samples_dir.mkdir(parents=True, exist_ok=True)
    return backup_dir


def next_sample_index(samples_dir: Path) -> int:
    return len(list(samples_dir.glob("sample_*.json"))) + 1


def verify_pose_updates(
    home_pose: dict[str, float],
    probe_waypoint: Waypoint,
    wsl_distro: str,
    min_delta_m: float,
) -> None:
    ok, message = call_move_joint_cmd(
        probe_waypoint.x,
        probe_waypoint.y,
        probe_waypoint.z,
        probe_waypoint.roll,
        probe_waypoint.pitch,
        probe_waypoint.yaw,
        wsl_distro=wsl_distro,
    )
    if not ok:
        raise RuntimeError(
            f"Preflight move to probe waypoint failed: {message}. "
            "Check MoveIt / driver stack."
        )
    time.sleep(0.5)
    moved_pose = read_roarm_pose(wsl_distro=wsl_distro, timeout_s=60)
    delta = pose_translation_m(home_pose, moved_pose)
    go_home_pose(home_pose, wsl_distro=wsl_distro)
    if delta < min_delta_m:
        raise RuntimeError(
            "Preflight check failed: /get_pose_cmd did not change after a move "
            f"({delta * 1000:.1f} mm < {min_delta_m * 1000:.1f} mm). "
            "Ensure roarm/launch_driver.sh is running and joint feedback is live."
        )


def show_preview(
    window: str,
    frame: np.ndarray,
    status: str,
    found: bool,
    corners,
    tag_id: int,
    tag_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> bool:
    """Draw overlay; return False if user requests abort."""
    display = frame.copy()
    if found and corners is not None:
        cv2.aruco.drawDetectedMarkers(display, corners, np.array([[tag_id]]))
        rvec, tvec = estimate_tag_pose(corners, tag_size, camera_matrix, dist_coeffs)
        cv2.drawFrameAxes(
            display, camera_matrix, dist_coeffs, rvec, tvec, tag_size * 0.5
        )
    cv2.putText(
        display,
        status,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 220, 0) if found else (0, 0, 255),
        2,
    )
    cv2.imshow(window, display)
    key = cv2.waitKey(1) & 0xFF
    return key not in (ord("q"), ord("Q"), 27)


def capture_synced_sample(
    cap: cv2.VideoCapture,
    wsl_distro: str,
    detector,
    api_mode,
    dictionary,
    tag_id: int,
    tag_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    settle_s: float,
) -> tuple[dict[str, float] | None, np.ndarray | None, np.ndarray | None, str]:
    """Arm is assumed stationary. Read pose, then grab a matched camera frame."""
    time.sleep(settle_s)
    try:
        pose = read_roarm_pose(wsl_distro=wsl_distro, timeout_s=60)
    except RuntimeError as exc:
        return None, None, None, f"pose read failed: {exc}"

    # Flush camera buffer while arm is still holding this pose, then grab.
    flush_camera(cap)
    ok, frame = cap.read()
    if not ok:
        return None, None, None, "camera read failed"

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners, _ = detect_apriltag(gray, detector, api_mode, dictionary, tag_id)
    if not found or corners is None:
        return pose, frame, None, "no tag"

    rvec, tvec = estimate_tag_pose(corners, tag_size, camera_matrix, dist_coeffs)
    T_cam_tag = rvec_tvec_to_matrix(rvec, tvec)
    return pose, frame, T_cam_tag, "ok"


def run_solve(tag_size: float, samples_dir: Path, intrinsics: Path) -> int:
    from solve import run as solve_run

    args = argparse.Namespace(
        samples=str(samples_dir),
        output=str(default_hand_eye_dir() / "extrinsics.yaml"),
        report=None,
        intrinsics=str(intrinsics),
        tag_size=tag_size,
        method="auto",
    )
    return solve_run(args)


def run(args: argparse.Namespace) -> int:
    samples_dir = Path(args.samples).expanduser()
    intrinsics_path = Path(args.intrinsics).expanduser()
    samples_dir.mkdir(parents=True, exist_ok=True)

    if args.fresh:
        backup = backup_samples_dir(samples_dir)
        if backup:
            print(f"Backed up previous samples to {backup}")

    waypoints = filter_waypoints()
    if args.dry_run:
        print("Dry run: accepted waypoints")
        for idx, wp in enumerate(waypoints, start=1):
            print(
                f"  {idx:02d}. {wp.label}: "
                f"x={wp.x:.3f} y={wp.y:.3f} z={wp.z:.3f} "
                f"roll={wp.roll:.3f} pitch={wp.pitch:.3f}"
            )
        return 0

    if not Path(args.device).exists():
        print(f"Error: {args.device} not found. Attach camera to Ubuntu-24.04 first.")
        return 1

    check_roarm_pose_only(wsl_distro=args.roarm_wsl)
    home_pose = read_roarm_pose(wsl_distro=args.roarm_wsl, timeout_s=60)
    print(
        "Home pose:",
        f"x={home_pose['x']:.3f} y={home_pose['y']:.3f} z={home_pose['z']:.3f}",
    )

    if not args.skip_preflight:
        verify_pose_updates(home_pose, waypoints[1], args.roarm_wsl, args.preflight_min_delta_m)
        print("Preflight OK: pose updates after move.")

    camera_matrix, dist_coeffs, intr_meta = load_intrinsics(intrinsics_path)
    width = int(args.width or intr_meta.get("image_width", 640))
    height = int(args.height or intr_meta.get("image_height", 400))
    dictionary = get_aruco_dictionary(args.dictionary)
    detector, api_mode = make_aruco_detector(dictionary)

    cap = open_camera(args.device, width, height)
    warmup_camera(cap)

    stop_flag = {"stop": False}

    def handle_signal(signum: int, _frame) -> None:
        print(f"\nSignal {signum} received, stopping...")
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    saved = 0
    saved_poses: list[dict[str, float]] = []
    for json_path in sorted(samples_dir.glob("sample_*.json")):
        data = json.loads(json_path.read_text(encoding="utf-8"))
        pose = data.get("robot_pose")
        if isinstance(pose, dict):
            saved_poses.append(pose)
            saved += 1
    if saved:
        print(f"Resuming with {saved} existing samples on disk.")

    deadline = time.time() + args.timeout_min * 60.0
    window = "Hand-Eye Auto (sync)"
    wp_index = 0

    print("=== Automated hand-eye calibration (synced move→settle→sample) ===")
    print("Arm holds still while image + /get_pose_cmd are captured together.")
    print(f"Target samples: {args.target_samples}, waypoints: {len(waypoints)}")
    print("Press Q in preview window or Ctrl+C to abort.")

    try:
        while saved < args.target_samples and time.time() < deadline and not stop_flag["stop"]:
            waypoint = waypoints[wp_index % len(waypoints)]
            wp_index += 1

            print(f"Moving to {waypoint.label} ...")
            ok, message = call_move_joint_cmd(
                waypoint.x,
                waypoint.y,
                waypoint.z,
                waypoint.roll,
                waypoint.pitch,
                waypoint.yaw,
                wsl_distro=args.roarm_wsl,
            )
            if not ok:
                print(f"Skip waypoint {waypoint.label}: move failed: {message}")
                continue

            pose, frame, T_cam_tag, status = capture_synced_sample(
                cap,
                args.roarm_wsl,
                detector,
                api_mode,
                dictionary,
                args.tag_id,
                args.tag_size,
                camera_matrix,
                dist_coeffs,
                settle_s=args.settle_s,
            )

            if frame is not None and args.preview:
                found = T_cam_tag is not None
                corners = None
                if found:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    found, corners, _ = detect_apriltag(
                        gray, detector, api_mode, dictionary, args.tag_id
                    )
                if not show_preview(
                    window,
                    frame,
                    f"Saved {saved}/{args.target_samples} | {status}",
                    found,
                    corners,
                    args.tag_id,
                    args.tag_size,
                    camera_matrix,
                    dist_coeffs,
                ):
                    print("Abort requested.")
                    return 1

            if pose is None or frame is None:
                print(f"Skip sample: {status}")
                continue
            if T_cam_tag is None:
                print(f"Skip sample: {status}")
                continue

            ok_constraints, reason = validate_runtime_constraints(pose, None, None)
            if not ok_constraints:
                print(f"Skip sample: {reason}")
                continue

            if not pose_is_diverse(pose, saved_poses):
                print("Skip sample: pose too similar to a previous one.")
                continue

            T_base_gripper = pose_xyz_rpy_to_matrix(
                pose["x"], pose["y"], pose["z"], pose["roll"], pose["pitch"], pose["yaw"]
            )

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
                "waypoint": waypoint.label,
                "robot_pose": pose,
                "T_base_gripper": T_base_gripper.tolist(),
                "T_cam_tag": T_cam_tag.tolist(),
            }
            save_sample_json(json_path, payload)
            saved += 1
            saved_poses.append(pose)
            tag_t = T_cam_tag[:3, 3]
            print(
                f"Saved {stem} ({saved}/{args.target_samples}) "
                f"wp={waypoint.label} "
                f"tcp=({pose['x']:.3f},{pose['y']:.3f},{pose['z']:.3f}) "
                f"tag_t=({tag_t[0]:.3f},{tag_t[1]:.3f},{tag_t[2]:.3f})"
            )
    finally:
        print("Returning home...")
        try:
            go_home_pose(home_pose, wsl_distro=args.roarm_wsl)
        except Exception as exc:  # noqa: BLE001
            print(f"Warning: go_home failed: {exc}")
        cap.release()
        cv2.destroyAllWindows()

    if saved < args.target_samples:
        print(
            f"Timeout or abort with only {saved}/{args.target_samples} samples. "
            "Not running solve."
        )
        return 1

    print(f"Collected {saved} samples. Running solve...")
    return run_solve(args.tag_size, samples_dir, intrinsics_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Synced automated eye-to-hand calibration (move→settle→sample)"
    )
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    parser.add_argument("--intrinsics", default=str(default_intrinsics_path()))
    parser.add_argument("--samples", default=str(default_hand_eye_dir() / "samples"))
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.06)
    parser.add_argument("--roarm-wsl", default="RoArm_M3")
    parser.add_argument("--target-samples", type=int, default=25)
    parser.add_argument(
        "--settle-s",
        type=float,
        default=0.6,
        help="Hold time after move before reading pose + image",
    )
    parser.add_argument("--timeout-min", type=float, default=40.0)
    parser.add_argument("--preflight-min-delta-m", type=float, default=0.005)
    parser.add_argument("--preview", action="store_true", default=True)
    parser.add_argument("--no-preview", action="store_false", dest="preview")
    parser.add_argument("--fresh", action="store_true", help="Backup existing samples first")
    parser.add_argument("--dry-run", action="store_true", help="Print waypoints only")
    parser.add_argument(
        "--skip-preflight",
        action="store_true",
        help="Skip move/pose-change preflight check",
    )
    return parser.parse_args()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(run(parse_args()))
