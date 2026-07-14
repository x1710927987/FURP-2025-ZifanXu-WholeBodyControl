#!/usr/bin/env python3
"""Automated eye-to-hand calibration: move RoArm, sample AprilTag at 1 Hz, solve."""

from __future__ import annotations

import argparse
import shutil
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from common import (
    call_move_joint_cmd,
    check_roarm_stack,
    default_hand_eye_dir,
    default_intrinsics_path,
    detect_apriltag,
    estimate_tag_pose,
    get_aruco_dictionary,
    go_home_pose,
    load_intrinsics,
    make_aruco_detector,
    pose_xyz_rpy_to_matrix,
    read_joint_states,
    read_roarm_pose,
    rvec_tvec_to_matrix,
    save_sample_json,
)
from waypoints import (
    GRIPPER_JOINT,
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
    moved_pose = read_roarm_pose(wsl_distro=wsl_distro)
    delta = pose_translation_m(home_pose, moved_pose)
    go_home_pose(home_pose, wsl_distro=wsl_distro)
    if delta < min_delta_m:
        raise RuntimeError(
            "Preflight check failed: /get_pose_cmd did not change after a move "
            f"({delta * 1000:.1f} mm < {min_delta_m * 1000:.1f} mm). "
            "Ensure roarm/launch_driver.sh is running and joint feedback is live."
        )


class MotionController:
    def __init__(
        self,
        waypoints: list[Waypoint],
        home_pose: dict[str, float],
        wsl_distro: str,
        stop_event: threading.Event,
    ) -> None:
        self.waypoints = waypoints
        self.home_pose = home_pose
        self.wsl_distro = wsl_distro
        self.stop_event = stop_event
        self.moving = threading.Event()
        self.last_move_ok = False
        self._thread = threading.Thread(target=self._run, name="handeye-motion", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        index = 0
        while not self.stop_event.is_set():
            waypoint = self.waypoints[index % len(self.waypoints)]
            self.moving.set()
            ok, message = call_move_joint_cmd(
                waypoint.x,
                waypoint.y,
                waypoint.z,
                waypoint.roll,
                waypoint.pitch,
                waypoint.yaw,
                wsl_distro=self.wsl_distro,
            )
            self.last_move_ok = ok
            self.moving.clear()
            if not ok:
                print(f"Move failed ({waypoint.label}): {message}")
            else:
                time.sleep(0.4)
            index += 1
        print("Motion loop stopping, returning home...")
        go_home_pose(self.home_pose, wsl_distro=self.wsl_distro)


def run_solve(tag_size: float, samples_dir: Path, intrinsics: Path) -> int:
    from solve import run as solve_run

    args = argparse.Namespace(
        samples=str(samples_dir),
        output=str(default_hand_eye_dir() / "extrinsics.yaml"),
        report=None,
        intrinsics=str(intrinsics),
        tag_size=tag_size,
        method="CALIB_HAND_EYE_TSAI",
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

    gripper_home_rad = check_roarm_stack(wsl_distro=args.roarm_wsl)
    home_pose = read_roarm_pose(wsl_distro=args.roarm_wsl)
    try:
        home_joints = read_joint_states(wsl_distro=args.roarm_wsl, timeout_s=90)
    except RuntimeError:
        home_joints = None
    if home_joints is not None and gripper_home_rad is None:
        gripper_home_rad = home_joints.get(GRIPPER_JOINT)
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

    stop_event = threading.Event()

    def handle_signal(signum: int, _frame) -> None:
        print(f"\nSignal {signum} received, stopping...")
        stop_event.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    motion = MotionController(waypoints, home_pose, args.roarm_wsl, stop_event)
    motion.start()

    saved = 0
    saved_poses: list[dict[str, float]] = []
    deadline = time.time() + args.timeout_min * 60.0
    next_sample_at = time.time()
    window = "Hand-Eye Auto"

    print("=== Automated hand-eye calibration (full constraints) ===")
    print(f"Target samples: {args.target_samples}, interval: {args.interval_s}s")
    print("Press Q in preview window or Ctrl+C to abort.")

    try:
        while saved < args.target_samples and time.time() < deadline and not stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                print("Camera read failed.")
                break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners, _marker_id = detect_apriltag(
                gray, detector, api_mode, dictionary, args.tag_id
            )

            display = frame.copy()
            status = f"Saved {saved}/{args.target_samples}"
            if found and corners is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, np.array([[args.tag_id]]))
                rvec, tvec = estimate_tag_pose(
                    corners, args.tag_size, camera_matrix, dist_coeffs
                )
                cv2.drawFrameAxes(
                    display,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    args.tag_size * 0.5,
                )
                status += " | tag OK"
            else:
                status += " | no tag"

            cv2.putText(
                display,
                status,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 220, 0) if found else (0, 0, 255),
                2,
            )
            if args.preview:
                cv2.imshow(window, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    print("Abort requested.")
                    return 1

            now = time.time()
            if now < next_sample_at:
                continue
            next_sample_at = now + args.interval_s

            if not found or corners is None:
                continue

            try:
                pose = read_roarm_pose(wsl_distro=args.roarm_wsl)
                try:
                    joints = read_joint_states(wsl_distro=args.roarm_wsl, timeout_s=90)
                except RuntimeError:
                    joints = None
            except RuntimeError as exc:
                print(f"Skip sample: {exc}")
                continue

            ok_constraints, reason = validate_runtime_constraints(
                pose, joints, gripper_home_rad
            )
            if not ok_constraints:
                print(f"Skip sample: {reason}")
                continue

            if not pose_is_diverse(pose, saved_poses):
                print("Skip sample: pose too similar to a previous one.")
                continue

            rvec, tvec = estimate_tag_pose(
                corners, args.tag_size, camera_matrix, dist_coeffs
            )
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
                "joint_positions": joints,
                "T_base_gripper": T_base_gripper.tolist(),
                "T_cam_tag": T_cam_tag.tolist(),
            }
            save_sample_json(json_path, payload)
            saved += 1
            saved_poses.append(pose)
            print(f"Saved {stem} ({saved}/{args.target_samples})")

            if saved >= args.target_samples:
                break
    finally:
        stop_event.set()
        motion.join(timeout=180.0)
        cap.release()
        cv2.destroyAllWindows()

    if saved < args.target_samples:
        print(
            f"Timeout or abort with only {saved}/{args.target_samples} samples. "
            "Not running solve."
        )
        return 1

    print(f"Collected {saved} samples. Running solve...")
    solve_code = run_solve(args.tag_size, samples_dir, intrinsics_path)
    return solve_code


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Automated eye-to-hand calibration (joint_states constraints)"
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
    parser.add_argument("--interval-s", type=float, default=1.0)
    parser.add_argument("--timeout-min", type=float, default=20.0)
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
