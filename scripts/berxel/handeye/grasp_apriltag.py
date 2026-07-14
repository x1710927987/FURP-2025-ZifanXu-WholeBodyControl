#!/usr/bin/env python3
"""Two-stage AprilTag approach with gripper close and lift (eye-to-hand)."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from common import (
    call_gripper_cmd,
    check_roarm_pose_only,
    default_hand_eye_dir,
    default_intrinsics_path,
    detect_apriltag,
    estimate_tag_pose,
    get_aruco_dictionary,
    load_intrinsics,
    make_aruco_detector,
    read_roarm_pose,
)
from move_to_tag import (
    apply_offset,
    compute_target_from_detection,
    detect_once,
    load_T_base_cam,
    move_tcp_to,
    open_camera,
    resolve_orientation,
    warmup_camera,
)


def command_gripper(
    position_rad: float,
    label: str,
    wsl_distro: str,
    dry_run: bool,
    settle_s: float,
) -> int:
    print(f"--- Gripper: {label} ({position_rad:.3f} rad) ---")
    if dry_run:
        print("Dry-run: skip gripper command.")
        return 0
    ok, message = call_gripper_cmd(position_rad, wsl_distro=wsl_distro)
    if not ok:
        print(f"Gripper command failed: {message}")
        return 1
    if settle_s > 0:
        print(f"Waiting {settle_s:.1f}s for gripper motion...")
        time.sleep(settle_s)
    return 0


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
    hover_offset = np.array(args.hover_offset, dtype=np.float64)
    grasp_offset = np.array(args.grasp_offset, dtype=np.float64)
    lift_offset = np.array(args.lift_offset, dtype=np.float64)

    cap = open_camera(args.device, width, height)
    warmup_camera(cap)

    print("=== Grasp AprilTag demo (eye-to-hand) ===")
    print(f"Tag: id={args.tag_id}, size={args.tag_size} m")
    print(f"Hover offset ({args.offset_frame}): {hover_offset.tolist()} m")
    print(f"Grasp offset ({args.offset_frame}): {grasp_offset.tolist()} m")
    print(f"Lift offset ({args.offset_frame}): {lift_offset.tolist()} m")
    if args.no_gripper:
        print("Gripper: disabled (--no-gripper)")
    else:
        print(
            f"Gripper: open={args.gripper_open:.3f} rad, "
            f"close={args.gripper_close:.3f} rad"
        )
    print(
        "Controls: G=run full sequence, Q=quit"
        if not args.once
        else "Mode: --once"
    )

    window = "Grasp AprilTag (G=go, Q=quit)"

    def execute_grasp() -> int:
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
            print("AprilTag not detected.")
            return 1

        tag_c = T_cam_tag[:3, 3]
        print(
            f"Detected tag in camera (m): "
            f"t=({tag_c[0]:.4f}, {tag_c[1]:.4f}, {tag_c[2]:.4f})"
        )
        T_base_tag, p_hover = compute_target_from_detection(
            T_base_cam, T_cam_tag, hover_offset, args.offset_frame
        )
        _, p_grasp = compute_target_from_detection(
            T_base_cam, T_cam_tag, grasp_offset, args.offset_frame
        )
        p_lift = apply_offset(T_base_tag, lift_offset, args.offset_frame)
        tag_b = T_base_tag[:3, 3]
        print(
            f"Tag in base (m): ({tag_b[0]:.4f}, {tag_b[1]:.4f}, {tag_b[2]:.4f})"
        )

        if not args.dry_run:
            check_roarm_pose_only(wsl_distro=args.roarm_wsl)
            current = read_roarm_pose(wsl_distro=args.roarm_wsl, timeout_s=60)
        else:
            current = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        roll, pitch, yaw = resolve_orientation(args, current)

        if not args.no_gripper and not args.skip_gripper_open:
            rc = command_gripper(
                args.gripper_open,
                "open (pre-approach)",
                args.roarm_wsl,
                args.dry_run,
                args.gripper_settle_s,
            )
            if rc != 0:
                return rc

        print("--- Stage 1: hover ---")
        rc = move_tcp_to(p_hover, roll, pitch, yaw, args.roarm_wsl, args.dry_run)
        if rc != 0:
            return rc

        if args.pause_s > 0:
            print(f"Pause {args.pause_s:.1f}s before descend...")
            time.sleep(args.pause_s)

        if args.redetect:
            time.sleep(args.settle_s)
            T_cam_tag2, _r2, _t2, _f2 = detect_once(
                cap,
                detector,
                api_mode,
                dictionary,
                args.tag_id,
                args.tag_size,
                camera_matrix,
                dist_coeffs,
            )
            if T_cam_tag2 is not None:
                T_cam_tag = T_cam_tag2
                T_base_tag, p_hover = compute_target_from_detection(
                    T_base_cam, T_cam_tag, hover_offset, args.offset_frame
                )
                _, p_grasp = compute_target_from_detection(
                    T_base_cam, T_cam_tag, grasp_offset, args.offset_frame
                )
                p_lift = apply_offset(T_base_tag, lift_offset, args.offset_frame)
                print("Re-detected tag before descend.")
            else:
                print("WARNING: re-detect failed; using first detection for descend.")

        print("--- Stage 2: descend (pre-grasp) ---")
        rc = move_tcp_to(p_grasp, roll, pitch, yaw, args.roarm_wsl, args.dry_run)
        if rc != 0:
            return rc

        if not args.no_gripper:
            rc = command_gripper(
                args.gripper_close,
                "close (grasp)",
                args.roarm_wsl,
                args.dry_run,
                args.gripper_settle_s,
            )
            if rc != 0:
                return rc

        if not args.skip_lift:
            print("--- Stage 3: lift (retreat) ---")
            rc = move_tcp_to(p_lift, roll, pitch, yaw, args.roarm_wsl, args.dry_run)
            if rc != 0:
                return rc

        print("Grasp sequence complete.")
        return 0

    try:
        if args.once:
            return execute_grasp()

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
            cv2.putText(
                display,
                "hover -> descend -> close -> lift",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
            )
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
            if not found:
                print("Skip: tag not detected")
                continue
            rc = execute_grasp()
            if rc != 0:
                return rc
            if args.once_after_go:
                return 0
    finally:
        cap.release()
        cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "AprilTag grasp demo: hover, descend, close gripper, lift. "
            "Uses eye-to-hand T_base_cam."
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
    parser.add_argument("--tag-size", type=float, default=0.06)
    parser.add_argument(
        "--hover-offset",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.08],
        metavar=("DX", "DY", "DZ"),
        help="Stage-1 TCP offset from tag (m). Default: 8 cm above in offset-frame",
    )
    parser.add_argument(
        "--grasp-offset",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.025],
        metavar=("DX", "DY", "DZ"),
        help="Stage-2 TCP offset from tag (m). Default: 2.5 cm above in offset-frame",
    )
    parser.add_argument(
        "--lift-offset",
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.10],
        metavar=("DX", "DY", "DZ"),
        help="Stage-3 lift TCP offset from tag (m). Default: 10 cm above in offset-frame",
    )
    parser.add_argument(
        "--offset-frame",
        choices=("base", "tag"),
        default="base",
        help="Interpret hover/grasp/lift offsets in base or tag frame (default: base)",
    )
    parser.add_argument("--roarm-wsl", default="RoArm_M3")
    parser.add_argument("--keep-orientation", action="store_true", default=True)
    parser.add_argument(
        "--set-orientation",
        action="store_false",
        dest="keep_orientation",
    )
    parser.add_argument("--safe-orientation", action="store_true")
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--once-after-go", action="store_true")
    parser.add_argument(
        "--redetect",
        action="store_true",
        help="Re-detect tag after hover before descending",
    )
    parser.add_argument("--settle-s", type=float, default=0.3)
    parser.add_argument(
        "--pause-s",
        type=float,
        default=0.5,
        help="Pause between hover and descend (seconds)",
    )
    parser.add_argument(
        "--gripper-open",
        type=float,
        default=0.0,
        help="Gripper open target (rad). RoArm range ~0.0-1.5",
    )
    parser.add_argument(
        "--gripper-close",
        type=float,
        default=1.2,
        help="Gripper close target (rad). RoArm range ~0.0-1.5",
    )
    parser.add_argument(
        "--gripper-settle-s",
        type=float,
        default=1.5,
        help="Wait after each gripper command (seconds)",
    )
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="Skip all gripper commands (TCP motion only)",
    )
    parser.add_argument(
        "--skip-gripper-open",
        action="store_true",
        help="Do not open gripper before approach (assume already open)",
    )
    parser.add_argument(
        "--skip-lift",
        action="store_true",
        help="Skip retreat/lift after closing gripper",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(run(parse_args()))
