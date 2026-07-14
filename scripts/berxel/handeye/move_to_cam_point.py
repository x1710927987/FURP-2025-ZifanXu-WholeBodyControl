#!/usr/bin/env python3
"""Move RoArm TCP to a point expressed in the camera optical frame."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

from common import (
    call_move_joint_cmd,
    check_roarm_pose_only,
    default_hand_eye_dir,
    read_roarm_pose,
)

UNIT_TO_METERS = {"m": 1.0, "cm": 0.01, "mm": 0.001}


def load_T_base_cam(path: Path) -> np.ndarray:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    return np.array(data["T_base_cam"], dtype=np.float64)


def cam_point_to_base(T_base_cam: np.ndarray, p_cam_m: np.ndarray) -> np.ndarray:
    homo = np.array([p_cam_m[0], p_cam_m[1], p_cam_m[2], 1.0], dtype=np.float64)
    return (T_base_cam @ homo)[:3]


def run(args: argparse.Namespace) -> int:
    scale = UNIT_TO_METERS[args.unit]
    p_cam_m = np.array(args.cam_xyz, dtype=np.float64) * scale
    extrinsics = Path(args.extrinsics).expanduser()
    if not extrinsics.is_file():
        print(f"Missing extrinsics: {extrinsics}")
        return 1

    T_base_cam = load_T_base_cam(extrinsics)
    p_base = cam_point_to_base(T_base_cam, p_cam_m)

    print(
        f"Camera point ({args.unit}): "
        f"({args.cam_xyz[0]}, {args.cam_xyz[1]}, {args.cam_xyz[2]})"
    )
    print(f"Camera point (m):     ({p_cam_m[0]:.4f}, {p_cam_m[1]:.4f}, {p_cam_m[2]:.4f})")
    print(f"Base TCP target (m):  ({p_base[0]:.4f}, {p_base[1]:.4f}, {p_base[2]:.4f})")

    if p_base[0] < 0.0 or p_base[2] < 0.0:
        print("WARNING: target has x<0 or z<0 (outside usual calibration workspace).")
    horiz = float(np.linalg.norm(p_base[:2]))
    if horiz > 0.45:
        print(
            f"WARNING: horizontal reach {horiz:.3f} m is large for RoArm-M3; "
            "IK/planning often fails. Prefer a point along camera +Z "
            "(e.g. --cam-xyz 0 0 30)."
        )

    if args.dry_run:
        print("Dry-run only - not moving the arm.")
        return 0

    check_roarm_pose_only(wsl_distro=args.roarm_wsl)
    current = read_roarm_pose(wsl_distro=args.roarm_wsl, timeout_s=60)
    print(
        "Current pose: "
        f"x={current['x']:.4f} y={current['y']:.4f} z={current['z']:.4f} "
        f"rpy=({current['roll']:.3f}, {current['pitch']:.3f}, {current.get('yaw', 0.0):.3f})"
    )

    if args.safe_orientation:
        roll, pitch, yaw = 0.0, 0.0, 0.0
    elif args.keep_orientation:
        roll, pitch, yaw = current["roll"], current["pitch"], current.get("yaw", 0.0)
    else:
        roll, pitch, yaw = args.roll, args.pitch, args.yaw

    print(
        "Commanding move_joint_cmd: "
        f"xyz=({p_base[0]:.4f}, {p_base[1]:.4f}, {p_base[2]:.4f}) "
        f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f})"
    )
    ok, message = call_move_joint_cmd(
        float(p_base[0]),
        float(p_base[1]),
        float(p_base[2]),
        float(roll),
        float(pitch),
        float(yaw),
        wsl_distro=args.roarm_wsl,
    )
    if not ok:
        print(f"Move failed: {message}")
        return 1

    final = read_roarm_pose(wsl_distro=args.roarm_wsl, timeout_s=60)
    err = np.linalg.norm(
        np.array([final["x"], final["y"], final["z"]]) - p_base
    )
    print(
        "Done. Reached: "
        f"x={final['x']:.4f} y={final['y']:.4f} z={final['z']:.4f} "
        f"(position error {err * 1000:.1f} mm vs target)"
    )
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move TCP to a camera-frame point using hand-eye extrinsics. "
            "Camera optical frame: X right, Y down, Z forward."
        )
    )
    parser.add_argument(
        "--cam-xyz",
        nargs=3,
        type=float,
        required=True,
        metavar=("X", "Y", "Z"),
        help="Target point in camera optical frame",
    )
    parser.add_argument(
        "--unit",
        choices=sorted(UNIT_TO_METERS),
        default="cm",
        help="Unit for --cam-xyz (default: cm). So 10 0 0 means 10 cm.",
    )
    parser.add_argument(
        "--extrinsics",
        default=str(default_hand_eye_dir() / "extrinsics.yaml"),
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
        help="Use roll=pitch=yaw=0 instead of current (often extreme) orientation",
    )
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(run(parse_args()))
