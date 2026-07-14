#!/usr/bin/env python3
"""Detect AprilTag and print MTC pick_place object_pose / launch command.

Bridge between eye-to-hand Tag detection (Ubuntu-24.04) and RoArm MTC
pick_place which reads object_pose [x,y,z,roll,pitch,yaw] from
roarm_moveit_mtc_demo/config/roarm_config.yaml (frame: world ≈ base_link).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from common import (
    default_hand_eye_dir,
    default_intrinsics_path,
    get_aruco_dictionary,
    load_intrinsics,
    make_aruco_detector,
    matrix_to_rpy,
)
from move_to_tag import (
    cam_to_base_transform,
    detect_once,
    load_T_base_cam,
    open_camera,
    warmup_camera,
)


def format_pose6(xyzrpy: list[float]) -> str:
    return "[" + ", ".join(f"{v:.4f}" for v in xyzrpy) + "]"


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

    cap = open_camera(args.device, width, height)
    warmup_camera(cap)
    try:
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
    finally:
        cap.release()

    if T_cam_tag is None:
        print("AprilTag not detected.")
        return 1

    T_base_tag = cam_to_base_transform(T_base_cam, T_cam_tag)
    tag_xyz = T_base_tag[:3, 3].copy()
    roll, pitch, yaw = matrix_to_rpy(T_base_tag[:3, :3])

    # MTC createObject(): object_pose.z is the BOTTOM of the cylinder,
    # then it adds +0.5 * height for the collision primitive center.
    obj_h = float(args.object_height)
    if args.tag_is == "top":
        obj_z = float(tag_xyz[2] - obj_h)
    elif args.tag_is == "center":
        obj_z = float(tag_xyz[2] - 0.5 * obj_h)
    else:  # bottom
        obj_z = float(tag_xyz[2])

    # Cylinder pick usually samples yaw; keep orientation identity unless asked.
    if args.use_tag_orientation:
        object_pose = [
            float(tag_xyz[0]),
            float(tag_xyz[1]),
            obj_z,
            float(roll),
            float(pitch),
            float(yaw),
        ]
    else:
        object_pose = [float(tag_xyz[0]), float(tag_xyz[1]), obj_z, 0.0, 0.0, 0.0]

    place_pose = list(args.place_pose)

    print("=== Tag -> MTC pick_place pose ===")
    print(
        f"Tag in base/world (m): "
        f"xyz=({tag_xyz[0]:.4f}, {tag_xyz[1]:.4f}, {tag_xyz[2]:.4f}) "
        f"rpy=({roll:.3f}, {pitch:.3f}, {yaw:.3f})"
    )
    print(f"Assumed tag location on object: {args.tag_is}")
    print(f"object_dimensions height={obj_h:.3f} m, radius={args.object_radius:.3f} m")
    print(f"object_pose [x,y,z,r,p,y]: {format_pose6(object_pose)}")
    print(f"place_pose  [x,y,z,r,p,y]: {format_pose6(place_pose)}")
    print()
    print("How MTC uses this:")
    print("  - pick_place reads object_pose from roarm_config.yaml (frame: world)")
    print("  - For RoArm, world is typically coincident with base_link")
    print("  - Stack: driver + launch_mtc_demo (NOT command_control)")
    print()
    print("--- RoArm_M3: override when launching pick_place ---")
    print(
        "bash $REPO/scripts/roarm/run_mtc.sh pick_place \\\n"
        f"  --object-pose {object_pose[0]:.4f} {object_pose[1]:.4f} {object_pose[2]:.4f} "
        f"{object_pose[3]:.4f} {object_pose[4]:.4f} {object_pose[5]:.4f} \\\n"
        f"  --place-pose {place_pose[0]:.4f} {place_pose[1]:.4f} {place_pose[2]:.4f} "
        f"{place_pose[3]:.4f} {place_pose[4]:.4f} {place_pose[5]:.4f} \\\n"
        f"  --object-dimensions {obj_h:.3f} {args.object_radius:.3f} \\\n"
        "  --spawn-table false"
    )

    out = {
        "object_pose": object_pose,
        "place_pose": place_pose,
        "object_dimensions": [obj_h, float(args.object_radius)],
        "tag_xyz_base": tag_xyz.tolist(),
        "tag_rpy_base": [roll, pitch, yaw],
        "frame": "world",
        "note": "Pass these to run_mtc.sh pick_place overrides",
    }
    if args.output:
        out_path = Path(args.output).expanduser()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(yaml.safe_dump(out, sort_keys=False), encoding="utf-8")
        print(f"\nWrote {out_path}")

    if args.json:
        print(json.dumps(out, indent=2))
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Detect AprilTag and emit MTC pick_place object_pose "
            "(for run_mtc.sh pick_place overrides)."
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
    parser.add_argument("--settle-s", type=float, default=0.3)
    parser.add_argument(
        "--tag-is",
        choices=("top", "center", "bottom"),
        default="top",
        help="Where Tag sits on the collision cylinder (affects object_pose.z)",
    )
    parser.add_argument(
        "--object-height",
        type=float,
        default=0.05,
        help="MTC cylinder height [m] (roarm_config default 0.05)",
    )
    parser.add_argument(
        "--object-radius",
        type=float,
        default=0.01,
        help="MTC cylinder radius [m] (roarm_config default 0.01)",
    )
    parser.add_argument(
        "--place-pose",
        nargs=6,
        type=float,
        default=[0.20, 0.15, 0.08, 0.0, 0.0, 0.0],
        metavar=("X", "Y", "Z", "R", "P", "YAW"),
        help="Where to place after pick [x y z r p y] in world",
    )
    parser.add_argument(
        "--use-tag-orientation",
        action="store_true",
        help="Copy Tag rpy into object_pose (default: zeros; MTC samples yaw)",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Optional YAML path to write poses",
    )
    parser.add_argument("--json", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(run(parse_args()))
