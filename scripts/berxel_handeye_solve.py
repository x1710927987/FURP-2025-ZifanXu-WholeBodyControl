#!/usr/bin/env python3
"""Solve eye-to-hand calibration from collected AprilTag samples."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from berxel_handeye_common import (
    default_hand_eye_dir,
    default_intrinsics_path,
    load_sample_json,
    matrix_to_rpy,
    split_rt,
)


def load_samples(samples_dir: Path) -> list[dict]:
    samples = []
    for json_path in sorted(samples_dir.glob("sample_*.json")):
        data = load_sample_json(json_path)
        data["_path"] = json_path
        samples.append(data)
    return samples


def compute_residuals(T_base_cam: np.ndarray, samples: list[dict]) -> tuple[float, float, list[str]]:
    rot_errors = []
    trans_errors = []
    lines = []

    for sample in samples:
        T_base_gripper = np.array(sample["T_base_gripper"], dtype=np.float64)
        T_cam_tag = np.array(sample["T_cam_tag"], dtype=np.float64)

        left = T_base_cam @ T_cam_tag
        right = T_base_gripper
        delta = np.linalg.inv(right) @ left

        trans_err = np.linalg.norm(delta[:3, 3]) * 1000.0
        rot = delta[:3, :3]
        angle = np.arccos(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0))
        rot_err = np.degrees(angle)

        trans_errors.append(trans_err)
        rot_errors.append(rot_err)
        lines.append(
            f"{sample['_path'].name}: trans={trans_err:.2f} mm, rot={rot_err:.3f} deg"
        )

    return float(np.mean(trans_errors)), float(np.mean(rot_errors)), lines


def run(args) -> int:
    samples_dir = Path(args.samples).expanduser()
    output_path = Path(args.output).expanduser()
    report_path = Path(args.report).expanduser() if args.report else output_path.with_name("hand_eye_report.txt")

    samples = load_samples(samples_dir)
    if len(samples) < 3:
        print(f"Need at least 3 samples, found {len(samples)} in {samples_dir}")
        return 1

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for sample in samples:
        T_base_gripper = np.array(sample["T_base_gripper"], dtype=np.float64)
        T_cam_tag = np.array(sample["T_cam_tag"], dtype=np.float64)
        T_gripper_base = np.linalg.inv(T_base_gripper)
        Rb, tb = split_rt(T_gripper_base)
        Rc, tc = split_rt(T_cam_tag)
        R_gripper2base.append(Rb)
        t_gripper2base.append(tb)
        R_target2cam.append(Rc)
        t_target2cam.append(tc)

    method = getattr(cv2, args.method, cv2.CALIB_HAND_EYE_TSAI)
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
        method=method,
    )

    T_base_cam = np.eye(4, dtype=np.float64)
    T_base_cam[:3, :3] = R_cam2base
    T_base_cam[:3, 3] = t_cam2base.reshape(3)

    mean_trans, mean_rot, residual_lines = compute_residuals(T_base_cam, samples)
    roll, pitch, yaw = matrix_to_rpy(T_base_cam[:3, :3])

    result = {
        "calibration_type": "eye_to_hand",
        "base_frame": "base_link",
        "camera_frame": "camera_optical",
        "effector_frame": "hand_tcp",
        "method": args.method,
        "intrinsics_file": str(Path(args.intrinsics).as_posix()),
        "marker": {
            "dictionary": samples[0].get("dictionary", "DICT_APRILTAG_36h11"),
            "id": samples[0].get("tag_id", 0),
            "size_m": samples[0].get("tag_size_m", args.tag_size),
        },
        "T_base_cam": T_base_cam.tolist(),
        "translation_m": {
            "x": float(T_base_cam[0, 3]),
            "y": float(T_base_cam[1, 3]),
            "z": float(T_base_cam[2, 3]),
        },
        "rpy_rad": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "sample_count": len(samples),
        "residual_translation_mm": mean_trans,
        "residual_rotation_deg": mean_rot,
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(result, f, sort_keys=False, allow_unicode=True)

    summary = [
        f"Samples used: {len(samples)}",
        f"Method: {args.method}",
        f"Mean residual translation: {mean_trans:.3f} mm",
        f"Mean residual rotation: {mean_rot:.3f} deg",
        f"Output: {output_path}",
        "",
        "Per-sample residuals:",
        *residual_lines,
        "",
        "Static TF publisher (verify in RViz):",
        (
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {T_base_cam[0,3]:.6f} --y {T_base_cam[1,3]:.6f} --z {T_base_cam[2,3]:.6f} "
            f"--roll {roll:.6f} --pitch {pitch:.6f} --yaw {yaw:.6f} "
            "--frame-id base_link --child-frame-id camera_optical"
        ),
    ]
    report_path.write_text("\n".join(summary) + "\n", encoding="utf-8")
    print("\n".join(summary[:6]))

    if mean_trans > 5.0 or mean_rot > 1.0:
        print("WARNING: residuals above target (5 mm / 1 deg). Consider recollecting samples.")
        return 2
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description="Solve eye-to-hand calibration")
    parser.add_argument("--samples", default=str(default_hand_eye_dir() / "samples"))
    parser.add_argument("--output", default=str(default_hand_eye_dir() / "extrinsics.yaml"))
    parser.add_argument("--report", default=None)
    parser.add_argument("--intrinsics", default=str(default_intrinsics_path()))
    parser.add_argument("--tag-size", type=float, default=0.006)
    parser.add_argument(
        "--method",
        default="CALIB_HAND_EYE_TSAI",
        help="OpenCV hand-eye method name, e.g. CALIB_HAND_EYE_TSAI",
    )
    return parser.parse_args()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(run(parse_args()))
