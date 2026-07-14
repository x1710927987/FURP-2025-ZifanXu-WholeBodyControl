#!/usr/bin/env python3
"""Solve eye-to-hand calibration from collected AprilTag samples."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from common import (
    default_hand_eye_dir,
    default_intrinsics_path,
    load_sample_json,
    matrix_to_rpy,
    split_rt,
)

HAND_EYE_METHODS = [
    "CALIB_HAND_EYE_TSAI",
    "CALIB_HAND_EYE_PARK",
    "CALIB_HAND_EYE_HORAUD",
    "CALIB_HAND_EYE_DANIILIDIS",
]


def load_samples(samples_dir: Path) -> list[dict]:
    samples = []
    for json_path in sorted(samples_dir.glob("sample_*.json")):
        data = load_sample_json(json_path)
        data["_path"] = json_path
        samples.append(data)
    return samples


def rotation_angle_deg(rotation: np.ndarray) -> float:
    angle = np.arccos(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(angle))


def average_transforms(transforms: list[np.ndarray]) -> np.ndarray:
    """Chordal L2 mean of SE(3) poses (translation average + SVD rotation mean)."""
    translations = np.array([T[:3, 3] for T in transforms], dtype=np.float64)
    t_mean = translations.mean(axis=0)
    M = np.zeros((3, 3), dtype=np.float64)
    for T in transforms:
        M += T[:3, :3]
    u, _, vt = np.linalg.svd(M)
    R = u @ vt
    if np.linalg.det(R) < 0:
        u[:, -1] *= -1.0
        R = u @ vt
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = R
    out[:3, 3] = t_mean
    return out


def gripper_tag_estimates(T_base_cam: np.ndarray, samples: list[dict]) -> list[np.ndarray]:
    """Each estimate of T_gripper_tag = inv(T_base_gripper) @ T_base_cam @ T_cam_tag."""
    estimates = []
    for sample in samples:
        T_base_gripper = np.array(sample["T_base_gripper"], dtype=np.float64)
        T_cam_tag = np.array(sample["T_cam_tag"], dtype=np.float64)
        estimates.append(np.linalg.inv(T_base_gripper) @ T_base_cam @ T_cam_tag)
    return estimates


def compute_offset_residuals(
    T_base_cam: np.ndarray, samples: list[dict]
) -> tuple[float, float, np.ndarray, list[str]]:
    """
    Residual for eye-to-hand when Tag ≠ TCP.

    inv(T_base_gripper) @ T_base_cam @ T_cam_tag should be a constant T_gripper_tag.
    """
    estimates = gripper_tag_estimates(T_base_cam, samples)
    T_gripper_tag = average_transforms(estimates)

    trans_errors = []
    rot_errors = []
    lines = []
    for sample, estimate in zip(samples, estimates):
        delta = np.linalg.inv(T_gripper_tag) @ estimate
        trans_err = float(np.linalg.norm(delta[:3, 3]) * 1000.0)
        rot_err = rotation_angle_deg(delta[:3, :3])
        trans_errors.append(trans_err)
        rot_errors.append(rot_err)
        lines.append(
            f"{sample['_path'].name}: trans={trans_err:.2f} mm, rot={rot_err:.3f} deg"
        )
    return (
        float(np.mean(trans_errors)),
        float(np.mean(rot_errors)),
        T_gripper_tag,
        lines,
    )


def refine_base_cam(T_base_cam: np.ndarray, samples: list[dict], rounds: int = 3) -> np.ndarray:
    """Alternate averaging T_gripper_tag and T_base_cam = G @ T_gripper_tag @ inv(C)."""
    current = T_base_cam.copy()
    for _ in range(rounds):
        T_gripper_tag = average_transforms(gripper_tag_estimates(current, samples))
        rebuilt = []
        for sample in samples:
            T_base_gripper = np.array(sample["T_base_gripper"], dtype=np.float64)
            T_cam_tag = np.array(sample["T_cam_tag"], dtype=np.float64)
            rebuilt.append(T_base_gripper @ T_gripper_tag @ np.linalg.inv(T_cam_tag))
        current = average_transforms(rebuilt)
    return current


def solve_hand_eye(samples: list[dict], method_name: str) -> np.ndarray:
    R_base2gripper = []
    t_base2gripper = []
    R_target2cam = []
    t_target2cam = []

    for sample in samples:
        T_base_gripper = np.array(sample["T_base_gripper"], dtype=np.float64)
        T_cam_tag = np.array(sample["T_cam_tag"], dtype=np.float64)
        # Eye-to-hand: pass base2gripper; OpenCV returns cam→base interpreted as T_base_cam.
        Rb, tb = split_rt(np.linalg.inv(T_base_gripper))
        Rc, tc = split_rt(T_cam_tag)
        R_base2gripper.append(Rb)
        t_base2gripper.append(tb)
        R_target2cam.append(Rc)
        t_target2cam.append(tc)

    method = getattr(cv2, method_name)
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_base2gripper,
        t_base2gripper,
        R_target2cam,
        t_target2cam,
        method=method,
    )
    T_base_cam = np.eye(4, dtype=np.float64)
    T_base_cam[:3, :3] = R_cam2base
    T_base_cam[:3, 3] = t_cam2base.reshape(3)
    return T_base_cam


def run(args) -> int:
    samples_dir = Path(args.samples).expanduser()
    output_path = Path(args.output).expanduser()
    report_path = (
        Path(args.report).expanduser()
        if args.report
        else output_path.with_name("hand_eye_report.txt")
    )

    samples = load_samples(samples_dir)
    if len(samples) < 3:
        print(f"Need at least 3 samples, found {len(samples)} in {samples_dir}")
        return 1

    method_names = [args.method] if args.method != "auto" else HAND_EYE_METHODS
    candidates: list[tuple[str, np.ndarray, float, float, np.ndarray, list[str]]] = []

    for method_name in method_names:
        try:
            T_raw = solve_hand_eye(samples, method_name)
        except cv2.error as exc:
            print(f"Skip method {method_name}: {exc}")
            continue
        T_ref = refine_base_cam(T_raw, samples)
        mean_trans, mean_rot, T_gripper_tag, residual_lines = compute_offset_residuals(
            T_ref, samples
        )
        candidates.append(
            (method_name, T_ref, mean_trans, mean_rot, T_gripper_tag, residual_lines)
        )
        print(
            f"Method {method_name}: offset-residual "
            f"{mean_trans:.3f} mm / {mean_rot:.3f} deg"
        )

    if not candidates:
        print("No hand-eye method succeeded.")
        return 1

    best = min(candidates, key=lambda item: (item[2] + 10.0 * item[3], item[2], item[3]))
    method_name, T_base_cam, mean_trans, mean_rot, T_gripper_tag, residual_lines = best
    roll, pitch, yaw = matrix_to_rpy(T_base_cam[:3, :3])
    gt_roll, gt_pitch, gt_yaw = matrix_to_rpy(T_gripper_tag[:3, :3])

    result = {
        "calibration_type": "eye_to_hand",
        "base_frame": "base_link",
        "camera_frame": "camera_optical",
        "effector_frame": "hand_tcp",
        "marker_frame": "apriltag",
        "method": method_name,
        "intrinsics_file": str(Path(args.intrinsics).as_posix()),
        "marker": {
            "dictionary": samples[0].get("dictionary", "DICT_APRILTAG_36h11"),
            "id": samples[0].get("tag_id", 0),
            "size_m": samples[0].get("tag_size_m", args.tag_size),
        },
        "T_base_cam": T_base_cam.tolist(),
        "T_gripper_tag": T_gripper_tag.tolist(),
        "translation_m": {
            "x": float(T_base_cam[0, 3]),
            "y": float(T_base_cam[1, 3]),
            "z": float(T_base_cam[2, 3]),
        },
        "rpy_rad": {"roll": roll, "pitch": pitch, "yaw": yaw},
        "gripper_tag_translation_m": {
            "x": float(T_gripper_tag[0, 3]),
            "y": float(T_gripper_tag[1, 3]),
            "z": float(T_gripper_tag[2, 3]),
        },
        "gripper_tag_rpy_rad": {"roll": gt_roll, "pitch": gt_pitch, "yaw": gt_yaw},
        "sample_count": len(samples),
        "residual_translation_mm": mean_trans,
        "residual_rotation_deg": mean_rot,
        "residual_definition": (
            "spread of inv(T_base_gripper)@T_base_cam@T_cam_tag around mean T_gripper_tag "
            "(Tag need not coincide with TCP)"
        ),
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(result, f, sort_keys=False, allow_unicode=True)

    summary = [
        f"Samples used: {len(samples)}",
        f"Method: {method_name}",
        f"Mean offset residual translation: {mean_trans:.3f} mm",
        f"Mean offset residual rotation: {mean_rot:.3f} deg",
        (
            "Estimated Tag pose in gripper/TCP frame [m]: "
            f"xyz=({T_gripper_tag[0, 3]:.4f}, {T_gripper_tag[1, 3]:.4f}, {T_gripper_tag[2, 3]:.4f})"
        ),
        f"Output: {output_path}",
        "",
        "Note: residual measures consistency of rigid Tag-TCP offset,",
        "NOT T_base_cam@T_cam_tag ~= T_base_gripper (that old check is wrong when Tag != TCP).",
        "",
        "Per-sample offset residuals:",
        *residual_lines,
        "",
        "Static TF publisher (verify in RViz):",
        (
            "ros2 run tf2_ros static_transform_publisher "
            f"--x {T_base_cam[0, 3]:.6f} --y {T_base_cam[1, 3]:.6f} --z {T_base_cam[2, 3]:.6f} "
            f"--roll {roll:.6f} --pitch {pitch:.6f} --yaw {yaw:.6f} "
            "--frame-id base_link --child-frame-id camera_optical"
        ),
    ]
    report_path.write_text("\n".join(summary) + "\n", encoding="utf-8")
    print("\n".join(summary[:8]))

    if mean_trans > 5.0 or mean_rot > 1.0:
        print(
            "WARNING: residuals above target (5 mm / 1 deg). "
            "Usable for rough setup if <~20 mm / ~10 deg; recollect/refine for higher accuracy."
        )
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
        default="auto",
        help="OpenCV method name, or 'auto' to try several and pick best",
    )
    return parser.parse_args()


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.exit(run(parse_args()))
