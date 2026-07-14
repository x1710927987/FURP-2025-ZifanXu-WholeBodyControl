#!/usr/bin/env python3
"""Generate synthetic hand-eye samples and verify solve pipeline."""

from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from common import rpy_to_matrix
from solve import compute_offset_residuals, refine_base_cam, solve_hand_eye


RNG = np.random.default_rng(42)


def random_pose(trans_scale: float = 0.2, rot_scale: float = 0.4) -> np.ndarray:
    t = RNG.uniform(-trans_scale, trans_scale, size=3)
    rpy = RNG.uniform(-rot_scale, rot_scale, size=3)
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = t
    return T


def main() -> int:
    T_base_cam_true = np.eye(4)
    T_base_cam_true[:3, :3] = rpy_to_matrix(0.1, -0.2, 0.3)
    T_base_cam_true[:3, 3] = [0.4, -0.1, 0.55]

    T_gripper_tag_true = np.eye(4)
    T_gripper_tag_true[:3, :3] = rpy_to_matrix(0.05, -0.1, 0.2)
    T_gripper_tag_true[:3, 3] = [0.02, 0.03, 0.06]

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        samples = []
        for i in range(20):
            T_base_gripper = random_pose()
            T_cam_tag = np.linalg.inv(T_base_cam_true) @ T_base_gripper @ T_gripper_tag_true
            payload = {
                "index": i + 1,
                "T_base_gripper": T_base_gripper.tolist(),
                "T_cam_tag": T_cam_tag.tolist(),
                "dictionary": "DICT_APRILTAG_36h11",
                "tag_id": 0,
                "tag_size_m": 0.06,
            }
            path = tmp_path / f"sample_{i + 1:03d}.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            payload["_path"] = path
            samples.append(payload)

        T_raw = solve_hand_eye(samples, "CALIB_HAND_EYE_TSAI")
        T_est = refine_base_cam(T_raw, samples)
        mean_trans, mean_rot, T_gt_est, _ = compute_offset_residuals(T_est, samples)
        cam_err = np.linalg.norm(T_est[:3, 3] - T_base_cam_true[:3, 3])
        tag_err = np.linalg.norm(T_gt_est[:3, 3] - T_gripper_tag_true[:3, 3])
        print(
            f"synthetic: cam_t_err={cam_err:.4f} m, tag_t_err={tag_err:.4f} m, "
            f"resid={mean_trans:.3f} mm / {mean_rot:.3f} deg"
        )
        ok = cam_err < 0.05 and tag_err < 0.05 and mean_trans < 1.0 and mean_rot < 1.0
        return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
