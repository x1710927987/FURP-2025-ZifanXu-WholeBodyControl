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

from berxel_handeye_common import rpy_to_matrix, split_rt


RNG = np.random.default_rng(42)


def random_pose() -> np.ndarray:
    t = RNG.uniform(-0.2, 0.2, size=3)
    rpy = RNG.uniform(-0.4, 0.4, size=3)
    T = np.eye(4)
    T[:3, :3] = rpy_to_matrix(*rpy)
    T[:3, 3] = t
    return T


def main() -> int:
    import cv2

    T_base_cam_true = np.eye(4)
    T_base_cam_true[:3, :3] = rpy_to_matrix(0.1, -0.2, 0.3)
    T_base_cam_true[:3, 3] = [0.4, -0.1, 0.55]

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        for i in range(18):
            T_base_gripper = random_pose()
            T_cam_tag = np.linalg.inv(T_base_cam_true) @ T_base_gripper
            payload = {
                "index": i + 1,
                "T_base_gripper": T_base_gripper.tolist(),
                "T_cam_tag": T_cam_tag.tolist(),
                "dictionary": "DICT_APRILTAG_36h11",
                "tag_id": 0,
                "tag_size_m": 0.006,
            }
            (tmp_path / f"sample_{i+1:03d}.json").write_text(json.dumps(payload), encoding="utf-8")
            T_gripper_base = np.linalg.inv(T_base_gripper)
            Rb, tb = split_rt(T_gripper_base)
            Rc, tc = split_rt(T_cam_tag)
            R_g2b.append(Rb)
            t_g2b.append(tb)
            R_t2c.append(Rc)
            t_t2c.append(tc)

        R_cam2base, t_cam2base = cv2.calibrateHandEye(
            R_g2b, t_g2b, R_t2c, t_t2c, method=cv2.CALIB_HAND_EYE_TSAI
        )
        T_est = np.eye(4)
        T_est[:3, :3] = R_cam2base
        T_est[:3, 3] = t_cam2base.reshape(3)
        err = np.linalg.norm(T_est - T_base_cam_true)
        print(f"synthetic solve error (Frobenius): {err:.6f}")
        return 0 if err < 0.05 else 1


if __name__ == "__main__":
    raise SystemExit(main())
