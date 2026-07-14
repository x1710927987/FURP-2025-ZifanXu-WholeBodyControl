#!/usr/bin/env python3
"""Waypoint table and constraint checks for automated hand-eye calibration."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

BASE_JOINT = "base_link_to_link1"
ROLL_JOINT = "link4_to_link5"
GRIPPER_JOINT = "link5_to_gripper_link"

BASE_LIMIT_RAD = math.radians(45.0)
ROLL_LIMIT_RAD = math.radians(45.0)
GRIPPER_TOLERANCE_RAD = 0.03

TCP_X_MIN = 0.0
TCP_Z_MIN = 0.0

WAYPOINT_MIN_TRANSLATION_M = 0.015
WAYPOINT_MIN_ROTATION_RAD = math.radians(7.0)


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float = 0.0
    label: str = ""


# Round-2: larger motion than v1, but keep Tag in left-side camera FOV.
# Empirically |y|<=0.12 and modest pitch keep AprilTag visible.
RAW_WAYPOINTS: list[Waypoint] = [
    Waypoint(0.22, 0.00, 0.10, 0.00, 0.05, label="v2_00"),
    Waypoint(0.22, 0.10, 0.10, 0.55, 0.12, label="v2_01"),
    Waypoint(0.22, 0.08, 0.14, -0.55, 0.20, label="v2_02"),
    Waypoint(0.24, 0.00, 0.08, 0.70, 0.18, label="v2_03"),
    Waypoint(0.24, 0.12, 0.12, -0.70, 0.08, label="v2_04"),
    Waypoint(0.25, 0.04, 0.16, 0.30, 0.32, label="v2_05"),
    Waypoint(0.25, 0.11, 0.07, -0.30, -0.05, label="v2_06"),
    Waypoint(0.26, 0.00, 0.13, 0.45, 0.25, label="v2_07"),
    Waypoint(0.26, 0.10, 0.09, -0.45, 0.28, label="v2_08"),
    Waypoint(0.27, 0.06, 0.15, 0.65, 0.00, label="v2_09"),
    Waypoint(0.27, 0.12, 0.11, -0.65, 0.15, label="v2_10"),
    Waypoint(0.28, 0.00, 0.07, 0.20, 0.30, label="v2_11"),
    Waypoint(0.28, 0.09, 0.16, -0.20, 0.10, label="v2_12"),
    Waypoint(0.29, 0.03, 0.10, 0.70, -0.08, label="v2_13"),
    Waypoint(0.29, 0.11, 0.14, -0.70, 0.22, label="v2_14"),
    Waypoint(0.30, 0.00, 0.12, 0.00, 0.22, label="v2_15"),
    Waypoint(0.30, 0.10, 0.08, 0.50, 0.05, label="v2_16"),
    Waypoint(0.31, 0.05, 0.15, -0.50, 0.28, label="v2_17"),
    Waypoint(0.31, 0.12, 0.10, 0.35, 0.18, label="v2_18"),
    Waypoint(0.32, 0.00, 0.14, -0.35, 0.12, label="v2_19"),
    Waypoint(0.32, 0.08, 0.09, 0.60, 0.30, label="v2_20"),
    Waypoint(0.33, 0.04, 0.11, -0.60, 0.00, label="v2_21"),
    Waypoint(0.33, 0.10, 0.13, 0.15, 0.25, label="v2_22"),
    Waypoint(0.34, 0.00, 0.10, -0.15, 0.15, label="v2_23"),
    Waypoint(0.34, 0.07, 0.12, 0.70, 0.08, label="v2_24"),
    Waypoint(0.23, 0.06, 0.11, -0.40, 0.28, label="v2_25"),
    Waypoint(0.24, 0.09, 0.15, 0.40, -0.05, label="v2_26"),
    Waypoint(0.28, 0.05, 0.18, 0.55, 0.15, label="v2_27"),
    Waypoint(0.30, 0.06, 0.06, -0.55, 0.20, label="v2_28"),
    Waypoint(0.32, 0.03, 0.16, 0.25, 0.32, label="v2_29"),
    Waypoint(0.26, 0.07, 0.12, -0.70, -0.08, label="v2_30"),
    Waypoint(0.29, 0.08, 0.13, 0.00, 0.00, label="v2_31"),
]


def estimate_base_joint_rad(x: float, y: float) -> float:
    if x <= 0.0:
        return math.pi
    return math.atan2(y, x)


def precheck_waypoint(wp: Waypoint) -> tuple[bool, str]:
    if wp.x < TCP_X_MIN:
        return False, f"x={wp.x:.3f} < {TCP_X_MIN}"
    if wp.z < TCP_Z_MIN:
        return False, f"z={wp.z:.3f} < {TCP_Z_MIN}"
    if abs(wp.roll) > ROLL_LIMIT_RAD + 1e-6:
        return False, f"|roll|={math.degrees(abs(wp.roll)):.1f}° > 45°"
    base_est = estimate_base_joint_rad(wp.x, wp.y)
    if abs(base_est) > BASE_LIMIT_RAD + 1e-6:
        return False, f"|base_est|={math.degrees(abs(base_est)):.1f}° > 45°"
    return True, "ok"


def waypoint_pose_dict(wp: Waypoint) -> dict[str, float]:
    return {
        "x": wp.x,
        "y": wp.y,
        "z": wp.z,
        "roll": wp.roll,
        "pitch": wp.pitch,
        "yaw": wp.yaw,
    }


def filter_waypoints(waypoints: list[Waypoint] | None = None) -> list[Waypoint]:
    source = RAW_WAYPOINTS if waypoints is None else waypoints
    accepted: list[Waypoint] = []
    accepted_poses: list[dict[str, float]] = []
    for wp in source:
        ok, reason = precheck_waypoint(wp)
        if not ok:
            print(f"Skip waypoint {wp.label or '(unnamed)'}: {reason}")
            continue
        pose = waypoint_pose_dict(wp)
        if not pose_is_diverse(
            pose,
            accepted_poses,
            min_translation_m=WAYPOINT_MIN_TRANSLATION_M,
            min_rotation_rad=WAYPOINT_MIN_ROTATION_RAD,
        ):
            print(f"Skip waypoint {wp.label or '(unnamed)'}: too close to an earlier waypoint")
            continue
        accepted.append(wp)
        accepted_poses.append(pose)
    if not accepted:
        raise RuntimeError("No valid waypoints after constraint filtering.")
    if len(accepted) < 20:
        print(
            f"WARNING: only {len(accepted)} diverse waypoints; "
            "hand-eye may struggle to reach 25 unique samples."
        )
    return accepted


def validate_runtime_constraints(
    pose: dict[str, float],
    joints: dict[str, float] | None,
    gripper_home_rad: float | None,
) -> tuple[bool, str]:
    if pose["x"] < TCP_X_MIN:
        return False, f"TCP x={pose['x']:.4f} < {TCP_X_MIN}"
    if pose["z"] < TCP_Z_MIN:
        return False, f"TCP z={pose['z']:.4f} < {TCP_Z_MIN}"

    if joints is None:
        if abs(pose["roll"]) > ROLL_LIMIT_RAD:
            return False, f"pose roll {math.degrees(pose['roll']):.1f}° exceeds ±45°"
        base_est = estimate_base_joint_rad(pose["x"], pose["y"])
        if abs(base_est) > BASE_LIMIT_RAD:
            return False, f"estimated base {math.degrees(base_est):.1f}° exceeds ±45°"
        return True, "ok"

    base = joints.get(BASE_JOINT)
    roll = joints.get(ROLL_JOINT)
    gripper = joints.get(GRIPPER_JOINT)
    if base is None or roll is None or gripper is None:
        return False, "missing required joint_states entries"

    if abs(base) > BASE_LIMIT_RAD:
        return False, f"base joint {math.degrees(base):.1f}° exceeds ±45°"
    if abs(roll) > ROLL_LIMIT_RAD:
        return False, f"roll joint {math.degrees(roll):.1f}° exceeds ±45°"
    if gripper_home_rad is not None and abs(gripper - gripper_home_rad) > GRIPPER_TOLERANCE_RAD:
        return False, (
            f"gripper moved {math.degrees(abs(gripper - gripper_home_rad)):.1f}° "
            f"(limit {math.degrees(GRIPPER_TOLERANCE_RAD):.1f}°)"
        )
    return True, "ok"


def pose_is_diverse(
    pose: dict[str, float],
    saved_poses: list[dict[str, float]],
    min_translation_m: float = 0.005,
    min_rotation_rad: float = math.radians(2.0),
) -> bool:
    for previous in saved_poses:
        delta = np.array(
            [
                pose["x"] - previous["x"],
                pose["y"] - previous["y"],
                pose["z"] - previous["z"],
            ],
            dtype=np.float64,
        )
        trans = float(np.linalg.norm(delta))
        rot = max(
            abs(pose["roll"] - previous["roll"]),
            abs(pose["pitch"] - previous["pitch"]),
            abs(pose.get("yaw", 0.0) - previous.get("yaw", 0.0)),
        )
        if trans < min_translation_m and rot < min_rotation_rad:
            return False
    return True


def pose_translation_m(pose_a: dict[str, float], pose_b: dict[str, float]) -> float:
    delta = np.array(
        [
            pose_a["x"] - pose_b["x"],
            pose_a["y"] - pose_b["y"],
            pose_a["z"] - pose_b["z"],
        ],
        dtype=np.float64,
    )
    return float(np.linalg.norm(delta))
