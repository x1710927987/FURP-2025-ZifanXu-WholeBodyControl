#!/usr/bin/env python3
"""Unit tests for hand-eye waypoint constraint logic."""

from __future__ import annotations

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from common import parse_joint_states_message
from waypoints import (
    BASE_LIMIT_RAD,
    ROLL_LIMIT_RAD,
    Waypoint,
    filter_waypoints,
    pose_is_diverse,
    precheck_waypoint,
    validate_runtime_constraints,
)


def test_parse_joint_states() -> None:
    sample = """
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name:
- base_link_to_link1
- link1_to_link2
- link2_to_link3
- link3_to_link4
- link4_to_link5
- link5_to_gripper_link
position:
- 0.1
- 0.2
- 2.6
- -1.0
- 0.3
- 3.14
"""
    joints = parse_joint_states_message(sample)
    assert joints["base_link_to_link1"] == 0.1
    assert joints["link4_to_link5"] == 0.3
    assert joints["link5_to_gripper_link"] == 3.14


def test_precheck_rejects_invalid() -> None:
    bad_x = Waypoint(-0.1, 0.0, 0.1, 0.0, 0.0, label="bad_x")
    ok, reason = precheck_waypoint(bad_x)
    assert not ok and "x=" in reason

    bad_roll = Waypoint(0.25, 0.05, 0.1, 0.9, 0.0, label="bad_roll")
    ok, reason = precheck_waypoint(bad_roll)
    assert not ok and "roll" in reason

    bad_base = Waypoint(0.2, 0.3, 0.1, 0.0, 0.0, label="bad_base")
    ok, reason = precheck_waypoint(bad_base)
    assert not ok and "base_est" in reason


def test_precheck_accepts_defaults() -> None:
    accepted = filter_waypoints()
    assert len(accepted) >= 25, f"expected >=25 diverse waypoints, got {len(accepted)}"


def test_runtime_constraints() -> None:
    pose = {"x": 0.27, "y": 0.05, "z": 0.1, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    joints = {
        "base_link_to_link1": 0.2,
        "link4_to_link5": 0.1,
        "link5_to_gripper_link": 3.14,
    }
    ok, _ = validate_runtime_constraints(pose, joints, gripper_home_rad=3.14)
    assert ok

    joints["link4_to_link5"] = ROLL_LIMIT_RAD + 0.1
    ok, reason = validate_runtime_constraints(pose, joints, gripper_home_rad=3.14)
    assert not ok and "roll" in reason

    joints["link4_to_link5"] = 0.1
    joints["base_link_to_link1"] = BASE_LIMIT_RAD + 0.1
    ok, reason = validate_runtime_constraints(pose, joints, gripper_home_rad=3.14)
    assert not ok and "base" in reason

    ok, _ = validate_runtime_constraints(pose, None, gripper_home_rad=None)
    assert ok


def test_pose_diversity() -> None:
    a = {"x": 0.27, "y": 0.0, "z": 0.1, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    b = {"x": 0.27, "y": 0.0, "z": 0.1, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    c = {"x": 0.30, "y": 0.0, "z": 0.1, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    assert not pose_is_diverse(b, [a])
    assert pose_is_diverse(c, [a])


def main() -> int:
    test_parse_joint_states()
    test_precheck_rejects_invalid()
    test_precheck_accepts_defaults()
    test_runtime_constraints()
    test_pose_diversity()
    print("waypoints_test: all passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
