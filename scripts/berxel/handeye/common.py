#!/usr/bin/env python3
"""Shared helpers for Berxel + RoArm eye-to-hand calibration."""

from __future__ import annotations

import json
import re
import subprocess
from pathlib import Path

import cv2
import numpy as np
import yaml

ARUCO_DICT_MAP = {
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def default_intrinsics_path() -> Path:
    return repo_root() / "calibration" / "berxel" / "intrinsics.yaml"


def default_hand_eye_dir() -> Path:
    return repo_root() / "calibration" / "hand_eye"


def load_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, dict]:
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["distortion_coefficients"], dtype=np.float64).reshape(-1, 1)
    return camera_matrix, dist_coeffs, data


def get_aruco_dictionary(name: str):
    if name not in ARUCO_DICT_MAP:
        raise ValueError(f"Unsupported dictionary: {name}")
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICT_MAP[name])


def make_aruco_detector(dictionary):
    if hasattr(cv2.aruco, "ArucoDetector"):
        params = cv2.aruco.DetectorParameters()
        return cv2.aruco.ArucoDetector(dictionary, params), "new"
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    return params, "legacy"


def detect_apriltag(
    gray: np.ndarray,
    detector,
    api_mode: str,
    dictionary,
    tag_id: int,
):
    if api_mode == "new":
        corners, ids, _rejected = detector.detectMarkers(gray)
    else:
        corners, ids, _rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=detector)

    if ids is None or len(ids) == 0:
        return False, None, None

    for idx, marker_id in enumerate(ids.flatten()):
        if int(marker_id) == tag_id:
            return True, corners[idx : idx + 1], int(marker_id)
    return False, None, None


def estimate_tag_pose(
    corners,
    tag_size: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    rvecs, tvecs, _obj = cv2.aruco.estimatePoseSingleMarkers(
        corners, tag_size, camera_matrix, dist_coeffs
    )
    return rvecs[0, 0], tvecs[0, 0]


def rvec_tvec_to_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    rot, _ = cv2.Rodrigues(rvec)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rot
    transform[:3, 3] = tvec.reshape(3)
    return transform


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RoArm-style fixed-axis roll-pitch-yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def pose_xyz_rpy_to_matrix(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rpy_to_matrix(roll, pitch, yaw)
    transform[:3, 3] = [x, y, z]
    return transform


def matrix_to_rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    sy = np.sqrt(rotation[0, 0] ** 2 + rotation[1, 0] ** 2)
    if sy >= 1e-6:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0.0
    return float(roll), float(pitch), float(yaw)


def split_rt(transform: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return transform[:3, :3].copy(), transform[:3, 3].reshape(3, 1).copy()


def wsl_executable() -> str:
    win_wsl = Path("/mnt/c/Windows/System32/wsl.exe")
    if win_wsl.exists():
        return str(win_wsl)
    return "wsl"


ROARM_WS_SETUP = "/home/ag/roarm_ws/roarm_ws-ros2-humble/install/setup.bash"


def ros_env_prefix() -> str:
    return (
        "source /opt/ros/humble/setup.bash && "
        "export ROARM_MODEL=roarm_m3 && "
        f"source {ROARM_WS_SETUP} && "
    )


def run_roarm_ros(
    ros_cmd: str,
    wsl_distro: str = "RoArm_M3",
    timeout_s: int = 60,
) -> subprocess.CompletedProcess[str]:
    # Nested wsl.exe from another distro fails if cwd is a Linux-home path
    # (e.g. /home/zifan -> \\wsl.localhost\Ubuntu-24.04\...). Use a
    # Windows-visible cwd and --cd into RoArm_M3 home.
    full_cmd = ros_env_prefix() + ros_cmd
    cmd = [
        wsl_executable(),
        "-d",
        wsl_distro,
        "--cd",
        "/home/ag",
        "bash",
        "-lc",
        full_cmd,
    ]
    host_cwd = "/mnt/c/Windows/System32"
    if not Path(host_cwd).is_dir():
        host_cwd = "/mnt/d"
    try:
        return subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
            cwd=host_cwd,
        )
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(
            f"RoArm ROS command timed out after {timeout_s}s.\n"
            f"Command: {ros_cmd}\n"
            "Ensure RoArm_M3 is running scripts/roarm/launch_driver.sh and "
            "scripts/roarm/launch_command_control.sh, then retry."
        ) from exc


def check_roarm_pose_only(wsl_distro: str = "RoArm_M3") -> None:
    """Verify /get_pose_cmd only (fast path; no joint_states probe)."""
    try:
        read_roarm_pose(wsl_distro=wsl_distro, timeout_s=60)
    except RuntimeError as exc:
        raise RuntimeError(
            "Cannot reach /get_pose_cmd. In RoArm_M3, start:\n"
            "  scripts/roarm/launch_driver.sh\n"
            "  scripts/roarm/launch_command_control.sh\n"
            "Then verify: ros2 service call /get_pose_cmd roarm_msgs/srv/GetPoseCmd"
        ) from exc


def check_roarm_stack(wsl_distro: str = "RoArm_M3") -> float | None:
    """Verify RoArm stack; return gripper home angle if joint_states is readable."""
    try:
        read_roarm_pose(wsl_distro=wsl_distro, timeout_s=90)
    except RuntimeError as exc:
        raise RuntimeError(
            "Cannot reach /get_pose_cmd. In RoArm_M3, start:\n"
            "  roarm_launch_driver.sh\n"
            "  roarm_launch_command_control.sh\n"
            "Then verify: ros2 service call /get_pose_cmd roarm_msgs/srv/GetPoseCmd"
        ) from exc

    try:
        joints = read_joint_states(wsl_distro=wsl_distro, timeout_s=90)
    except RuntimeError:
        print(
            "WARNING: /joint_states not readable via cross-WSL probe. "
            "Continuing with pose-only constraint checks."
        )
        return None
    return joints.get("link5_to_gripper_link")


def read_roarm_pose(wsl_distro: str = "RoArm_M3", timeout_s: int = 90) -> dict:
    result = run_roarm_ros(
        "timeout 45 ros2 service call /get_pose_cmd roarm_msgs/srv/GetPoseCmd",
        wsl_distro=wsl_distro,
        timeout_s=timeout_s,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "Failed to call /get_pose_cmd. Is RoArm driver + command_control running?\n"
            f"stderr: {result.stderr.strip()}"
        )
    match = re.search(
        r"x=([-+0-9.eE]+).*y=([-+0-9.eE]+).*z=([-+0-9.eE]+).*"
        r"roll=([-+0-9.eE]+).*pitch=([-+0-9.eE]+).*yaw=([-+0-9.eE]+)",
        result.stdout,
        flags=re.S,
    )
    if not match:
        raise RuntimeError(f"Could not parse GetPoseCmd response:\n{result.stdout}")
    keys = ["x", "y", "z", "roll", "pitch", "yaw"]
    return {k: float(v) for k, v in zip(keys, match.groups())}


def parse_joint_states_message(text: str) -> dict[str, float]:
    names: list[str] = []
    positions: list[float] = []
    section: str | None = None
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line == "name:":
            section = "name"
            continue
        if line == "position:":
            section = "position"
            continue
        if not line.startswith("- "):
            if section == "name" and line and not line.endswith(":"):
                section = None
            continue
        value = line[2:].strip()
        if section == "name":
            names.append(value)
        elif section == "position":
            positions.append(float(value))
    if not names or len(names) != len(positions):
        raise RuntimeError(f"Could not parse /joint_states message:\n{text}")
    return dict(zip(names, positions))


def read_joint_states(wsl_distro: str = "RoArm_M3", timeout_s: int = 90) -> dict[str, float]:
    result = run_roarm_ros(
        "timeout 75 ros2 topic echo /joint_states --once",
        wsl_distro=wsl_distro,
        timeout_s=timeout_s,
    )
    if result.returncode != 0:
        raise RuntimeError(
            "Failed to read /joint_states.\n"
            f"stderr: {result.stderr.strip()}"
        )
    return parse_joint_states_message(result.stdout)


def call_move_joint_cmd(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float = 0.0,
    wsl_distro: str = "RoArm_M3",
    timeout_s: int = 180,
) -> tuple[bool, str]:
    request = (
        f"{{x: {x}, y: {y}, z: {z}, roll: {roll}, pitch: {pitch}, yaw: {yaw}}}"
    )
    result = run_roarm_ros(
        "ros2 service call /move_joint_cmd roarm_msgs/srv/MoveJointCmd "
        f"'{request}'",
        wsl_distro=wsl_distro,
        timeout_s=timeout_s,
    )
    if result.returncode != 0:
        return False, result.stderr.strip() or "move_joint_cmd failed"
    if "success=True" in result.stdout or "success: true" in result.stdout.lower():
        return True, "ok"
    if "Planning failed" in result.stdout:
        return False, "planning failed"
    message_match = re.search(r"message='([^']*)'", result.stdout)
    message = message_match.group(1) if message_match else result.stdout.strip()
    return False, message or "unknown move_joint_cmd failure"


def call_gripper_cmd(
    position_rad: float,
    wsl_distro: str = "RoArm_M3",
    timeout_s: int = 30,
) -> tuple[bool, str]:
    """Publish gripper target angle (rad). RoArm docs: ~0.0 open, ~1.5 closed."""
    result = run_roarm_ros(
        "ros2 topic pub --once /gripper_cmd std_msgs/msg/Float32 "
        f"'{{data: {float(position_rad)}}}'",
        wsl_distro=wsl_distro,
        timeout_s=timeout_s,
    )
    if result.returncode != 0:
        return False, result.stderr.strip() or "gripper_cmd publish failed"
    return True, "ok"


def go_home_pose(
    home_pose: dict[str, float],
    wsl_distro: str = "RoArm_M3",
    timeout_s: int = 120,
) -> tuple[bool, str]:
    return call_move_joint_cmd(
        home_pose["x"],
        home_pose["y"],
        home_pose["z"],
        home_pose["roll"],
        home_pose["pitch"],
        home_pose.get("yaw", 0.0),
        wsl_distro=wsl_distro,
        timeout_s=timeout_s,
    )


def save_sample_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def load_sample_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))
