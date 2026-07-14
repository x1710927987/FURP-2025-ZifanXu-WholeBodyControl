#!/bin/bash
# Quick RoArm stack probe from Ubuntu-24.04 (cross-WSL).
set -eu
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHONPATH="$SCRIPT_DIR" python3 - <<'PY'
from common import check_roarm_pose_only, read_roarm_pose

check_roarm_pose_only()
pose = read_roarm_pose(timeout_s=60)
print("OK:", pose)
PY
