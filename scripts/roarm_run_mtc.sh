#!/usr/bin/env bash
# Run one MoveIt Task Constructor demo for RoArm-M3 (RoArm_M3 terminal 3).
# Prerequisites: roarm_launch_driver.sh + roarm_launch_mtc_demo.sh running.
# Usage: roarm_run_mtc.sh <cartesian|cartesian_modular|pick_place>
# Stop the previous run.launch.py (Ctrl+C) before switching demos.
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

EXE="${1:-}"
case "$EXE" in
  cartesian|cartesian_modular|pick_place) ;;
  *)
    echo "Usage: $0 <cartesian|cartesian_modular|pick_place>"
    echo "  cartesian         - Cartesian demo"
    echo "  cartesian_modular - Cartesian modular demo"
    echo "  pick_place        - Pick and place demo"
    exit 1
    ;;
esac

echo "==> Starting MTC demo: exe:=${EXE}"
echo "==> In the MTC UI: select the full path, then click Exec."
exec ros2 launch roarm_moveit_mtc_demo run.launch.py "exe:=${EXE}"
