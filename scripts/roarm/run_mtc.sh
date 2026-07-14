#!/usr/bin/env bash
# Run one MoveIt Task Constructor demo for RoArm-M3 (RoArm_M3 terminal 3).
# Prerequisites: roarm_launch_driver.sh + roarm_launch_mtc_demo.sh running.
# Usage:
#   bash run_mtc.sh <cartesian|cartesian_modular|pick_place>
#   bash run_mtc.sh pick_place --object-pose 0.25 0.05 0.08 0 0 0
#   bash run_mtc.sh pick_place --object-pose ... --place-pose ... --spawn-table false
# Stop the previous run.launch.py (Ctrl+C) before switching demos.
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROARM_MODEL=roarm_m3
source ~/roarm_ws/roarm_ws-ros2-humble/install/setup.bash
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

EXE="${1:-}"
shift || true

case "$EXE" in
  cartesian|cartesian_modular|pick_place) ;;
  *)
    echo "Usage: $0 <cartesian|cartesian_modular|pick_place> [pick_place options...]"
    echo "  cartesian         - Cartesian demo"
    echo "  cartesian_modular - Cartesian modular demo"
    echo "  pick_place        - Pick and place demo"
    echo ""
    echo "pick_place options:"
    echo "  --object-pose X Y Z R P YAW"
    echo "  --place-pose  X Y Z R P YAW"
    echo "  --object-dimensions HEIGHT RADIUS"
    echo "  --spawn-table true|false"
    echo "  --execute true|false"
    exit 1
    ;;
esac

ROS_ARGS=()
OBJECT_POSE=()
PLACE_POSE=()
OBJECT_DIMS=()
SPAWN_TABLE=""
EXECUTE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --object-pose)
      OBJECT_POSE=("$2" "$3" "$4" "$5" "$6" "$7")
      shift 7
      ;;
    --place-pose)
      PLACE_POSE=("$2" "$3" "$4" "$5" "$6" "$7")
      shift 7
      ;;
    --object-dimensions)
      OBJECT_DIMS=("$2" "$3")
      shift 3
      ;;
    --spawn-table)
      SPAWN_TABLE="$2"
      shift 2
      ;;
    --execute)
      EXECUTE="$2"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1"
      exit 1
      ;;
  esac
done

if ((${#OBJECT_POSE[@]} == 6)); then
  ROS_ARGS+=(-p "object_pose:=[${OBJECT_POSE[0]}, ${OBJECT_POSE[1]}, ${OBJECT_POSE[2]}, ${OBJECT_POSE[3]}, ${OBJECT_POSE[4]}, ${OBJECT_POSE[5]}]")
fi
if ((${#PLACE_POSE[@]} == 6)); then
  ROS_ARGS+=(-p "place_pose:=[${PLACE_POSE[0]}, ${PLACE_POSE[1]}, ${PLACE_POSE[2]}, ${PLACE_POSE[3]}, ${PLACE_POSE[4]}, ${PLACE_POSE[5]}]")
fi
if ((${#OBJECT_DIMS[@]} == 2)); then
  ROS_ARGS+=(-p "object_dimensions:=[${OBJECT_DIMS[0]}, ${OBJECT_DIMS[1]}]")
fi
if [[ -n "$SPAWN_TABLE" ]]; then
  ROS_ARGS+=(-p "spawn_table:=${SPAWN_TABLE}")
fi
if [[ -n "$EXECUTE" ]]; then
  ROS_ARGS+=(-p "execute:=${EXECUTE}")
fi

echo "==> Starting MTC demo: exe:=${EXE}"
if ((${#ROS_ARGS[@]} > 0)); then
  echo "==> Parameter overrides: ${ROS_ARGS[*]}"
fi
echo "==> In the MTC UI: select the full path, then click Exec."

if ((${#ROS_ARGS[@]} > 0)); then
  exec ros2 launch roarm_moveit_mtc_demo run.launch.py "exe:=${EXE}" --ros-args "${ROS_ARGS[@]}"
else
  exec ros2 launch roarm_moveit_mtc_demo run.launch.py "exe:=${EXE}"
fi
