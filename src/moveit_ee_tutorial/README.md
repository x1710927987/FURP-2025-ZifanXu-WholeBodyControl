# Hello MoveIt — End-Effector Planning (Panda Proxy)

Phase 1 tutorial for FURP: plan and execute end-effector poses with **MoveIt 2** and **pymoveit2**, using Franka Panda in simulation as a stand-in until the lab dual-arm URDF is available.

## Environment

- WSL2 Ubuntu 24.04 + ROS 2 Jazzy
- Workspace: `~/ros2_ws` (`panda_gz_moveit2` + `pymoveit2`)
- Setup script: [`../../scripts/wsl_setup_moveit.sh`](../../scripts/wsl_setup_moveit.sh)

Add to `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM=xcb
source ~/ros2_ws/install/local_setup.bash
```

## Quick start

**Terminal 1 — RViz + MoveIt (fake hardware):**

```bash
bash scripts/wsl_launch_fake_control.sh
```

This stops stale nodes, launches `ex_fake_control`, waits 15s, then re-spawns controllers (fixes Jazzy spawner race). Log: `/tmp/panda_fake_control.log`.

Or manually:

```bash
ros2 launch panda_moveit_config ex_fake_control.launch.py
```

**Terminal 2 — built-in pymoveit2 examples:**

```bash
# Joint-space goal (sanity check)
ros2 run pymoveit2 ex_joint_goal.py --ros-args \
  -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"

# Cartesian pose goal (end-effector)
ros2 run pymoveit2 ex_pose_goal.py --ros-args \
  -p position:="[0.25, 0.0, 1.0]" \
  -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" \
  -p cartesian:=False

# Gripper
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"

# Collision object + replan
ros2 run pymoveit2 ex_collision_primitive.py --ros-args \
  -p shape:="sphere" -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.04]"
```

**Terminal 2 — this tutorial wrapper:**

```bash
cd /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/src/moveit_ee_tutorial
python3 move_to_pose.py --x 0.25 --y 0.0 --z 1.0
```

## Gazebo (physics simulation)

```bash
ros2 launch panda_moveit_config ex_gz_control.launch.py
# Click Play in Gazebo, then run pose commands above.
```

## Batch test script

From repo root (inside WSL):

```bash
bash scripts/wsl_run_ee_tests.sh
```

## What you should see

- **RViz2**: orange Panda arm; MotionPlanning / Planned Path displays motion.
- **Terminal**: `Moving to {position: ...}` then successful execution (no `Planning failed!`).

## Collision-aware planning

With fake control running:

```bash
bash scripts/wsl_test_collision_planning.sh
```

This adds a sphere obstacle (`ex_collision_primitive.py`), plans an EE pose, then clears the scene (`ex_clear_planning_scene.py` requires `pip3 install trimesh`).

Note: `panda_gz_moveit2` ships without link collision meshes, so collision-aware planning may return `FAILURE` even after adding scene obstacles. Adding the primitive to the planning scene still works and is visible in RViz.


`panda_gz_moveit2` uses `mock_components/GenericSystem` instead of `fake_components` on Jazzy. Re-apply via `scripts/wsl_setup_moveit.sh` after a fresh clone.

## Next: lab dual-arm platform

When the real URDF is available, replace in `move_to_pose.py`:

- `pymoveit2.robots.panda` preset → custom joint names / `group_name` / `end_effector_name`
- Launch file → your `moveit_config` package

The pymoveit2 call pattern stays the same.
