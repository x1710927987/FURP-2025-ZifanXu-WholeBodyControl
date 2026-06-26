# MOGI Week 3-4 — URDF + Gazebo Harmonic (Jazzy)

Parallel track while practicing MoveIt end-effector control on Panda.

## Clone starter package (WSL)

```bash
cd /mnt/d/Repository/internship_and_research/FURP-2025-ZifanXu-WholeBodyControl/src/gazebo_basics
git clone -b starter-branch https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics.git
```

## Build (separate colcon workspace or add to existing)

```bash
# Option A: dedicated workspace
mkdir -p ~/gazebo_ws/src
ln -s $(pwd)/Week-3-4-Gazebo-basics/bme_gazebo_basics ~/gazebo_ws/src/
cd ~/gazebo_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Prerequisites

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-urdf-launch
export QT_QPA_PLATFORM=xcb
# Optional offline models (WSL): export GZ_SIM_RESOURCE_PATH=~/gazebo_models
```

## Learning goals (from MOGI)

1. URDF / xacro: links, joints, inertial tags
2. `robot_state_publisher` + RViz display
3. Spawn robot in Gazebo Harmonic via `ros_gz_sim`
4. `ros_gz_bridge` for topic bridging

## Relation to FURP project

- **Now:** Panda + MoveIt (`src/moveit_ee_tutorial/`) for end-effector planning
- **This track:** URDF/Gazebo skills for importing the lab dual-arm URDF later
- **Not here:** HopperTrex Isaac Lab (separate mobile-base RL sandbox)

## Reference

- [MOGI-ROS/Week-3-4-Gazebo-basics](https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics)
