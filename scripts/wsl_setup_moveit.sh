#!/usr/bin/env bash
# WSL setup for pymoveit2 + panda_gz_moveit2 (ROS 2 Jazzy)
# Run inside WSL: bash scripts/wsl_setup_moveit.sh

set -eo pipefail

echo "==> Installing system dependencies (requires sudo password)..."
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-jazzy-moveit \
  ros-jazzy-moveit-servo \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-urdf-launch \
  python3-colcon-common-extensions \
  python3-rosdep \
  git \
  x11-apps

echo "==> Sourcing ROS 2 Jazzy..."
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

echo "==> Ensuring workspace repos exist..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if [ ! -d panda_gz_moveit2 ]; then
  git clone -b jazzy https://github.com/AndrejOrsula/panda_gz_moveit2.git
fi
if [ ! -d pymoveit2 ]; then
  git clone https://github.com/AndrejOrsula/pymoveit2.git
fi

echo "==> Jazzy fix: fake_components -> mock_components..."
sed -i 's|fake_components/GenericSystem|mock_components/GenericSystem|g' \
  ~/ros2_ws/src/panda_gz_moveit2/panda_description/urdf/panda.ros2_control

echo "==> Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

echo "==> Installing package dependencies..."
cd ~/ros2_ws
rosdep install -y -r -i --rosdistro jazzy --from-paths src || true

echo "==> Building workspace..."
colcon build --merge-install --symlink-install --cmake-args "-DCMAKE_BUILD_TYPE=Release"

cat <<'EOF'

==> Setup complete.

Add to ~/.bashrc:
  source /opt/ros/jazzy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export QT_QPA_PLATFORM=xcb
  source ~/ros2_ws/install/local_setup.bash

Terminal 1 (fake control / RViz only):
  ros2 launch panda_moveit_config ex_fake_control.launch.py

Terminal 2 (pymoveit2 joint goal):
  ros2 run pymoveit2 ex_joint_goal.py --ros-args \
    -p joint_positions:="[1.57, -1.57, 0.0, -1.57, 0.0, 1.57, 0.7854]"

Gazebo simulation:
  ros2 launch panda_moveit_config ex_gz_control.launch.py
  # Click Play in Gazebo GUI, then run the pymoveit2 command above.

EOF
