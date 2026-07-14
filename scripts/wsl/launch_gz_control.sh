#!/usr/bin/env bash
# Gazebo + MoveIt launch helper (run inside WSL)
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
echo "Launching Gazebo + MoveIt. Click Play in Gazebo, then run pose tests."
ros2 launch panda_moveit_config ex_gz_control.launch.py
