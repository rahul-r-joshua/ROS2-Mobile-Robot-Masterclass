#!/usr/bin/env bash
# ==============================================================================
# Script: sync_ros2_humble_env.sh
# Purpose: Fix Exit Code 127, install twist_mux, and synchronize all dependencies
# Author: Rahul Ramasamy
# ==============================================================================
set -e

echo -e "\n\033[1;34m[1/5] Updating system apt repositories...\033[0m"
sudo apt-get update -y

echo -e "\n\033[1;34m[2/5] Installing twist_mux and core teleop packages...\033[0m"
sudo apt-get install -y \
  ros-humble-twist-mux \
  ros-humble-teleop-twist-keyboard \
  ros-humble-teleop-twist-joy \
  ros-humble-joy \
  ros-humble-joy-linux

echo -e "\n\033[1;34m[3/5] Installing ros2_control & Gazebo integration...\033[0m"
sudo apt-get install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro

echo -e "\n\033[1;34m[4/5] Installing Navigation2, SLAM Toolbox & Transform libraries...\033[0m"
sudo apt-get install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-tf-transformations \
  python3-transforms3d \
  python3-rosdep \
  python3-colcon-common-extensions

echo -e "\n\033[1;34m[5/5] Initializing rosdep & resolving workspace dependencies...\033[0m"
sudo rosdep init 2>/dev/null || true
rosdep update

if [ -d "$HOME/ros2_mobile_robot_ws/src" ]; then
  cd "$HOME/ros2_mobile_robot_ws"
  rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
fi

echo -e "\n\033[1;32m[SUCCESS] Environment synchronized! twist_mux and all runtime dependencies are ready.\033[0m"
