#!/usr/bin/env bash
# ==============================================================================
# Script: install_ros2_dependencies.sh
# Description: Installs all system and ROS 2 Humble dependencies for the
#              4-Wheel Autonomous Mobile Robot workspace.
# Author: Rahul Ramasamy
# Email: rahul.r.joshua123@gmail.com
# ==============================================================================

set -e

# Colored Output Formats
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

echo -e "${BLUE}${BOLD}====================================================================${NC}"
echo -e "${CYAN}${BOLD} 📦 INSTALLING ROS 2 HUMBLE MOBILE ROBOT DEPENDENCIES${NC}"
echo -e "${BLUE}${BOLD}====================================================================${NC}"

# 1. Verify ROS 2 Humble Underlay
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}[ERROR] ROS 2 Humble is not installed under /opt/ros/humble!${NC}"
    echo -e "${YELLOW}Please install ROS 2 Humble Hawksbill first: https://docs.ros.org/en/humble/Installation.html${NC}"
    exit 1
fi

source /opt/ros/humble/setup.bash
echo -e "${GREEN}>>> [1/6] Sourced /opt/ros/humble/setup.bash successfully.${NC}"

# 2. Update System Package Lists
echo -e "${CYAN}>>> [2/6] Updating apt package index...${NC}"
sudo apt-get update -y

# 3. Install Core Build & Simulation Tools
echo -e "${CYAN}>>> [3/6] Installing build essentials, gedit, python utilities & serial libraries...${NC}"
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    gedit \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pytest \
    python3-serial

# 4. Install ROS 2 Packages (Gazebo, Controllers, Teleop, Twist Mux, RViz2)
echo -e "${CYAN}>>> [4/6] Installing ROS 2 simulation, control, and teleop packages...${NC}"
sudo apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros-gz \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-interfaces \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-twist-mux \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard

# 5. Initialize and Update rosdep
echo -e "${CYAN}>>> [5/6] Updating rosdep definitions...${NC}"
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
    sudo rosdep init || true
fi
rosdep update

# 6. Install Workspace Source Dependencies
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo -e "${CYAN}>>> [6/6] Resolving workspace dependencies inside ${WS_DIR}/src...${NC}"
cd "${WS_DIR}"
rosdep install --from-paths src --ignore-src -r -y || true

echo -e "${BLUE}${BOLD}====================================================================${NC}"
echo -e "${GREEN}${BOLD} ✅ ALL DEPENDENCIES INSTALLED SUCCESSFULLY!${NC}"
echo -e "${CYAN} Next Step: Run ${YELLOW}bash scripts/build_all.sh${CYAN} to compile the workspace.${NC}"
echo -e "${BLUE}${BOLD}====================================================================${NC}"
