#!/usr/bin/env bash
# ==============================================================================
# Script: setup_environment.sh
# Description: Sources ROS 2 Humble underlay and local workspace overlay,
#              configures isolated lab networking parameters (ROS_DOMAIN_ID),
#              and exports simulation backend variables.
# Author: Rahul Ramasamy
# Email: rahul.r.joshua123@gmail.com
# ==============================================================================

# Determine Workspace Directory Root
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Colors
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "${CYAN}${BOLD}>>> Configuring ROS 2 Mobile Robot Environment...${NC}"

# 1. Source Global ROS 2 Humble Underlay
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}  ✓ Sourced /opt/ros/humble/setup.bash${NC}"
else
    echo -e "${YELLOW}  ⚠ Warning: /opt/ros/humble/setup.bash not found!${NC}"
fi

# 2. Source Local Workspace Overlay if built
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
    echo -e "${GREEN}  ✓ Sourced ${WS_DIR}/install/setup.bash${NC}"
else
    echo -e "${YELLOW}  ⚠ Notice: ${WS_DIR}/install/setup.bash not found. Run 'bash scripts/build_all.sh' first.${NC}"
fi

# 3. Configure DDS & Localhost Network Isolation
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1

# 4. Configure Gazebo & Ignition Model Paths
export GAZEBO_MODEL_PATH="${WS_DIR}/src/mobile_robot_description/urdf:${GAZEBO_MODEL_PATH}"
export IGN_GAZEBO_RESOURCE_PATH="${WS_DIR}/src/mobile_robot_description/urdf:${IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="${WS_DIR}/src/mobile_robot_description/urdf:${GZ_SIM_RESOURCE_PATH}"

echo -e "${GREEN}  ✓ Network: ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}${NC}"
echo -e "${GREEN}  ✓ Middleware: RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}${NC}"
echo -e "${CYAN}${BOLD}>>> Workspace Environment Ready! 🚀${NC}"
