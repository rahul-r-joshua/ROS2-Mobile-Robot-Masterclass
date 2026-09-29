#!/usr/bin/env bash
# ==============================================================================
# Script: clean_build.sh
# Description: Purges build/, install/, and log/ cache directories, cleans
#              stale simulation processes, and performs a fresh rebuild from scratch.
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
NC='\033[0m'

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${WS_DIR}"

echo -e "${BLUE}${BOLD}====================================================================${NC}"
echo -e "${YELLOW}${BOLD} 🧹 PURGING BUILD CACHES & PERFORMING CLEAN REBUILD${NC}"
echo -e "${BLUE}${BOLD}====================================================================${NC}"

# 1. Clean simulation and DDS processes first
if [ -f "${WS_DIR}/clean_simulation.sh" ]; then
    echo -e "${CYAN}>>> [1/4] Terminating background ROS 2 and Gazebo processes...${NC}"
    bash "${WS_DIR}/clean_simulation.sh" || true
fi

# 2. Remove build artifacts
echo -e "${CYAN}>>> [2/4] Removing build/, install/, and log/ directories...${NC}"
rm -rf "${WS_DIR}/build" "${WS_DIR}/install" "${WS_DIR}/log"
echo -e "${GREEN}  ✓ Caches cleared.${NC}"

# 3. Source ROS 2 Underlay
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# 4. Trigger Fresh Build
echo -e "${CYAN}>>> [3/4] Compiling fresh workspace with colcon...${NC}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5. Sourcing
echo -e "${CYAN}>>> [4/4] Sourcing fresh workspace overlay...${NC}"
source "${WS_DIR}/install/setup.bash"

echo -e "${BLUE}${BOLD}====================================================================${NC}"
echo -e "${GREEN}${BOLD} ✨ FRESH REBUILD COMPLETED SUCCESSFULLY!${NC}"
echo -e "${BLUE}${BOLD}====================================================================${NC}"
