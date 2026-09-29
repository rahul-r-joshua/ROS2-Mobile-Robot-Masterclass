#!/usr/bin/env bash
# ==============================================================================
# Master ROS 2 & Gazebo Simulation Cleanup Utility
# ------------------------------------------------------------------------------
# Author: Rahul Ramasamy
# Email: rahul.r.joshua123@gmail.com
# GitHub: https://github.com/rahul-r-joshua
# Portfolio: https://rahul-r-joshua.github.io/portfolio/
#
# Complete teardown of all simulation, controller, and visualization processes:
#  1. Force terminates gzserver, gzclient, rviz2, and ROS 2 launch processes.
#  2. Kills controller_manager, spawners, bridges, and python utility nodes.
#  3. Stops the ROS 2 daemon to clear stale DDS discovery tables.
#  4. Purges orphaned FastDDS shared memory (/dev/shm) blocks & semaphore mutexes.
#  5. Cleans temporary Gazebo IPC sockets (/tmp/gazebo* /tmp/gz*).
#  6. Restarts a clean ROS 2 daemon ready for fresh simulation.
# ==============================================================================

set -e

echo "===================================================================="
echo "🧹 Cleaning ROS 2, Gazebo, RViz & FastDDS Shared Memory Environment"
echo "===================================================================="

echo ">>> [1/5] Terminating simulation and visualization processes..."
killall -9 gzserver gzclient rviz2 robot_state_publisher ruby ign gz parameter_bridge 2>/dev/null || true
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "ros2 launch" 2>/dev/null || true
pkill -9 -f "controller_manager" 2>/dev/null || true
pkill -9 -f "spawner" 2>/dev/null || true
pkill -9 -f "twist_relay" 2>/dev/null || true
pkill -9 -f "joint_state_publisher" 2>/dev/null || true
pkill -9 -f "diff_drive_controller" 2>/dev/null || true
pkill -9 -f "safety_zone_controller" 2>/dev/null || true

echo ">>> [2/5] Stopping ROS 2 daemon..."
ros2 daemon stop 2>/dev/null || true

echo ">>> [3/5] Purging orphaned FastDDS shared memory (/dev/shm)..."
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true

echo ">>> [4/5] Removing stale Gazebo IPC sockets (/tmp)..."
rm -rf /tmp/gazebo* /tmp/gz* /tmp/ign* 2>/dev/null || true

echo ">>> [5/5] Restarting clean ROS 2 daemon..."
ros2 daemon start 2>/dev/null || true

echo "===================================================================="
echo "✅ Environment completely cleaned! Ready for fresh simulation launch."
echo "===================================================================="
