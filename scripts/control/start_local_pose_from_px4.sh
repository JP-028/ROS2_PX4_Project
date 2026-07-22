#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

cd /home/user/ros2_ws/ROS2_PX4_Project

echo "[INFO] Starting local pose adapter."
echo "[INFO] Current source: /fmu/out/vehicle_odometry"
echo "[INFO] Published pose: /uav/local_pose"
echo
echo "[INFO] Later this script can be replaced by a camera/grid/ToF-based pose estimator."
echo

./scripts/control/px4_odometry_to_local_pose.py
