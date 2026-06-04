#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

cd /home/user/ros2_ws/ROS2_PX4_Project

echo "[INFO] Starting square path analysis"
echo "[INFO] Make sure the system is already running:"
echo "       1. start_agent.sh"
echo "       2. start_px4_gazebo_office_cpr_px4.sh or another PX4/Gazebo startup script"
echo "       3. start_offboard_control.sh"
echo
echo "[INFO] Now start the square flight in another terminal:"
echo "       ./scripts/version_2/start_square_flight.sh"
echo
echo "[INFO] When the flight is finished, stop this analyzer with Ctrl+C."
echo "[INFO] Results will be saved in:"
echo "       path_analysis/<date>_<time>_square/"
echo

./scripts/version_2/analyze_path_from_odometry.py --mode square --side-length 5.0
