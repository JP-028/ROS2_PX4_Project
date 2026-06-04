#!/bin/bash
set -e

SPEC_FILE="$1"

if [ -z "$SPEC_FILE" ]; then
  echo "[ERROR] No YAML path specification provided."
  echo
  echo "Usage:"
  echo "  ./scripts/version_2/start_path_flight.sh path_specs/circle_3m.yaml"
  echo "  ./scripts/version_2/start_path_flight.sh path_specs/square_3m.yaml"
  echo
  exit 1
fi

if [ ! -f "$SPEC_FILE" ]; then
  echo "[ERROR] YAML path specification not found:"
  echo "$SPEC_FILE"
  exit 1
fi

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

cd /home/user/ros2_ws/ROS2_PX4_Project

echo "[INFO] Executing YAML flight using /uav/local_pose:"
echo "       $SPEC_FILE"
echo
echo "[INFO] Required before running this:"
echo "       1. start_agent.sh"
echo "       2. sim_environment_menu.sh"
echo "       3. start_offboard_control.sh"
echo "       4. start_local_pose_from_px4.sh or another /uav/local_pose provider"
echo

./scripts/version_2/execute_path_from_local_pose.py "$SPEC_FILE"
