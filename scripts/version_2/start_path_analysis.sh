#!/bin/bash
set -e

SPEC_FILE="$1"

if [ -z "$SPEC_FILE" ]; then
  echo "[ERROR] No YAML path specification provided."
  echo
  echo "Usage:"
  echo "  ./scripts/version_2/start_path_analysis.sh path_specs/circle_3m.yaml"
  echo "  ./scripts/version_2/start_path_analysis.sh path_specs/square_3m.yaml"
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

echo "[INFO] Starting path analysis from YAML spec:"
echo "       $SPEC_FILE"
echo
echo "[INFO] Start the matching flight executor in another terminal."
echo "[INFO] Stop this analyzer with Ctrl+C after the flight is complete."
echo

./scripts/version_2/analyze_path_from_spec.py "$SPEC_FILE"
