#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

ROS2_WS="${ROS2_WS:-/home/user/ros2_ws}"
PX4_DIR="${PX4_DIR:-/home/user/PX4-Autopilot}"
AGENT_DIR="${AGENT_DIR:-/home/user/Micro-XRCE-DDS-Agent}"

echo "=== Checking required directories ==="

required_directories=(
    "$ROS2_WS/src/px4_msgs"
    "$PX4_DIR"
    "$AGENT_DIR"
)

for directory in "${required_directories[@]}"; do
    if [[ ! -d "$directory" ]]; then
        echo "[ERROR] Required directory not found:"
        echo "        $directory"
        echo
        echo "Run this first:"
        echo "  $PROJECT_ROOT/scripts/setup/install_external_repositories.sh"
        exit 1
    fi
done

echo
echo "=== Building Micro XRCE-DDS Agent ==="

cmake -S "$AGENT_DIR" \
      -B "$AGENT_DIR/build" \
      -DCMAKE_BUILD_TYPE=Release

cmake --build "$AGENT_DIR/build" --parallel

sudo cmake --install "$AGENT_DIR/build"

echo
echo "=== Building the ROS 2 workspace ==="

source /opt/ros/humble/setup.bash

cd "$ROS2_WS"

colcon build \
    --symlink-install \
    --packages-select px4_msgs px4_offboard_control

echo
echo "=== Verifying installation ==="

source "$ROS2_WS/install/setup.bash"

command -v MicroXRCEAgent >/dev/null

ros2 pkg prefix px4_msgs >/dev/null
ros2 pkg prefix px4_offboard_control >/dev/null

echo
echo "[SUCCESS] Micro XRCE-DDS Agent and ROS 2 packages were built."
echo
echo "The PX4 SITL target is compiled automatically when the simulation"
echo "is started for the first time."
