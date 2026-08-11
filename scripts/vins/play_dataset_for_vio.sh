#!/usr/bin/env bash

set -euo pipefail

PROJECT_DIR="/home/user/ros2_ws/ROS2_PX4_Project"

if [ "$#" -ne 1 ]; then
    echo "Usage:"
    echo "  $0 <rosbag-directory>"
    echo
    echo "Example:"
    echo "  $0 batch_experiments/<experiment>/runs/run_001/rosbag/sensor_recording"
    exit 1
fi

INPUT_BAG="$1"

if [ ! -d "$INPUT_BAG" ]; then
    echo "[ERROR] Rosbag directory not found:"
    echo "        $INPUT_BAG"
    exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
set -u

cd "$PROJECT_DIR"

ADAPTER_PID=""

cleanup() {
    echo
    echo "[INFO] Stopping dataset adapter..."

    if [ -n "$ADAPTER_PID" ] && kill -0 "$ADAPTER_PID" 2>/dev/null; then
        kill -SIGINT "$ADAPTER_PID" 2>/dev/null || true
        wait "$ADAPTER_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

echo "============================================================"
echo " VINS-compatible dataset playback"
echo "============================================================"
echo "Input bag:"
echo "$INPUT_BAG"
echo
echo "Standardized output topics:"
echo "  /stereo/left/image_raw"
echo "  /stereo/right/image_raw"
echo "  /vins/imu"
echo "============================================================"
echo

python3 scripts/vins/px4_bag_to_vins_adapter.py &
ADAPTER_PID=$!

echo "[INFO] Waiting for dataset adapter..."

ADAPTER_READY=false

for ATTEMPT in $(seq 1 30); do
    if ros2 node list 2>/dev/null |
       grep -q '^/px4_bag_to_vins_adapter$'; then
        ADAPTER_READY=true
        break
    fi

    if ! kill -0 "$ADAPTER_PID" 2>/dev/null; then
        echo "[ERROR] Dataset adapter exited during startup."
        wait "$ADAPTER_PID" 2>/dev/null || true
        exit 1
    fi

    sleep 1
done

if [ "$ADAPTER_READY" != true ]; then
    echo "[ERROR] Dataset adapter did not become ready."
    exit 1
fi

echo "[OK] Dataset adapter is ready."
echo "[INFO] Starting rosbag playback..."
echo

ros2 bag play "$INPUT_BAG" \
    --clock \
    --remap \
    /stereo/left/image_raw:=/dataset/stereo/left/image_raw \
    /stereo/right/image_raw:=/dataset/stereo/right/image_raw \
    /fmu/out/sensor_combined:=/dataset/fmu/out/sensor_combined \
    </dev/null

echo
echo "[OK] Rosbag playback finished."
