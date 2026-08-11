#!/usr/bin/env bash

set -euo pipefail

PROJECT="/home/user/ros2_ws/ROS2_PX4_Project"

BAG="$PROJECT/batch_experiments/2026-07-24_12-38-34_vio_marker_arena_segment_vio_marker_arena_circle/runs/run_001/rosbag/sensor_recording"

VINS_CSV="$PROJECT/vins_output/stereo_imu/vio.csv"

OUTPUT_DIR="$PROJECT/vins_output/stereo_imu/circle_6m_run01_comparison"

set +u
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
set -u

if [ ! -d "$BAG" ]; then
    echo "[ERROR] Reference rosbag not found:"
    echo "        $BAG"
    exit 1
fi

if [ ! -f "$VINS_CSV" ]; then
    echo "[ERROR] Stereo+IMU trajectory not found:"
    echo "        $VINS_CSV"
    exit 1
fi

LINE_COUNT=$(wc -l < "$VINS_CSV")

if [ "$LINE_COUNT" -lt 20 ]; then
    echo "[ERROR] The trajectory contains only $LINE_COUNT rows."
    echo "        This is an incomplete VINS run and should not be evaluated."
    exit 1
fi

if [ -d "$OUTPUT_DIR" ]; then
    BACKUP_DIR="${OUTPUT_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
    mv "$OUTPUT_DIR" "$BACKUP_DIR"

    echo "[INFO] Existing comparison archived:"
    echo "       $BACKUP_DIR"
fi

mkdir -p "$OUTPUT_DIR"

echo "============================================================"
echo " VINS-Fusion Stereo + IMU evaluation"
echo "============================================================"
echo "Reference bag:"
echo "  $BAG"
echo
echo "VINS trajectory:"
echo "  $VINS_CSV"
echo
echo "Trajectory rows:"
echo "  $LINE_COUNT"
echo
echo "Output folder:"
echo "  $OUTPUT_DIR"
echo "============================================================"
echo

python3 "$PROJECT/scripts/analysis/compare_vins_to_reference.py" \
    "$BAG" \
    "$VINS_CSV" \
    "$OUTPUT_DIR" \
    --reference-topic /uav/local_pose \
    --association progress \
    --samples 300

echo
echo "=== Created comparison files ==="
find "$OUTPUT_DIR" \
    -maxdepth 1 \
    -type f \
    -printf '%f\n' |
sort

echo
echo "[OK] Stereo+IMU comparison completed."
echo
echo "HTML report:"
echo "$OUTPUT_DIR/trajectory_comparison.html"
