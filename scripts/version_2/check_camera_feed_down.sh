#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

TOPIC="/world/office_cpr_px4/model/x500_dual_cam_0/link/down_camera_link/sensor/down_camera/image"

echo "[INFO] Checking down camera:"
echo "$TOPIC"

timeout 10 ros2 topic hz "$TOPIC"
