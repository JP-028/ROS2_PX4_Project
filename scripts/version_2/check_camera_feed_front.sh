#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

TOPIC="/world/office_cpr_px4/model/x500_dual_cam_0/link/camera_link/sensor/camera/image"

echo "[INFO] Checking front camera:"
echo "$TOPIC"

timeout 10 ros2 topic hz "$TOPIC"
