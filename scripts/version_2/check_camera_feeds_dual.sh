#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

FRONT_TOPIC="/world/office_cpr_px4/model/x500_dual_cam_0/link/camera_link/sensor/camera/image"
DOWN_TOPIC="/world/office_cpr_px4/model/x500_dual_cam_0/link/down_camera_link/sensor/down_camera/image"

echo "[INFO] Checking front camera:"
echo "$FRONT_TOPIC"
timeout 10 ros2 topic hz "$FRONT_TOPIC" || true

echo "[INFO] Checking down camera:"
echo "$DOWN_TOPIC"
timeout 10 ros2 topic hz "$DOWN_TOPIC" || true
