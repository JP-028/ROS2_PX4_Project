#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

FRONT_TOPIC="/world/office_cpr_px4/model/x500_dual_cam_0/link/camera_link/sensor/camera/image"
DOWN_TOPIC="/world/office_cpr_px4/model/x500_dual_cam_0/link/down_camera_link/sensor/down_camera/image"

echo "[INFO] Bridging front camera:"
echo "$FRONT_TOPIC"

echo "[INFO] Bridging down camera:"
echo "$DOWN_TOPIC"

ros2 run ros_gz_bridge parameter_bridge \
  "$FRONT_TOPIC@sensor_msgs/msg/Image@gz.msgs.Image" \
  "$DOWN_TOPIC@sensor_msgs/msg/Image@gz.msgs.Image"
