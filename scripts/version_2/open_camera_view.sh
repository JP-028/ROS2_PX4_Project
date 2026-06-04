#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

export DISPLAY=host.docker.internal:0
export QT_X11_NO_MITSHM=1

echo "[INFO] Available image topics:"
ROS2CLI_DISABLE_DAEMON=1 ros2 topic list | grep "/image" | sort -u || true

echo
echo "[INFO] Opening rqt_image_view..."
ros2 run rqt_image_view rqt_image_view
