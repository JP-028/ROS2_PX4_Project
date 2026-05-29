#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

pkill -f ros_gz_bridge || true
pkill -f ros_gz_image || true
pkill -f parameter_bridge || true

ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_dual_cam_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image
