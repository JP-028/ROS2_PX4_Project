#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
/world/default/model/x500_dual_cam_0/link/down_camera_link/sensor/down_camera/image@sensor_msgs/msg/Image[gz.msgs.Image
