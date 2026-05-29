#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

timeout 10 ros2 topic hz /world/default/model/x500_dual_cam_0/link/down_camera_link/sensor/down_camera/image
