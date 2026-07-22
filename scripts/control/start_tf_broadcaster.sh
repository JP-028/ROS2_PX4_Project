#!/bin/bash
set -e

cd /home/user/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run px4_offboard_control uav_tf_broadcaster_node
