#!/bin/bash
set -e

cd /home/user/ros2_ws

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run px4_offboard_control circle_flight_node
