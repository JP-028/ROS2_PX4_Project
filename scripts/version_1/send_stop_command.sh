#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -1
