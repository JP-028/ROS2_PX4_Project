#!/bin/bash

echo "Stopping ROS2 / PX4 / Gazebo v2 processes..."

pkill -f keyboard_cmd_vel_node || true
pkill -f circle_flight_node || true
pkill -f square_flight_node || true
pkill -f cmd_vel_offboard_node || true

pkill -f ros_gz_bridge || true
pkill -f ros_gz_image || true
pkill -f parameter_bridge || true
pkill -f rqt_image_view || true

pkill -f MicroXRCEAgent || true

pkill -f px4 || true
pkill -f gz || true
pkill -f gazebo || true
pkill -f make || true

echo "Done."
