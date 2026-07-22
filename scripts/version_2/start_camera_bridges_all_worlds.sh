#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

echo "[INFO] Stopping old camera bridges..."
pkill -f "ros_gz_bridge.*parameter_bridge" || true
pkill -f "parameter_bridge" || true
sleep 1

echo "[INFO] Searching Gazebo stereo camera topics..."

LEFT_IMAGE_TOPIC=$(gz topic -l | grep "/camera_left_link/sensor/left_camera/image$" | head -n 1)
RIGHT_IMAGE_TOPIC=$(gz topic -l | grep "/camera_right_link/sensor/right_camera/image$" | head -n 1)

LEFT_INFO_TOPIC=$(gz topic -l | grep "/camera_left_link/sensor/left_camera/camera_info$" | head -n 1)
RIGHT_INFO_TOPIC=$(gz topic -l | grep "/camera_right_link/sensor/right_camera/camera_info$" | head -n 1)

if [ -z "$LEFT_IMAGE_TOPIC" ] || [ -z "$RIGHT_IMAGE_TOPIC" ]; then
    echo "[ERROR] Stereo image topics not found."
    gz topic -l | grep -E "left_camera|right_camera|image|camera_info" || true
    exit 1
fi

if [ -z "$LEFT_INFO_TOPIC" ] || [ -z "$RIGHT_INFO_TOPIC" ]; then
    echo "[ERROR] Stereo camera_info topics not found."
    gz topic -l | grep -E "left_camera|right_camera|camera_info" || true
    exit 1
fi

echo "[INFO] Left image:"
echo "       $LEFT_IMAGE_TOPIC"

echo "[INFO] Right image:"
echo "       $RIGHT_IMAGE_TOPIC"

echo "[INFO] Left camera info:"
echo "       $LEFT_INFO_TOPIC"

echo "[INFO] Right camera info:"
echo "       $RIGHT_INFO_TOPIC"

echo
echo "[INFO] Starting bridges..."

PIDS=()

ros2 run ros_gz_bridge parameter_bridge \
    "$LEFT_IMAGE_TOPIC@sensor_msgs/msg/Image@gz.msgs.Image" \
    --ros-args -r "$LEFT_IMAGE_TOPIC:=/stereo/left/image_raw" &
PIDS+=("$!")

ros2 run ros_gz_bridge parameter_bridge \
    "$RIGHT_IMAGE_TOPIC@sensor_msgs/msg/Image@gz.msgs.Image" \
    --ros-args -r "$RIGHT_IMAGE_TOPIC:=/stereo/right/image_raw" &
PIDS+=("$!")

ros2 run ros_gz_bridge parameter_bridge \
    "$LEFT_INFO_TOPIC@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
    --ros-args -r "$LEFT_INFO_TOPIC:=/stereo/left/camera_info" &
PIDS+=("$!")

ros2 run ros_gz_bridge parameter_bridge \
    "$RIGHT_INFO_TOPIC@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo" \
    --ros-args -r "$RIGHT_INFO_TOPIC:=/stereo/right/camera_info" &
PIDS+=("$!")

echo
echo "[INFO] Stereo camera bridges started."
echo "[INFO] ROS 2 topics:"
echo "       /stereo/left/image_raw"
echo "       /stereo/right/image_raw"
echo "       /stereo/left/camera_info"
echo "       /stereo/right/camera_info"

echo
echo "[INFO] Keep this terminal open. Press Ctrl+C to stop."

trap 'echo "[INFO] Stopping stereo camera bridges..."; kill "${PIDS[@]}" 2>/dev/null || true' INT TERM EXIT

wait
