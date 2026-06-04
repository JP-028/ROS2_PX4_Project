#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

echo "[INFO] Gazebo image topics:"
gz topic -l | grep "^/world/" | grep "/image$" | sort -u || true

echo
echo "[INFO] ROS image topics:"
ROS_TOPICS=$(ROS2CLI_DISABLE_DAEMON=1 ros2 topic list | grep "^/world/" | grep "/image$" | sort -u || true)

if [ -z "$ROS_TOPICS" ]; then
  echo "[ERROR] No ROS image topics found."
  exit 1
fi

echo "$ROS_TOPICS"
echo

while IFS= read -r TOPIC; do
  [ -z "$TOPIC" ] && continue
  echo "----------------------------------------"
  echo "[INFO] Checking:"
  echo "$TOPIC"
  timeout 5 ros2 topic hz "$TOPIC" || true
done <<< "$ROS_TOPICS"
