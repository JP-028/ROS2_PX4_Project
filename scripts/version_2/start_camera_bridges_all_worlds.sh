#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

echo "[INFO] Stopping old parameter bridges..."
pkill -f "ros_gz_bridge.*parameter_bridge" || true
sleep 1

echo "[INFO] Searching all Gazebo image camera topics..."

TOPICS=$(gz topic -l | grep "/sensor/" | grep "/image" | grep "/camera" | sort -u)

if [ -z "$TOPICS" ]; then
  echo "[ERROR] No Gazebo camera image topics found."
  echo "[DEBUG] Available camera/image topics:"
  gz topic -l | grep -E "camera|image" || true
  exit 1
fi

echo "[INFO] Found camera image topics:"
echo "$TOPICS"
echo

BRIDGE_ARGS=""

while IFS= read -r TOPIC; do
  [ -z "$TOPIC" ] && continue
  echo "[INFO] Adding bridge for:"
  echo "       $TOPIC"
  BRIDGE_ARGS="$BRIDGE_ARGS $TOPIC@sensor_msgs/msg/Image@gz.msgs.Image"
done <<< "$TOPICS"

echo
echo "[INFO] Starting ros_gz_bridge parameter_bridge for all found camera image topics..."
echo "[INFO] Keep this terminal open. Press Ctrl+C to stop."

ros2 run ros_gz_bridge parameter_bridge $BRIDGE_ARGS
