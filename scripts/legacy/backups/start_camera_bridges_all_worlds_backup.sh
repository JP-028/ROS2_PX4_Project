#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true

echo "[INFO] Stopping old camera bridges..."
pkill -f "ros_gz_bridge.*parameter_bridge" || true
pkill -f "parameter_bridge" || true
sleep 1

echo "[INFO] Searching Gazebo image topics..."

mapfile -t IMAGE_TOPICS < <(
  gz topic -l \
    | grep "^/world/" \
    | grep "/image$" \
    | sort -u
)

if [ "${#IMAGE_TOPICS[@]}" -eq 0 ]; then
  echo "[ERROR] No Gazebo image topics found."
  echo "[DEBUG] Available camera/image topics:"
  gz topic -l | grep -E "camera|image" || true
  exit 1
fi

echo "[INFO] Found image topics:"
printf '  %s\n' "${IMAGE_TOPICS[@]}"
echo

PIDS=()

for TOPIC in "${IMAGE_TOPICS[@]}"; do
  echo "[INFO] Starting image bridge:"
  echo "       $TOPIC"

  ros2 run ros_gz_bridge parameter_bridge \
    "$TOPIC@sensor_msgs/msg/Image@gz.msgs.Image" &

  PIDS+=("$!")
  sleep 0.5
done

echo
echo "[INFO] Started ${#PIDS[@]} image bridge process(es)."
echo "[INFO] Bridge PIDs: ${PIDS[*]}"
echo "[INFO] Keep this terminal open. Press Ctrl+C to stop."

trap 'echo "[INFO] Stopping camera bridges..."; kill "${PIDS[@]}" 2>/dev/null || true' INT TERM EXIT

wait
