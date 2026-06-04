#!/bin/bash
set -e

ENV_NAME="$1"

if [ -z "$ENV_NAME" ]; then
  echo "[ERROR] No simulation environment selected."
  echo
  echo "Usage:"
  echo "  ./scripts/version_2/start_px4_gazebo_select_env.sh default"
  echo "  ./scripts/version_2/start_px4_gazebo_select_env.sh office_cpr"
  echo
  exit 1
fi

cd /home/user/ros2_ws/ROS2_PX4_Project

case "$ENV_NAME" in
  default)
    echo "[INFO] Starting default PX4 Gazebo dual-camera environment..."
    ./scripts/version_2/start_px4_gazebo_dual_cam.sh
    ;;

  office_cpr)
    echo "[INFO] Starting Office CPR PX4 Gazebo environment..."
    ./scripts/version_2/start_px4_gazebo_office_cpr_px4.sh
    ;;

  *)
    echo "[ERROR] Unknown environment: $ENV_NAME"
    echo
    echo "Available environments:"
    echo "  default"
    echo "  office_cpr"
    echo
    exit 1
    ;;
esac
