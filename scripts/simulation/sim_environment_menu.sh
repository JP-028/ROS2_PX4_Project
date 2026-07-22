#!/bin/bash
set -e

PROJECT_DIR="/home/user/ros2_ws/ROS2_PX4_Project"
PX4_DIR="/home/user/PX4-Autopilot"
WORLD_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
ASSET_DIR="/home/user/sim_assets/gazebo_models_worlds_collection"

cd "$PROJECT_DIR"

if [ ! -d "$WORLD_DIR" ]; then
  echo "[ERROR] PX4 Gazebo world directory not found:"
  echo "$WORLD_DIR"
  exit 1
fi

mapfile -t WORLD_FILES < <(find "$WORLD_DIR" -maxdepth 1 -type f -name "*.sdf" | sort)

if [ "${#WORLD_FILES[@]}" -eq 0 ]; then
  echo "[ERROR] No PX4 Gazebo .sdf worlds found in:"
  echo "$WORLD_DIR"
  exit 1
fi

echo
echo "Available PX4 Gazebo environments:"
echo

for i in "${!WORLD_FILES[@]}"; do
  FILE="${WORLD_FILES[$i]}"
  BASENAME="$(basename "$FILE")"
  ENV_NAME="${BASENAME%.sdf}"

  if [ "$ENV_NAME" = "office_cpr_px4" ]; then
    LABEL="Office CPR custom environment"
  elif [ "$ENV_NAME" = "default" ]; then
    LABEL="Default PX4 dual-camera environment"
  else
    LABEL="PX4 Gazebo world"
  fi

  printf "  %2d) %-25s %s\n" "$((i+1))" "$ENV_NAME" "$LABEL"
done

echo
read -rp "Select environment number: " SELECTION

if ! [[ "$SELECTION" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] Selection must be a number."
  exit 1
fi

INDEX=$((SELECTION-1))

if [ "$INDEX" -lt 0 ] || [ "$INDEX" -ge "${#WORLD_FILES[@]}" ]; then
  echo "[ERROR] Invalid selection: $SELECTION"
  exit 1
fi

SELECTED_FILE="${WORLD_FILES[$INDEX]}"
ENV_NAME="$(basename "$SELECTED_FILE" .sdf)"

echo
echo "[INFO] Selected environment:"
echo "       $ENV_NAME"
echo "[INFO] World file:"
echo "       $SELECTED_FILE"
echo

export DISPLAY=host.docker.internal:0
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_RENDER_ENGINE=ogre

export GZ_SIM_RESOURCE_PATH=$ASSET_DIR:$ASSET_DIR/models:$ASSET_DIR/worlds:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH

export PX4_GZ_WORLD="$ENV_NAME"
export PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-0,-5.5,0.15,0,0,-1.5708}"

cd "$PX4_DIR"

echo "[INFO] Starting PX4 SITL + Gazebo with gz_x500_dual_cam..."
echo "[INFO] Keep this terminal open."
echo

PX4_SYS_AUTOSTART=4050 make px4_sitl gz_x500_dual_cam
