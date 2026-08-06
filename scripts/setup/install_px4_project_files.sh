#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PX4_DIR="${PX4_DIR:-/home/user/PX4-Autopilot}"

AIRFRAME_NAME="4050_gz_x500_dual_cam"
AIRFRAME_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
AIRFRAME_LIST="$AIRFRAME_DIR/CMakeLists.txt"

GZ_DIR="$PX4_DIR/Tools/simulation/gz"
MODEL_DIR="$GZ_DIR/models"
WORLD_DIR="$GZ_DIR/worlds"

if [[ ! -d "$PX4_DIR" ]]; then
    echo "[ERROR] PX4 directory not found:"
    echo "        $PX4_DIR"
    echo
    echo "Set another location with:"
    echo "PX4_DIR=/path/to/PX4-Autopilot $0"
    exit 1
fi

if [[ ! -f "$AIRFRAME_LIST" ]]; then
    echo "[ERROR] PX4 airframe list not found:"
    echo "        $AIRFRAME_LIST"
    exit 1
fi

echo "[1/4] Installing custom PX4 airframe..."
install -m 755 \
    "$PROJECT_ROOT/$AIRFRAME_NAME" \
    "$AIRFRAME_DIR/$AIRFRAME_NAME"

if ! grep -qE "^[[:space:]]*$AIRFRAME_NAME[[:space:]]*$" "$AIRFRAME_LIST"; then
    sed -i \
        "/^[[:space:]]*4021_gz_x500_flow[[:space:]]*$/a\\     $AIRFRAME_NAME" \
        "$AIRFRAME_LIST"

    echo "      Added $AIRFRAME_NAME to CMakeLists.txt."
else
    echo "      Airframe is already registered."
fi

echo "[2/4] Installing stereo camera models..."
mkdir -p "$MODEL_DIR"
rm -rf \
    "$MODEL_DIR/mono_cam_left" \
    "$MODEL_DIR/mono_cam_right" \
    "$MODEL_DIR/x500_dual_cam"

cp -a "$PROJECT_ROOT/mono_cam_left" "$MODEL_DIR/"
cp -a "$PROJECT_ROOT/mono_cam_right" "$MODEL_DIR/"
cp -a "$PROJECT_ROOT/x500_dual_cam" "$MODEL_DIR/"

echo "[3/4] Installing the reproducible Gazebo world..."
mkdir -p "$WORLD_DIR"
install -m 644 \
    "$PROJECT_ROOT/simulation_assets/worlds/vio_marker_arena.sdf" \
    "$WORLD_DIR/vio_marker_arena.sdf"

echo "[4/4] Verifying installed files..."

required_paths=(
    "$AIRFRAME_DIR/$AIRFRAME_NAME"
    "$MODEL_DIR/mono_cam_left/model.sdf"
    "$MODEL_DIR/mono_cam_right/model.sdf"
    "$MODEL_DIR/x500_dual_cam/model.sdf"
    "$WORLD_DIR/vio_marker_arena.sdf"
)

for required_path in "${required_paths[@]}"; do
    if [[ ! -e "$required_path" ]]; then
        echo "[ERROR] Installation verification failed:"
        echo "        $required_path"
        exit 1
    fi
done

echo
echo "[SUCCESS] Project-specific PX4 files are installed."
echo "PX4 directory: $PX4_DIR"
