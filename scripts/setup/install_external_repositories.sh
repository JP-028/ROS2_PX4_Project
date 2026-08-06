#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$PROJECT_ROOT/scripts/setup/versions.env"

PX4_DIR="${PX4_DIR:-/home/user/PX4-Autopilot}"
ROS2_WS="${ROS2_WS:-/home/user/ros2_ws}"
PX4_MSGS_DIR="$ROS2_WS/src/px4_msgs"
AGENT_DIR="${AGENT_DIR:-/home/user/Micro-XRCE-DDS-Agent}"

checkout_repository() {
    local name="$1"
    local repository="$2"
    local commit="$3"
    local destination="$4"
    local recursive="${5:-false}"

    echo
    echo "=== $name ==="

    if [[ ! -d "$destination/.git" ]]; then
        echo "[INFO] Cloning into $destination..."

        if [[ "$recursive" == "true" ]]; then
            git clone --recursive "$repository" "$destination"
        else
            git clone "$repository" "$destination"
        fi

        git -C "$destination" checkout "$commit"

        if [[ "$recursive" == "true" ]]; then
            git -C "$destination" submodule update --init --recursive
        fi
    else
        echo "[INFO] Existing repository found:"
        echo "       $destination"
        echo "[INFO] Existing repositories are checked only."
    fi

    local installed_commit
    installed_commit="$(git -C "$destination" rev-parse HEAD)"

    if [[ "$installed_commit" != "$commit" ]]; then
        echo "[ERROR] Incorrect commit checked out for $name."
        echo "Expected: $commit"
        echo "Found:    $installed_commit"
        exit 1
    fi

    echo "[OK] $name is pinned to:"
    echo "     $installed_commit"
}

mkdir -p "$ROS2_WS/src"

checkout_repository \
    "PX4-Autopilot" \
    "$PX4_REPOSITORY" \
    "$PX4_COMMIT" \
    "$PX4_DIR" \
    "true"

checkout_repository \
    "px4_msgs" \
    "$PX4_MSGS_REPOSITORY" \
    "$PX4_MSGS_COMMIT" \
    "$PX4_MSGS_DIR"

checkout_repository \
    "Micro-XRCE-DDS-Agent" \
    "$MICRO_XRCE_AGENT_REPOSITORY" \
    "$MICRO_XRCE_AGENT_COMMIT" \
    "$AGENT_DIR" \
    "true"

echo
echo "=== Installing project-specific PX4 files ==="

PX4_DIR="$PX4_DIR" \
    "$PROJECT_ROOT/scripts/setup/install_px4_project_files.sh"

echo
echo "[SUCCESS] All external repositories are present at the pinned commits."
echo
echo "PX4-Autopilot:"
echo "  $PX4_DIR"
echo
echo "px4_msgs:"
echo "  $PX4_MSGS_DIR"
echo
echo "Micro-XRCE-DDS-Agent:"
echo "  $AGENT_DIR"
echo
echo "This script has not built or installed the software yet."
