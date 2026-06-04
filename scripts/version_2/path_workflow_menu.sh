#!/bin/bash
set -e

PROJECT_DIR="/home/user/ros2_ws/ROS2_PX4_Project"
SPEC_DIR="$PROJECT_DIR/path_specs"

cd "$PROJECT_DIR"

if [ ! -d "$SPEC_DIR" ]; then
  echo "[ERROR] path_specs folder not found:"
  echo "$SPEC_DIR"
  exit 1
fi

mapfile -t YAML_FILES < <(find "$SPEC_DIR" -maxdepth 1 -type f \( -name "*.yaml" -o -name "*.yml" \) | sort)

if [ "${#YAML_FILES[@]}" -eq 0 ]; then
  echo "[ERROR] No YAML files found in:"
  echo "$SPEC_DIR"
  exit 1
fi

echo
echo "Available YAML flight paths:"
echo

for i in "${!YAML_FILES[@]}"; do
  FILE="${YAML_FILES[$i]}"
  BASENAME="$(basename "$FILE")"
  NAME="${BASENAME%.*}"

  YAML_NAME=$(python3 - "$FILE" <<'PY'
import sys
try:
    import yaml
    with open(sys.argv[1], "r") as f:
        data = yaml.safe_load(f) or {}
    print(data.get("name", ""))
except Exception:
    print("")
PY
)

  if [ -n "$YAML_NAME" ]; then
    DISPLAY_NAME="$YAML_NAME"
  else
    DISPLAY_NAME="$NAME"
  fi

  printf "  %2d) %-30s %s\n" "$((i+1))" "$DISPLAY_NAME" "path_specs/$BASENAME"
done

echo
read -rp "Select path number: " SELECTION

if ! [[ "$SELECTION" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] Selection must be a number."
  exit 1
fi

INDEX=$((SELECTION-1))

if [ "$INDEX" -lt 0 ] || [ "$INDEX" -ge "${#YAML_FILES[@]}" ]; then
  echo "[ERROR] Invalid selection: $SELECTION"
  exit 1
fi

SELECTED_FILE="${YAML_FILES[$INDEX]}"
REL_SELECTED_FILE="path_specs/$(basename "$SELECTED_FILE")"

echo
echo "[INFO] Selected YAML:"
echo "       $REL_SELECTED_FILE"
echo

echo "What do you want to do?"
echo
echo "  1) Start analyzer only"
echo "  2) Execute flight only"
echo "  3) Show required startup order"
echo "  4) Show YAML file content"
echo
read -rp "Select action number: " ACTION

case "$ACTION" in
  1)
    echo
    echo "[INFO] Starting analyzer for:"
    echo "       $REL_SELECTED_FILE"
    echo
    echo "[INFO] In another terminal, start this menu again, select the same YAML, and choose:"
    echo "       Execute flight only"
    echo
    ./scripts/version_2/start_path_analysis.sh "$REL_SELECTED_FILE"
    ;;

  2)
    echo
    echo "[INFO] Executing flight for:"
    echo "       $REL_SELECTED_FILE"
    echo
    ./scripts/version_2/start_path_flight.sh "$REL_SELECTED_FILE"
    ;;

  3)
    echo
    echo "Required startup order:"
    echo
    echo "Terminal 1:"
    echo "  ./scripts/version_2/start_agent.sh"
    echo
    echo "Terminal 2:"
    echo "  ./scripts/version_2/sim_environment_menu.sh"
    echo
    echo "Terminal 3:"
    echo "  ./scripts/version_2/start_camera_bridges_all_worlds.sh"
    echo
    echo "Terminal 4:"
    echo "  ./scripts/version_2/start_tf_broadcaster.sh"
    echo
    echo "Terminal 5:"
    echo "  ./scripts/version_2/start_offboard_control.sh"
    echo
    echo "Terminal 6:"
    echo "  ./scripts/version_2/path_workflow_menu.sh"
    echo "  Select YAML -> Start analyzer only"
    echo
    echo "Terminal 7:"
    echo "  ./scripts/version_2/path_workflow_menu.sh"
    echo "  Select same YAML -> Execute flight only"
    echo
    ;;

  4)
    echo
    echo "----- $REL_SELECTED_FILE -----"
    cat "$SELECTED_FILE"
    echo
    ;;

  *)
    echo "[ERROR] Unknown action: $ACTION"
    exit 1
    ;;
esac
