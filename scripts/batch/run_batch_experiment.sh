#!/usr/bin/env bash

PROJECT_DIR="/home/user/ros2_ws/ROS2_PX4_Project"
cd "$PROJECT_DIR" || exit 1

# ROS setup scripts may reference unset variables internally.
# Therefore strict unset-variable checking is enabled only after sourcing.
set +u
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash 2>/dev/null || true
set -u

echo
echo "============================================================"
echo " Optional Batch Experiment Runner"
echo "============================================================"
echo "This script runs the same YAML flight path multiple times."
echo "It is opt-in only. Nothing is recorded unless you choose it here."
echo

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found. ROS2 environment is not sourced."
  exit 1
fi

if [ ! -d "path_specs" ]; then
  echo "[ERROR] path_specs directory not found."
  exit 1
fi

echo "[CHECK] Checking /uav/local_pose..."
if ! timeout 4s ros2 topic echo /uav/local_pose --once >/tmp/batch_local_pose_check.txt 2>/tmp/batch_local_pose_check.err; then
  echo "[ERROR] No /uav/local_pose received."
  echo "Start first in another terminal:"
  echo "  ./scripts/control/start_local_pose_from_px4.sh"
  exit 1
fi
echo "[OK] /uav/local_pose is available."

# Only show YAML files that are actually compatible with the current executor.
# The current batch executor uses execute_path_from_local_pose.py and therefore expects:
#   type: segment_path
mapfile -t YAML_FILES < <(
  find path_specs -maxdepth 1 -type f -name "*.yaml" | sort | while read -r yaml_file; do
    if grep -Eq '^type:[[:space:]]*segment_path[[:space:]]*$' "$yaml_file"; then
      echo "$yaml_file"
    fi
  done
)

if [ "${#YAML_FILES[@]}" -eq 0 ]; then
  echo "[ERROR] No compatible segment_path YAML files found in path_specs/"
  echo
  echo "This batch runner only supports YAML files with:"
  echo "  type: segment_path"
  echo
  exit 1
fi

echo
echo "Available YAML flight paths:"
echo

for i in "${!YAML_FILES[@]}"; do
  idx=$((i + 1))
  name="$(basename "${YAML_FILES[$i]}" .yaml)"
  printf "  %2d) %-30s %s\n" "$idx" "$name" "${YAML_FILES[$i]}"
done

echo
read -rp "Select path number: " SELECTED

if ! [[ "$SELECTED" =~ ^[0-9]+$ ]]; then
  echo "[ERROR] Invalid number."
  exit 1
fi

SELECTED_INDEX=$((SELECTED - 1))

if [ "$SELECTED_INDEX" -lt 0 ] || [ "$SELECTED_INDEX" -ge "${#YAML_FILES[@]}" ]; then
  echo "[ERROR] Selection out of range."
  exit 1
fi

YAML_FILE="${YAML_FILES[$SELECTED_INDEX]}"
PATH_NAME="$(basename "$YAML_FILE" .yaml)"

echo
echo "[INFO] Selected YAML: $YAML_FILE"
echo

read -rp "Number of runs [default: 3]: " RUN_COUNT
RUN_COUNT="${RUN_COUNT:-3}"

if ! [[ "$RUN_COUNT" =~ ^[0-9]+$ ]] || [ "$RUN_COUNT" -lt 1 ]; then
  echo "[ERROR] Run count must be a positive integer."
  exit 1
fi

read -rp "Record camera/IMU rosbag data? This can become very large. [y/N]: " RECORD_BAG_ANSWER
RECORD_BAG_ANSWER="${RECORD_BAG_ANSWER:-N}"

RECORD_BAG="false"
if [[ "$RECORD_BAG_ANSWER" =~ ^[Yy]$ ]]; then
  RECORD_BAG="true"
fi

read -rp "Optional note for this experiment [press Enter to skip]: " EXP_NOTE

WORLD_NAME="$(gz topic -l 2>/dev/null | sed -n 's#^/world/\([^/]*\)/.*#\1#p' | sort -u | head -n 1)"
WORLD_NAME="${WORLD_NAME:-unknown_world}"

TIMESTAMP="$(date +"%Y-%m-%d_%H-%M-%S")"
EXPERIMENT_NAME="${TIMESTAMP}_${WORLD_NAME}_${PATH_NAME}"
EXPERIMENT_DIR="$PROJECT_DIR/batch_experiments/$EXPERIMENT_NAME"

RUNS_DIR="$EXPERIMENT_DIR/runs"
SUMMARY_DIR="$EXPERIMENT_DIR/summary"
CONFIG_DIR="$EXPERIMENT_DIR/config"

mkdir -p "$RUNS_DIR" "$SUMMARY_DIR" "$CONFIG_DIR"

cp "$YAML_FILE" "$CONFIG_DIR/path_spec_used.yaml"

cat > "$CONFIG_DIR/experiment_metadata.txt" <<EOF
Experiment name: $EXPERIMENT_NAME
Created: $TIMESTAMP
World: $WORLD_NAME
Path name: $PATH_NAME
YAML file: $YAML_FILE
Run count: $RUN_COUNT
Record bag: $RECORD_BAG
Note: $EXP_NOTE
EOF

cat > "$CONFIG_DIR/experiment_metadata.json" <<EOF
{
  "experiment_name": "$EXPERIMENT_NAME",
  "created": "$TIMESTAMP",
  "world": "$WORLD_NAME",
  "path_name": "$PATH_NAME",
  "yaml_file": "$YAML_FILE",
  "run_count": $RUN_COUNT,
  "record_bag": $RECORD_BAG,
  "note": "$(echo "$EXP_NOTE" | sed 's/"/\\"/g')"
}
EOF

echo
echo "============================================================"
echo " Experiment configuration"
echo "============================================================"
echo "World:       $WORLD_NAME"
echo "Path:        $PATH_NAME"
echo "YAML:        $YAML_FILE"
echo "Runs:        $RUN_COUNT"
echo "Record bag:  $RECORD_BAG"
echo "Output:      $EXPERIMENT_DIR"
echo "============================================================"
echo

read -rp "Start batch experiment now? [y/N]: " START_ANSWER
START_ANSWER="${START_ANSWER:-N}"

if ! [[ "$START_ANSWER" =~ ^[Yy]$ ]]; then
  echo "[INFO] Cancelled. No batch experiment was started."
  exit 0
fi

collect_bag_topics() {
  local topic_list
  topic_list="$(ros2 topic list 2>/dev/null || true)"
  {
    echo "$topic_list" | grep -E '^/uav/local_pose$' || true
    echo "$topic_list" | grep -E '^/cmd_vel$' || true
    echo "$topic_list" | grep -E '^/fmu/out/vehicle_odometry$' || true
    echo "$topic_list" | grep -E '^/fmu/out/vehicle_status$' || true
    echo "$topic_list" | grep -E '^/fmu/out/sensor_combined$' || true
    echo "$topic_list" | grep -E '^/fmu/out/vehicle_imu$' || true
    echo "$topic_list" | grep -E '^/fmu/out/vehicle_imu_status$' || true
    echo "$topic_list" | grep -E 'image$' || true
    echo "$topic_list" | grep -E 'camera_info$' || true
  } | sort -u
}

for run_idx in $(seq 1 "$RUN_COUNT"); do
  RUN_ID="$(printf "run_%03d" "$run_idx")"
  RUN_DIR="$RUNS_DIR/$RUN_ID"

  mkdir -p "$RUN_DIR/logs"

  echo
  echo "============================================================"
  echo " Starting $RUN_ID / $RUN_COUNT"
  echo "============================================================"

  date +"%Y-%m-%d_%H-%M-%S" > "$RUN_DIR/start_time.txt"

  BAG_PID=""
  if [ "$RECORD_BAG" = "true" ]; then
    mkdir -p "$RUN_DIR/rosbag"

    mapfile -t BAG_TOPICS < <(collect_bag_topics)

    if [ "${#BAG_TOPICS[@]}" -eq 0 ]; then
      echo "[WARN] No rosbag topics found. Skipping rosbag for this run."
    else
      echo "[INFO] Recording rosbag topics:"
      printf '       %s\n' "${BAG_TOPICS[@]}" | tee "$RUN_DIR/logs/rosbag_topics.txt"

      ros2 bag record \
        -o "$RUN_DIR/rosbag/sensor_recording" \
        "${BAG_TOPICS[@]}" \
        > "$RUN_DIR/logs/rosbag.log" 2>&1 &

      BAG_PID=$!
      echo "[INFO] rosbag PID: $BAG_PID"
      sleep 2
    fi
  fi

  echo "[INFO] Starting analyzer..."
  python3 scripts/analysis/analyze_path_from_spec.py "$YAML_FILE" \
    > "$RUN_DIR/logs/analyzer.log" 2>&1 &

  ANALYZER_PID=$!
  echo "[INFO] Analyzer PID: $ANALYZER_PID"
  sleep 3

  echo "[INFO] Executing flight..."
  python3 scripts/flight/execute_path_from_local_pose.py "$YAML_FILE" \
    > "$RUN_DIR/logs/flight.log" 2>&1

  FLIGHT_EXIT_CODE=$?
  echo "$FLIGHT_EXIT_CODE" > "$RUN_DIR/flight_exit_code.txt"

  if [ "$FLIGHT_EXIT_CODE" -ne 0 ]; then
    echo "[WARN] Flight exited with code $FLIGHT_EXIT_CODE. Check:"
    echo "       $RUN_DIR/logs/flight.log"
  else
    echo "[OK] Flight finished."
  fi

  echo "[INFO] Stopping analyzer..."
  if kill -0 "$ANALYZER_PID" >/dev/null 2>&1; then
    kill -SIGINT "$ANALYZER_PID" >/dev/null 2>&1 || true
    wait "$ANALYZER_PID" >/dev/null 2>&1 || true
  fi

  sleep 2

  if [ -n "$BAG_PID" ]; then
    echo "[INFO] Stopping rosbag..."
    if kill -0 "$BAG_PID" >/dev/null 2>&1; then
      kill -SIGINT "$BAG_PID" >/dev/null 2>&1 || true
      wait "$BAG_PID" >/dev/null 2>&1 || true
    fi
  fi

  LATEST_ANALYSIS="$(find "$PROJECT_DIR/path_analysis" -maxdepth 1 -mindepth 1 -type d 2>/dev/null | sort | tail -n 1 || true)"

  if [ -n "$LATEST_ANALYSIS" ] && [ -d "$LATEST_ANALYSIS" ]; then
    echo "[INFO] Moving analysis output:"
    echo "       $LATEST_ANALYSIS"
    echo "       -> $RUN_DIR/path_analysis"

    rm -rf "$RUN_DIR/path_analysis"
    mv "$LATEST_ANALYSIS" "$RUN_DIR/path_analysis"
  else
    echo "[WARN] No path_analysis output folder found for $RUN_ID."
    echo "missing" > "$RUN_DIR/path_analysis_status.txt"
  fi

  date +"%Y-%m-%d_%H-%M-%S" > "$RUN_DIR/end_time.txt"
  echo "[OK] Finished $RUN_ID"
done

echo
echo "============================================================"
echo " Creating batch summary"
echo "============================================================"

python3 scripts/batch/batch_collect_summary.py "$EXPERIMENT_DIR" \
  > "$SUMMARY_DIR/batch_collect_summary.log" 2>&1

cat "$SUMMARY_DIR/batch_collect_summary.log"

echo
echo "============================================================"
echo " Creating batch 3D HTML visualization"
echo "============================================================"

read -rp "Deviation warning threshold in percent of path size [default: 5]: " DEVIATION_THRESHOLD_PERCENT
DEVIATION_THRESHOLD_PERCENT="${DEVIATION_THRESHOLD_PERCENT:-5}"

python3 scripts/batch/batch_create_html.py "$EXPERIMENT_DIR" "$DEVIATION_THRESHOLD_PERCENT" \
  > "$SUMMARY_DIR/batch_create_html.log" 2>&1

cat "$SUMMARY_DIR/batch_create_html.log"

echo
echo "============================================================"
echo " Batch experiment finished"
echo "============================================================"
echo "Experiment folder:"
echo "$EXPERIMENT_DIR"
echo
echo "Important files:"
echo "$SUMMARY_DIR/batch_report.txt"
echo "$SUMMARY_DIR/batch_summary.csv"
echo "$SUMMARY_DIR/numeric_stats.csv"
echo
