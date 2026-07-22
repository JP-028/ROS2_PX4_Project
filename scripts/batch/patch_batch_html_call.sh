#!/usr/bin/env bash
set -e

PROJECT_DIR="/home/user/ros2_ws/ROS2_PX4_Project"
cd "$PROJECT_DIR"

TARGET="scripts/batch/run_batch_experiment.sh"

if [ ! -f "$TARGET" ]; then
  echo "[ERROR] Missing $TARGET"
  exit 1
fi

if grep -q "batch_create_html.py" "$TARGET"; then
  echo "[INFO] batch_create_html.py call already exists. No patch needed."
  exit 0
fi

python3 - <<'PY'
from pathlib import Path

p = Path("scripts/batch/run_batch_experiment.sh")
s = p.read_text()

old = '''python3 scripts/batch/batch_collect_summary.py "$EXPERIMENT_DIR" \\
  > "$SUMMARY_DIR/batch_collect_summary.log" 2>&1

cat "$SUMMARY_DIR/batch_collect_summary.log"
'''

new = '''python3 scripts/batch/batch_collect_summary.py "$EXPERIMENT_DIR" \\
  > "$SUMMARY_DIR/batch_collect_summary.log" 2>&1

cat "$SUMMARY_DIR/batch_collect_summary.log"

echo
echo "============================================================"
echo " Creating batch 3D HTML visualization"
echo "============================================================"

read -rp "Deviation warning threshold in percent of path size [default: 5]: " DEVIATION_THRESHOLD_PERCENT
DEVIATION_THRESHOLD_PERCENT="${DEVIATION_THRESHOLD_PERCENT:-5}"

python3 scripts/batch/batch_create_html.py "$EXPERIMENT_DIR" "$DEVIATION_THRESHOLD_PERCENT" \\
  > "$SUMMARY_DIR/batch_create_html.log" 2>&1

cat "$SUMMARY_DIR/batch_create_html.log"
'''

if old not in s:
    raise SystemExit("[ERROR] Could not find insertion point in run_batch_experiment.sh")

p.write_text(s.replace(old, new))
print("[OK] Patched run_batch_experiment.sh")
PY
