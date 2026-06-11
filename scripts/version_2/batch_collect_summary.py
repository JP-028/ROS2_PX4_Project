#!/usr/bin/env python3
import csv
import json
import math
import statistics
import sys
from pathlib import Path
from datetime import datetime

def flatten_json(data, prefix=""):
    out = {}
    if isinstance(data, dict):
        for key, value in data.items():
            new_key = f"{prefix}.{key}" if prefix else str(key)
            out.update(flatten_json(value, new_key))
    elif isinstance(data, list):
        out[prefix] = json.dumps(data)
    else:
        out[prefix] = data
    return out

def is_number(value):
    if isinstance(value, bool):
        return False
    if isinstance(value, (int, float)):
        return math.isfinite(float(value))
    try:
        float(value)
        return True
    except Exception:
        return False

def main():
    if len(sys.argv) != 2:
        print("Usage: batch_collect_summary.py <experiment_dir>")
        sys.exit(1)

    experiment_dir = Path(sys.argv[1]).resolve()
    runs_dir = experiment_dir / "runs"
    summary_dir = experiment_dir / "summary"
    summary_dir.mkdir(parents=True, exist_ok=True)

    if not runs_dir.exists():
        print(f"[ERROR] No runs directory found: {runs_dir}")
        sys.exit(1)

    rows = []

    for run_dir in sorted(runs_dir.glob("run_*")):
        report_path = run_dir / "path_analysis" / "path_analysis_report.json"

        if not report_path.exists():
            rows.append({
                "run": run_dir.name,
                "status": "missing_report",
                "run_dir": str(run_dir),
            })
            continue

        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
            flat = flatten_json(report)
            flat["run"] = run_dir.name
            flat["status"] = "ok"
            flat["run_dir"] = str(run_dir)
            flat["report_path"] = str(report_path)
            rows.append(flat)
        except Exception as exc:
            rows.append({
                "run": run_dir.name,
                "status": f"read_error: {exc}",
                "run_dir": str(run_dir),
                "report_path": str(report_path),
            })

    if not rows:
        print("[ERROR] No runs found.")
        sys.exit(1)

    all_keys = sorted({key for row in rows for key in row.keys()})
    preferred = ["run", "status", "run_dir", "report_path"]
    ordered_keys = preferred + [k for k in all_keys if k not in preferred]

    batch_summary_csv = summary_dir / "batch_summary.csv"
    with batch_summary_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=ordered_keys)
        writer.writeheader()
        writer.writerows(rows)

    numeric_keys = []
    ok_rows = [r for r in rows if r.get("status") == "ok"]
    for key in ordered_keys:
        numeric_values = [float(r[key]) for r in ok_rows if key in r and is_number(r[key])]
        if numeric_values:
            numeric_keys.append(key)

    stats_rows = []
    for key in numeric_keys:
        vals = [float(r[key]) for r in ok_rows if key in r and is_number(r[key])]
        stats_rows.append({
            "metric": key,
            "count": len(vals),
            "mean": statistics.mean(vals),
            "std": statistics.stdev(vals) if len(vals) > 1 else 0.0,
            "min": min(vals),
            "max": max(vals),
            "median": statistics.median(vals),
        })

    numeric_stats_csv = summary_dir / "numeric_stats.csv"
    with numeric_stats_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["metric", "count", "mean", "std", "min", "max", "median"])
        writer.writeheader()
        writer.writerows(stats_rows)

    ok_count = len(ok_rows)
    failed_count = len(rows) - ok_count

    txt_path = summary_dir / "batch_report.txt"
    with txt_path.open("w", encoding="utf-8") as f:
        f.write("Batch Experiment Summary\n")
        f.write("========================\n\n")
        f.write(f"Created: {datetime.now().isoformat(timespec='seconds')}\n")
        f.write(f"Experiment directory: {experiment_dir}\n")
        f.write(f"Total runs: {len(rows)}\n")
        f.write(f"Successful runs: {ok_count}\n")
        f.write(f"Failed/missing runs: {failed_count}\n\n")
        f.write("Generated files:\n")
        f.write(f"- {batch_summary_csv}\n")
        f.write(f"- {numeric_stats_csv}\n")
        f.write(f"- {txt_path}\n\n")
        f.write("Numeric metrics:\n")
        f.write("----------------\n")
        if not stats_rows:
            f.write("No numeric metrics found.\n")
        else:
            for row in stats_rows:
                f.write(
                    f"{row['metric']}:\n"
                    f"  count  = {row['count']}\n"
                    f"  mean   = {row['mean']:.6f}\n"
                    f"  std    = {row['std']:.6f}\n"
                    f"  min    = {row['min']:.6f}\n"
                    f"  max    = {row['max']:.6f}\n"
                    f"  median = {row['median']:.6f}\n\n"
                )
        if failed_count:
            f.write("\nRuns with problems:\n")
            f.write("-------------------\n")
            for row in rows:
                if row.get("status") != "ok":
                    f.write(f"- {row.get('run')}: {row.get('status')}\n")

    print("[OK] Batch summary created:")
    print(f"     {batch_summary_csv}")
    print(f"     {numeric_stats_csv}")
    print(f"     {txt_path}")

if __name__ == "__main__":
    main()
