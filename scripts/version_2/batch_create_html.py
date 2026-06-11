#!/usr/bin/env python3
import csv
import json
import math
import statistics
import sys
from pathlib import Path


def _clean_name(name: str) -> str:
    return name.strip().lower().replace(" ", "_").replace("-", "_")


def _is_time_like(name: str) -> bool:
    n = _clean_name(name)
    time_tokens = [
        "time", "timestamp", "stamp", "elapsed", "duration",
        "sample", "index", "seq", "frame", "nanosec", "nsec", "sec"
    ]
    return any(tok in n for tok in time_tokens)


def _is_numeric(value) -> bool:
    try:
        v = float(value)
        return math.isfinite(v)
    except Exception:
        return False


def _numeric_columns(rows, fields):
    numeric_cols = []
    for f in fields:
        count = 0
        for row in rows[:20]:
            if _is_numeric(row.get(f, "")):
                count += 1
        if count > 0:
            numeric_cols.append(f)
    return numeric_cols


def _pick_axis_column(fields, axis):
    """
    Robustly pick x/y/z columns from different possible CSV header styles.

    Supported examples:
      x, y, z
      pos_x, pos_y, pos_z
      position_x, position_y, position_z
      position_x_m, position_y_m, position_z_m
      actual_x, actual_y, actual_z
      ideal_x, ideal_y, ideal_z
      odom_x, odom_y, odom_z
      local_x, local_y, local_z
    """
    axis = axis.lower()
    cleaned = {f: _clean_name(f) for f in fields}

    exact_candidates = [
        axis,
        f"{axis}_m",
        f"{axis}_meter",
        f"pos_{axis}",
        f"pos_{axis}_m",
        f"position_{axis}",
        f"position_{axis}_m",
        f"actual_{axis}",
        f"actual_{axis}_m",
        f"ideal_{axis}",
        f"ideal_{axis}_m",
        f"odom_{axis}",
        f"odom_{axis}_m",
        f"local_{axis}",
        f"local_{axis}_m",
        f"world_{axis}",
        f"world_{axis}_m",
        f"px4_{axis}",
        f"px4_{axis}_m",
    ]

    for f, n in cleaned.items():
        if n in exact_candidates:
            return f

    # Match names ending in _x, _y, _z or containing position_x etc.
    suffix_patterns = [
        f"_{axis}",
        f"_{axis}_m",
        f"_{axis}_meter",
    ]
    semantic_tokens = ["pos", "position", "actual", "ideal", "odom", "local", "world", "px4"]

    for f, n in cleaned.items():
        if _is_time_like(n):
            continue
        if any(n.endswith(pat) for pat in suffix_patterns):
            return f
        if any(tok in n for tok in semantic_tokens) and f"_{axis}" in n:
            return f

    return None


def read_csv_points(path: Path):
    if not path.exists():
        return []

    with path.open("r", encoding="utf-8", errors="replace", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            return []

        fields = [c.strip() for c in reader.fieldnames]
        rows = list(reader)

    if not rows:
        return []

    x_key = _pick_axis_column(fields, "x")
    y_key = _pick_axis_column(fields, "y")
    z_key = _pick_axis_column(fields, "z")

    # Fallback: choose first 2 or 3 numeric non-time columns.
    # This prevents using elapsed/time as x, which caused huge fake paths.
    if x_key is None or y_key is None:
        numeric_cols = _numeric_columns(rows, fields)
        non_time_numeric = [c for c in numeric_cols if not _is_time_like(c)]

        if len(non_time_numeric) >= 2:
            x_key = non_time_numeric[0]
            y_key = non_time_numeric[1]
            z_key = non_time_numeric[2] if len(non_time_numeric) >= 3 else None
        else:
            print(f"[WARN] Could not identify x/y columns in {path}")
            print(f"[WARN] Header was: {fields}")
            return []

    print(f"[INFO] Reading {path.name}: x='{x_key}', y='{y_key}', z='{z_key}'")

    pts = []
    for row in rows:
        try:
            x = float(row[x_key])
            y = float(row[y_key])
            z = float(row[z_key]) if z_key and row.get(z_key, "") not in ("", None) else 0.0
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                pts.append([x, y, z])
        except Exception:
            continue

    return pts


def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def cumulative_s(points):
    if not points:
        return []
    s = [0.0]
    for i in range(1, len(points)):
        s.append(s[-1] + dist(points[i - 1], points[i]))
    return s


def interp_polyline(points, n=300):
    if not points:
        return []

    if len(points) == 1:
        return [points[0][:] for _ in range(n)]

    s = cumulative_s(points)
    total = s[-1]

    if total <= 1e-9:
        return [points[0][:] for _ in range(n)]

    out = []
    j = 0

    for k in range(n):
        target = total * k / max(n - 1, 1)

        while j < len(s) - 2 and s[j + 1] < target:
            j += 1

        s0 = s[j]
        s1 = s[j + 1]
        p0 = points[j]
        p1 = points[j + 1]

        alpha = 0.0 if abs(s1 - s0) < 1e-12 else (target - s0) / (s1 - s0)

        out.append([
            p0[0] + alpha * (p1[0] - p0[0]),
            p0[1] + alpha * (p1[1] - p0[1]),
            p0[2] + alpha * (p1[2] - p0[2]),
        ])

    return out


def nearest_dist_to_polyline(point, ref_points):
    if not ref_points:
        return None
    return min(dist(point, p) for p in ref_points)


def path_length(points):
    if len(points) < 2:
        return 0.0
    return cumulative_s(points)[-1]


def characteristic_size(points):
    if not points:
        return 1.0

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    dx = max(xs) - min(xs) if xs else 0.0
    dy = max(ys) - min(ys) if ys else 0.0
    dz = max(zs) - min(zs) if zs else 0.0
    size = max(dx, dy, dz)

    if size <= 1e-6:
        size = max(path_length(points), 1.0)

    return size


def split_bad_segments(points, bad_flags):
    segments = []
    current = []

    for p, bad in zip(points, bad_flags):
        if bad:
            current.append(p)
        else:
            if len(current) >= 2:
                segments.append(current)
            current = []

    if len(current) >= 2:
        segments.append(current)

    return segments


def js_trace(name, points, mode="lines", color=None, width=4, dash=None, visible=True, opacity=1.0):
    line = {"width": width}
    if color:
        line["color"] = color
    if dash:
        line["dash"] = dash

    return {
        "type": "scatter3d",
        "mode": mode,
        "name": name,
        "x": [p[0] for p in points],
        "y": [p[1] for p in points],
        "z": [p[2] for p in points],
        "line": line,
        "opacity": opacity,
        "visible": True if visible else "legendonly",
    }


def js_marker(name, point, color=None, size=5):
    marker = {"size": size}
    if color:
        marker["color"] = color

    return {
        "type": "scatter3d",
        "mode": "markers",
        "name": name,
        "x": [point[0]],
        "y": [point[1]],
        "z": [point[2]],
        "marker": marker,
    }


def main():
    if len(sys.argv) < 2:
        print("Usage: batch_create_html.py <experiment_dir> [threshold_percent]")
        sys.exit(1)

    experiment_dir = Path(sys.argv[1]).resolve()
    threshold_percent = float(sys.argv[2]) if len(sys.argv) >= 3 else 5.0

    runs_dir = experiment_dir / "runs"
    summary_dir = experiment_dir / "summary"
    summary_dir.mkdir(parents=True, exist_ok=True)

    if not runs_dir.exists():
        print(f"[ERROR] Missing runs directory: {runs_dir}")
        sys.exit(1)

    run_data = []
    ideal_points = []

    for run_dir in sorted(runs_dir.glob("run_*")):
        analysis_dir = run_dir / "path_analysis"
        actual_csv = analysis_dir / "actual_path.csv"
        ideal_csv = analysis_dir / "ideal_path.csv"

        actual = read_csv_points(actual_csv)
        ideal = read_csv_points(ideal_csv)

        if actual:
            run_data.append({
                "name": run_dir.name,
                "dir": str(run_dir),
                "actual": actual,
                "actual_length": path_length(actual),
            })

        if not ideal_points and ideal:
            ideal_points = ideal

    if not run_data:
        print("[ERROR] No actual_path.csv data found in runs.")
        sys.exit(1)

    if not ideal_points:
        print("[WARN] No ideal_path.csv found. Average path will be shown without deviation-to-ideal coloring.")

    sample_count = 300
    resampled_runs = [interp_polyline(r["actual"], sample_count) for r in run_data]

    average_path = []
    for i in range(sample_count):
        xs = [run[i][0] for run in resampled_runs if len(run) > i]
        ys = [run[i][1] for run in resampled_runs if len(run) > i]
        zs = [run[i][2] for run in resampled_runs if len(run) > i]
        average_path.append([
            statistics.mean(xs),
            statistics.mean(ys),
            statistics.mean(zs),
        ])

    ideal_resampled = interp_polyline(ideal_points, sample_count) if ideal_points else []
    char_size = characteristic_size(ideal_points if ideal_points else average_path)
    threshold_m = char_size * (threshold_percent / 100.0)

    average_deviation = []
    bad_flags = []

    if ideal_resampled:
        for p in average_path:
            d = nearest_dist_to_polyline(p, ideal_resampled)
            average_deviation.append(d)
            bad_flags.append(d is not None and d > threshold_m)
    else:
        average_deviation = [0.0 for _ in average_path]
        bad_flags = [False for _ in average_path]

    bad_segments = split_bad_segments(average_path, bad_flags)

    traces = []

    if ideal_points:
        traces.append(js_trace("Ideal path", ideal_points, color="gray", width=5, dash="dash", visible=True))

    for r in run_data:
        traces.append(js_trace(
            f"Actual {r['name']}",
            r["actual"],
            color="blue",
            width=3,
            visible=True,
            opacity=0.45,
        ))

    traces.append(js_trace("Average actual path", average_path, color="orange", width=7, visible=True))

    for idx, seg in enumerate(bad_segments, start=1):
        traces.append(js_trace(
            f"Average deviation > {threshold_percent:.1f}% segment {idx}",
            seg,
            color="red",
            width=10,
            visible=True,
        ))

    if average_path:
        traces.append(js_marker("Average start", average_path[0], color="green", size=5))
        traces.append(js_marker("Average end", average_path[-1], color="red", size=5))

    mean_avg_dev = statistics.mean(average_deviation) if average_deviation else 0.0
    max_avg_dev = max(average_deviation) if average_deviation else 0.0
    bad_count = sum(1 for b in bad_flags if b)
    bad_percent = 100.0 * bad_count / max(len(bad_flags), 1)

    run_lengths = [r["actual_length"] for r in run_data]
    html_path = summary_dir / "batch_paths_3d.html"

    summary = {
        "experiment_dir": str(experiment_dir),
        "run_count": len(run_data),
        "threshold_percent": threshold_percent,
        "threshold_m": threshold_m,
        "characteristic_size_m": char_size,
        "average_path_mean_deviation_to_ideal_m": mean_avg_dev,
        "average_path_max_deviation_to_ideal_m": max_avg_dev,
        "average_path_bad_sample_percent": bad_percent,
        "actual_path_length_mean_m": statistics.mean(run_lengths) if run_lengths else 0.0,
        "actual_path_length_std_m": statistics.stdev(run_lengths) if len(run_lengths) > 1 else 0.0,
        "actual_path_length_min_m": min(run_lengths) if run_lengths else 0.0,
        "actual_path_length_max_m": max(run_lengths) if run_lengths else 0.0,
    }

    (summary_dir / "batch_visual_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    report_path = summary_dir / "batch_visual_report.txt"
    with report_path.open("w", encoding="utf-8") as f:
        f.write("Batch 3D Visualization Report\n")
        f.write("=============================\n\n")
        f.write(f"Experiment: {experiment_dir}\n")
        f.write(f"Runs included: {len(run_data)}\n")
        f.write(f"Threshold: {threshold_percent:.2f}% of characteristic path size\n")
        f.write(f"Characteristic path size: {char_size:.3f} m\n")
        f.write(f"Threshold: {threshold_m:.3f} m\n\n")
        f.write("Average path deviation to ideal path:\n")
        f.write(f"  mean: {mean_avg_dev:.4f} m\n")
        f.write(f"  max:  {max_avg_dev:.4f} m\n")
        f.write(f"  samples above threshold: {bad_percent:.2f}%\n\n")
        f.write("Actual path length over all runs:\n")
        f.write(f"  mean: {summary['actual_path_length_mean_m']:.4f} m\n")
        f.write(f"  std:  {summary['actual_path_length_std_m']:.4f} m\n")
        f.write(f"  min:  {summary['actual_path_length_min_m']:.4f} m\n")
        f.write(f"  max:  {summary['actual_path_length_max_m']:.4f} m\n\n")
        f.write("Generated HTML:\n")
        f.write(f"  {html_path}\n\n")
        f.write("Notes:\n")
        f.write("- Blue lines: individual actual runs\n")
        f.write("- Orange line: average actual path\n")
        f.write("- Red line sections: average path deviation above threshold\n")
        f.write("- The threshold is relative to the characteristic path size, not the full path length.\n")

    html = f"""<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>Batch Path Comparison 3D</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    body {{
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 0;
    }}
    #header {{
      padding: 14px 18px;
      border-bottom: 1px solid #ddd;
    }}
    #plot {{
      width: 100vw;
      height: 82vh;
    }}
    .small {{
      color: #555;
      font-size: 0.9em;
    }}
    code {{
      background: #f2f2f2;
      padding: 2px 4px;
      border-radius: 3px;
    }}
  </style>
</head>
<body>
  <div id="header">
    <h2>Batch Path Comparison 3D</h2>
    <div class="small">
      Experiment: <code>{experiment_dir}</code><br>
      Runs included: <b>{len(run_data)}</b> |
      Threshold: <b>{threshold_percent:.1f}%</b> of characteristic path size =
      <b>{threshold_m:.3f} m</b><br>
      Mean average-path deviation: <b>{mean_avg_dev:.4f} m</b> |
      Max average-path deviation: <b>{max_avg_dev:.4f} m</b> |
      Samples above threshold: <b>{bad_percent:.2f}%</b><br>
      Legend entries can be clicked to hide/show individual runs.
    </div>
  </div>

  <div id="plot"></div>

  <script>
    const data = {json.dumps(traces)};

    const layout = {{
      scene: {{
        xaxis: {{title: "x [m]"}},
        yaxis: {{title: "y [m]"}},
        zaxis: {{title: "z [m]"}},
        aspectmode: "data"
      }},
      legend: {{
        orientation: "v",
        x: 1.02,
        y: 1.0
      }},
      margin: {{l: 0, r: 280, b: 0, t: 0}},
      hovermode: "closest"
    }};

    Plotly.newPlot("plot", data, layout, {{responsive: true}});
  </script>
</body>
</html>
"""

    html_path.write_text(html, encoding="utf-8")

    print("[OK] Batch 3D HTML created:")
    print(f"     {html_path}")
    print(f"     {report_path}")
    print(f"     {summary_dir / 'batch_visual_summary.json'}")


if __name__ == "__main__":
    main()
