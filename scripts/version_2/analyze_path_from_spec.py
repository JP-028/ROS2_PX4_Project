#!/usr/bin/env python3

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry


def load_spec(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def deg_to_rad(value):
    return value * math.pi / 180.0


def path_length(points):
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def generate_circle(spec):
    p = spec.get("parameters", {})
    diameter = float(p.get("diameter_m", 5.0))
    radius = diameter / 2.0
    direction = str(p.get("direction", "ccw")).lower()
    samples = int(p.get("samples", 1000))
    altitude = float(p.get("altitude_m", 0.0))

    if direction in ["cw", "right"]:
        center = np.array([0.0, -radius, altitude])
        angles = np.linspace(math.pi / 2.0, -3.0 * math.pi / 2.0, samples)
    else:
        center = np.array([0.0, radius, altitude])
        angles = np.linspace(-math.pi / 2.0, 3.0 * math.pi / 2.0, samples)

    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    z = np.ones_like(x) * altitude

    return np.column_stack([x, y, z])


def generate_square(spec):
    p = spec.get("parameters", {})
    side = float(p.get("side_length_m", 5.0))
    direction = str(p.get("direction", "left")).lower()
    samples_per_side = int(p.get("samples_per_side", 250))
    altitude = float(p.get("altitude_m", 0.0))

    sign = 1.0 if direction in ["left", "ccw"] else -1.0

    points = [
        np.array([0.0, 0.0, altitude]),
        np.array([side, 0.0, altitude]),
        np.array([side, sign * side, altitude]),
        np.array([0.0, sign * side, altitude]),
        np.array([0.0, 0.0, altitude]),
    ]

    out = []

    for a, b in zip(points[:-1], points[1:]):
        for t in np.linspace(0.0, 1.0, samples_per_side, endpoint=False):
            out.append(a + t * (b - a))

    out.append(points[-1])
    return np.array(out)


def generate_command_sequence(spec):
    p = spec.get("parameters", {})
    step_size = float(p.get("step_size_m", 0.02))

    pos = np.array([0.0, 0.0, 0.0])
    yaw = 0.0
    out = [pos.copy()]

    for cmd in spec.get("commands", []):
        action = str(cmd.get("action", "")).lower()

        if action == "yaw":
            yaw += deg_to_rad(float(cmd.get("angle_deg", 0.0)))

        elif action in ["forward", "backward"]:
            distance = float(cmd.get("distance_m", 0.0))

            if action == "backward":
                distance *= -1.0

            steps = max(1, int(abs(distance) / step_size))
            direction = np.array([math.cos(yaw), math.sin(yaw), 0.0])
            start = pos.copy()

            for i in range(1, steps + 1):
                pos = start + direction * distance * (i / steps)
                out.append(pos.copy())

        elif action in ["up", "down"]:
            distance = float(cmd.get("distance_m", 0.0))

            if action == "down":
                distance *= -1.0

            steps = max(1, int(abs(distance) / step_size))
            start = pos.copy()

            for i in range(1, steps + 1):
                pos = start + np.array([0.0, 0.0, distance * (i / steps)])
                out.append(pos.copy())

        else:
            raise ValueError(f"Unknown command action: {action}")

    return np.array(out)


def generate_ideal_path(spec):
    path_type = str(spec.get("type", "")).lower()

    if path_type == "circle":
        return generate_circle(spec)
    if path_type == "square":
        return generate_square(spec)
    if path_type == "command_sequence":
        return generate_command_sequence(spec)

    raise ValueError(f"Unsupported path type: {path_type}")


def estimate_initial_heading(points):
    if len(points) < 10:
        return 0.0

    p0 = points[0, :2]

    for i in range(5, min(len(points), 100)):
        d = points[i, :2] - p0

        if np.linalg.norm(d) > 0.2:
            return math.atan2(d[1], d[0])

    return 0.0


def rotate_xy(points, heading):
    c = math.cos(heading)
    s = math.sin(heading)

    rot = np.array([[c, -s], [s, c]])

    out = points.copy()
    out[:, :2] = out[:, :2] @ rot.T

    return out


def nearest_distances(actual, ideal):
    distances = []
    nearest_indices = []
    chunk_size = 500

    for i in range(0, len(actual), chunk_size):
        chunk = actual[i:i + chunk_size]
        diff = chunk[:, None, :] - ideal[None, :, :]
        dist = np.linalg.norm(diff, axis=2)
        idx = np.argmin(dist, axis=1)

        distances.extend(dist[np.arange(len(chunk)), idx])
        nearest_indices.extend(idx)

    return np.array(distances), np.array(nearest_indices)


def make_3d_html(path, actual, ideal, report):
    actual_json = actual.tolist()
    ideal_json = ideal.tolist()
    report_json = json.dumps(report, indent=2)

    html = f"""<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>3D Flight Path Comparison</title>
  <style>
    body {{ margin: 0; font-family: Arial, sans-serif; display: flex; height: 100vh; }}
    #panel {{ width: 360px; padding: 16px; background: #f2f2f2; overflow: auto; box-sizing: border-box; }}
    #canvas {{ flex: 1; background: white; }}
    pre {{ white-space: pre-wrap; font-size: 12px; }}
    .legend {{ margin-top: 10px; }}
    .line {{ display: flex; align-items: center; gap: 8px; margin: 4px 0; }}
    .swatch {{ width: 30px; height: 4px; }}
  </style>
</head>
<body>
  <div id="panel">
    <h2>3D Flight Path Comparison</h2>
    <div class="legend">
      <div class="line"><div class="swatch" style="background: #777;"></div>Ideal path</div>
      <div class="line"><div class="swatch" style="background: #1f77b4;"></div>Recorded path</div>
      <div class="line"><div class="swatch" style="background: #2ca02c;"></div>Start</div>
      <div class="line"><div class="swatch" style="background: #d62728;"></div>End</div>
    </div>
    <p><b>Controls:</b></p>
    <p>Drag mouse to rotate. Mouse wheel to zoom.</p>
    <pre>{report_json}</pre>
  </div>
  <canvas id="canvas"></canvas>

<script>
const actual = {actual_json};
const ideal = {ideal_json};

const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

let yaw = -0.7;
let pitch = 0.7;
let zoom = 70;
let dragging = false;
let lastX = 0;
let lastY = 0;

function resize() {{
  canvas.width = window.innerWidth - 360;
  canvas.height = window.innerHeight;
  draw();
}}

window.addEventListener("resize", resize);

canvas.addEventListener("mousedown", e => {{
  dragging = true;
  lastX = e.clientX;
  lastY = e.clientY;
}});

canvas.addEventListener("mouseup", () => dragging = false);
canvas.addEventListener("mouseleave", () => dragging = false);

canvas.addEventListener("mousemove", e => {{
  if (!dragging) return;
  yaw += (e.clientX - lastX) * 0.01;
  pitch += (e.clientY - lastY) * 0.01;
  lastX = e.clientX;
  lastY = e.clientY;
  draw();
}});

canvas.addEventListener("wheel", e => {{
  e.preventDefault();
  zoom *= e.deltaY < 0 ? 1.1 : 0.9;
  draw();
}});

function allPoints() {{
  return actual.concat(ideal);
}}

function centerPoint() {{
  const pts = allPoints();
  let cx = 0, cy = 0, cz = 0;

  for (const p of pts) {{
    cx += p[0];
    cy += p[1];
    cz += p[2];
  }}

  return [cx / pts.length, cy / pts.length, cz / pts.length];
}}

function rotateProject(p, center) {{
  let x = p[0] - center[0];
  let y = p[1] - center[1];
  let z = p[2] - center[2];

  let cy = Math.cos(yaw), sy = Math.sin(yaw);
  let cp = Math.cos(pitch), sp = Math.sin(pitch);

  let x1 = cy * x - sy * y;
  let y1 = sy * x + cy * y;
  let z1 = z;

  let y2 = cp * y1 - sp * z1;
  let z2 = sp * y1 + cp * z1;

  return [
    canvas.width / 2 + x1 * zoom,
    canvas.height / 2 - z2 * zoom
  ];
}}

function drawPolyline(points, color, dashed=false) {{
  if (points.length < 2) return;

  const center = centerPoint();

  ctx.beginPath();
  ctx.strokeStyle = color;
  ctx.lineWidth = 3;
  ctx.setLineDash(dashed ? [10, 8] : []);

  const p0 = rotateProject(points[0], center);
  ctx.moveTo(p0[0], p0[1]);

  for (let i = 1; i < points.length; i++) {{
    const p = rotateProject(points[i], center);
    ctx.lineTo(p[0], p[1]);
  }}

  ctx.stroke();
  ctx.setLineDash([]);
}}

function drawPoint(p, color, r=6) {{
  const center = centerPoint();
  const q = rotateProject(p, center);

  ctx.beginPath();
  ctx.fillStyle = color;
  ctx.arc(q[0], q[1], r, 0, Math.PI * 2);
  ctx.fill();
}}

function drawAxes() {{
  const center = centerPoint();
  const axes = [
    [[0,0,0], [1,0,0], "X"],
    [[0,0,0], [0,1,0], "Y"],
    [[0,0,0], [0,0,1], "Z"]
  ];

  ctx.lineWidth = 2;
  ctx.strokeStyle = "#aaa";
  ctx.fillStyle = "#555";
  ctx.font = "16px Arial";

  for (const [a,b,label] of axes) {{
    const pa = rotateProject(a, center);
    const pb = rotateProject(b, center);

    ctx.beginPath();
    ctx.moveTo(pa[0], pa[1]);
    ctx.lineTo(pb[0], pb[1]);
    ctx.stroke();

    ctx.fillText(label, pb[0] + 5, pb[1] + 5);
  }}
}}

function draw() {{
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  drawAxes();
  drawPolyline(ideal, "#777", true);
  drawPolyline(actual, "#1f77b4", false);

  if (actual.length > 0) {{
    drawPoint(actual[0], "#2ca02c", 7);
    drawPoint(actual[actual.length - 1], "#d62728", 7);
  }}
}}

resize();
</script>
</body>
</html>
"""
    Path(path).write_text(html)


class PathAnalyzer(Node):
    def __init__(self, spec, spec_path):
        super().__init__("yaml_path_analyzer")

        self.spec = spec
        self.spec_path = spec_path
        self.samples = []
        self.start_time = time.time()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos,
        )

        self.get_logger().info("Recording /fmu/out/vehicle_odometry ...")
        self.get_logger().info("Stop with Ctrl+C after the flight is complete.")

    def odom_callback(self, msg):
        try:
            x = float(msg.position[0])
            y = float(msg.position[1])
            z_ned = float(msg.position[2])
        except Exception:
            return

        if not all(math.isfinite(v) for v in [x, y, z_ned]):
            return

        t = time.time() - self.start_time
        self.samples.append([t, x, y, z_ned])

    def save_results(self):
        if len(self.samples) < 20:
            print(f"Not enough odometry samples recorded: {len(self.samples)}")
            return

        spec_name = str(self.spec.get("name", "unnamed_path")).replace(" ", "_")
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")

        out_dir = Path("/home/user/ros2_ws/ROS2_PX4_Project/path_analysis") / f"{timestamp}_{spec_name}"
        out_dir.mkdir(parents=True, exist_ok=True)

        raw = np.array(self.samples)
        t = raw[:, 0]
        xyz_raw = raw[:, 1:4]

        actual = np.zeros_like(xyz_raw)
        actual[:, 0] = xyz_raw[:, 0] - xyz_raw[0, 0]
        actual[:, 1] = xyz_raw[:, 1] - xyz_raw[0, 1]
        actual[:, 2] = -(xyz_raw[:, 2] - xyz_raw[0, 2])

        ideal = generate_ideal_path(self.spec)

        heading = estimate_initial_heading(actual)
        ideal_aligned = rotate_xy(ideal, heading)

        distances, nearest_idx = nearest_distances(actual, ideal_aligned)

        ideal_len = path_length(ideal_aligned)
        actual_len = path_length(actual)
        reference = max(ideal_len, 1e-6)

        mean_error = float(np.mean(distances))
        median_error = float(np.median(distances))
        max_error = float(np.max(distances))
        rmse = float(math.sqrt(np.mean(distances ** 2)))

        mean_percent = 100.0 * mean_error / reference
        max_percent = 100.0 * max_error / reference
        length_error_percent = 100.0 * abs(actual_len - ideal_len) / reference

        max_idx = int(np.argmax(distances))

        report = {
            "spec_file": str(self.spec_path),
            "name": self.spec.get("name", "unnamed_path"),
            "type": self.spec.get("type", "unknown"),
            "samples_recorded": int(len(actual)),
            "ideal_path_length_m": round(ideal_len, 4),
            "actual_path_length_m": round(actual_len, 4),
            "path_length_error_percent": round(length_error_percent, 3),
            "mean_3d_deviation_m": round(mean_error, 4),
            "mean_3d_deviation_percent_of_ideal_length": round(mean_percent, 3),
            "median_3d_deviation_m": round(median_error, 4),
            "max_3d_deviation_m": round(max_error, 4),
            "max_3d_deviation_percent_of_ideal_length": round(max_percent, 3),
            "rmse_3d_m": round(rmse, 4),
            "max_deviation_sample_index": max_idx,
            "max_deviation_time_s": round(float(t[max_idx]), 3),
            "estimated_initial_heading_rad": round(float(heading), 4),
        }

        actual_csv = out_dir / "actual_path.csv"
        ideal_csv = out_dir / "ideal_path.csv"
        deviation_csv = out_dir / "deviation_samples.csv"
        report_txt = out_dir / "path_analysis_report.txt"
        report_json = out_dir / "path_analysis_report.json"
        html_3d = out_dir / "path_comparison_3d.html"

        np.savetxt(
            actual_csv,
            np.column_stack([t, actual]),
            delimiter=",",
            header="time_s,x_m,y_m,z_up_m",
            comments="",
        )

        np.savetxt(
            ideal_csv,
            ideal_aligned,
            delimiter=",",
            header="ideal_x_m,ideal_y_m,ideal_z_up_m",
            comments="",
        )

        np.savetxt(
            deviation_csv,
            np.column_stack([t, distances, nearest_idx]),
            delimiter=",",
            header="time_s,nearest_3d_deviation_m,nearest_ideal_index",
            comments="",
        )

        with open(report_json, "w") as f:
            json.dump(report, f, indent=2)

        with open(report_txt, "w") as f:
            f.write("Path Analysis Report\n")
            f.write("====================\n\n")
            for k, v in report.items():
                f.write(f"{k}: {v}\n")

        make_3d_html(html_3d, actual, ideal_aligned, report)

        print()
        print("Path Analysis Report")
        print("====================")
        for k, v in report.items():
            print(f"{k}: {v}")

        print()
        print(f"[INFO] Results saved in: {out_dir}")
        print(f"[INFO] Open 3D comparison in browser: {html_3d}")


def main():
    if len(sys.argv) != 2:
        print("Usage: analyze_path_from_spec.py <path_spec.yaml>")
        sys.exit(1)

    spec_path = Path(sys.argv[1])
    spec = load_spec(spec_path)

    rclpy.init()
    node = PathAnalyzer(spec, spec_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
