#!/usr/bin/env python3

import argparse
import math
import time
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry


def rotation_matrix(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]])


def generate_square(side_length=5.0, points_per_side=200, direction=1):
    p0 = np.array([0.0, 0.0])
    p1 = np.array([side_length, 0.0])
    p2 = np.array([side_length, direction * side_length])
    p3 = np.array([0.0, direction * side_length])
    p4 = np.array([0.0, 0.0])

    segments = [(p0, p1), (p1, p2), (p2, p3), (p3, p4)]
    points = []

    for a, b in segments:
        for t in np.linspace(0.0, 1.0, points_per_side, endpoint=False):
            points.append(a + t * (b - a))

    points.append(p4)
    return np.array(points)


def generate_circle(radius=2.5, points=1000, direction=1):
    center = np.array([0.0, direction * radius])

    if direction == 1:
        angles = np.linspace(-math.pi / 2.0, 3.0 * math.pi / 2.0, points)
    else:
        angles = np.linspace(math.pi / 2.0, -3.0 * math.pi / 2.0, points)

    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return np.column_stack([x, y])


def estimate_initial_heading(points):
    if len(points) < 10:
        return 0.0

    p0 = points[0]
    for i in range(5, min(len(points), 80)):
        d = points[i] - p0
        if np.linalg.norm(d) > 0.2:
            return math.atan2(d[1], d[0])

    return 0.0


def nearest_distances(actual, ideal):
    distances = []
    chunk_size = 500

    for i in range(0, len(actual), chunk_size):
        chunk = actual[i:i + chunk_size]
        diff = chunk[:, None, :] - ideal[None, :, :]
        dist = np.linalg.norm(diff, axis=2)
        distances.extend(np.min(dist, axis=1))

    return np.array(distances)


def align_ideal_to_actual(ideal, heading):
    rot = rotation_matrix(heading)
    return ideal @ rot.T


def choose_best_ideal(actual_rel, mode, side_length, diameter):
    heading = estimate_initial_heading(actual_rel)

    if mode == "square":
        ideal_a = align_ideal_to_actual(generate_square(side_length, direction=1), heading)
        ideal_b = align_ideal_to_actual(generate_square(side_length, direction=-1), heading)
        reference_size = side_length

    elif mode == "circle":
        radius = diameter / 2.0
        ideal_a = align_ideal_to_actual(generate_circle(radius, direction=1), heading)
        ideal_b = align_ideal_to_actual(generate_circle(radius, direction=-1), heading)
        reference_size = diameter

    else:
        raise ValueError(f"Unknown mode: {mode}")

    dist_a = nearest_distances(actual_rel, ideal_a)
    dist_b = nearest_distances(actual_rel, ideal_b)

    if np.mean(dist_a) <= np.mean(dist_b):
        return ideal_a, dist_a, heading, "left/ccw", reference_size
    else:
        return ideal_b, dist_b, heading, "right/cw", reference_size


def polyline_svg(points, min_x, min_y, scale, height, margin):
    svg_points = []
    for x, y in points:
        sx = margin + (x - min_x) * scale
        sy = height - margin - (y - min_y) * scale
        svg_points.append(f"{sx:.1f},{sy:.1f}")
    return " ".join(svg_points)


def save_svg(svg_path, actual, ideal, title):
    all_points = np.vstack([actual, ideal])
    min_x = float(np.min(all_points[:, 0]))
    max_x = float(np.max(all_points[:, 0]))
    min_y = float(np.min(all_points[:, 1]))
    max_y = float(np.max(all_points[:, 1]))

    width = 900
    height = 900
    margin = 80

    span_x = max(max_x - min_x, 0.1)
    span_y = max(max_y - min_y, 0.1)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

    ideal_line = polyline_svg(ideal, min_x, min_y, scale, height, margin)
    actual_line = polyline_svg(actual, min_x, min_y, scale, height, margin)

    start_x = margin + (actual[0, 0] - min_x) * scale
    start_y = height - margin - (actual[0, 1] - min_y) * scale

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect width="100%" height="100%" fill="white"/>
  <text x="40" y="40" font-family="Arial" font-size="24" fill="black">{title}</text>

  <line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="black" stroke-width="2"/>
  <line x1="{margin}" y1="{height-margin}" x2="{margin}" y2="{margin}" stroke="black" stroke-width="2"/>
  <text x="{width/2-30}" y="{height-25}" font-family="Arial" font-size="18" fill="black">x [m]</text>
  <text x="20" y="{height/2}" font-family="Arial" font-size="18" fill="black">y [m]</text>

  <polyline points="{ideal_line}" fill="none" stroke="gray" stroke-width="4" stroke-dasharray="12,8"/>
  <polyline points="{actual_line}" fill="none" stroke="blue" stroke-width="4"/>

  <circle cx="{start_x:.1f}" cy="{start_y:.1f}" r="8" fill="green"/>

  <rect x="560" y="70" width="290" height="90" fill="white" stroke="black"/>
  <line x1="580" y1="100" x2="650" y2="100" stroke="gray" stroke-width="4" stroke-dasharray="12,8"/>
  <text x="670" y="106" font-family="Arial" font-size="18" fill="black">Ideal path</text>
  <line x1="580" y1="135" x2="650" y2="135" stroke="blue" stroke-width="4"/>
  <text x="670" y="141" font-family="Arial" font-size="18" fill="black">Recorded path</text>
</svg>
'''
    Path(svg_path).write_text(svg)


class PathAnalyzer(Node):
    def __init__(self, args):
        super().__init__("path_analyzer_node")

        self.args = args
        self.samples = []

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

        self.start_time = time.time()
        self.get_logger().info("Recording /fmu/out/vehicle_odometry ...")
        self.get_logger().info("Press Ctrl+C to stop and generate CSV, SVG and report.")

    def odom_callback(self, msg):
        try:
            x = float(msg.position[0])
            y = float(msg.position[1])
            z = float(msg.position[2])
        except Exception:
            return

        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return

        t = time.time() - self.start_time
        self.samples.append([t, x, y, z])

    def save_results(self):
        if len(self.samples) < 20:
            print("Not enough odometry samples recorded.")
            print(f"Samples recorded: {len(self.samples)}")
            return

        out_dir = Path(self.args.output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        data = np.array(self.samples)
        t = data[:, 0]
        xy = data[:, 1:3]

        actual_rel = xy - xy[0]

        ideal, distances, heading, chosen_direction, reference_size = choose_best_ideal(
            actual_rel=actual_rel,
            mode=self.args.mode,
            side_length=self.args.side_length,
            diameter=self.args.diameter,
        )

        mean_error_m = float(np.mean(distances))
        max_error_m = float(np.max(distances))
        median_error_m = float(np.median(distances))

        mean_error_percent = 100.0 * mean_error_m / reference_size
        max_error_percent = 100.0 * max_error_m / reference_size
        median_error_percent = 100.0 * median_error_m / reference_size

        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        run_dir = out_dir / f"{timestamp}_{self.args.mode}"
        run_dir.mkdir(parents=True, exist_ok=True)

        csv_path = str(run_dir / "actual_path.csv")
        ideal_csv_path = str(run_dir / "ideal_path.csv")
        report_path = str(run_dir / "path_analysis_report.txt")
        svg_path = str(run_dir / "path_comparison.svg")

        csv_data = np.column_stack([t, actual_rel[:, 0], actual_rel[:, 1], data[:, 3]])
        np.savetxt(
            csv_path,
            csv_data,
            delimiter=",",
            header="time_s,relative_x_m,relative_y_m,z_m",
            comments="",
        )

        np.savetxt(
            ideal_csv_path,
            ideal,
            delimiter=",",
            header="ideal_x_m,ideal_y_m",
            comments="",
        )

        title = f"{self.args.mode.capitalize()} path comparison | mean deviation {mean_error_percent:.2f}%"
        save_svg(svg_path, actual_rel, ideal, title)

        report = f"""Path Analysis Report
Mode: {self.args.mode}
Samples: {len(self.samples)}
Chosen ideal direction: {chosen_direction}
Estimated initial heading rad: {heading:.4f}

Reference size:
  square side length or circle diameter: {reference_size:.3f} m

Deviation metric:
  nearest distance from each recorded odometry point to the ideal path
  percentage = distance / reference size * 100

Mean deviation:
  {mean_error_m:.3f} m
  {mean_error_percent:.2f} %

Median deviation:
  {median_error_m:.3f} m
  {median_error_percent:.2f} %

Max deviation:
  {max_error_m:.3f} m
  {max_error_percent:.2f} %

Output files:
  Actual CSV: {csv_path}
  Ideal CSV: {ideal_csv_path}
  SVG graph: {svg_path}
  Report: {report_path}
"""

        with open(report_path, "w") as f:
            f.write(report)

        print()
        print(report)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["square", "circle"], required=True)
    parser.add_argument("--side-length", type=float, default=5.0)
    parser.add_argument("--diameter", type=float, default=5.0)
    parser.add_argument("--output-dir", default="/home/user/ros2_ws/ROS2_PX4_Project/path_analysis")
    args = parser.parse_args()

    rclpy.init()
    node = PathAnalyzer(args)

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
