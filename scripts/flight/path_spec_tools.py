#!/usr/bin/env python3

import math
import numpy as np


def deg_to_rad(value):
    return float(value) * math.pi / 180.0


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def cumulative_distance(points):
    if len(points) < 2:
        return np.array([0.0])
    diffs = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(diffs)])


def unit_tangent(points, idx):
    idx = max(0, min(idx, len(points) - 2))
    direction = points[idx + 1] - points[idx]
    norm = np.linalg.norm(direction)

    if norm < 1e-9:
        return np.array([1.0, 0.0, 0.0])

    return direction / norm


def interpolate_path(points, cumdist, distance):
    if len(points) == 0:
        return np.zeros(3), np.array([1.0, 0.0, 0.0])

    if distance <= 0.0:
        return points[0], unit_tangent(points, 0)

    if distance >= cumdist[-1]:
        return points[-1], unit_tangent(points, len(points) - 2)

    idx = int(np.searchsorted(cumdist, distance) - 1)
    idx = max(0, min(idx, len(points) - 2))

    seg_len = max(cumdist[idx + 1] - cumdist[idx], 1e-9)
    alpha = (distance - cumdist[idx]) / seg_len

    point = points[idx] + alpha * (points[idx + 1] - points[idx])
    tangent = unit_tangent(points, idx)

    return point, tangent


def add_line(points, start, direction, distance_m, step_size_m):
    distance_m = float(distance_m)
    steps = max(1, int(abs(distance_m) / max(step_size_m, 1e-6)))

    for i in range(1, steps + 1):
        p = start + direction * distance_m * (i / steps)
        points.append(p.copy())

    return points[-1].copy()


def add_arc(points, start, heading_rad, radius_m, angle_deg, direction, step_size_m):
    radius_m = float(radius_m)
    angle_rad = abs(deg_to_rad(angle_deg))

    if radius_m <= 0.0 or angle_rad <= 0.0:
        return start.copy(), heading_rad

    sign = 1.0 if str(direction).lower() in ["left", "ccw"] else -1.0

    # Heading vector and left normal in local XY.
    forward = np.array([math.cos(heading_rad), math.sin(heading_rad), 0.0])
    left = np.array([-math.sin(heading_rad), math.cos(heading_rad), 0.0])

    center = start + sign * radius_m * left

    radial_start = start - center
    start_angle = math.atan2(radial_start[1], radial_start[0])

    arc_length = radius_m * angle_rad
    steps = max(2, int(arc_length / max(step_size_m, 1e-6)))

    for i in range(1, steps + 1):
        a = start_angle + sign * angle_rad * (i / steps)
        p = np.array([
            center[0] + radius_m * math.cos(a),
            center[1] + radius_m * math.sin(a),
            start[2],
        ])
        points.append(p.copy())

    new_heading = wrap_angle(heading_rad + sign * angle_rad)
    return points[-1].copy(), new_heading


def generate_segment_path(spec):
    params = spec.get("parameters", {})
    defaults = spec.get("motion_defaults", {})

    step_size_m = float(
        params.get(
            "step_size_m",
            defaults.get("step_size_m", 0.02),
        )
    )

    pos = np.array([0.0, 0.0, 0.0])
    heading_rad = 0.0
    points = [pos.copy()]

    path_items = spec.get("path", [])

    for item in path_items:
        if item is None:
            continue

        # Format:
        # - move: forward
        #   distance_m: 2.0
        if "move" in item:
            move = str(item.get("move", "")).lower()
            distance = float(item.get("distance_m", 0.0))

            if move == "forward":
                direction = np.array([math.cos(heading_rad), math.sin(heading_rad), 0.0])
                pos = add_line(points, pos, direction, distance, step_size_m)

            elif move == "backward":
                direction = np.array([math.cos(heading_rad), math.sin(heading_rad), 0.0])
                pos = add_line(points, pos, direction, -distance, step_size_m)

            elif move == "left":
                direction = np.array([-math.sin(heading_rad), math.cos(heading_rad), 0.0])
                pos = add_line(points, pos, direction, distance, step_size_m)

            elif move == "right":
                direction = np.array([-math.sin(heading_rad), math.cos(heading_rad), 0.0])
                pos = add_line(points, pos, direction, -distance, step_size_m)

            elif move == "up":
                direction = np.array([0.0, 0.0, 1.0])
                pos = add_line(points, pos, direction, distance, step_size_m)

            elif move == "down":
                direction = np.array([0.0, 0.0, 1.0])
                pos = add_line(points, pos, direction, -distance, step_size_m)

            else:
                raise ValueError(f"Unsupported move command: {move}")

        # Format:
        # - turn: left
        #   angle_deg: 90
        elif "turn" in item:
            turn = str(item.get("turn", "")).lower()
            angle = abs(float(item.get("angle_deg", 0.0)))

            if turn in ["left", "ccw"]:
                heading_rad = wrap_angle(heading_rad + deg_to_rad(angle))
            elif turn in ["right", "cw"]:
                heading_rad = wrap_angle(heading_rad - deg_to_rad(angle))
            else:
                raise ValueError(f"Unsupported turn command: {turn}")

            # Add a duplicate position so the heading change is represented without translation.
            points.append(pos.copy())

        # Format:
        # - arc:
        #     radius_m: 1.5
        #     angle_deg: 360
        #     direction: left
        elif "arc" in item:
            arc = item.get("arc", {})
            radius = float(arc.get("radius_m", 1.0))
            angle_deg = float(arc.get("angle_deg", 90.0))
            direction = str(arc.get("direction", "left")).lower()

            sign = 1.0 if direction in ["left", "ccw"] else -1.0
            angle_rad = abs(deg_to_rad(angle_deg))

            center_offset_x = arc.get("center_offset_x_m", None)
            center_offset_y = arc.get("center_offset_y_m", None)

            if center_offset_x is not None and center_offset_y is not None:
                center = pos + np.array([
                    float(center_offset_x),
                    float(center_offset_y),
                    0.0,
                ])

                radial_start = pos - center
                start_angle = math.atan2(radial_start[1], radial_start[0])

                arc_length = radius * angle_rad
                samples = max(2, int(math.ceil(arc_length / step_size_m)))

                for a in np.linspace(
                    start_angle,
                    start_angle + sign * angle_rad,
                    samples + 1,
                )[1:]:
                    p_arc = np.array([
                        center[0] + radius * math.cos(a),
                        center[1] + radius * math.sin(a),
                        pos[2],
                    ])
                    points.append(p_arc)

                pos = points[-1].copy()
                heading_rad = wrap_angle(
                    heading_rad + sign * angle_rad
                )

            else:
                pos, heading_rad = add_arc(
                    points,
                    pos,
                    heading_rad,
                    radius,
                    angle_deg,
                    direction,
                    step_size_m,
                )

        # Format:
        # - hold:
        #     duration_s: 2.0
        # Hold does not change the geometric path.
        elif "hold" in item:
            points.append(pos.copy())

        else:
            raise ValueError(f"Unsupported path item: {item}")

    return np.array(points)


def generate_circle(spec):
    params = spec.get("parameters", {})

    diameter = float(params.get("diameter_m", 3.0))
    radius = diameter / 2.0
    direction = str(params.get("direction", "ccw")).lower()
    altitude = float(params.get("altitude_m", 0.0))
    samples = int(params.get("samples", 1500))

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
    params = spec.get("parameters", {})

    side = float(params.get("side_length_m", 3.0))
    direction = str(params.get("direction", "left")).lower()
    altitude = float(params.get("altitude_m", 0.0))
    samples_per_side = int(params.get("samples_per_side", 300))

    sign = 1.0 if direction in ["left", "ccw"] else -1.0

    corners = [
        np.array([0.0, 0.0, altitude]),
        np.array([side, 0.0, altitude]),
        np.array([side, sign * side, altitude]),
        np.array([0.0, sign * side, altitude]),
        np.array([0.0, 0.0, altitude]),
    ]

    points = []

    for a, b in zip(corners[:-1], corners[1:]):
        for t in np.linspace(0.0, 1.0, samples_per_side, endpoint=False):
            points.append(a + t * (b - a))

    points.append(corners[-1])
    return np.array(points)


def generate_command_sequence(spec):
    # Backward compatibility for older YAML format.
    converted = {
        "name": spec.get("name", "converted_command_sequence"),
        "type": "segment_path",
        "parameters": spec.get("parameters", {}),
        "path": [],
    }

    for cmd in spec.get("commands", []):
        action = str(cmd.get("action", "")).lower()

        if action in ["forward", "backward", "up", "down"]:
            converted["path"].append({
                "move": action,
                "distance_m": float(cmd.get("distance_m", 0.0)),
            })

        elif action == "yaw":
            angle = float(cmd.get("angle_deg", 0.0))
            if angle >= 0.0:
                converted["path"].append({"turn": "left", "angle_deg": abs(angle)})
            else:
                converted["path"].append({"turn": "right", "angle_deg": abs(angle)})

        else:
            raise ValueError(f"Unsupported command_sequence action: {action}")

    return generate_segment_path(converted)


def generate_ideal_path(spec):
    path_type = str(spec.get("type", "")).lower()

    if path_type == "segment_path":
        return generate_segment_path(spec)

    if path_type == "circle":
        return generate_circle(spec)

    if path_type == "square":
        return generate_square(spec)

    if path_type == "command_sequence":
        return generate_command_sequence(spec)

    raise ValueError(f"Unsupported path type: {path_type}")
