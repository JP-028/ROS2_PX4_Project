#!/usr/bin/env python3

import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


def load_spec(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def deg_to_rad(value):
    return value * math.pi / 180.0


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_pose(pose):
    q = pose.orientation

    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


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

    out = []

    for a, b in zip(corners[:-1], corners[1:]):
        for t in np.linspace(0.0, 1.0, samples_per_side, endpoint=False):
            out.append(a + t * (b - a))

    out.append(corners[-1])
    return np.array(out)


def generate_command_sequence(spec):
    params = spec.get("parameters", {})
    step_size = float(params.get("step_size_m", 0.02))

    pos = np.array([0.0, 0.0, 0.0])
    yaw = 0.0
    out = [pos.copy()]

    for cmd in spec.get("commands", []):
        action = str(cmd.get("action", "")).lower()

        if action == "yaw":
            yaw += deg_to_rad(float(cmd.get("angle_deg", 0.0)))
            yaw = wrap_angle(yaw)

        elif action in ["forward", "backward"]:
            distance = float(cmd.get("distance_m", 0.0))

            if action == "backward":
                distance *= -1.0

            steps = max(1, int(abs(distance) / step_size))
            start = pos.copy()
            direction = np.array([math.cos(yaw), math.sin(yaw), 0.0])

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


class LocalPosePathFollower(Node):
    def __init__(self, spec):
        super().__init__("local_pose_path_follower")

        self.spec = spec
        self.params = spec.get("parameters", {})

        self.path = generate_ideal_path(spec)
        self.cumdist = cumulative_distance(self.path)
        self.total_length = float(self.cumdist[-1])

        self.speed = float(self.params.get("speed_m_s", 0.12))
        self.max_speed = float(self.params.get("max_speed_m_s", 0.25))
        self.vertical_speed = float(self.params.get("vertical_speed_m_s", 0.20))

        self.lookahead_m = float(self.params.get("lookahead_m", 0.40))
        self.goal_tolerance_m = float(self.params.get("goal_tolerance_m", 0.20))

        self.kp_xy = float(self.params.get("kp_xy", 0.6))
        self.kp_z = float(self.params.get("kp_z", 0.5))

        self.k_yaw = float(self.params.get("k_yaw", 0.8))
        self.max_yaw_rate = deg_to_rad(float(self.params.get("yaw_rate_deg_s", 25.0)))

        # command_frame:
        # local = send velocity in local_start/world-like frame
        # body  = rotate velocity into UAV body frame before publishing
        self.command_frame = str(self.params.get("command_frame", "local")).lower()

        self.max_runtime_s = float(
            self.params.get(
                "max_runtime_s",
                max(30.0, self.total_length / max(self.speed, 1e-6) * 3.0),
            )
        )

        self.pose = None
        self.start_time = None

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(
            PoseStamped,
            "/uav/local_pose",
            self.pose_callback,
            10,
        )

        self.get_logger().info("Local pose path follower initialized.")
        self.get_logger().info("Input pose: /uav/local_pose")
        self.get_logger().info("Output cmd: /cmd_vel")
        self.get_logger().info(f"Path length: {self.total_length:.3f} m")
        self.get_logger().info(f"Command frame: {self.command_frame}")
        self.get_logger().info("Waiting for /uav/local_pose...")

    def pose_callback(self, msg):
        self.pose = msg

        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Received first /uav/local_pose. Starting path following.")

    def current_position(self):
        return np.array([
            float(self.pose.pose.position.x),
            float(self.pose.pose.position.y),
            float(self.pose.pose.position.z),
        ])

    def current_yaw(self):
        return yaw_from_pose(self.pose.pose)

    def nearest_path_distance(self, actual):
        diff = self.path - actual
        dist = np.linalg.norm(diff, axis=1)
        idx = int(np.argmin(dist))

        return float(self.cumdist[idx]), idx, float(dist[idx])

    def convert_velocity_to_body_frame(self, v_local):
        yaw = self.current_yaw()

        c = math.cos(-yaw)
        s = math.sin(-yaw)

        vx = c * v_local[0] - s * v_local[1]
        vy = s * v_local[0] + c * v_local[1]

        return np.array([vx, vy, v_local[2]])

    def publish_stop(self):
        self.pub.publish(Twist())

    def publish_velocity(self, v_local):
        if self.command_frame == "body":
            v_cmd = self.convert_velocity_to_body_frame(v_local)
        else:
            v_cmd = v_local

        msg = Twist()
        msg.linear.x = float(v_cmd[0])
        msg.linear.y = float(v_cmd[1])
        msg.linear.z = float(v_cmd[2])

        horizontal = math.sqrt(v_local[0] ** 2 + v_local[1] ** 2)

        if horizontal > 0.03 and self.k_yaw > 0.0:
            desired_yaw = math.atan2(v_local[1], v_local[0])
            yaw_error = wrap_angle(desired_yaw - self.current_yaw())
            msg.angular.z = clamp(
                self.k_yaw * yaw_error,
                -self.max_yaw_rate,
                self.max_yaw_rate,
            )
        else:
            msg.angular.z = 0.0

        self.pub.publish(msg)

    def control_step(self):
        actual = self.current_position()

        nearest_s, _nearest_idx, _nearest_error = self.nearest_path_distance(actual)

        target_s = min(nearest_s + self.lookahead_m, self.total_length)
        target, tangent = interpolate_path(self.path, self.cumdist, target_s)

        error = target - actual

        v_local = tangent * self.speed

        v_local[0] += self.kp_xy * error[0]
        v_local[1] += self.kp_xy * error[1]
        v_local[2] += self.kp_z * error[2]

        horizontal_speed = math.sqrt(v_local[0] ** 2 + v_local[1] ** 2)

        if horizontal_speed > self.max_speed:
            scale = self.max_speed / max(horizontal_speed, 1e-9)
            v_local[0] *= scale
            v_local[1] *= scale

        v_local[2] = clamp(v_local[2], -self.vertical_speed, self.vertical_speed)

        self.publish_velocity(v_local)

        final_error = np.linalg.norm(self.path[-1] - actual)
        elapsed = time.time() - self.start_time

        if target_s >= self.total_length - 1e-6 and final_error < self.goal_tolerance_m:
            self.get_logger().info(f"Goal reached. Final error: {final_error:.3f} m")
            return True

        if elapsed > self.max_runtime_s:
            self.get_logger().warn(f"Max runtime reached. Final error: {final_error:.3f} m")
            return True

        return False

    def run(self):
        rate_hz = 30.0
        period = 1.0 / rate_hz
        finished = False

        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        while rclpy.ok() and not finished:
            rclpy.spin_once(self, timeout_sec=0.0)
            finished = self.control_step()
            time.sleep(period)

        for _ in range(30):
            self.publish_stop()
            time.sleep(0.05)

        self.get_logger().info("Local pose path follower finished.")


def main():
    if len(sys.argv) != 2:
        print("Usage: execute_path_from_local_pose.py <path_spec.yaml>")
        sys.exit(1)

    spec = load_spec(Path(sys.argv[1]))

    rclpy.init()
    node = LocalPosePathFollower(spec)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
