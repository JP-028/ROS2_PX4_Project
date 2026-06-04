#!/usr/bin/env python3

import math
import sys
import time
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def load_spec(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def deg_to_rad(value):
    return value * math.pi / 180.0


class PathExecutor(Node):
    def __init__(self, spec):
        super().__init__("yaml_path_executor")
        self.spec = spec
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def publish_for_duration(self, twist, duration_s, rate_hz=20.0):
        period = 1.0 / rate_hz
        end_time = time.time() + max(0.0, duration_s)

        while rclpy.ok() and time.time() < end_time:
            self.pub.publish(twist)
            time.sleep(period)

        self.stop_motion()

    def stop_motion(self, hold_s=0.5):
        msg = Twist()
        end_time = time.time() + hold_s

        while rclpy.ok() and time.time() < end_time:
            self.pub.publish(msg)
            time.sleep(0.05)

    def fly_forward(self, distance_m, speed_m_s):
        duration = abs(distance_m) / max(abs(speed_m_s), 1e-6)

        msg = Twist()
        msg.linear.x = math.copysign(abs(speed_m_s), distance_m)

        self.get_logger().info(f"Forward {distance_m:.2f} m at {msg.linear.x:.2f} m/s")
        self.publish_for_duration(msg, duration)

    def fly_vertical(self, distance_m, speed_m_s):
        duration = abs(distance_m) / max(abs(speed_m_s), 1e-6)

        msg = Twist()
        msg.linear.z = math.copysign(abs(speed_m_s), distance_m)

        self.get_logger().info(f"Vertical {distance_m:.2f} m at {msg.linear.z:.2f} m/s")
        self.publish_for_duration(msg, duration)

    def yaw(self, angle_deg, yaw_rate_deg_s):
        duration = abs(angle_deg) / max(abs(yaw_rate_deg_s), 1e-6)

        msg = Twist()
        msg.angular.z = math.copysign(deg_to_rad(abs(yaw_rate_deg_s)), angle_deg)

        self.get_logger().info(f"Yaw {angle_deg:.2f} deg at {yaw_rate_deg_s:.2f} deg/s")
        self.publish_for_duration(msg, duration)

    def execute_circle(self):
        params = self.spec.get("parameters", {})
        diameter = float(params.get("diameter_m", 5.0))
        speed = float(params.get("speed_m_s", 0.3))
        direction = str(params.get("direction", "ccw")).lower()

        radius = diameter / 2.0
        yaw_rate = speed / max(radius, 1e-6)
        duration = 2.0 * math.pi * radius / max(speed, 1e-6)

        if direction in ["cw", "right"]:
            yaw_rate *= -1.0

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = yaw_rate

        self.get_logger().info(
            f"Circle diameter={diameter:.2f} m, speed={speed:.2f} m/s, yaw_rate={yaw_rate:.3f} rad/s"
        )
        self.publish_for_duration(msg, duration)

    def execute_square(self):
        params = self.spec.get("parameters", {})
        side = float(params.get("side_length_m", 5.0))
        speed = float(params.get("speed_m_s", 0.3))
        yaw_rate = float(params.get("yaw_rate_deg_s", 22.5))
        direction = str(params.get("direction", "left")).lower()

        turn_angle = 90.0 if direction in ["left", "ccw"] else -90.0

        self.get_logger().info(
            f"Square side={side:.2f} m, speed={speed:.2f} m/s, turn={turn_angle:.1f} deg"
        )

        for _ in range(4):
            self.fly_forward(side, speed)
            self.yaw(turn_angle, yaw_rate)

    def execute_command_sequence(self):
        params = self.spec.get("parameters", {})
        speed = float(params.get("speed_m_s", 0.3))
        vertical_speed = float(params.get("vertical_speed_m_s", speed))
        yaw_rate = float(params.get("yaw_rate_deg_s", 22.5))

        commands = self.spec.get("commands", [])

        for i, cmd in enumerate(commands, start=1):
            action = str(cmd.get("action", "")).lower()
            self.get_logger().info(f"Command {i}: {cmd}")

            if action == "forward":
                self.fly_forward(float(cmd.get("distance_m", 0.0)), speed)

            elif action == "backward":
                self.fly_forward(-float(cmd.get("distance_m", 0.0)), speed)

            elif action == "up":
                self.fly_vertical(float(cmd.get("distance_m", 0.0)), vertical_speed)

            elif action == "down":
                self.fly_vertical(-float(cmd.get("distance_m", 0.0)), vertical_speed)

            elif action == "yaw":
                self.yaw(float(cmd.get("angle_deg", 0.0)), yaw_rate)

            else:
                raise ValueError(f"Unknown command action: {action}")

    def execute(self):
        path_type = str(self.spec.get("type", "")).lower()
        name = self.spec.get("name", "unnamed_path")

        self.get_logger().info(f"Executing YAML path: {name} ({path_type})")
        self.stop_motion(1.0)

        if path_type == "circle":
            self.execute_circle()
        elif path_type == "square":
            self.execute_square()
        elif path_type == "command_sequence":
            self.execute_command_sequence()
        else:
            raise ValueError(f"Unsupported path type: {path_type}")

        self.stop_motion(1.0)
        self.get_logger().info("Path execution finished.")


def main():
    if len(sys.argv) != 2:
        print("Usage: execute_path_from_spec.py <path_spec.yaml>")
        sys.exit(1)

    spec_path = Path(sys.argv[1])
    spec = load_spec(spec_path)

    rclpy.init()
    node = PathExecutor(spec)

    try:
        node.execute()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
