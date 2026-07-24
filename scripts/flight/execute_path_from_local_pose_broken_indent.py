#!/usr/bin/env python3

import math
import signal
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
    return float(value) * math.pi / 180.0


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(high, value))


def yaw_from_pose(pose):
    q = pose.orientation

    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


class SegmentExecutor(Node):
    def __init__(self, spec):
        super().__init__("segment_path_executor")

        self.spec = spec
        self.params = spec.get("parameters", {})
        self.defaults = spec.get("motion_defaults", {})

        self.path_items = spec.get("path", [])

        self.speed = float(self.defaults.get("speed_m_s", self.params.get("speed_m_s", 0.10)))
        self.max_speed = float(self.defaults.get("max_speed_m_s", self.params.get("max_speed_m_s", 0.22)))
        self.vertical_speed = float(self.defaults.get("vertical_speed_m_s", self.params.get("vertical_speed_m_s", 0.15)))

        self.kp_xy = float(self.defaults.get("kp_xy", self.params.get("kp_xy", 0.45)))
        self.kp_z = float(self.defaults.get("kp_z", self.params.get("kp_z", 0.40)))

        self.position_tolerance_m = float(self.defaults.get("position_tolerance_m", self.params.get("position_tolerance_m", 0.18)))
        self.segment_timeout_factor = float(self.defaults.get("segment_timeout_factor", self.params.get("segment_timeout_factor", 4.0)))
        self.corner_hold_s = float(self.defaults.get("corner_hold_s", self.params.get("corner_hold_s", 0.8)))
        self.finish_hold_s = float(self.defaults.get("finish_hold_s", self.params.get("finish_hold_s", 1.0)))

        self.command_frame = str(self.defaults.get("command_frame", self.params.get("command_frame", "local"))).lower()

        self.pose = None
        self.start_time = None
        self.abort_requested = False
        self.returning_home = False

        self.current_target = np.array([0.0, 0.0, 0.0])
        self.path_start_position = np.array([0.0, 0.0, 0.0])
        self.path_start_yaw = 0.0
        self.current_heading = 0.0

        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.sub = self.create_subscription(PoseStamped, "/uav/local_pose", self.pose_callback, 10)

        self.get_logger().info("Generic segment executor initialized.")
        self.get_logger().info("Input pose: /uav/local_pose")
        self.get_logger().info("Output cmd:  /cmd_vel")
        self.get_logger().info(f"speed={self.speed:.3f}, max_speed={self.max_speed:.3f}")
        self.get_logger().info(f"position_tolerance={self.position_tolerance_m:.3f}")
        self.get_logger().info(f"command_frame={self.command_frame}")
        self.get_logger().info("Waiting for /uav/local_pose...")

    def pose_callback(self, msg):
        self.pose = msg

        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info("Received first /uav/local_pose.")

    def current_position(self):
        return np.array([
            float(self.pose.pose.position.x),
            float(self.pose.pose.position.y),
            float(self.pose.pose.position.z),
        ])

    def current_yaw(self):
        return yaw_from_pose(self.pose.pose)

    def to_body_frame(self, v_local):
        yaw = self.current_yaw()

        c = math.cos(-yaw)
        s = math.sin(-yaw)

        vx = c * v_local[0] - s * v_local[1]
        vy = s * v_local[0] + c * v_local[1]

        return np.array([vx, vy, v_local[2]])

    def publish_stop(self):
        self.pub.publish(Twist())

    def request_abort(self):
        self.abort_requested = True
        self.get_logger().warn(
            "Flight abort requested. Returning to start position."
        )

    def publish_velocity(self, v_local):
        if self.command_frame == "body":
            v_cmd = self.to_body_frame(v_local)
        else:
            v_cmd = v_local

        msg = Twist()
        msg.linear.x = float(v_cmd[0])
        msg.linear.y = float(v_cmd[1])
        msg.linear.z = float(v_cmd[2])

        # Yaw is kept disabled for now.
        # First priority: stable position tracking.
        msg.angular.z = 0.0

        self.pub.publish(msg)

    def wait_for_pose(self):
        while rclpy.ok() and self.pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def hold_position(self, duration_s):
        end = time.time() + float(duration_s)

        while rclpy.ok() and time.time() < end:
            self.publish_stop()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def move_to_target(self, target, nominal_direction=None, nominal_speed=None):
        if nominal_speed is None:
            nominal_speed = self.speed

        target = np.array(target, dtype=float)
        start_pos = self.current_position()

        distance = float(np.linalg.norm(target - start_pos))
        nominal_duration = distance / max(nominal_speed, 1e-6)
        timeout_s = max(8.0, nominal_duration * self.segment_timeout_factor)

        start_time = time.time()

        self.get_logger().info(
            f"Move segment: target=({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}), "
            f"distance={distance:.2f} m, timeout={timeout_s:.1f} s"
        )

        period = 1.0 / 30.0

        while rclpy.ok():
            if self.abort_requested:
                self.publish_stop()
                return False

            rclpy.spin_once(self, timeout_sec=0.0)

            actual = self.current_position()
            error = target - actual

            error_xy = math.sqrt(error[0] ** 2 + error[1] ** 2)
            error_3d = float(np.linalg.norm(error))

            if error_3d <= self.position_tolerance_m:
                self.get_logger().info(f"Segment target reached. error={error_3d:.3f} m")
                self.hold_position(self.corner_hold_s)
                return True

            if time.time() - start_time > timeout_s:
                self.get_logger().warn(f"Segment timeout. error={error_3d:.3f} m")
                self.hold_position(self.corner_hold_s)
                return False

            if nominal_direction is not None:
                v_local = nominal_direction * nominal_speed
            else:
                v_local = np.zeros(3)

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
            time.sleep(period)

    def execute_move(self, item):
        move = str(item.get("move", "")).lower()
        distance = float(item.get("distance_m", 0.0))

        if move == "forward":
            direction = np.array([math.cos(self.current_heading), math.sin(self.current_heading), 0.0])
            target = self.current_target + direction * distance

        elif move == "backward":
            direction = np.array([math.cos(self.current_heading), math.sin(self.current_heading), 0.0])
            target = self.current_target - direction * distance
            direction = -direction

        elif move == "left":
            direction = np.array([-math.sin(self.current_heading), math.cos(self.current_heading), 0.0])
            target = self.current_target + direction * distance

        elif move == "right":
            direction = np.array([-math.sin(self.current_heading), math.cos(self.current_heading), 0.0])
            target = self.current_target - direction * distance
            direction = -direction

        elif move == "up":
            direction = np.array([0.0, 0.0, 1.0])
            target = self.current_target + direction * distance

        elif move == "down":
            direction = np.array([0.0, 0.0, -1.0])
            target = self.current_target + direction * distance

        else:
            raise ValueError(f"Unsupported move command: {move}")

        ok = self.move_to_target(target, nominal_direction=direction, nominal_speed=self.speed)
        self.current_target = target.copy()
        return ok

    def execute_turn(self, item):
        turn = str(item.get("turn", "")).lower()
        angle_deg = abs(float(item.get("angle_deg", 0.0)))

        if turn in ["left", "ccw"]:
            self.current_heading = wrap_angle(self.current_heading + deg_to_rad(angle_deg))
        elif turn in ["right", "cw"]:
            self.current_heading = wrap_angle(self.current_heading - deg_to_rad(angle_deg))
        else:
            raise ValueError(f"Unsupported turn command: {turn}")

        self.get_logger().info(f"Turn command: {turn} {angle_deg:.1f} deg. New internal heading={math.degrees(self.current_heading):.1f} deg")
        self.hold_position(self.corner_hold_s)
        return True

    def execute_arc(self, item):
        arc = item.get("arc", {})

        radius = float(arc.get("radius_m", 1.0))
        angle_deg = float(arc.get("angle_deg", 90.0))
        direction = str(arc.get("direction", "left")).lower()

        sign = 1.0 if direction in ["left", "ccw"] else -1.0
        angle_rad = abs(deg_to_rad(angle_deg))

        if radius <= 0.0 or angle_rad <= 0.0:
            return True

        forward = np.array([math.cos(self.current_heading), math.sin(self.current_heading), 0.0])
left = np.array([-math.sin(self.current_heading), math.cos(self.current_heading), 0.0])

        center_offset_x = arc.get("center_offset_x_m", None)
        center_offset_y = arc.get("center_offset_y_m", None)

        if center_offset_x is not None and center_offset_y is not None:
            center = self.current_target + np.array([
                float(center_offset_x),
                float(center_offset_y),
                0.0,
            ])
        else:
            center = self.current_target + sign * radius * left

        radial_start = self.current_target - center
        start_angle = math.atan2(radial_start[1], radial_start[0])

        arc_length = radius * angle_rad
        duration = arc_length / max(self.speed, 1e-6)

        self.get_logger().info(
            f"Arc segment: radius={radius:.2f} m, angle={angle_deg:.1f} deg, "
            f"duration={duration:.1f} s"
        )

        start_time = time.time()
        period = 1.0 / 30.0

        while rclpy.ok():
            if self.abort_requested:
                self.publish_stop()
                return False

            rclpy.spin_once(self, timeout_sec=0.0)

            elapsed = time.time() - start_time
            active_t = min(elapsed, duration)
            progress = active_t / max(duration, 1e-6)

            angle = start_angle + sign * angle_rad * progress

            target = np.array([
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
                self.current_target[2],
            ])

            dangle_dt = sign * angle_rad / max(duration, 1e-6)

            nominal_vel = np.array([
                -radius * math.sin(angle) * dangle_dt,
                radius * math.cos(angle) * dangle_dt,
                0.0,
            ])

            actual = self.current_position()
            error = target - actual

            v_local = nominal_vel.copy()
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

            if elapsed >= duration:
                break

            time.sleep(period)

        end_angle = start_angle + sign * angle_rad
        self.current_target = np.array([
            center[0] + radius * math.cos(end_angle),
            center[1] + radius * math.sin(end_angle),
            self.current_target[2],
        ])

        self.current_heading = wrap_angle(self.current_heading + sign * angle_rad)
        self.hold_position(self.corner_hold_s)
        return True

    def execute_hold(self, item):
        hold = item.get("hold", {})
        duration = float(hold.get("duration_s", self.corner_hold_s))
        self.get_logger().info(f"Hold segment: {duration:.2f} s")
        self.hold_position(duration)
        return True

    def rotate_to_yaw(self, target_yaw):
        yaw_tolerance = math.radians(2.0)
        kp_yaw = 1.0
        max_yaw_rate = math.radians(30.0)

        self.get_logger().info(
            f"Rotating to start yaw: {math.degrees(target_yaw):.1f} deg"
        )

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            current_yaw = self.current_yaw()
            error = wrap_angle(target_yaw - current_yaw)

            if abs(error) <= yaw_tolerance:
                self.publish_stop()
                self.get_logger().info(
                    f"Start yaw restored. Error={math.degrees(error):.2f} deg"
                )
                return True

            yaw_rate = clamp(
                kp_yaw * error,
                -max_yaw_rate,
                max_yaw_rate,
            )

            msg = Twist()
            msg.angular.z = float(yaw_rate)
            self.pub.publish(msg)

            time.sleep(1.0 / 30.0)

        return False

    def return_to_start(self):
        if self.pose is None:
            self.get_logger().warn("Cannot return to start: no pose available.")
            return False

        self.returning_home = True
        self.abort_requested = False

        self.get_logger().info(
            "Returning to path start position: "
            f"({self.path_start_position[0]:.3f}, "
            f"{self.path_start_position[1]:.3f}, "
            f"{self.path_start_position[2]:.3f})"
        )

        try:
            ok = self.move_to_target(
                self.path_start_position,
                nominal_direction=None,
                nominal_speed=self.speed,
            )

            self.publish_stop()
            self.hold_position(self.finish_hold_s)

            if ok:
                self.get_logger().info("Returned to path start position.")
            else:
                self.get_logger().warn("Return to start finished with timeout.")

            return ok
        finally:
            self.returning_home = False

    def run(self):
        self.wait_for_pose()

        # The YAML path is relative to the UAV position at execution start.
        # This avoids depending on an absolute /uav/local_pose origin.
        self.path_start_position = self.current_position().copy()
        self.path_start_yaw = self.current_yaw()
        self.current_target = self.path_start_position.copy()
        self.current_heading = 0.0

        self.get_logger().info(
            "Starting generic segment execution from current local pose: "
            f"({self.path_start_position[0]:.3f}, "
            f"{self.path_start_position[1]:.3f}, "
            f"{self.path_start_position[2]:.3f})"
        )

        for i, item in enumerate(self.path_items, start=1):
            if self.abort_requested:
                self.get_logger().warn("Path execution aborted.")
                break

            self.get_logger().info(f"Executing segment {i}: {item}")

            if "move" in item:
                self.execute_move(item)
            elif "turn" in item:
                self.execute_turn(item)
            elif "arc" in item:
                self.execute_arc(item)
            elif "hold" in item:
                self.execute_hold(item)
            else:
                raise ValueError(f"Unsupported path item: {item}")

            if self.abort_requested:
                self.get_logger().warn("Path execution aborted.")
                break

        self.hold_position(self.finish_hold_s)

        if self.abort_requested:
            self.get_logger().warn("Generic segment path aborted.")
        else:
            self.get_logger().info("Generic segment path finished.")

        self.return_to_start()


def main():
    if len(sys.argv) != 2:
        print("Usage: execute_path_from_local_pose.py <path_spec.yaml>")
        sys.exit(1)

    spec = load_spec(Path(sys.argv[1]))

    if str(spec.get("type", "")).lower() != "segment_path":
        print("[ERROR] This executor expects type: segment_path")
        print("Use segment_circle_3m.yaml, segment_square_3m.yaml, or another segment_path YAML.")
        sys.exit(1)

    rclpy.init()
    node = SegmentExecutor(spec)

    def handle_sigint(signum, frame):
        if node.returning_home:
            print("\n[INFO] Ctrl+C received during return-to-start. Stopping immediately.")
            node.publish_stop()
            raise KeyboardInterrupt

        if node.abort_requested:
            print("\n[INFO] Second Ctrl+C received. Stopping immediately.")
            node.publish_stop()
            raise KeyboardInterrupt

        print("\n[INFO] Ctrl+C received. Aborting path and returning to start.")
        node.request_abort()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        node.run()
    except KeyboardInterrupt:
        node.publish_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
