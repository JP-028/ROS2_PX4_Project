#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CircleFlightNode(Node):
    def __init__(self):
        super().__init__("circle_flight_node")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Target: circle path with 5 m diameter
        self.diameter = 5.0
        self.radius = self.diameter / 2.0

        # Low speed to reduce pitch angle and keep front camera usable
        self.forward_speed = 0.3

        # Circle relation: yaw_rate = v / r
        self.yaw_rate = self.forward_speed / self.radius

        # One full circle duration: circumference / speed
        self.circle_duration = (2.0 * math.pi * self.radius) / self.forward_speed

        self.takeoff_duration = 3.0
        self.pause_duration = 1.0
        self.final_stop_duration = 2.0

        self.publish_rate = 20.0
        self.dt = 1.0 / self.publish_rate

        self.phase = "takeoff"
        self.phase_start_time = time.time()
        self.finished = False

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info("Circle Flight Node started")
        self.get_logger().info("Target: 5 m diameter circle path")
        self.get_logger().info("Low-speed velocity-based maneuver for front camera visibility")
        self.get_logger().info(f"Target diameter: {self.diameter:.2f} m")
        self.get_logger().info(f"Radius: {self.radius:.2f} m")
        self.get_logger().info(f"Forward speed: {self.forward_speed:.2f} m/s")
        self.get_logger().info(f"Yaw rate: {self.yaw_rate:.3f} rad/s")
        self.get_logger().info(f"Circle duration: {self.circle_duration:.2f} s")

    def publish_cmd(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)
        msg.angular.z = float(yaw)
        self.publisher.publish(msg)

    def switch_phase(self, new_phase):
        self.phase = new_phase
        self.phase_start_time = time.time()
        self.get_logger().info(f"Switching to phase: {new_phase}")

    def timer_callback(self):
        if self.finished:
            return

        elapsed = time.time() - self.phase_start_time

        if self.phase == "takeoff":
            self.publish_cmd(z=0.5)

            if elapsed >= self.takeoff_duration:
                self.publish_cmd()
                self.switch_phase("pause_before_circle")

        elif self.phase == "pause_before_circle":
            self.publish_cmd()

            if elapsed >= self.pause_duration:
                self.switch_phase("circle")

        elif self.phase == "circle":
            self.publish_cmd(
                x=self.forward_speed,
                y=0.0,
                z=0.0,
                yaw=self.yaw_rate
            )

            if elapsed >= self.circle_duration:
                self.publish_cmd()
                self.switch_phase("final_stop")

        elif self.phase == "final_stop":
            self.publish_cmd()

            if elapsed >= self.final_stop_duration:
                self.get_logger().info("Circle flight finished.")
                self.finished = True


def main(args=None):
    rclpy.init(args=args)

    node = CircleFlightNode()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info("Circle Flight Node stopped manually.")

    finally:
        if rclpy.ok():
            node.publish_cmd()
            time.sleep(0.2)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
