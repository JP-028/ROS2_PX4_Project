#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SquareFlightNode(Node):
    def __init__(self):
        super().__init__("square_flight_node")

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.side_length = 3.0
        self.forward_speed = 0.6

        self.yaw_speed = math.pi / 4.0      # 45 deg/s
        self.turn_angle = math.pi / 2.0     # 90 deg

        self.forward_duration = self.side_length / self.forward_speed
        self.turn_duration = self.turn_angle / self.yaw_speed

        self.pause_duration = 0.7

        self.publish_rate = 20.0
        self.dt = 1.0 / self.publish_rate

        self.phases = [
            ("takeoff", 0.0, 0.0, 0.5, 0.0, 3.0),
            ("pause_after_takeoff", 0.0, 0.0, 0.0, 0.0, self.pause_duration),

            ("side_1_forward", self.forward_speed, 0.0, 0.0, 0.0, self.forward_duration),
            ("pause_1", 0.0, 0.0, 0.0, 0.0, self.pause_duration),
            ("turn_1_left_90", 0.0, 0.0, 0.0, self.yaw_speed, self.turn_duration),
            ("pause_2", 0.0, 0.0, 0.0, 0.0, self.pause_duration),

            ("side_2_forward", self.forward_speed, 0.0, 0.0, 0.0, self.forward_duration),
            ("pause_3", 0.0, 0.0, 0.0, 0.0, self.pause_duration),
            ("turn_2_left_90", 0.0, 0.0, 0.0, self.yaw_speed, self.turn_duration),
            ("pause_4", 0.0, 0.0, 0.0, 0.0, self.pause_duration),

            ("side_3_forward", self.forward_speed, 0.0, 0.0, 0.0, self.forward_duration),
            ("pause_5", 0.0, 0.0, 0.0, 0.0, self.pause_duration),
            ("turn_3_left_90", 0.0, 0.0, 0.0, self.yaw_speed, self.turn_duration),
            ("pause_6", 0.0, 0.0, 0.0, 0.0, self.pause_duration),

            ("side_4_forward", self.forward_speed, 0.0, 0.0, 0.0, self.forward_duration),
            ("final_stop", 0.0, 0.0, 0.0, 0.0, 2.0),
        ]

        self.phase_index = 0
        self.phase_start_time = time.time()
        self.finished = False

        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info("Square Flight Node gestartet")
        self.get_logger().info("Klassischer Ablauf:")
        self.get_logger().info("geradeaus -> 90 Grad links -> geradeaus -> 90 Grad links ...")
        self.get_logger().info(f"Seitenlänge: {self.side_length:.2f} m")
        self.get_logger().info(f"Forward speed: {self.forward_speed:.2f} m/s")
        self.get_logger().info(f"Forward duration: {self.forward_duration:.2f} s")
        self.get_logger().info(f"Turn duration: {self.turn_duration:.2f} s")

    def publish_cmd(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)
        msg.angular.z = float(yaw)
        self.publisher.publish(msg)

    def timer_callback(self):
        if self.finished:
            return

        if self.phase_index >= len(self.phases):
            self.publish_cmd()
            self.get_logger().info("Quadratflug beendet.")
            self.finished = True
            return

        phase_name, x, y, z, yaw, duration = self.phases[self.phase_index]
        elapsed = time.time() - self.phase_start_time

        self.publish_cmd(x=x, y=y, z=z, yaw=yaw)

        if elapsed >= duration:
            self.publish_cmd()
            self.get_logger().info(f"Phase fertig: {phase_name}")

            self.phase_index += 1
            self.phase_start_time = time.time()

            if self.phase_index < len(self.phases):
                next_phase = self.phases[self.phase_index][0]
                self.get_logger().info(f"Wechsel zu: {next_phase}")


def main(args=None):
    rclpy.init(args=args)

    node = SquareFlightNode()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Square Flight manuell beendet.")
    finally:
        if rclpy.ok():
            node.publish_cmd()
            time.sleep(0.2)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
