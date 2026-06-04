#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import PoseStamped


class Px4OdometryToLocalPose(Node):
    def __init__(self):
        super().__init__("px4_odometry_to_local_pose")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.sub = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos,
        )

        self.pub = self.create_publisher(
            PoseStamped,
            "/uav/local_pose",
            10,
        )

        self.origin_position = None

        self.get_logger().info("PX4 odometry to local pose adapter started.")
        self.get_logger().info("Input:  /fmu/out/vehicle_odometry")
        self.get_logger().info("Output: /uav/local_pose")
        self.get_logger().info("The first received odometry sample becomes local origin (0, 0, 0).")

    def odom_callback(self, msg):
        x = float(msg.position[0])
        y = float(msg.position[1])
        z_ned = float(msg.position[2])

        if not all(math.isfinite(v) for v in [x, y, z_ned]):
            return

        if self.origin_position is None:
            self.origin_position = (x, y, z_ned)
            self.get_logger().info("Local origin initialized from PX4 odometry.")

        ox, oy, oz_ned = self.origin_position

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "local_start"

        out.pose.position.x = x - ox
        out.pose.position.y = y - oy

        # PX4 odometry usually uses NED coordinates:
        # positive z = down.
        # For /uav/local_pose we use positive z = up.
        out.pose.position.z = -(z_ned - oz_ned)

        # PX4 VehicleOdometry quaternion order is q = [w, x, y, z].
        # ROS geometry_msgs Quaternion order is x, y, z, w.
        out.pose.orientation.w = float(msg.q[0])
        out.pose.orientation.x = float(msg.q[1])
        out.pose.orientation.y = float(msg.q[2])
        out.pose.orientation.z = float(msg.q[3])

        self.pub.publish(out)


def main():
    rclpy.init()
    node = Px4OdometryToLocalPose()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
