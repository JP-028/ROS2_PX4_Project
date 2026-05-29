#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class UavTfBroadcasterNode(Node):
    def __init__(self):
        super().__init__("uav_tf_broadcaster_node")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.publish_static_transforms()

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.vehicle_odometry_callback,
            px4_qos,
        )

        self.get_logger().info("UAV TF Broadcaster Node started")
        self.get_logger().info("Publishing TF tree: map -> odom -> base_link -> camera links")

    def publish_static_transforms(self):
        static_transforms = []

        # map -> odom
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        static_transforms.append(map_to_odom)

        # base_link -> front_camera_link
        front_camera = TransformStamped()
        front_camera.header.stamp = self.get_clock().now().to_msg()
        front_camera.header.frame_id = "base_link"
        front_camera.child_frame_id = "front_camera_link"
        front_camera.transform.translation.x = 0.18
        front_camera.transform.translation.y = 0.0
        front_camera.transform.translation.z = 0.02
        front_camera.transform.rotation.x = 0.0
        front_camera.transform.rotation.y = 0.0
        front_camera.transform.rotation.z = 0.0
        front_camera.transform.rotation.w = 1.0
        static_transforms.append(front_camera)

        # base_link -> down_camera_link
        down_camera = TransformStamped()
        down_camera.header.stamp = self.get_clock().now().to_msg()
        down_camera.header.frame_id = "base_link"
        down_camera.child_frame_id = "down_camera_link"
        down_camera.transform.translation.x = 0.0
        down_camera.transform.translation.y = 0.0
        down_camera.transform.translation.z = -0.05

        # Rotate camera roughly downward: -90 deg around Y axis.
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, -math.pi / 2.0, 0.0)
        down_camera.transform.rotation.x = qx
        down_camera.transform.rotation.y = qy
        down_camera.transform.rotation.z = qz
        down_camera.transform.rotation.w = qw
        static_transforms.append(down_camera)

        self.static_tf_broadcaster.sendTransform(static_transforms)

    def vehicle_odometry_callback(self, msg: VehicleOdometry):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"

        # PX4 local odometry is typically NED:
        # x = North, y = East, z = Down
        # ROS odom is ENU:
        # x = East, y = North, z = Up
        #
        # Minimal NED -> ENU position conversion:
        transform.transform.translation.x = float(msg.position[1])
        transform.transform.translation.y = float(msg.position[0])
        transform.transform.translation.z = float(-msg.position[2])

        # NOTE:
        # PX4 quaternion convention/order and NED->ENU attitude conversion can be subtle.
        # For this first working TF tree, we publish position correctly and use a neutral orientation.
        # This avoids publishing an incorrect orientation transform.
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw


def main(args=None):
    rclpy.init(args=args)

    node = UavTfBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UAV TF Broadcaster stopped manually.")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
