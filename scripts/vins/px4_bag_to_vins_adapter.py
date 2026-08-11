#!/usr/bin/env python3

import rclpy
from px4_msgs.msg import SensorCombined
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, Imu


INPUT_LEFT_TOPIC = "/dataset/stereo/left/image_raw"
INPUT_RIGHT_TOPIC = "/dataset/stereo/right/image_raw"
INPUT_IMU_TOPIC = "/dataset/fmu/out/sensor_combined"

OUTPUT_LEFT_TOPIC = "/stereo/left/image_raw"
OUTPUT_RIGHT_TOPIC = "/stereo/right/image_raw"
OUTPUT_IMU_TOPIC = "/vins/imu"


class Px4BagToVinsAdapter(Node):
    def __init__(self) -> None:
        super().__init__(
            "px4_bag_to_vins_adapter",
            parameter_overrides=[
                Parameter(
                    "use_sim_time",
                    Parameter.Type.BOOL,
                    True,
                )
            ],
        )

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=500,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        output_image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        output_imu_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.left_publisher = self.create_publisher(
            Image,
            OUTPUT_LEFT_TOPIC,
            output_image_qos,
        )
        self.right_publisher = self.create_publisher(
            Image,
            OUTPUT_RIGHT_TOPIC,
            output_image_qos,
        )
        self.imu_publisher = self.create_publisher(
            Imu,
            OUTPUT_IMU_TOPIC,
            output_imu_qos,
        )

        self.create_subscription(
            Image,
            INPUT_LEFT_TOPIC,
            self.left_callback,
            input_qos,
        )
        self.create_subscription(
            Image,
            INPUT_RIGHT_TOPIC,
            self.right_callback,
            input_qos,
        )
        self.create_subscription(
            SensorCombined,
            INPUT_IMU_TOPIC,
            self.imu_callback,
            input_qos,
        )

        self.left_count = 0
        self.right_count = 0
        self.imu_count = 0
        self.dropped_imu_before_sync = 0

        # Difference between rosbag /clock time and the original
        # Gazebo camera-header timeline.
        self.camera_clock_offsets_ns = []
        self.bag_to_camera_offset_ns = None

        # Difference between the PX4 microsecond clock and the
        # Gazebo camera timeline.
        self.px4_to_camera_offset_ns = None

        self.create_timer(5.0, self.print_status)

        self.get_logger().info("VINS dataset adapter started")
        self.get_logger().info(
            f"{INPUT_LEFT_TOPIC} -> {OUTPUT_LEFT_TOPIC}"
        )
        self.get_logger().info(
            f"{INPUT_RIGHT_TOPIC} -> {OUTPUT_RIGHT_TOPIC}"
        )
        self.get_logger().info(
            f"{INPUT_IMU_TOPIC} -> {OUTPUT_IMU_TOPIC}"
        )

    @staticmethod
    def stamp_to_ns(stamp) -> int:
        return (
            int(stamp.sec) * 1_000_000_000
            + int(stamp.nanosec)
        )

    def learn_camera_clock_offset(self, message: Image) -> None:
        if self.bag_to_camera_offset_ns is not None:
            return

        camera_ns = self.stamp_to_ns(message.header.stamp)
        bag_clock_ns = self.get_clock().now().nanoseconds

        if camera_ns <= 0 or bag_clock_ns <= 0:
            return

        self.camera_clock_offsets_ns.append(
            bag_clock_ns - camera_ns
        )

        if len(self.camera_clock_offsets_ns) >= 10:
            ordered = sorted(self.camera_clock_offsets_ns)
            middle = len(ordered) // 2

            if len(ordered) % 2:
                offset_ns = ordered[middle]
            else:
                offset_ns = (
                    ordered[middle - 1] + ordered[middle]
                ) // 2

            self.bag_to_camera_offset_ns = offset_ns

            self.get_logger().info(
                "Camera clock alignment initialized: "
                f"bag_to_camera_offset="
                f"{offset_ns / 1e9:.9f} s"
            )

    def left_callback(self, message: Image) -> None:
        # Keep the original Gazebo timestamp. Left and right images
        # already contain matching stereo timestamps in the bag.
        self.learn_camera_clock_offset(message)
        message.header.frame_id = "camera_left_link"

        self.left_publisher.publish(message)
        self.left_count += 1

    def right_callback(self, message: Image) -> None:
        # Preserve the recorded stereo timestamp.
        message.header.frame_id = "camera_right_link"

        self.right_publisher.publish(message)
        self.right_count += 1

    def imu_callback(self, message: SensorCombined) -> None:
        if self.bag_to_camera_offset_ns is None:
            self.dropped_imu_before_sync += 1
            return

        px4_ns = int(message.timestamp) * 1000

        # Establish the fixed PX4-to-camera offset once. Afterwards,
        # use the original PX4 timestamps so the IMU retains its exact
        # sampling intervals instead of callback-arrival jitter.
        if self.px4_to_camera_offset_ns is None:
            bag_clock_ns = self.get_clock().now().nanoseconds
            camera_now_ns = (
                bag_clock_ns - self.bag_to_camera_offset_ns
            )

            self.px4_to_camera_offset_ns = (
                px4_ns - camera_now_ns
            )

            self.get_logger().info(
                "PX4 IMU clock alignment initialized: "
                f"px4_to_camera_offset="
                f"{self.px4_to_camera_offset_ns / 1e9:.9f} s"
            )

        imu_stamp_ns = px4_ns - self.px4_to_camera_offset_ns

        imu = Imu()
        imu.header.stamp = rclpy.time.Time(
            nanoseconds=imu_stamp_ns
        ).to_msg()
        imu.header.frame_id = "imu_link"

        # PX4 SensorCombined: FRD
        #   x forward, y right, z down
        #
        # Converted VINS body frame: FLU
        #   x forward, y left, z up
        imu.angular_velocity.x = float(message.gyro_rad[0])
        imu.angular_velocity.y = -float(message.gyro_rad[1])
        imu.angular_velocity.z = -float(message.gyro_rad[2])

        imu.linear_acceleration.x = float(
            message.accelerometer_m_s2[0]
        )
        imu.linear_acceleration.y = -float(
            message.accelerometer_m_s2[1]
        )
        imu.linear_acceleration.z = -float(
            message.accelerometer_m_s2[2]
        )

        imu.orientation_covariance[0] = -1.0

        self.imu_publisher.publish(imu)
        self.imu_count += 1

    def print_status(self) -> None:
        self.get_logger().info(
            "Published totals: "
            f"left={self.left_count}, "
            f"right={self.right_count}, "
            f"imu={self.imu_count}, "
            f"imu_waiting_for_sync="
            f"{self.dropped_imu_before_sync}"
        )


def main() -> None:
    rclpy.init()
    node = Px4BagToVinsAdapter()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
