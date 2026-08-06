#!/usr/bin/env python3

import argparse
import time
from pathlib import Path
from typing import Optional, Tuple, Any

import rclpy
import rosbag2_py

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Imu


LEFT_TOPIC = "/stereo/left/image_raw"
RIGHT_TOPIC = "/stereo/right/image_raw"
IMU_TOPIC = "/gazebo/imu"

CAMERA_TOPICS = {LEFT_TOPIC, RIGHT_TOPIC}


def open_reader(path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(path),
            storage_id="sqlite3",
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def stamp_to_ns(msg) -> int:
    return (
        int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec)
    )


class CameraStream:
    def __init__(self, bag_path: Path):
        self.reader = open_reader(bag_path)

        self.topic_types = {
            item.name: get_message(item.type)
            for item in self.reader.get_all_topics_and_types()
        }

        for topic in CAMERA_TOPICS:
            if topic not in self.topic_types:
                raise RuntimeError(
                    f"Required camera topic missing from bag: {topic}"
                )

    def next_message(
        self,
    ) -> Optional[Tuple[int, str, Any]]:
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()

            if topic not in CAMERA_TOPICS:
                continue

            msg = deserialize_message(
                data,
                self.topic_types[topic],
            )

            return stamp_to_ns(msg), topic, msg

        return None


class ImuStream:
    def __init__(self, bag_path: Path):
        self.reader = open_reader(bag_path)

        topic_names = {
            item.name
            for item in self.reader.get_all_topics_and_types()
        }

        if IMU_TOPIC not in topic_names:
            raise RuntimeError(
                f"Required IMU topic missing from bag: {IMU_TOPIC}"
            )

    def next_message(
        self,
    ) -> Optional[Tuple[int, str, Imu]]:
        while self.reader.has_next():
            topic, data, _ = self.reader.read_next()

            if topic != IMU_TOPIC:
                continue

            msg = deserialize_message(data, Imu)
            return stamp_to_ns(msg), topic, msg

        return None


class SplitDatasetPlayer(Node):
    def __init__(
        self,
        sensor_bag: Path,
        imu_bag: Path,
        playback_rate: float,
    ):
        super().__init__("split_vins_dataset_player")

        self.camera_stream = CameraStream(sensor_bag)
        self.imu_stream = ImuStream(imu_bag)
        self.playback_rate = playback_rate

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2000,
        )

        image_type = self.camera_stream.topic_types[LEFT_TOPIC]

        self.left_publisher = self.create_publisher(
            image_type,
            LEFT_TOPIC,
            camera_qos,
        )

        self.right_publisher = self.create_publisher(
            image_type,
            RIGHT_TOPIC,
            camera_qos,
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            IMU_TOPIC,
            imu_qos,
        )

    def publish_dataset(self):
        next_camera = self.camera_stream.next_message()
        next_imu = self.imu_stream.next_message()

        if next_camera is None:
            raise RuntimeError("No stereo images found.")

        if next_imu is None:
            raise RuntimeError("No IMU messages found.")

        first_stamp_ns = min(
            next_camera[0],
            next_imu[0],
        )

        left_count = 0
        right_count = 0
        imu_count = 0

        self.get_logger().info(
            "Waiting 3 seconds for VINS subscriptions..."
        )

        wait_end = time.monotonic() + 3.0

        while time.monotonic() < wait_end:
            rclpy.spin_once(self, timeout_sec=0.05)

        wall_start = time.monotonic()

        while (
            next_camera is not None
            or next_imu is not None
        ):
            use_camera = (
                next_camera is not None
                and (
                    next_imu is None
                    or next_camera[0] <= next_imu[0]
                )
            )

            if use_camera:
                stamp_ns, topic, msg = next_camera

                target_wall_time = (
                    wall_start
                    + (
                        stamp_ns - first_stamp_ns
                    )
                    / 1_000_000_000.0
                    / self.playback_rate
                )

                while time.monotonic() < target_wall_time:
                    rclpy.spin_once(
                        self,
                        timeout_sec=0.001,
                    )

                if topic == LEFT_TOPIC:
                    self.left_publisher.publish(msg)
                    left_count += 1
                else:
                    self.right_publisher.publish(msg)
                    right_count += 1

                next_camera = (
                    self.camera_stream.next_message()
                )

            else:
                stamp_ns, _, msg = next_imu

                target_wall_time = (
                    wall_start
                    + (
                        stamp_ns - first_stamp_ns
                    )
                    / 1_000_000_000.0
                    / self.playback_rate
                )

                while time.monotonic() < target_wall_time:
                    rclpy.spin_once(
                        self,
                        timeout_sec=0.001,
                    )

                self.imu_publisher.publish(msg)
                imu_count += 1

                next_imu = self.imu_stream.next_message()

            if (
                imu_count > 0
                and imu_count % 2500 == 0
            ):
                self.get_logger().info(
                    "Published totals: "
                    f"left={left_count}, "
                    f"right={right_count}, "
                    f"imu={imu_count}"
                )

        self.get_logger().info(
            "Playback finished: "
            f"left={left_count}, "
            f"right={right_count}, "
            f"imu={imu_count}"
        )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Play split stereo and Gazebo IMU bags "
            "for VINS-Fusion."
        )
    )

    parser.add_argument(
        "sensor_bag",
        type=Path,
    )

    parser.add_argument(
        "imu_bag",
        type=Path,
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
    )

    args = parser.parse_args()

    if args.rate <= 0:
        raise ValueError("--rate must be greater than zero.")

    rclpy.init()

    node = SplitDatasetPlayer(
        args.sensor_bag.resolve(),
        args.imu_bag.resolve(),
        args.rate,
    )

    try:
        node.publish_dataset()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
