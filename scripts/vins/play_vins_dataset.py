#!/usr/bin/env python3

import argparse
import statistics
import time
from pathlib import Path

import rclpy
import rosbag2_py
from builtin_interfaces.msg import Time
from px4_msgs.msg import SensorCombined
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, Imu


LEFT_TOPIC = "/stereo/left/image_raw"
RIGHT_TOPIC = "/stereo/right/image_raw"
PX4_IMU_TOPIC = "/fmu/out/sensor_combined"
VINS_IMU_TOPIC = "/vins/imu"


def stamp_to_ns(stamp) -> int:
    return (
        int(stamp.sec) * 1_000_000_000
        + int(stamp.nanosec)
    )


def ns_to_stamp(value_ns: int) -> Time:
    result = Time()
    result.sec = value_ns // 1_000_000_000
    result.nanosec = value_ns % 1_000_000_000
    return result


def open_reader(bag: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag),
            storage_id="sqlite3",
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def determine_clock_alignment(bag: Path):
    reader = open_reader(bag)

    topic_types = {
        entry.name: entry.type
        for entry in reader.get_all_topics_and_types()
    }

    camera_offsets = []
    imu_offsets = []

    print("[INFO] Determining camera/IMU clock alignment...")

    while reader.has_next():
        topic, data, bag_ns = reader.read_next()

        if topic in (LEFT_TOPIC, RIGHT_TOPIC):
            msg_type = get_message(topic_types[topic])
            image = deserialize_message(data, msg_type)
            image_ns = stamp_to_ns(image.header.stamp)

            if image_ns > 0:
                camera_offsets.append(bag_ns - image_ns)

        elif topic == PX4_IMU_TOPIC:
            imu = deserialize_message(data, SensorCombined)
            px4_ns = int(imu.timestamp) * 1000
            imu_offsets.append(px4_ns - bag_ns)

        # A limited sample is sufficient because both relationships
        # are fixed clock offsets. Do not scan the complete large bag.
        if (
            len(camera_offsets) >= 100
            and len(imu_offsets) >= 500
        ):
            break

    print(
        "[OK] Alignment samples collected: "
        f"camera={len(camera_offsets)}, "
        f"imu={len(imu_offsets)}"
    )

    if not camera_offsets:
        raise RuntimeError("No valid camera timestamps found.")

    if not imu_offsets:
        raise RuntimeError("No PX4 IMU timestamps found.")

    bag_minus_camera_ns = int(
        statistics.median(camera_offsets)
    )
    px4_minus_bag_ns = int(
        statistics.median(imu_offsets)
    )

    px4_minus_camera_ns = (
        px4_minus_bag_ns + bag_minus_camera_ns
    )

    return px4_minus_camera_ns


class VinsDatasetPlayer(Node):
    def __init__(self, bag: Path, rate: float) -> None:
        super().__init__("vins_dataset_player")

        self.bag = bag
        self.rate = rate

        image_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        imu_qos = QoSProfile(
            depth=2000,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.left_pub = self.create_publisher(
            Image,
            LEFT_TOPIC,
            image_qos,
        )
        self.right_pub = self.create_publisher(
            Image,
            RIGHT_TOPIC,
            image_qos,
        )
        self.imu_pub = self.create_publisher(
            Imu,
            VINS_IMU_TOPIC,
            imu_qos,
        )

    def run(self) -> None:
        px4_minus_camera_ns = determine_clock_alignment(
            self.bag
        )

        self.get_logger().info(
            "PX4-to-camera clock offset: "
            f"{px4_minus_camera_ns / 1e9:.9f} s"
        )

        reader = open_reader(self.bag)

        topic_types = {
            entry.name: entry.type
            for entry in reader.get_all_topics_and_types()
        }

        first_bag_ns = None
        start_wall = None

        counts = {
            LEFT_TOPIC: 0,
            RIGHT_TOPIC: 0,
            VINS_IMU_TOPIC: 0,
        }

        while reader.has_next() and rclpy.ok():
            topic, data, bag_ns = reader.read_next()

            if topic not in (
                LEFT_TOPIC,
                RIGHT_TOPIC,
                PX4_IMU_TOPIC,
            ):
                continue

            if first_bag_ns is None:
                first_bag_ns = bag_ns
                start_wall = time.monotonic()

            target_elapsed = (
                (bag_ns - first_bag_ns) / 1e9 / self.rate
            )
            actual_elapsed = time.monotonic() - start_wall
            remaining = target_elapsed - actual_elapsed

            if remaining > 0:
                time.sleep(remaining)

            if topic in (LEFT_TOPIC, RIGHT_TOPIC):
                msg_type = get_message(topic_types[topic])
                image: Image = deserialize_message(
                    data,
                    msg_type,
                )

                # Preserve the original Gazebo camera timestamp.
                # Left and right matching frames therefore retain
                # exactly the same timestamp.
                if topic == LEFT_TOPIC:
                    self.left_pub.publish(image)
                else:
                    self.right_pub.publish(image)

                counts[topic] += 1

            else:
                px4 = deserialize_message(
                    data,
                    SensorCombined,
                )

                px4_ns = int(px4.timestamp) * 1000
                camera_time_ns = (
                    px4_ns - px4_minus_camera_ns
                )

                imu = Imu()
                imu.header.stamp = ns_to_stamp(
                    camera_time_ns
                )
                imu.header.frame_id = "imu_link"

                # PX4 FRD -> ROS/VINS FLU
                imu.angular_velocity.x = float(
                    px4.gyro_rad[0]
                )
                imu.angular_velocity.y = -float(
                    px4.gyro_rad[1]
                )
                imu.angular_velocity.z = -float(
                    px4.gyro_rad[2]
                )

                imu.linear_acceleration.x = float(
                    px4.accelerometer_m_s2[0]
                )
                imu.linear_acceleration.y = -float(
                    px4.accelerometer_m_s2[1]
                )
                imu.linear_acceleration.z = -float(
                    px4.accelerometer_m_s2[2]
                )

                imu.orientation_covariance[0] = -1.0

                self.imu_pub.publish(imu)
                counts[VINS_IMU_TOPIC] += 1

            rclpy.spin_once(self, timeout_sec=0.0)

            total = sum(counts.values())
            if total and total % 1000 == 0:
                self.get_logger().info(
                    "Published totals: "
                    f"left={counts[LEFT_TOPIC]}, "
                    f"right={counts[RIGHT_TOPIC]}, "
                    f"imu={counts[VINS_IMU_TOPIC]}"
                )

        self.get_logger().info(
            "Playback finished: "
            f"left={counts[LEFT_TOPIC]}, "
            f"right={counts[RIGHT_TOPIC]}, "
            f"imu={counts[VINS_IMU_TOPIC]}"
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
    )
    args = parser.parse_args()

    if not args.bag.is_dir():
        raise FileNotFoundError(args.bag)

    if args.rate <= 0:
        raise ValueError("--rate must be greater than zero.")

    rclpy.init()
    node = VinsDatasetPlayer(
        args.bag.resolve(),
        args.rate,
    )

    try:
        time.sleep(3.0)
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
