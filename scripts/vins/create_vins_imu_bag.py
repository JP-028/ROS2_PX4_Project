#!/usr/bin/env python3

import argparse
import shutil
from pathlib import Path

import rosbag2_py
from builtin_interfaces.msg import Time
from px4_msgs.msg import SensorCombined
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, Imu


IMAGE_TOPICS = {
    "/stereo/left/image_raw",
    "/stereo/right/image_raw",
}
PX4_IMU_TOPIC = "/fmu/out/sensor_combined"
OUTPUT_IMU_TOPIC = "/vins/imu"


def ns_to_time(timestamp_ns: int) -> Time:
    stamp = Time()
    stamp.sec = timestamp_ns // 1_000_000_000
    stamp.nanosec = timestamp_ns % 1_000_000_000
    return stamp


def create_topic_metadata(
    name: str,
    type_name: str,
    serialization_format: str = "cdr",
):
    try:
        return rosbag2_py.TopicMetadata(
            id=0,
            name=name,
            type=type_name,
            serialization_format=serialization_format,
            offered_qos_profiles="",
        )
    except TypeError:
        return rosbag2_py.TopicMetadata(
            name=name,
            type=type_name,
            serialization_format=serialization_format,
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Create a VINS-ready rosbag from stereo images and PX4 "
            "SensorCombined IMU data."
        )
    )
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("output_bag", type=Path)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Delete the output bag first if it already exists.",
    )
    args = parser.parse_args()

    input_bag = args.input_bag.resolve()
    output_bag = args.output_bag.resolve()

    if not input_bag.is_dir():
        raise FileNotFoundError(f"Input bag not found: {input_bag}")

    if output_bag.exists():
        if not args.overwrite:
            raise FileExistsError(
                f"Output already exists: {output_bag}\n"
                "Use --overwrite only when replacement is intended."
            )
        shutil.rmtree(output_bag)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(input_bag),
            storage_id="sqlite3",
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    required_topics = IMAGE_TOPICS | {PX4_IMU_TOPIC}
    missing_topics = required_topics - set(topic_types)

    if missing_topics:
        raise RuntimeError(
            "Required topics missing from input bag: "
            + ", ".join(sorted(missing_topics))
        )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(
            uri=str(output_bag),
            storage_id="sqlite3",
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )

    for topic in sorted(IMAGE_TOPICS):
        writer.create_topic(
            create_topic_metadata(
                topic,
                topic_types[topic],
            )
        )

    writer.create_topic(
        create_topic_metadata(
            OUTPUT_IMU_TOPIC,
            "sensor_msgs/msg/Imu",
        )
    )

    first_bag_timestamp_ns = None
    counts = {
        "/stereo/left/image_raw": 0,
        "/stereo/right/image_raw": 0,
        OUTPUT_IMU_TOPIC: 0,
    }

    first_output_ns = {}
    last_output_ns = {}

    while reader.has_next():
        topic, serialized_data, bag_timestamp_ns = reader.read_next()

        if topic not in required_topics:
            continue

        if first_bag_timestamp_ns is None:
            first_bag_timestamp_ns = bag_timestamp_ns

        normalized_ns = bag_timestamp_ns - first_bag_timestamp_ns

        if topic in IMAGE_TOPICS:
            image_type = get_message(topic_types[topic])
            image_msg: Image = deserialize_message(
                serialized_data,
                image_type,
            )

            image_msg.header.stamp = ns_to_time(normalized_ns)

            writer.write(
                topic,
                serialize_message(image_msg),
                normalized_ns,
            )
            output_topic = topic

        else:
            px4_msg: SensorCombined = deserialize_message(
                serialized_data,
                SensorCombined,
            )

            imu_msg = Imu()
            imu_msg.header.stamp = ns_to_time(normalized_ns)
            imu_msg.header.frame_id = "imu_link"

            # PX4 SensorCombined is FRD:
            #   x forward, y right, z down
            #
            # Convert to ROS FLU:
            #   x forward, y left, z up
            imu_msg.angular_velocity.x = float(px4_msg.gyro_rad[0])
            imu_msg.angular_velocity.y = -float(px4_msg.gyro_rad[1])
            imu_msg.angular_velocity.z = -float(px4_msg.gyro_rad[2])

            imu_msg.linear_acceleration.x = float(
                px4_msg.accelerometer_m_s2[0]
            )
            imu_msg.linear_acceleration.y = -float(
                px4_msg.accelerometer_m_s2[1]
            )
            imu_msg.linear_acceleration.z = -float(
                px4_msg.accelerometer_m_s2[2]
            )

            # Orientation is unavailable in SensorCombined.
            imu_msg.orientation_covariance[0] = -1.0

            writer.write(
                OUTPUT_IMU_TOPIC,
                serialize_message(imu_msg),
                normalized_ns,
            )
            output_topic = OUTPUT_IMU_TOPIC

        counts[output_topic] += 1
        first_output_ns.setdefault(output_topic, normalized_ns)
        last_output_ns[output_topic] = normalized_ns

    print()
    print("VINS-ready bag created")
    print("======================")
    print(f"Input:  {input_bag}")
    print(f"Output: {output_bag}")
    print()

    for topic in (
        "/stereo/left/image_raw",
        "/stereo/right/image_raw",
        OUTPUT_IMU_TOPIC,
    ):
        count = counts[topic]
        if count:
            duration = (
                last_output_ns[topic] - first_output_ns[topic]
            ) / 1e9
        else:
            duration = 0.0

        print(
            f"{topic}: {count} messages, "
            f"duration {duration:.3f} s"
        )


if __name__ == "__main__":
    main()
