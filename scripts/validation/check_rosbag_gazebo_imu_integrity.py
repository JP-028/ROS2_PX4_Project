#!/usr/bin/env python3

import argparse
import math
import statistics
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu


IMU_TOPIC = "/gazebo/imu"
MAX_ALLOWED_GAP_MS = 20.0
MIN_REQUIRED_MESSAGES = 100


def open_reader(bag_dir: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(bag_dir),
            storage_id="sqlite3",
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def percentile(values, fraction):
    ordered = sorted(values)
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def stamp_to_ns(msg: Imu) -> int:
    return (
        int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec)
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("report_file", type=Path)
    args = parser.parse_args()

    bag_dir = args.bag_dir.resolve()
    report_file = args.report_file.resolve()

    reader = open_reader(bag_dir)

    timestamps_ns = []
    accel_norms = []
    gyro_norms = []

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic != IMU_TOPIC:
            continue

        msg = deserialize_message(data, Imu)
        timestamps_ns.append(stamp_to_ns(msg))

        ax = float(msg.linear_acceleration.x)
        ay = float(msg.linear_acceleration.y)
        az = float(msg.linear_acceleration.z)

        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)

        accel_norms.append(
            math.sqrt(ax * ax + ay * ay + az * az)
        )
        gyro_norms.append(
            math.sqrt(gx * gx + gy * gy + gz * gz)
        )

    lines = [
        "ROSbag Gazebo IMU Integrity Report",
        "==================================",
        f"Bag: {bag_dir}",
        f"Topic: {IMU_TOPIC}",
        "",
    ]

    if len(timestamps_ns) < 2:
        lines.extend(
            [
                "Status: FAIL",
                f"Messages: {len(timestamps_ns)}",
                "Reason: Fewer than two IMU messages were recorded.",
            ]
        )

        report_file.write_text(
            "\n".join(lines) + "\n",
            encoding="utf-8",
        )

        print("\n".join(lines))
        return 2

    intervals_ms = [
        (current - previous) / 1_000_000.0
        for previous, current in zip(
            timestamps_ns[:-1],
            timestamps_ns[1:],
        )
    ]

    positive = [value for value in intervals_ms if value > 0]
    nonpositive = sum(value <= 0 for value in intervals_ms)

    duration_s = (
        timestamps_ns[-1] - timestamps_ns[0]
    ) / 1_000_000_000.0

    rate_hz = (
        (len(timestamps_ns) - 1) / duration_s
        if duration_s > 0
        else 0.0
    )

    gaps_above_limit = sum(
        value > MAX_ALLOWED_GAP_MS
        for value in positive
    )

    failure_reasons = []

    if len(timestamps_ns) < MIN_REQUIRED_MESSAGES:
        failure_reasons.append(
            f"Only {len(timestamps_ns)} messages were recorded."
        )

    if nonpositive > 0:
        failure_reasons.append(
            f"{nonpositive} non-positive timestamp intervals found."
        )

    if gaps_above_limit > 0:
        failure_reasons.append(
            f"{gaps_above_limit} gaps exceeded "
            f"{MAX_ALLOWED_GAP_MS:.0f} ms."
        )

    status = "PASS" if not failure_reasons else "FAIL"

    lines.extend(
        [
            f"Status: {status}",
            "",
            "Stream summary",
            "--------------",
            f"Messages: {len(timestamps_ns)}",
            f"Timestamp duration: {duration_s:.3f} s",
            f"Header-based rate: {rate_hz:.3f} Hz",
            "",
            "Timestamp intervals",
            "-------------------",
            f"Minimum: {min(positive):.3f} ms",
            f"Median: {statistics.median(positive):.3f} ms",
            f"90th percentile: {percentile(positive, 0.90):.3f} ms",
            f"95th percentile: {percentile(positive, 0.95):.3f} ms",
            f"99th percentile: {percentile(positive, 0.99):.3f} ms",
            f"Maximum: {max(positive):.3f} ms",
            f"Non-positive intervals: {nonpositive}",
            f"Gaps above 20 ms: {sum(v > 20 for v in positive)}",
            f"Gaps above 100 ms: {sum(v > 100 for v in positive)}",
            f"Gaps above 500 ms: {sum(v > 500 for v in positive)}",
            "",
            "Measurement magnitudes",
            "----------------------",
            (
                "Acceleration norm: "
                f"median={statistics.median(accel_norms):.5f} m/s^2, "
                f"min={min(accel_norms):.5f}, "
                f"max={max(accel_norms):.5f}"
            ),
            (
                "Angular-rate norm: "
                f"median={statistics.median(gyro_norms):.6f} rad/s, "
                f"min={min(gyro_norms):.6f}, "
                f"max={max(gyro_norms):.6f}"
            ),
        ]
    )

    if failure_reasons:
        lines.extend(
            [
                "",
                "Failure reasons",
                "---------------",
                *[f"- {reason}" for reason in failure_reasons],
            ]
        )

    report_file.parent.mkdir(parents=True, exist_ok=True)
    report_file.write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )

    print("\n".join(lines))
    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
