#!/usr/bin/env python3

import argparse
import math
import statistics
from pathlib import Path

import rosbag2_py
from px4_msgs.msg import SensorCombined
from rclpy.serialization import deserialize_message


IMU_TOPIC = "/fmu/out/sensor_combined"

# A gap above this limit makes a run unsuitable for VINS-Fusion.
MAX_ALLOWED_GAP_MS = 500.0
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
    if not values:
        return float("nan")

    ordered = sorted(values)
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check PX4 SensorCombined continuity in a rosbag."
    )
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("report_file", type=Path)
    args = parser.parse_args()

    bag_dir = args.bag_dir.resolve()
    report_file = args.report_file.resolve()

    if not bag_dir.is_dir():
        raise FileNotFoundError(f"Rosbag not found: {bag_dir}")

    reader = open_reader(bag_dir)

    timestamps_us = []
    accel_norms = []
    gyro_norms = []
    gyro_integral_dts = []
    accel_integral_dts = []
    accel_clipping_count = 0
    gyro_clipping_count = 0

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic != IMU_TOPIC:
            continue

        msg = deserialize_message(data, SensorCombined)

        timestamps_us.append(int(msg.timestamp))
        gyro_integral_dts.append(int(msg.gyro_integral_dt))
        accel_integral_dts.append(
            int(msg.accelerometer_integral_dt)
        )

        ax, ay, az = map(float, msg.accelerometer_m_s2)
        gx, gy, gz = map(float, msg.gyro_rad)

        accel_norms.append(
            math.sqrt(ax * ax + ay * ay + az * az)
        )
        gyro_norms.append(
            math.sqrt(gx * gx + gy * gy + gz * gz)
        )

        if msg.accelerometer_clipping:
            accel_clipping_count += 1

        if msg.gyro_clipping:
            gyro_clipping_count += 1

    report_lines = [
        "ROSbag IMU Integrity Report",
        "===========================",
        f"Bag: {bag_dir}",
        f"Topic: {IMU_TOPIC}",
        "",
    ]

    if len(timestamps_us) < 2:
        report_lines.extend(
            [
                "Status: FAIL",
                f"Messages: {len(timestamps_us)}",
                "Reason: Fewer than two IMU messages were recorded.",
            ]
        )

        report_file.parent.mkdir(parents=True, exist_ok=True)
        report_file.write_text(
            "\n".join(report_lines) + "\n",
            encoding="utf-8",
        )

        print("\n".join(report_lines))
        return 2

    intervals_ms = [
        (current - previous) / 1000.0
        for previous, current in zip(
            timestamps_us[:-1],
            timestamps_us[1:],
        )
    ]

    positive_intervals = [
        value for value in intervals_ms if value > 0
    ]

    nonpositive_count = sum(
        value <= 0 for value in intervals_ms
    )

    duration_s = (
        timestamps_us[-1] - timestamps_us[0]
    ) / 1_000_000.0

    effective_rate_hz = (
        (len(timestamps_us) - 1) / duration_s
        if duration_s > 0
        else 0.0
    )

    maximum_gap_ms = (
        max(positive_intervals)
        if positive_intervals
        else float("inf")
    )

    large_gap_count = sum(
        value > MAX_ALLOWED_GAP_MS
        for value in positive_intervals
    )

    failure_reasons = []

    if len(timestamps_us) < MIN_REQUIRED_MESSAGES:
        failure_reasons.append(
            f"Only {len(timestamps_us)} IMU messages were recorded."
        )

    if nonpositive_count > 0:
        failure_reasons.append(
            f"{nonpositive_count} non-positive timestamp intervals found."
        )

    if large_gap_count > 0:
        failure_reasons.append(
            f"{large_gap_count} IMU gaps exceeded "
            f"{MAX_ALLOWED_GAP_MS:.0f} ms."
        )

    status = "PASS" if not failure_reasons else "FAIL"

    report_lines.extend(
        [
            f"Status: {status}",
            "",
            "Stream summary",
            "--------------",
            f"Messages: {len(timestamps_us)}",
            f"Timestamp duration: {duration_s:.3f} s",
            f"Effective average rate: {effective_rate_hz:.3f} Hz",
            "",
            "Timestamp intervals",
            "-------------------",
            f"Minimum: {min(positive_intervals):.3f} ms",
            f"Median: {statistics.median(positive_intervals):.3f} ms",
            f"90th percentile: {percentile(positive_intervals, 0.90):.3f} ms",
            f"95th percentile: {percentile(positive_intervals, 0.95):.3f} ms",
            f"99th percentile: {percentile(positive_intervals, 0.99):.3f} ms",
            f"Maximum: {maximum_gap_ms:.3f} ms",
            f"Non-positive intervals: {nonpositive_count}",
            f"Gaps above 20 ms: {sum(value > 20 for value in positive_intervals)}",
            f"Gaps above 50 ms: {sum(value > 50 for value in positive_intervals)}",
            f"Gaps above 100 ms: {sum(value > 100 for value in positive_intervals)}",
            f"Gaps above 500 ms: {sum(value > 500 for value in positive_intervals)}",
            "",
            "Reported integration intervals",
            "------------------------------",
            (
                "Gyroscope: "
                f"median={statistics.median(gyro_integral_dts):.1f} us, "
                f"min={min(gyro_integral_dts)} us, "
                f"max={max(gyro_integral_dts)} us"
            ),
            (
                "Accelerometer: "
                f"median={statistics.median(accel_integral_dts):.1f} us, "
                f"min={min(accel_integral_dts)} us, "
                f"max={max(accel_integral_dts)} us"
            ),
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
            f"Accelerometer clipping messages: {accel_clipping_count}",
            f"Gyroscope clipping messages: {gyro_clipping_count}",
        ]
    )

    if failure_reasons:
        report_lines.extend(
            [
                "",
                "Failure reasons",
                "---------------",
                *[f"- {reason}" for reason in failure_reasons],
            ]
        )

    report_file.parent.mkdir(parents=True, exist_ok=True)
    report_file.write_text(
        "\n".join(report_lines) + "\n",
        encoding="utf-8",
    )

    print("\n".join(report_lines))

    return 0 if status == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
