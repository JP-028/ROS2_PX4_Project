#!/usr/bin/env python3

import argparse
import csv
import math
import sqlite3
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


REFERENCE_TOPIC = "/uav/local_pose"


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def normalize_quaternion(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / n


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quaternion_to_rotation(q):
    x, y, z, w = normalize_quaternion(q)

    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w),     2 * (x*z + y*w)],
        [2 * (x*y + z*w),     1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w),     2 * (y*z + x*w),     1 - 2 * (x*x + y*y)],
    ])


def rotation_to_quaternion(R):
    trace = np.trace(R)

    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return normalize_quaternion([x, y, z, w])


def slerp(q0, q1, alpha):
    q0 = normalize_quaternion(q0)
    q1 = normalize_quaternion(q1)

    dot = np.dot(q0, q1)

    if dot < 0:
        q1 = -q1
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)

    if dot > 0.9995:
        return normalize_quaternion(q0 + alpha * (q1 - q0))

    theta = math.acos(dot)
    sin_theta = math.sin(theta)

    return (
        math.sin((1 - alpha) * theta) / sin_theta * q0
        + math.sin(alpha * theta) / sin_theta * q1
    )


def interpolate_positions(times, positions, targets):
    return np.column_stack([
        np.interp(targets, times, positions[:, i])
        for i in range(3)
    ])


def interpolate_quaternions(times, quaternions, targets):
    result = []

    for t in targets:
        j = np.searchsorted(times, t)

        if j <= 0:
            result.append(quaternions[0])
            continue

        if j >= len(times):
            result.append(quaternions[-1])
            continue

        t0 = times[j - 1]
        t1 = times[j]
        alpha = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)

        result.append(
            slerp(quaternions[j - 1], quaternions[j], alpha)
        )

    return np.asarray(result)


def load_reference(bag_dir, topic_name):
    db_files = sorted(Path(bag_dir).glob("*.db3"))

    if not db_files:
        raise RuntimeError(f"No rosbag database found in {bag_dir}")

    # Build mapping:
    # rosbag timestamp -> stereo camera header timestamp.
    camera_bag_times = []
    camera_sensor_times = []

    for db_file in db_files:
        connection = sqlite3.connect(str(db_file))

        try:
            topic = connection.execute(
                "SELECT id, type FROM topics WHERE name = ?",
                ("/stereo/left/image_raw",),
            ).fetchone()

            if topic is None:
                continue

            topic_id, type_name = topic
            message_class = get_message(type_name)

            rows = connection.execute(
                "SELECT timestamp, data FROM messages "
                "WHERE topic_id = ? ORDER BY timestamp",
                (topic_id,),
            )

            for bag_timestamp, data in rows:
                msg = deserialize_message(data, message_class)

                sensor_time = stamp_to_seconds(msg.header.stamp)

                if sensor_time > 0:
                    camera_bag_times.append(
                        bag_timestamp * 1e-9
                    )
                    camera_sensor_times.append(
                        sensor_time
                    )

        finally:
            connection.close()

    if len(camera_bag_times) < 2:
        raise RuntimeError(
            "Could not build bag-to-camera timestamp mapping."
        )

    camera_bag_times = np.asarray(
        camera_bag_times,
        dtype=float,
    )

    camera_sensor_times = np.asarray(
        camera_sensor_times,
        dtype=float,
    )

    order = np.argsort(camera_bag_times)

    camera_bag_times = camera_bag_times[order]
    camera_sensor_times = camera_sensor_times[order]

    times = []
    positions = []
    quaternions = []

    for db_file in db_files:
        connection = sqlite3.connect(str(db_file))

        try:
            topic = connection.execute(
                "SELECT id, type FROM topics WHERE name = ?",
                (topic_name,),
            ).fetchone()

            if topic is None:
                continue

            topic_id, type_name = topic
            message_class = get_message(type_name)

            rows = connection.execute(
                "SELECT timestamp, data FROM messages "
                "WHERE topic_id = ? ORDER BY timestamp",
                (topic_id,),
            )

            for bag_timestamp, data in rows:
                msg = deserialize_message(data, message_class)

                bag_time_s = bag_timestamp * 1e-9

                # Convert the reference bag timestamp onto the
                # Gazebo/camera sensor timeline.
                sensor_time_s = float(
                    np.interp(
                        bag_time_s,
                        camera_bag_times,
                        camera_sensor_times,
                    )
                )

                times.append(sensor_time_s)

                positions.append([
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ])

                quaternions.append([
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ])

        finally:
            connection.close()

    if not times:
        raise RuntimeError(
            f"No reference data found on {topic_name}"
        )

    order = np.argsort(times)

    return (
        np.asarray(times, dtype=float)[order],
        np.asarray(positions, dtype=float)[order],
        np.asarray(quaternions, dtype=float)[order],
    )

def load_estimate(path, estimator_format):
    times = []
    positions = []
    quaternions = []

    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()

            if not line:
                continue

            row = (
                [v.strip() for v in line.split(",")]
                if "," in line
                else line.split()
            )

            if len(row) < 8:
                continue

            try:
                values = [float(v) for v in row[:8]]
            except ValueError:
                continue

            t, x, y, z = values[:4]

            if estimator_format == "vins":
                qw, qx, qy, qz = values[4:8]
            else:
                qx, qy, qz, qw = values[4:8]

            times.append(t)
            positions.append([x, y, z])
            quaternions.append(
                normalize_quaternion([qx, qy, qz, qw])
            )

    if not times:
        raise RuntimeError(f"No valid estimator poses found in {path}")

    times = np.asarray(times, dtype=float)

    if len(times) > 1:
        dt = np.median(np.abs(np.diff(times)))

        if dt > 1e6:
            times *= 1e-9
        elif dt > 1e3:
            times *= 1e-6
        elif dt > 10:
            times *= 1e-3

    order = np.argsort(times)

    return (
        times[order],
        np.asarray(positions, dtype=float)[order],
        np.asarray(quaternions, dtype=float)[order],
    )


def associate_by_time(
    ref_t, ref_p, ref_q,
    est_t, est_p, est_q,
    samples,
):
    start = max(ref_t[0], est_t[0])
    end = min(ref_t[-1], est_t[-1])

    if end <= start:
        raise RuntimeError(
            "Reference and estimate timestamps do not overlap.\n"
            f"Reference: {ref_t[0]:.6f} .. {ref_t[-1]:.6f}\n"
            f"Estimate:  {est_t[0]:.6f} .. {est_t[-1]:.6f}"
        )

    targets = np.linspace(start, end, samples)

    return (
        targets,
        interpolate_positions(ref_t, ref_p, targets),
        interpolate_quaternions(ref_t, ref_q, targets),
        interpolate_positions(est_t, est_p, targets),
        interpolate_quaternions(est_t, est_q, targets),
    )


def align_positions(source, target, mode):
    src_mean = source.mean(axis=0)
    tgt_mean = target.mean(axis=0)

    src = source - src_mean
    tgt = target - tgt_mean

    if mode == "se3":
        u, _, vt = np.linalg.svd(src.T @ tgt)
        rotation = vt.T @ u.T

        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T

    else:
        u, _, vt = np.linalg.svd(src[:, :2].T @ tgt[:, :2])
        r2 = vt.T @ u.T

        if np.linalg.det(r2) < 0:
            vt[-1] *= -1
            r2 = vt.T @ u.T

        rotation = np.eye(3)
        rotation[:2, :2] = r2

    translation = tgt_mean - rotation @ src_mean
    aligned = source @ rotation.T + translation

    return aligned, rotation


def align_quaternions(quaternions, rotation):
    q_align = rotation_to_quaternion(rotation)

    return np.asarray([
        normalize_quaternion(
            quaternion_multiply(q_align, q)
        )
        for q in quaternions
    ])


def pose_matrix(position, quaternion):
    T = np.eye(4)
    T[:3, :3] = quaternion_to_rotation(quaternion)
    T[:3, 3] = position
    return T


def calculate_rpe(
    times,
    ref_positions,
    ref_quaternions,
    est_positions,
    est_quaternions,
    delta_s,
):
    translation_errors = []
    rotation_errors = []

    for i in range(len(times)):
        target_time = times[i] + delta_s
        j = np.searchsorted(times, target_time)

        if j >= len(times):
            break

        ref_relative = (
            np.linalg.inv(
                pose_matrix(ref_positions[i], ref_quaternions[i])
            )
            @ pose_matrix(ref_positions[j], ref_quaternions[j])
        )

        est_relative = (
            np.linalg.inv(
                pose_matrix(est_positions[i], est_quaternions[i])
            )
            @ pose_matrix(est_positions[j], est_quaternions[j])
        )

        error_transform = (
            np.linalg.inv(ref_relative) @ est_relative
        )

        translation_errors.append(
            np.linalg.norm(error_transform[:3, 3])
        )

        trace = np.trace(error_transform[:3, :3])
        angle = math.acos(
            np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
        )

        rotation_errors.append(math.degrees(angle))

    if not translation_errors:
        raise RuntimeError("Not enough samples to calculate RPE.")

    translation_errors = np.asarray(translation_errors)
    rotation_errors = np.asarray(rotation_errors)

    return (
        float(np.sqrt(np.mean(translation_errors ** 2))),
        float(np.sqrt(np.mean(rotation_errors ** 2))),
    )


def scale_diagnostic(source, target):
    src = source - source.mean(axis=0)
    tgt = target - target.mean(axis=0)

    denominator = np.sum(src ** 2)

    if denominator <= 1e-12:
        return float("nan")

    u, singular, vt = np.linalg.svd(src.T @ tgt)

    correction = np.ones(3)

    if np.linalg.det(vt.T @ u.T) < 0:
        correction[-1] = -1

    return float(
        np.sum(singular * correction) / denominator
    )


def write_metrics(path, metrics):
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value"])

        for key, value in metrics.items():
            writer.writerow([key, f"{value:.9f}"])


def write_trajectory(path, times, reference, estimate, errors):
    with Path(path).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)

        writer.writerow([
            "timestamp_s",
            "reference_x_m", "reference_y_m", "reference_z_m",
            "estimate_x_m", "estimate_y_m", "estimate_z_m",
            "error_3d_m",
        ])

        for t, ref, est, error in zip(
            times, reference, estimate, errors
        ):
            writer.writerow([
                f"{t:.9f}",
                *[f"{v:.9f}" for v in ref],
                *[f"{v:.9f}" for v in est],
                f"{error:.9f}",
            ])


def plot_trajectory(reference, estimate, method, path):
    plt.figure(figsize=(8, 7))

    plt.plot(
        reference[:, 0],
        reference[:, 1],
        label="Reference",
        linewidth=2,
    )

    plt.plot(
        estimate[:, 0],
        estimate[:, 1],
        label=method,
        linewidth=1.5,
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(f"{method} trajectory")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()


def plot_error(times, errors, method, path):
    relative_time = times - times[0]

    plt.figure(figsize=(9, 5))

    plt.plot(relative_time, errors, linewidth=1.5)

    plt.xlabel("Time [s]")
    plt.ylabel("Position error [m]")
    plt.title(f"{method} position error")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(path, dpi=180)
    plt.close()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("estimate_csv", type=Path)
    parser.add_argument("output_dir", type=Path)

    parser.add_argument(
        "--format",
        choices=("vins", "supervins"),
        required=True,
    )

    parser.add_argument(
        "--alignment",
        choices=("se3", "yaw"),
        required=True,
    )

    parser.add_argument(
        "--method-label",
        required=True,
    )

    parser.add_argument(
        "--reference-topic",
        default=REFERENCE_TOPIC,
    )

    parser.add_argument(
        "--samples",
        type=int,
        default=300,
    )

    parser.add_argument(
        "--rpe-delta-s",
        type=float,
        default=1.0,
    )

    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    ref_t, ref_p, ref_q = load_reference(
        args.bag_dir,
        args.reference_topic,
    )

    est_t, est_p, est_q = load_estimate(
        args.estimate_csv,
        args.format,
    )

    print(
        f"[INFO] Reference time: "
        f"{ref_t[0]:.6f} .. {ref_t[-1]:.6f}"
    )
    print(
        f"[INFO] Estimate time:  "
        f"{est_t[0]:.6f} .. {est_t[-1]:.6f}"
    )

    times, ref_p, ref_q, est_p, est_q = associate_by_time(
        ref_t, ref_p, ref_q,
        est_t, est_p, est_q,
        args.samples,
    )

    scale = scale_diagnostic(est_p, ref_p)

    est_p, alignment_rotation = align_positions(
        est_p,
        ref_p,
        args.alignment,
    )

    est_q = align_quaternions(
        est_q,
        alignment_rotation,
    )

    position_errors = np.linalg.norm(
        est_p - ref_p,
        axis=1,
    )

    ate_rmse = float(
        np.sqrt(np.mean(position_errors ** 2))
    )

    endpoint_error = float(position_errors[-1])

    rpe_translation, rpe_rotation = calculate_rpe(
        times,
        ref_p,
        ref_q,
        est_p,
        est_q,
        args.rpe_delta_s,
    )

    metrics = {
        "ate_rmse_m": ate_rmse,
        "rpe_translation_rmse_m": rpe_translation,
        "rpe_rotation_rmse_deg": rpe_rotation,
        "endpoint_error_m": endpoint_error,
        "scale_diagnostic": scale,
        "compared_samples": float(len(times)),
        "comparison_duration_s": float(times[-1] - times[0]),
    }

    write_metrics(
        args.output_dir / "metrics.csv",
        metrics,
    )

    write_trajectory(
        args.output_dir / "trajectory.csv",
        times,
        ref_p,
        est_p,
        position_errors,
    )

    plot_trajectory(
        ref_p,
        est_p,
        args.method_label,
        args.output_dir / "trajectory_xy.png",
    )

    plot_error(
        times,
        position_errors,
        args.method_label,
        args.output_dir / "error.png",
    )

    print()
    print(f"[OK] {args.method_label}")
    print(f"ATE RMSE:        {ate_rmse:.3f} m")
    print(f"RPE translation: {rpe_translation:.3f} m")
    print(f"RPE rotation:    {rpe_rotation:.3f} deg")
    print(f"Endpoint error:  {endpoint_error:.3f} m")
    print(f"Scale:           {scale:.3f}")
    print(f"Output:          {args.output_dir}")


if __name__ == "__main__":
    main()
