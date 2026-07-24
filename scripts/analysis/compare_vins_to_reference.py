#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py

from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message


REFERENCE_TOPIC = "/uav/local_pose"


def load_reference_from_bag(bag_dir: Path) -> np.ndarray:
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    points = []

    while reader.has_next():
        topic, data, _ = reader.read_next()

        if topic != REFERENCE_TOPIC:
            continue

        msg = deserialize_message(data, PoseStamped)

        points.append([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    if not points:
        raise RuntimeError(
            f"No messages found on {REFERENCE_TOPIC} in {bag_dir}"
        )

    return np.asarray(points, dtype=float)


def load_vins_csv(csv_path: Path) -> np.ndarray:
    points = []

    with csv_path.open("r", newline="") as handle:
        reader = csv.reader(handle)

        for row_number, row in enumerate(reader, start=1):
            if len(row) < 4:
                continue

            try:
                x = float(row[1])
                y = float(row[2])
                z = float(row[3])
            except ValueError:
                print(f"[WARN] Skipping invalid VINS row {row_number}")
                continue

            if not np.all(np.isfinite([x, y, z])):
                continue

            points.append([x, y, z])

    if not points:
        raise RuntimeError(f"No valid VINS positions found in {csv_path}")

    return np.asarray(points, dtype=float)


def cumulative_distance(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros(len(points))

    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate(([0.0], np.cumsum(segment_lengths)))


def resample_by_progress(points: np.ndarray, count: int) -> np.ndarray:
    distances = cumulative_distance(points)

    if distances[-1] <= 1e-12:
        return np.repeat(points[:1], count, axis=0)

    normalized = distances / distances[-1]
    targets = np.linspace(0.0, 1.0, count)

    result = np.empty((count, 3), dtype=float)

    for axis in range(3):
        result[:, axis] = np.interp(
            targets,
            normalized,
            points[:, axis],
        )

    return result


def rigid_alignment(source: np.ndarray, target: np.ndarray):
    """
    Find R and t so that:
        aligned_source = source @ R.T + t

    Scale is intentionally fixed to 1.0.
    """
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)

    source_centered = source - source_mean
    target_centered = target - target_mean

    covariance = source_centered.T @ target_centered

    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T

    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T

    translation = target_mean - rotation @ source_mean

    aligned = source @ rotation.T + translation

    return aligned, rotation, translation


def similarity_scale_diagnostic(source: np.ndarray, target: np.ndarray) -> float:
    source_centered = source - source.mean(axis=0)
    target_centered = target - target.mean(axis=0)

    denominator = np.sum(source_centered ** 2)

    if denominator <= 1e-12:
        return float("nan")

    covariance = source_centered.T @ target_centered
    u, singular_values, vt = np.linalg.svd(covariance)

    correction = np.ones(3)

    if np.linalg.det(vt.T @ u.T) < 0:
        correction[-1] = -1.0

    return float(np.sum(singular_values * correction) / denominator)


def path_length(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0

    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def calculate_metrics(reference: np.ndarray, estimate: np.ndarray) -> dict:
    errors = np.linalg.norm(reference - estimate, axis=1)

    return {
        "ate_rmse_m": float(np.sqrt(np.mean(errors ** 2))),
        "ate_mean_m": float(np.mean(errors)),
        "ate_median_m": float(np.median(errors)),
        "ate_max_m": float(np.max(errors)),
        "endpoint_error_m": float(
            np.linalg.norm(reference[-1] - estimate[-1])
        ),
        "reference_path_length_m": path_length(reference),
        "estimate_path_length_m": path_length(estimate),
        "reference_start_end_distance_m": float(
            np.linalg.norm(reference[-1] - reference[0])
        ),
        "estimate_start_end_distance_m": float(
            np.linalg.norm(estimate[-1] - estimate[0])
        ),
        "estimate_vertical_range_m": float(
            np.max(estimate[:, 2]) - np.min(estimate[:, 2])
        ),
    }


def save_xy_plot(
    reference: np.ndarray,
    estimate: np.ndarray,
    output_path: Path,
    title: str,
):
    plt.figure(figsize=(9, 8))
    plt.plot(
        reference[:, 0],
        reference[:, 1],
        label="PX4 reference",
        linewidth=2,
    )
    plt.plot(
        estimate[:, 0],
        estimate[:, 1],
        label="VINS Stereo VO",
        linewidth=1.5,
    )

    plt.scatter(
        reference[0, 0],
        reference[0, 1],
        marker="o",
        label="Reference start",
    )
    plt.scatter(
        estimate[0, 0],
        estimate[0, 1],
        marker="x",
        label="VINS start",
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()


def save_xyz_plot(
    reference: np.ndarray,
    estimate: np.ndarray,
    output_path: Path,
):
    figure = plt.figure(figsize=(10, 8))
    axis = figure.add_subplot(111, projection="3d")

    axis.plot(
        reference[:, 0],
        reference[:, 1],
        reference[:, 2],
        label="PX4 reference",
        linewidth=2,
    )
    axis.plot(
        estimate[:, 0],
        estimate[:, 1],
        estimate[:, 2],
        label="VINS Stereo VO",
        linewidth=1.5,
    )

    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_zlabel("Z [m]")
    axis.set_title("Rigid-aligned 3D trajectories")
    axis.legend()

    figure.tight_layout()
    figure.savefig(output_path, dpi=180)
    plt.close(figure)


def save_coordinate_plot(
    reference: np.ndarray,
    estimate: np.ndarray,
    output_path: Path,
):
    progress = np.linspace(0.0, 100.0, len(reference))

    plt.figure(figsize=(11, 7))

    for axis, label in enumerate(("X", "Y", "Z")):
        plt.plot(
            progress,
            reference[:, axis],
            label=f"Reference {label}",
            linewidth=2,
        )
        plt.plot(
            progress,
            estimate[:, axis],
            linestyle="--",
            label=f"VINS {label}",
            linewidth=1.3,
        )

    plt.xlabel("Normalized path progress [%]")
    plt.ylabel("Position [m]")
    plt.title("Trajectory coordinates over normalized path progress")
    plt.grid(True)
    plt.legend(ncol=2)
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()


def main():
    parser = argparse.ArgumentParser(
        description="Compare VINS Stereo VO with /uav/local_pose."
    )
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("vins_csv", type=Path)
    parser.add_argument("output_dir", type=Path)

    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Reading reference bag: {args.bag_dir}")
    reference_raw = load_reference_from_bag(args.bag_dir)

    print(f"[INFO] Reading VINS CSV: {args.vins_csv}")
    vins_raw = load_vins_csv(args.vins_csv)

    print(f"[INFO] Reference samples: {len(reference_raw)}")
    print(f"[INFO] VINS samples:      {len(vins_raw)}")

    # Both trajectories are moved to their own starting point for the raw plot.
    reference_start_normalized = reference_raw - reference_raw[0]
    vins_start_normalized = vins_raw - vins_raw[0]

    save_xy_plot(
        reference_start_normalized,
        vins_start_normalized,
        args.output_dir / "01_xy_start_normalized.png",
        "Start-normalized trajectories without rotational alignment",
    )

    # Compare by normalized traveled-path progress because the current
    # VINS CSV timestamps have insufficient precision.
    sample_count = min(300, len(reference_raw), len(vins_raw))

    reference_resampled = resample_by_progress(
        reference_start_normalized,
        sample_count,
    )
    vins_resampled = resample_by_progress(
        vins_start_normalized,
        sample_count,
    )

    vins_aligned, rotation, translation = rigid_alignment(
        vins_resampled,
        reference_resampled,
    )

    metrics = calculate_metrics(reference_resampled, vins_aligned)

    diagnostic_scale = similarity_scale_diagnostic(
        vins_resampled,
        reference_resampled,
    )

    save_xy_plot(
        reference_resampled,
        vins_aligned,
        args.output_dir / "02_xy_rigid_aligned.png",
        "Rigid-aligned trajectories, scale fixed to 1.0",
    )

    save_xyz_plot(
        reference_resampled,
        vins_aligned,
        args.output_dir / "03_xyz_rigid_aligned.png",
    )

    save_coordinate_plot(
        reference_resampled,
        vins_aligned,
        args.output_dir / "04_coordinates_over_progress.png",
    )

    report_path = args.output_dir / "comparison_report.txt"

    with report_path.open("w") as report:
        report.write("VINS-Fusion Stereo VO comparison\n")
        report.write("================================\n\n")

        report.write(f"Reference samples: {len(reference_raw)}\n")
        report.write(f"VINS samples:      {len(vins_raw)}\n")
        report.write(f"Compared samples:  {sample_count}\n\n")

        report.write(
            "Alignment: rigid 3D alignment with scale fixed to 1.0\n"
        )
        report.write(
            "Time association: normalized traveled-path progress\n\n"
        )

        for key, value in metrics.items():
            report.write(f"{key}: {value:.6f}\n")

        report.write(
            f"\nsimilarity_scale_diagnostic: {diagnostic_scale:.6f}\n"
        )

        report.write("\nRotation matrix:\n")
        report.write(np.array2string(rotation, precision=6))
        report.write("\n\nTranslation vector:\n")
        report.write(np.array2string(translation, precision=6))
        report.write("\n")

    print()
    print("[OK] Comparison complete.")
    print(f"[OK] Results: {args.output_dir}")
    print()
    print(f"ATE RMSE:       {metrics['ate_rmse_m']:.3f} m")
    print(f"Endpoint error: {metrics['endpoint_error_m']:.3f} m")
    print(
        "Scale diagnostic: "
        f"{diagnostic_scale:.3f} "
        "(1.0 would indicate matching metric scale)"
    )


if __name__ == "__main__":
    main()
