#!/usr/bin/env python3
import argparse, csv, html, json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

METHOD_LABEL = "VINS-Fusion Stereo VO"
import rosbag2_py
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.serialization import deserialize_message

DEFAULT_REFERENCE_TOPIC = "/uav/local_pose"


def storage_id(bag_dir: Path) -> str:
    meta = bag_dir / "metadata.yaml"
    if not meta.exists():
        return "sqlite3"
    try:
        data = yaml.safe_load(meta.read_text(encoding="utf-8"))
        return str(data["rosbag2_bagfile_information"]["storage_identifier"])
    except Exception:
        return "sqlite3"


def load_reference(bag_dir, topic_name):
    """
    Load only the requested reference topic directly from the rosbag
    SQLite database.

    This avoids sequentially reading and deserializing all recorded
    stereo-image messages.
    """
    import sqlite3
    from pathlib import Path

    import numpy as np
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_path = Path(bag_dir)

    database_files = sorted(bag_path.glob("*.db3"))

    if not database_files:
        raise FileNotFoundError(
            f"No .db3 rosbag database found in: {bag_path}"
        )

    positions = []
    timestamps_ns = []
    message_type_name = None

    for database_file in database_files:
        connection = sqlite3.connect(str(database_file))

        try:
            topic_row = connection.execute(
                """
                SELECT id, type
                FROM topics
                WHERE name = ?
                """,
                (topic_name,),
            ).fetchone()

            if topic_row is None:
                continue

            topic_id, topic_type = topic_row
            message_type_name = topic_type
            message_class = get_message(topic_type)

            cursor = connection.execute(
                """
                SELECT timestamp, data
                FROM messages
                WHERE topic_id = ?
                ORDER BY timestamp
                """,
                (topic_id,),
            )

            for timestamp_ns, serialized_data in cursor:
                message = deserialize_message(
                    serialized_data,
                    message_class,
                )

                positions.append(
                    [
                        float(message.pose.position.x),
                        float(message.pose.position.y),
                        float(message.pose.position.z),
                    ]
                )

                timestamps_ns.append(int(timestamp_ns))

        finally:
            connection.close()

    if not positions:
        raise RuntimeError(
            f"No messages found for topic '{topic_name}' in {bag_path}"
        )

    order = np.argsort(np.asarray(timestamps_ns, dtype=np.int64))

    positions_array = np.asarray(
        positions,
        dtype=float,
    )[order]

    timestamps_array = np.asarray(
        timestamps_ns,
        dtype=np.int64,
    )[order]

    time_seconds = (
        timestamps_array - timestamps_array[0]
    ).astype(float) * 1e-9

    print(
        f"[INFO] Loaded {len(positions_array)} messages directly "
        f"from {topic_name} ({message_type_name})."
    )

    return positions_array, time_seconds

def load_vins_csv(path: Path):
    points, times = [], []
    with path.open("r", newline="", encoding="utf-8") as handle:
        for row_number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue

            if "," in line:
                row = [part.strip() for part in line.split(",")]
            else:
                row = line.split()

            if len(row) < 4:
                continue
            try:
                t, x, y, z = map(float, row[:4])
            except ValueError:
                if row_number != 1:
                    print(f"[WARN] Skipping invalid VINS row {row_number}")
                continue
            if np.all(np.isfinite([t, x, y, z])):
                times.append(t)
                points.append([x, y, z])
    if not points:
        raise RuntimeError(f"No valid VINS positions found in {path}")

    times = np.asarray(times, dtype=float)
    if len(times) > 1:
        dt = float(np.median(np.abs(np.diff(times))))
        if dt > 1e6:
            times *= 1e-9
        elif dt > 1e3:
            times *= 1e-6
        elif dt > 10:
            times *= 1e-3
    times -= times[0]
    return np.asarray(points, dtype=float), times


def cumulative_distance(points):
    if len(points) < 2:
        return np.zeros(len(points))
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]


def path_length(points):
    return float(cumulative_distance(points)[-1]) if len(points) else 0.0


def resample_progress(points, count):
    s = cumulative_distance(points)
    if s[-1] <= 1e-12:
        return np.repeat(points[:1], count, axis=0)
    u = s / s[-1]
    targets = np.linspace(0.0, 1.0, count)
    return np.column_stack([np.interp(targets, u, points[:, i]) for i in range(3)])


def resample_time(points, times, target_times):
    return np.column_stack(
        [np.interp(target_times, times, points[:, i]) for i in range(3)]
    )


def associate(ref, ref_t, est, est_t, method, count):
    if method == "time":
        duration = min(float(ref_t[-1]), float(est_t[-1]))
        if duration <= 1e-6:
            raise RuntimeError("Time association failed because duration is invalid.")
        target = np.linspace(0.0, duration, count)
        progress = target / duration * 100.0
        return (
            resample_time(ref, ref_t, target),
            resample_time(est, est_t, target),
            progress,
            "relative timestamp interpolation",
        )
    return (
        resample_progress(ref, count),
        resample_progress(est, count),
        np.linspace(0.0, 100.0, count),
        "normalized traveled-path progress",
    )


def rigid_alignment(source, target):
    src_mean, tgt_mean = source.mean(0), target.mean(0)
    src_c, tgt_c = source - src_mean, target - tgt_mean
    u, _, vt = np.linalg.svd(src_c.T @ tgt_c)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1
        rotation = vt.T @ u.T
    translation = tgt_mean - rotation @ src_mean
    return source @ rotation.T + translation, rotation, translation


def scale_diagnostic(source, target):
    src_c, tgt_c = source - source.mean(0), target - target.mean(0)
    denominator = np.sum(src_c ** 2)
    if denominator <= 1e-12:
        return float("nan")
    u, singular, vt = np.linalg.svd(src_c.T @ tgt_c)
    correction = np.ones(3)
    if np.linalg.det(vt.T @ u.T) < 0:
        correction[-1] = -1.0
    return float(np.sum(singular * correction) / denominator)


def metrics(ref, est, ref_raw, est_raw):
    vector = est - ref
    error = np.linalg.norm(vector, axis=1)
    horizontal = np.linalg.norm(vector[:, :2], axis=1)
    vertical = np.abs(vector[:, 2])
    ref_len, est_len = path_length(ref_raw), path_length(est_raw)
    out = {
        "ate_rmse_m": float(np.sqrt(np.mean(error ** 2))),
        "ate_mean_m": float(np.mean(error)),
        "ate_median_m": float(np.median(error)),
        "ate_std_m": float(np.std(error)),
        "ate_max_m": float(np.max(error)),
        "endpoint_error_m": float(error[-1]),
        "horizontal_rmse_m": float(np.sqrt(np.mean(horizontal ** 2))),
        "horizontal_mean_m": float(np.mean(horizontal)),
        "horizontal_max_m": float(np.max(horizontal)),
        "vertical_rmse_m": float(np.sqrt(np.mean(vertical ** 2))),
        "vertical_mean_m": float(np.mean(vertical)),
        "vertical_max_m": float(np.max(vertical)),
        "reference_path_length_m": ref_len,
        "estimate_path_length_m": est_len,
        "path_length_difference_m": est_len - ref_len,
        "path_length_ratio": est_len / ref_len if ref_len > 1e-12 else float("nan"),
        "reference_start_end_distance_m": float(np.linalg.norm(ref_raw[-1] - ref_raw[0])),
        "estimate_start_end_distance_m": float(np.linalg.norm(est_raw[-1] - est_raw[0])),
        "estimate_vertical_range_m": float(np.ptp(est_raw[:, 2])),
    }
    return out, error, vector


def plot_xy(ref, est, path, title):
    plt.figure(figsize=(9, 8))
    plt.plot(ref[:, 0], ref[:, 1], label="PX4 reference", linewidth=2)
    plt.plot(est[:, 0], est[:, 1], label=METHOD_LABEL, linewidth=1.5)
    plt.scatter(ref[0, 0], ref[0, 1], marker="o", label="Reference start")
    plt.scatter(est[0, 0], est[0, 1], marker="x", label=f"{METHOD_LABEL} start")
    plt.scatter(ref[-1, 0], ref[-1, 1], marker="s", label="Reference end")
    plt.scatter(est[-1, 0], est[-1, 1], marker="+", label=f"{METHOD_LABEL} end")
    plt.xlabel("X [m]"); plt.ylabel("Y [m]"); plt.title(title)
    plt.axis("equal"); plt.grid(True); plt.legend(); plt.tight_layout()
    plt.savefig(path, dpi=180); plt.close()


def plot_3d(ref, est, path):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(ref[:, 0], ref[:, 1], ref[:, 2], label="PX4 reference", linewidth=2)
    ax.plot(est[:, 0], est[:, 1], est[:, 2], label=METHOD_LABEL, linewidth=1.5)
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
    ax.set_title("Rigid-aligned 3D trajectories"); ax.legend()
    fig.tight_layout(); fig.savefig(path, dpi=180); plt.close(fig)


def plot_coordinates(ref, est, progress, path):
    plt.figure(figsize=(11, 7))
    for i, axis in enumerate(("X", "Y", "Z")):
        plt.plot(progress, ref[:, i], label=f"Reference {axis}", linewidth=2)
        plt.plot(progress, est[:, i], "--", label=f"{METHOD_LABEL} {axis}", linewidth=1.3)
    plt.xlabel("Comparison progress [%]"); plt.ylabel("Position [m]")
    plt.title("Trajectory coordinates"); plt.grid(True); plt.legend(ncol=2)
    plt.tight_layout(); plt.savefig(path, dpi=180); plt.close()


def plot_error(progress, error, path):
    plt.figure(figsize=(11, 6))
    plt.plot(progress, error, linewidth=1.8)
    plt.xlabel("Comparison progress [%]"); plt.ylabel("3D position error [m]")
    plt.title(f"{METHOD_LABEL} position error relative to PX4 reference")
    plt.grid(True); plt.tight_layout(); plt.savefig(path, dpi=180); plt.close()


def plot_axis_error(progress, vector, path):
    plt.figure(figsize=(11, 7))
    for i, label in enumerate(("X error", "Y error", "Z error")):
        plt.plot(progress, vector[:, i], label=label, linewidth=1.5)
    plt.axhline(0.0, linewidth=1)
    plt.xlabel("Comparison progress [%]"); plt.ylabel("Signed error [m]")
    plt.title(f"Signed coordinate errors: {METHOD_LABEL} minus PX4")
    plt.grid(True); plt.legend(); plt.tight_layout()
    plt.savefig(path, dpi=180); plt.close()


def write_csv(path, values):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f); writer.writerow(["metric", "value"])
        for key, value in values.items():
            writer.writerow([key, f"{value:.9f}"])


def write_matches(path, progress, ref, est, error, vector):
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "progress_percent", "reference_x_m", "reference_y_m", "reference_z_m",
            "vins_x_m", "vins_y_m", "vins_z_m",
            "error_x_m", "error_y_m", "error_z_m", "error_3d_m"
        ])
        for i in range(len(progress)):
            writer.writerow([
                f"{progress[i]:.9f}", *[f"{v:.9f}" for v in ref[i]],
                *[f"{v:.9f}" for v in est[i]], *[f"{v:.9f}" for v in vector[i]],
                f"{error[i]:.9f}"
            ])


def write_html(path, values, ref_n, est_n, matched_n, association, scale):
    rows = "\n".join(
        f"<tr><td>{html.escape(k)}</td><td>{v:.6f}</td></tr>"
        for k, v in values.items()
    )
    path.write_text(f"""<!doctype html>
<html><head><meta charset="utf-8"><title>{METHOD_LABEL} Evaluation</title>
<style>
body{{font-family:Arial,sans-serif;max-width:1180px;margin:auto;padding:24px;background:#f4f4f4;color:#222}}
section{{background:white;padding:20px;margin-bottom:18px;border:1px solid #ddd}}
table{{border-collapse:collapse;width:100%}}td,th{{border:1px solid #ddd;padding:8px;text-align:left}}
th{{background:#eee}}img{{max-width:100%;height:auto;border:1px solid #ddd}}
.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(470px,1fr));gap:16px}}
</style></head><body>
<section><h1>{METHOD_LABEL} Evaluation</h1>
<p><b>Reference:</b> recorded PX4 local pose</p>
<p><b>Estimate:</b> {METHOD_LABEL}, processed offline from the recorded dataset</p>
<p><b>Alignment:</b> rigid 3D rotation and translation; scale fixed to 1.0</p>
<p><b>Association:</b> {html.escape(association)}</p>
<p><b>Samples:</b> reference {ref_n}, VINS {est_n}, compared {matched_n}</p>
<p><b>Similarity-scale diagnostic:</b> {scale:.6f}</p></section>
<section><h2>Metrics</h2><table><tr><th>Metric</th><th>Value</th></tr>{rows}</table></section>
<section><h2>Trajectory plots</h2><div class="grid">
<div><h3>Start-normalized</h3><img src="01_xy_start_normalized.png"></div>
<div><h3>Rigid-aligned XY</h3><img src="02_xy_rigid_aligned.png"></div>
<div><h3>Rigid-aligned 3D</h3><img src="03_xyz_rigid_aligned.png"></div>
<div><h3>Coordinates</h3><img src="04_coordinates_over_progress.png"></div>
</div></section>
<section><h2>Error plots</h2><div class="grid">
<div><h3>3D position error</h3><img src="05_position_error_over_progress.png"></div>
<div><h3>Signed axis errors</h3><img src="06_axis_errors_over_progress.png"></div>
</div></section></body></html>""", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(
        description="Compare an offline VINS estimate with the recorded PX4 reference."
    )
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("vins_csv", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--reference-topic", default=DEFAULT_REFERENCE_TOPIC)
    parser.add_argument("--association", choices=("progress", "time"), default="progress")
    parser.add_argument("--samples", type=int, default=300)
    parser.add_argument(
        "--method-label",
        default="VINS-Fusion Stereo VO",
        help=(
            "Clear method name used in reports, HTML pages, "
            "plot titles, and legends."
        ),
    )

    args = parser.parse_args()

    global METHOD_LABEL
    METHOD_LABEL = args.method_label
    if args.samples < 10:
        raise ValueError("--samples must be at least 10")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    ref_raw, ref_t = load_reference(args.bag_dir, args.reference_topic)
    est_raw, est_t = load_vins_csv(args.vins_csv)
    print(f"[INFO] Reference samples: {len(ref_raw)}")
    print(f"[INFO] VINS samples:      {len(est_raw)}")

    ref_zero, est_zero = ref_raw - ref_raw[0], est_raw - est_raw[0]
    plot_xy(ref_zero, est_zero, args.output_dir / "01_xy_start_normalized.png",
            "Start-normalized trajectories without rotational alignment")

    count = min(args.samples, len(ref_raw), len(est_raw))
    ref, est, progress, association = associate(
        ref_zero, ref_t, est_zero, est_t, args.association, count
    )
    est_aligned, rotation, translation = rigid_alignment(est, ref)
    values, error, vector = metrics(ref, est_aligned, ref_zero, est_zero)
    scale = scale_diagnostic(est, ref)
    values.update({
        "similarity_scale_diagnostic": scale,
        "reference_samples": float(len(ref_raw)),
        "vins_samples": float(len(est_raw)),
        "compared_samples": float(count),
        "reference_duration_s": float(ref_t[-1]),
        "vins_duration_s": float(est_t[-1]),
    })

    plot_xy(ref, est_aligned, args.output_dir / "02_xy_rigid_aligned.png",
            "Rigid-aligned trajectories, scale fixed to 1.0")
    plot_3d(ref, est_aligned, args.output_dir / "03_xyz_rigid_aligned.png")
    plot_coordinates(ref, est_aligned, progress, args.output_dir / "04_coordinates_over_progress.png")
    plot_error(progress, error, args.output_dir / "05_position_error_over_progress.png")
    plot_axis_error(progress, vector, args.output_dir / "06_axis_errors_over_progress.png")

    (args.output_dir / "metrics.json").write_text(json.dumps(values, indent=2), encoding="utf-8")
    write_csv(args.output_dir / "metrics.csv", values)
    write_matches(args.output_dir / "matched_trajectory_errors.csv", progress, ref, est_aligned, error, vector)

    with (args.output_dir / "comparison_report.txt").open("w", encoding="utf-8") as f:
        report_title = f"{METHOD_LABEL} comparison"
        f.write(report_title + "\n")
        f.write("=" * len(report_title) + "\n\n")
        f.write(f"Reference topic: {args.reference_topic}\n")
        f.write(f"Association: {association}\n")
        f.write("Alignment: rigid 3D alignment with scale fixed to 1.0\n\n")
        for key, value in values.items():
            f.write(f"{key}: {value:.9f}\n")
        f.write("\nRotation matrix:\n" + np.array2string(rotation, precision=9))
        f.write("\n\nTranslation vector:\n" + np.array2string(translation, precision=9) + "\n")

    write_html(args.output_dir / "trajectory_comparison.html", values,
               len(ref_raw), len(est_raw), count, association, scale)

    print(f"[OK] Results: {args.output_dir}")
    print(f"[OK] HTML: {args.output_dir / 'trajectory_comparison.html'}")
    print(f"ATE RMSE: {values['ate_rmse_m']:.3f} m")
    print(f"Endpoint error: {values['endpoint_error_m']:.3f} m")
    print(f"Scale diagnostic: {scale:.3f}")


if __name__ == "__main__":
    main()
