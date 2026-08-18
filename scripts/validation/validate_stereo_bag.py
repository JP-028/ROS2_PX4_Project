#!/usr/bin/env python3

import argparse
import sqlite3
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


LEFT = "/stereo/left/image_raw"
RIGHT = "/stereo/right/image_raw"

WARN_GAP_S = 0.20
FAIL_GAP_S = 1.00
SYNC_LIMIT_S = 0.003


def find_db(bag):
    bag = Path(bag)

    if bag.is_file() and bag.suffix == ".db3":
        return bag

    dbs = sorted(bag.glob("*.db3"))

    if len(dbs) != 1:
        raise RuntimeError(
            f"Expected exactly one .db3 in {bag}, found {len(dbs)}"
        )

    return dbs[0]


def read_image_stamps(con, topic):
    cur = con.cursor()

    row = cur.execute(
        "SELECT id FROM topics WHERE name=?",
        (topic,),
    ).fetchone()

    if row is None:
        raise RuntimeError(f"Missing topic: {topic}")

    topic_id = row[0]

    rows = cur.execute(
        """
        SELECT data
        FROM messages
        WHERE topic_id=?
        ORDER BY timestamp
        """,
        (topic_id,),
    )

    stamps = []

    for (blob,) in rows:
        msg = deserialize_message(blob, Image)

        stamps.append(
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

    return np.asarray(stamps, dtype=np.float64)


def camera_stats(name, stamps):
    dt = np.diff(stamps)

    maximum = float(np.max(dt))
    warnings = int(np.sum(dt > WARN_GAP_S))
    failures = int(np.sum(dt > FAIL_GAP_S))

    print()
    print(name)
    print("-" * 45)
    print(f"Images:             {len(stamps)}")
    print(f"Median interval:    {np.median(dt):.4f} s")
    print(f"95% interval:       {np.percentile(dt,95):.4f} s")
    print(f"Maximum gap:        {maximum:.4f} s")
    print(f"Gaps > {WARN_GAP_S:.2f}s:      {warnings}")
    print(f"Gaps > {FAIL_GAP_S:.2f}s:      {failures}")

    print("Largest gaps:")

    for i in np.argsort(dt)[::-1][:5]:
        print(
            f"  {stamps[i]:10.3f} -> "
            f"{stamps[i+1]:10.3f}   "
            f"{dt[i]:7.3f}s"
        )

    return failures == 0


def stereo_stats(left, right):
    nearest = []

    for t in left:
        j = np.searchsorted(right, t)
        choices = []

        if j < len(right):
            choices.append(abs(right[j] - t))

        if j > 0:
            choices.append(abs(right[j - 1] - t))

        nearest.append(min(choices))

    nearest = np.asarray(nearest)

    over = int(np.sum(nearest > SYNC_LIMIT_S))

    print()
    print("STEREO SYNCHRONIZATION")
    print("-" * 45)
    print(
        f"Median |L-R|:       "
        f"{np.median(nearest)*1000:.3f} ms"
    )
    print(
        f"95% |L-R|:          "
        f"{np.percentile(nearest,95)*1000:.3f} ms"
    )
    print(
        f"Maximum |L-R|:      "
        f"{np.max(nearest)*1000:.3f} ms"
    )
    print(
        f"Pairs > 3 ms:       {over}"
    )

    # Don't reject for one isolated unsynced pair.
    # Repeated synchronization failure is what matters.
    return over <= max(5, int(0.01 * len(left)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    db = find_db(args.bag)

    print("=" * 64)
    print("FINAL STEREO BAG VALIDATION")
    print("=" * 64)
    print("Database:", db)

    con = sqlite3.connect(
        f"file:{db}?mode=ro",
        uri=True,
    )

    try:
        left = read_image_stamps(con, LEFT)
        right = read_image_stamps(con, RIGHT)
    finally:
        con.close()

    left_ok = camera_stats("LEFT CAMERA", left)
    right_ok = camera_stats("RIGHT CAMERA", right)
    sync_ok = stereo_stats(left, right)

    count_ok = abs(len(left) - len(right)) <= 5

    print()
    print("=" * 64)
    print("FINAL RESULT")
    print("=" * 64)
    print(f"Left continuity:     {'PASS' if left_ok else 'FAIL'}")
    print(f"Right continuity:    {'PASS' if right_ok else 'FAIL'}")
    print(f"Stereo sync:         {'PASS' if sync_ok else 'FAIL'}")
    print(f"Stereo count match:  {'PASS' if count_ok else 'FAIL'}")

    final = left_ok and right_ok and sync_ok and count_ok

    print()
    print(
        "DATASET STATUS:",
        "PASS - SAFE FOR ESTIMATOR EVALUATION"
        if final
        else "FAIL - DO NOT USE FOR FINAL EVALUATION"
    )

    raise SystemExit(0 if final else 2)


if __name__ == "__main__":
    main()
