#!/usr/bin/env python3
import sys, sqlite3, numpy as np
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image

if len(sys.argv) != 2:
    print("Usage: check_stereo_header_continuity.py <bag_dir>")
    sys.exit(2)

bag = Path(sys.argv[1])
dbs = list(bag.glob("*.db3"))

if not dbs:
    print("[FAIL] No db3 file found")
    sys.exit(2)

con = sqlite3.connect(f"file:{dbs[0]}?mode=ro", uri=True)
cur = con.cursor()

good = True

for topic in ["/stereo/left/image_raw", "/stereo/right/image_raw"]:
    row = cur.execute(
        "SELECT id FROM topics WHERE name=?", (topic,)
    ).fetchone()

    if not row:
        print(f"[FAIL] Missing {topic}")
        good = False
        continue

    stamps = []

    for (blob,) in cur.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (row[0],)
    ):
        msg = deserialize_message(blob, Image)
        stamps.append(
            msg.header.stamp.sec +
            msg.header.stamp.nanosec * 1e-9
        )

    if len(stamps) < 2:
        print(f"[FAIL] Too few frames on {topic}: {len(stamps)}")
        good = False
        continue

    s = np.asarray(stamps)
    dt = np.diff(s)

    hz = (len(s)-1) / (s[-1]-s[0])

    print()
    print(topic)
    print(f"frames:       {len(s)}")
    print(f"effective Hz: {hz:.2f}")
    print(f"median dt:    {np.median(dt):.3f} s")
    print(f"95% dt:       {np.percentile(dt,95):.3f} s")
    print(f"max dt:       {dt.max():.3f} s")
    print(f"gaps >0.10s:  {np.sum(dt>0.10)}")
    print(f"gaps >0.20s:  {np.sum(dt>0.20)}")
    print(f"gaps >1.00s:  {np.sum(dt>1.00)}")

    if (
        hz < 25.0 or
        dt.max() > 0.20 or
        np.sum(dt > 1.0) > 0
    ):
        good = False

con.close()

print()
print("====================================")
print("STEREO CONTINUITY:", "PASS" if good else "FAIL")
print("====================================")

sys.exit(0 if good else 1)
