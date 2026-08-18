#!/usr/bin/env python3

import argparse
import sqlite3
from pathlib import Path


REQUIRED = [
    "/stereo/left/image_raw",
    "/stereo/right/image_raw",
    "/uav/local_pose",
]


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    bag = Path(args.bag)

    dbs = list(bag.glob("*.db3"))

    if len(dbs) != 1:
        print(
            f"[FAIL] Expected one db3, found {len(dbs)}"
        )
        raise SystemExit(2)

    con = sqlite3.connect(
        f"file:{dbs[0]}?mode=ro",
        uri=True,
    )

    cur = con.cursor()

    topics = dict(
        cur.execute(
            "SELECT name,id FROM topics"
        ).fetchall()
    )

    counts = {}

    failed = False

    print("=" * 60)
    print("FAST BAG VALIDATION")
    print("=" * 60)

    for topic in REQUIRED:

        tid = topics.get(topic)

        if tid is None:
            print(f"[FAIL] Missing: {topic}")
            failed = True
            continue

        count = cur.execute(
            "SELECT COUNT(*) FROM messages WHERE topic_id=?",
            (tid,),
        ).fetchone()[0]

        counts[topic] = count

        print(
            f"{topic:<32} {count:>7}"
        )

        if count < 100:
            print(
                f"[FAIL] Too few messages on {topic}"
            )
            failed = True

    con.close()

    if (
        "/stereo/left/image_raw" in counts
        and "/stereo/right/image_raw" in counts
    ):

        left = counts["/stereo/left/image_raw"]
        right = counts["/stereo/right/image_raw"]

        difference = abs(left - right)
        allowed = max(5, int(max(left, right) * 0.01))

        print()
        print(
            f"Stereo count difference: "
            f"{difference} "
            f"(allowed {allowed})"
        )

        if difference > allowed:
            print("[FAIL] Stereo count mismatch.")
            failed = True

    print()

    if failed:
        print("BAG STATUS: FAIL")
        raise SystemExit(2)

    print("BAG STATUS: PASS")


if __name__ == "__main__":
    main()
