#!/usr/bin/env python3

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


LEFT_TOPIC = "/stereo/left/image_raw"
RIGHT_TOPIC = "/stereo/right/image_raw"

WARN_GAP_S = 0.20
FAIL_GAP_S = 1.00
REPORT_INTERVAL_S = 3.0


class CameraWatchdog(Node):

    def __init__(self, status_file):
        super().__init__("camera_recording_watchdog")

        self.status_file = Path(status_file)

        self.data = {
            "LEFT": {
                "count": 0,
                "last_stamp": None,
                "max_gap": 0.0,
                "warn_gaps": 0,
                "fail_gaps": 0,
            },
            "RIGHT": {
                "count": 0,
                "last_stamp": None,
                "max_gap": 0.0,
                "warn_gaps": 0,
                "fail_gaps": 0,
            },
        }

        self.failed = False

        self.create_subscription(
            Image,
            LEFT_TOPIC,
            lambda msg: self.on_image(msg, "LEFT"),
            50,
        )

        self.create_subscription(
            Image,
            RIGHT_TOPIC,
            lambda msg: self.on_image(msg, "RIGHT"),
            50,
        )

        self.create_timer(REPORT_INTERVAL_S, self.report)

        self.write_status()

    @staticmethod
    def stamp(msg):
        return (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

    def on_image(self, msg, side):

        stamp = self.stamp(msg)
        d = self.data[side]

        if d["last_stamp"] is not None:

            gap = stamp - d["last_stamp"]

            if gap > d["max_gap"]:
                d["max_gap"] = gap

            if gap > FAIL_GAP_S:
                d["fail_gaps"] += 1
                self.failed = True

                print()
                print("=" * 64)
                print(
                    f"[HARD FAIL] {side} CAMERA GAP: "
                    f"{gap:.3f} s"
                )
                print("THIS ATTEMPT WILL BE REJECTED.")
                print("=" * 64)

            elif gap > WARN_GAP_S:
                d["warn_gaps"] += 1

                print(
                    f"[WARN] {side} camera gap: "
                    f"{gap:.3f} s"
                )

        d["count"] += 1
        d["last_stamp"] = stamp

        self.write_status()

    def write_status(self):

        payload = {
            "failed": self.failed,
            "thresholds": {
                "warn_gap_s": WARN_GAP_S,
                "fail_gap_s": FAIL_GAP_S,
            },
            "left": self.data["LEFT"],
            "right": self.data["RIGHT"],
        }

        tmp = self.status_file.with_suffix(".tmp")

        tmp.write_text(
            json.dumps(payload, indent=2)
        )

        tmp.replace(self.status_file)

    def report(self):

        l = self.data["LEFT"]
        r = self.data["RIGHT"]

        print(
            "[WATCHDOG] "
            f"L={l['count']} max={l['max_gap']:.3f}s | "
            f"R={r['count']} max={r['max_gap']:.3f}s | "
            f"{'FAIL' if self.failed else 'PASS'}"
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--status-file",
        required=True,
    )

    args = parser.parse_args()

    rclpy.init()

    node = CameraWatchdog(
        args.status_file
    )

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.write_status()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
