#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer


class StereoSyncChecker(Node):
    def __init__(self):
        super().__init__("stereo_sync_checker")

        self.left_sub = Subscriber(
            self,
            Image,
            "/stereo/left/image_raw",
        )

        self.right_sub = Subscriber(
            self,
            Image,
            "/stereo/right/image_raw",
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=30,
            slop=0.1,
        )

        self.sync.registerCallback(self.callback)

        self.samples = 0
        self.sum_diff_ms = 0.0
        self.max_diff_ms = 0.0

        self.get_logger().info("Stereo synchronization checker started.")
        self.get_logger().info("Left:  /stereo/left/image_raw")
        self.get_logger().info("Right: /stereo/right/image_raw")

    def callback(self, left_msg, right_msg):
        left_time = (
            float(left_msg.header.stamp.sec)
            + float(left_msg.header.stamp.nanosec) * 1e-9
        )

        right_time = (
            float(right_msg.header.stamp.sec)
            + float(right_msg.header.stamp.nanosec) * 1e-9
        )

        diff_ms = abs(left_time - right_time) * 1000.0

        self.samples += 1
        self.sum_diff_ms += diff_ms
        self.max_diff_ms = max(self.max_diff_ms, diff_ms)

        mean_diff_ms = self.sum_diff_ms / self.samples

        self.get_logger().info(
            f"Pair {self.samples:4d} | "
            f"dt = {diff_ms:7.3f} ms | "
            f"mean = {mean_diff_ms:7.3f} ms | "
            f"max = {self.max_diff_ms:7.3f} ms"
        )


def main():
    rclpy.init()

    node = StereoSyncChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
