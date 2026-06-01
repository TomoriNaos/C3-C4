#!/usr/bin/env python3
"""Publish STF gated0/gated1/gated2 images as ROS Image topics for testing."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay STF gated slices as ROS image topics.")
    parser.add_argument("--dataset", default="/home/hu/STF_Dataset", help="STF subset root directory.")
    parser.add_argument("--rate", type=float, default=2.0, help="Publish rate in Hz.")
    parser.add_argument("--limit", type=int, default=0, help="Optional number of frames to publish.")
    parser.add_argument("--loop", action="store_true", help="Loop after the last frame.")
    parser.add_argument("--frame-id", default="gated_camera_link", help="ROS frame id.")
    parser.add_argument("--near-topic", default="/gated_camera/slice_near")
    parser.add_argument("--mid-topic", default="/gated_camera/slice_mid")
    parser.add_argument("--far-topic", default="/gated_camera/slice_far")
    return parser.parse_args()


def common_stems(dataset: Path) -> list[str]:
    sets = []
    for name in ["gated0_rect8", "gated1_rect8", "gated2_rect8"]:
        sets.append({p.stem for p in (dataset / name).glob("*.png")})
    stems = sorted(set.intersection(*sets))
    return stems


def image_msg(path: Path, frame_id: str, stamp) -> Image:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(path)
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = image.shape[:2]
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = msg.width
    msg.data = image.tobytes()
    return msg


class StfSlicePublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("stf_slice_publisher")
        self.args = args
        self.dataset = Path(args.dataset).expanduser()
        self.stems = common_stems(self.dataset)
        if args.limit > 0:
            self.stems = self.stems[: args.limit]
        if not self.stems:
            raise RuntimeError(f"No synchronized gated slices found under {self.dataset}")
        self.index = 0
        self.done = False
        self.near_pub = self.create_publisher(Image, args.near_topic, qos_profile_sensor_data)
        self.mid_pub = self.create_publisher(Image, args.mid_topic, qos_profile_sensor_data)
        self.far_pub = self.create_publisher(Image, args.far_topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / max(args.rate, 0.1), self.on_timer)
        self.get_logger().info(f"Publishing {len(self.stems)} STF frames from {self.dataset}")

    def on_timer(self) -> None:
        if self.index >= len(self.stems):
            if self.args.loop:
                self.index = 0
            else:
                self.get_logger().info("Finished STF slice replay")
                self.done = True
                self.timer.cancel()
                return
        stem = self.stems[self.index]
        stamp = self.get_clock().now().to_msg()
        self.near_pub.publish(image_msg(self.dataset / "gated0_rect8" / f"{stem}.png", self.args.frame_id, stamp))
        self.mid_pub.publish(image_msg(self.dataset / "gated1_rect8" / f"{stem}.png", self.args.frame_id, stamp))
        self.far_pub.publish(image_msg(self.dataset / "gated2_rect8" / f"{stem}.png", self.args.frame_id, stamp))
        self.index += 1


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = StfSlicePublisher(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
