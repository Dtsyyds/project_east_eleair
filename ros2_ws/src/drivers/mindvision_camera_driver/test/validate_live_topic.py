#!/usr/bin/env python3
"""Bounded live Topic validator; it never stores image payloads."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class Validator(Node):
    def __init__(
        self, image_topic: str, info_topic: str, frames: int,
        width: int, height: int, frame_id: str,
    ):
        super().__init__("mindvision_topic_validator")
        self.frames_required = frames
        self.width_required = width
        self.height_required = height
        self.frame_id_required = frame_id
        self.images = 0
        self.infos = 0
        self.matched_headers = 0
        self.errors: list[str] = []
        self.first_receive_ns = None
        self.last_receive_ns = None
        self.last_stamp = None
        self._image_stamps = set()
        self._info_stamps = set()
        self.first_frame_sample = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, image_topic, self._image, qos)
        self.create_subscription(CameraInfo, info_topic, self._info, qos)

    @staticmethod
    def _stamp(message):
        return (int(message.header.stamp.sec), int(message.header.stamp.nanosec))

    def _image(self, message: Image):
        receive_ns = time.monotonic_ns()
        stamp = self._stamp(message)
        if self.first_receive_ns is None:
            self.first_receive_ns = receive_ns
        self.last_receive_ns = receive_ns
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.errors.append("image header timestamps are not strictly monotonic")
        self.last_stamp = stamp
        if message.encoding != "bgr8":
            self.errors.append(f"encoding={message.encoding!r}, expected 'bgr8'")
        if int(message.width) != self.width_required or int(message.height) != self.height_required:
            self.errors.append(f"shape={message.width}x{message.height}, expected {self.width_required}x{self.height_required}")
        if int(message.step) != self.width_required * 3:
            self.errors.append(f"step={message.step}, expected {self.width_required * 3}")
        if len(message.data) != self.width_required * self.height_required * 3:
            self.errors.append(f"data length={len(message.data)}, expected {self.width_required * self.height_required * 3}")
        if message.header.frame_id != self.frame_id_required:
            self.errors.append(
                f"frame_id={message.header.frame_id!r}, expected {self.frame_id_required!r}"
            )
        if self.first_frame_sample is None and message.data:
            stride = max(3, len(message.data) // 4096)
            sample = list(message.data[::stride])
            self.first_frame_sample = {
                "count": len(sample), "min": min(sample), "max": max(sample),
                "mean": round(sum(sample) / len(sample), 3),
            }
            if self.first_frame_sample["min"] == self.first_frame_sample["max"]:
                self.errors.append("sampled image content is constant")
        self._image_stamps.add(stamp)
        if stamp in self._info_stamps:
            self.matched_headers += 1
        self.images += 1

    def _info(self, message: CameraInfo):
        stamp = self._stamp(message)
        if int(message.width) != self.width_required or int(message.height) != self.height_required:
            self.errors.append("CameraInfo dimensions do not match expected image dimensions")
        if float(message.k[0]) != 0.0:
            self.errors.append("CameraInfo K[0] is non-zero although calibration is not complete")
        self.infos += 1
        self._info_stamps.add(stamp)
        if stamp in self._image_stamps:
            self.matched_headers += 1

    def result(self):
        elapsed = 0.0
        if self.first_receive_ns is not None and self.last_receive_ns is not None:
            elapsed = (self.last_receive_ns - self.first_receive_ns) / 1_000_000_000
        fps = (self.images - 1) / elapsed if elapsed > 0 and self.images > 1 else 0.0
        return {
            "images": self.images,
            "camera_infos": self.infos,
            "matched_headers": self.matched_headers,
            "receive_fps": round(fps, 6),
            "frame_id": self.frame_id_required,
            "first_frame_sample": self.first_frame_sample,
            "errors": self.errors,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/camera/front/image_raw")
    parser.add_argument("--info-topic", default="/camera/front/camera_info")
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--width", type=int, default=4096)
    parser.add_argument("--height", type=int, default=3072)
    parser.add_argument("--frame-id", default="mindvision_front_optical_frame")
    parser.add_argument("--min-fps", type=float, default=0.1)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    args = parser.parse_args()
    rclpy.init()
    node = Validator(
        args.image_topic, args.info_topic, args.frames,
        args.width, args.height, args.frame_id,
    )
    deadline = time.monotonic() + args.timeout_s
    try:
        while time.monotonic() < deadline and (
            node.images < args.frames or node.matched_headers < args.frames
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
        result = node.result()
        print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        return int(
            node.images < args.frames
            or node.matched_headers < args.frames
            or result["receive_fps"] < args.min_fps
            or bool(node.errors)
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
