#!/usr/bin/env python3
"""Encode a ROS Image topic into recoverable MKV segments with timestamp sidecars.

This process is only a ROS subscriber. It never imports or opens the MindVision
SDK, preserving the single-camera-owner invariant.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import atomic_json, utc_now  # noqa: E402


def stamp_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


from inspection_tools.video_index import create_frame_map

class Segment:
    def __init__(
        self, output: Path, index: int, width: int, height: int, encoding: str,
        fps: float, bitrate_mbps: float,
    ) -> None:
        self.output = output
        self.index = index
        self.width = width
        self.height = height
        self.encoding = encoding.lower()
        self.fps = fps
        self.bitrate_mbps = bitrate_mbps
        self.frames = 0
        self.first_stamp_ns: int | None = None
        self.last_stamp_ns: int | None = None
        stem = f"front_{index:06d}"
        self.video_final = output / f"{stem}.mkv"
        self.video_partial = output / f"{stem}.mkv.partial"
        self.sidecar_final = output / f"{stem}.frames.jsonl"
        self.sidecar_partial = output / f"{stem}.frames.jsonl.partial"
        self.log_path = output / f"{stem}.encoder.log"
        pix_fmt = {"bgr8": "bgr24", "rgb8": "rgb24", "mono8": "gray"}.get(self.encoding)
        if not pix_fmt:
            raise ValueError(f"unsupported ROS encoding for video: {encoding}")
        self.sidecar = self.sidecar_partial.open("w", encoding="utf-8", buffering=1)
        self.log = self.log_path.open("w", encoding="utf-8")
        command = [
            "ffmpeg", "-hide_banner", "-loglevel", "warning", "-nostdin",
            "-f", "rawvideo", "-pix_fmt", pix_fmt,
            "-video_size", f"{width}x{height}", "-framerate", f"{fps:g}",
            "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "ultrafast",
            "-tune", "zerolatency", "-b:v", f"{bitrate_mbps:g}M",
            "-maxrate", f"{bitrate_mbps:g}M", "-bufsize", f"{bitrate_mbps * 2:g}M",
            "-g", str(max(1, round(fps))), "-pix_fmt", "yuv420p",
            "-f", "matroska", "-y", str(self.video_partial),
        ]
        self.process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log,
        )

    def write(self, frame: dict[str, Any]) -> None:
        data = frame["data"]
        expected_step = self.width * {"bgr8": 3, "rgb8": 3, "mono8": 1}[self.encoding]
        if frame["step"] != expected_step or len(data) != expected_step * self.height:
            raise ValueError(
                f"non-contiguous image: step={frame['step']} expected={expected_step} "
                f"bytes={len(data)} expected_bytes={expected_step * self.height}"
            )
        if self.process.stdin is None:
            raise BrokenPipeError("ffmpeg stdin is closed")
        encode_input_ns = time.monotonic_ns()
        self.process.stdin.write(data)
        source_stamp = int(frame["header_stamp_ns"])
        self.first_stamp_ns = source_stamp if self.first_stamp_ns is None else self.first_stamp_ns
        self.last_stamp_ns = source_stamp
        item = {
            "schema_version": "source_frame_journal_v2",
            "capture_state": "image_received", "enqueue_state": "accepted",
            "write_state": "encoder_stdin_written",
            "encoder_contract": "rawvideo_fixed_fps_unique_pts_v2",
            "frame_index": self.frames,
            "segment_index": self.index,
            "ros_header_stamp_ns": source_stamp,
            "receive_monotonic_ns": frame["receive_monotonic_ns"],
            "encoding_input_monotonic_ns": encode_input_ns,
            "encoded_pts_ns": round(self.frames * 1_000_000_000 / self.fps),
            "width": self.width,
            "height": self.height,
            "encoding": self.encoding,
            "source_sequence": None,
            "source_sequence_note": "ROS 2 sensor_msgs/Image has no sequence field",
            "received_sequence": frame["received_sequence"],
            "dropped_before_encode": frame["dropped_before_encode"],
            "encoder_queue_depth": frame["encoder_queue_depth"],
        }
        self.sidecar.write(json.dumps(item, ensure_ascii=False) + "\n")
        self.frames += 1

    def close(self) -> dict[str, Any]:
        if self.process.stdin is not None:
            try:
                self.process.stdin.close()
            except BrokenPipeError:
                pass
        try:
            returncode = self.process.wait(timeout=90)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                returncode = self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                returncode = self.process.wait(timeout=5)
        self.sidecar.flush()
        os.fsync(self.sidecar.fileno())
        self.sidecar.close()
        self.log.close()
        complete = returncode == 0 and self.frames > 0 and self.video_partial.is_file()
        if complete:
            mapping_partial = self.video_final.with_suffix(".frame_map.json.partial")
            create_frame_map(self.video_partial, self.sidecar_partial, mapping_partial)
            os.replace(self.video_partial, self.video_final)
            os.replace(self.sidecar_partial, self.sidecar_final)
            os.replace(mapping_partial, self.video_final.with_suffix(".frame_map.json"))
        return {
            "index": self.index, "frames": self.frames, "returncode": returncode,
            "complete": complete,
            "video": str(self.video_final if complete else self.video_partial),
            "sidecar": str(self.sidecar_final if complete else self.sidecar_partial),
            "first_ros_header_stamp_ns": self.first_stamp_ns,
            "last_ros_header_stamp_ns": self.last_stamp_ns,
        }


class FrontVideoRecorder(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("field_w5_front_video_recorder")
        self.args = args
        self.output = Path(args.output).resolve()
        self.output.mkdir(parents=True, exist_ok=True)
        self.status_path = Path(args.status_file).resolve()
        self.queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=args.queue_size)
        self.stop_event = threading.Event()
        self.received = 0
        self.enqueued = 0
        self.encoded = 0
        self.dropped = 0
        self.write_errors = 0
        self.last_error: str | None = None
        self.segments: list[dict[str, Any]] = []
        self.started_monotonic_ns = time.monotonic_ns()
        self.last_status_monotonic = 0.0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=args.subscription_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(Image, args.topic, self.received_image, qos)
        self.worker = threading.Thread(target=self.encode_loop, name="front-video-encoder", daemon=False)
        self.worker.start()
        self.timer = self.create_timer(1.0, self.write_status)

    def received_image(self, message: Image) -> None:
        self.received += 1
        item = {
            "data": bytes(message.data),
            "header_stamp_ns": stamp_ns(message),
            "receive_monotonic_ns": time.monotonic_ns(),
            "width": int(message.width), "height": int(message.height),
            "encoding": str(message.encoding), "step": int(message.step),
            "received_sequence": self.received - 1,
            "dropped_before_encode": self.dropped,
            "encoder_queue_depth": self.queue.qsize(),
        }
        try:
            self.queue.put_nowait(item)
            self.enqueued += 1
        except queue.Full:
            self.dropped += 1

    def encode_loop(self) -> None:
        segment: Segment | None = None
        segment_index = 1
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                frame = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                rotate = False
                if segment is not None and segment.first_stamp_ns is not None:
                    rotate = (
                        int(frame["header_stamp_ns"]) - segment.first_stamp_ns
                        >= int(self.args.segment_sec * 1_000_000_000)
                    )
                if segment is None or rotate:
                    if segment is not None:
                        self.segments.append(segment.close())
                        segment_index += 1
                    segment = Segment(
                        self.output, segment_index, frame["width"], frame["height"],
                        frame["encoding"], self.args.fps, self.args.bitrate_mbps,
                    )
                segment.write(frame)
                self.encoded += 1
            except Exception as exc:  # encoder boundary; retain evidence and continue shutdown
                self.write_errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(self.last_error)
                self.stop_event.set()
            finally:
                self.queue.task_done()
        if segment is not None:
            self.segments.append(segment.close())

    def status_document(self) -> dict[str, Any]:
        elapsed = max(1e-9, (time.monotonic_ns() - self.started_monotonic_ns) / 1e9)
        return {
            "schema_version": "field_w5_front_video_status_v1",
            "updated_at": utc_now(),
            "topic": self.args.topic,
            "encoder": "ffmpeg/libx264",
            "encoder_tested_profile": "4096x3072@10, ultrafast, yuv420p, Matroska",
            "bitrate_mbps": self.args.bitrate_mbps,
            "received_frames": self.received,
            "enqueued_frames": self.enqueued,
            "encoded_frames": self.encoded,
            "input_hz": self.received / elapsed,
            "encoded_hz": self.encoded / elapsed,
            "queue_depth": self.queue.qsize(),
            "queue_capacity": self.args.queue_size,
            "queue_drop_count": self.dropped,
            "write_error_count": self.write_errors,
            "last_error": self.last_error,
            "current_segment": len(self.segments) + 1,
            "closed_segments": self.segments,
            "complete": self.stop_event.is_set() and not self.worker.is_alive()
            and self.write_errors == 0 and all(item["complete"] for item in self.segments),
        }

    def write_status(self) -> None:
        atomic_json(self.status_path, self.status_document())

    def close(self) -> None:
        self.stop_event.set()
        self.worker.join(timeout=120)
        if self.worker.is_alive():
            self.write_errors += 1
            self.last_error = "encoder worker did not stop within 120 seconds"
        self.write_status()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/front/image_raw")
    parser.add_argument("--output", required=True)
    parser.add_argument("--status-file", required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--bitrate-mbps", type=float, default=12.0)
    parser.add_argument("--segment-sec", type=float, default=300.0)
    parser.add_argument("--queue-size", type=int, default=4)
    parser.add_argument("--subscription-depth", type=int, default=1)
    args = parser.parse_args(argv)
    if args.fps <= 0 or args.segment_sec <= 0 or args.queue_size < 1:
        parser.error("fps and segment-sec must be positive; queue-size must be at least one")
    rclpy.init(args=None)
    node = FrontVideoRecorder(args)
    exit_code = 0
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(node.subscription)
        node.close()
        exit_code = 0 if node.write_errors == 0 else 2
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
