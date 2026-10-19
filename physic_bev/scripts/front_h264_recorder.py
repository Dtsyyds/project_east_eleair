#!/usr/bin/env python3
"""Mux tested S100P hobot_codec H.264 ROS frames into MKV plus sidecars."""

from __future__ import annotations

import argparse
from concurrent.futures import Future, ThreadPoolExecutor
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import time
from typing import Any

import rclpy
from img_msgs.msg import H26XFrame
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import atomic_json, utc_now  # noqa: E402


def _stamp_ns(value: Any) -> int:
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


from inspection_tools.video_index import create_frame_map

class Segment:
    def __init__(self, output: Path, index: int, fps: float, parameter_sets: bytes) -> None:
        self.index = index
        self.fps = fps
        self.frames = 0
        self.first_stamp_ns: int | None = None
        self.last_stamp_ns: int | None = None
        stem = f"front_{index:06d}"
        self.raw_partial = output / f"{stem}.h264.partial"
        self.video_partial = output / f"{stem}.mkv.partial"
        self.video_final = output / f"{stem}.mkv"
        self.sidecar_partial = output / f"{stem}.frames.jsonl.partial"
        self.sidecar_final = output / f"{stem}.frames.jsonl"
        self.log_path = output / f"{stem}.mux.log"
        self.raw = self.raw_partial.open("wb")
        if parameter_sets:
            self.raw.write(parameter_sets)
        self.sidecar = self.sidecar_partial.open("w", encoding="utf-8", buffering=1)

    def write(self, frame: dict[str, Any]) -> None:
        raw_offset = self.raw.tell()
        self.raw.write(frame["data"])
        stamp = int(frame["ros_header_stamp_ns"])
        self.first_stamp_ns = stamp if self.first_stamp_ns is None else self.first_stamp_ns
        self.last_stamp_ns = stamp
        relative_source_ns = stamp - self.first_stamp_ns
        self.sidecar.write(json.dumps({
            "schema_version": "source_frame_journal_v2",
            "capture_state": "encoded_access_unit_received",
            "enqueue_state": "accepted", "write_state": "raw_access_unit_written",
            "raw_offset": raw_offset, "raw_size": len(frame["data"]),
            "frame_index": self.frames,
            "segment_index": self.index,
            "ros_header_stamp_ns": stamp,
            "device_pts_ns": frame["pts_ns"],
            "device_dts_ns": frame["dts_ns"],
            "receive_monotonic_ns": frame["receive_monotonic_ns"],
            "encoding_input_ros_stamp_ns": frame["pts_ns"],
            "encoding_input_monotonic_ns": None,
            "encoding_input_note": (
                "hobot_codec preserves the source ROS stamp in H26XFrame.pts but does not "
                "expose its internal host-monotonic enqueue time"),
            # The source ROS clock is authoritative.  A fixed nominal FPS made
            # an 8.9 Hz hardware stream play about 12% too fast in the old W6
            # run even though every access unit was retained.
            "encoded_pts_ns": relative_source_ns,
            "width": frame["width"], "height": frame["height"],
            "encoding": "h264", "source_sequence": frame["source_sequence"],
            "is_idr": frame["is_idr"],
            "dropped_before_encode": frame["dropped_before_encode"],
            "encoder_queue_depth": frame["encoder_queue_depth"],
        }, ensure_ascii=False) + "\n")
        self.frames += 1

    def close(self) -> dict[str, Any]:
        self.raw.flush(); os.fsync(self.raw.fileno()); self.raw.close()
        self.sidecar.flush(); os.fsync(self.sidecar.fileno()); self.sidecar.close()
        source_span_sec = (
            (self.last_stamp_ns - self.first_stamp_ns) / 1e9
            if self.frames > 1 and self.first_stamp_ns is not None
            and self.last_stamp_ns is not None and self.last_stamp_ns > self.first_stamp_ns
            else 0.0
        )
        mux_fps = ((self.frames - 1) / source_span_sec
                   if source_span_sec > 0 else self.fps)
        with self.log_path.open("w", encoding="utf-8") as log:
            result = subprocess.run([
                "ffmpeg", "-hide_banner", "-loglevel", "warning", "-fflags", "+genpts",
                "-r", f"{mux_fps:.9f}", "-f", "h264", "-i", str(self.raw_partial),
                "-an", "-c:v", "copy", "-f", "matroska", "-y", str(self.video_partial),
            ], stdout=subprocess.DEVNULL, stderr=log, timeout=120, check=False)
        complete = result.returncode == 0 and self.frames > 0 and self.video_partial.is_file()
        if complete:
            mapping_partial = self.video_final.with_suffix(".frame_map.json.partial")
            create_frame_map(self.video_partial, self.sidecar_partial, mapping_partial, self.raw_partial)
            os.replace(self.video_partial, self.video_final)
            os.replace(self.sidecar_partial, self.sidecar_final)
            os.replace(mapping_partial, self.video_final.with_suffix(".frame_map.json"))
            self.raw_partial.unlink()
        return {
            "index": self.index, "frames": self.frames, "returncode": result.returncode,
            "complete": complete,
            "video": str(self.video_final if complete else self.video_partial),
            "sidecar": str(self.sidecar_final if complete else self.sidecar_partial),
            "first_ros_header_stamp_ns": self.first_stamp_ns,
            "last_ros_header_stamp_ns": self.last_stamp_ns,
            "source_span_sec": source_span_sec,
            "mux_fps": mux_fps,
        }


def _annexb_details(data: bytes) -> tuple[bytes, bool]:
    """Return SPS/PPS bytes and IDR presence from an Annex-B access unit.

    ``bytes.find`` performs the large-buffer scan in C. A Python byte-by-byte
    scan here cuts the 4K live callback rate roughly in half on the target SoC.
    Searching for the three-byte marker also finds a four-byte start code at
    its second zero, which remains a valid three-byte Annex-B prefix.
    """
    marker = b"\x00\x00\x01"
    starts: list[int] = []
    offset = data.find(marker)
    while offset >= 0:
        starts.append(offset)
        offset = data.find(marker, offset + len(marker))
    output = bytearray()
    contains_idr = False
    for index, start in enumerate(starts):
        header = start + len(marker)
        end = starts[index + 1] if index + 1 < len(starts) else len(data)
        if header >= end:
            continue
        nal_type = data[header] & 0x1F
        if nal_type in {7, 8}:
            output.extend(data[start:end])
        elif nal_type == 5:
            contains_idr = True
    return bytes(output), contains_idr


def _annexb_parameter_sets(data: bytes) -> bytes:
    return _annexb_details(data)[0]


def _annexb_contains_idr(data: bytes) -> bool:
    return _annexb_details(data)[1]


class Recorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("field_w5_front_h264_recorder")
        self.args = args
        self.output = Path(args.output).resolve(); self.output.mkdir(parents=True, exist_ok=True)
        self.status_path = Path(args.status_file).resolve()
        self.queue: queue.Queue[dict[str, Any]] = queue.Queue(maxsize=args.queue_size)
        self.stop_event = threading.Event()
        self.received = self.enqueued = self.encoded = self.dropped = self.write_errors = 0
        self.pre_idr_discarded = 0
        self.first_stamp_ns: int | None = None
        self.last_stamp_ns: int | None = None
        self.last_error: str | None = None
        self.parameter_sets = b""
        self.segments: list[dict[str, Any]] = []
        self.segment_index = 0
        self.mux_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="h264-mux")
        self.mux_futures: list[Future[dict[str, Any]]] = []
        self.started_monotonic_ns = time.monotonic_ns()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=args.subscription_depth,
            reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(H26XFrame, args.topic, self.callback, qos)
        self.worker = threading.Thread(target=self.encode_loop, name="h264-mkv-writer", daemon=False)
        self.worker.start()
        self.timer = self.create_timer(1.0, self.write_status)

    def callback(self, message: H26XFrame) -> None:
        self.received += 1
        data = bytes(message.data)
        found, is_idr = _annexb_details(data)
        if found:
            self.parameter_sets = found
        pts_ns = _stamp_ns(message.pts)
        self.first_stamp_ns = pts_ns if self.first_stamp_ns is None else self.first_stamp_ns
        self.last_stamp_ns = pts_ns
        item = {
            "data": data, "pts_ns": pts_ns, "dts_ns": _stamp_ns(message.dts),
            "ros_header_stamp_ns": pts_ns, "receive_monotonic_ns": time.monotonic_ns(),
            "is_idr": is_idr,
            "width": int(message.width), "height": int(message.height),
            "source_sequence": int(message.index), "dropped_before_encode": self.dropped,
            "encoder_queue_depth": self.queue.qsize(),
        }
        try:
            self.queue.put_nowait(item); self.enqueued += 1
        except queue.Full:
            self.dropped += 1

    def encode_loop(self) -> None:
        segment: Segment | None = None
        index = 1
        while not self.stop_event.is_set() or not self.queue.empty():
            try:
                frame = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if segment is None and not frame["is_idr"]:
                    self.pre_idr_discarded += 1
                    continue
                rotate = segment is not None and segment.first_stamp_ns is not None and (
                    frame["ros_header_stamp_ns"] - segment.first_stamp_ns
                    >= int(self.args.segment_sec * 1e9)
                ) and frame["is_idr"]
                if segment is None or rotate:
                    if segment is not None:
                        # Remuxing a five-minute 4K segment can take longer than the
                        # live queue can absorb.  Hand the closed segment to a
                        # dedicated worker so the next access unit is written
                        # immediately instead of blocking the subscriber pipeline.
                        self.mux_futures.append(self.mux_executor.submit(segment.close))
                        index += 1
                    segment = Segment(self.output, index, self.args.fps, self.parameter_sets)
                    self.segment_index = index
                segment.write(frame); self.encoded += 1
            except Exception as exc:
                self.write_errors += 1; self.last_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(self.last_error); self.stop_event.set()
            finally:
                self.queue.task_done()
        if segment is not None:
            self.mux_futures.append(self.mux_executor.submit(segment.close))
        self.mux_executor.shutdown(wait=True)
        for future in self.mux_futures:
            try:
                self.segments.append(future.result())
            except Exception as exc:
                self.write_errors += 1
                self.last_error = f"{type(exc).__name__}: {exc}"
                self.get_logger().error(self.last_error)
        self.segments.sort(key=lambda value: int(value["index"]))

    def document(self) -> dict[str, Any]:
        process_elapsed = max(1e-9, (time.monotonic_ns() - self.started_monotonic_ns) / 1e9)
        stream_elapsed = (
            (self.last_stamp_ns - self.first_stamp_ns) / 1e9
            if self.first_stamp_ns is not None and self.last_stamp_ns is not None
            and self.last_stamp_ns > self.first_stamp_ns else 0.0
        )
        output_bytes = sum(
            path.stat().st_size
            for pattern in ("front_*.mkv", "front_*.h264.partial")
            for path in self.output.glob(pattern)
        )
        return {
            "schema_version": "field_w5_front_video_status_v1", "updated_at": utc_now(),
            "topic": self.args.topic, "encoder": "hobot_codec S100P VPU H.264",
            "encoder_package_version": "2.3.5", "bitrate_mbps": None,
            "measured_bitrate_mbps": output_bytes * 8 / stream_elapsed / 1e6 if stream_elapsed else None,
            "received_frames": self.received, "enqueued_frames": self.enqueued,
            "encoded_frames": self.encoded,
            "input_hz": (self.received - 1) / stream_elapsed if stream_elapsed and self.received > 1 else None,
            "encoded_hz": (self.encoded - 1) / stream_elapsed if stream_elapsed and self.encoded > 1 else None,
            "process_elapsed_sec": process_elapsed, "stream_elapsed_sec": stream_elapsed,
            "queue_depth": self.queue.qsize(),
            "queue_capacity": self.args.queue_size, "queue_drop_count": self.dropped,
            "pre_idr_discard_count": self.pre_idr_discarded,
            "write_error_count": self.write_errors, "last_error": self.last_error,
            "current_segment": self.segment_index, "closed_segments": self.segments,
            "complete": self.stop_event.is_set() and not self.worker.is_alive()
            and self.write_errors == 0 and all(item["complete"] for item in self.segments),
        }

    def write_status(self) -> None:
        atomic_json(self.status_path, self.document())

    def close(self) -> None:
        self.stop_event.set(); self.worker.join(timeout=180)
        if self.worker.is_alive():
            self.write_errors += 1; self.last_error = "writer did not stop within 180 seconds"
        self.write_status()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/front/image_h264")
    parser.add_argument("--output", required=True); parser.add_argument("--status-file", required=True)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--segment-sec", type=float, default=300.0)
    parser.add_argument("--queue-size", type=int, default=16)
    parser.add_argument("--subscription-depth", type=int, default=4)
    args = parser.parse_args(argv)
    rclpy.init(args=None); node = Recorder(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(node.subscription); node.close(); errors = node.write_errors
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0 if errors == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
