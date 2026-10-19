#!/usr/bin/env python3
"""Summarize measured closed visual MCAP throughput; never fabricates a result."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import yaml


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--input-hz", type=float, required=True)
    parser.add_argument("--image-topic", default="/camera/front/image_raw")
    parser.add_argument("--width", type=int, default=4096)
    parser.add_argument("--height", type=int, default=3072)
    parser.add_argument("--bytes-per-pixel", type=float, default=3.0)
    parser.add_argument("--format", default="bgr8")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    info = subprocess.run(["ros2", "bag", "info", str(args.bag)], capture_output=True, text=True, check=False)
    metadata_path = args.bag / "metadata.yaml"
    if info.returncode != 0 or not metadata_path.is_file():
        print(info.stdout + info.stderr)
        return 1
    metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
    duration_ns = int(metadata.get("duration", {}).get("nanoseconds", 0))
    files = [args.bag / value for value in metadata.get("relative_file_paths", [])]
    size = sum(path.stat().st_size for path in files if path.is_file())
    seconds = duration_ns / 1e9
    topics = {item["topic_metadata"]["name"]: item["message_count"]
              for item in metadata.get("topics_with_message_count", [])}
    image_count = int(topics.get(args.image_topic, 0))
    recorder_written_hz = image_count / seconds if seconds > 0 else None
    payload_bytes = args.width * args.height * args.bytes_per_pixel
    required_payload_MBps = payload_bytes * args.input_hz / 1e6
    report = {"bag": str(args.bag.resolve()), "input_hz": args.input_hz,
              "duration_sec": seconds, "mcap_bytes": size,
              "actual_write_MBps": size / seconds / 1e6 if seconds > 0 else None,
              "image_topic": args.image_topic, "image_format": args.format,
              "image_dimensions": [args.width, args.height],
              "payload_bytes_per_frame": payload_bytes,
              "required_payload_MBps_at_input_rate": required_payload_MBps,
              "recorder_written_frames": image_count,
              "recorder_written_hz": recorder_written_hz,
              "recorder_capture_ratio": (
                  recorder_written_hz / args.input_hz
                  if recorder_written_hz is not None and args.input_hz > 0 else None),
              "file_size_60_sec_MB": size / seconds * 60 / 1e6 if seconds > 0 else None,
              "compression_ratio": "requires uncompressed byte counter; not inferred",
              "writer_drop": "read from /diagnostics and rosbag recorder output; not inferred",
              "cpu": "capture process evidence required; not inferred",
              "stop_finalize_time_sec": "measure from SIGINT to metadata close in stop log",
              "stage_semantics": {
                  "input_hz": "independently measured ROS publisher/CameraInfo rate",
                  "recorder_written_hz": "messages present in the closed MCAP; recorder receive and disk commit cannot be separated by rosbag2 metadata",
                  "actual_write_MBps": "closed MCAP bytes divided by metadata duration",
              },
              "topics": topics, "source": "closed MCAP metadata"}
    output = args.output or (args.bag / "throughput_report.json")
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
