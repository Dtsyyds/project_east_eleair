#!/usr/bin/env python3
"""Rebuild a compact JSONL trajectory from recorded /aft_mapped_to_init."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from validate_vehicle_collection import _odometry_record  # noqa: E402
from vehicle_collection_common import atomic_json, atomic_write_text, utc_now  # noqa: E402


def rebuild(bag: Path, output: Path) -> dict[str, object]:
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    expected = "nav_msgs/msg/Odometry"
    if topic_types.get("/aft_mapped_to_init") != expected:
        raise RuntimeError(
            f"/aft_mapped_to_init has type {topic_types.get('/aft_mapped_to_init')!r}, "
            f"expected {expected}"
        )
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/aft_mapped_to_init"]))
    records: list[dict[str, object]] = []
    regressions = 0
    last_stamp: int | None = None
    while reader.has_next():
        topic, serialized, bag_timestamp = reader.read_next()
        if topic != "/aft_mapped_to_init":
            continue
        message = deserialize_message(serialized, Odometry)
        record = _odometry_record(message, bag_timestamp, len(records))
        stamp = record["header_stamp_ns"]
        if isinstance(stamp, int) and last_stamp is not None and stamp < last_stamp:
            regressions += 1
        if isinstance(stamp, int):
            last_stamp = stamp
        records.append(record)
    if not records:
        raise RuntimeError("/aft_mapped_to_init contains no messages")
    atomic_write_text(
        output,
        "".join(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n"
                for item in records),
    )
    return {
        "schema_version": "field_w6_compact_trajectory_v1",
        "created_at": utc_now(),
        "valid": regressions == 0,
        "source_bag": str(bag),
        "source_topic": "/aft_mapped_to_init",
        "output": str(output),
        "format": "jsonl_pose_v1",
        "count": len(records),
        "first_header_stamp_ns": records[0]["header_stamp_ns"],
        "last_header_stamp_ns": records[-1]["header_stamp_ns"],
        "header_stamp_regressions": regressions,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--summary", type=Path)
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    output = (args.output or run_dir / "reports" / "compact_trajectory.jsonl").resolve()
    summary = (args.summary or run_dir / "reports" / "compact_trajectory_summary.json").resolve()
    report = rebuild(run_dir / "mcap" / "mapping", output)
    atomic_json(summary, report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
