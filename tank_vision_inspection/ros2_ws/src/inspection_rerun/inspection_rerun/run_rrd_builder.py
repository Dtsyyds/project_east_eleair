"""Post-collection, run-level MCAP/MKV to Rerun builder.

This module never starts a ROS graph participant.  It streams selected messages
from rosbag2 readers, merges the reader heads by recorded timestamp, and writes
directly to a headless Rerun recording.  Source data is read-only throughout.
"""

from __future__ import annotations

from inspection_tools.video_index import aligned_rows, VideoAlignmentError

import argparse
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
import hashlib
import heapq
import json
import logging
import math
import os
from pathlib import Path
import platform
import re
import resource
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Any, Iterable

import numpy as np
import yaml

from .image_adapter import image_to_numpy
from .imu_adapter import imu_values
from .inspection_adapter import diagnostic_level
from .pointcloud_adapter import pointcloud_to_numpy
from .rerun_session import RerunSession
from .tf_adapter import transform_values
from .timestamp_utils import RateLimiter, TimelineTracker, message_timestamp_ns, stamp_to_nanoseconds


FROZEN_RERUN_VERSION = "0.36.2"
RUN_ID_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
SUPPORTED_TYPES = {
    "diagnostic_msgs/msg/DiagnosticArray",
    "geometry_msgs/msg/PoseStamped",
    "nav_msgs/msg/Odometry",
    "nav_msgs/msg/Path",
    "sensor_msgs/msg/CameraInfo",
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/PointCloud2",
    "std_msgs/msg/String",
    "tf2_msgs/msg/TFMessage",
    "eddy_driver/msg/EddyChannelMean",
    "eddy_driver/msg/EddyStatus",
}


class BuildError(RuntimeError):
    """A user-visible gate, conversion, or validation failure."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_json(path: Path, *, required: bool = True) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        if required:
            raise BuildError(f"cannot parse required JSON {path}: {exc}") from exc
        return {}
    if not isinstance(value, dict):
        raise BuildError(f"JSON document must be an object: {path}")
    return value


def atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.partial")
    with temporary.open("w", encoding="utf-8") as stream:
        stream.write(text)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def atomic_json(path: Path, value: Any) -> None:
    atomic_text(path, json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def is_relative_to(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def sanitize_entity(topic: str) -> str:
    parts = [re.sub(r"[^A-Za-z0-9_.-]+", "_", item) or "unnamed" for item in topic.split("/") if item]
    return "/ros/" + "/".join(parts or ["unnamed"])


def process_start_ticks(pid: int) -> int | None:
    try:
        tail = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").rsplit(")", 1)[1].split()
        return int(tail[19])
    except (OSError, ValueError, IndexError):
        return None


def process_cmdline(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(
            "utf-8", errors="replace")
    except OSError:
        return ""


def current_rss_bytes() -> int:
    try:
        for line in Path("/proc/self/status").read_text(encoding="utf-8").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    return 0


@dataclass
class TopicStatistics:
    type: str = ""
    source_count: int = 0
    received_count: int = 0
    logged_count: int = 0
    rate_limited_count: int = 0
    parse_failure_count: int = 0
    invalid_point_count: int = 0
    capped_point_count: int = 0
    camera_decode_failure_count: int = 0
    timestamp_fallback_count: int = 0
    timestamp_regression_count: int = 0
    first_source_time_ns: int | None = None
    last_source_time_ns: int | None = None
    first_logged_time_ns: int | None = None
    last_logged_time_ns: int | None = None

    def observe_source_time(self, value: int | None) -> None:
        if value is None:
            return
        value = int(value)
        self.first_source_time_ns = value if self.first_source_time_ns is None else min(self.first_source_time_ns, value)
        self.last_source_time_ns = value if self.last_source_time_ns is None else max(self.last_source_time_ns, value)

    def observe_logged_time(self, value: int) -> None:
        value = int(value)
        self.first_logged_time_ns = value if self.first_logged_time_ns is None else min(self.first_logged_time_ns, value)
        self.last_logged_time_ns = value if self.last_logged_time_ns is None else max(self.last_logged_time_ns, value)


@dataclass
class BagSource:
    group: str
    path: Path
    topics: dict[str, str]
    topic_counts: dict[str, int] = field(default_factory=dict)
    topic_ranges: dict[str, tuple[int | None, int | None]] = field(default_factory=dict)
    start_ns: int | None = None
    end_ns: int | None = None


@dataclass
class CameraSegment:
    video: Path
    sidecar: Path
    frame_count: int
    first_ns: int | None
    last_ns: int | None


@dataclass
class BuildOptions:
    run_dir: Path
    config: Path
    output: Path
    mode: str = "strict"
    ros_domain_id: int = 184
    replay_rate: float = 1.0
    timeout_sec: float = 0.0
    include_camera: bool = False
    camera_mode: str = "none"
    force: bool = False
    dry_run: bool = False
    keep_partial_on_failure: bool = False
    log_level: str = "INFO"

    def normalized(self) -> "BuildOptions":
        run_dir = self.run_dir.expanduser().resolve()
        config = self.config.expanduser().resolve()
        output = self.output.expanduser().resolve()
        camera_mode = "preview" if self.include_camera and self.camera_mode == "none" else self.camera_mode
        if self.mode == "allow-incomplete" and "_INCOMPLETE" not in output.stem:
            output = output.with_name(output.stem + "_INCOMPLETE" + output.suffix)
        return BuildOptions(
            run_dir=run_dir, config=config, output=output, mode=self.mode,
            ros_domain_id=int(self.ros_domain_id), replay_rate=float(self.replay_rate),
            timeout_sec=float(self.timeout_sec), include_camera=self.include_camera,
            camera_mode=camera_mode, force=bool(self.force), dry_run=bool(self.dry_run),
            keep_partial_on_failure=bool(self.keep_partial_on_failure), log_level=self.log_level.upper(),
        )


def validate_paths(options: BuildOptions) -> BuildOptions:
    options = options.normalized()
    if options.mode not in {"strict", "allow-incomplete"}:
        raise BuildError("mode must be strict or allow-incomplete")
    if options.camera_mode not in {"none", "preview", "full"}:
        raise BuildError("camera-mode must be none, preview or full")
    if not options.run_dir.is_dir():
        raise BuildError(f"run directory does not exist: {options.run_dir}")
    if not RUN_ID_RE.fullmatch(options.run_dir.name):
        raise BuildError(f"invalid run directory name: {options.run_dir.name!r}")
    if not options.config.is_file():
        raise BuildError(f"config does not exist: {options.config}")
    if options.output.suffix != ".rrd":
        raise BuildError("output must have a .rrd suffix")
    output_root = (options.run_dir / "rerun").resolve()
    analysis_root = (options.run_dir.parent.parent / "analysis" / options.run_dir.name).resolve()
    if not (is_relative_to(options.output, output_root) or is_relative_to(options.output, analysis_root)):
        raise BuildError(f"output must be inside {output_root} or {analysis_root}")
    historical_root = Path("/media/mcap_data/runs").resolve()
    if is_relative_to(options.run_dir, historical_root) and is_relative_to(options.output, options.run_dir):
        raise BuildError("historical runs are read-only; choose analysis/<run_id>/<analysis_id>/outputs")
    if options.output == options.run_dir or is_relative_to(options.run_dir, options.output):
        raise BuildError("output may not contain or replace the source run")
    if options.mode == "allow-incomplete" and "_INCOMPLETE" not in options.output.stem:
        raise BuildError("allow-incomplete output must contain _INCOMPLETE")
    if not 0 <= options.ros_domain_id <= 232:
        raise BuildError("ros-domain-id must be in [0, 232]")
    if options.replay_rate <= 0.0:
        raise BuildError("replay-rate must be positive")
    if options.timeout_sec < 0.0:
        raise BuildError("timeout-sec must be non-negative")
    return options


def load_config(path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise BuildError(f"cannot parse config {path}: {exc}") from exc
    try:
        bridge = document["rerun_bridge"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise BuildError("config lacks rerun_bridge.ros__parameters") from exc
    builder = document.get("rrd_builder", {})
    if not isinstance(bridge, dict) or not isinstance(builder, dict):
        raise BuildError("config bridge and builder sections must be mappings")
    return bridge, builder


def metadata_topics(path: Path) -> tuple[dict[str, str], dict[str, int]]:
    try:
        data = yaml.safe_load((path / "metadata.yaml").read_text(encoding="utf-8"))
        info = data["rosbag2_bagfile_information"]
        entries = info["topics_with_message_count"]
    except (OSError, yaml.YAMLError, KeyError, TypeError) as exc:
        raise BuildError(f"cannot read rosbag metadata in {path}: {exc}") from exc
    topics: dict[str, str] = {}
    counts: dict[str, int] = {}
    for entry in entries:
        metadata = entry.get("topic_metadata", {})
        name, type_name = str(metadata.get("name", "")), str(metadata.get("type", ""))
        if not name or not type_name:
            raise BuildError(f"invalid topic metadata in {path}")
        if name in topics and topics[name] != type_name:
            raise BuildError(f"topic {name} has conflicting types in {path}")
        topics[name] = type_name
        counts[name] = int(entry.get("message_count", 0))
    return topics, counts


def _validation_topic_details(group: dict[str, Any]) -> dict[str, Any]:
    return (((group.get("message_validation") or {}).get("topics")) or {})


def discover_bag_sources(
        run_dir: Path, manifest: dict[str, Any], validation: dict[str, Any],
        allow_discovery: bool) -> list[BagSource]:
    sources: list[BagSource] = []
    groups = validation.get("bags") if isinstance(validation.get("bags"), dict) else {}
    for group_name, details in sorted(groups.items()):
        if not isinstance(details, dict):
            continue
        raw_path = details.get("path")
        if not raw_path:
            continue
        path = Path(str(raw_path)).expanduser().resolve()
        mcap_root = (run_dir / "mcap").resolve()
        if not is_relative_to(path, mcap_root):
            raise BuildError(f"validation references a bag outside this run: {path}")
        if path.parent != mcap_root:
            raise BuildError(f"bag group must be a direct child of run/mcap: {path}")
        if path.name != str(group_name):
            raise BuildError(f"bag group identity mismatch: key={group_name}, path={path}")
        topics, metadata_counts = metadata_topics(path)
        report_topics = _validation_topic_details(details)
        counts: dict[str, int] = {}
        ranges: dict[str, tuple[int | None, int | None]] = {}
        for topic, type_name in topics.items():
            report = report_topics.get(topic, {}) if isinstance(report_topics, dict) else {}
            report_type = str(report.get("type", type_name))
            if report_type != type_name:
                raise BuildError(f"metadata/report type mismatch for {topic} in {group_name}")
            counts[topic] = int(report.get("count", metadata_counts.get(topic, 0)))
            first = report.get("first_bag_timestamp_ns")
            last = report.get("last_bag_timestamp_ns")
            ranges[topic] = (
                int(first) if first is not None else None,
                int(last) if last is not None else None,
            )
        starts = [value[0] for value in ranges.values() if value[0] is not None]
        ends = [value[1] for value in ranges.values() if value[1] is not None]
        sources.append(BagSource(
            str(group_name), path, topics, counts, ranges,
            min(starts) if starts else None, max(ends) if ends else None,
        ))
    if sources or not allow_discovery:
        return sources
    mcap_root = (run_dir / "mcap").resolve()
    if not mcap_root.is_dir():
        return []
    for path in sorted(item.resolve() for item in mcap_root.iterdir() if item.is_dir()):
        if path.parent != mcap_root or not (path / "metadata.yaml").is_file():
            continue
        topics, counts = metadata_topics(path)
        sources.append(BagSource(path.name, path, topics, counts))
    return sources


def discover_camera(run_dir: Path) -> list[CameraSegment]:
    camera_root = run_dir / "video" / "front"
    if not camera_root.is_dir():
        return []
    segments: list[CameraSegment] = []
    for video in sorted(camera_root.glob("*.mkv")):
        sidecar = video.with_suffix(".frames.jsonl")
        if not sidecar.is_file():
            raise BuildError(f"camera sidecar is missing for {video}")
        try:
            rows = aligned_rows(video, sidecar)
        except (VideoAlignmentError, OSError, ValueError) as exc:
            raise BuildError(f"camera cannot be strictly aligned: {video}: {exc}") from exc
        count = len(rows)
        first_ns = int(rows[0]["ros_header_stamp_ns"]) if rows else None
        last_ns = int(rows[-1]["ros_header_stamp_ns"]) if rows else None
        if count <= 0 or video.stat().st_size <= 0:
            raise BuildError(f"empty camera segment: {video}")
        segments.append(CameraSegment(video.resolve(), sidecar.resolve(), count, first_ns, last_ns))
    for previous, current in zip(segments, segments[1:]):
        if previous.last_ns is not None and current.first_ns is not None and current.first_ns <= previous.last_ns:
            raise BuildError(f"camera segments overlap or regress: {previous.video}, {current.video}")
    return segments


def active_collection_processes(run_dir: Path) -> list[dict[str, Any]]:
    registry = read_json(run_dir / "logs" / "process_registry.json", required=False)
    active = []
    for item in registry.get("processes", []):
        if not isinstance(item, dict):
            continue
        try:
            pid, expected_ticks = int(item["pid"]), int(item["start_ticks"])
        except (KeyError, TypeError, ValueError):
            continue
        cmdline = process_cmdline(pid)
        if process_start_ticks(pid) == expected_ticks and str(run_dir) in cmdline:
            active.append({"name": item.get("name"), "pid": pid, "cmdline": cmdline})
    return active


def expected_groups(manifest: dict[str, Any]) -> set[str]:
    writers = manifest.get("enabled_writers", {})
    mapping = {
        "core": "core", "mapping": "mapping", "mid360": "mid360",
        "d405": "d405", "eddy": "eddy", "front_raw_burst": "raw_burst",
    }
    return {group for key, group in mapping.items() if bool(writers.get(key, False))}


def group_has_supported_payload(source: BagSource) -> bool:
    return any(type_name in SUPPORTED_TYPES and source.topic_counts.get(topic, 0) > 0
               for topic, type_name in source.topics.items())


def group_required_types(group: str) -> set[str]:
    requirements = {
        "mapping": {"nav_msgs/msg/Odometry", "nav_msgs/msg/Path"},
        "mid360": {"sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Imu"},
        "d405": {"sensor_msgs/msg/Image"},
        "eddy": {"eddy_driver/msg/EddyChannelMean"},
    }
    return requirements.get(group, set())


def load_registered_hashes(manifest: dict[str, Any]) -> dict[str, str]:
    hashes: dict[str, str] = {}
    for item in manifest.get("files", []):
        if not isinstance(item, dict):
            continue
        path, digest = str(item.get("path", "")), str(item.get("sha256", ""))
        if path and re.fullmatch(r"[0-9a-f]{64}", digest):
            hashes[path] = digest
    return hashes


def source_file_list(sources: Iterable[BagSource], camera: Iterable[CameraSegment]) -> list[Path]:
    files: list[Path] = []
    for source in sources:
        files.extend(sorted(source.path.glob("*.mcap")))
        files.append(source.path / "metadata.yaml")
    for segment in camera:
        files.extend([segment.video, segment.sidecar, segment.video.with_suffix(".frame_map.json")])
    return files


def quality_gates(
        options: BuildOptions, manifest: dict[str, Any], complete: dict[str, Any],
        validation: dict[str, Any], sources: list[BagSource], camera: list[CameraSegment],
        builder_config: dict[str, Any], rerun_version: str,
        front_image_in_mcap: bool = False,
        supplemental_reports: dict[str, dict[str, Any]] | None = None,
        ) -> tuple[dict[str, Any], list[str]]:
    results: dict[str, Any] = {}
    bypassed: list[str] = []

    def check(name: str, passed: bool, detail: Any) -> None:
        results[name] = {"passed": bool(passed), "detail": detail}
        if not passed:
            if options.mode == "strict":
                raise BuildError(f"strict quality gate failed: {name}: {detail}")
            results[name]["bypassed"] = True
            bypassed.append(name)

    check("run_id", manifest.get("run_id") == options.run_dir.name,
          {"manifest": manifest.get("run_id"), "directory": options.run_dir.name})
    active = active_collection_processes(options.run_dir)
    check("collection_stopped", not active, active)
    check("complete", complete.get("complete") is True, complete.get("complete"))
    check("finalized", complete.get("finalized") is True, complete.get("finalized"))
    check("complete_validation", complete.get("validation") == "passed",
          complete.get("validation"))
    check("complete_sha256_valid", complete.get("sha256_valid") is True,
          complete.get("sha256_valid"))
    check("manifest_complete", (manifest.get("status") or {}).get("complete") is True,
          (manifest.get("status") or {}).get("complete"))
    check("manifest_finalized", (manifest.get("status") or {}).get("finalized") is True,
          (manifest.get("status") or {}).get("finalized"))
    check("manifest_formal_validation", (manifest.get("status") or {}).get(
        "valid_for_formal_validation") is True,
        (manifest.get("status") or {}).get("valid_for_formal_validation"))
    check("mcap_validation", validation.get("valid") is True, validation.get("valid"))
    validation_groups = validation.get("bags") if isinstance(validation.get("bags"), dict) else {}
    invalid_validation_groups = sorted(
        name for name, details in validation_groups.items()
        if not isinstance(details, dict) or details.get("valid") is not True)
    check("mcap_group_validation", not invalid_validation_groups, invalid_validation_groups)
    for report_name, report in sorted((supplemental_reports or {}).items()):
        if report:
            check(f"report_{report_name}", report.get("valid") is True,
                  report.get("valid"))
    check("sources_present", bool(sources), [str(source.path) for source in sources])
    actual_groups = {source.group for source in sources}
    missing_groups = sorted(expected_groups(manifest) - actual_groups)
    check("enabled_groups_present", not missing_groups, missing_groups)
    invalid_groups = sorted(source.group for source in sources if not group_has_supported_payload(source))
    check("supported_payload_per_group", not invalid_groups, invalid_groups)
    missing_group_types: dict[str, list[str]] = {}
    for source in sources:
        required = group_required_types(source.group)
        actual = {type_name for topic, type_name in source.topics.items()
                  if source.topic_counts.get(topic, 0) > 0}
        if required and not (required & actual):
            missing_group_types[source.group] = sorted(required)
    check("required_group_types", not missing_group_types, missing_group_types)
    clock_sources = [f"{source.group}:{topic}" for source in sources for topic in source.topics if topic == "/clock"]
    check("clock_sources", len(clock_sources) <= 1, clock_sources)
    ranged = [source for source in sources if source.start_ns is not None and source.end_ns is not None]
    overlap = not ranged or max(source.start_ns for source in ranged) <= min(source.end_ns for source in ranged)
    check("multi_bag_time_overlap", overlap,
          {source.group: [source.start_ns, source.end_ns] for source in ranged})
    if options.camera_mode != "none":
        check("camera_source_present", bool(camera) or front_image_in_mcap, {
            "mcap_front_image": front_image_in_mcap,
            "video": [str(item.video) for item in camera],
        })
        if camera and ranged:
            camera_start = min(item.first_ns for item in camera if item.first_ns is not None)
            camera_end = max(item.last_ns for item in camera if item.last_ns is not None)
            bag_start = min(source.start_ns for source in ranged)
            bag_end = max(source.end_ns for source in ranged)
            check("camera_ros_time_alignment", camera_start <= bag_end and camera_end >= bag_start,
                  {"camera": [camera_start, camera_end], "bags": [bag_start, bag_end]})
    frozen = str(builder_config.get("frozen_rerun_sdk_version", FROZEN_RERUN_VERSION))
    check("rerun_sdk_version", rerun_version == frozen,
          {"actual": rerun_version, "frozen": frozen})
    registered_hashes = load_registered_hashes(manifest)
    files = source_file_list(sources, camera if options.camera_mode != "none" else [])
    # Include declared files even if deleted after sealing. A glob must not erase
    # a missing MCAP from the selected dependency set.
    for source in sources:
        prefix = str(source.path.relative_to(options.run_dir)) + "/"
        files.extend(options.run_dir / rel for rel in registered_hashes
                     if rel.startswith(prefix) and (rel.endswith(".mcap") or rel.endswith("metadata.yaml")))
    frozen = options.run_dir / "frozen_configs"
    files.extend(options.run_dir / rel for rel in registered_hashes
                 if rel.startswith("frozen_configs/") and Path(rel).suffix in {".yaml", ".yml", ".json", ".msg"})
    if frozen.is_dir():
        files.extend(path for path in frozen.rglob("*")
                     if path.is_file() and path.suffix in {".yaml", ".yml", ".json", ".msg"})
    actual_hashes = {}
    for path in sorted(set(files)):
        if not is_relative_to(path.resolve(), options.run_dir.resolve()):
            check("source_path", False, str(path))
            continue
        rel = str(path.relative_to(options.run_dir))
        if not path.is_file():
            check("source_files", False, rel)
            continue
        expected = registered_hashes.get(rel)
        actual = sha256_file(path)
        actual_hashes[rel] = actual
        check("source_hash_" + rel, expected == actual,
              {"path": rel, "expected": expected, "actual": actual})
    check("source_files_present", bool(actual_hashes), sorted(actual_hashes))
    results["source_hashes_verified"] = {"passed": not any(
        name.startswith("source_") for name in bypassed), "detail": actual_hashes}
    source_bytes = sum(path.stat().st_size for path in set(files) if path.is_file())
    usage = shutil.disk_usage(options.output.parent if options.output.parent.exists() else options.run_dir)
    minimum = max(
        int(builder_config.get("min_free_bytes", 1024 ** 3)),
        int(source_bytes * float(builder_config.get("source_size_headroom_ratio", 0.25))),
    )
    check("output_disk_space", usage.free >= minimum,
          {"free_bytes": usage.free, "required_bytes": minimum, "source_bytes": source_bytes})
    return results, bypassed


class ReaderStream:
    """One filtered rosbag2 reader with a single buffered head item."""

    def __init__(self, source: BagSource, selected_topics: set[str]):
        import rosbag2_py

        self.source = source
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=str(source.path), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        self.reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(selected_topics)))
        self.next_item: tuple[int, str, bytes, str] | None = None
        self.advance()

    def advance(self) -> None:
        self.next_item = None
        while self.reader.has_next():
            topic, payload, timestamp = self.reader.read_next()
            type_name = self.source.topics.get(topic)
            if type_name:
                self.next_item = (int(timestamp), topic, payload, type_name)
                return


class OfflineEntityLogger:
    def __init__(
            self, session: RerunSession, bridge_config: dict[str, Any],
            builder_config: dict[str, Any], statistics: dict[str, TopicStatistics],
            camera_mode: str = "none"):
        import rerun as rr

        self.rr = rr
        self.session = session
        self.bridge_config = bridge_config
        self.builder_config = builder_config
        self.statistics = statistics
        self.camera_mode = camera_mode
        self.timeline = TimelineTracker()
        self.limiters: dict[str, RateLimiter] = {}
        self.entities: set[str] = set()

    def _limiter(self, topic: str, type_name: str) -> RateLimiter | None:
        rates = {
            "sensor_msgs/msg/Image": float(self.bridge_config.get("image_max_hz", 2.0)),
            "sensor_msgs/msg/PointCloud2": float(self.bridge_config.get("pointcloud_max_hz", 1.0)),
            "sensor_msgs/msg/Imu": float(self.bridge_config.get("signal_max_hz", 2.0)),
            "eddy_driver/msg/EddyChannelMean": float(self.bridge_config.get("signal_max_hz", 2.0)),
        }
        if (type_name == "sensor_msgs/msg/Image"
                and topic == str(self.bridge_config.get("image_topic", "/camera/front/image_raw"))
                and self.camera_mode != "none"):
            rates[type_name] = (
                float(self.builder_config.get("camera_preview_max_hz", 1.0))
                if self.camera_mode == "preview" else 0.0)
        if type_name not in rates:
            return None
        return self.limiters.setdefault(topic, RateLimiter(rates[type_name]))

    def _log(self, entity: str, archetype: Any, *, static: bool = False) -> None:
        self.session.log(entity, archetype, static=static)
        self.entities.add(entity)

    def handle(self, topic: str, type_name: str, message: Any, bag_timestamp_ns: int) -> None:
        stats = self.statistics[topic]
        stats.received_count += 1
        timestamp_ns, fallback = message_timestamp_ns(message, bag_timestamp_ns)
        if fallback:
            stats.timestamp_fallback_count += 1
        sequence, regressed = self.timeline.observe(topic, timestamp_ns)
        if regressed:
            stats.timestamp_regression_count += 1
        limiter = self._limiter(topic, type_name)
        if limiter is not None and not limiter.allow(timestamp_ns):
            stats.rate_limited_count += 1
            return
        try:
            self.session.set_time(timestamp_ns, sequence)
            self._dispatch(topic, type_name, message, stats)
        except Exception:
            stats.parse_failure_count += 1
            raise
        stats.logged_count += 1
        stats.observe_logged_time(timestamp_ns)

    def _dispatch(self, topic: str, type_name: str, message: Any, stats: TopicStatistics) -> None:
        entity = sanitize_entity(topic)
        rr = self.rr
        if type_name == "sensor_msgs/msg/Image":
            pixels, encoding = image_to_numpy(message)
            original_height, original_width = pixels.shape[:2]
            max_width = int(self.builder_config.get("camera_preview_max_width", 1280))
            front_topic = str(self.bridge_config.get("image_topic", "/camera/front/image_raw"))
            if topic == front_topic and max_width > 0 and original_width > max_width:
                import cv2
                display_height = max(1, round(original_height * max_width / original_width))
                pixels = cv2.resize(pixels, (max_width, display_height), interpolation=cv2.INTER_AREA)
            if encoding == "16uc1":
                units_per_meter = float(
                    self.builder_config.get("depth_units_per_meter", 1000.0))
                if units_per_meter <= 0:
                    raise ValueError("depth_units_per_meter must be positive")
                self._log(entity, rr.DepthImage(
                    pixels, meter=units_per_meter, colormap="viridis"))
                display_encoding = f"depth:{units_per_meter:g}_units_per_meter"
            elif topic == front_topic and self.camera_mode == "preview":
                import cv2
                jpeg_input = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR) \
                    if pixels.ndim == 3 and pixels.shape[2] >= 3 else pixels
                quality = int(self.builder_config.get("camera_preview_jpeg_quality", 85))
                encoded, payload = cv2.imencode(
                    ".jpg", jpeg_input, [cv2.IMWRITE_JPEG_QUALITY, quality])
                if not encoded:
                    raise ValueError("failed to JPEG-encode front camera preview")
                self._log(entity, rr.EncodedImage(
                    contents=payload.tobytes(), media_type="image/jpeg"))
                display_encoding = f"jpeg:q{quality}"
            else:
                self._log(entity, rr.Image(pixels))
                display_encoding = encoding
            self._log(entity + "/metadata", rr.AnyValues(
                source_encoding=encoding, original_width=original_width,
                original_height=original_height, display_width=int(pixels.shape[1]),
                display_height=int(pixels.shape[0]), display_encoding=display_encoding))
        elif type_name == "sensor_msgs/msg/CameraInfo":
            matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
            if np.any(matrix):
                self._log(entity, rr.Pinhole(
                    image_from_camera=matrix,
                    resolution=[int(message.width), int(message.height)]))
            else:
                self._log(entity + "/metadata", rr.TextLog("CameraInfo intrinsics are all zero", level="WARN"))
        elif type_name == "sensor_msgs/msg/PointCloud2":
            points, intensity, point_stats = pointcloud_to_numpy(
                message,
                int(self.bridge_config.get("point_stride", 1)),
                min(100000, int(self.bridge_config.get("max_point_count", 100000))),
                float(self.builder_config.get("voxel_size", 0.0)),
                return_stats=True,
            )
            stats.invalid_point_count += point_stats["invalid_point_count"]
            stats.capped_point_count += point_stats["capped_point_count"]
            colors = None
            if intensity is not None and intensity.size:
                low, high = float(np.min(intensity)), float(np.max(intensity))
                scaled = np.zeros_like(intensity, dtype=np.uint8) if high <= low else np.clip(
                    (intensity - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
                colors = np.column_stack([scaled, scaled, scaled])
            self._log(entity, rr.Points3D(points, colors=colors))
            self._log(entity + "/metadata", rr.AnyValues(
                frame_id=str(message.header.frame_id), displayed_points=len(points),
                invalid_points=point_stats["invalid_point_count"],
                capped_points=point_stats["capped_point_count"]))
        elif type_name == "sensor_msgs/msg/Imu":
            acceleration, angular, orientation = imu_values(message)
            for axis, value in zip("xyz", acceleration):
                self._log(f"{entity}/acceleration/{axis}", rr.Scalars([value]))
            for axis, value in zip("xyz", angular):
                self._log(f"{entity}/angular_velocity/{axis}", rr.Scalars([value]))
            if orientation is not None:
                for axis, value in zip("xyzw", orientation):
                    self._log(f"{entity}/orientation/{axis}", rr.Scalars([value]))
        elif type_name == "tf2_msgs/msg/TFMessage":
            for transform in message.transforms:
                parent, child, xyz, xyzw = transform_values(transform)
                path = f"/world/frames/{re.sub(r'[^A-Za-z0-9_.-]+', '_', child)}"
                self._log(path, rr.Transform3D(
                    translation=xyz, quaternion=rr.Quaternion(xyzw=xyzw),
                    parent_frame=parent, child_frame=child), static=(topic == "/tf_static"))
        elif type_name == "diagnostic_msgs/msg/DiagnosticArray":
            for index, status in enumerate(message.status):
                numeric = diagnostic_level(status.level)
                level = "ERROR" if numeric >= 2 else "WARN" if numeric == 1 else "INFO"
                name = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(status.name)) or str(index)
                self._log(f"{entity}/{name}", rr.TextLog(
                    f"{status.message}; hardware={status.hardware_id}", level=level))
        elif type_name in {"nav_msgs/msg/Odometry", "geometry_msgs/msg/PoseStamped"}:
            pose = message.pose.pose if type_name == "nav_msgs/msg/Odometry" else message.pose
            values = [float(pose.orientation.x), float(pose.orientation.y),
                      float(pose.orientation.z), float(pose.orientation.w)]
            norm = math.sqrt(sum(value * value for value in values))
            if norm <= 1.0e-12 or not math.isfinite(norm):
                raise ValueError("pose has invalid quaternion")
            self._log(entity, rr.Transform3D(
                translation=[pose.position.x, pose.position.y, pose.position.z],
                quaternion=rr.Quaternion(xyzw=[value / norm for value in values])))
        elif type_name == "nav_msgs/msg/Path":
            points = [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
                      for pose in message.poses]
            if points:
                self._log(entity, rr.LineStrips3D([points]))
        elif type_name == "std_msgs/msg/String":
            self._log(entity, rr.TextLog(str(message.data)))
        elif type_name == "eddy_driver/msg/EddyChannelMean":
            values = [float(value) for value in message.channel_mean]
            for index, value in enumerate(values):
                self._log(f"{entity}/channel_{index}", rr.Scalars([value]))
        elif type_name == "eddy_driver/msg/EddyStatus":
            self._log(entity, rr.AnyValues(
                connected=bool(message.connected), received_frames=int(message.received_frames),
                dropped_frames=int(message.dropped_frames), queue_depth=int(message.queue_depth),
                last_status_is_error=bool(message.last_status_is_error)))
        else:
            raise ValueError(f"no offline adapter for {type_name}")


def select_camera_rows(segment: CameraSegment, mode: str, max_hz: float) -> tuple[list[dict[str, Any]], int]:
    selected: list[dict[str, Any]] = []
    limiter = RateLimiter(max_hz if mode == "preview" else 0.0)
    rate_limited = 0
    for row in aligned_rows(segment.video, segment.sidecar):
        stamp = int(row["ros_header_stamp_ns"])
        if limiter.allow(stamp): selected.append(row)
        else: rate_limited += 1
    return selected, rate_limited


def log_camera_segments(
        camera: list[CameraSegment], mode: str, builder_config: dict[str, Any],
        entity_logger: OfflineEntityLogger, statistics: dict[str, TopicStatistics],
        check_abort: Any) -> None:
    import cv2

    cv2.setNumThreads(2)

    topic = "/camera/front/image_raw"
    stats = statistics.setdefault(topic, TopicStatistics(type="video/h264+frames.jsonl"))
    max_hz = float(builder_config.get("camera_preview_max_hz", 1.0))
    max_width = int(builder_config.get("camera_preview_max_width", 1280))
    for segment in camera:
        rows, limited = select_camera_rows(segment, mode, max_hz)
        rows.sort(key=lambda row: int(row["frame_index"]))
        stats.source_count += segment.frame_count
        stats.rate_limited_count += limited
        stats.observe_source_time(segment.first_ns)
        stats.observe_source_time(segment.last_ns)
        logging.info(
            "camera preview start: segment=%s, selected=%d/%d, decode=sequential",
            segment.video.name, len(rows), segment.frame_count)
        capture = cv2.VideoCapture(
            str(segment.video), cv2.CAP_FFMPEG,
            [cv2.CAP_PROP_N_THREADS, 2],
        )
        if not capture.isOpened():
            stats.camera_decode_failure_count += len(rows)
            logging.error("cannot open camera segment %s", segment.video)
            continue
        try:
            row_index = 0
            decoded_frame_index = 0
            last_progress = time.monotonic()
            while row_index < len(rows):
                check_abort()
                row = rows[row_index]
                frame_index = int(row["frame_index"])
                if frame_index < decoded_frame_index:
                    stats.received_count += 1
                    stats.camera_decode_failure_count += 1
                    logging.error(
                        "camera sidecar frame order invalid: %s frame=%d decoded=%d",
                        segment.video, frame_index, decoded_frame_index)
                    row_index += 1
                    continue
                ok, frame = capture.read()
                if not ok or frame is None:
                    remaining = len(rows) - row_index
                    stats.received_count += remaining
                    stats.camera_decode_failure_count += remaining
                    logging.error(
                        "camera sequential decode ended early: %s decoded_frame=%d "
                        "next_selected=%d remaining_selected=%d",
                        segment.video, decoded_frame_index, frame_index, remaining)
                    break
                if decoded_frame_index < frame_index:
                    decoded_frame_index += 1
                    now = time.monotonic()
                    if now - last_progress >= 10.0:
                        logging.info(
                            "camera preview progress: segment=%s, selected=%d/%d, "
                            "decoded=%d/%d",
                            segment.video.name, row_index, len(rows),
                            decoded_frame_index, segment.frame_count)
                        last_progress = now
                    continue
                stats.received_count += 1
                original_height, original_width = frame.shape[:2]
                if max_width > 0 and original_width > max_width:
                    display_height = max(1, round(original_height * max_width / original_width))
                    frame = cv2.resize(frame, (max_width, display_height), interpolation=cv2.INTER_AREA)
                timestamp_ns = int(row["ros_header_stamp_ns"])
                sequence, regressed = entity_logger.timeline.observe(topic, timestamp_ns)
                if regressed:
                    stats.timestamp_regression_count += 1
                entity_logger.session.set_time(timestamp_ns, sequence)
                if mode == "preview":
                    quality = int(builder_config.get("camera_preview_jpeg_quality", 85))
                    encoded, payload = cv2.imencode(
                        ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                    if not encoded:
                        stats.camera_decode_failure_count += 1
                        logging.error("camera JPEG encode failed: %s frame=%d", segment.video, frame_index)
                        continue
                    entity_logger._log("/camera/front/image", entity_logger.rr.EncodedImage(
                        contents=payload.tobytes(), media_type="image/jpeg"))
                    display_encoding = f"jpeg:q{quality}"
                    display_height, display_width = frame.shape[:2]
                else:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    entity_logger._log("/camera/front/image", entity_logger.rr.Image(rgb))
                    display_encoding = "rgb8"
                    display_height, display_width = rgb.shape[:2]
                entity_logger._log("/camera/front/metadata", entity_logger.rr.AnyValues(
                    source_video=str(segment.video), frame_index=frame_index,
                    original_width=original_width, original_height=original_height,
                    display_width=int(display_width), display_height=int(display_height),
                    display_encoding=display_encoding, source_sha_registered=True))
                stats.logged_count += 1
                stats.observe_logged_time(timestamp_ns)
                row_index += 1
                decoded_frame_index += 1
                now = time.monotonic()
                if row_index == len(rows) or row_index % 25 == 0 or now - last_progress >= 10.0:
                    logging.info(
                        "camera preview progress: segment=%s, selected=%d/%d, "
                        "decoded=%d/%d",
                        segment.video.name, row_index, len(rows),
                        decoded_frame_index, segment.frame_count)
                    last_progress = now
        finally:
            capture.release()
        logging.info(
            "camera preview complete: segment=%s, selected=%d, cumulative_logged=%d, "
            "cumulative_decode_failures=%d",
            segment.video.name, len(rows), stats.logged_count,
            stats.camera_decode_failure_count)


def validate_message_types(sources: list[BagSource]) -> dict[str, Any]:
    from rosidl_runtime_py.utilities import get_message

    classes: dict[str, Any] = {}
    failures: dict[str, str] = {}
    for type_name in sorted({value for source in sources for value in source.topics.values()} & SUPPORTED_TYPES):
        try:
            classes[type_name] = get_message(type_name)
        except Exception as exc:
            failures[type_name] = f"{type(exc).__name__}: {exc}"
    if failures:
        raise BuildError(f"required message types cannot be loaded: {failures}")
    return classes


def rerun_environment() -> tuple[str, Path]:
    try:
        import rerun as rr
    except ImportError as exc:
        raise BuildError("rerun-sdk is unavailable in this Python environment") from exc
    cli = Path(sys.executable).absolute().parent / "rerun"
    if not cli.is_file():
        candidate = shutil.which("rerun")
        if candidate:
            cli = Path(candidate).resolve()
    if not cli.is_file():
        raise BuildError("matching Rerun CLI was not found beside the Python interpreter")
    result = subprocess.run([str(cli), "--version"], text=True, capture_output=True, check=False)
    if result.returncode != 0 or f"rerun-cli {rr.__version__}" not in result.stdout:
        raise BuildError(f"Rerun CLI/SDK mismatch: sdk={rr.__version__}, cli={result.stdout.strip()}")
    return str(rr.__version__), cli


def verify_rrd(cli: Path, partial: Path) -> dict[str, Any]:
    environment = dict(os.environ)
    environment["RERUN_ANALYTICS_ENABLED"] = "false"
    verify = subprocess.run(
        [str(cli), "rrd", "verify", "--check-footers", "true", str(partial)],
        text=True, capture_output=True, check=False, env=environment,
    )
    if verify.returncode != 0:
        raise BuildError(f"Rerun CLI failed to verify partial: {verify.stderr.strip()}")
    stats = subprocess.run(
        [str(cli), "rrd", "stats", "--no-decode", str(partial)],
        text=True, capture_output=True, check=False, env=environment,
    )
    if stats.returncode != 0:
        raise BuildError(f"Rerun CLI failed to inspect partial: {stats.stderr.strip()}")
    return {
        "verify_returncode": verify.returncode,
        "verify_stdout": verify.stdout.strip(),
        "verify_stderr": verify.stderr.strip(),
        "stats_returncode": stats.returncode,
        "stats_stdout": stats.stdout.strip(),
    }


def parse_arguments(argv: list[str] | None = None) -> BuildOptions:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--mode", choices=("strict", "allow-incomplete"), default="strict")
    parser.add_argument("--ros-domain-id", type=int, default=184)
    parser.add_argument("--replay-rate", type=float, default=1.0)
    parser.add_argument("--timeout-sec", type=float, default=0.0)
    parser.add_argument("--include-camera", action="store_true")
    parser.add_argument("--camera-mode", choices=("none", "preview", "full"), default="none")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--keep-partial-on-failure", action="store_true")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    return BuildOptions(**vars(parser.parse_args(argv)))


class RunBuilder:
    def __init__(self, options: BuildOptions):
        self.options = validate_paths(options)
        self.partial = self.options.output.with_suffix(self.options.output.suffix + ".partial")
        self.output_dir = self.options.output.parent
        self.manifest_path = self.output_dir / "rrd_manifest.json"
        self.validation_path = self.output_dir / "rrd_validation.json"
        self.log_path = self.output_dir / "conversion.log"
        self.lock_path = self.output_dir / ".build_run_rrd.lock"
        self.stop_event = threading.Event()
        self.started_monotonic = time.monotonic()
        self.started_cpu = time.process_time()
        self.baseline_rss = current_rss_bytes()
        self.peak_rss = self.baseline_rss
        self.statistics: dict[str, TopicStatistics] = {}
        self.committed = False
        self._old_signal_handlers: dict[int, Any] = {}

    def write_manifest(self, document: dict[str, Any]) -> None:
        # Existing RRD manifest remains the public contract. The standard
        # derived-result record is generated from the same current state.
        atomic_json(self.manifest_path, document)
        result = {
            "schema_version": "tank_derived_result_v1",
            "analysis_type": "rrd", "source_run_id": self.options.run_dir.name,
            "source_sha256": document.get("source_sha256", {}),
            "hash_verified": document.get("hash_verified", False),
            "config_sha256": {str(self.options.config): document.get("config_sha256")},
            "code_status": document.get("code_status", {"status": "unknown"}),
            "time_range_ns": [document.get("start_ros_time_ns"), document.get("end_ros_time_ns")],
            "coordinate_frames": document.get("coordinate_frames", "unknown"),
            "calibration_status": document.get("calibration_status", "unverified"),
            "complete": document.get("complete", False),
            "valid_for_formal_validation": document.get("valid_for_formal_validation", False),
            "failure": document.get("failure"),
            "outputs": [{"path": str(self.options.output), "sha256": document.get("rrd_sha256")}],
            "transport_acceptance": {"status": "not_applicable", "reason": "direct reader, no DDS subscriber"},
            "algorithm_acceptance": {"status": "not_executed"},
        }
        atomic_json(self.output_dir / "result_manifest.json", result)

    def check_abort(self) -> None:
        if self.stop_event.is_set():
            raise BuildError("conversion interrupted by SIGINT/SIGTERM")
        if self.options.timeout_sec and time.monotonic() - self.started_monotonic > self.options.timeout_sec:
            raise BuildError(f"conversion exceeded timeout-sec={self.options.timeout_sec}")

    def check_memory(self, limit: int) -> None:
        current = current_rss_bytes()
        self.peak_rss = max(self.peak_rss, current)
        if limit > 0 and current - self.baseline_rss > limit:
            raise BuildError(
                f"RSS growth exceeded limit: growth={current - self.baseline_rss}, limit={limit}")

    def _signal_handler(self, signum: int, _frame: Any) -> None:
        logging.warning("received signal %s; stopping after current operation", signum)
        self.stop_event.set()

    def _install_signal_handlers(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            return
        for signum in (signal.SIGINT, signal.SIGTERM):
            self._old_signal_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, self._signal_handler)

    def _restore_signal_handlers(self) -> None:
        for signum, handler in self._old_signal_handlers.items():
            signal.signal(signum, handler)

    def _acquire_lock(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        try:
            descriptor = os.open(self.lock_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o664)
        except FileExistsError as exc:
            raise BuildError(f"another run builder may be active: {self.lock_path}") from exc
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(f"pid={os.getpid()}\nstarted={utc_now()}\n")

    def _release_lock(self) -> None:
        try:
            self.lock_path.unlink()
        except FileNotFoundError:
            pass

    def _setup_logging(self) -> None:
        root = logging.getLogger()
        root.handlers.clear()
        root.setLevel(getattr(logging, self.options.log_level))
        formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
        for handler in (logging.StreamHandler(), logging.FileHandler(self.log_path, encoding="utf-8")):
            handler.setFormatter(formatter)
            root.addHandler(handler)

    def _initial_manifest(
            self, source_manifest: dict[str, Any], sources: list[BagSource],
            camera: list[CameraSegment], config_hash: str, rerun_version: str,
            gates: dict[str, Any], bypassed: list[str], registered_hashes: dict[str, str],
            front_image_in_mcap: bool = False) -> dict[str, Any]:
        files = source_file_list(sources, camera if self.options.camera_mode != "none" else [])
        source_hashes = {
            str(path.relative_to(self.options.run_dir)): registered_hashes.get(
                str(path.relative_to(self.options.run_dir))) for path in files
        }
        starts = [source.start_ns for source in sources if source.start_ns is not None]
        ends = [source.end_ns for source in sources if source.end_ns is not None]
        if self.options.camera_mode != "none":
            starts.extend(item.first_ns for item in camera if item.first_ns is not None)
            ends.extend(item.last_ns for item in camera if item.last_ns is not None)
        return {
            "schema_version": "inspection_rerun_run_rrd_v1",
            "created_at": utc_now(),
            "run_id": self.options.run_dir.name,
            "source_run_dir": str(self.options.run_dir),
            "source_run_id": self.options.run_dir.name,
            "code_status": {"installed_builder_sha256": sha256_file(Path(__file__)),
                            "repository_head": "unknown_for_installed_module", "dirty": "unknown"},
            "coordinate_frames": "message frame_id; no spatial extrinsic claim",
            "calibration_status": "unverified",
            "output_path": str(self.options.output),
            "source_bags": [
                {"group": source.group, "path": str(source.path), "start_ns": source.start_ns,
                 "end_ns": source.end_ns, "topics": source.topics}
                for source in sources
            ],
            "source_camera": (
                None if self.options.camera_mode == "none" else
                {"kind": "mcap", "topic": "/camera/front/image_raw"}
                if front_image_in_mcap else
                {"kind": "mkv_sidecar", "segments": [
                    {"video": str(item.video), "sidecar": str(item.sidecar),
                     "frame_count": item.frame_count, "first_ns": item.first_ns,
                     "last_ns": item.last_ns}
                    for item in camera
                ]}
            ),
            "source_sha256": dict(registered_hashes),
            "rerun_sdk_version": rerun_version,
            "rerun_rrd_version": "rerun-sdk/CLI 0.36.2 footer-bearing RRD",
            "pc_viewer_version": rerun_version,
            "python_version": platform.python_version(),
            "ros_distro": os.environ.get("ROS_DISTRO", source_manifest.get("ros_distro", "unknown")),
            "machine_arch": platform.machine(),
            "config_path": str(self.options.config),
            "config_sha256": config_hash,
            "start_ros_time_ns": min(starts) if starts else 0,
            "end_ros_time_ns": max(ends) if ends else 0,
            "topic_statistics": {},
            "parse_failures": {},
            "rate_limits": {},
            "pointcloud_filter": {},
            "camera_policy": {
                "mode": self.options.camera_mode,
                "include_camera": self.options.camera_mode != "none",
            },
            "quality_gate_mode": self.options.mode,
            "quality_gate_results": gates,
            "bypassed_quality_gates": bypassed,
            "valid_for_formal_validation": False,
            "hash_verified": False,
            "complete": False,
            "hash_verified": False,
            "dry_run": self.options.dry_run,
            "ros_domain_id": self.options.ros_domain_id,
            "scheduler": {
                "kind": "direct_rosbag2_heap_merge",
                "dds_used": False,
                "clock_publishers_started": 0,
                "subscriber_drop_possible": False,
                "replay_rate_requested": self.options.replay_rate,
                "replay_rate_note": "recorded for compatibility; direct streaming is not wall-clock paced",
            },
        }

    def run(self) -> int:
        self._acquire_lock()
        self._setup_logging()
        self._install_signal_handlers()
        session: RerunSession | None = None
        manifest_document: dict[str, Any] = {
            "run_id": self.options.run_dir.name,
            "source_run_dir": str(self.options.run_dir),
            "source_run_id": self.options.run_dir.name,
            "code_status": {"installed_builder_sha256": sha256_file(Path(__file__)),
                            "repository_head": "unknown_for_installed_module", "dirty": "unknown"},
            "coordinate_frames": "message frame_id; no spatial extrinsic claim",
            "calibration_status": "unverified",
            "output_path": str(self.options.output),
            "quality_gate_mode": self.options.mode,
            "valid_for_formal_validation": False,
            "hash_verified": False,
            "complete": False,
        }
        validation_document: dict[str, Any] = {
            "schema_version": "inspection_rerun_rrd_validation_v1",
            "created_at": utc_now(), "success": False, "errors": [],
        }
        try:
            if self.options.output.exists() and not self.options.force:
                raise BuildError(f"refusing to overwrite existing RRD: {self.options.output}")
            if self.partial.exists() and not self.options.force:
                raise BuildError(f"refusing to overwrite existing partial: {self.partial}")
            if self.options.force and self.partial.exists():
                self.partial.unlink()
            bridge_config, builder_config = load_config(self.options.config)
            strict = self.options.mode == "strict"
            source_manifest = read_json(
                self.options.run_dir / "run_manifest.json", required=strict)
            complete = read_json(self.options.run_dir / "complete.json", required=strict)
            validation = read_json(self.options.run_dir / "reports" / "mcap_validation.json",
                                   required=strict)
            supplemental_reports = {
                "system_acceptance": read_json(
                    self.options.run_dir / "reports" / "system_acceptance.json",
                    required=False),
                "video_validation": read_json(
                    self.options.run_dir / "reports" / "video_validation.json",
                    required=False) if self.options.camera_mode != "none" else {},
            }
            sources = discover_bag_sources(
                self.options.run_dir, source_manifest, validation,
                allow_discovery=self.options.mode == "allow-incomplete")
            camera = discover_camera(self.options.run_dir) if self.options.camera_mode != "none" else []
            front_topic = str(bridge_config.get("image_topic", "/camera/front/image_raw"))
            front_image_in_mcap = any(
                source.topics.get(front_topic) == "sensor_msgs/msg/Image"
                and source.topic_counts.get(front_topic, 0) > 0 for source in sources)
            camera_to_log = [] if front_image_in_mcap else camera
            if front_image_in_mcap and camera:
                logging.info("front camera exists in MCAP and MKV; selecting MCAP to avoid duplicate imagery")
            rerun_version, rerun_cli = rerun_environment()
            message_classes = validate_message_types(sources)
            gates, bypassed = quality_gates(
                self.options, source_manifest, complete, validation, sources,
                camera_to_log, builder_config, rerun_version, front_image_in_mcap,
                supplemental_reports)
            registered_hashes = gates["source_hashes_verified"]["detail"]
            manifest_document = self._initial_manifest(
                source_manifest, sources, camera_to_log, sha256_file(self.options.config),
                rerun_version, gates, bypassed, registered_hashes, front_image_in_mcap)
            manifest_document["hash_verified"] = gates["source_hashes_verified"]["passed"]
            manifest_document["source_verification"] = gates["source_hashes_verified"]
            manifest_document["control_file_sha256"] = {
                str(path.relative_to(self.options.run_dir)): sha256_file(path)
                for path in [self.options.run_dir / "run_manifest.json", self.options.run_dir / "complete.json",
                             self.options.run_dir / "reports/mcap_validation.json",
                             self.options.run_dir / "reports/system_acceptance.json",
                             self.options.run_dir / "reports/video_validation.json"] if path.is_file()}
            self.write_manifest(manifest_document)
            if self.options.dry_run:
                validation_document.update({
                    "success": True, "dry_run": True, "quality_gate_results": gates,
                    "hash_verified": manifest_document["hash_verified"],
                    "note": "preflight only; no RRD was written",
                })
                atomic_json(self.validation_path, validation_document)
                logging.info("dry-run preflight succeeded for %s", self.options.run_dir)
                return 0

            for source in sources:
                for topic, type_name in source.topics.items():
                    stats = self.statistics.setdefault(topic, TopicStatistics(type=type_name))
                    if stats.type != type_name:
                        raise BuildError(f"topic type conflict across bags: {topic}: {stats.type} vs {type_name}")
                    stats.source_count += int(source.topic_counts.get(topic, 0))
                    first, last = source.topic_ranges.get(topic, (None, None))
                    stats.observe_source_time(first)
                    stats.observe_source_time(last)
            session = RerunSession("save", self.options.run_dir.name, str(self.partial))
            entity_logger = OfflineEntityLogger(
                session, bridge_config, builder_config, self.statistics,
                camera_mode=self.options.camera_mode)
            entity_logger.session.set_time(manifest_document["start_ros_time_ns"], 0)
            entity_logger._log("/run/metadata", entity_logger.rr.AnyValues(
                run_id=self.options.run_dir.name, source_run_dir=str(self.options.run_dir),
                quality_gate_mode=self.options.mode,
                valid_for_formal_validation=(self.options.mode == "strict")))
            if self.options.mode == "allow-incomplete":
                entity_logger._log("/run/INCOMPLETE_WARNING", entity_logger.rr.TextLog(
                    "INCOMPLETE DIAGNOSTIC DERIVATIVE: NOT VALID FOR FORMAL VALIDATION; "
                    f"bypassed gates={','.join(bypassed)}", level="WARN"), static=True)

            selected_by_source: list[tuple[BagSource, set[str]]] = []
            for source in sources:
                selected = {topic for topic, type_name in source.topics.items()
                            if type_name in SUPPORTED_TYPES and source.topic_counts.get(topic, 0) > 0}
                selected_by_source.append((source, selected))
            selected_message_count = sum(
                int(source.topic_counts.get(topic, 0))
                for source, selected in selected_by_source for topic in selected)
            streams = [ReaderStream(source, selected) for source, selected in selected_by_source if selected]
            heap: list[tuple[int, int]] = []
            for index, stream in enumerate(streams):
                if stream.next_item is not None:
                    heapq.heappush(heap, (stream.next_item[0], index))
            processed = 0
            memory_interval = max(1, int(builder_config.get("memory_check_interval_messages", 10000)))
            memory_limit = int(builder_config.get("max_rss_growth_bytes", 1024 ** 3))
            failure_log_counts: dict[tuple[str, str, str, str], int] = {}

            def log_message_failure(prefix: str, topic: str, type_name: str, exc: Exception) -> None:
                key = (topic, type_name, type(exc).__name__, str(exc))
                count = failure_log_counts.get(key, 0) + 1
                failure_log_counts[key] = count
                # Preserve exact counters while preventing a bad high-rate
                # topic from flooding the terminal and conversion.log.
                if count <= 3 or count % 100 == 0:
                    logging.error(
                        "%s topic=%s type=%s: %s: %s (occurrence=%d%s)",
                        prefix, topic, type_name, type(exc).__name__, exc, count,
                        "; repeated messages suppressed" if count == 3 else "")

            from rclpy.serialization import deserialize_message
            while heap:
                self.check_abort()
                _, index = heapq.heappop(heap)
                stream = streams[index]
                item = stream.next_item
                if item is None:
                    continue
                stream.advance()
                if stream.next_item is not None:
                    heapq.heappush(heap, (stream.next_item[0], index))
                timestamp, topic, payload, type_name = item
                stats = self.statistics[topic]
                try:
                    message = deserialize_message(payload, message_classes[type_name])
                except Exception as exc:
                    stats.received_count += 1
                    stats.parse_failure_count += 1
                    log_message_failure(
                        "skipping undecodable message", topic, type_name, exc)
                    processed += 1
                    continue
                try:
                    entity_logger.handle(topic, type_name, message, timestamp)
                except Exception as exc:
                    log_message_failure("skipping bad message", topic, type_name, exc)
                processed += 1
                if processed % memory_interval == 0:
                    self.check_memory(memory_limit)
                    percent = 100.0 * processed / max(1, selected_message_count)
                    partial_bytes = self.partial.stat().st_size if self.partial.exists() else 0
                    logging.info(
                        "conversion progress: processed=%d/%d (%.1f%%), elapsed=%.1fs, "
                        "rss=%d MiB, partial=%d MiB",
                        processed, selected_message_count, min(percent, 100.0),
                        time.monotonic() - self.started_monotonic,
                        current_rss_bytes() // (1024 * 1024),
                        partial_bytes // (1024 * 1024))

            logging.info(
                "MCAP conversion complete: processed=%d/%d; starting camera stage=%s",
                processed, selected_message_count,
                "enabled" if self.options.camera_mode != "none" and camera_to_log else "none")
            if self.options.camera_mode != "none":
                if camera_to_log:
                    log_camera_segments(
                        camera_to_log, self.options.camera_mode, builder_config,
                        entity_logger, self.statistics, self.check_abort)
            self.check_memory(memory_limit)
            logging.info("camera stage complete; closing Rerun writer")
            session.close()
            session = None
            if not self.partial.is_file() or self.partial.stat().st_size <= 0:
                raise BuildError("RRD partial is missing or empty after writer close")
            logging.info(
                "Rerun writer closed; validating RRD footer and entities: %s (%d MiB)",
                self.partial, self.partial.stat().st_size // (1024 * 1024))
            cli_validation = verify_rrd(rerun_cli, self.partial)

            required_failures = {
                topic: asdict(stats) for topic, stats in self.statistics.items()
                if stats.source_count > 0 and stats.type in SUPPORTED_TYPES and stats.logged_count <= 0
                and stats.type not in {"sensor_msgs/msg/CameraInfo", "tf2_msgs/msg/TFMessage"}
            }
            if required_failures:
                raise BuildError(f"supported source topics produced no Rerun rows: {list(required_failures)}")
            delivery_mismatches = {
                topic: {"source_count": stats.source_count, "received_count": stats.received_count}
                for topic, stats in self.statistics.items()
                if stats.type in SUPPORTED_TYPES and stats.source_count != stats.received_count
            }
            camera_stats = self.statistics.get("/camera/front/image_raw")
            if (camera_stats is not None and camera_stats.type == "video/h264+frames.jsonl"
                    and camera_stats.source_count != (
                        camera_stats.received_count + camera_stats.rate_limited_count)):
                delivery_mismatches["/camera/front/image_raw"] = {
                    "source_count": camera_stats.source_count,
                    "selected_or_rate_limited": (
                        camera_stats.received_count + camera_stats.rate_limited_count),
                }
            if delivery_mismatches:
                raise BuildError(f"unexplained source/reader count mismatch: {delivery_mismatches}")
            parse_failure_count = sum(item.parse_failure_count for item in self.statistics.values())
            received_count = sum(item.received_count for item in self.statistics.values())
            failure_ratio = parse_failure_count / max(1, received_count)
            maximum_ratio = float(builder_config.get("max_parse_failure_ratio", 0.01))
            if failure_ratio > maximum_ratio:
                raise BuildError(
                    f"parse failure ratio {failure_ratio:.6f} exceeds configured {maximum_ratio:.6f}")
            if not entity_logger.entities:
                raise BuildError("RRD contains no application entities")
            logging.info("RRD validation passed; calculating SHA256")
            partial_hash = sha256_file(self.partial)
            elapsed = time.monotonic() - self.started_monotonic
            cpu_seconds = time.process_time() - self.started_cpu
            output_bytes = self.partial.stat().st_size
            source_bytes = sum(path.stat().st_size for path in source_file_list(
                sources, camera_to_log if self.options.camera_mode != "none" else []))
            statistics_json = {topic: asdict(stats) for topic, stats in sorted(self.statistics.items())}
            observed_starts = [
                value for stats in self.statistics.values()
                for value in (stats.first_source_time_ns, stats.first_logged_time_ns)
                if value is not None
            ]
            observed_ends = [
                value for stats in self.statistics.values()
                for value in (stats.last_source_time_ns, stats.last_logged_time_ns)
                if value is not None
            ]
            manifest_document.update({
                "start_ros_time_ns": min(observed_starts) if observed_starts else 0,
                "end_ros_time_ns": max(observed_ends) if observed_ends else 0,
                "topic_statistics": statistics_json,
                "parse_failures": {topic: stats.parse_failure_count for topic, stats in self.statistics.items()
                                   if stats.parse_failure_count},
                "rate_limits": {topic: stats.rate_limited_count for topic, stats in self.statistics.items()
                                if stats.rate_limited_count},
                "pointcloud_filter": {
                    "point_stride": int(bridge_config.get("point_stride", 1)),
                    "voxel_size": float(builder_config.get("voxel_size", 0.0)),
                    "max_point_count": min(100000, int(bridge_config.get("max_point_count", 100000))),
                },
                "camera_policy": {
                    "mode": self.options.camera_mode,
                    "max_hz": float(builder_config.get("camera_preview_max_hz", 1.0)),
                    "max_width": int(builder_config.get("camera_preview_max_width", 1280)),
                    "encoding": "jpeg" if self.options.camera_mode == "preview" else "raw",
                    "jpeg_quality": int(builder_config.get("camera_preview_jpeg_quality", 85)),
                    "decode_failures": self.statistics.get(
                        "/camera/front/image_raw", TopicStatistics()).camera_decode_failure_count,
                },
                "entities": sorted(entity_logger.entities),
                "resource_usage": {
                    "wall_elapsed_sec": elapsed,
                    "cpu_seconds": cpu_seconds,
                    "peak_rss_bytes": max(self.peak_rss, resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024),
                    "baseline_rss_bytes": self.baseline_rss,
                    "source_bytes": source_bytes,
                    "rrd_bytes": output_bytes,
                    "rrd_to_source_ratio": output_bytes / max(1, source_bytes),
                },
                "rrd_sha256": partial_hash,
                "valid_for_formal_validation": self.options.mode == "strict",
                "complete": True,
                "completed_at": utc_now(),
            })
            validation_document.update({
                "success": True,
                "partial_nonzero": True,
                "writer_closed": True,
                "rerun_cli": cli_validation,
                "required_topic_failures": required_failures,
                "source_reader_count_mismatches": delivery_mismatches,
                "parse_failure_ratio": failure_ratio,
                "parse_failure_limit": maximum_ratio,
                "entities_present": sorted(entity_logger.entities),
                "time_range_ns": [manifest_document["start_ros_time_ns"], manifest_document["end_ros_time_ns"]],
                "subscriber_drop": {"applicable": False, "reason": "direct rosbag2 reader; no DDS subscriber"},
                "sha256": partial_hash,
                "atomic_commit": True,
            })
            # Prepare reports before making the formal RRD visible.  The final
            # manifest already describes the validated bytes in the partial.
            self.write_manifest(manifest_document)
            atomic_json(self.validation_path, validation_document)
            checksum_temporary = self.options.output.with_suffix(self.options.output.suffix + ".sha256.partial")
            atomic_text(checksum_temporary, f"{partial_hash}  {self.options.output.name}\n")
            # Recheck just before publishing. Source data may have changed while decoding.
            for rel, expected in {**registered_hashes, **manifest_document["control_file_sha256"]}.items():
                path = self.options.run_dir / rel
                if not path.is_file() or sha256_file(path) != expected:
                    manifest_document["hash_verified"] = False
                    raise BuildError(f"source_hash changed during build: {rel}")
            if sha256_file(self.options.config) != manifest_document["config_sha256"]:
                raise BuildError("builder config changed during build")
            os.replace(self.partial, self.options.output)
            os.replace(checksum_temporary, self.options.output.with_suffix(self.options.output.suffix + ".sha256"))
            self.committed = True
            logging.info("RRD committed: %s (%d bytes)", self.options.output, output_bytes)
            return 0
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
            logging.exception("run RRD build failed: %s", error)
            validation_document.setdefault("errors", []).append(error)
            validation_document.update({"success": False, "failed_at": utc_now(), "atomic_commit": False})
            manifest_document.update({
                "complete": False,
                "valid_for_formal_validation": False,
                "failure": error,
                "topic_statistics": {topic: asdict(stats) for topic, stats in sorted(self.statistics.items())},
            })
            try:
                self.write_manifest(manifest_document)
                atomic_json(self.validation_path, validation_document)
            except Exception as report_exc:
                logging.error("failed to persist failure reports: %s", report_exc)
            if self.committed:
                # Only a file created by this invocation can reach this branch.
                try:
                    self.options.output.unlink()
                except OSError:
                    pass
                self.committed = False
            if session is not None:
                try:
                    session.close()
                except Exception:
                    logging.exception("failed to close Rerun session after conversion error")
            # Failure evidence is always retained, including source-hash failures.
            # The old CLI switch remains accepted for compatibility.
            if self.partial.exists():
                logging.error("retained incomplete partial: %s", self.partial)
            return 1
        finally:
            self._restore_signal_handlers()
            self._release_lock()


def main(argv: list[str] | None = None) -> int:
    try:
        options = parse_arguments(argv)
        return RunBuilder(options).run()
    except BuildError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
