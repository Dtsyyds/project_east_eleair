#!/usr/bin/env python3
"""Replay split FIELD-W5 bags and timestamped video on one unified timeline."""

from __future__ import annotations

import argparse
import math
import os
import signal
import tempfile
import heapq
import json
from pathlib import Path
import subprocess
import sys
import threading
import time
from typing import Any

import cv2
import rclpy
from builtin_interfaces.msg import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, Imu
from ament_index_python.packages import get_package_prefix
from std_srvs.srv import SetBool

try:
    from rosgraph_msgs.msg import Clock
except ImportError:
    Clock = None

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import atomic_json, load_json, sha256_file, utc_now  # noqa: E402


from inspection_tools.video_index import aligned_rows

MID360_SENSOR_REPLAY_TOPICS = {"/livox/lidar", "/livox/imu_raw"}


def _time_message(nanoseconds: int) -> Time:
    return Time(sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)


class BagSource:
    def __init__(self, path: Path, topics: set[str] | None = None) -> None:
        import rosbag2_py

        self.path = path
        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(
            rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        self.types = {item.name: item.type for item in self.reader.get_all_topics_and_types()}
        self.topics = topics
        self.next_item: tuple[int, str, bytes, str] | None = None
        self.advance()

    def advance(self) -> None:
        self.next_item = None
        while self.reader.has_next():
            topic, data, timestamp = self.reader.read_next()
            if topic == "/clock":
                continue
            if self.topics is None or topic in self.topics:
                self.next_item = (int(timestamp), topic, data, self.types[topic])
                return


class VideoSource:
    def __init__(self, run_dir: Path) -> None:
        self.segments: list[tuple[Path, list[dict[str, Any]]]] = []
        for video in sorted((run_dir / "video" / "front").glob("front_*.mkv")):
            sidecar = video.with_suffix(".frames.jsonl")
            rows = aligned_rows(video, sidecar)
            if self.segments and rows and self.segments[-1][1] and int(rows[0]["ros_header_stamp_ns"]) <= int(self.segments[-1][1][-1]["ros_header_stamp_ns"]):
                raise ValueError("camera segment timestamps overlap or regress")
            self.segments.append((video, rows))
        self.segment_index = 0
        self.frame_index = 0
        self.capture: cv2.VideoCapture | None = None
        self.next_item: tuple[int, str, Image, str] | None = None
        self.advance()

    def advance(self) -> None:
        self.next_item = None
        while self.segment_index < len(self.segments):
            video, rows = self.segments[self.segment_index]
            if self.capture is None:
                self.capture = cv2.VideoCapture(str(video))
                if not self.capture.isOpened():
                    raise RuntimeError(f"cannot open video segment: {video}")
            if self.frame_index >= len(rows):
                self.capture.release()
                self.capture = None
                self.segment_index += 1
                self.frame_index = 0
                continue
            ok, frame = self.capture.read()
            if not ok:
                raise RuntimeError(
                    f"cannot decode segment={video} frame={self.frame_index}"
                )
            row = rows[self.frame_index]
            message = Image()
            message.header.stamp = _time_message(int(row["ros_header_stamp_ns"]))
            message.header.frame_id = "mindvision_optical_frame"
            message.height, message.width = frame.shape[:2]
            message.encoding = "bgr8"
            message.is_bigendian = 0
            message.step = message.width * 3
            message.data = frame.tobytes()
            self.next_item = (
                int(row["ros_header_stamp_ns"]), "/camera/front/image_raw",
                message, "sensor_msgs/msg/Image",
            )
            self.frame_index += 1
            return


class ReplayNode(Node):
    def __init__(self, publish_clock: bool) -> None:
        super().__init__("field_w5_unified_replay")
        # rclpy.Node exposes a read-only ``publishers`` property. Keep the
        # replay-owned cache private instead of shadowing that API property.
        self._field_publishers: dict[tuple[str, str], Any] = {}
        self.paused = False
        self.pause_lock = threading.Lock()
        self.pause_service = self.create_service(SetBool, "/vehicle_replay/pause", self.set_pause)
        self.clock_publisher = self.create_publisher(Clock, "/clock", 10) \
            if publish_clock and Clock is not None else None

    def set_pause(self, request: SetBool.Request, response: SetBool.Response):
        with self.pause_lock:
            self.paused = bool(request.data)
        response.success = True
        response.message = "paused" if self.paused else "resumed"
        return response

    def is_paused(self) -> bool:
        with self.pause_lock:
            return self.paused

    def publisher(self, topic: str, type_name: str):
        key = (topic, type_name)
        if key not in self._field_publishers:
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_ALL if topic == "/tf_static" else HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL if topic == "/tf_static" else DurabilityPolicy.VOLATILE,
            )
            self._field_publishers[key] = self.create_publisher(
                get_message(type_name), topic, qos
            )
        return self._field_publishers[key]


class ReplayImuPublisher:
    """Reuse the installed native g->SI adapter; never open a Livox device.

    The source clock still schedules native input. A bounded journal checks the
    actual converted header/count before closing, without copying conversion math.
    """
    def __init__(self,node: ReplayNode):
        self.node=node;self.lock=threading.Condition();self.pending={}
        self.sent=0;self.received=0;self.foreign=0
        self.name=f"field_replay_imu_{os.getpid()}"
        self.input_topic=f"/vehicle_replay/_imu_raw_{os.getpid()}"
        self.converted_topic=f"/vehicle_replay/_imu_si_{os.getpid()}"
        self.output=node.create_publisher(Imu,"/livox/imu",QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE))
        self.error=None
        self.feed=node.create_publisher(Imu,self.input_topic,QoSProfile(depth=200,reliability=ReliabilityPolicy.RELIABLE))
        self.observer=node.create_subscription(Imu,self.converted_topic,self.observe,QoSProfile(depth=200,reliability=ReliabilityPolicy.RELIABLE))
        binary=Path(get_package_prefix("inspection_livox_adapter"))/"lib/inspection_livox_adapter/livox_imu_adapter_node"
        if not binary.is_file():raise FileNotFoundError(binary)
        self.binary=str(binary);self.binary_sha=sha256_file(binary)
        self.log=tempfile.TemporaryFile(mode="w+t")
        self.process=subprocess.Popen([self.binary,"--ros-args","-r",f"__node:={self.name}","-r",f"/livox/imu_raw:={self.input_topic}","-r",f"/livox/imu:={self.converted_topic}","-r","/diagnostics:=/vehicle_replay/imu_diagnostics"],stdout=self.log,stderr=subprocess.STDOUT)

    def get_subscription_count(self):
        if self.process.poll() is not None or self.feed.get_subscription_count()==0 or self.node.count_publishers(self.converted_topic)==0:return 0
        return self.output.get_subscription_count()

    @staticmethod
    def identity(message):
        return (message.header.stamp.sec*1000000000+message.header.stamp.nanosec,message.header.frame_id)

    def failure_detail(self):
        self.log.seek(0);return self.log.read()[-2000:]

    def publish(self,message):
        if self.process.poll() is not None:raise RuntimeError("native IMU adapter exited: "+self.failure_detail())
        key=self.identity(message)
        with self.lock:
            if self.sent-self.received>=200:raise RuntimeError("native IMU conversion backlog exceeds200; offline results cannot be silently dropped")
            self.pending[key]=self.pending.get(key,0)+1;self.sent+=1
        self.feed.publish(message)

    def observe(self,message):
        key=self.identity(message)
        with self.lock:
            if key not in self.pending:self.foreign+=1;return
            self.pending[key]-=1
            if self.pending[key]==0:del self.pending[key]
            try:self.output.publish(message)
            except Exception as error:self.error=str(error);self.lock.notify_all();return
            self.received+=1;self.lock.notify_all()

    def drain(self,timeout=2.):
        deadline=time.monotonic()+timeout
        with self.lock:
            while self.received<self.sent and rclpy.ok() and self.process.poll() is None and self.error is None:
                remaining=deadline-time.monotonic()
                if remaining<=0:break
                self.lock.wait(min(.05,remaining))
            if self.received!=self.sent:raise RuntimeError(f"native IMU conversion incomplete: input={self.sent}, output={self.received}, pending={self.sent-self.received}, error={self.error}; adapter_log={self.failure_detail()}")

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
                try:self.process.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    self.process.terminate();self.process.wait(timeout=1.)
        finally:self.log.close()

    def result(self):
        return {'status':'verified' if self.sent==self.received else 'failed','native_input_count':self.sent,
                'converted_output_count':self.received,'pending':self.sent-self.received,'foreign_messages_ignored':self.foreign,
                'input_units':'g','output_units':'m/s^2','gyro_units':'rad/s unchanged',
                'orientation':'unavailable; native adapter covariance[0]=-1',
                'implementation':'installed inspection_livox_adapter, no duplicated conversion',
                'binary':self.binary,'binary_sha256':self.binary_sha,'hardware_started':False}


def map_topic(mode: str, topic: str) -> str:
    if mode in {"sensor-only", "full-sensor"}:
        if topic == "/livox/lidar":
            return "/livox/lidar_points"
        if topic == "/livox/imu_raw":
            return "/livox/imu"
    if mode != "algorithm-replay":
        return topic
    if topic == "/camera/front/image_raw":
        return "/replay/camera/front/image_raw"
    if topic == "/camera/front/camera_info":
        return "/replay/camera/front/camera_info"
    return "/replay/recorded" + topic


def advertise_source_topics(node: ReplayNode, sources: list[Any], mode: str) -> None:
    """Create bag publishers before scheduling starts so ROS tools can discover them.

    Video decode can be expensive on the target, but that must not hide the D405
    and MID-360 topics from rviz2/ros2 topic list while the first frames decode.
    """
    for source in sources:
        if isinstance(source, BagSource):
            for topic, type_name in source.types.items():
                if topic == "/clock":
                    continue
                if source.topics is not None and topic not in source.topics:
                    continue
                node.publisher(map_topic(mode, topic), type_name)
        elif isinstance(source, VideoSource):
            node.publisher(
                map_topic(mode, "/camera/front/image_raw"),
                "sensor_msgs/msg/Image",
            )


def wait_for_replay_subscribers(node: ReplayNode, timeout_sec: float) -> dict[str, Any]:
    """Bounded DDS discovery before the media clock starts, never change rate/QoS.

    Matching is publisher-side evidence only. Independent receipt/count checks
    remain necessary, even if every endpoint is discovered.
    """
    began=time.monotonic();deadline=began+timeout_sec;settled=0.0
    counts={}
    def snapshot():
        return {topic:publisher.get_subscription_count()
                for (topic,_),publisher in node._field_publishers.items()}
    while timeout_sec>0 and rclpy.ok():
        counts=snapshot()
        if counts and all(counts.values()):
            # RMW endpoint match can precede the reader transport being ready.
            settle_begin=time.monotonic();settle_end=min(deadline,settle_begin+.3)
            while rclpy.ok() and time.monotonic()<settle_end:
                time.sleep(min(.01,max(0.,settle_end-time.monotonic())))
            settled=time.monotonic()-settle_begin
            if all(snapshot().values()):break
        if time.monotonic()>=deadline:break
        time.sleep(min(.01,max(0.,deadline-time.monotonic())))
    counts=snapshot()
    return {'status':'not_executed' if timeout_sec==0 else 'verified' if counts and all(counts.values()) else 'unverified',
            'evidence':'publisher_endpoint_matching_only','data_receipt_verified':False,
            'timeout_sec':timeout_sec,'elapsed_sec':time.monotonic()-began,
            'settle_sec':settled,'subscriber_counts':counts,
            'missing_topics':sorted(topic for topic,count in counts.items() if count==0),
            'media_clock_started_after_discovery':True}


def source_hashes(run_dir: Path, sources: list[Any]) -> dict[str, str]:
    """Hash selected dependencies once outside the timed replay scheduler."""
    files = [run_dir / "run_manifest.json"]
    for source in sources:
        if isinstance(source, BagSource):
            files.extend(source.path.glob("*.mcap"))
            files.append(source.path / "metadata.yaml")
        elif isinstance(source, VideoSource):
            for video, _ in source.segments:
                files.extend([video, video.with_suffix(".frames.jsonl"), video.with_suffix(".frame_map.json")])
    hashes = {}
    for path in sorted(set(files)):
        if not path.resolve().is_relative_to(run_dir.resolve()):
            raise ValueError("replay source escapes run directory")
        if not path.is_file(): raise FileNotFoundError(path)
        hashes[path.relative_to(run_dir).as_posix()] = sha256_file(path)
    return hashes


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--mode",
        choices=(
            "core-only", "sensor-only", "perception", "full-sensor",
            "algorithm-replay",
            "mapping-only", "mapping-eddy", "full",
        ),
        required=True,
    )
    parser.add_argument("--rate", type=float, choices=(0.5, 1.0), default=1.0)
    parser.add_argument("--start-offset", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--publish-clock", action="store_true")
    parser.add_argument("--discovery-wait-sec", type=float, default=2.0, help="bounded subscriber discovery before replay time origin; 0 restores immediate startup")
    parser.add_argument("--report-dir", type=Path, help="derived reports, never inside a sealed run")
    args = parser.parse_args(argv)
    if not math.isfinite(args.discovery_wait_sec) or args.discovery_wait_sec<0:
        parser.error("discovery-wait-sec must be finite and non-negative")
    if args.start_offset < 0 or args.duration < 0:
        parser.error("start-offset and duration must be non-negative")
    run_dir = args.run_dir.resolve()
    import uuid
    analysis_id = "replay_" + utc_now().replace(":", "").replace("+", "_") + "_" + uuid.uuid4().hex[:8]
    report_dir = (args.report_dir or run_dir.parent.parent / "analysis" / run_dir.name / analysis_id / "metrics").resolve()
    if report_dir.is_relative_to(run_dir):
        raise ValueError("replay reports must be outside source run")
    if any((report_dir / name).exists() for name in ("replay_validation.json", "result_manifest.json")):
        raise FileExistsError("refusing to overwrite existing replay report")
    sources: list[Any] = []
    mapping_mode = args.mode in {"mapping-only", "mapping-eddy", "full"}
    if mapping_mode:
        mapping = run_dir / "mcap" / "mapping"
        if not mapping.is_dir():
            raise RuntimeError("mapping MCAP is missing")
        sources.append(BagSource(mapping, {"/aft_mapped_to_init"}))
    core_filter: set[str] | None = None
    if args.mode == "algorithm-replay":
        core_filter = {"/camera/front/camera_info"}
    core = run_dir / "mcap" / "core"
    if core.is_dir() and (not mapping_mode or args.mode == "full"):
        sources.append(BagSource(core, core_filter))
    if args.mode in {"mapping-eddy", "full"}:
        eddy = run_dir / "mcap" / "eddy"
        if not eddy.is_dir():
            raise RuntimeError("Eddy MCAP is missing")
        sources.append(BagSource(eddy))
    if args.mode in {"sensor-only", "full-sensor"}:
        for name in ("mid360", "d405", "eddy"):
            path = run_dir / "mcap" / name
            if path.is_dir():
                topics = MID360_SENSOR_REPLAY_TOPICS if name == "mid360" else None
                sources.append(BagSource(path, topics))
    if args.mode in {"perception", "full-sensor", "algorithm-replay", "full"}:
        if list((run_dir / "video" / "front").glob("front_*.mkv")):
            sources.append(VideoSource(run_dir))
        elif (run_dir / "mcap" / "raw_burst").is_dir():
            sources.append(BagSource(
                run_dir / "mcap" / "raw_burst", {"/camera/front/image_raw"}
            ))
        else:
            raise RuntimeError("mode requires front video or raw_burst evidence")
    hash_started = time.monotonic()
    actual_source_sha = source_hashes(run_dir, sources)
    hash_elapsed = time.monotonic() - hash_started
    heap = []
    for index, source in enumerate(sources):
        if source.next_item is not None:
            heapq.heappush(heap, (source.next_item[0], index))
    if not heap:
        raise RuntimeError("no replay messages available")
    first_timestamp = heap[0][0]
    begin_timestamp = first_timestamp + int(args.start_offset * 1e9)
    end_timestamp = (
        begin_timestamp + int(args.duration * 1e9) if args.duration > 0 else None
    )
    rclpy.init(args=None)
    node = ReplayNode(args.publish_clock)
    imu_adapter=None
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    published: dict[str, int] = {}
    first_published: dict[str, int] = {}
    last_published: dict[str, int] = {}
    start_monotonic: float | None = None
    paused_since: float | None = None
    paused_total = 0.0
    startup_discovery: dict[str, Any] = {}
    try:
        if args.mode in {"sensor-only","full-sensor"} and any(isinstance(source,BagSource) and "/livox/imu_raw" in source.types and (source.topics is None or "/livox/imu_raw" in source.topics) for source in sources):
            if any(isinstance(source,BagSource) and "/livox/imu" in source.types and (source.topics is None or "/livox/imu" in source.topics) for source in sources):
                raise ValueError("selected run contains both native and SI IMU streams; output ownership is ambiguous")
            imu_adapter=ReplayImuPublisher(node)
            node._field_publishers[("/livox/imu","sensor_msgs/msg/Imu")]=imu_adapter
        advertise_source_topics(node, sources, args.mode)
        startup_discovery=wait_for_replay_subscribers(node,args.discovery_wait_sec)
        while heap and rclpy.ok():
            timestamp, index = heapq.heappop(heap)
            source = sources[index]
            item = source.next_item
            if item is None:
                continue
            source.advance()
            if source.next_item is not None:
                heapq.heappush(heap, (source.next_item[0], index))
            if timestamp < begin_timestamp:
                continue
            if end_timestamp is not None and timestamp > end_timestamp:
                break
            if start_monotonic is None:
                start_monotonic = time.monotonic()
            while node.is_paused() and rclpy.ok():
                if paused_since is None:
                    paused_since = time.monotonic()
                time.sleep(0.05)
            if paused_since is not None:
                paused_total += time.monotonic() - paused_since
                paused_since = None
            target = start_monotonic + paused_total + (timestamp - begin_timestamp) / 1e9 / args.rate
            while rclpy.ok():
                remaining = target - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(0.01, remaining))
            _, topic, payload, type_name = item
            message = payload if isinstance(payload, Image) else deserialize_message(payload, get_message(type_name))
            output_topic = map_topic(args.mode, topic)
            node.publisher(output_topic, type_name).publish(message)
            published[output_topic] = published.get(output_topic, 0) + 1
            first_published.setdefault(output_topic, timestamp)
            last_published[output_topic] = timestamp
            if node.clock_publisher is not None:
                clock = Clock()
                clock.clock = _time_message(timestamp)
                node.clock_publisher.publish(clock)
        if imu_adapter is not None:
            imu_adapter.drain()
            published["/livox/imu"]=imu_adapter.received
    finally:
        try:
            if imu_adapter is not None:imu_adapter.close()
        finally:
            executor.shutdown(timeout_sec=5.0)
            node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
            spin_thread.join(timeout=5.0)
    manifest = load_json(run_dir / "run_manifest.json", {}) or {}
    frozen = manifest.get("frozen_sources", [])
    media_first = min(first_published.values()) if first_published else None
    media_last = max(last_published.values()) if last_published else None
    media_span = ((media_last - media_first) / 1e9
                  if media_first is not None and media_last is not None else 0.0)
    wall_elapsed = (time.monotonic() - start_monotonic) if start_monotonic is not None else 0.0
    required = {"/aft_mapped_to_init"} if mapping_mode else set()
    if args.mode in {"mapping-eddy", "full"}:
        required.add("/eddy/channel_mean")
    if args.mode == "full":
        required.update({"/camera/front/image_raw", "/camera/front/camera_info", "/diagnostics"})
    missing_required = sorted(topic for topic in required if published.get(topic, 0) <= 0)
    realtime_achieved = wall_elapsed <= media_span / args.rate * 1.10 + 2.0 if media_span else False
    report = {
        "schema_version": ("field_w6_replay_validation_v1" if mapping_mode
                           else "field_w5_replay_validation_v1"),
        "created_at": utc_now(), "executed": True,
        "mode": args.mode, "rate": args.rate,
        "start_offset_sec": args.start_offset, "duration_limit_sec": args.duration,
        "topic_counts": published,
        "topic_spans_sec": {
            topic: (last_published[topic] - first_published[topic]) / 1e9
            for topic in published
        },
        "required_topics": sorted(required), "missing_required_topics": missing_required,
        "media_span_sec": media_span, "wall_elapsed_sec": wall_elapsed,
        "realtime_achieved": realtime_achieved,
        "unified_scheduler": True,
        "startup_discovery": startup_discovery,
        "imu_adaptation": imu_adapter.result() if imu_adapter is not None else {"status":"not_applicable","reason":"no selected native IMU stream requiring SI conversion"},
        "clock_publishers_started": 1 if args.publish_clock else 0,
        "pause_service": "/vehicle_replay/pause",
        "config_reference_sha": [
            {"frozen": item.get("frozen"), "sha256": item.get("sha256")}
            for item in frozen if "yaml" in str(item.get("frozen")) or "reference" in str(item.get("frozen"))
        ],
        "video_frame_header_alignment": "verified decoded identity map uses original ros_header_stamp_ns",
        "detected_state_consistency": None,
        "geometry_comparison": None,
        "first_mismatch": None,
        "note": "algorithm output comparison requires algorithm-replay validation subscriber; encoded pixels are not raw-equivalent",
        "realtime_expectation": (
            "software H.264 decode may make full mode non-realtime; this is not MCAP loss"
            if args.mode == "full" else "mapping replay is expected to meet the selected rate"
        ),
        "valid": bool(published) and not missing_required and (
            realtime_achieved if args.mode in {"mapping-only", "mapping-eddy"} else True
        ),
    }
    report["transport_acceptance"] = {"status": "not_executed", "passed": None,
        "reason": "publisher counts recorded; requires independent ROS subscriber/type/QoS/count validation"}
    report["algorithm_acceptance"] = {"status": "not_executed", "passed": None}
    report["source_run_id"] = run_dir.name
    report["source_data_modified"] = False
    report["source_sha256"] = actual_source_sha
    report["source_hash_checked_at"] = "before scheduler; source mutation during playback not continuously monitored"
    report["source_hash_preflight_elapsed_sec"] = hash_elapsed
    config_hashes = {}
    for item in frozen:
        relative = item.get("frozen")
        if relative:
            path = run_dir / str(relative)
            if path.resolve().is_relative_to(run_dir) and path.is_file():
                config_hashes[str(relative)] = sha256_file(path)
    atomic_json(report_dir / "replay_validation.json", report)
    atomic_json(report_dir / "result_manifest.json", {
        "schema_version": "tank_derived_result_v1", "source_run_id": run_dir.name,
        "analysis_type": "replay", "source_sha256": actual_source_sha,
        "config_sha256": config_hashes, "code_status": {"script_sha256": sha256_file(Path(__file__)),
            "repository_head": "unavailable_unborn_repository", "imu_adapter":imu_adapter.result() if imu_adapter is not None else {"status":"not_applicable"}},
        "time_range_ns": [media_first, media_last], "coordinate_frames": "source message frame_id",
        "calibration_status": "unverified", "complete": True,
        "replay_publisher_acceptance": report["valid"],
        "transport_acceptance": report["transport_acceptance"],
        "algorithm_acceptance": report["algorithm_acceptance"],
        "outputs": [{"path": "replay_validation.json", "sha256": sha256_file(report_dir / "replay_validation.json")}],
    })
    # Replay is derived analysis. It never refinalizes or rewrites source seals.
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["valid"] and report.get("post_replay_refinalize_returncode", 0) == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
