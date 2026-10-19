from __future__ import annotations

import gc
import hashlib
import json
from pathlib import Path
import signal
import struct

import pytest
import yaml

from inspection_rerun.pointcloud_adapter import pointcloud_to_numpy
from inspection_rerun.run_rrd_builder import (
    BagSource,
    BuildError,
    BuildOptions,
    CameraSegment,
    OfflineEntityLogger,
    RunBuilder,
    TopicStatistics,
    discover_bag_sources,
    discover_camera,
    log_camera_segments,
    quality_gates,
    sanitize_entity,
    sha256_file,
    validate_paths,
)


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding="utf-8")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def minimal_source(run: Path, group="core", start=100, end=200) -> BagSource:
    path = run / "mcap" / group
    path.mkdir(parents=True, exist_ok=True)
    (path / "metadata.yaml").write_text("metadata", encoding="utf-8")
    (path / f"{group}_0.mcap").write_bytes(b"mcap")
    return BagSource(
        group, path.resolve(), {"/status": "std_msgs/msg/String"},
        {"/status": 1}, {"/status": (start, end)}, start, end,
    )


def gate_documents(run: Path, source: BagSource):
    files = []
    for path in (source.path / "metadata.yaml", source.path / f"{source.group}_0.mcap"):
        files.append({
            "path": str(path.relative_to(run)), "size": path.stat().st_size,
            "sha256": digest(path),
        })
    manifest = {
        "run_id": run.name, "ros_distro": "humble",
        "enabled_writers": {"core": True},
        "status": {"complete": True, "finalized": True,
                   "valid_for_formal_validation": True}, "files": files,
    }
    complete = {"complete": True, "finalized": True, "sha256_valid": True,
                "validation": "passed"}
    validation = {"valid": True, "bags": {}}
    return manifest, complete, validation


def test_run_and_output_path_validation(tmp_path: Path):
    run = tmp_path / "RUN-1"
    (run / "rerun").mkdir(parents=True)
    config = tmp_path / "rerun.yaml"
    config.write_text("rerun_bridge: {ros__parameters: {}}", encoding="utf-8")
    valid = validate_paths(BuildOptions(run, config, run / "rerun" / "RUN-1.rrd"))
    assert valid.run_dir.is_absolute() and valid.output.is_absolute()
    with pytest.raises(BuildError, match="inside"):
        validate_paths(BuildOptions(run, config, tmp_path / "escape.rrd"))


def test_allow_incomplete_output_is_marked(tmp_path: Path):
    run = tmp_path / "RUN-2"
    (run / "rerun").mkdir(parents=True)
    config = tmp_path / "rerun.yaml"
    config.write_text("rerun_bridge: {ros__parameters: {}}", encoding="utf-8")
    value = validate_paths(BuildOptions(
        run, config, run / "rerun" / "RUN-2.rrd", mode="allow-incomplete"))
    assert value.output.name == "RUN-2_INCOMPLETE.rrd"


def test_strict_gate_success_and_failure(tmp_path: Path):
    run = tmp_path / "RUN-3"
    source = minimal_source(run)
    manifest, complete, validation = gate_documents(run, source)
    gates, bypassed = quality_gates(
        BuildOptions(run, tmp_path / "c", run / "rerun" / "x.rrd"),
        manifest, complete, validation, [source], [], {}, "0.36.2")
    assert all(item["passed"] for item in gates.values())
    assert bypassed == []
    with pytest.raises(BuildError, match="complete"):
        quality_gates(
            BuildOptions(run, tmp_path / "c", run / "rerun" / "x.rrd"),
            manifest, {"complete": False, "finalized": True}, validation,
            [source], [], {}, "0.36.2")


def test_allow_incomplete_records_bypassed_gate(tmp_path: Path):
    run = tmp_path / "RUN-4"
    source = minimal_source(run)
    manifest, complete, validation = gate_documents(run, source)
    complete["complete"] = False
    options = BuildOptions(
        run, tmp_path / "c", run / "rerun" / "x_INCOMPLETE.rrd",
        mode="allow-incomplete")
    gates, bypassed = quality_gates(
        options, manifest, complete, validation, [source], [], {}, "0.36.2")
    assert "complete" in bypassed
    assert gates["complete"]["bypassed"] is True


def test_manifest_driven_discovery_and_cross_run_rejection(tmp_path: Path):
    run = tmp_path / "RUN-5"
    bag = run / "mcap" / "core"
    bag.mkdir(parents=True)
    metadata = {
        "rosbag2_bagfile_information": {
            "topics_with_message_count": [{
                "topic_metadata": {"name": "/status", "type": "std_msgs/msg/String"},
                "message_count": 2,
            }]
        }
    }
    (bag / "metadata.yaml").write_text(yaml.safe_dump(metadata), encoding="utf-8")
    validation = {"bags": {"core": {
        "path": str(bag), "message_validation": {"topics": {
            "/status": {"type": "std_msgs/msg/String", "count": 2,
                        "first_bag_timestamp_ns": 10, "last_bag_timestamp_ns": 20}
        }}
    }}}
    sources = discover_bag_sources(run, {}, validation, False)
    assert sources[0].topic_counts["/status"] == 2
    validation["bags"]["core"]["path"] = str(tmp_path / "OTHER" / "mcap" / "core")
    with pytest.raises(BuildError, match="outside"):
        discover_bag_sources(run, {}, validation, False)


def test_multi_bag_time_ranges_must_overlap(tmp_path: Path):
    run = tmp_path / "RUN-6"
    first = minimal_source(run, "core", 100, 200)
    second = minimal_source(run, "mapping", 300, 400)
    second.topics = {"/pose": "nav_msgs/msg/Odometry"}
    second.topic_counts = {"/pose": 1}
    manifest, complete, validation = gate_documents(run, first)
    manifest["enabled_writers"]["mapping"] = True
    for path in (second.path / "metadata.yaml", second.path / "mapping_0.mcap"):
        manifest["files"].append({"path": str(path.relative_to(run)), "sha256": digest(path)})
    with pytest.raises(BuildError, match="time_overlap"):
        quality_gates(
            BuildOptions(run, tmp_path / "c", run / "rerun" / "x.rrd"),
            manifest, complete, validation, [first, second], [], {}, "0.36.2")


def test_existing_rrd_is_not_overwritten(tmp_path: Path):
    run = tmp_path / "RUN-7"
    (run / "rerun").mkdir(parents=True)
    config = tmp_path / "rerun.yaml"
    config.write_text("rerun_bridge: {ros__parameters: {}}", encoding="utf-8")
    output = run / "rerun" / "RUN-7.rrd"
    output.write_bytes(b"original")
    assert RunBuilder(BuildOptions(run, config, output)).run() == 1
    assert output.read_bytes() == b"original"


class Field:
    def __init__(self, name, offset):
        self.name, self.offset, self.datatype, self.count = name, offset, 7, 1


class Cloud:
    pass


def cloud(points):
    value = Cloud()
    value.width, value.height = len(points), 1
    value.point_step = 16
    value.row_step = 16 * len(points)
    value.is_bigendian = False
    value.fields = [Field("x", 0), Field("y", 4), Field("z", 8), Field("intensity", 12)]
    value.data = b"".join(struct.pack("<ffff", *point) for point in points)
    return value


def test_pointcloud_invalid_voxel_and_cap_statistics():
    points, _, stats = pointcloud_to_numpy(
        cloud([(0, 0, 0, 1), (0.01, 0.01, 0.01, 2),
               (float("nan"), 0, 0, 3), (2, 2, 2, 4)]),
        voxel_size=0.1, max_point_count=1, return_stats=True)
    assert points.shape == (1, 3)
    assert stats["invalid_point_count"] == 1
    assert stats["voxel_removed_point_count"] == 1
    assert stats["capped_point_count"] == 1


class FakeSession:
    def __init__(self):
        self.logs = []
        self.times = []

    def set_time(self, stamp, sequence):
        self.times.append((stamp, sequence))

    def log(self, path, archetype, static=False):
        self.logs.append((path, archetype, static))


class Stamp:
    sec = 1
    nanosec = 0


class Header:
    stamp = Stamp()


class TextMessage:
    header = Header()
    data = "ok"


def test_topic_counts_and_parse_failure_isolation():
    stats = {"/status": TopicStatistics(type="std_msgs/msg/String")}
    logger = OfflineEntityLogger(FakeSession(), {}, {}, stats)
    logger.handle("/status", "std_msgs/msg/String", TextMessage(), 10)
    assert stats["/status"].received_count == 1
    assert stats["/status"].logged_count == 1
    with pytest.raises(ValueError):
        logger.handle("/status", "unsupported/type", TextMessage(), 11)
    assert stats["/status"].parse_failure_count == 1
    logger.handle("/status", "std_msgs/msg/String", TextMessage(), 12)
    assert stats["/status"].logged_count == 2


def test_d405_16uc1_is_logged_as_depth_image():
    import rerun as rr

    message = type("DepthMessage", (), {})()
    message.header = Header()
    message.width, message.height, message.step = 2, 1, 4
    message.encoding = "16UC1"
    message.is_bigendian = 0
    message.data = bytes([0xE8, 0x03, 0xD0, 0x07])
    topic = "/d405/camera/depth/image_rect_raw"
    stats = {topic: TopicStatistics(type="sensor_msgs/msg/Image")}
    session = FakeSession()
    logger = OfflineEntityLogger(
        session, {"image_max_hz": 2.0}, {"depth_units_per_meter": 1000.0}, stats)

    logger.handle(topic, "sensor_msgs/msg/Image", message, 1_000_000_000)

    assert stats[topic].logged_count == 1
    assert stats[topic].parse_failure_count == 0
    assert any(isinstance(archetype, rr.DepthImage) for _, archetype, _ in session.logs)


def test_camera_sidecar_discovery_and_preview_decode(monkeypatch, tmp_path: Path):
    import cv2
    import numpy as np

    run = tmp_path / "RUN-8"
    root = run / "video" / "front"
    root.mkdir(parents=True)
    video = root / "front_000001.mkv"
    from inspection_tools.video_index import create_frame_map
    import subprocess
    subprocess.run(["ffmpeg", "-v", "error", "-f", "lavfi", "-i", "testsrc=size=40x20:rate=10",
                    "-frames:v", "12", "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
                    "-pix_fmt", "yuv420p", str(video)], check=True)
    sidecar = video.with_suffix(".frames.jsonl")
    sidecar.write_text("\n".join(json.dumps({
        "frame_index": index, "ros_header_stamp_ns": 1_000_000_000 + index * 100_000_000,
        "encoded_pts_ns": index * 100_000_000,
        "encoder_contract": "rawvideo_fixed_fps_unique_pts_v2",
    }) for index in range(12)) + "\n", encoding="utf-8")
    create_frame_map(video, sidecar, video.with_suffix(".frame_map.json"))
    segments = discover_camera(run)
    assert segments[0].frame_count == 12

    class Capture:
        def __init__(self, _path, *_args): self.index = 0
        def isOpened(self): return True
        def set(self, _kind, value): self.index = int(value); return True
        def read(self): return True, np.zeros((20, 40, 3), dtype=np.uint8)
        def release(self): pass

    monkeypatch.setattr(cv2, "VideoCapture", Capture)
    session = FakeSession()
    stats = {}
    logger = OfflineEntityLogger(session, {}, {"camera_preview_max_width": 10}, stats, "preview")
    log_camera_segments(segments, "preview", {
        "camera_preview_max_hz": 1.0, "camera_preview_max_width": 10,
    }, logger, stats, lambda: None)
    assert stats["/camera/front/image_raw"].source_count == 12
    assert stats["/camera/front/image_raw"].logged_count == 2
    assert stats["/camera/front/image_raw"].rate_limited_count == 10
    assert stats["/camera/front/image_raw"].camera_decode_failure_count == 0


def test_camera_none_does_not_discover_or_decode(monkeypatch, tmp_path: Path):
    import inspection_rerun.run_rrd_builder as module

    run = tmp_path / "RUN-9"
    (run / "rerun").mkdir(parents=True)
    config = tmp_path / "rerun.yaml"
    config.write_text("rerun_bridge: {ros__parameters: {}}", encoding="utf-8")
    write_json(run / "run_manifest.json", {})
    write_json(run / "complete.json", {})
    write_json(run / "reports" / "mcap_validation.json", {})
    monkeypatch.setattr(module, "discover_camera", lambda _run: pytest.fail("camera was touched"))
    builder = RunBuilder(BuildOptions(run, config, run / "rerun" / "x.rrd", camera_mode="none"))
    assert builder.run() == 1


def test_signal_requests_safe_shutdown(tmp_path: Path):
    run = tmp_path / "RUN-10"
    (run / "rerun").mkdir(parents=True)
    config = tmp_path / "rerun.yaml"
    config.write_text("rerun_bridge: {ros__parameters: {}}", encoding="utf-8")
    builder = RunBuilder(BuildOptions(run, config, run / "rerun" / "x.rrd"))
    builder._signal_handler(signal.SIGTERM, None)
    with pytest.raises(BuildError, match="interrupted"):
        builder.check_abort()


def test_empty_sources_fail_strict_gate(tmp_path: Path):
    run = tmp_path / "RUN-11"
    run.mkdir()
    with pytest.raises(BuildError, match="sources_present"):
        quality_gates(
            BuildOptions(run, tmp_path / "c", run / "rerun" / "x.rrd"),
            {"run_id": run.name, "status": {
                "complete": True, "finalized": True,
                "valid_for_formal_validation": True}},
            {"complete": True, "finalized": True, "sha256_valid": True,
             "validation": "passed"},
            {"valid": True, "bags": {}},
            [], [], {}, "0.36.2")


def create_fixture_run(tmp_path: Path) -> tuple[Path, Path]:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String

    run = tmp_path / "FIXTURE-RUN"
    bag = run / "mcap" / "core"
    bag.parent.mkdir(parents=True)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        name="/field/collection_status", type="std_msgs/msg/String", serialization_format="cdr"))
    for index in range(3):
        message = String(data=f"fixture-{index}")
        writer.write("/field/collection_status", serialize_message(message), 1_000_000_000 + index)
    del writer
    gc.collect()
    files = []
    for path in sorted(bag.iterdir()):
        if path.is_file():
            files.append({"path": str(path.relative_to(run)), "size": path.stat().st_size,
                          "sha256": digest(path)})
    manifest = {
        "run_id": run.name, "ros_distro": "humble",
        "enabled_writers": {"core": True},
        "status": {"complete": True, "finalized": True,
                   "valid_for_formal_validation": True}, "files": files,
    }
    validation = {"valid": True, "bags": {"core": {
        "path": str(bag.resolve()), "valid": True,
        "message_validation": {"topics": {"/field/collection_status": {
            "type": "std_msgs/msg/String", "count": 3,
            "first_bag_timestamp_ns": 1_000_000_000,
            "last_bag_timestamp_ns": 1_000_000_002,
        }}},
    }}}
    write_json(run / "run_manifest.json", manifest)
    write_json(run / "complete.json", {
        "complete": True, "finalized": True, "sha256_valid": True,
        "validation": "passed"})
    write_json(run / "reports" / "mcap_validation.json", validation)
    config = tmp_path / "rerun.yaml"
    config.write_text(yaml.safe_dump({
        "rerun_bridge": {"ros__parameters": {
            "image_max_hz": 2.0, "pointcloud_max_hz": 1.0,
            "signal_max_hz": 2.0, "max_point_count": 100000,
            "point_stride": 1,
        }},
        "rrd_builder": {
            "frozen_rerun_sdk_version": "0.36.2", "min_free_bytes": 1,
            "source_size_headroom_ratio": 0, "max_parse_failure_ratio": 0.01,
        },
    }), encoding="utf-8")
    return run, config


def test_fixture_e2e_atomic_manifest_and_sha(tmp_path: Path):
    run, config = create_fixture_run(tmp_path)
    output = run / "rerun" / f"{run.name}.rrd"
    builder = RunBuilder(BuildOptions(run, config, output))
    assert builder.run() == 0
    assert output.is_file() and output.stat().st_size > 0
    assert not output.with_suffix(".rrd.partial").exists()
    manifest = json.loads((output.parent / "rrd_manifest.json").read_text())
    validation = json.loads((output.parent / "rrd_validation.json").read_text())
    assert manifest["complete"] is True
    assert manifest["valid_for_formal_validation"] is True
    assert manifest["topic_statistics"]["/field/collection_status"]["source_count"] == 3
    assert manifest["topic_statistics"]["/field/collection_status"]["logged_count"] == 3
    assert validation["success"] is True and validation["atomic_commit"] is True
    line = output.with_suffix(".rrd.sha256").read_text().strip()
    assert line == f"{sha256_file(output)}  {output.name}"


def test_mid_conversion_failure_does_not_commit(monkeypatch, tmp_path: Path):
    import inspection_rerun.run_rrd_builder as module

    run, config = create_fixture_run(tmp_path)
    output = run / "rerun" / f"{run.name}.rrd"
    monkeypatch.setattr(module, "verify_rrd", lambda *_args: (_ for _ in ()).throw(BuildError("boom")))
    assert RunBuilder(BuildOptions(run, config, output)).run() == 1
    assert not output.exists()
    manifest = json.loads((output.parent / "rrd_manifest.json").read_text())
    assert manifest["complete"] is False


def test_entity_sanitization_and_sha256(tmp_path: Path):
    path = tmp_path / "bytes"
    path.write_bytes(b"abc")
    assert sanitize_entity("/a topic/b") == "/ros/a_topic/b"
    assert sha256_file(path) == hashlib.sha256(b"abc").hexdigest()

@pytest.mark.parametrize("damage", ["zero", "tamper", "missing", "escape"])
def test_strict_checks_actual_selected_source_bytes(tmp_path, damage):
    run = tmp_path / "RUN-HASH"
    source = minimal_source(run)
    manifest, complete, validation = gate_documents(run, source)
    victim = source.path / "core_0.mcap"
    if damage == "zero":
        manifest["files"][1]["sha256"] = "0" * 64
    elif damage == "tamper":
        victim.write_bytes(b"changed after historical validation")
    elif damage == "missing":
        victim.unlink()
    else:
        victim.unlink()
        external = tmp_path / "outside.mcap"
        external.write_bytes(b"mcap")
        victim.symlink_to(external)
    with pytest.raises(BuildError, match="source_(hash|files|path)"):
        quality_gates(BuildOptions(run, tmp_path / "c", run / "rerun/x.rrd"),
                      manifest, complete, validation, [source], [], {}, "0.36.2")


@pytest.mark.parametrize("tamper", [False, True])
def test_atomic_rrd_and_recheck_with_injected_cdr_reader(monkeypatch, tmp_path, tamper):
    """Actual Rerun writer/CLI and ROS CDR; reader injection is not MCAP validation."""
    import base64
    import inspection_rerun.run_rrd_builder as module
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    run=tmp_path / 'RRD-CDR-FIXTURE'
    source=minimal_source(run, start=1000000000, end=1200000000)
    source.topics={'/field/collection_status':'std_msgs/msg/String'}
    source.topic_counts={'/field/collection_status':3}
    source.topic_ranges={'/field/collection_status':(1000000000,1200000000)}
    fixture=source.path/'core_0.mcap'
    # This file is a test-only CDR journal with an injected reader, deliberately
    # not represented as a working MCAP plugin result in the refactor report.
    payloads=[serialize_message(String(data='fixture-'+str(i))) for i in range(3)]
    fixture.write_bytes(b''.join(payloads))
    manifest,complete,validation=gate_documents(run,source)
    write_json(run/'run_manifest.json',manifest)
    write_json(run/'complete.json',complete)
    write_json(run/'reports/mcap_validation.json',validation)
    config=tmp_path/'rrd_config.yaml'
    config.write_text(yaml.safe_dump({'rerun_bridge':{'ros__parameters':{}},'run_rrd_builder':{
        'required_topics':['/field/collection_status'],'min_free_bytes':1}}))
    monkeypatch.setattr(module,'discover_bag_sources',lambda *args,**kwargs:[source])
    class Reader:
        def __init__(self, selected_source, selected_topics):
            self.index=0
            self.next_item=(1000000000,'/field/collection_status',payloads[0],'std_msgs/msg/String')
        def advance(self):
            self.index+=1
            self.next_item=(1000000000+self.index*100000000,'/field/collection_status',payloads[self.index],'std_msgs/msg/String') if self.index<3 else None
            if tamper and self.index==2:fixture.write_bytes(b'changed during conversion')
    monkeypatch.setattr(module,'ReaderStream',Reader)
    output=run/'rerun/fixture.rrd'
    builder=RunBuilder(BuildOptions(run,config,output))
    rc=builder.run()
    document=json.loads((output.parent/'rrd_manifest.json').read_text())
    derived=json.loads((output.parent/'result_manifest.json').read_text())
    assert derived['source_run_id']==run.name
    assert derived['complete']==document['complete']
    assert derived['hash_verified']==document['hash_verified']
    if tamper:
        assert rc==1 and not output.exists()
        assert output.with_suffix('.rrd.partial').is_file()
        assert document['complete'] is False and document['hash_verified'] is False
    else:
        assert rc==0 and output.stat().st_size>0
        assert not output.with_suffix('.rrd.partial').exists()
        assert document['hash_verified'] is True
        assert document['rrd_sha256']==digest(output)
        assert document['topic_statistics']['/field/collection_status']['received_count']==3
        assert (output.parent/'rrd_validation.json').is_file()


def test_strict_rejects_missing_frozen_interface(tmp_path):
    run=tmp_path/'RUN-MISSING-INTERFACE';run.mkdir()
    source=minimal_source(run)
    manifest,complete,validation=gate_documents(run,source)
    manifest['files'].append({'path':'frozen_configs/EddyFrame.msg','sha256':'1'*64})
    with pytest.raises(BuildError,match='source_files'):
        quality_gates(BuildOptions(run,tmp_path/'config',run/'rerun/out.rrd'),manifest,complete,
                      validation,[source],[],{},'0.36.2')
