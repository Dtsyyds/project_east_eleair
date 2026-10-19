from __future__ import annotations

import importlib.util
from pathlib import Path
import subprocess
import struct
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


def _load(name: str):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_mapping_profile_bandwidth_is_separate_from_raw_mid360():
    manager = _load("vehicle_collection_manager.py")
    args = SimpleNamespace(
        record_core=True, record_mapping=True, record_mid360=False,
        record_d405=False, record_eddy=True, record_front_video=True,
        record_front_raw_burst=False, raw_burst_mode="full",
        video_bitrate_mbps=16.0, storage_safety_factor=1.5,
    )
    plan = manager._planned_bandwidth(args)
    assert plan["components_MBps"]["mapping"] == 0.05
    assert plan["components_MBps"]["mid360"] == 0.0
    assert plan["components_MBps"]["d405"] == 0.0


def test_w6_eddy_group_includes_real_raw_frame_and_status():
    manager = (ROOT / "scripts" / "vehicle_collection_manager.py").read_text()
    startup = (ROOT / "scripts" / "start_vehicle_collection.sh").read_text()
    assert 'eddy["/eddy/data"]' in manager
    assert '"eddy_driver/msg/EddyFrame"' in manager
    assert '.record_groups.eddy["/eddy/data"] == "eddy_driver/msg/EddyFrame"' in startup


def test_runtime_monitor_uses_raw_callbacks_for_large_messages():
    monitor = (ROOT / "scripts" / "vehicle_collection_monitor.py").read_text()
    assert "def update_raw" in monitor
    assert "raw=True" in monitor
    assert '"nav_msgs/msg/Path"' in monitor


def test_vehicle_eddy_readiness_uses_topics_selected_by_monitor():
    startup = (ROOT / "scripts" / "start_vehicle_collection.sh").read_text()
    monitor = (ROOT / "scripts" / "vehicle_collection_monitor.py").read_text()
    assert 'raw_eddy_available = topics.get("/eddy/data")' in monitor
    assert 'type_name == "eddy_driver/msg/EddyChannelMean" and not raw_eddy_available' in monitor
    assert '.topics["/eddy/data"].count > 0 and .topics["/eddy/status"].count > 0' in startup
    assert '.topics[$topic].count > 0' not in startup


def test_w6_records_only_compact_odometry_but_audits_all_mapping_sources():
    manager = (ROOT / "scripts" / "vehicle_collection_manager.py").read_text()
    startup = (ROOT / "scripts" / "start_vehicle_collection.sh").read_text()
    assert '"storage_policy": "compact_odometry_only"' in manager
    assert '"/aft_mapped_to_init": topics["/aft_mapped_to_init"]' in manager
    assert '.record_groups.mapping == {"/aft_mapped_to_init":"nav_msgs/msg/Odometry"}' in startup
    assert '.mapping_sources["/cloud_registered"]' in startup
    assert '.mapping_sources["/path"]' in startup


def test_fast_cdr_header_stamp_parser_supports_little_endian():
    validator = _load("validate_vehicle_collection.py")
    payload = b"\x00\x01\x00\x00" + struct.pack("<iI", 123, 456)
    assert validator._cdr_header_stamp_ns(payload) == 123_000_000_456


def test_replay_and_validator_use_compact_mapping_topic():
    replay = (ROOT / "scripts" / "replay_vehicle_collection.py").read_text()
    validator = (ROOT / "scripts" / "validate_vehicle_collection.py").read_text()
    assert 'BagSource(mapping, {"/aft_mapped_to_init"})' in replay
    assert 'required = {"/aft_mapped_to_init"} if mapping_mode else set()' in replay
    assert '"compact_trajectory_rebuilt": trajectory_report["valid"]' in validator


def test_sensor_replay_publishes_one_lidar_and_one_g_unit_imu_topic():
    replay = _load("replay_vehicle_collection.py")
    assert replay.MID360_SENSOR_REPLAY_TOPICS == {
        "/livox/lidar", "/livox/imu_raw",
    }
    for mode in ("sensor-only", "full-sensor"):
        assert replay.map_topic(mode, "/livox/lidar") == "/livox/lidar_points"
        assert replay.map_topic(mode, "/livox/imu_raw") == "/livox/imu"


def test_video_mux_uses_observed_ros_timeline():
    recorder = (ROOT / "scripts" / "front_h264_recorder.py").read_text()
    assert 'relative_source_ns = stamp - self.first_stamp_ns' in recorder
    assert 'mux_fps = ((self.frames - 1) / source_span_sec' in recorder
    assert '"encoded_pts_ns": relative_source_ns' in recorder


def test_w6_start_enforces_attach_only_and_exclusions():
    text = (ROOT / "scripts" / "start_mapping_eddy_collection.sh").read_text()
    for value in (
        "collection_profile:=mapping_eddy", "record_mapping:=true",
        "record_mid360:=false", "record_d405:=false", "enable_tracker:=false",
        "provenance_enabled:=false", "record_front_raw_burst:=false",
    ):
        assert value in text


def test_replay_modes_are_exposed_by_shared_scheduler():
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts" / "replay_vehicle_collection.py"), "--help"],
        text=True, capture_output=True, check=False,
    )
    assert result.returncode == 0
    assert "mapping-only" in result.stdout
    assert "mapping-eddy" in result.stdout
    assert "full" in result.stdout


def test_mapping_probe_requires_exact_types():
    probe = _load("mapping_eddy_live_probe.py")
    assert probe.EXPECTED == {
        "/cloud_registered": "sensor_msgs/msg/PointCloud2",
        "/path": "nav_msgs/msg/Path",
        "/aft_mapped_to_init": "nav_msgs/msg/Odometry",
    }


def test_mapping_probe_waits_for_complete_graph():
    probe = _load("mapping_eddy_live_probe.py")
    one_endpoint = [object()]
    complete = {
        topic: ([expected], one_endpoint)
        for topic, expected in probe.EXPECTED.items()
    }
    assert probe._graph_ready(complete)
    assert probe._mapping_node_ready({
        topic: [expected] for topic, expected in probe.EXPECTED.items()
    })
    complete["/path"] = ([], [])
    assert not probe._graph_ready(complete)


def test_mapping_probe_cross_checks_anonymous_fastdds_endpoint():
    probe = _load("mapping_eddy_live_probe.py")
    item = {
        "publishers": [{"node_name": "_NODE_NAME_UNKNOWN_"}],
        "laser_mapping_node_declares_topic": True,
    }
    assert probe._publisher_identity_verified(item)
    item["laser_mapping_node_declares_topic"] = False
    assert not probe._publisher_identity_verified(item)
