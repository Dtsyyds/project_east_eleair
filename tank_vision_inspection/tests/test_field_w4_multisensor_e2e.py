import json
from pathlib import Path

from scripts.field_w4_multisensor_e2e import _diagnostics_audit, _phase_pass, _stats, checkpoint


def test_stats_and_percentiles_are_stable():
    result = _stats([1.0, 2.0, 3.0, 4.0, 5.0])
    assert result["count"] == 5
    assert result["p50"] == 3.0
    assert result["p95"] == 4.8
    assert result["max"] == 5.0


def test_lidar_only_phase_requires_real_lidar_and_mcap():
    probe = {
        "topics": {
            "/livox/lidar": {"header_rate_hz": 10.0, "count": 10, "non_monotonic_count": 0},
        }
    }
    bag = {"validated": True, "finalized": True}
    shutdown = {"pass": True}
    result = _phase_pass("w4a", probe, bag, shutdown)
    assert result["pass"] is True
    assert result["rates"]["lidar_hz"] == 10.0


def test_parallel_phase_rejects_uncalibrated_camera_info_and_slow_tracker():
    probe = {
        "topics": {
            "/camera/front/camera_info": {"header_rate_hz": 10.0, "count": 10, "camera_info_calibrated": False},
            "/vision/front/seam_geometry": {"count": 10},
            "/vision/front/seam_state": {"header_rate_hz": 5.5, "count": 10},
            "/diagnostics": {"count": 10},
            "/livox/lidar": {"header_rate_hz": 10.0, "count": 10, "non_monotonic_count": 0},
        },
        "slot_overwritten": 0,
    }
    result = _phase_pass("w4b", probe, {"validated": True, "finalized": True}, {"pass": True})
    assert result["checks"]["camera_info_calibrated"] is False
    assert result["checks"]["tracker_output_ge_6hz"] is False
    assert result["pass"] is False


def test_checkpoint_writes_required_state(tmp_path: Path):
    output = tmp_path / "w3_checkpoint.json"
    assert checkpoint(type("Args", (), {"output": str(output)})()) == 0
    report = json.loads(output.read_text())
    assert report["default_formal_profile"]["name"] == "P1"
    assert report["production_ready"] is False
    assert report["camera_info_calibration_complete"] is False


def test_diagnostics_audit_distinguishes_cross_publisher_interleaving():
    stats = {
        "/diagnostics": {
            "non_monotonic_count": 1,
            "arrival_non_monotonic_count": 0,
        }
    }
    history = [
        {"header_stamp_ns": 20, "receive_timestamp_ns": 100,
         "status_names": ["camera"], "hardware_ids": ["cam"]},
        {"header_stamp_ns": 10, "receive_timestamp_ns": 101,
         "status_names": ["livox"], "hardware_ids": ["lidar"]},
    ]
    audit = _diagnostics_audit(stats, history)
    assert audit["classification"] == "aggregate_multi_publisher_interleaving"
    assert audit["raw_validator_relevant"] is False


def test_diagnostics_audit_marks_receive_regression_as_invalid():
    stats = {
        "/diagnostics": {
            "non_monotonic_count": 0,
            "arrival_non_monotonic_count": 1,
        }
    }
    history = [
        {"header_stamp_ns": 10, "receive_timestamp_ns": 101,
         "status_names": ["camera"], "hardware_ids": ["cam"]},
        {"header_stamp_ns": 20, "receive_timestamp_ns": 100,
         "status_names": ["camera"], "hardware_ids": ["cam"]},
    ]
    audit = _diagnostics_audit(stats, history)
    assert audit["classification"] == "bag_receive_timestamp_regression"
    assert audit["raw_validator_relevant"] is True
