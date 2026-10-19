from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from tank_vision.geometry_provenance_recorder import GeometryProvenanceRecorder, ProvenanceTask
from tank_vision.seam_tracking_types import StableSeamResult
from tank_vision.stable_seam_tracker import StableSeamTracker


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"


def test_default_tracker_trace_is_off_and_trace_has_geometry_evidence() -> None:
    image = np.zeros((3072, 4096, 3), dtype=np.uint8)
    plain = StableSeamTracker(CONFIG, REFERENCE)
    traced = StableSeamTracker(CONFIG, REFERENCE)
    plain_result = plain.process(image, diagnostics=False, provenance=False)
    traced_result = traced.process(image, diagnostics=True, provenance=True)
    assert plain.last_provenance is None
    assert plain_result.to_dict() | {"processing_ms": 0, "stage_timings_ms": {}} \
        == traced_result.to_dict() | {"processing_ms": 0, "stage_timings_ms": {}}
    assert traced.last_provenance is not None
    result = traced.last_provenance["result"]
    assert "geometry_gates" in result
    assert "temporal_input" in result and "temporal_output" in result
    assert "center_inlier_mask" in result and "width_inlier_mask" in result


def test_provenance_writer_is_bounded_and_metadata_is_last(tmp_path: Path) -> None:
    recorder = GeometryProvenanceRecorder(tmp_path, CONFIG, REFERENCE, queue_size=2,
                                           sample_period_sec=5.0, max_events=2)
    result = StableSeamResult(roi_original_px=(0, 0, 20, 20), fit_inlier_ratio=0.5, fit_residual_px=12.0)
    task = ProvenanceTask(
        roi_bgr=np.zeros((20, 20, 3), dtype=np.uint8), roi=(0, 0, 20, 20),
        metadata={"header_stamp_ns": 1, "captured_index": 1, "roi_original_px": [0, 0, 20, 20]},
        geometry={"detected": False},
        provenance={"scales": {"sx": 1.0, "sy": 1.0}, "selected_raw_path": [],
                    "result": {"geometry_gates": {}, "temporal_output": {}}},
        diagnostics=None, result=result, labels=["test"],
    )
    assert recorder.submit_periodic(task)
    recorder.commit_extreme(task, ["joint_inlier_min", "joint_inlier_max"])
    summary = recorder.close()
    assert summary["complete"] is True
    session = next(tmp_path.glob("*/complete.json")).parent
    events = list(session.glob("event_*/metadata.json"))
    assert len(events) == 1
    metadata = json.loads(events[0].read_text(encoding="utf-8"))
    assert metadata["complete"] is True
    assert "roi_raw.bmp" in metadata["files_sha256"]


def test_provenance_capacity_guard_stops_sampling_before_second_pass_work(tmp_path: Path) -> None:
    recorder = GeometryProvenanceRecorder(tmp_path, CONFIG, REFERENCE, queue_size=1,
                                           sample_period_sec=0.001, max_events=1)
    result = StableSeamResult(roi_original_px=(0, 0, 20, 20), fit_inlier_ratio=0.5, fit_residual_px=12.0)
    task = ProvenanceTask(
        roi_bgr=np.zeros((20, 20, 3), dtype=np.uint8), roi=(0, 0, 20, 20),
        metadata={"header_stamp_ns": 1, "captured_index": 1},
        geometry={"detected": False}, provenance={"result": {}}, diagnostics=None,
        result=result, labels=["test"],
    )
    assert recorder.has_capacity()
    assert recorder.submit_periodic(task)
    assert not recorder.has_capacity()
    assert not recorder.periodic_due()
    summary = recorder.close()
    assert summary["events_written"] == 1
    assert summary["capacity_suppressed_count"] == 0


def test_retained_extreme_also_reserves_the_event_budget(tmp_path: Path) -> None:
    recorder = GeometryProvenanceRecorder(tmp_path, CONFIG, REFERENCE, queue_size=1,
                                           sample_period_sec=0.001, max_events=1)
    result = StableSeamResult(roi_original_px=(0, 0, 20, 20), fit_inlier_ratio=0.5, fit_residual_px=12.0)
    task = ProvenanceTask(
        roi_bgr=np.zeros((20, 20, 3), dtype=np.uint8), roi=(0, 0, 20, 20),
        metadata={"header_stamp_ns": 2, "captured_index": 2},
        geometry={"detected": False}, provenance={"result": {}}, diagnostics=None,
        result=result, labels=["extreme"],
    )
    recorder.commit_extreme(task, ["joint_inlier_min"])
    assert not recorder.has_capacity()
    assert not recorder.periodic_due()
    summary = recorder.close()
    assert summary["events_written"] == 1
    assert summary["capacity_suppressed_count"] == 0
