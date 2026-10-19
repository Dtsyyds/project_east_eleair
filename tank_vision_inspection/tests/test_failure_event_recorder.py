from __future__ import annotations

import json
from pathlib import Path
import time

import numpy as np

from tank_vision.failure_event_recorder import (
    FailureEventTask,
    LiveSessionRecorder,
    gate_evidence,
    human_readable_reason,
    write_failure_event,
)
from tank_vision.seam_tracking_types import StableSeamResult, TrackingState


class _Stamp:
    sec = 10
    nanosec = 20


class _Header:
    stamp = _Stamp()
    frame_id = "test"


def _config() -> dict:
    return {"geometry": {"minimum_point_count": 20, "minimum_coverage": .6,
                          "minimum_effective_span": .65, "maximum_missing_run": 3,
                          "maximum_abrupt_jump_count": 3, "minimum_inlier_ratio": .68,
                          "maximum_fit_residual_px": 14.0, "minimum_path_confidence": .42}}


def _result() -> StableSeamResult:
    return StableSeamResult(
        detected=False, tracking_state=TrackingState.LOST, confidence=.2,
        roi_original_px=(1, 2, 20, 20), coverage=.9, effective_span=.8,
        accepted_scan_bands=32, maximum_missing_run=0, fit_inlier_ratio=.43,
        fit_residual_px=15.2, failure_reasons=["inlier_ratio_below_minimum", "fit_residual_above_maximum"],
    )


def test_gate_truth_and_human_reason_keep_observed_and_threshold() -> None:
    result = _result()
    gate = gate_evidence(_config(), result)
    assert gate["checks"]["joint_inlier"] == {"observed": .43, "threshold": .68, "pass": False}
    reason = human_readable_reason(_config(), result)
    assert "0.43 < 0.68" in reason
    assert "15.2 px > 14.0 px" in reason


def test_failure_event_writes_context_and_metadata_last(tmp_path: Path) -> None:
    config = tmp_path / "config.yaml"
    config.write_text(
        "geometry:\n  minimum_point_count: 20\n  minimum_coverage: 0.6\n"
        "  minimum_effective_span: 0.65\n  maximum_missing_run: 3\n"
        "  maximum_abrupt_jump_count: 3\n  minimum_inlier_ratio: 0.68\n"
        "  maximum_fit_residual_px: 14.0\n  minimum_path_confidence: 0.42\n"
    )
    reference = tmp_path / "reference.json"; reference.write_text("{}\n")
    result = _result()
    image = np.zeros((30, 30, 3), dtype=np.uint8)
    task = FailureEventTask(
        image=image, header=_Header(), result=result, geometry={"detected": False},
        gate=gate_evidence(_config(), result), metrics={"pre_state": "LOST", "exposure_us": None},
        trigger_reason="first_failure", automatic=True, source_mode="test",
        config_path=str(config), reference_path=str(reference), config_sha256=None,
        reference_sha256=None, save_full_frame=False, save_context_jpeg=True,
        context_jpeg_quality=90, save_lossless_roi=True, event_id="event_test",
    )
    directory = write_failure_event(task, tmp_path / "events")
    metadata = json.loads((directory / "metadata.json").read_text())
    assert metadata["complete"] is True
    assert (directory / "roi_raw.bmp").is_file()
    assert (directory / "context.jpg").is_file()
    assert (directory / "overlay.png").is_file()
    assert (directory / "failure_reason.json").is_file()


def test_max_snapshot_limit_rejects_before_image_factory(tmp_path: Path) -> None:
    recorder = LiveSessionRecorder(tmp_path, __file__, __file__, {}, max_snapshots=0)
    called = []
    accepted, event_id, reason = recorder.enqueue_snapshot(lambda _event: called.append(1) or None, automatic=True)
    assert not accepted and event_id is None and reason == "max_failure_snapshots_per_session"
    assert called == []
    recorder.finalize({}, graceful=True)


def test_metrics_and_events_are_bounded_writer_outputs(tmp_path: Path) -> None:
    recorder = LiveSessionRecorder(tmp_path, __file__, __file__, {}, snapshot_queue_size=1)
    assert recorder.submit_metrics({"frame_index": 1})
    assert recorder.submit_event_log({"event_id": "e1"})
    result = recorder.finalize({"captured_frames": 1}, graceful=True)
    assert result["complete"] is True
    assert (Path(result["session"]) / "frame_metrics.jsonl").read_text().strip()
    assert (Path(result["session"]) / "events.jsonl").read_text().strip()
