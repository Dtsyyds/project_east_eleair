from __future__ import annotations

import json
from queue import Full

import numpy as np

from tank_vision.roi_validation_recorder import ROIValidationRecorder


def _metadata() -> dict[str, object]:
    return {
        "header_stamp": {"sec": 1, "nanosec": 2},
        "captured_index": 4,
        "processed_index": 3,
        "config_sha256": "config",
        "reference_sha256": "reference",
        "state": "TRACKING",
        "detected": True,
        "predicted_only": False,
        "failure_reasons": [],
        "geometry": {"coverage": 1.0},
        "stage_timings_ms": {"dynamic_programming": 2.0},
        "crossing": {
            "selected_pair_crossing_count": 0,
            "fitted_output_crossing_count": 0,
            "minimum_fitted_width_px": 190.0,
        },
    }


def test_roi_recorder_writes_raw_and_complete_last(tmp_path):
    config = tmp_path / "config.yaml"
    reference = tmp_path / "reference.json"
    config.write_text("config\n")
    reference.write_text("reference\n")
    recorder = ROIValidationRecorder(tmp_path / "runs", config, reference)
    image = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    assert recorder.submit(image, (1, 1, 3, 2), _metadata())
    summary = recorder.close()

    session = recorder.session
    assert summary["complete"] is True
    assert (session / "roi_frames.raw").read_bytes() == image[1:3, 1:4].tobytes()
    row = json.loads((session / "frames.jsonl").read_text().strip())
    assert row["offset"] == 0
    assert row["length"] == 18
    assert row["shape"] == [2, 3, 3]
    assert row["dtype"] == "uint8"
    assert json.loads((session / "complete.json").read_text())["complete"] is True


def test_roi_recorder_queue_drop_marks_incomplete(tmp_path, monkeypatch):
    config = tmp_path / "config.yaml"
    reference = tmp_path / "reference.json"
    config.write_text("config\n")
    reference.write_text("reference\n")
    recorder = ROIValidationRecorder(tmp_path / "runs", config, reference)

    def reject(_task):
        raise Full

    monkeypatch.setattr(recorder.queue, "put_nowait", reject)
    image = np.zeros((2, 2, 3), dtype=np.uint8)
    assert recorder.submit(image, (0, 0, 2, 2), _metadata()) is False
    summary = recorder.close()
    assert summary["complete"] is False
    assert summary["counters"]["queue_dropped"] == 1


def test_roi_recorder_roi_change_marks_incomplete(tmp_path):
    config = tmp_path / "config.yaml"
    reference = tmp_path / "reference.json"
    config.write_text("config\n")
    reference.write_text("reference\n")
    recorder = ROIValidationRecorder(tmp_path / "runs", config, reference)
    image = np.zeros((4, 4, 3), dtype=np.uint8)
    assert recorder.submit(image, (0, 0, 2, 2), _metadata())
    assert recorder.submit(image, (0, 0, 3, 2), _metadata()) is False
    summary = recorder.close()
    assert summary["complete"] is False
    assert any("roi_changed" in value for value in summary["counters"]["write_errors"])
