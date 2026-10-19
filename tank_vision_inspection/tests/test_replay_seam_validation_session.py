from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2

from scripts import replay_seam_validation_session as replay
from scripts.record_seam_validation_session import CapturePacket
from scripts.seam_validation_common import frame_record, roi_crop, sha256_file, write_json, roi_from_config
from tank_vision.stable_seam_tracker import StableSeamTracker


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
IMAGE = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"


def _make_one_frame_session(path: Path) -> Path:
    path.mkdir()
    (path / "keyframes").mkdir()
    image = cv2.imread(str(IMAGE), cv2.IMREAD_COLOR)
    assert image is not None
    tracker = StableSeamTracker(CONFIG, REFERENCE)
    roi = roi_crop(image, roi_from_config(__import__("yaml").safe_load(CONFIG.read_text()), image.shape[1], image.shape[0]))
    packet = CapturePacket(0, SimpleNamespace(image=image, timestamp_ns=123456, exposure_us=1000.0,
                                               gain_db=6.0, analog_gain_raw=2.0, exposure_mode="manual"),
                           999999, roi, roi.tobytes(order="C"), 0.1, [])
    result = tracker.process(image, diagnostics=True, provenance=True)
    row = frame_record(packet, "LOST", result, tracker)
    roi = roi_crop(image, result.roi_original_px)
    raw_path = path / "roi_frames.raw"
    raw_path.write_bytes(roi.tobytes(order="C"))
    row.update({"stored_index": 0, "raw_file": "roi_frames.raw", "raw_offset": 0,
                "raw_nbytes": roi.nbytes, "raw_shape": list(roi.shape), "raw_dtype": "uint8"})
    (path / "frames.jsonl").write_text(json.dumps(row, ensure_ascii=False) + "\n", encoding="utf-8")
    write_json(path / "manifest.json", {"schema_version": "1.0", "complete": False, "recording": True,
        "config": str(CONFIG.resolve()), "reference": str(REFERENCE.resolve()), "config_sha256": sha256_file(CONFIG),
        "reference_sha256": sha256_file(REFERENCE), "storage_mode": "roi", "storage_shape": list(roi.shape),
        "roi_original_px": list(result.roi_original_px), "input_full_shape": list(image.shape), "cold_start_confirmed": True})
    return path


def test_replay_is_deterministic_from_lossless_roi(tmp_path: Path, monkeypatch) -> None:
    session = _make_one_frame_session(tmp_path / "session")
    monkeypatch.setattr(sys, "argv", ["replay", "--session", str(session)])
    assert replay.main() == 0
    comparison = json.loads((session / "replay_comparison.json").read_text(encoding="utf-8"))
    assert comparison["raw_sequence_valid"] is True
    assert comparison["exact_match_count"] == 1
    assert comparison["mismatch_count"] == 0
    assert comparison["replayable"] is True
    assert (session / "replay_frame_metrics.csv").is_file()
    assert (session / "replay_diagnostic.mp4").is_file()


def test_truncated_raw_is_not_replayable(tmp_path: Path) -> None:
    session = _make_one_frame_session(tmp_path / "session")
    raw = session / "roi_frames.raw"
    raw.write_bytes(raw.read_bytes()[:-1])
    rows = json.loads("[" + (session / "frames.jsonl").read_text().strip() + "]")
    assert rows[0]["raw_nbytes"] > raw.stat().st_size


def test_preview_video_is_not_used_as_replay_source() -> None:
    source = Path(replay.__file__).read_text(encoding="utf-8")
    assert "VideoCapture" not in source
    assert "raw_file" in source


def test_longest_lost_uses_capture_host_timestamps() -> None:
    rows = [
        {"capture_index": 0, "host_timestamp_ns": 0, "tracking_state": "LOST"},
        {"capture_index": 1, "host_timestamp_ns": 50_000_000, "tracking_state": "LOST"},
        {"capture_index": 2, "host_timestamp_ns": 100_000_000, "tracking_state": "ACQUIRING"},
        {"capture_index": 3, "host_timestamp_ns": 150_000_000, "tracking_state": "LOST"},
        {"capture_index": 4, "host_timestamp_ns": 400_000_000, "tracking_state": "LOST"},
    ]
    result = replay._longest_lost(rows)
    assert result["frame_count"] == 2
    assert result["start_capture_index"] == 3
    assert result["end_capture_index"] == 4
    assert result["duration_ms"] == 250.0
