from __future__ import annotations

import json
import math

from scripts.validate_ros_seam_realtime import BoundedKeys, SeamRealtimeAccumulator


def _camera(accumulator: SeamRealtimeAccumulator, stamp: int, frame_id: str = "camera") -> None:
    accumulator.observe_camera_info(stamp, frame_id)


def _seam(
    accumulator: SeamRealtimeAccumulator,
    stamp: int,
    frame_id: str = "camera",
    detected: bool = True,
    confidence: float | None = 0.8,
    lateral: float | None = 2.0,
    heading: float | None = 1.0,
    receive_ns: int | None = None,
) -> None:
    accumulator.observe_seam(stamp, frame_id, detected, confidence, lateral, heading, receive_ns)


def test_cross_second_exact_match_and_output_age() -> None:
    stats = SeamRealtimeAccumulator("cross_second")
    _camera(stats, 1_999_999_999, "cam")
    _seam(stats, 1_999_999_999, "cam", receive_ns=2_000_099_999)
    report = stats.report(1.0)
    assert report["camera_stamp_monotonic"] is True
    assert report["seam_stamp_monotonic"] is True
    assert report["exact_header_match_count"] == 1
    assert report["exact_header_match_ratio"] == 1.0
    assert report["output_age_ms"]["available"] is True
    assert report["output_age_ms"]["mean"] == 0.1


def test_duplicate_non_monotonic_unmatched_and_detected_ratio() -> None:
    stats = SeamRealtimeAccumulator("association")
    _camera(stats, 3_000_000_000, "cam")
    _camera(stats, 2_999_000_000, "cam")
    _seam(stats, 3_000_000_000, "cam", detected=True)
    _seam(stats, 3_000_000_000, "cam", detected=False)
    _seam(stats, 4_000_000_000, "other", detected=False)
    report = stats.report(1.0)
    assert report["camera_stamp_monotonic"] is False
    assert report["seam_stamp_monotonic"] is False
    assert report["duplicate_seam_stamp_count"] == 1
    assert report["exact_header_match_count"] == 2
    assert report["unmatched_seam_count"] == 1
    assert report["seam_detected_count"] == 1
    assert report["seam_detected_ratio"] == 1 / 3


def test_none_nan_empty_topics_and_deterministic_json() -> None:
    stats = SeamRealtimeAccumulator("empty_and_invalid")
    _camera(stats, 5_000_000_000)
    _seam(stats, 5_000_000_000, confidence=math.nan, lateral=None, heading=math.nan)
    stats.observe_quality(False, True, (1.0, math.nan), ["bad_exposure"])
    report = stats.report(2.0)
    assert report["confidence"]["valid_count"] == 0
    assert report["lateral_error_px"]["valid_count"] == 0
    assert report["heading_error_deg"]["valid_count"] == 0
    assert report["invalid_quality_count"] == 1
    assert "quality:bad_exposure" in report["warnings"]
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
    assert encoded == json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
    empty = SeamRealtimeAccumulator("empty")
    empty_report = empty.report(1.0)
    assert empty_report["camera_info_count"] == 0
    assert empty_report["seam_count"] == 0
    assert "no_camera_info_messages:CameraInfo_is_only_lightweight_Header_evidence" in empty_report["warnings"]


def test_bounded_cache_eviction_is_reported() -> None:
    keys = BoundedKeys(2)
    assert keys.add((1, "a")) is True
    assert keys.add((2, "a")) is True
    assert keys.add((3, "a")) is True
    assert len(keys) == 2
    assert keys.evicted is True
    assert keys.contains((1, "a")) is False

    stats = SeamRealtimeAccumulator("bounded", max_cache_entries=2)
    _camera(stats, 1, "cam")
    _camera(stats, 2, "cam")
    _camera(stats, 3, "cam")
    _seam(stats, 1, "cam")
    report = stats.report(1.0)
    assert report["max_cache_entries"] == 2
    assert "camera_header_cache_evicted_association_is_bounded" in report["warnings"]
    assert "camera_info_count_is_not_Image_count" in report["warnings"]
