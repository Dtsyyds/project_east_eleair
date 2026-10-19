from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

import tank_vision.stable_seam_tracker as tracker_module
from scripts.build_seam_geometry_reference import build
from tank_vision.seam_tracking_types import TrackingState
from tank_vision.seam_width_reference import SeamWidthReference
from tank_vision.stable_seam_tracker import StableSeamTracker, _local_peaks_batch

ROOT = Path(__file__).parents[1]
GUIDED_CONFIG = ROOT / "configs/s100p_guided_seam_4096x3072.yaml"
LIVE_REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
LIVE_IMAGE = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"
LIVE_ANNOTATION = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json"


def test_live_reference_has_frozen_hashes_and_manual_polynomial() -> None:
    reference = json.loads(LIVE_REFERENCE.read_text(encoding="utf-8"))
    digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    assert reference["status"] == "frozen"
    assert reference["generated_from_manual_annotation"] is True
    assert reference["input_image_sha256"] == digest(LIVE_IMAGE)
    assert reference["annotation_sha256"] == digest(LIVE_ANNOTATION)
    assert len(reference["center_ref"]["coefficients"]) == 3
    assert len(reference["width_ref"]["coefficients"]) == 2
    assert reference["fit_source"] == "manual_annotation_only"


def test_reference_center_and_width_are_evaluated_per_y() -> None:
    reference = SeamWidthReference.load(LIVE_REFERENCE)
    raw = json.loads(LIVE_REFERENCE.read_text(encoding="utf-8"))
    ys = np.asarray([1600.0, 2100.0, 2700.0])
    centers, widths = reference.at(ys, 4096, 3072)
    yn = ys / 3072.0
    assert np.allclose(centers / 4096.0, np.polyval(raw["center_ref"]["coefficients"], yn))
    assert np.allclose(widths / 4096.0, np.polyval(raw["width_ref"]["coefficients"], yn))
    assert not np.isclose(centers[0], centers[-1])


def test_signed_gradient_peaks_do_not_mix_sides() -> None:
    values = np.zeros((1, 64), dtype=np.float32)
    values[0, 17] = 4.0
    values[0, 43] = -5.0
    thresholds = np.asarray([0.5])
    positive = _local_peaks_batch(values, True, thresholds, 3, 3, 0, 64)[0]
    negative = _local_peaks_batch(values, False, thresholds, 3, 3, 0, 64)[0]
    assert 17 in positive and 43 not in positive
    assert 43 in negative and 17 not in negative


def test_guided_local_windows_limit_each_side_to_three_and_preserve_coordinates() -> None:
    image = cv2.imread(str(LIVE_IMAGE), cv2.IMREAD_COLOR)
    tracker = StableSeamTracker(GUIDED_CONFIG)
    result = tracker.process(image, diagnostics=True)
    diagnostics = tracker.last_diagnostics
    assert diagnostics is not None
    assert all(len(items) <= 3 for items in diagnostics.left_candidates)
    assert all(len(items) <= 3 for items in diagnostics.right_candidates)
    assert all(item["left_mode"] in {"local", "global_fallback"} for item in diagnostics.candidate_search)
    assert result.roi_original_px == (1229, 1536, 1229, 1290)
    assert all(0 <= x < 4096 and 0 <= y < 3072 for x, y in result.centerline_original_px)
    assert all(left[0] < right[0] for left, right in zip(
        result.left_boundary_original_px, result.right_boundary_original_px))


def test_guided_empty_local_side_uses_only_that_sides_global_fallback(monkeypatch) -> None:
    config = json.loads(json.dumps(__import__("yaml").safe_load(GUIDED_CONFIG.read_text())))
    config["camera"]["same_fov_scaled"] = True
    reference = SeamWidthReference.load(LIVE_REFERENCE)
    original = tracker_module._local_peaks_batch

    def fake(values, positive, thresholds, top_k, separation, x0, x1):
        if np.asarray(x0).ndim:
            return [np.empty(0, dtype=np.int32) for _ in range(values.shape[0])]
        return original(values, positive, thresholds, top_k, separation, x0, x1)

    monkeypatch.setattr(tracker_module, "_local_peaks_batch", fake)
    tracker = StableSeamTracker(config, reference)
    image = cv2.imread(str(LIVE_IMAGE), cv2.IMREAD_COLOR)
    tracker.process(image, diagnostics=True)
    diagnostics = tracker.last_diagnostics
    assert diagnostics is not None
    assert all(item["left_mode"] == "global_fallback" for item in diagnostics.candidate_search)
    assert all(item["right_mode"] == "global_fallback" for item in diagnostics.candidate_search)


def test_guided_reaches_tracking_and_negative_stays_lost() -> None:
    image = cv2.imread(str(LIVE_IMAGE), cv2.IMREAD_COLOR)
    tracker = StableSeamTracker(GUIDED_CONFIG)
    states = [tracker.process(image).tracking_state for _ in range(3)]
    assert states == [TrackingState.ACQUIRING, TrackingState.ACQUIRING, TrackingState.TRACKING]
    negative = np.full_like(image, int(np.mean(image)))
    tracker.reset()
    result = tracker.process(negative)
    assert result.tracking_state == TrackingState.LOST and not result.detected


def test_reference_builder_does_not_use_tracker_output(tmp_path: Path) -> None:
    output = tmp_path / "reference.json"
    before_image = hashlib.sha256(LIVE_IMAGE.read_bytes()).hexdigest()
    before_annotation = hashlib.sha256(LIVE_ANNOTATION.read_bytes()).hexdigest()
    result = build(LIVE_ANNOTATION, LIVE_IMAGE, output, sample_count=16)
    assert result["generated_from_manual_annotation"] is True
    assert result["input_image_sha256"] == before_image
    assert result["annotation_sha256"] == before_annotation
    assert "tracker" not in json.dumps(result).lower()
    assert hashlib.sha256(LIVE_IMAGE.read_bytes()).hexdigest() == before_image
    assert hashlib.sha256(LIVE_ANNOTATION.read_bytes()).hexdigest() == before_annotation
