"""专项测试：S1-P2G1 normalized local step anchor A/B."""

from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.experiment_normalized_step_anchor import CONFIG, OUT, REFERENCE, RAW, config_for  # noqa: E402
from tank_vision.seam_tracking_types import TrackingState  # noqa: E402
from tank_vision.stable_seam_tracker import (  # noqa: E402
    StableSeamTracker,
    _find_normalized_step_anchor,
)


def test_step_formula_and_polarity() -> None:
    rising = np.full(128, 20.0, dtype=np.float32)
    rising[64:] = 80.0
    falling = np.full(128, 80.0, dtype=np.float32)
    falling[64:] = 20.0
    left = _find_normalized_step_anchor(rising, 45, 85, True, 100.0, 2)
    right = _find_normalized_step_anchor(falling, 45, 85, False, 100.0, 2)
    assert left["valid"] and right["valid"]
    assert left["step"] > 0 and right["step"] > 0
    assert left["support_width_processed"] == 8
    assert abs(left["score"] - left["step"] / (left["local_noise"] + 1.0)) < 1e-8
    assert abs(right["score"] - right["step"] / (right["local_noise"] + 1.0)) < 1e-8


def test_median_and_mad_resist_single_pixel_reflection() -> None:
    clean = np.full(128, 30.0, dtype=np.float32)
    clean[64:] = 90.0
    reflected = clean.copy()
    reflected[60] = 255.0
    reflected[68] = 0.0
    first = _find_normalized_step_anchor(clean, 48, 82, True, 100.0, 2)
    second = _find_normalized_step_anchor(reflected, 48, 82, True, 100.0, 2)
    assert first["valid"] and second["valid"]
    assert abs(first["position_processed"] - second["position_processed"]) <= 2
    assert abs(first["step"] - second["step"]) < 1e-6


def test_window_boundary_and_non_positive_step_are_rejected() -> None:
    profile = np.zeros(64, dtype=np.float32)
    invalid = _find_normalized_step_anchor(profile, 0, 4, True, 100.0, 4)
    no_step = _find_normalized_step_anchor(profile, 10, 30, True, 100.0, 2)
    assert not invalid["valid"] and invalid["reason"] == "invalid_window"
    assert not no_step["valid"] and no_step["reason"] == "non_positive_step"


def test_anchor_response_option_is_default_sobel_and_b_is_memory_only() -> None:
    original = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    a = config_for("A")
    b = config_for("B")
    assert a == original
    assert "reference_anchor_response" not in a["guided_search"]
    assert b["guided_search"]["reference_anchor_response"] == "normalized_step"
    assert yaml.safe_load(CONFIG.read_text(encoding="utf-8")) == original


def test_hard_negative_falls_back_without_detection() -> None:
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    assert image is not None
    negative = np.full_like(image, int(np.mean(image)))
    tracker = StableSeamTracker(config_for("B"), REFERENCE)
    result = tracker.process(negative, diagnostics=True)
    assert not result.detected and result.tracking_state == TrackingState.LOST
    assert tracker.last_diagnostics is not None
    assert all(not item.get("reference_anchor", {}).get("left", {}).get("step_anchor_used", False)
               for item in tracker.last_diagnostics.candidate_search)


def test_completed_ab_output_and_fixed_baseline() -> None:
    baseline = json.loads((OUT / "baseline_reproduction.json").read_text(encoding="utf-8"))
    assert baseline["status"] == "matched"
    summary = json.loads((OUT / "diagnostic_summary.json").read_text(encoding="utf-8"))
    assert summary["baseline_status"] == "matched"
    assert summary["fixed_input_hashes_unchanged"] is True
    assert summary["production_changes"] == {
        "reference": False, "roi": False, "pair": False, "dp": False,
        "gate": False, "temporal": False, "ros": False, "production_yaml": False,
    }
    negative = json.loads((OUT / "negative_regression.json").read_text(encoding="utf-8"))
    assert negative["A"]["hard_negative_pass"] and negative["B"]["hard_negative_pass"]
    for name in ("six_frame_ab_montage.jpg", "right_boundary_diagnostic.png", "benchmark.json"):
        assert (OUT / name).is_file()
