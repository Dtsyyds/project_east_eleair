"""专项测试：S1-P2E0 诊断只读且复用真实 tracker。"""

from __future__ import annotations

import sys
import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.diagnose_guided_pair_unary import (  # noqa: E402
    CONFIG,
    EXPECTED,
    LOFO_FEATURES,
    OUT,
    RAW,
    ANNOTATION,
    REFERENCE,
    sha256,
)
from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402
import yaml  # noqa: E402


def test_frozen_inputs_and_generated_baseline_are_verified() -> None:
    assert sha256(RAW) == EXPECTED["raw"]
    assert sha256(ANNOTATION) == EXPECTED["annotation"]
    assert sha256(CONFIG) == EXPECTED["config"]
    assert sha256(REFERENCE) == EXPECTED["reference"]
    report = json.loads((OUT / "baseline_reproduction.json").read_text(encoding="utf-8"))
    assert report["status"] == "matched"


def test_current_tracker_behavior_and_production_parameters_are_verified() -> None:
    """Verify behavior/parameters, without pinning an implementation SHA."""
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert config["path"]["center_transition_weight"] == 0.42
    assert config["geometry"]["minimum_inlier_ratio"] == 0.68
    assert config["geometry"]["maximum_fit_residual_px"] == 14.0
    assert config["pairing"]["max_pairs_per_band"] == 8
    assert config["guided_search"]["enabled"] is True

    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    assert image is not None
    tracker = StableSeamTracker(CONFIG, REFERENCE)
    results = [tracker.process(image, diagnostics=(index == 0)) for index in range(3)]
    assert len(tracker.last_diagnostics.selected_pairs) == 116
    assert results[0].coverage == 116 / 117
    assert [result.tracking_state.name for result in results] == ["ACQUIRING", "ACQUIRING", "TRACKING"]
    assert round(results[0].fit_inlier_ratio, 6) == round(0.7931034482758621, 6)


def test_leave_one_feature_out_artifact_covers_only_allowed_features() -> None:
    import json

    report = json.loads((OUT / "leave_one_feature_out.json").read_text(encoding="utf-8"))
    assert tuple(report["features"]) == LOFO_FEATURES
    assert "no candidate generation or DP rerun" in report["definition"]


def test_diagnostics_disabled_keeps_tracker_result_and_no_debug_state() -> None:
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    assert image is not None
    without = StableSeamTracker(CONFIG, REFERENCE)
    with_debug = StableSeamTracker(CONFIG, REFERENCE)
    result_without = without.process(image, diagnostics=False)
    result_with = with_debug.process(image, diagnostics=True)
    assert without.last_diagnostics is None
    assert with_debug.last_diagnostics is not None
    for field in ("detected", "tracking_state", "accepted_scan_bands", "rejected_scan_bands",
                  "coverage", "effective_span", "fit_inlier_ratio", "fit_residual_px",
                  "failure_reasons"):
        assert getattr(result_without, field) == getattr(result_with, field)
    assert result_without.roi_original_px == result_with.roi_original_px
    assert result_without.left_boundary_original_px == result_with.left_boundary_original_px
    assert result_without.right_boundary_original_px == result_with.right_boundary_original_px
