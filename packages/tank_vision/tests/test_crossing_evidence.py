from __future__ import annotations

from tank_vision.failure_event_recorder import gate_evidence
from tank_vision.seam_tracking_types import StableSeamResult


def test_crossing_is_reported_as_a_fast_path_gate():
    result = StableSeamResult(
        detected=True,
        fitted_output_crossing_count=1,
        minimum_fitted_width_px=180.0,
        fit_residual_px=4.0,
        median_width_px=180.0,
    )
    config = {
        "geometry": {
            "minimum_point_count": 1,
            "minimum_coverage": 0.0,
            "minimum_effective_span": 0.0,
            "maximum_missing_run": 3,
            "maximum_abrupt_jump_count": 3,
            "minimum_inlier_ratio": 0.0,
            "maximum_fit_residual_px": 14.0,
            "minimum_path_confidence": 0.0,
        }
    }
    evidence = gate_evidence(config, result)
    assert evidence["checks"]["non_crossing"]["observed"] == 1
    assert evidence["checks"]["non_crossing"]["pass"] is False
