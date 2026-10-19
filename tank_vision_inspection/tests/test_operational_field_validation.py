from __future__ import annotations

from scripts.evaluate_operational_field_validation import evaluate


def _row(index: int, *, state: str = "TRACKING", detected: bool = True, predicted: bool = False,
         reasons: list[str] | None = None) -> dict[str, object]:
    return {
        "host_timestamp_ns": index * 125_000_000,
        "post_state": state,
        "detected": detected,
        "predicted_only": predicted,
        "failure_reasons": reasons or [],
        "coverage": 1.0,
        "effective_span": 1.0,
        "fit_residual_px": 10.0,
        "minimum_fitted_width_px": 190.0,
        "fitted_output_crossing_count": 0,
        "dropped_processed_frames": 0,
        "processing_ms": 50.0,
        "end_to_end_ms": 80.0,
    }


def test_operational_profile_passes_isolated_inlier_gate_failure():
    rows = [_row(index) for index in range(20)]
    rows[5] = _row(5, state="DEGRADED", detected=False, predicted=True,
                   reasons=["inlier_ratio_below_minimum"])
    report = evaluate(rows)
    assert report["status"] == "passed"
    assert report["observed"]["geometry_pass_rate"] == 1.0
    assert report["observed"]["maximum_consecutive_predicted_only"] == 1


def test_operational_profile_rejects_crossing():
    rows = [_row(index) for index in range(20)]
    rows[7]["fitted_output_crossing_count"] = 1
    report = evaluate(rows)
    assert report["status"] == "failed"
    assert report["checks"]["fitted_output_crossing_count"] is False
