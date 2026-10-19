"""专项测试：S1-P2E5 width-tail audit."""

from __future__ import annotations

import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/s1_p2e5_width_tail_diagnostic"


def load(name: str):
    return json.loads((OUT / name).read_text(encoding="utf-8"))


def test_p2e4_b_reproduction_and_layer_identity() -> None:
    baseline = load("baseline_reproduction.json")
    assert baseline["status"] == "matched"
    assert baseline["actual"]["path"] == 117
    assert baseline["actual"]["states"] == ["ACQUIRING", "ACQUIRING", "TRACKING"]
    stages = load("stagewise_accuracy.json")
    assert stages["p2e4_width_p95_layer"] == "raw_selected_pair"
    assert abs(stages["raw_selected_pair"]["width"]["absolute"]["p95"] - 32.532251387164905) < 1e-6
    assert stages["irls_fitted_model"]["width"]["absolute"]["p95"] < 30.0
    assert stages["temporal_model_after_three_repeats"]["width"]["absolute"]["p95"] < 30.0


def test_tail_counts_worst_bands_and_p95_sensitivity() -> None:
    tail = load("width_tail_distribution.json")
    assert tail["thresholds"]["over_30_px"]["count"] == 9
    assert tail["thresholds"]["over_40_px"]["count"] == 3
    assert tail["p95_sample"]["one_based_interpolation"].startswith("between sorted samples 111")
    worst = load("worst_width_bands.json")["bands"]
    assert len(worst) == 12
    assert worst[0]["band_index"] == 103
    summary = load("diagnostic_summary.json")
    sensitivity = summary["p95_sensitivity_correct_worst_k"]
    assert sensitivity["3"] < 30.0
    assert summary["choice"] == "D"


def test_key_bands_and_oracle_are_diagnostic_only() -> None:
    key = load("key_band_trace.json")["bands"]
    assert {item["band_index"] for item in key} == {82, 83, 103}
    band103 = next(item for item in key if item["band_index"] == 103)
    assert band103["anchor_pair"]["pair_id"] == "b103:l322:r505"
    assert band103["selected"]["pair_id"] == "b103:l351:r505"
    oracle = load("oracle_counterfactual.json")
    assert oracle["raw"]["anchor_anchor"]["definition"].find("diagnostic-only") >= 0
    assert oracle["raw"]["manual_nearest"]["pair_present_count"] == 117


def test_fixed_inputs_unchanged_and_outputs_exist() -> None:
    summary = load("diagnostic_summary.json")
    assert summary["fixed_input_hashes_unchanged"] is True
    assert summary["production_changes"] == {
        "algorithm": False, "parameters": False, "reference": False,
        "gate": False, "ros": False,
    }
    for name in ("width_tail_plot.png", "key_band_profile_montage.png"):
        image = cv2.imread(str(OUT / name), cv2.IMREAD_COLOR)
        assert image is not None and image.size > 0
    for name in (
        "baseline_reproduction.json", "stagewise_accuracy.json", "worst_width_bands.json",
        "width_tail_distribution.json", "width_failure_classification.json", "key_band_trace.json",
        "fit_residual_audit.json", "oracle_counterfactual.json", "diagnostic_summary.json",
        "next_single_change.json",
    ):
        assert (OUT / name).is_file()
