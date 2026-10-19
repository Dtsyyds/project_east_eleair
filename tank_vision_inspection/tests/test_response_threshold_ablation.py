from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from tank_vision.stable_seam_tracker import scale_effective_response_threshold


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/evaluate_response_threshold_ablation.py"


def load_ablation_module():
    spec = importlib.util.spec_from_file_location("response_threshold_ablation_test_module", SCRIPT)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_scale_is_single_multiplication_and_baseline_identity():
    thresholds = np.asarray([0.8, 1.25, 7.5], dtype=np.float64)
    np.testing.assert_array_equal(scale_effective_response_threshold(thresholds, 1.0), thresholds)
    np.testing.assert_array_equal(scale_effective_response_threshold(thresholds, 0.8), thresholds * 0.8)


@pytest.mark.parametrize("scale", [0.59, 1.01, 0.0, 2.0])
def test_scale_range_is_rejected(scale):
    with pytest.raises(ValueError):
        scale_effective_response_threshold(np.asarray([1.0]), scale)


def test_allowed_scale_endpoints():
    np.testing.assert_array_equal(scale_effective_response_threshold(np.asarray([1.0]), 0.6), [0.6])
    np.testing.assert_array_equal(scale_effective_response_threshold(np.asarray([1.0]), 1.0), [1.0])


def test_first_loss_categories_are_mutually_exclusive():
    module = load_ablation_module()
    stage = {
        "raw_local_extrema": [{"x": 10, "signed_response": 1.0}],
        "thresholded_extrema": [{"x": 10, "signed_response": 1.0}],
        "nms_survivor": [{"x": 10, "signed_response": 1.0}],
        "side_topk_survivor": [{"x": 10, "signed_response": 1.0}],
        "final_candidate": [{"x": 10, "signed_response": 1.0}],
    }
    assert module.first_loss(stage, 10.0, 20.0, {}) == "candidate_survived"
    assert module.first_loss({**stage, "thresholded_extrema": []}, 10.0, 20.0, {}) == "response_below_threshold"
    assert module.first_loss({**stage, "raw_local_extrema": []}, 10.0, 20.0, {}) == "local_extrema_not_detected"


def test_required_scale_set_and_independent_output():
    module = load_ablation_module()
    assert module.SCALES == (1.0, 0.9, 0.8, 0.7, 0.6)
    assert "field_w2_a01_response_threshold_ablation" in str(module.ROOT / "output/field_w2_a01_response_threshold_ablation")


def test_input_sha_is_stable():
    module = load_ablation_module()
    path = module.ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
    assert module.digest(path) == module.digest(path)


def test_persisted_baseline_funnel_counts_are_exact():
    output = ROOT / "output/field_w2_a01_response_threshold_ablation/scale_comparison.json"
    assert output.is_file()
    import json
    data = json.loads(output.read_text(encoding="utf-8"))
    baseline = next(item for item in data["aggregates"] if item["scale"] == 1.0)
    assert baseline["side_records"] == 702
    assert baseline["first_loss_20px"] == {
        "candidate_survived": 297,
        "response_below_threshold": 329,
        "removed_by_side_topk": 60,
        "anchor_merge_not_retained": 16,
    }
    assert baseline["pair_counts"] == {
        "one_side_candidate_missing": 204,
        "entered_dp": 146,
        "removed_by_pair_topk": 1,
    }
