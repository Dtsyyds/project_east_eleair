"""专项测试：S1-P2E1 provenance 默认关闭且不改变生产结果。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.diagnose_guided_dp_provenance import CONFIG, OUT, RAW, REFERENCE  # noqa: E402
from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402


def _core(result):
    value = result.to_dict()
    value.pop("processing_ms", None)
    value.pop("stage_timings_ms", None)
    return value


def test_trace_false_and_true_match_all_required_fields() -> None:
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    assert image is not None
    plain = StableSeamTracker(CONFIG, REFERENCE)
    traced = StableSeamTracker(CONFIG, REFERENCE)
    plain_result = plain.process(image, diagnostics=True, provenance=False)
    traced_result = traced.process(image, diagnostics=True, provenance=True)
    assert _core(plain_result) == _core(traced_result)
    assert plain.last_provenance is None
    assert traced.last_provenance is not None
    assert np.array_equal(plain.last_diagnostics.edge_response, traced.last_diagnostics.edge_response)
    assert np.array_equal(plain.last_diagnostics.scan_ys, traced.last_diagnostics.scan_ys)
    for left, right in zip(plain.last_diagnostics.left_candidates, traced.last_diagnostics.left_candidates):
        assert np.array_equal(left, right)
    for left, right in zip(plain.last_diagnostics.right_candidates, traced.last_diagnostics.right_candidates):
        assert np.array_equal(left, right)
    assert [[(p.left, p.right, p.score, p.score_parts) for p in row] for row in plain.last_diagnostics.all_pairs] == [[(p.left, p.right, p.score, p.score_parts) for p in row] for row in traced.last_diagnostics.all_pairs]
    assert [(p.band_index, p.left, p.right, p.score) for p in plain.last_diagnostics.selected_pairs] == [(p.band_index, p.left, p.right, p.score) for p in traced.last_diagnostics.selected_pairs]
    fit = traced.last_provenance["result"]
    assert fit["fit_center_coefficients"]
    assert fit["fit_width_coefficients"]


def test_tolerance_artifact_has_mutually_exclusive_117_band_counts() -> None:
    report = json.loads((OUT / "tolerance_reconciliation.json").read_text(encoding="utf-8"))
    assert report["counts_sum"] == {"15": 117, "25": 117, "40": 117}
    assert report["p2e0_reconciliation"]["manual_near_pair_final_list"] == 116


def test_pre_and_post_pair_trace_matches_production_pair_lists() -> None:
    trace = json.loads((OUT / "pair_pre_post_truncation.json").read_text(encoding="utf-8"))
    audit = json.loads((OUT / "candidate_stage_trace.json").read_text(encoding="utf-8"))
    assert trace["pre_truncation_observable"] is True
    assert len(trace["bands"]) == 117
    assert len(audit["bands"]) == 117
    assert all(row["pair_stages"]["post_truncation_count"] <= row["pair_stages"]["max_pairs_per_band"] for row in audit["bands"])
