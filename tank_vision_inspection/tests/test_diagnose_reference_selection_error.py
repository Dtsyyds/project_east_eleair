"""专项测试：S1-P2E3 reference/selection error decomposition."""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.diagnose_reference_selection_error import OUT  # noqa: E402


def test_baseline_and_required_artifacts_are_complete() -> None:
    baseline = json.loads((OUT / "baseline_reproduction.json").read_text(encoding="utf-8"))
    assert baseline["baseline"]["status"] == "matched"
    required = ("reference_only_accuracy.json", "per_band_error_decomposition.json",
                "per_band_error_decomposition.csv", "reference_vs_selection_correlation.json",
                "y_2400_2700_diagnostic.json", "candidate_profile_key_bands.json",
                "oracle_pair_comparison.json", "failure_classification.json",
                "reference_manual_selected_overlay.png", "y_2400_2700_profile_montage.png",
                "diagnostic_summary.json", "next_single_change.json")
    assert all((OUT / name).is_file() for name in required)


def test_decomposition_is_exact_and_covers_117_bands() -> None:
    report = json.loads((OUT / "per_band_error_decomposition.json").read_text(encoding="utf-8"))
    assert report["record_count"] == 117
    assert report["identity_check"]["max_abs_residual"] < 1e-9
    assert len(report["records"]) == 117


def test_reference_representation_and_oracles_are_reported() -> None:
    reference = json.loads((OUT / "reference_only_accuracy.json").read_text(encoding="utf-8"))
    oracle = json.loads((OUT / "oracle_pair_comparison.json").read_text(encoding="utf-8"))
    assert reference["representation"]["center_degree"] == 2
    assert reference["representation"]["width_degree"] == 1
    assert oracle["counts"]["selected"] == 116
    assert oracle["counts"]["manual_nearest"] == 116


def test_frozen_inputs_unchanged_and_no_production_change_claimed() -> None:
    summary = json.loads((OUT / "diagnostic_summary.json").read_text(encoding="utf-8"))
    assert summary["fixed_input_hashes_unchanged"] is True
    assert all(value is False for value in summary["production_changes"].values())
