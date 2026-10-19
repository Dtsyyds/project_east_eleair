from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).parents[1]
OUT = ROOT / "output/s1_p2c_live_bad_diagnostic"
FREEZE = ROOT / "output/s1_p2c_live_bad_ground_truth_v1"


def _sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def test_p2c_evidence_is_frozen_and_matches_input() -> None:
    manifest = json.loads((FREEZE / "ground_truth_manifest.json").read_text())
    assert manifest["status"] == "frozen"
    assert manifest["annotation_type"] == "manual"
    assert manifest["input_sha256_matches"] is True
    assert manifest["human_overlay_approved"] is True
    assert manifest["files"]["raw.bmp"]["sha256"] == "ada05bfa684cccfa87d54481985ca8614a9f3d9acd140e1af61f5aa80baccd02"
    assert _sha(FREEZE / "raw.bmp") == manifest["files"]["raw.bmp"]["sha256"]


def test_p2c_outputs_cover_all_required_diagnostic_artifacts() -> None:
    required = [
        "evidence_manifest.json", "source_call_graph.json", "baseline_reproduction.json",
        "reference_vs_manual_width.json", "scan_band_diagnostics.json", "scan_band_diagnostics.csv",
        "failure_classification.json", "dp_transition_trace.json", "fit_inlier_diagnostic.json",
        "manual_vs_candidates_overlay.png", "selected_prefit_path_overlay.png",
        "fit_inlier_outlier_overlay.png", "diagnostic_montage.jpg", "diagnostic_summary.json",
    ]
    assert all((OUT / name).is_file() for name in required)
    summary = json.loads((OUT / "diagnostic_summary.json").read_text())
    assert summary["status"] == "complete_read_only_diagnostic"
    assert summary["single_result"]["fit_inlier_ratio"] == 0.5527950310559007


def test_p2c_reproduction_has_no_temporal_lock() -> None:
    baseline = json.loads((OUT / "baseline_reproduction.json").read_text())
    assert baseline["single_frame"]["tracking_state"] == "LOST"
    assert all(frame["tracking_state"] == "LOST" for frame in baseline["repeated_frames"])
    assert all(frame["predicted_only"] is False for frame in baseline["repeated_frames"])
