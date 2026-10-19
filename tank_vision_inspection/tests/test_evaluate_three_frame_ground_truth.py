"""专项测试：S1-P2H three-frame gate truth evaluation."""

from __future__ import annotations

import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/s1_p2h_three_frame_ground_truth_evaluation"


def load(name: str):
    return json.loads((OUT / name).read_text(encoding="utf-8"))


def test_annotation_manifest_is_frozen_and_valid() -> None:
    manifest = load("annotation_manifest.json")
    assert manifest["all_valid"] is True
    assert set(manifest["cases"]) == {"capture_01", "capture_03", "capture_05"}
    for item in manifest["cases"].values():
        assert item["valid"] is True
        assert "stable_seam_failure_snapshots" in item["raw"]
        assert all(item["checks"].values())


def test_gate_truth_and_warm_capture_three_are_recorded() -> None:
    gate = load("gate_truth_table.json")
    assert set(gate) == {"capture_01", "capture_03", "capture_05"}
    assert gate["capture_01"]["failed_items"] == ["inlier_ratio_below_minimum", "fit_residual_above_maximum"]
    assert gate["capture_05"]["observed"]["joint_inlier_ratio"] < 0.68
    assert gate["capture_05"]["observed"]["fit_residual_px"] > 14.0
    stages = load("stagewise_accuracy.json")
    warm = stages["capture_03"]["field_recorded_geometry_extra"]["accuracy"]
    assert warm["center"]["absolute"]["p95"] <= 25.0
    assert warm["left"]["absolute"]["p95"] <= 30.0
    assert warm["right"]["absolute"]["p95"] <= 30.0
    assert stages["capture_03"]["irls_fitted"]["accuracy"]["left"]["absolute"]["p95"] > 30.0


def test_decision_and_scope_outputs() -> None:
    summary = load("diagnostic_summary.json")
    assert summary["decision"] == "B"
    assert summary["all_annotation_valid"] is True
    assert summary["normalized_step_disabled"] is True
    assert summary["center_transition_weight"] == 0.42
    assert summary["production_changes"] == {
        "algorithm": False, "gate": False, "reference": False,
        "configuration": False, "ROS": False,
    }
    image = cv2.imread(str(OUT / "comparison_montage.jpg"), cv2.IMREAD_COLOR)
    assert image is not None and image.size > 0
