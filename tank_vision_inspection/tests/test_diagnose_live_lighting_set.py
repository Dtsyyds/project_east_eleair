"""专项测试：S1-P2F1 live lighting diagnosis."""

from __future__ import annotations

import json
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "output/s1_p2f1_live_lighting_diagnostic"


def load(name: str):
    return json.loads((OUT / name).read_text(encoding="utf-8"))


def test_six_evidence_items_are_complete() -> None:
    inventory = load("evidence_inventory.json")
    assert len(inventory["items"]) == 6
    assert inventory["all_integrity_pass"] is True
    assert all(item["metadata"]["complete"] is True for item in inventory["items"])


def test_observed_and_offline_states_are_recorded() -> None:
    summary = load("diagnostic_summary.json")
    assert summary["observed_detected_count"] == 1
    assert summary["observed_lost_count"] == 5
    assert summary["all_coverage_one"] is True
    assert summary["choice"] in {"A", "B", "C", "D"}
    offline = load("offline_reproduction.json")
    assert len(offline["items"]) == 6
    replay = load("chronological_replay.json")
    assert len(replay["items"]) == 6


def test_quality_snr_telemetry_and_outputs_exist() -> None:
    quality = load("image_quality_comparison.json")
    assert len(quality) == 6
    for item in quality.values():
        assert "roi_sobel_abs_percentiles" in item
        assert "roi_region_gray_mean" in item
    snr = load("anchor_response_snr.json")
    assert len(snr["items"]) == 6
    telemetry = load("camera_telemetry_audit.json")
    assert telemetry["telemetry_missing"] is True
    for name in ("six_frame_montage.jpg", "illumination_heatmap_montage.jpg", "residual_by_y_montage.png"):
        image = cv2.imread(str(OUT / name), cv2.IMREAD_COLOR)
        assert image is not None and image.size > 0


def test_no_production_scope_changes_are_recorded() -> None:
    summary = load("diagnostic_summary.json")
    assert summary["production_changes"] == {
        "algorithm": False, "configuration": False, "reference": False,
        "gate": False, "ros": False,
    }
    assert (OUT / "next_single_action.json").is_file()
