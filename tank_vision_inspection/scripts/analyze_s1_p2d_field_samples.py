#!/usr/bin/env python3
"""Replay saved field samples and compare them with a supplied manual annotation."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import statistics
import sys
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tank_vision.seam_width_reference import SeamWidthReference
from tank_vision.stable_seam_tracker import StableSeamTracker, draw_stable_seam_overlay


def _points(annotation: dict[str, Any], key: str) -> tuple[np.ndarray, np.ndarray]:
    points = sorted(annotation[key], key=lambda item: item["y"])
    return (np.asarray([float(item["y"]) for item in points]),
            np.asarray([float(item["x"]) for item in points]))


def _stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"count": 0, "p50": None, "p90": None, "p95": None, "max": None}
    absolute = np.abs(np.asarray(values, dtype=np.float64))
    return {"count": len(values), "p50": float(np.percentile(absolute, 50)),
            "p90": float(np.percentile(absolute, 90)), "p95": float(np.percentile(absolute, 95)),
            "max": float(np.max(absolute))}


def _dist(values: list[float]) -> dict[str, float | None]:
    return {"count": len(values), "mean": float(statistics.fmean(values)) if values else None,
            "p50": float(np.percentile(values, 50)) if values else None,
            "p95": float(np.percentile(values, 95)) if values else None,
            "max": float(max(values)) if values else None}


def _manual_at(ys: np.ndarray, xs: np.ndarray, y: float) -> float:
    return float(np.interp(y, ys, xs))


def analyze(collection: Path, annotation_path: Path, config: Path, reference: Path, output: Path,
            disable_guided: bool = False) -> dict[str, Any]:
    annotation = json.loads(annotation_path.read_text(encoding="utf-8"))
    left_y, left_x = _points(annotation, "left_boundary_manual_points")
    right_y, right_x = _points(annotation, "right_boundary_manual_points")
    image_paths = sorted(collection.glob("frame_*.bmp"))
    if not image_paths:
        raise ValueError(f"no saved BMP frames found under {collection}")
    output.mkdir(parents=True, exist_ok=True)
    reference_model = SeamWidthReference.load(reference)
    common_y0, common_y1 = max(left_y[0], right_y[0]), min(left_y[-1], right_y[-1])

    reference_errors = {key: [] for key in ("left", "right", "center", "width")}
    reference_ys = np.linspace(common_y0, common_y1, 256)
    manual_left = np.interp(reference_ys, left_y, left_x)
    manual_right = np.interp(reference_ys, right_y, right_x)
    expected_center, expected_width = reference_model.at(reference_ys, 4096, 3072)
    for key, values in (
        ("left", expected_center - expected_width * 0.5 - manual_left),
        ("right", expected_center + expected_width * 0.5 - manual_right),
        ("center", expected_center - 0.5 * (manual_left + manual_right)),
        ("width", expected_width - (manual_right - manual_left)),
    ):
        reference_errors[key].extend(float(value) for value in values)

    # Use the explicitly supplied reference for both metric calculation and
    # runtime candidate guidance.  This keeps field replay experiments
    # isolated from the reference path embedded in a production config.
    tracker_config: Path | dict[str, Any] = config
    if disable_guided:
        tracker_config = yaml.safe_load(config.read_text(encoding="utf-8"))
        tracker_config["guided_search"] = {"enabled": False}
    tracker = StableSeamTracker(tracker_config, reference=reference_model)
    frames: list[dict[str, Any]] = []
    all_errors = {key: [] for key in ("left", "right", "center", "width")}
    panels: list[np.ndarray] = []
    for path in image_paths:
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"cannot read {path}")
        result = tracker.process(image)
        frame_errors = {key: [] for key in all_errors}
        for (lx, ly), (rx, _ry), (cx, _cy) in zip(
                result.left_boundary_original_px, result.right_boundary_original_px, result.centerline_original_px):
            if not (common_y0 <= ly <= common_y1):
                continue
            ml, mr = _manual_at(left_y, left_x, ly), _manual_at(right_y, right_x, ly)
            frame_errors["left"].append(lx - ml)
            frame_errors["right"].append(rx - mr)
            frame_errors["center"].append(cx - 0.5 * (ml + mr))
            frame_errors["width"].append((rx - lx) - (mr - ml))
        for key in all_errors:
            all_errors[key].extend(frame_errors[key])
        frames.append({
            "input": path.name,
            "detected": result.detected,
            "predicted_only": result.predicted_only,
            "state": result.tracking_state.name,
            "coverage": result.coverage,
            "fit_inlier_ratio": result.fit_inlier_ratio,
            "fit_residual_px": result.fit_residual_px,
            "processing_ms": result.processing_ms,
            "abrupt_left_jump_count": result.abrupt_left_jump_count,
            "abrupt_right_jump_count": result.abrupt_right_jump_count,
            "failure_reasons": result.failure_reasons,
            "manual_errors_px": {key: _stats(value) for key, value in frame_errors.items()},
        })
        if len(panels) < 6:
            panels.append(cv2.resize(draw_stable_seam_overlay(image, result), (512, 384)))

    if panels:
        rows = [np.hstack(panels[index:index + 3]) for index in range(0, len(panels), 3)]
        cv2.imwrite(str(output / "tracker_replay_montage.jpg"), np.vstack(rows))
    report = {
        "collection": str(collection),
        "annotation": str(annotation_path),
        "annotation_sha256": hashlib.sha256(annotation_path.read_bytes()).hexdigest(),
        "config": str(config),
        "reference": str(reference),
        "manual_common_y_range_original_px": [float(common_y0), float(common_y1)],
        "saved_frame_count": len(image_paths),
        "frozen_reference_vs_current_manual_errors_px": {key: _stats(value) for key, value in reference_errors.items()},
        "replay": {
            "detected_count": sum(item["detected"] for item in frames),
            "detected_rate": sum(item["detected"] for item in frames) / len(frames),
            "state_counts": {state: sum(item["state"] == state for item in frames)
                             for state in sorted({item["state"] for item in frames})},
            "processing_ms": _dist([item["processing_ms"] for item in frames]),
            "coverage": _dist([item["coverage"] for item in frames]),
            "fit_inlier_ratio": _dist([item["fit_inlier_ratio"] for item in frames]),
            "fit_residual_px": _dist([item["fit_residual_px"] for item in frames if item["fit_residual_px"] is not None]),
            "abrupt_jump_total": sum(item["abrupt_left_jump_count"] + item["abrupt_right_jump_count"] for item in frames),
            "selected_boundary_errors_px": {key: _stats(value) for key, value in all_errors.items()},
            "frames": frames,
        },
        "precision_claimed": True,
    }
    (output / "field_manual_comparison.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection", type=Path, required=True)
    parser.add_argument("--annotation", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=Path("configs/s100p_stable_seam_4096x3072.yaml"))
    parser.add_argument("--reference", type=Path, default=Path("configs/seam_width_reference_s100p_v1.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--disable-guided", action="store_true",
                        help="disable guided candidate windows while keeping the supplied reference")
    args = parser.parse_args()
    report = analyze(args.collection, args.annotation, args.config, args.reference, args.output,
                     disable_guided=args.disable_guided)
    print(json.dumps({"saved_frame_count": report["saved_frame_count"],
                      "reference_vs_manual": report["frozen_reference_vs_current_manual_errors_px"],
                      "replay": {key: value for key, value in report["replay"].items() if key != "frames"}}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
