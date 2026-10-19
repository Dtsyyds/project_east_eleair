#!/usr/bin/env python3
"""Thin S1-D2 batch wrapper around the unchanged S1-D1 experiment."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.experiment_real_seam_band import run_experiment
from tank_vision.config import load_config
from tank_vision.preprocessing.image_quality import assess_image_quality


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "output/s1_d2_four_position_validation"
DEFAULT_CONFIG = ROOT / "configs/default.yaml"
CASES = {
    "center": ROOT / "output/s1_real_seam_images/seam_center_full_fov_01.bmp",
    "left": ROOT / "output/s1_real_seam_images/seam_left_full_fov_01.bmp",
    "right": ROOT / "output/s1_real_seam_images/seam_right_full_fov_01.bmp",
    "tilt": ROOT / "output/s1_real_seam_images/seam_tilt_full_fov_01.bmp",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _preflight(path: Path, config_path: Path) -> dict[str, Any]:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"OpenCV could not read {path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    quality = assess_image_quality(image, load_config(config_path).quality)
    return {
        "input_path": str(path),
        "sha256_before": _sha256(path),
        "readable_by_opencv": True,
        "width": int(image.shape[1]),
        "height": int(image.shape[0]),
        "channels": int(image.shape[2]) if image.ndim == 3 else 1,
        "dtype": str(image.dtype),
        "mean_brightness": float(np.mean(gray)),
        "contrast_std": float(np.std(gray)),
        "laplacian_variance": float(quality.laplacian_variance),
        "underexposed_ratio": float(quality.underexposed_ratio),
        "overexposed_ratio": float(quality.overexposed_ratio),
        "quality_warnings": list(quality.warnings),
    }


def _summary_row(case: str, result: dict[str, Any], preflight: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "geometry_candidate_valid", "failure_reason", "image_quality_valid",
        "valid_left_boundary_point_count", "valid_right_boundary_point_count",
        "valid_center_point_count", "vertical_coverage_ratio", "weld_width_px_median",
        "weld_width_px_p05", "weld_width_px_p95", "centerline_fit_residual_px",
        "centerline_inlier_ratio", "lateral_error_px", "heading_error_deg",
        "centerline_between_boundaries", "restored_coordinates_valid", "processing_time_ms",
    )
    row = {"case": case, "input_path": result["input_path"], **{key: result.get(key) for key in keys}}
    row.update({"sha256_before": preflight["sha256_before"], "sha256_after": result["input_sha256_after"],
                "input_unchanged": result["input_unchanged"], "production_valid": result["production_valid"]})
    return row


def _annotated_tile(image: np.ndarray, case: str, row: dict[str, Any]) -> np.ndarray:
    tile = cv2.resize(image, (1024, 768), interpolation=cv2.INTER_AREA)
    lines = [
        case,
        f"geometry_candidate_valid={row['geometry_candidate_valid']}",
        f"coverage={float(row['vertical_coverage_ratio']):.3f} median_width={row['weld_width_px_median']}",
        f"lateral={row['lateral_error_px']} heading={row['heading_error_deg']}",
    ]
    for index, text in enumerate(lines):
        point = (18, 32 + index * 30)
        cv2.putText(tile, text, point, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(tile, text, point, cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2, cv2.LINE_AA)
    return tile


def validate_set(output_dir: Path = DEFAULT_OUTPUT, config_path: Path = DEFAULT_CONFIG) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    preflight = {case: _preflight(path, config_path) for case, path in CASES.items()}
    results: dict[str, dict[str, Any]] = {}
    rows: list[dict[str, Any]] = []
    for case, path in CASES.items():
        case_dir = output_dir / case
        result = run_experiment(path, config_path, case_dir, max_dimension=896)
        results[case] = result
        rows.append(_summary_row(case, result, preflight[case]))
    for row in rows:
        if row["input_unchanged"] is not True:
            raise AssertionError(f"Input changed during experiment: {row['case']}")
    center_lateral = rows[0]["lateral_error_px"]
    left_lateral = rows[1]["lateral_error_px"]
    right_lateral = rows[2]["lateral_error_px"]
    checks = {
        "all_inputs_readable": all(item["readable_by_opencv"] for item in preflight.values()),
        "all_input_hashes_unchanged": all(item["input_unchanged"] for item in rows),
        "left_lateral_more_negative_than_center": (left_lateral < center_lateral) if left_lateral is not None and center_lateral is not None else None,
        "right_lateral_more_positive_than_center": (right_lateral > center_lateral) if right_lateral is not None and center_lateral is not None else None,
        "tilt_heading_error_deg": rows[3]["heading_error_deg"],
        "tilt_heading_nonzero": abs(rows[3]["heading_error_deg"]) > 0.1 if rows[3]["heading_error_deg"] is not None else None,
        "all_center_points_between_boundaries": all(row["centerline_between_boundaries"] for row in rows),
        "all_restored_coordinates_valid": all(row["restored_coordinates_valid"] for row in rows),
        "all_production_valid_false": all(row["production_valid"] is False for row in rows),
    }
    # The montage is for human comparison only; case labels do not alter data.
    tiles = [_annotated_tile(cv2.imread(str(output_dir / case / "centerline_overlay.jpg")), case, row)
             for case, row in zip(CASES, rows)]
    montage = np.vstack((np.hstack((tiles[0], tiles[1])), np.hstack((tiles[2], tiles[3]))))
    cv2.imwrite(str(output_dir / "comparison_montage.jpg"), montage, [cv2.IMWRITE_JPEG_QUALITY, 95])
    fields = ["case", "input_path", "geometry_candidate_valid", "failure_reason", "image_quality_valid",
              "valid_left_boundary_point_count", "valid_right_boundary_point_count", "valid_center_point_count",
              "vertical_coverage_ratio", "weld_width_px_median", "weld_width_px_p05", "weld_width_px_p95",
              "centerline_fit_residual_px", "centerline_inlier_ratio", "lateral_error_px", "heading_error_deg",
              "centerline_between_boundaries", "restored_coordinates_valid", "processing_time_ms",
              "sha256_before", "sha256_after", "input_unchanged", "production_valid"]
    with (output_dir / "summary.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "experiment": "S1-D2",
        "algorithm_source": "scripts/experiment_real_seam_band.py",
        "parameters_unchanged": True,
        "parameters": {"max_dimension": 896, "scan_step_processed_px": 8,
                        "scan_band_half_height_processed_px": 5,
                        "centerline_ransac_degree": 2, "centerline_ransac_iterations": 120,
                        "centerline_ransac_residual_threshold_processed_px": 8.0},
        "preflight": preflight,
        "results": rows,
        "checks": checks,
        "note": "Position checks are recorded only; no filename-based correction or per-image tuning is applied.",
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    summary = validate_set(args.output, args.config)
    print(json.dumps({"output": str(args.output), "checks": summary["checks"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
