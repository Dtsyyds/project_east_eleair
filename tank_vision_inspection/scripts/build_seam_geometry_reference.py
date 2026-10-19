#!/usr/bin/env python3
"""Build the frozen, calibration-guided geometry model from manual points only."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tank_vision.seam_width_reference import sha256_file


def _stats(values: np.ndarray) -> dict[str, float]:
    return {name: float(value) for name, value in zip(
        ("min", "p05", "p50", "p95", "max"),
        (np.min(values), np.percentile(values, 5), np.percentile(values, 50),
         np.percentile(values, 95), np.max(values)),
    )}


def _points(annotation: dict[str, Any], key: str) -> tuple[np.ndarray, np.ndarray]:
    values = annotation.get(key)
    if not isinstance(values, list) or len(values) < 3:
        raise ValueError(f"annotation must contain at least three points in {key}")
    ordered = sorted(values, key=lambda item: (float(item["y"]), float(item["x"])))
    ys = np.asarray([float(item["y"]) for item in ordered], dtype=np.float64)
    xs = np.asarray([float(item["x"]) for item in ordered], dtype=np.float64)
    if not (np.isfinite(xs).all() and np.isfinite(ys).all()) or np.any(np.diff(ys) <= 0):
        raise ValueError(f"annotation points in {key} must have finite, increasing y")
    return ys, xs


def build(annotation_path: Path, image_path: Path, output_path: Path,
          roi: dict[str, float] | None = None, sample_count: int = 128) -> dict[str, Any]:
    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim < 2:
        raise ValueError(f"cannot read reference image: {image_path}")
    height, width = image.shape[:2]
    annotation = json.loads(annotation_path.read_text(encoding="utf-8"))
    if int(annotation.get("image_width", width)) != width or int(annotation.get("image_height", height)) != height:
        raise ValueError("annotation dimensions do not match the input image")
    image_hash = sha256_file(image_path)
    annotation_hash = sha256_file(annotation_path)
    if annotation.get("input_sha256") != image_hash:
        raise ValueError("annotation input_sha256 does not match the supplied image")

    roi = roi or {"x_min_ratio": 0.30, "x_max_ratio": 0.60, "y_min_ratio": 0.50, "y_max_ratio": 0.92}
    x_min = int(round(float(roi["x_min_ratio"]) * width))
    x_max = int(round(float(roi["x_max_ratio"]) * width))
    y_min = int(round(float(roi["y_min_ratio"]) * height))
    y_max = int(round(float(roi["y_max_ratio"]) * height))
    if not (0 <= x_min < x_max <= width and 0 <= y_min < y_max <= height):
        raise ValueError("ROI ratios produce an invalid pixel ROI")

    left_y, left_x = _points(annotation, "left_boundary_manual_points")
    right_y, right_x = _points(annotation, "right_boundary_manual_points")
    common_y0 = max(float(left_y[0]), float(right_y[0]), float(y_min))
    common_y1 = min(float(left_y[-1]), float(right_y[-1]), float(y_max - 1))
    if common_y1 <= common_y0:
        raise ValueError("manual boundaries and ROI have no common y interval")
    ys = np.linspace(common_y0, common_y1, max(8, int(sample_count)), dtype=np.float64)
    lx = np.interp(ys, left_y, left_x)
    rx = np.interp(ys, right_y, right_x)
    centers = 0.5 * (lx + rx)
    widths = rx - lx
    if np.any(widths <= 0):
        raise ValueError("manual boundaries cross in the common y interval")

    yn = ys / float(height)
    center_coefficients = np.polyfit(yn, centers / float(width), 2)
    width_coefficients = np.polyfit(yn, widths / float(width), 1)
    stats = {
        "left_statistics_px": _stats(lx),
        "right_statistics_px": _stats(rx),
        "center_statistics_px": _stats(centers),
        "width_statistics_px": _stats(widths),
    }
    samples = [
        {"y_normalized": float(y / height), "center_x_normalized": float(c / width),
         "width_normalized": float(w / width), "width_px_at_reference_resolution": float(w),
         "left_px": float(l), "right_px": float(r), "center_px": float(c)}
        for y, l, r, c, w in zip(ys, lx, rx, centers, widths)
    ]
    result: dict[str, Any] = {
        "schema_version": "1.0",
        "status": "frozen",
        "generated_from_manual_annotation": True,
        "source_image_path": str(image_path.relative_to(ROOT) if image_path.is_relative_to(ROOT) else image_path),
        "input_image_sha256": image_hash,
        "source_image_sha256": image_hash,
        "annotation_path": str(annotation_path.relative_to(ROOT) if annotation_path.is_relative_to(ROOT) else annotation_path),
        "annotation_json_sha256": annotation_hash,
        "annotation_sha256": annotation_hash,
        "source_width": width,
        "source_height": height,
        "reference_image_size": {"width": width, "height": height},
        "used_roi": {
            **{key: float(value) for key, value in roi.items()},
            "x_min_px": x_min, "x_max_px": x_max, "y_min_px": y_min, "y_max_px": y_max,
        },
        "manual_common_y_range_original_px": [common_y0, common_y1],
        "common_y_range_original_px": [common_y0, common_y1],
        "center_ref": {
            "degree": 2, "independent_variable": "y_normalized",
            "dependent_variable": "x_normalized", "coefficients": [float(v) for v in center_coefficients],
        },
        "width_ref": {
            "degree": 1, "independent_variable": "y_normalized",
            "dependent_variable": "width_normalized", "coefficients": [float(v) for v in width_coefficients],
        },
        "center_ref_coefficients": [float(v) for v in center_coefficients],
        "width_ref_coefficients": [float(v) for v in width_coefficients],
        "reference_samples": samples,
        **stats,
        "statistics_px": stats,
        "fit_source": "manual_annotation_only",
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, default=ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp")
    parser.add_argument("--annotation", type=Path, default=ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json")
    parser.add_argument("--output", type=Path, default=ROOT / "configs/seam_geometry_reference_s100p_live_v1.json")
    args = parser.parse_args()
    result = build(args.annotation, args.image, args.output)
    print(json.dumps({"output": str(args.output), "image_sha256": result["input_image_sha256"],
                      "annotation_sha256": result["annotation_sha256"],
                      "manual_common_y_range_original_px": result["manual_common_y_range_original_px"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
