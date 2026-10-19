#!/usr/bin/env python3
"""Build the S100P normalized width prior from the frozen S1-D7 V2 annotation."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tank_vision.seam_width_reference import sha256_file

EXPECTED_SOURCE_SHA256 = "f49df2fd8e7cffa72b87f5d004096f1b2410a88cf503984b2dc89e53d9bbaf92"
EXPECTED_ANNOTATION_SHA256 = "f960bf210508e011d3646d64a181df01d10ad4e416834e42cf23cfac7f3a68c6"


def build(annotation_path: Path, manifest_path: Path, output_path: Path, sample_count: int = 64) -> dict:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    annotation = json.loads(annotation_path.read_text(encoding="utf-8"))
    source_image = ROOT / manifest.get("input_image_relative", "")
    overlay_path = manifest_path.parent / "seam_center_lit_quality_01_annotation_overlay_v2.png"
    checks = {
        "manifest_status_frozen": manifest.get("status") == "frozen",
        "human_overlay_approved": manifest.get("human_overlay_approved") is True,
        "json_overlay_consistent": manifest.get("json_overlay_consistent") is True,
        "source_image_sha256": (source_image.is_file()
                                and sha256_file(source_image) == manifest.get("input_sha256") == EXPECTED_SOURCE_SHA256),
        "annotation_sha256": (annotation_path.is_file()
                              and sha256_file(annotation_path) == manifest.get("json_sha256")
                              == EXPECTED_ANNOTATION_SHA256),
        "overlay_sha256": overlay_path.is_file() and sha256_file(overlay_path) == manifest.get("overlay_sha256"),
    }
    if not all(checks.values()):
        raise ValueError(f"Frozen V2 manifest/hash validation failed: {checks}")
    left = sorted(annotation["left_boundary_manual_points"], key=lambda p: p["y"])
    right = sorted(annotation["right_boundary_manual_points"], key=lambda p: p["y"])
    y0 = max(left[0]["y"], right[0]["y"])
    y1 = min(left[-1]["y"], right[-1]["y"])
    ys = np.linspace(y0, y1, sample_count)
    lx = np.interp(ys, [p["y"] for p in left], [p["x"] for p in left])
    rx = np.interp(ys, [p["y"] for p in right], [p["x"] for p in right])
    widths = rx - lx
    if np.any(widths <= 0):
        raise ValueError("Frozen annotation boundaries cross")
    # Three-sample median is a light robust smoother; endpoints are preserved
    # and no sample is allowed to move outside its local three-point envelope.
    smooth = widths.copy()
    for index in range(1, len(widths) - 1):
        smooth[index] = np.median(widths[index - 1:index + 2])
    w, h = int(annotation["image_width"]), int(annotation["image_height"])
    samples = [
        {"y_normalized": float(y / h), "center_x_normalized": float((l + r) * 0.5 / w),
         "width_normalized": float(sw / w), "width_px_at_reference_resolution": float(sw)}
        for y, l, r, sw in zip(ys, lx, rx, smooth)
    ]
    stats = {name: float(value) for name, value in zip(
        ("min", "p05", "p50", "p95", "max"),
        (np.min(smooth), np.percentile(smooth, 5), np.median(smooth), np.percentile(smooth, 95), np.max(smooth)),
    )}
    result = {
        "schema_version": "1.0", "source_image_path": annotation["input_path"],
        "source_image_sha256": annotation["input_sha256"], "annotation_path": str(annotation_path.relative_to(ROOT)),
        "annotation_sha256": sha256_file(annotation_path), "source_width": w, "source_height": h,
        "reference_samples": samples, "width_statistics_px": stats, "common_y_range_original_px": [float(y0), float(y1)],
        "generated_at": datetime.now(timezone.utc).isoformat(), "reference_is_manual": True,
        "smoothing": "three_sample_running_median_with_preserved_endpoints",
        "frozen_v2_validation": checks,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--annotation", type=Path, default=ROOT / "output/s1_d7_manual_ground_truth_v2/seam_center_lit_quality_01_annotation_v2.json")
    parser.add_argument("--manifest", type=Path, default=ROOT / "output/s1_d7_manual_ground_truth_v2/ground_truth_manifest.json")
    parser.add_argument("--output", type=Path, default=ROOT / "configs/seam_width_reference_s100p_v1.json")
    args = parser.parse_args()
    print(json.dumps(build(args.annotation, args.manifest, args.output)["width_statistics_px"], indent=2))


if __name__ == "__main__":
    main()
