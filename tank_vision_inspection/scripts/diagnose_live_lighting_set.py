#!/usr/bin/env python3
"""S1-P2F1: read-only diagnosis of six live lighting snapshots."""

from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.diagnose_anchor_width_tail import pstats  # noqa: E402
from scripts.diagnose_reference_selection_error import manual_at  # noqa: E402
from tank_vision.seam_tracking_types import TrackingState  # noqa: E402
from tank_vision.stable_seam_tracker import StableSeamTracker, draw_stable_seam_overlay  # noqa: E402

SNAPSHOT_ROOT = ROOT / "output/stable_seam_failure_snapshots"
OUT = ROOT / "output/s1_p2f1_live_lighting_diagnostic"
CONFIG_FALLBACK = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
REFERENCE_FALLBACK = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
CAMERA_SOURCE = ROOT / "ros2_ws/src/tank_vision_ros/tank_vision_ros/stable_seam_node.py"
CAMERA_SOURCE_ALT = ROOT / "tank_vision/sources/vendor_camera_source.py"
DIR_NAMES = [
    "20260910_160926_591_stamp_21705_511000000",
    "20260910_161005_409_stamp_21744_357800000",
    "20260910_161037_929_stamp_21776_838700000",
    "20260910_161145_623_stamp_21844_557300000",
    "20260910_161207_181_stamp_21866_60800000",
    "20260910_161223_283_stamp_21882_201000000",
]
REQUIRED_FILES = ("raw.bmp", "overlay.png", "roi_raw.bmp", "roi_overlay.png", "metadata.json", "seam_geometry.json")
EXPECTED_RAW_SHAPE = (3072, 4096, 3)


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [clean(x) for x in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        x = float(value)
        return x if np.isfinite(x) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, TrackingState):
        return value.name
    return value


def write_json(name: str, value: Any) -> None:
    (OUT / name).write_text(json.dumps(clean(value), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pstats_or_none(values: list[float]) -> dict[str, Any]:
    return pstats(values) if values else {"count": 0, "mean": None, "p50": None, "p90": None, "p95": None, "max": None}


def snapshot_paths() -> list[Path]:
    return [SNAPSHOT_ROOT / name for name in DIR_NAMES]


def read_evidence() -> tuple[list[dict[str, Any]], list[np.ndarray]]:
    evidence = []
    images = []
    for index, directory in enumerate(snapshot_paths(), start=1):
        files = {name: directory / name for name in REQUIRED_FILES}
        metadata = json.loads(files["metadata.json"].read_text(encoding="utf-8")) if files["metadata.json"].is_file() else {}
        geometry = json.loads(files["seam_geometry.json"].read_text(encoding="utf-8")) if files["seam_geometry.json"].is_file() else {}
        raw = cv2.imread(str(files["raw.bmp"]), cv2.IMREAD_UNCHANGED) if files["raw.bmp"].is_file() else None
        overlay = cv2.imread(str(files["overlay.png"]), cv2.IMREAD_UNCHANGED) if files["overlay.png"].is_file() else None
        roi_raw = cv2.imread(str(files["roi_raw.bmp"]), cv2.IMREAD_UNCHANGED) if files["roi_raw.bmp"].is_file() else None
        roi_overlay = cv2.imread(str(files["roi_overlay.png"]), cv2.IMREAD_UNCHANGED) if files["roi_overlay.png"].is_file() else None
        config_path = Path(metadata.get("tracker_config_path", ""))
        reference_path = Path(metadata.get("width_reference_path", ""))
        roi_values = metadata.get("roi_original_px", [0, 0, 0, 0])
        checks = {"directory_exists": directory.is_dir(),
                  "required_files_exist": all(path.is_file() for path in files.values()),
                  "metadata_complete": metadata.get("complete") is True,
                  "raw_shape_dtype": raw is not None and tuple(raw.shape) == EXPECTED_RAW_SHAPE and str(raw.dtype) == "uint8",
                  "overlay_shape_dtype": overlay is not None and tuple(overlay.shape) == EXPECTED_RAW_SHAPE and str(overlay.dtype) == "uint8",
                  "roi_image_shape_dtype": roi_raw is not None and roi_overlay is not None
                  and tuple(roi_raw.shape[:2]) == (int(roi_values[3]), int(roi_values[2]))
                  and tuple(roi_overlay.shape[:2]) == (int(roi_values[3]), int(roi_values[2]))
                  and str(roi_raw.dtype) == "uint8" and str(roi_overlay.dtype) == "uint8",
                  "header_stamp_match": metadata.get("header_stamp") == geometry.get("header", {}).get("stamp"),
                  "roi_match": metadata.get("roi_original_px") == [geometry.get("roi", {}).get(key) for key in ("x_offset", "y_offset", "width", "height")],
                  "config_sha_match": config_path.is_file() and metadata.get("tracker_config_sha256") == sha256(config_path),
                  "reference_sha_match": reference_path.is_file() and metadata.get("width_reference_sha256") == sha256(reference_path)}
        for field, file_name in (("raw_sha256", "raw.bmp"), ("overlay_sha256", "overlay.png"),
                                 ("roi_raw_sha256", "roi_raw.bmp"), ("seam_geometry_sha256", "seam_geometry.json")):
            checks[field + "_match"] = files[file_name].is_file() and metadata.get(field) == sha256(files[file_name])
        if raw is None:
            raw = np.zeros(EXPECTED_RAW_SHAPE, dtype=np.uint8)
        images.append(raw)
        evidence.append({"capture": f"capture_{index:02d}", "directory": str(directory),
                         "files": {name: {"path": str(path), "exists": path.is_file(),
                                          "sha256": sha256(path) if path.is_file() else None} for name, path in files.items()},
                         "metadata": metadata, "geometry": geometry, "checks": checks,
                         "integrity_pass": all(checks.values())})
    return evidence, images


def load_config(reference_path: str | Path, anchor_enabled: bool = True) -> dict[str, Any]:
    payload = yaml.safe_load(Path(reference_path).read_text(encoding="utf-8"))
    payload = copy.deepcopy(payload)
    payload.setdefault("guided_search", {})["reference_anchor_enabled"] = bool(anchor_enabled)
    return payload


def pair_original(pair: Any, result: Any, diagnostics: Any) -> dict[str, float | int | str]:
    x0, _y0, rw, _rh = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / rw
    return {"pair_id": f"b{pair.band_index}:l{int(round(pair.left))}:r{int(round(pair.right))}",
            "band_index": int(pair.band_index), "left": float(x0 + pair.left / sx),
            "right": float(x0 + pair.right / sx), "center": float(x0 + pair.center / sx),
            "width": float(pair.width / sx), "score": float(pair.score),
            "score_parts": {k: float(v) for k, v in pair.score_parts.items()}}


def anchor_use_stats(diagnostics: Any, provenance: dict[str, Any], result: Any) -> dict[str, Any]:
    traces = provenance.get("candidate_stage_trace", [])
    valid_left = valid_right = both_valid = selected_left = selected_right = 0
    selected_pairs = {p.band_index: p for p in diagnostics.selected_pairs}
    anchor_rows = []
    for band, trace in enumerate(traces):
        anchor = trace.get("reference_anchor")
        if not anchor:
            continue
        left, right = anchor.get("left", {}), anchor.get("right", {})
        lv, rv = bool(left.get("valid")), bool(right.get("valid"))
        valid_left += int(lv); valid_right += int(rv); both_valid += int(lv and rv)
        selected = selected_pairs.get(band)
        final_l = anchor.get("final_left_candidates_processed", [])
        final_r = anchor.get("final_right_candidates_processed", [])
        lpos = min(final_l, key=lambda x: abs(int(x) - int(left.get("position_processed", -10**9)))) if lv and final_l else None
        rpos = min(final_r, key=lambda x: abs(int(x) - int(right.get("position_processed", -10**9)))) if rv and final_r else None
        lused = selected is not None and lpos is not None and int(round(selected.left)) == int(lpos)
        rused = selected is not None and rpos is not None and int(round(selected.right)) == int(rpos)
        selected_left += int(lused); selected_right += int(rused)
        anchor_rows.append({"band_index": band, "left_valid": lv, "right_valid": rv,
                            "left_position_processed": left.get("position_processed"),
                            "right_position_processed": right.get("position_processed"),
                            "left_final_candidate_processed": lpos, "right_final_candidate_processed": rpos,
                            "selected_pair_id": None if selected is None else f"b{band}:l{int(round(selected.left))}:r{int(round(selected.right))}",
                            "left_used_by_selected": lused, "right_used_by_selected": rused})
    n = max(len(traces), 1)
    return {"scan_bands": len(traces), "left_valid_count": valid_left, "right_valid_count": valid_right,
            "both_valid_count": both_valid, "left_generation_rate": valid_left / n,
            "right_generation_rate": valid_right / n, "both_generation_rate": both_valid / n,
            "left_selected_use_count": selected_left, "right_selected_use_count": selected_right,
            "left_selected_use_rate": selected_left / n, "right_selected_use_rate": selected_right / n,
            "by_band": anchor_rows}


def fit_audit(tracker: StableSeamTracker, result: Any, diagnostics: Any, image: np.ndarray) -> dict[str, Any]:
    roi = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / roi[2]
    sy = diagnostics.processed_gray.shape[0] / roi[3]
    path = diagnostics.selected_pairs
    fit = tracker._fit(path, roi, sx, sy, image.shape[1], image.shape[0]) if path else None
    y_values = [float(roi[1] + p.y / sy) for p in path]
    raw_rows = []
    for p, y in zip(path, y_values):
        original = pair_original(p, result, diagnostics)
        center, width = tracker.reference.at(y, image.shape[1], image.shape[0])
        raw_rows.append({"band_index": p.band_index, "y_original": y, **original,
                         "reference_left": float(center - width / 2), "reference_right": float(center + width / 2),
                         "reference_width": float(width), "reference_distance_l1": abs(original["left"] - (center - width / 2)) + abs(original["right"] - (center + width / 2))})
    if fit is None:
        return {"fit_available": False, "raw_selected": raw_rows, "inlier_count": 0, "width_inlier_count": 0,
                "joint_inlier_count": 0, "by_y": []}
    ys = np.asarray(fit["ys"], dtype=np.float64)
    centers = np.asarray(fit["centers"], dtype=np.float64)
    widths = np.asarray(fit["widths"], dtype=np.float64)
    center_residual = centers - np.polyval(fit["center"], ys)
    width_residual = widths - np.polyval(fit["width"], ys)
    left_residual = center_residual - 0.5 * width_residual
    right_residual = center_residual + 0.5 * width_residual
    rows = []
    for idx, row in enumerate(raw_rows):
        rows.append({**row, "center_residual_px": float(center_residual[idx]), "width_residual_px": float(width_residual[idx]),
                     "left_residual_px": float(left_residual[idx]), "right_residual_px": float(right_residual[idx]),
                     "center_inlier": bool(abs(center_residual[idx]) <= 20.0),
                     "width_inlier": bool(abs(width_residual[idx]) <= 40.0),
                     "joint_inlier": bool(fit["inliers"][idx]),
                     "dominant_boundary_residual": "left" if abs(left_residual[idx]) > abs(right_residual[idx]) else "right"})
    outliers = [row for row in rows if not row["joint_inlier"]]
    return {"fit_available": True, "fit_center_coefficients": fit["center"], "fit_width_coefficients": fit["width"],
            "fit_residual_px": fit["residual_px"], "raw_selected": raw_rows,
            "inlier_count": int(np.count_nonzero(np.abs(center_residual) <= 20.0)),
            "width_inlier_count": int(np.count_nonzero(np.abs(width_residual) <= 40.0)),
            "joint_inlier_count": int(np.count_nonzero(fit["inliers"])),
            "joint_inlier_ratio": float(np.mean(fit["inliers"])),
            "excluded_y_ranges": [float(row["y_original"]) for row in outliers],
            "outlier_boundary_contribution": {"left": sum(abs(row["left_residual_px"]) > abs(row["right_residual_px"]) for row in outliers),
                                                "right": sum(abs(row["right_residual_px"]) >= abs(row["left_residual_px"]) for row in outliers)},
            "by_y": rows}


def sample_means(profile: np.ndarray, gray_profile: np.ndarray, left: int, right: int, config: dict[str, Any]) -> dict[str, Any]:
    left = int(left); right = int(right)
    if right <= left + 4:
        return {"valid": False}
    q = config["pairing"]
    width = right - left
    margin = int(q["interior_margin_px"])
    outside = max(int(q["exterior_sample_min_px"]), int(round(width * float(q["exterior_sample_width_scale"]))))
    la, lb = max(0, left - outside), left
    ia, ib = min(len(gray_profile), left + margin), max(0, right - margin)
    ra, rb = right, min(len(gray_profile), right + outside)
    if not (lb > la and ib > ia and rb > ra):
        return {"valid": False}
    interior = float(np.mean(gray_profile[ia:ib]))
    left_exterior = float(np.mean(gray_profile[la:lb]))
    right_exterior = float(np.mean(gray_profile[ra:rb]))
    left_step = interior - left_exterior
    right_step = interior - right_exterior
    return {"valid": True, "interior_mean": interior, "left_exterior_mean": left_exterior,
            "right_exterior_mean": right_exterior, "left_step_contrast": left_step,
            "right_step_contrast": right_step,
            "step_balance": min(max(left_step, 0.0), max(right_step, 0.0)) /
            (max(abs(left_step), abs(right_step)) + 1e-6),
            "left_index": left, "right_index": right}


def anchor_snr_audit(result: Any, diagnostics: Any, provenance: dict[str, Any], config: dict[str, Any], image: np.ndarray) -> list[dict[str, Any]]:
    gray = diagnostics.processed_gray
    half = int(config["processing"]["scan_band_half_height_px"])
    gray_profiles = cv2.blur(gray.astype(np.float32), (1, 2 * half + 1))[diagnostics.scan_ys]
    output = []
    for band, trace in enumerate(provenance.get("candidate_stage_trace", [])):
        anchor = trace.get("reference_anchor")
        if not anchor:
            continue
        response = diagnostics.edge_response[band].astype(np.float64)
        gray_profile = gray_profiles[band]
        row = {"band_index": band, "y_processed": int(diagnostics.scan_ys[band]), "left": {}, "right": {}}
        for side in ("left", "right"):
            item = anchor[side]
            search = diagnostics.candidate_search[band] if band < len(diagnostics.candidate_search) else {}
            window = search.get(f"{side}_window_processed_px", [0, len(response)])
            lo, hi = int(window[0]), int(window[1])
            lo, hi = max(0, lo), min(len(response), hi)
            values = np.abs(response[lo:hi])
            pos = item.get("position_processed")
            strength = float(item.get("strength")) if item.get("strength") is not None else None
            if pos is not None:
                excluded = np.ones(len(values), dtype=bool)
                center = int(pos) - lo
                excluded[max(0, center - 5):min(len(values), center + 6)] = False
                noise = values[excluded] if np.any(excluded) else values
            else:
                noise = values
            median = float(np.median(noise)) if len(noise) else None
            mad = float(np.median(np.abs(noise - median))) if len(noise) and median is not None else None
            reference_x = anchor.get(f"predicted_{side}_processed")
            step = sample_means(response, gray_profile, int(round(anchor["predicted_left_processed"] if side == "left" else anchor["predicted_right_processed"])),
                                int(round(anchor["predicted_right_processed"])), config) if side == "right" else None
            row[side] = {"valid": bool(item.get("valid")), "position_processed": pos,
                         "signed_response": item.get("signed_response"), "strength": strength,
                         "window_processed_px": [lo, hi], "noise_abs_median": median, "noise_abs_mad": mad,
                         "response_over_noise_mad": None if strength is None or mad is None else strength / max(mad, 1e-6),
                         "reference_position_processed": reference_x,
                         "anchor_reference_distance_processed": None if pos is None or reference_x is None else abs(float(pos) - float(reference_x))}
        # Use the retained anchor endpoints when possible for exact pair-side
        # intensity evidence; otherwise keep the field explicitly unavailable.
        left_pos = anchor["left"].get("position_processed")
        right_pos = anchor["right"].get("position_processed")
        if left_pos is not None and right_pos is not None:
            means = sample_means(response, gray_profile, int(left_pos), int(right_pos), config)
            row["pair_step_evidence"] = means
        else:
            row["pair_step_evidence"] = {"valid": False}
        output.append(row)
    return output


def image_quality(image: np.ndarray, roi: tuple[int, int, int, int]) -> dict[str, Any]:
    x, y, w, h = roi
    roi_image = image[y:y + h, x:x + w]
    def stats(arr: np.ndarray) -> dict[str, Any]:
        gray = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY) if arr.ndim == 3 else arr
        values = gray.astype(np.float32)
        return {"bgr_mean": np.mean(arr.reshape(-1, 3), axis=0) if arr.ndim == 3 else None,
                "bgr_std": np.std(arr.reshape(-1, 3), axis=0) if arr.ndim == 3 else None,
                "gray_mean": float(np.mean(values)), "gray_std": float(np.std(values)),
                "gray_percentiles": {f"p{p}": float(np.percentile(values, p)) for p in (1, 5, 50, 95, 99)},
                "underexposed_ratio_gray_lt_10": float(np.mean(values < 10.0)),
                "saturated_ratio_gray_ge_250": float(np.mean(values >= 250.0)),
                "laplacian_variance": float(cv2.Laplacian(gray, cv2.CV_64F).var())}
    gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
    row_means = np.mean(gray, axis=1); col_means = np.mean(gray, axis=0)
    local_mean = cv2.GaussianBlur(gray.astype(np.float32), (0, 0), 3)
    local_sq = cv2.GaussianBlur(np.square(gray.astype(np.float32)), (0, 0), 3)
    local_std = np.sqrt(np.maximum(local_sq - np.square(local_mean), 0.0))
    sobel = np.abs(cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3))
    h3, w3 = max(1, h // 3), max(1, w // 3)
    regions = {}
    for name, sub in {"left": roi_image[:, :w3], "middle": roi_image[:, w3:2 * w3], "right": roi_image[:, 2 * w3:],
                      "top": roi_image[:h3], "middle_y": roi_image[h3:2 * h3], "bottom": roi_image[2 * h3:]}.items():
        regions[name] = float(np.mean(cv2.cvtColor(sub, cv2.COLOR_BGR2GRAY)))
    result = {"full": stats(image), "roi": stats(roi_image), "roi_region_gray_mean": regions,
              "roi_region_max_min_ratio": max(regions.values()) / max(min(regions.values()), 1e-6),
              "roi_row_mean_std": float(np.std(row_means)), "roi_col_mean_std": float(np.std(col_means)),
              "roi_local_contrast_percentiles": {f"p{p}": float(np.percentile(local_std, p)) for p in (10, 50, 90)},
              "roi_sobel_abs_percentiles": {f"p{p}": float(np.percentile(sobel, p)) for p in (50, 90, 95, 99)}}
    return result


def diagnostic_overlay(image: np.ndarray, result: Any, tracker: StableSeamTracker, fit: dict[str, Any] | None,
                       diagnostics: Any) -> np.ndarray:
    if fit is None:
        return image.copy()
    path = diagnostics.selected_pairs
    left, right, center, width = tracker._sample_model(fit["center"], fit["width"], path,
                                                        result.roi_original_px,
                                                        diagnostics.processed_gray.shape[0] / result.roi_original_px[3],
                                                        image.shape[1], image.shape[0])
    candidate = copy.deepcopy(result)
    candidate.left_boundary_original_px = left
    candidate.right_boundary_original_px = right
    candidate.centerline_original_px = center
    candidate.width_profile_px = width
    candidate.failure_reasons = list(result.failure_reasons) + ["diagnostic_only_fit_overlay"]
    return draw_stable_seam_overlay(image, candidate, None)


def draw_montages(evidence: list[dict[str, Any]], images: list[np.ndarray], analyses: list[dict[str, Any]]) -> None:
    rows = []
    for index, (item, image, analysis) in enumerate(zip(evidence, images, analyses), start=1):
        directory = Path(item["directory"])
        roi = tuple(int(x) for x in item["metadata"].get("roi_original_px", [0, 0, 0, 0]))
        roi_image = image[roi[1]:roi[1] + roi[3], roi[0]:roi[0] + roi[2]]
        overlay = cv2.imread(str(directory / "overlay.png"), cv2.IMREAD_COLOR)
        roi_overlay = cv2.imread(str(directory / "roi_overlay.png"), cv2.IMREAD_COLOR)
        diagnostic = cv2.imread(str(OUT / "diagnostic_overlays" / f"capture_{index:02d}.png"), cv2.IMREAD_COLOR)
        if overlay is None: overlay = image
        if roi_overlay is None: roi_overlay = roi_image
        if diagnostic is None: diagnostic = overlay
        def tile(value: np.ndarray) -> np.ndarray: return cv2.resize(value, (300, 180), interpolation=cv2.INTER_AREA)
        text = np.full((180, 500, 3), 245, dtype=np.uint8)
        lines = [f"capture_{index:02d} {analysis['observed_state']} detected={analysis['observed_detected']}",
                 f"conf={analysis['confidence']:.3f} cov={analysis['coverage']:.3f} inlier={analysis['joint_inlier_ratio']:.3f}",
                 f"residual={analysis['fit_residual_px']:.3f} width={analysis['median_width']}",
                 f"lateral={analysis['lateral']} heading={analysis['heading']}",
                 f"processing={analysis['processing_ms']:.3f}ms", ",".join(analysis['failure_reasons']) or "failure_reasons=[]",
                 "diagnostic overlay: candidate path + fit only"]
        for line_no, line in enumerate(lines):
            cv2.putText(text, line[:68], (8, 23 + line_no * 23), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 20, 20), 1, cv2.LINE_AA)
        rows.append(np.hstack([tile(image), tile(roi_image), tile(overlay), tile(roi_overlay), text]))
    cv2.imwrite(str(OUT / "six_frame_montage.jpg"), np.vstack(rows))


def draw_heatmaps(images: list[np.ndarray], evidence: list[dict[str, Any]]) -> None:
    panels = []
    for index, (image, item) in enumerate(zip(images, evidence), start=1):
        x, y, w, h = (int(v) for v in item["metadata"]["roi_original_px"])
        gray = cv2.cvtColor(image[y:y + h, x:x + w], cv2.COLOR_BGR2GRAY)
        normalized = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        heat = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        heat = cv2.resize(heat, (500, 260), interpolation=cv2.INTER_AREA)
        cv2.putText(heat, f"capture_{index:02d} ROI illumination", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        panels.append(heat)
    cv2.imwrite(str(OUT / "illumination_heatmap_montage.jpg"), np.vstack(panels))


def draw_residuals(analyses: list[dict[str, Any]]) -> None:
    panels = []
    for index, analysis in enumerate(analyses, start=1):
        rows = analysis["fit_audit"].get("by_y", [])
        canvas = np.full((250, 950, 3), 248, dtype=np.uint8)
        if rows:
            ys = np.asarray([r["y_original"] for r in rows]); x0, x1 = float(ys.min()), float(ys.max())
            def xp(y: float) -> int: return int(50 + (y - x0) * 850 / max(x1 - x0, 1))
            def yp(value: float) -> int: return int(120 - np.clip(value, -80, 80) * 0.9)
            for field, color, label in (("center_residual_px", (0, 140, 255), "center"), ("width_residual_px", (0, 0, 220), "width")):
                pts = np.asarray([(xp(r["y_original"]), yp(r[field])) for r in rows], dtype=np.int32).reshape(-1, 1, 2)
                cv2.polylines(canvas, [pts], False, color, 2, cv2.LINE_AA)
                cv2.putText(canvas, label, (760, 25 + (0 if field.startswith("center") else 22)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            for r in rows:
                if not r["joint_inlier"]:
                    cv2.circle(canvas, (xp(r["y_original"]), yp(r["width_residual_px"])), 4, (0, 0, 0), -1)
        cv2.line(canvas, (50, 120), (900, 120), (100, 100, 100), 1)
        cv2.putText(canvas, f"capture_{index:02d}: black=joint outlier", (50, 225), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 1)
        panels.append(canvas)
    cv2.imwrite(str(OUT / "residual_by_y_montage.png"), np.vstack(panels))


def one_frame(path: Path, config_path: Path, reference_path: Path, image: np.ndarray, anchor_enabled: bool = True) -> dict[str, Any]:
    config = load_config(config_path, anchor_enabled)
    tracker = StableSeamTracker(config, str(reference_path))
    result = tracker.process(image, diagnostics=True, provenance=True)
    diagnostics = tracker.last_diagnostics
    provenance = tracker.last_provenance or {}
    if diagnostics is None:
        raise RuntimeError(f"missing diagnostics for {path}")
    fit = fit_audit(tracker, result, diagnostics, image)
    metrics = provenance.get("result", {}).get("geometry_metrics") or {}
    return {"image_path": str(path), "config_path": str(config_path), "reference_path": str(reference_path),
            "anchor_enabled": anchor_enabled, "observed_detected": bool(result.detected),
            "observed_state": result.tracking_state.name, "coverage": float(result.coverage),
            "effective_span": float(result.effective_span), "path_length": len(diagnostics.selected_pairs),
            "joint_inlier_ratio": float(result.fit_inlier_ratio), "fit_residual_px": result.fit_residual_px,
            "path_confidence": metrics.get("path_confidence"), "abrupt_left": result.abrupt_left_jump_count,
            "abrupt_right": result.abrupt_right_jump_count,
            "crossing": sum(int(p.left >= p.right) for p in diagnostics.selected_pairs),
            "median_width": result.median_width_px, "lateral": result.lateral_error_px,
            "heading": result.heading_error_deg, "confidence": result.confidence,
            "processing_ms": result.processing_ms, "failure_reasons": list(result.failure_reasons),
            "anchor_stats": anchor_use_stats(diagnostics, provenance, result),
            "anchor_snr": anchor_snr_audit(result, diagnostics, provenance, config, image),
            "fit_audit": fit, "tracker_provenance_result": provenance.get("result", {}),
            "diagnostics": diagnostics, "result": result, "tracker": tracker}


def strip_analysis(item: dict[str, Any]) -> dict[str, Any]:
    return {key: value for key, value in item.items() if key not in {"diagnostics", "result", "tracker"}}


def telemetry_audit(evidence: list[dict[str, Any]]) -> dict[str, Any]:
    keys = ("exposure_us", "analog_gain", "gain_raw", "auto_exposure", "white_balance", "gamma", "ae_target")
    metadata_keys = sorted({key for item in evidence for key in item["metadata"]})
    source_text = CAMERA_SOURCE.read_text(encoding="utf-8") + CAMERA_SOURCE_ALT.read_text(encoding="utf-8") if CAMERA_SOURCE.exists() and CAMERA_SOURCE_ALT.exists() else ""
    absent = [key for key in keys if not any(key in item["metadata"] for item in evidence)]
    return {"telemetry_missing": bool(absent), "requested_fields": list(keys), "metadata_keys": metadata_keys,
            "absent_fields": absent, "camera_source_records_requested_fields": {key: key in source_text for key in keys},
            "actual_values": "unknown; not guessed", "next_capture_should_record": ["exposure_us", "analog_gain", "auto_exposure_state"]}


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty output: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    evidence, images = read_evidence()
    write_json("evidence_inventory.json", {"capture_order": DIR_NAMES, "items": [{k: v for k, v in item.items() if k != "geometry"} for item in evidence],
                                            "all_integrity_pass": all(item["integrity_pass"] for item in evidence)})
    analyses = []
    anchor_off = []
    for item, image in zip(evidence, images):
        config_path = Path(item["metadata"].get("tracker_config_path", CONFIG_FALLBACK))
        reference_path = Path(item["metadata"].get("width_reference_path", REFERENCE_FALLBACK))
        if not config_path.is_file(): config_path = CONFIG_FALLBACK
        if not reference_path.is_file(): reference_path = REFERENCE_FALLBACK
        analysis = one_frame(Path(item["directory"]) / "raw.bmp", config_path, reference_path, image, True)
        analyses.append(analysis)
        anchor_off.append(one_frame(Path(item["directory"]) / "raw.bmp", config_path, reference_path, image, False))
    write_json("image_quality_comparison.json", {f"capture_{i:02d}": image_quality(image, tuple(item["metadata"]["roi_original_px"]))
                                                 for i, (image, item) in enumerate(zip(images, evidence), start=1)})
    frame_summary = []
    for index, (item, analysis) in enumerate(zip(evidence, analyses), start=1):
        record = strip_analysis(analysis)
        record["capture"] = f"capture_{index:02d}"
        record["field_observed"] = {"detected": item["metadata"].get("detected"),
                                    "state": item["metadata"].get("tracking_state"),
                                    "confidence": item["metadata"].get("confidence"),
                                    "coverage": item["metadata"].get("coverage"),
                                    "failure_reasons": item["geometry"].get("failure_reasons", []),
                                    "fit_residual_px": item["geometry"].get("fit_residual_px"),
                                    "processing_ms": item["geometry"].get("processing_ms")}
        record["offline_reproduction"] = {"detected": analysis["observed_detected"],
                                           "state": analysis["observed_state"],
                                           "joint_inlier_ratio": analysis["joint_inlier_ratio"],
                                           "fit_residual_px": analysis["fit_residual_px"]}
        frame_summary.append(record)
    write_json("six_frame_summary.json", {"items": frame_summary,
                                          "scene_labels": "capture_01..capture_06 only; no visual scene inference",
                                          "field_vs_offline_state_mismatch": any(item["field_observed"]["state"] != item["offline_reproduction"]["state"] for item in frame_summary)})
    write_json("offline_reproduction.json", {"items": [strip_analysis(x) for x in analyses], "anchor_enabled": True,
                                               "reset_single_frame": True, "repeated_three_frame": "see chronological_replay"})
    continuous_tracker = None
    if analyses:
        config_path = Path(analyses[0]["config_path"]); reference_path = Path(analyses[0]["reference_path"])
        config = load_config(config_path, True)
        continuous_tracker = StableSeamTracker(config, str(reference_path))
    chronological = []
    for index, (image, analysis) in enumerate(zip(images, analyses), start=1):
        result = continuous_tracker.process(image) if continuous_tracker is not None else None
        chronological.append({"capture": f"capture_{index:02d}", "field_state": evidence[index - 1]["metadata"].get("tracking_state"),
                              "field_detected": evidence[index - 1]["metadata"].get("detected"),
                              "state": None if result is None else result.tracking_state.name,
                              "detected": None if result is None else result.detected,
                              "coverage": None if result is None else result.coverage,
                              "fit_inlier_ratio": None if result is None else result.fit_inlier_ratio,
                              "fit_residual_px": None if result is None else result.fit_residual_px,
                              "failure_reasons": None if result is None else result.failure_reasons})
    write_json("chronological_replay.json", {"items": chronological, "one_tracker_reset_before_sequence": True,
                                               "note": "each capture is one real frame in timestamp order; no parameters changed"})
    write_json("anchor_on_off_comparison.json", {"anchor_on": [strip_analysis(x) for x in analyses],
                                                   "anchor_off": [strip_analysis(x) for x in anchor_off],
                                                   "comparison_scope": "single reset frame per image"})
    snr_rows = [dict(strip_analysis(item), anchor_snr=item["anchor_snr"]) for item in analyses]
    write_json("anchor_response_snr.json", {"items": snr_rows, "interpretation": "correlation only; six frames are insufficient for causal statistics"})
    write_json("joint_inlier_failure_by_y.json", {f"capture_{i:02d}": analysis["fit_audit"] for i, analysis in enumerate(analyses, start=1)})
    telemetry = telemetry_audit(evidence)
    write_json("camera_telemetry_audit.json", telemetry)
    (OUT / "diagnostic_overlays").mkdir(parents=True, exist_ok=True)
    for index, (image, analysis) in enumerate(zip(images, analyses), start=1):
        overlay = diagnostic_overlay(image, analysis["result"], analysis["tracker"],
                                     {"center": np.asarray(analysis["fit_audit"].get("fit_center_coefficients")) if analysis["fit_audit"].get("fit_available") else None,
                                      "width": np.asarray(analysis["fit_audit"].get("fit_width_coefficients")) if analysis["fit_audit"].get("fit_available") else None},
                                     analysis["diagnostics"])
        cv2.imwrite(str(OUT / "diagnostic_overlays" / f"capture_{index:02d}.png"), overlay)
    draw_montages(evidence, images, analyses)
    draw_heatmaps(images, evidence)
    draw_residuals(analyses)
    field_detected = [item for item in evidence if item["metadata"].get("detected") is True]
    field_lost = [item for item in evidence if item["metadata"].get("detected") is not True]
    qualities = json.loads((OUT / "image_quality_comparison.json").read_text(encoding="utf-8"))
    detected_capture = field_detected[0]["capture"] if field_detected else None
    lost_names = [f"capture_{i:02d}" for i in range(1, 7) if f"capture_{i:02d}" != detected_capture]
    detected_analysis = analyses[int(detected_capture[-2:]) - 1] if detected_capture else None
    summary = {"task": "S1-P2F1", "evidence_complete": all(item["integrity_pass"] for item in evidence),
               "observed_detected_count": len(field_detected), "observed_lost_count": len(field_lost),
               "detected_capture": detected_capture, "lost_captures": lost_names,
               "all_coverage_one": all(item["coverage"] == 1.0 for item in analyses),
               "failure_reason_counts": {reason: sum(reason in item["failure_reasons"] for item in analyses) for reason in
                                           sorted({reason for item in analyses for reason in item["failure_reasons"]})},
               "detected_vs_lost_roi_gray_mean": {"detected": None if not detected_capture else qualities[detected_capture]["roi"]["gray_mean"],
                                                   "lost": [qualities[name]["roi"]["gray_mean"] for name in lost_names]},
               "anchor_snr_detected": None if detected_analysis is None else detected_analysis["anchor_snr"],
               "anchor_snr_lost_medians": [{"capture": f"capture_{i:02d}", "left": float(np.median([x["left"].get("response_over_noise_mad") for x in analyses[i - 1]["anchor_snr"] if x["left"].get("response_over_noise_mad") is not None])),
                                             "right": float(np.median([x["right"].get("response_over_noise_mad") for x in analyses[i - 1]["anchor_snr"] if x["right"].get("response_over_noise_mad") is not None]))}
                                            for i in range(1, 7) if f"capture_{i:02d}" != detected_capture],
               "telemetry_missing": telemetry["telemetry_missing"],
               "field_vs_offline_mismatch": any(item["metadata"].get("tracking_state") != analysis["observed_state"] for item, analysis in zip(evidence, analyses)),
               "choice": "D" if telemetry["telemetry_missing"] or any(item["metadata"].get("tracking_state") != analysis["observed_state"] for item, analysis in zip(evidence, analyses)) else "A",
               "correlation_caveat": "six frames do not establish causality; missing exposure/gain telemetry and field/offline state mismatch prevent a stronger imaging-control conclusion",
               "production_changes": {"algorithm": False, "configuration": False, "reference": False, "gate": False, "ros": False}}
    write_json("diagnostic_summary.json", summary)
    next_action = {"choice": summary["choice"], "implemented": False,
                   "unique_next_action": "repeat the same eight scene captures while recording exposure_us, auto-exposure state, analog_gain, and tracker warm-state provenance; hold lighting geometry fixed before algorithm changes" if summary["choice"] in ("A", "C", "D") else "test one local double-sided step response normalized by local noise MAD",
                   "do_not_change": ["tracker", "anchor radius", "candidate", "pair", "unary", "DP", "fit", "geometry gate", "temporal", "YAML", "reference", "ROS"]}
    write_json("next_single_action.json", next_action)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
