#!/usr/bin/env python3
"""Shared, ROS-free helpers for seam validation recording and replay."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from tank_vision.seam_tracking_types import TrackingState
from tank_vision.stable_seam_tracker import StableSeamDiagnostics, StableSeamTracker, draw_stable_seam_overlay


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [json_safe(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, TrackingState):
        return value.name
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    return value


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(json_safe(value), indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def percentile(values: list[float], percent: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), percent))


def distribution(values: list[float]) -> dict[str, Any]:
    finite = [float(value) for value in values if np.isfinite(float(value))]
    return {"count": len(finite), "p05": percentile(finite, 5), "p50": percentile(finite, 50),
            "p95": percentile(finite, 95), "max": max(finite) if finite else None}


def camera_telemetry(frame: Any) -> dict[str, Any]:
    """Return only semantics exposed by the current FrameData contract.

    MindVision's frame header exposes ``fAnalogGain`` as a floating gain
    factor.  The separate integer control value returned by
    ``CameraGetAnalogGain`` is not carried by FrameData, so it must remain
    unavailable rather than being relabeled as raw gain.
    """
    gain_db = getattr(frame, "gain_db", None)
    analog_gain = None if gain_db is None else float(10.0 ** (float(gain_db) / 20.0))
    unavailable = []
    if getattr(frame, "exposure_mode", None) is None: unavailable.append("exposure_mode")
    if getattr(frame, "exposure_us", None) is None: unavailable.append("exposure_us")
    if analog_gain is None: unavailable.append("analog_gain")
    unavailable.append("analog_gain_raw")
    if gain_db is None: unavailable.append("gain_db")
    return {"exposure_mode": getattr(frame, "exposure_mode", None), "exposure_us": getattr(frame, "exposure_us", None),
            "analog_gain": analog_gain, "analog_gain_raw": None, "gain_db": gain_db,
            "telemetry_unavailable_fields": sorted(set(unavailable))}


def roi_crop(image: np.ndarray, roi: tuple[int, int, int, int]) -> np.ndarray:
    x, y, width, height = (int(value) for value in roi)
    return np.ascontiguousarray(image[y:y + height, x:x + width])


def roi_from_config(config: dict[str, Any], width: int, height: int) -> tuple[int, int, int, int]:
    roi = config["roi"]
    x0 = int(round(float(roi["x_min_ratio"]) * width))
    x1 = int(round(float(roi["x_max_ratio"]) * width))
    y0 = int(round(float(roi["y_min_ratio"]) * height))
    y1 = int(round(float(roi["y_max_ratio"]) * height))
    return x0, y0, x1 - x0, y1 - y0


def reconstruct_from_roi(roi_frame: np.ndarray, full_shape: tuple[int, ...], roi: tuple[int, int, int, int]) -> np.ndarray:
    canvas = np.zeros(full_shape, dtype=np.uint8)
    x, y, width, height = (int(value) for value in roi)
    canvas[y:y + height, x:x + width] = roi_frame.reshape((height, width) + full_shape[2:])
    return canvas


def pair_signature(pair: Any) -> dict[str, Any]:
    return {"band_index": int(pair.band_index), "y": float(pair.y), "left": float(pair.left),
            "right": float(pair.right), "score": float(pair.score), "score_parts": dict(pair.score_parts)}


def _diagnostic_signature(diagnostics: StableSeamDiagnostics | None) -> dict[str, Any]:
    if diagnostics is None:
        return {"all_pairs": [], "selected_path": [], "left_candidates": [], "right_candidates": []}
    return {
        "all_pairs": [[pair_signature(pair) for pair in band] for band in diagnostics.all_pairs],
        "selected_path": [pair_signature(pair) for pair in diagnostics.selected_pairs],
        "left_candidates": [values.tolist() for values in diagnostics.left_candidates],
        "right_candidates": [values.tolist() for values in diagnostics.right_candidates],
    }


def tracker_signature(tracker: StableSeamTracker, result: Any) -> dict[str, Any]:
    provenance = tracker.last_provenance or {}
    trace = provenance.get("result") or {}
    metrics = trace.get("geometry_metrics") or {}
    signature = {
        "candidates": _diagnostic_signature(tracker.last_diagnostics),
        "fit_coefficients": {"center": trace.get("fit_center_coefficients"), "width": trace.get("fit_width_coefficients")},
        "geometry_metrics": metrics,
        "detected": bool(result.detected),
        "predicted_only": bool(result.predicted_only),
        "state": result.tracking_state.name,
        "failure_reasons": list(result.failure_reasons),
        "confidence": float(result.confidence),
        "coverage": float(result.coverage),
        "effective_span": float(result.effective_span),
        "boundaries": {"left": result.left_boundary_original_px, "right": result.right_boundary_original_px,
                       "center": result.centerline_original_px, "width": result.width_profile_px},
    }
    return json_safe(signature)


def _changed(a: Any, b: Any, path: str, mismatches: list[str], atol: float = 1e-8) -> None:
    if isinstance(a, dict) and isinstance(b, dict):
        for key in sorted(set(a) | set(b)):
            if key not in a or key not in b:
                mismatches.append(f"{path}.{key}")
            else:
                _changed(a[key], b[key], f"{path}.{key}", mismatches, atol)
        return
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            mismatches.append(path + ".length")
            return
        for index, (left, right) in enumerate(zip(a, b)):
            _changed(left, right, f"{path}[{index}]", mismatches, atol)
        return
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        if not np.isclose(float(a), float(b), rtol=0.0, atol=atol):
            mismatches.append(path)
        return
    if a != b:
        mismatches.append(path)


def compare_signatures(left: dict[str, Any], right: dict[str, Any]) -> list[str]:
    mismatches: list[str] = []
    _changed(left, right, "result", mismatches)
    return mismatches


def roi_reconstruction_check(config: str | Path | dict[str, Any], reference: str | Path, image: np.ndarray) -> dict[str, Any]:
    """Verify that zero-filled pixels outside the configured ROI do not alter output."""
    direct_tracker = StableSeamTracker(config, reference)
    direct = direct_tracker.process(image, diagnostics=True, provenance=True)
    roi = direct.roi_original_px
    reconstructed = reconstruct_from_roi(roi_crop(image, roi), image.shape, roi)
    roi_tracker = StableSeamTracker(config, reference)
    replay = roi_tracker.process(reconstructed, diagnostics=True, provenance=True)
    left = tracker_signature(direct_tracker, direct)
    right = tracker_signature(roi_tracker, replay)
    mismatches = compare_signatures(left, right)
    return {"checked": True, "roi_original_px": list(roi), "reconstructed_shape": list(reconstructed.shape),
            "match": not mismatches, "mismatches": mismatches, "direct": left, "reconstructed": right}


def gate_truth(config: dict[str, Any], result: Any, provenance: dict[str, Any] | None,
               selected_pairs: list[Any] | None = None) -> dict[str, Any]:
    geometry = config["geometry"]
    metrics = (provenance or {}).get("result", {}).get("geometry_metrics") or {}
    path_count = len((provenance or {}).get("dp_final_path", [])) if provenance else int(result.accepted_scan_bands)
    fit_success = ((provenance or {}).get("result", {}).get("fit_center_coefficients") is not None
                   if provenance else result.fit_residual_px is not None)
    checks = {
        "point_count_pass": path_count >= int(geometry["minimum_point_count"]),
        "coverage_pass": float(result.coverage) >= float(geometry["minimum_coverage"]),
        "effective_span_pass": float(result.effective_span) >= float(geometry["minimum_effective_span"]),
        "missing_run_pass": int(result.maximum_missing_run) <= int(geometry["maximum_missing_run"]),
        "jump_pass": max(int(result.abrupt_left_jump_count), int(result.abrupt_right_jump_count)) <= int(geometry["maximum_abrupt_jump_count"]),
        "fit_success": fit_success,
        "joint_inlier_pass": float(result.fit_inlier_ratio) >= float(geometry["minimum_inlier_ratio"]),
        "residual_pass": result.fit_residual_px is not None and float(result.fit_residual_px) <= float(geometry["maximum_fit_residual_px"]),
        "path_confidence_pass": (float(metrics["path_confidence"]) >= float(geometry["minimum_path_confidence"])
                                 if "path_confidence" in metrics else None),
        "positive_width_pass": (all(float(value) > 0 for value in metrics["widths"])
                                if "widths" in metrics else None),
        "inside_roi_pass": ("fitted_boundaries_invalid_or_outside_roi" not in (provenance or {}).get("result", {}).get("geometry_failure_reasons", [])
                            if provenance else None),
        "non_crossing_pass": (int(getattr(result, "fitted_output_crossing_count", 0)) == 0
                              if hasattr(result, "fitted_output_crossing_count") else
                              (all(pair.left < pair.right for pair in (selected_pairs or [])) if provenance else None)),
        "temporal_pass": not any(reason.startswith("temporal_") for reason in result.failure_reasons),
    }
    path_pairs = (provenance or {}).get("dp_final_path", [])
    checks["non_crossing_pass"] = (int(getattr(result, "fitted_output_crossing_count", 0)) == 0
                                    if hasattr(result, "fitted_output_crossing_count") else
                                    (all(pair.left < pair.right for pair in (selected_pairs or [])) if provenance else None))
    known_checks = [value for value in checks.values() if value is not None]
    return {"checks": checks, "all_pass": bool(known_checks) and all(known_checks) and all(value is not None for value in checks.values()),
            "observed": {"point_count": path_count, "coverage": result.coverage, "effective_span": result.effective_span,
                         "maximum_missing_run": result.maximum_missing_run, "abrupt_left_jump_count": result.abrupt_left_jump_count,
                         "abrupt_right_jump_count": result.abrupt_right_jump_count, "path_confidence": metrics.get("path_confidence"),
                         "center_inlier_ratio": (provenance or {}).get("result", {}).get("center_inlier_ratio"),
                         "width_inlier_ratio": (provenance or {}).get("result", {}).get("width_inlier_ratio"),
                         "joint_inlier_ratio": result.fit_inlier_ratio, "fit_residual_px": result.fit_residual_px}}


def human_reason(config: dict[str, Any], result: Any, provenance: dict[str, Any] | None) -> str:
    reasons = list(result.failure_reasons)
    metrics = (provenance or {}).get("result", {}).get("geometry_metrics") or {}
    if reasons:
        if "inlier_ratio_below_minimum" in reasons:
            return f"左右候选路径完整，但联合内点率{result.fit_inlier_ratio:.2f}低于{float(config['geometry']['minimum_inlier_ratio']):.2f}"
        if "fit_residual_above_maximum" in reasons and result.fit_residual_px is not None:
            return f"联合内点率通过，但拟合残差{result.fit_residual_px:.1f} px高于{float(config['geometry']['maximum_fit_residual_px']):.1f} px"
        return "；".join(reasons)
    if result.tracking_state.name == "TRACKING":
        return "当前帧通过并处于TRACKING"
    if result.tracking_state.name == "ACQUIRING":
        return "单帧几何通过，正在等待连续帧进入TRACKING"
    if result.predicted_only:
        return "检测失败，当前输出为预测/降级结果"
    return f"当前帧状态为{result.tracking_state.name}"


def frame_record(packet: Any, pre_state: str, result: Any, tracker: StableSeamTracker) -> dict[str, Any]:
    provenance = tracker.last_provenance or {}
    metrics = provenance.get("result", {}).get("geometry_metrics") or {}
    diagnostics = tracker.last_diagnostics
    selected = diagnostics.selected_pairs if diagnostics is not None else []
    crossing = int(getattr(result, "fitted_output_crossing_count", 0))
    selected_crossing = int(getattr(result, "selected_pair_crossing_count", 0))
    trace = tracker_signature(tracker, result)
    gate = gate_truth(tracker.config, result, provenance, selected)
    telemetry = camera_telemetry(packet.frame)
    unavailable = []
    if not provenance:
        unavailable.extend(["path_confidence", "center_inlier_ratio", "width_inlier_ratio"])
    return {
        "frame_index": int(packet.captured_index), "captured_index": int(packet.captured_index), "stored_index": None,
        "host_timestamp_ns": int(packet.host_timestamp_ns), "device_timestamp_ns": int(packet.frame.timestamp_ns),
        "pre_state": pre_state, "post_state": result.tracking_state.name, "detected": bool(result.detected),
        "predicted_only": bool(result.predicted_only), "tracking_state": result.tracking_state.name,
        "confidence": float(result.confidence), "coverage": float(result.coverage), "effective_span": float(result.effective_span),
        "point_count": len(selected), "maximum_missing_run": int(result.maximum_missing_run),
        "abrupt_left_jump_count": int(result.abrupt_left_jump_count), "abrupt_right_jump_count": int(result.abrupt_right_jump_count),
        "selected_pair_crossing_count": selected_crossing,
        "fitted_output_crossing_count": crossing,
        "minimum_fitted_width_px": getattr(result, "minimum_fitted_width_px", None),
        "crossing_count": crossing, "path_confidence": metrics.get("path_confidence"),
        "center_inlier_ratio": provenance.get("result", {}).get("center_inlier_ratio"),
        "width_inlier_ratio": provenance.get("result", {}).get("width_inlier_ratio"),
        "joint_inlier_ratio": float(result.fit_inlier_ratio), "fit_residual_px": result.fit_residual_px,
        "median_width_px": result.median_width_px, "lateral_error_px": result.lateral_error_px,
        "heading_error_deg": result.heading_error_deg, "processing_ms": float(result.processing_ms),
        "failure_reasons": list(result.failure_reasons), "human_readable_reason": human_reason(tracker.config, result, provenance),
        "exposure_mode": telemetry["exposure_mode"], "exposure_us": telemetry["exposure_us"],
        "analog_gain": telemetry["analog_gain"], "analog_gain_raw": telemetry["analog_gain_raw"], "gain_db": telemetry["gain_db"],
        "telemetry_unavailable_fields": sorted(set(packet.telemetry_unavailable_fields) | set(telemetry["telemetry_unavailable_fields"])),
        "metrics_unavailable_fields": unavailable,
        "roi_original_px": list(result.roi_original_px),
        "selected_path": [pair_signature(pair) for pair in selected],
        "gate_truth": gate,
        "fit_center_coefficients": provenance.get("result", {}).get("fit_center_coefficients"),
        "fit_width_coefficients": provenance.get("result", {}).get("fit_width_coefficients"),
        "signature": trace,
    }


def diagnostic_overlay(image: np.ndarray, result: Any, tracker: StableSeamTracker, max_size: tuple[int, int] | None = None) -> np.ndarray:
    output = draw_stable_seam_overlay(image, result, None)
    diagnostics = tracker.last_diagnostics
    provenance = tracker.last_provenance or {}
    if not result.detected and diagnostics is not None:
        roi = result.roi_original_px
        sx = diagnostics.processed_gray.shape[1] / max(roi[2], 1)
        sy = diagnostics.processed_gray.shape[0] / max(roi[3], 1)
        def to_original(pair: Any, side: str) -> tuple[int, int]:
            x = pair.left if side == "left" else pair.right
            return int(round(roi[0] + x / sx)), int(round(roi[1] + pair.y / sy))
        points_left = [to_original(pair, "left") for pair in diagnostics.selected_pairs]
        points_right = [to_original(pair, "right") for pair in diagnostics.selected_pairs]
        for points in (points_left, points_right):
            for a, b in zip(points[::2], points[1::2]):
                cv2.line(output, a, b, (0, 0, 255), 3, cv2.LINE_AA)
        coeff = provenance.get("result", {}).get("fit_center_coefficients")
        width_coeff = provenance.get("result", {}).get("fit_width_coefficients")
        if coeff is not None and width_coeff is not None and points_left:
            ys = np.linspace(points_left[0][1], points_left[-1][1], 32)
            centers = np.polyval(np.asarray(coeff, dtype=float), ys)
            widths = np.polyval(np.asarray(width_coeff, dtype=float), ys)
            rejected_left = [(int(round(c - w / 2)), int(round(y))) for c, w, y in zip(centers, widths, ys)]
            rejected_right = [(int(round(c + w / 2)), int(round(y))) for c, w, y in zip(centers, widths, ys)]
            for points in (rejected_left, rejected_right):
                for a, b in zip(points[::2], points[1::2]):
                    cv2.line(output, a, b, (0, 0, 255), 2, cv2.LINE_AA)
    label = "REJECTED / DIAGNOSTIC ONLY" if not result.detected else "VALID GEOMETRY"
    cv2.putText(output, label, (25, 105), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(output, label, (25, 105), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255) if not result.detected else (0, 180, 0), 2, cv2.LINE_AA)
    if max_size is not None:
        scale = min(1.0, max_size[0] / output.shape[1], max_size[1] / output.shape[0])
        if scale < 1:
            output = cv2.resize(output, (round(output.shape[1] * scale), round(output.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    return output


def preview_frame(image: np.ndarray, result: Any, tracker: StableSeamTracker, row: dict[str, Any], history: list[str], width: int) -> np.ndarray:
    target_h = round(width * image.shape[0] / image.shape[1])
    target_h = min(target_h, 680)
    annotated = diagnostic_overlay(image, result, tracker, (width - 360, target_h))
    canvas = np.full((720, width, 3), 24, dtype=np.uint8)
    h, w = annotated.shape[:2]
    canvas[:h, :w] = annotated
    panel_x = width - 350
    lines = [f"state: {result.tracking_state.name}", f"detected: {result.detected}", f"frame: {row['frame_index']}",
             f"confidence: {result.confidence:.3f}", f"coverage: {result.coverage:.3f}", f"span: {result.effective_span:.3f}",
             f"inlier: {result.fit_inlier_ratio:.3f} / 0.680", f"residual: {result.fit_residual_px!s} / 14.0",
             f"path conf: {row.get('path_confidence')!s} / 0.420", f"missing: {result.maximum_missing_run}",
             f"jumps L/R: {result.abrupt_left_jump_count}/{result.abrupt_right_jump_count}", f"processing: {result.processing_ms:.2f} ms",
             f"exposure: {row.get('exposure_us')!s} us", f"gain: {row.get('analog_gain_raw')!s}"]
    color = (0, 210, 0) if result.detected and result.tracking_state.name == "TRACKING" else (0, 210, 255) if result.tracking_state.name == "ACQUIRING" else (0, 140, 255) if result.predicted_only else (0, 0, 255)
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (panel_x, 32 + index * 30), cv2.FONT_HERSHEY_SIMPLEX, .65, color if index < 2 else (230, 230, 230), 1, cv2.LINE_AA)
    reason = row.get("human_readable_reason", "")[:38]
    cv2.putText(canvas, reason, (20, 705), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
    start = max(0, len(history) - 60)
    for index, state in enumerate(history[start:]):
        color_map = {"TRACKING": (0, 190, 0), "ACQUIRING": (0, 210, 255), "DEGRADED": (0, 140, 255), "LOST": (0, 0, 220)}
        cv2.line(canvas, (20 + index * 8, 688), (26 + index * 8, 688), color_map.get(state, (180, 180, 180)), 6)
    return canvas
