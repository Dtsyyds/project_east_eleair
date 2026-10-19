#!/usr/bin/env python3
"""Offline/video/MindVision runner for the stable seam production candidate."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import statistics
import sys
import time
from typing import Iterator

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from tank_vision.sources import MindVisionCameraSource, SourceError
from tank_vision.stable_seam_tracker import StableSeamTracker, draw_stable_seam_overlay


IMAGE_SUFFIXES = {".bmp", ".png", ".jpg", ".jpeg", ".tif", ".tiff"}


def _images(path: Path) -> Iterator[tuple[str, np.ndarray]]:
    paths = [path] if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES else (
        sorted(item for item in path.iterdir() if item.suffix.lower() in IMAGE_SUFFIXES) if path.is_dir() else [])
    for item in paths:
        image = cv2.imread(str(item), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError(f"OpenCV could not read {item}")
        yield item.stem, image


def _diagnostic_images(tracker: StableSeamTracker, image: np.ndarray, result: object) -> dict[str, np.ndarray]:
    diagnostics = tracker.last_diagnostics
    images = {"final_overlay": draw_stable_seam_overlay(image, result)}
    if diagnostics is None:
        return images
    edge = cv2.normalize(np.abs(diagnostics.edge_response), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    selected = cv2.cvtColor(diagnostics.processed_gray, cv2.COLOR_GRAY2BGR)
    for pair in diagnostics.selected_pairs:
        y = round(pair.y)
        cv2.circle(selected, (round(pair.left), y), 3, (255, 0, 0), -1)
        cv2.circle(selected, (round(pair.right), y), 3, (0, 255, 0), -1)
        cv2.circle(selected, (round(pair.center), y), 2, (0, 255, 255), -1)
    images.update({"candidate_edges": edge, "selected_boundaries": selected})
    return images


def _save(output: Path, name: str, image: np.ndarray, tracker: StableSeamTracker, result: object,
          save_overlay: bool, save_json: bool) -> None:
    output.mkdir(parents=True, exist_ok=True)
    if save_json:
        (output / f"{name}.json").write_text(json.dumps(result.to_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if save_overlay:
        for label, value in _diagnostic_images(tracker, image, result).items():
            cv2.imwrite(str(output / f"{name}_{label}.png"), value)


def _benchmark(tracker: StableSeamTracker, image: np.ndarray, warmup: int = 30, iterations: int = 300) -> dict:
    tracker.reset()
    for _ in range(warmup):
        tracker.process(image)
    samples = []
    stages: dict[str, list[float]] = {}
    for _ in range(iterations):
        result = tracker.process(image)
        samples.append(result.processing_ms)
        for name, value in result.stage_timings_ms.items():
            stages.setdefault(name, []).append(value)
    def stats(values: list[float]) -> dict[str, float]:
        return {"mean": statistics.fmean(values), "p50": float(np.percentile(values, 50)),
                "p95": float(np.percentile(values, 95)), "max": max(values)}
    return {"warmup_frames": warmup, "measured_frames": iterations, "total_ms": stats(samples),
            "stages_ms": {name: stats(values) for name, values in stages.items()},
            "algorithm_output_hz": 1000.0 / statistics.fmean(samples),
            "target_mean_below_100ms": statistics.fmean(samples) < 100.0,
            "display_debug_and_saving_disabled": True}


def _video(path: Path) -> Iterator[tuple[str, np.ndarray]]:
    capture = cv2.VideoCapture(str(path))
    if not capture.isOpened():
        raise ValueError(f"Could not open video {path}")
    index = 0
    try:
        while True:
            ok, image = capture.read()
            if not ok:
                break
            yield f"frame_{index:06d}", image
            index += 1
    finally:
        capture.release()


def _mindvision_frames(source: MindVisionCameraSource, max_attempts: int,
                       counters: dict[str, int]) -> Iterator[tuple[str, np.ndarray]]:
    """Keep acquisition alive across SDK timeouts without retaining old frames."""
    attempts = captured = 0
    while not max_attempts or attempts < max_attempts:
        attempts += 1
        try:
            frame = next(source)
        except SourceError as exc:
            key = "timeout_count" if "timeout" in str(exc).lower() else "capture_error_count"
            counters[key] += 1
            print(f"MindVision {key}: {exc}", file=sys.stderr)
            continue
        yield f"frame_{captured:06d}", frame.image
        captured += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", nargs="?", type=Path)
    parser.add_argument("--mindvision", action="store_true")
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/s100p_stable_seam_4096x3072.yaml")
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "output/s1_p1_stable_seam_tracker/run")
    parser.add_argument("--display", dest="display", action="store_true")
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.set_defaults(display=False)
    parser.add_argument("--save-overlay", action="store_true")
    parser.add_argument("--save-json", action="store_true")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    tracker = StableSeamTracker(args.config, args.reference)
    source = None
    capture_counters = {"timeout_count": 0, "capture_error_count": 0}
    if args.mindvision:
        camera = tracker.config["camera"]
        source = MindVisionCameraSource({
            "sdk_root": camera["sdk_root"], "resolution_index": camera["resolution_index"],
            "width": camera["expected_width"], "height": camera["expected_height"],
            "exposure_mode": camera["exposure_mode"], "exposure_us": camera["exposure_us"],
            "timeout_ms": camera["timeout_ms"],
            "ae_target": camera["ae_target"], "ae_exposure_min_us": camera["ae_exposure_min_us"],
            "ae_exposure_max_us": camera["ae_exposure_max_us"], "gain_min_raw": camera["gain_min_raw"],
            "gain_max_raw": camera["gain_max_raw"],
        })
        for _ in range(int(camera["warmup_frames"])):
            try:
                next(source)
            except SourceError as exc:
                key = "timeout_count" if "timeout" in str(exc).lower() else "capture_error_count"
                capture_counters[key] += 1
                print(f"MindVision warmup {key}: {exc}", file=sys.stderr)
        frames = _mindvision_frames(source, args.max_frames, capture_counters)
    elif args.input is None:
        parser.error("input is required unless --mindvision is selected")
    else:
        frames = _video(args.input) if args.video else _images(args.input)
    processed = 0
    try:
        for name, image in frames:
            result = tracker.process(image, diagnostics=args.save_overlay)
            if args.benchmark:
                report = _benchmark(tracker, image)
                args.output.mkdir(parents=True, exist_ok=True)
                (args.output / "benchmark.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
                print(json.dumps(report, indent=2))
                break
            _save(args.output, name, image, tracker, result, args.save_overlay, args.save_json)
            print(json.dumps({"frame": name, **result.to_dict()}, ensure_ascii=False))
            if args.display:
                shown = draw_stable_seam_overlay(image, result, (1280, 960))
                cv2.imshow("stable seam tracker", shown)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            processed += 1
            if args.max_frames and processed >= args.max_frames:
                break
    finally:
        if source is not None:
            source.close()
        if args.display:
            cv2.destroyAllWindows()
        if args.mindvision:
            print(json.dumps({"processed_frames": processed, **capture_counters}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
