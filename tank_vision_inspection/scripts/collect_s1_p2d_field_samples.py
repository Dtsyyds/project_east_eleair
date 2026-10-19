#!/usr/bin/env python3
"""Collect lossless 4096x3072 field frames for manual geometry review."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import cv2
import numpy as np

from tank_vision.sources.vendor_camera_source import MindVisionCameraSource


def _source_with_full_resolution_frame_speed(serial: str) -> tuple[MindVisionCameraSource, classmethod]:
    original_descriptor = MindVisionCameraSource.__dict__["_load_sdk"]
    original_loader = MindVisionCameraSource._load_sdk

    def loader(cls, root):
        sdk = original_loader(root)
        original_play = sdk.CameraPlay

        def play(handle):
            status = sdk.CameraSetFrameSpeed(handle, 0)
            if status != sdk.CAMERA_STATUS_SUCCESS:
                raise RuntimeError(f"CameraSetFrameSpeed(0) failed: {status}")
            return original_play(handle)

        sdk.CameraPlay = play
        return sdk

    MindVisionCameraSource._load_sdk = classmethod(loader)
    try:
        source = MindVisionCameraSource({
            "sdk_root": "/home/sunrise/east_eleair/MindVisionlinuxSDK",
            "serial_number": serial,
            "resolution_index": 0,
            "width": 4096,
            "height": 3072,
            "pixel_format": "bgr8",
            "exposure_mode": "auto",
            "exposure_us": None,
            "timeout_ms": 1500,
            "ae_target": 120,
            "ae_exposure_min_us": 100,
            "ae_exposure_max_us": 50000,
            "gain_min_raw": 2,
            "gain_max_raw": 16,
        })
        return source, original_descriptor
    except Exception:
        MindVisionCameraSource._load_sdk = original_descriptor
        raise


def collect(output: Path, frames: int, warmup: int, save_every: int, serial: str) -> dict:
    if frames < 1 or warmup < 0 or save_every < 1:
        raise ValueError("frames must be positive; warmup must be non-negative; save_every must be positive")
    output.mkdir(parents=True, exist_ok=False)
    source, original_loader_descriptor = _source_with_full_resolution_frame_speed(serial)
    records: list[dict] = []
    started = time.perf_counter()
    first_image = last_image = None
    try:
        for _ in range(warmup):
            next(source)
        for index in range(frames):
            frame = next(source)
            image = frame.image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            saved_path = None
            if index % save_every == 0 or index == frames - 1:
                saved_path = output / f"frame_{index:06d}.bmp"
                if not cv2.imwrite(str(saved_path), image):
                    raise OSError(f"failed to save {saved_path}")
            if first_image is None:
                first_image = image.copy()
            last_image = image.copy()
            records.append({
                "frame": index,
                "sequence_id": int(frame.sequence_id),
                "timestamp_ns": int(frame.timestamp_ns),
                "shape": list(image.shape),
                "saved_path": None if saved_path is None else saved_path.name,
                "gray_mean": float(gray.mean()),
                "gray_std": float(gray.std()),
                "gray_p01": float(np.percentile(gray, 1)),
                "gray_p50": float(np.percentile(gray, 50)),
                "gray_p99": float(np.percentile(gray, 99)),
                "saturated_ratio": float(np.mean(gray >= 255)),
                "underexposed_ratio": float(np.mean(gray <= 12)),
                "exposure_us": frame.exposure_us,
                "gain_db": frame.gain_db,
            })
    finally:
        source.close()
        MindVisionCameraSource._load_sdk = original_loader_descriptor
    timestamps = [item["timestamp_ns"] for item in records]
    manifest = {
        "status": "collected",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "serial_number": serial,
        "resolution_index": 0,
        "frame_speed_index": 0,
        "image_size": [4096, 3072],
        "pixel_format": "bgr8",
        "exposure_mode": "auto",
        "warmup_frames": warmup,
        "requested_frames": frames,
        "saved_frame_count": sum(item["saved_path"] is not None for item in records),
        "save_every": save_every,
        "duration_s": time.perf_counter() - started,
        "timestamp_monotonic": all(b > a for a, b in zip(timestamps, timestamps[1:])),
        "shape_consistent": len({tuple(item["shape"]) for item in records}) == 1,
        "precision_claimed": False,
        "tracker_run": False,
        "records": records,
    }
    (output / "collection_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--warmup", type=int, default=60)
    parser.add_argument("--save-every", type=int, default=6)
    parser.add_argument("--serial", default="046081820650")
    args = parser.parse_args()
    result = collect(args.output, args.frames, args.warmup, args.save_every, args.serial)
    print(json.dumps({key: result[key] for key in (
        "status", "serial_number", "image_size", "saved_frame_count", "duration_s",
        "timestamp_monotonic", "shape_consistent", "precision_claimed", "tracker_run",
    )}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
