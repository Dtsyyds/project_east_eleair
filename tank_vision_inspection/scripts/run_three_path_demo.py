#!/usr/bin/env python3
"""Run independent live seam, triggered inspection, and asynchronous recording paths."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision import __version__
from tank_vision.config import load_config
from tank_vision.inspection_pipeline import InspectionPipeline
from tank_vision.metrics import FrameMetrics, StageTiming
from tank_vision.outputs import AsyncResultWriter
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.sources import FolderSource, MindVisionCameraSource, VideoSource
from tank_vision.types import VisionResult


def _make_source(args: argparse.Namespace, config):
    if args.source == "mindvision":
        camera_options = asdict(config.camera)
        if args.mindvision_sdk_root:
            camera_options["sdk_root"] = args.mindvision_sdk_root
        return MindVisionCameraSource(camera_options)
    input_path = Path(args.input)
    mode = args.source
    if mode == "auto":
        mode = "folder" if input_path.is_dir() else "video"
    return FolderSource(input_path) if mode == "folder" else VideoSource(input_path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Three-path tank vision demo")
    parser.add_argument("--config", default="configs/default.yaml")
    parser.add_argument("--input", default="sample_data/synthetic", help="Image directory or video; ignored for MindVision")
    parser.add_argument("--source", choices=("auto", "folder", "video", "mindvision"), default="auto")
    parser.add_argument("--mindvision-sdk-root", help="Official MindVision SDK root; overrides camera.sdk_root")
    parser.add_argument("--output", default="output_three_path")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--repeat", type=int, default=1, help="Repeat finite file/video input for profiling")
    parser.add_argument("--pace-hz", type=float, default=None, help="Submission rate; defaults to 15 for files and camera-native speed for MindVision")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.repeat < 1:
        parser.error("--repeat must be positive")
    if args.pace_hz is not None and args.pace_hz < 0:
        parser.error("--pace-hz must be non-negative")
    realtime = RealtimeSeamPipeline(config)
    inspection = InspectionPipeline(config)
    submitted = 0
    started_monotonic = time.perf_counter()
    unpaired_inspection = 0
    pace_hz = args.pace_hz if args.pace_hz is not None else (0.0 if args.source == "mindvision" else 15.0)
    interval = 1.0 / pace_hz if pace_hz > 0 else 0.0
    next_deadline = time.perf_counter()
    stop = False
    repeat_count = 1 if args.source == "mindvision" else args.repeat
    # Complete file/video processing uses synchronous algorithms and a blocking
    # bounded writer. Live mode retains the independent latest-frame workers;
    # drain each bounded batch before accepting more demo frames.
    from dataclasses import replace
    from queue import Empty
    live = args.source == "mindvision"
    storage = config.storage if live else replace(config.storage, drop_policy="block")
    with AsyncResultWriter(args.output, config.pipeline, storage) as writer:
        def save(item, detailed):
            timing = StageTiming(**item.metrics.timing.to_dict())
            defects = []
            warnings = list(item.warnings)
            if detailed is not None:
                defects = detailed.defect_results
                warnings.extend(detailed.warnings)
                for stage in ("tiling_ms", "defect_inference_ms", "postprocess_ms"):
                    setattr(timing, stage, getattr(detailed.metrics.timing, stage))
            writer.submit(VisionResult(
                frame=item.frame, seam_result=item.seam_result, defect_results=defects,
                quality_result=item.quality_result,
                processing_time_ms=item.metrics.timing.total_algorithm_ms,
                warnings=warnings, algorithm_version=__version__, config=config.to_dict(),
                metrics=FrameMetrics(timing=timing)))

        def drain_live():
            nonlocal unpaired_inspection
            realtime.flush()
            inspection.flush()
            detailed = {}
            while True:
                try:
                    value = inspection.get_result(0)
                    detailed[value.frame.sequence_id] = value
                except Empty: break
            while True:
                try:
                    item = realtime.get_result(0)
                    save(item, detailed.pop(item.frame.sequence_id, None))
                except Empty: break
            unpaired_inspection += len(detailed)

        try:
            for _repeat_index in range(repeat_count):
                source = _make_source(args, config)
                with source:
                    for frame in source:
                        frame.sequence_id = submitted
                        if live:
                            realtime.submit(frame)
                            inspection.submit(frame)
                        else:
                            save(realtime.process(frame), inspection.process(frame)
                                 if inspection.should_trigger(frame) else None)
                        submitted += 1
                        if live and submitted % 2 == 0:
                            drain_live()
                        if args.max_frames > 0 and submitted >= args.max_frames:
                            stop = True
                            break
                        next_deadline += interval
                        remaining = next_deadline - time.perf_counter()
                        if remaining > 0: time.sleep(remaining)
                if stop: break
            if live: drain_live()
            writer.flush()
            writer_stats = writer.stats.summary()
            realtime_rates = realtime.rates
        finally:
            realtime.close()
            inspection.close()
    report = {
        "source": args.source,
        "pace_hz": pace_hz,
        "submitted_frames": submitted,
        "runtime_mode": "live_bounded_two_frame_batches" if live else "offline_complete_stream",
        "wall_processing_sec": time.perf_counter() - started_monotonic,
        "realtime_output_dropped": realtime.output_dropped_frames,
        "inspection_unpaired_with_realtime": unpaired_inspection,
        "lifecycle": {"realtime": realtime.lifecycle_status, "inspection": inspection.lifecycle_status},
        "realtime": {**realtime.stats.summary(), **realtime_rates},
        "inspection": inspection.stats.summary(),
        "writer": writer_stats,
        "note": (
            "live MindVision hardware run" if args.source == "mindvision"
            else "file/video software-path measurement; no industrial-camera hardware result is implied"
        ),
    }
    (Path(args.output) / "three_path_summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
