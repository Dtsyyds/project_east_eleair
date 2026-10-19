"""Offline and OpenCV-camera command line entry point."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import logging
from pathlib import Path
import sys

from tank_vision.config import load_config
from tank_vision.outputs import ResultWriter
from tank_vision.metrics import PipelineStats
from tank_vision.pipeline import VisionPipeline
from tank_vision.sources import FolderSource, MindVisionCameraSource, OpenCVCameraSource, SourceError, VideoSource


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Tank weld seam and surface anomaly inspection MVP")
    parser.add_argument("--config", default="configs/default.yaml", help="YAML configuration path")
    parser.add_argument("--input", help="Input image folder or video file")
    parser.add_argument("--output", default="output", help="Output directory")
    parser.add_argument("--source", choices=("auto", "folder", "video", "camera", "mindvision"), default="auto")
    parser.add_argument("--camera-device", default="0", help="OpenCV device index or device path")
    parser.add_argument("--mindvision-sdk-root", help="Official MindVision Linux SDK root; overrides camera.sdk_root")
    parser.add_argument("--max-frames", type=int, default=0, help="Stop after N frames; 0 means all")
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    return parser


def _make_source(args: argparse.Namespace, config):
    mode = args.source
    if mode == "camera":
        device: int | str = int(args.camera_device) if str(args.camera_device).isdigit() else args.camera_device
        return OpenCVCameraSource(device, config.camera.width, config.camera.height)
    if mode == "mindvision":
        camera_options = asdict(config.camera)
        if args.mindvision_sdk_root:
            camera_options["sdk_root"] = args.mindvision_sdk_root
        return MindVisionCameraSource(camera_options)
    if not args.input:
        raise SourceError("--input is required unless --source camera is selected")
    input_path = Path(args.input)
    if mode == "auto":
        mode = "folder" if input_path.is_dir() else "video"
    return FolderSource(input_path) if mode == "folder" else VideoSource(input_path)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    try:
        config = load_config(args.config)
        pipeline = VisionPipeline(config)
        source = _make_source(args, config)
    except (OSError, ValueError, SourceError) as exc:
        logging.error("Initialization failed: %s", exc)
        return 2
    processed = 0
    successful = 0
    seams = 0
    defects = 0
    total_ms = 0.0
    run_stats = PipelineStats()
    try:
        with source, ResultWriter(args.output, config.pipeline) as writer:
            for frame in source:
                result = pipeline.process(frame)
                writer.write(result, pipeline.last_defect_mask)
                run_stats.record(result.metrics.timing)
                processed += 1
                successful += int(result.quality_result.valid_image)
                seams += int(result.seam_result.detected)
                defects += len(result.defect_results)
                total_ms += result.processing_time_ms
                if args.max_frames > 0 and processed >= args.max_frames:
                    break
    except KeyboardInterrupt:
        logging.info("Interrupted after %d frames", processed)
    except (OSError, SourceError) as exc:
        logging.error("Processing stopped: %s", exc)
        return 3
    detection_rate = seams / processed if processed else 0.0
    average_ms = total_ms / processed if processed else 0.0
    stats = run_stats.summary()
    algorithm = stats["stages"]["total_algorithm_ms"]
    disk = stats["stages"]["disk_write_ms"]
    ranked = sorted(
        (
            (name, values["mean"])
            for name, values in stats["stages"].items()
            if name not in {"total_algorithm_ms", "total_end_to_end_ms", "disk_write_ms", "writer_queue_ms"}
        ),
        key=lambda item: item[1],
        reverse=True,
    )
    print(f"处理帧数: {processed}")
    print(f"成功帧数: {successful}")
    print(f"焊缝检测率: {detection_rate:.2%}")
    print(f"异常区域数量: {defects}")
    print(f"平均处理时间: {average_ms:.2f} ms")
    print(
        "算法延迟 P50/P90/P95/最大: "
        f"{algorithm['p50']:.2f}/{algorithm['p90']:.2f}/{algorithm['p95']:.2f}/{algorithm['max']:.2f} ms"
    )
    print(f"平均算法 FPS: {stats['average_algorithm_fps']:.2f}")
    print(f"平均写盘耗时: {disk['mean']:.2f} ms（未计入算法耗时）")
    print("最耗时三个阶段: " + ", ".join(f"{name}={value:.2f} ms" for name, value in ranked[:3]))
    performance_path = Path(args.output) / "performance_summary.json"
    performance_path.write_text(json.dumps(stats, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"输出目录: {Path(args.output).resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
