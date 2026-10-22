from __future__ import annotations

from pathlib import Path

import numpy as np

from tank_vision.config import load_config
from tank_vision.defects.onnx_backend import ONNXBackend
from tank_vision.outputs import ResultWriter
from tank_vision.pipeline import VisionPipeline
from tank_vision.synthetic import rust_anomaly_image, seam_image
from tank_vision.types import FrameData


def test_empty_image_does_not_crash(config) -> None:
    frame = FrameData(np.empty((0, 0, 3), dtype=np.uint8), 12, "camera", 0, "test")
    result = VisionPipeline(config).process(frame)
    assert not result.quality_result.valid_image
    assert not result.seam_result.detected
    assert result.defect_results == []
    assert "empty_image" in result.warnings


def test_surface_anomaly_is_generic(config) -> None:
    result = VisionPipeline(config).process(FrameData(rust_anomaly_image(), 13, "camera", 0, "test"))
    assert result.defect_results
    assert {item.class_name for item in result.defect_results} == {"surface_anomaly"}


def test_output_tree_is_written(config, tmp_path: Path) -> None:
    pipeline = VisionPipeline(config)
    result = pipeline.process(FrameData(seam_image(), 14, "camera", 2, "test", original_filename="input.png"))
    with ResultWriter(tmp_path, config.pipeline) as writer:
        writer.write(result, pipeline.last_defect_mask)
    assert list((tmp_path / "json").glob("*.json"))
    assert list((tmp_path / "overlays").glob("*.jpg"))
    assert list((tmp_path / "masks").glob("*.png"))
    assert (tmp_path / "summary.csv").is_file()


def test_missing_onnx_model_is_nonfatal(config, tmp_path: Path) -> None:
    backend = ONNXBackend(tmp_path / "missing.onnx")
    assert not backend.available
    assert backend.error and "does not exist" in backend.error
    config.defect.backend = "onnx"
    config.defect.model_path = str(tmp_path / "missing.onnx")
    result = VisionPipeline(config).process(FrameData(seam_image(), 15, "camera", 0, "test"))
    assert result.defect_results == []
    assert any("onnx_detection_skipped" in warning for warning in result.warnings)
