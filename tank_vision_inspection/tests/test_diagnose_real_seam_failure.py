from __future__ import annotations

import copy
from pathlib import Path

import cv2

from scripts.diagnose_real_seam_failure import _run_variant
from tank_vision.config import load_config
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.types import FrameData


ROOT = Path(__file__).parents[1]
IMAGE = ROOT / "output/s1_b2c_positive_evidence_insufficient_20260907/representative_input.bmp"
PROFILE = ROOT / "configs/s100p_realtime_1920x1200_896.yaml"


def _image():
    image = cv2.imread(str(IMAGE), cv2.IMREAD_UNCHANGED)
    assert image is not None
    return image


def test_real_evidence_reproduces_first_minimum_response_gate() -> None:
    image = _image()
    config = load_config(PROFILE)
    result = RealtimeSeamPipeline(config).process(FrameData(image, 1, "camera", 0, "test"))
    diagnosis, _ = _run_variant(image, config, "test_896")

    assert image.shape == (1200, 1920, 3)
    assert result.seam_result.detected is False
    assert result.seam_result.lost_reason == "no_valid_component"
    assert result.metrics.timing.seam_geometry_ms == 0.0
    assert diagnosis["processing"]["processed_size"] == {"width": 896, "height": 560}
    assert diagnosis["threshold"]["minimum_response_gate_passed"] is False
    assert diagnosis["threshold"]["response_percentile_value"] == 1.0
    assert diagnosis["seam"]["geometry_not_executed_direct_reason"] == "response_percentile_below_minimum_response"
    assert diagnosis["seam"]["production_component_present"] is False
    assert diagnosis["seam"]["diagnostic_component_reconstruction_matches_production"] is True


def test_scale_and_channel_variants_do_not_change_production_result() -> None:
    image = _image()
    base = load_config(PROFILE)
    for dimension, channel, direction in ((1024, "gray", "vertical"), (896, "gray", "horizontal"),
                                           (896, "blue", "vertical"), (896, "green", "vertical"), (896, "red", "vertical")):
        config = copy.deepcopy(base)
        config.pipeline.thumbnail_max_dimension = dimension
        config.seam.color_channel = channel
        config.seam.expected_direction = direction
        diagnosis, _ = _run_variant(image, config, f"{dimension}_{channel}_{direction}")
        assert diagnosis["seam"]["detected"] is False
        assert diagnosis["threshold"]["minimum_response_gate_passed"] is False
        assert diagnosis["seam"]["geometry_not_executed_direct_reason"] == "response_percentile_below_minimum_response"
        assert diagnosis["timing_ms"]["seam_geometry_ms"] == 0.0


def test_diagnostic_reconstruction_is_read_only_for_protected_input() -> None:
    before = IMAGE.read_bytes()
    config = load_config(PROFILE)
    diagnosis, _ = _run_variant(_image(), config, "read_only")
    assert diagnosis["seam"]["diagnostic_component_reconstruction_matches_production"] is True
    assert IMAGE.read_bytes() == before
