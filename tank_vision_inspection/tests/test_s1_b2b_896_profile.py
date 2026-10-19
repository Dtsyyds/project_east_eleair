from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from tank_vision.config import load_config
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.ros_adapter import debug_image_subscribed
from tank_vision.synthetic import seam_image
from tank_vision.types import FrameData


ROOT = Path(__file__).parents[1]
PROFILE_1024 = ROOT / "configs" / "s100p_realtime_1920x1200.yaml"
PROFILE_896 = ROOT / "configs" / "s100p_realtime_1920x1200_896.yaml"


def _yaml(path: Path) -> dict[str, object]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def test_896_profile_only_changes_processing_scale() -> None:
    profile_1024 = _yaml(PROFILE_1024)
    profile_896 = _yaml(PROFILE_896)

    assert profile_1024["pipeline"]["thumbnail_max_dimension"] == 1024
    assert profile_896["pipeline"]["thumbnail_max_dimension"] == 896
    profile_896_as_1024 = {section: dict(values) for section, values in profile_896.items()}
    profile_896_as_1024["pipeline"]["thumbnail_max_dimension"] = 1024
    assert profile_896_as_1024 == profile_1024

    assert profile_1024["inspection"]["enabled"] is False
    assert profile_896["inspection"]["enabled"] is False
    assert profile_1024["pipeline"]["roi"] is None
    assert profile_896["pipeline"]["roi"] is None
    for key in ("save_overlay", "save_json", "save_masks"):
        assert profile_1024["pipeline"][key] is False
        assert profile_896["pipeline"][key] is False

    # This explicitly guards the semantic detection controls and both RANSAC sections.
    assert profile_896["seam"] == profile_1024["seam"]
    assert profile_896["laser"] == profile_1024["laser"]


def test_1024_and_896_restore_geometry_to_original_1920x1200_coordinates() -> None:
    image = seam_image(1920, 1200, center_x=960.0, slope=0.08, seed=2301)
    observed = {}
    for dimension, path in ((1024, PROFILE_1024), (896, PROFILE_896)):
        config = load_config(path)
        pipeline = RealtimeSeamPipeline(config)
        result = pipeline.process(FrameData(image, 1, "camera", 0, "synthetic"))
        seam = result.seam_result
        assert seam.detected
        assert seam.lateral_error_px is not None
        assert seam.heading_error_deg is not None
        assert 10.0 < seam.lateral_error_px < 30.0
        assert 1.0 < seam.heading_error_deg < 8.0
        assert len(seam.centerline_points) > 100
        points = np.asarray(seam.centerline_points, dtype=np.float64)
        assert np.all(points[:, 0] >= 0.0) and np.all(points[:, 0] < 1920.0)
        assert np.all(points[:, 1] >= 0.0) and np.all(points[:, 1] < 1200.0)
        # x is in the original image frame, not in the 896/1024 thumbnail frame.
        assert float(np.max(points[:, 0])) > 900.0
        observed[dimension] = seam

    assert abs(observed[1024].lateral_error_px - observed[896].lateral_error_px) < 3.0
    assert abs(observed[1024].heading_error_deg - observed[896].heading_error_deg) < 1.0


def test_disabled_inspection_and_debug_image_do_not_create_extra_work() -> None:
    for path in (PROFILE_1024, PROFILE_896):
        config = load_config(path)
        assert config.inspection.enabled is False
        assert config.pipeline.save_overlay is False
        assert config.pipeline.save_json is False
        assert config.pipeline.save_masks is False

    class Publisher:
        def get_subscription_count(self) -> int:
            return 5

    # The ROS callback checks this guard before calling create_overlay or CvBridge.
    assert debug_image_subscribed(False, Publisher()) is False
