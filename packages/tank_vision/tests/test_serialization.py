from __future__ import annotations

import json

from tank_vision.pipeline import VisionPipeline
from tank_vision.synthetic import rust_anomaly_image
from tank_vision.types import FrameData


def test_result_is_json_safe(config) -> None:
    result = VisionPipeline(config).process(FrameData(rust_anomaly_image(), 7, "camera", 0, "test", gain_db=1.25))
    encoded = json.dumps(result.to_dict(), allow_nan=False)
    decoded = json.loads(encoded)
    assert decoded["timestamp_ns"] == 7
    assert isinstance(decoded["seam_result"]["centerline_points"], list)

