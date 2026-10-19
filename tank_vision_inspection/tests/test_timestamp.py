from __future__ import annotations

from tank_vision.pipeline import VisionPipeline
from tank_vision.synthetic import seam_image
from tank_vision.types import FrameData


def test_acquisition_timestamp_is_unchanged(config) -> None:
    timestamp = 9_876_543_210_123_456
    frame = FrameData(seam_image(), timestamp, "optical_frame", 4, "test")
    result = VisionPipeline(config).process(frame)
    assert result.frame.timestamp_ns == timestamp
    assert result.seam_result.timestamp_ns == timestamp
    assert result.to_dict()["timestamp_ns"] == timestamp

