from tank_vision.ros_adapter import (
    debug_image_subscribed,
    format_realtime_performance,
    ros_header_metadata,
    ros_stamp_to_timestamp_ns,
)


class _Publisher:
    def __init__(self, subscribers: int) -> None:
        self.subscribers = subscribers

    def get_subscription_count(self) -> int:
        return self.subscribers


def test_debug_image_creation_requires_parameter_and_subscriber() -> None:
    assert not debug_image_subscribed(False, _Publisher(1))
    assert not debug_image_subscribed(True, None)
    assert not debug_image_subscribed(True, _Publisher(0))
    assert debug_image_subscribed(True, _Publisher(1))


def test_realtime_performance_log_contains_required_metrics() -> None:
    stage = {"mean": 1.0, "p50": 0.8, "p95": 2.0, "max": 3.0}
    message = format_realtime_performance(
        120,
        100,
        20,
        {"stages": {"total_algorithm_ms": stage, "seam_detect_ms": stage, "seam_geometry_ms": stage}},
        5.68,
    )
    assert "input_frames=120" in message
    assert "real_results=100" in message
    assert "dropped_frames=20" in message
    assert "total_algorithm_ms(mean/p50/p95/max)=1.000/0.800/2.000/3.000" in message
    assert "seam_detect_ms=1.000" in message
    assert "seam_geometry_ms=1.000" in message
    assert "output_hz=5.680" in message


def test_ros_timestamp_is_preserved_exactly() -> None:
    assert ros_stamp_to_timestamp_ns(123, 456789) == 123_000_456_789


def test_ros_frame_id_is_preserved_exactly() -> None:
    timestamp, frame_id = ros_header_metadata(1, 2, "mindvision_optical_frame")
    assert timestamp == 1_000_000_002
    assert frame_id == "mindvision_optical_frame"
