"""ROS-independent metadata conversion used by the ROS 2 wrapper."""

from __future__ import annotations


def ros_stamp_to_timestamp_ns(sec: int, nanosec: int) -> int:
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("ROS stamp requires sec >= 0 and 0 <= nanosec < 1e9")
    return int(sec) * 1_000_000_000 + int(nanosec)


def ros_header_metadata(sec: int, nanosec: int, frame_id: str) -> tuple[int, str]:
    return ros_stamp_to_timestamp_ns(sec, nanosec), str(frame_id)


def debug_image_subscribed(publish_debug_image: bool, publisher: object | None) -> bool:
    """Return whether creating a debug image can result in a ROS publication."""
    if not publish_debug_image or publisher is None:
        return False
    return int(publisher.get_subscription_count()) > 0


def format_realtime_performance(
    input_frames: int,
    result_frames: int,
    dropped_frames: int,
    summary: dict[str, object],
    output_hz: float,
) -> str:
    """Format the low-frequency real-time performance report."""
    stages = summary["stages"]
    total = stages["total_algorithm_ms"]
    seam_detect = stages["seam_detect_ms"]
    seam_geometry = stages["seam_geometry_ms"]
    return (
        "Realtime performance: input_frames=%d real_results=%d dropped_frames=%d "
        "total_algorithm_ms(mean/p50/p95/max)=%.3f/%.3f/%.3f/%.3f "
        "seam_detect_ms=%.3f seam_geometry_ms=%.3f output_hz=%.3f"
        % (
            input_frames,
            result_frames,
            dropped_frames,
            total["mean"],
            total["p50"],
            total["p95"],
            total["max"],
            seam_detect["mean"],
            seam_geometry["mean"],
            output_hz,
        )
    )
