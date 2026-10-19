from __future__ import annotations

from pathlib import Path


def test_stable_node_contract_is_isolated_and_latest_only() -> None:
    root = Path(__file__).parents[1]
    source = (root / "ros2_ws/src/tank_vision_ros/tank_vision_ros/stable_seam_node.py").read_text()
    assert "LatestFrameSlot" in source
    assert "ReliabilityPolicy.BEST_EFFORT" in source
    assert "HistoryPolicy.KEEP_LAST" in source and "depth=1" in source
    assert "message.header = header" in source
    assert "get_subscription_count() > 0" in source
    assert "cv2.imshow" not in source
    assert "MindVisionCameraSource" in source
    assert "create_service(Trigger, \"/stable_seam_node/save_snapshot\"" in source
    assert "Queue(maxsize=4)" in source
    assert "_snapshot_writer_loop" in source
    assert "snapshot_output_dir" in source


def test_existing_vision_node_not_replaced() -> None:
    root = Path(__file__).parents[1]
    source = (root / "ros2_ws/src/tank_vision_ros/tank_vision_ros/vision_node.py").read_text()
    assert "class TankVisionNode" in source
    assert "stable_seam" not in source


def test_seam_geometry_coordinates_are_documented_as_points() -> None:
    root = Path(__file__).parents[1]
    message = (root / "ros2_ws/src/tank_vision_interfaces/msg/SeamGeometry.msg").read_text()
    for field in ("left_boundary", "right_boundary", "centerline"):
        assert f"geometry_msgs/Point32[] {field}" in message


def test_viewer_snapshot_key_and_shutdown_are_outside_subscription_callback() -> None:
    root = Path(__file__).parents[1]
    source = (root / "ros2_ws/src/tank_vision_ros/tank_vision_ros/seam_debug_viewer.py").read_text()
    assert "ord(\"s\")" in source and "call_async(Trigger.Request())" in source
    assert "rclpy.shutdown()" in source
    callback = source.split("def _show", 1)[1].split("def _poll_key", 1)[0]
    assert "rclpy.shutdown()" not in callback
    assert "SingleThreadedExecutor" in source


def test_snapshot_copy_is_after_one_shot_request_check() -> None:
    root = Path(__file__).parents[1]
    source = (root / "ros2_ws/src/tank_vision_ros/tank_vision_ros/stable_seam_node.py").read_text()
    capture = source.split("def _capture_snapshot_if_requested", 1)[1].split("def _snapshot_writer_loop", 1)[0]
    assert capture.index("get_nowait()") < capture.index("image.copy()")
    assert "_snapshot_tasks: Queue[SnapshotTask | None] = Queue(maxsize=4)" in source
