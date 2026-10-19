from __future__ import annotations

from contextlib import contextmanager
import math
import os
from pathlib import Path
import time
from typing import Any, Iterator
import uuid

import numpy as np
import pytest
import yaml


_DISCOVERY_TIMEOUT_SEC = 8.0
_MESSAGE_TIMEOUT_SEC = 12.0
_TOPICS = {
    "seam": "/vision/seam_state",
    "quality": "/vision/quality",
    "defects": "/vision/defects",
    "debug": "/vision/debug_image",
}


def _require_ros2_e2e_environment() -> None:
    if os.environ.get("TANK_VISION_RUN_ROS2_E2E") != "1":
        pytest.skip("set TANK_VISION_RUN_ROS2_E2E=1 to explicitly enable synthetic ROS 2 E2E")
    if os.environ.get("ROS_DISTRO") != "humble":
        pytest.skip("ROS_DISTRO=humble is required; source the clean Humble/TROS environment first")
    if not os.environ.get("AMENT_PREFIX_PATH"):
        pytest.skip("AMENT_PREFIX_PATH is missing; source Humble/TROS and install_s100p_humble first")


def _write_e2e_config(tmp_path: Path) -> Path:
    source = Path(__file__).parents[1] / "configs" / "default.yaml"
    raw = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise AssertionError("default.yaml must contain a mapping")
    inspection = dict(raw.get("inspection", {}))
    inspection.update({"enabled": True, "trigger_mode": "always"})
    raw["inspection"] = inspection
    defect = dict(raw.get("defect", {}))
    defect["backend"] = "classical"
    raw["defect"] = defect
    path = tmp_path / "ros2_e2e_config.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return path


def _spin_until(executor: Any, predicate: Any, description: str, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    while not predicate():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise AssertionError(f"timed out waiting for {description}")
        executor.spin_once(timeout_sec=min(0.1, remaining))


def _header_key(message: Any) -> tuple[int, int, str]:
    return (
        int(message.header.stamp.sec),
        int(message.header.stamp.nanosec),
        str(message.header.frame_id),
    )


def _assert_header(message: Any, expected: tuple[int, int, str]) -> None:
    assert _header_key(message) == expected


def _assert_finite(values: list[float] | tuple[float, ...]) -> None:
    assert all(math.isfinite(float(value)) for value in values)


@contextmanager
def _running_nodes(config_path: Path) -> Iterator[dict[str, Any]]:
    _require_ros2_e2e_environment()
    rclpy = pytest.importorskip("rclpy")
    pytest.importorskip("tank_vision_ros")
    pytest.importorskip("cv_bridge")

    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from tank_vision_interfaces.msg import DefectArray, SeamState, VisionQuality
    from tank_vision_ros.vision_node import TankVisionNode

    if rclpy.ok():
        raise RuntimeError("rclpy is already initialized; each E2E test requires an isolated context")

    previous_ros_log_dir = os.environ.get("ROS_LOG_DIR")
    os.environ["ROS_LOG_DIR"] = str(config_path.parent / "ros_log")
    executor = None
    vision = None
    probe = None
    try:
        rclpy.init(args=["--ros-args", "-p", f"config_file:={config_path}"])
        executor = SingleThreadedExecutor()
        vision = TankVisionNode()
        probe = Node(f"tank_vision_e2e_probe_{uuid.uuid4().hex[:8]}")
        messages: dict[str, list[Any]] = {name: [] for name in _TOPICS}
        subscriptions = [
            probe.create_subscription(SeamState, _TOPICS["seam"], messages["seam"].append, 10),
            probe.create_subscription(VisionQuality, _TOPICS["quality"], messages["quality"].append, 10),
            probe.create_subscription(DefectArray, _TOPICS["defects"], messages["defects"].append, 10),
            probe.create_subscription(Image, _TOPICS["debug"], messages["debug"].append, 10),
        ]
        publisher = probe.create_publisher(Image, "/vision/image_raw", qos_profile_sensor_data)
        output_publishers = [
            vision.seam_publisher,
            vision.quality_publisher,
            vision.defects_publisher,
            vision.debug_publisher,
        ]
        executor.add_node(vision)
        executor.add_node(probe)
        _spin_until(
            executor,
            lambda: publisher.get_subscription_count() >= 1
            and all(output_publisher.get_subscription_count() >= 1 for output_publisher in output_publishers),
            "input/output DDS endpoint matching",
            _DISCOVERY_TIMEOUT_SEC,
        )
        assert vision.config.inspection.enabled is True
        assert vision.config.inspection.trigger_mode == "always"
        assert vision.config.defect.backend == "classical"
        assert getattr(vision.inspection_pipeline.defect_detector, "backend_name", "") == "classical_surface_anomaly"
        yield {"bridge": CvBridge(), "executor": executor, "messages": messages, "publisher": publisher, "vision": vision}
    finally:
        if probe is not None and executor is not None:
            try:
                executor.remove_node(probe)
            except Exception:
                pass
        if vision is not None and executor is not None:
            try:
                executor.remove_node(vision)
            except Exception:
                pass
        if probe is not None:
            try:
                probe.destroy_node()
            except Exception:
                pass
        if vision is not None:
            try:
                vision.destroy_node()
            except Exception:
                pass
        if executor is not None:
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        if previous_ros_log_dir is None:
            os.environ.pop("ROS_LOG_DIR", None)
        else:
            os.environ["ROS_LOG_DIR"] = previous_ros_log_dir


def _image_message(bridge: Any, image: Any, expected: tuple[int, int, str]) -> Any:
    message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    message.header.stamp.sec = expected[0]
    message.header.stamp.nanosec = expected[1]
    message.header.frame_id = expected[2]
    return message


def _wait_for_one(executor: Any, messages: dict[str, list[Any]]) -> None:
    _spin_until(
        executor,
        lambda: all(messages[name] for name in _TOPICS),
        "one message on each visual output topic",
        _MESSAGE_TIMEOUT_SEC,
    )


def _validate_one_frame(
    state: dict[str, Any],
    expected: tuple[int, int, str],
    image_shape: tuple[int, int, int],
) -> tuple[Any, Any, Any, Any]:
    messages = state["messages"]
    selected = {}
    for name in _TOPICS:
        matching = [message for message in messages[name] if _header_key(message) == expected]
        assert len(matching) == 1, f"expected one {name} message for {expected}, got {len(matching)}"
        selected[name] = matching[0]
        _assert_header(matching[0], expected)

    quality = selected["quality"]
    assert quality.valid_image is True
    assert quality.valid_for_measurement is True
    _assert_finite(
        [
            quality.mean_brightness,
            quality.overexposed_ratio,
            quality.underexposed_ratio,
            quality.laplacian_variance,
            quality.contrast_std,
            quality.confidence_scale,
        ]
    )

    debug = selected["debug"]
    assert debug.encoding == "bgr8"
    assert (int(debug.width), int(debug.height)) == (image_shape[1], image_shape[0])
    assert int(debug.step) == image_shape[1] * 3
    decoded = state["bridge"].imgmsg_to_cv2(debug, desired_encoding="bgr8")
    assert decoded.shape == image_shape
    assert decoded.dtype == np.uint8

    return selected["seam"], quality, selected["defects"], debug


def _assert_internal_success(vision: Any, expected_processed: int) -> None:
    stats = vision.inspection_pipeline.stats.summary()
    assert stats["processed_frames"] >= expected_processed
    assert stats["failed_frames"] == 0
    assert vision.inspection_pipeline.defect_detector.last_warning is None


@pytest.mark.integration
@pytest.mark.ros2
def test_ros2_normal_weld_emits_all_outputs(tmp_path: Path) -> None:
    _require_ros2_e2e_environment()
    from tank_vision.synthetic import seam_image

    image = seam_image(640, 480, seed=11)
    expected = (1_700_000_001, 101, "synthetic_normal")
    with _running_nodes(_write_e2e_config(tmp_path)) as state:
        message = _image_message(state["bridge"], image, expected)
        state["publisher"].publish(message)
        _wait_for_one(state["executor"], state["messages"])
        seam, quality, defects, _debug = _validate_one_frame(state, expected, image.shape)
        assert seam.detected is True
        assert math.isfinite(float(seam.confidence))
        assert len(defects.defects) == 0
        _assert_internal_success(state["vision"], expected_processed=1)


@pytest.mark.integration
@pytest.mark.ros2
def test_ros2_surface_anomaly_emits_fine_detection(tmp_path: Path) -> None:
    _require_ros2_e2e_environment()
    from tank_vision.synthetic import rust_anomaly_image

    image = rust_anomaly_image(640, 480)
    expected = (1_700_000_002, 202, "synthetic_surface_anomaly")
    with _running_nodes(_write_e2e_config(tmp_path)) as state:
        state["publisher"].publish(_image_message(state["bridge"], image, expected))
        _wait_for_one(state["executor"], state["messages"])
        seam, _quality, defects, _debug = _validate_one_frame(state, expected, image.shape)
        assert seam.detected is True
        assert defects.defects, "classical backend did not emit a surface anomaly candidate"
        for defect in defects.defects:
            assert defect.class_name == "surface_anomaly"
            assert defect.backend_name == "classical_surface_anomaly"
            assert math.isfinite(float(defect.confidence))
            assert 0.0 <= float(defect.confidence) <= 1.0
            assert math.isfinite(float(defect.area_px))
            assert 0.0 < float(defect.area_px) <= image.shape[0] * image.shape[1]
            assert len(defect.bbox_xyxy) == 4
            x1, y1, x2, y2 = [int(value) for value in defect.bbox_xyxy]
            assert 0 <= x1 < x2 <= image.shape[1]
            assert 0 <= y1 < y2 <= image.shape[0]
            assert 0.0 <= float(defect.centroid_uv.x) <= image.shape[1]
            assert 0.0 <= float(defect.centroid_uv.y) <= image.shape[0]
            for point in defect.polygon:
                assert 0.0 <= float(point.x) <= image.shape[1]
                assert 0.0 <= float(point.y) <= image.shape[0]
        _assert_internal_success(state["vision"], expected_processed=1)


@pytest.mark.integration
@pytest.mark.ros2
def test_ros2_multiple_frames_preserve_headers_and_defect_association(tmp_path: Path) -> None:
    _require_ros2_e2e_environment()
    from tank_vision.synthetic import rust_anomaly_image, seam_image

    frames = [
        ((1_700_000_003, 303, "synthetic_normal_a"), seam_image(640, 480, center_x=260, seed=31), False),
        ((1_700_000_004, 404, "synthetic_anomaly_b"), rust_anomaly_image(640, 480), True),
        ((1_700_000_005, 505, "synthetic_normal_c"), seam_image(640, 480, center_x=390, slope=0.04, seed=33), False),
    ]
    expected_keys = {key for key, _image, _has_defect in frames}
    with _running_nodes(_write_e2e_config(tmp_path)) as state:
        for expected, image, _has_defect in frames:
            state["publisher"].publish(_image_message(state["bridge"], image, expected))
        _spin_until(
            state["executor"],
            lambda: all(len(state["messages"][name]) >= len(frames) for name in _TOPICS),
            "three complete frame output sets",
            _MESSAGE_TIMEOUT_SEC,
        )

        by_topic = {
            name: {_header_key(message): message for message in state["messages"][name]}
            for name in _TOPICS
        }
        for name in _TOPICS:
            assert set(by_topic[name]) == expected_keys
            assert len(state["messages"][name]) == len(frames)

        for expected, image, should_have_defect in frames:
            for name in _TOPICS:
                _assert_header(by_topic[name][expected], expected)
            quality = by_topic["quality"][expected]
            assert quality.valid_image is True
            assert quality.valid_for_measurement is True
            debug = by_topic["debug"][expected]
            assert debug.encoding == "bgr8"
            assert (int(debug.width), int(debug.height)) == (image.shape[1], image.shape[0])
            decoded = state["bridge"].imgmsg_to_cv2(debug, desired_encoding="bgr8")
            assert decoded.shape == image.shape
            defects = by_topic["defects"][expected]
            if should_have_defect:
                assert defects.defects
            else:
                assert not defects.defects
            for defect in defects.defects:
                assert defect.class_name == "surface_anomaly"
                assert defect.backend_name == "classical_surface_anomaly"
                assert 0 <= int(defect.bbox_xyxy[0]) < int(defect.bbox_xyxy[2]) <= image.shape[1]
                assert 0 <= int(defect.bbox_xyxy[1]) < int(defect.bbox_xyxy[3]) <= image.shape[0]
        _assert_internal_success(state["vision"], expected_processed=len(frames))
