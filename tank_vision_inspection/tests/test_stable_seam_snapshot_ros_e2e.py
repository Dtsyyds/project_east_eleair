from __future__ import annotations

import json
import os
from pathlib import Path
import time

import cv2
import pytest


def _require() -> None:
    if os.environ.get("TANK_VISION_RUN_ROS2_E2E") != "1":
        pytest.skip("set TANK_VISION_RUN_ROS2_E2E=1 to explicitly enable ROS 2 E2E")
    if os.environ.get("ROS_DISTRO") != "humble" or not os.environ.get("AMENT_PREFIX_PATH"):
        pytest.skip("source the Humble/TROS and s100p install workspaces first")


@pytest.mark.integration
@pytest.mark.ros2
def test_topic_single_frame_snapshot_is_complete(tmp_path: Path) -> None:
    _require()
    rclpy = pytest.importorskip("rclpy")
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_srvs.srv import Trigger
    from tank_vision_ros.stable_seam_node import StableSeamNode

    if rclpy.ok():
        raise RuntimeError("rclpy is already initialized")
    root = Path(__file__).parents[1]
    image_path = root / "output/s1_real_seam_images/seam_center_lit_quality_01.bmp"
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    assert image is not None
    output_dir = tmp_path / "snapshots"
    old_domain = os.environ.get("ROS_DOMAIN_ID")
    old_localhost = os.environ.get("ROS_LOCALHOST_ONLY")
    os.environ["ROS_DOMAIN_ID"] = "83"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    old_ros_log_dir = os.environ.get("ROS_LOG_DIR")
    os.environ["ROS_LOG_DIR"] = str(tmp_path / "ros_log")
    try:
        rclpy.init(args=[
            "--ros-args", "-p", "input_mode:=ros_topic", "-p", f"snapshot_output_dir:={output_dir}",
            "-p", f"live_session_output_root:={tmp_path / 'live_sessions'}",
        ])
    except Exception:
        if old_ros_log_dir is None:
            os.environ.pop("ROS_LOG_DIR", None)
        else:
            os.environ["ROS_LOG_DIR"] = old_ros_log_dir
        if old_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = old_domain
        if old_localhost is None:
            os.environ.pop("ROS_LOCALHOST_ONLY", None)
        else:
            os.environ["ROS_LOCALHOST_ONLY"] = old_localhost
        raise
    executor = SingleThreadedExecutor()
    node = StableSeamNode()
    probe = Node("stable_snapshot_probe")
    bridge = CvBridge()
    publisher = probe.create_publisher(Image, "/camera/front/image_raw", qos_profile_sensor_data)
    client = probe.create_client(Trigger, "/stable_seam_node/save_snapshot")
    geometry_messages = []
    from tank_vision_interfaces.msg import SeamGeometry
    probe.create_subscription(SeamGeometry, "/vision/front/seam_geometry", geometry_messages.append, 10)
    executor.add_node(node)
    executor.add_node(probe)
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=0.1):
            executor.spin_once(timeout_sec=0.1)
        if not client.service_is_ready():
            pytest.skip("ROS transport/discovery unavailable in this environment")
        future = client.call_async(Trigger.Request())
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert future.result() is not None and future.result().success is True
        message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
        message.header.stamp.sec = 1788922627
        message.header.stamp.nanosec = 983752476
        message.header.frame_id = "snapshot_test"
        publisher.publish(message)
        while not list(output_dir.glob("*/metadata.json")) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        metadata_files = list(output_dir.glob("*/metadata.json"))
        assert len(metadata_files) == 1
        directory = metadata_files[0].parent
        metadata = json.loads(metadata_files[0].read_text())
        assert metadata["complete"] is True
        assert metadata["header_stamp"] == {"sec": 1788922627, "nanosec": 983752476}
        assert cv2.imread(str(directory / "raw.bmp"), cv2.IMREAD_COLOR).shape == image.shape
        assert cv2.imread(str(directory / "roi_raw.bmp"), cv2.IMREAD_COLOR).shape == (1782, 1229, 3)
        assert geometry_messages and geometry_messages[-1].header.frame_id == "snapshot_test"
    finally:
        executor.remove_node(probe)
        executor.remove_node(node)
        probe.destroy_node()
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        if old_ros_log_dir is None:
            os.environ.pop("ROS_LOG_DIR", None)
        else:
            os.environ["ROS_LOG_DIR"] = old_ros_log_dir
        if old_domain is None:
            os.environ.pop("ROS_DOMAIN_ID", None)
        else:
            os.environ["ROS_DOMAIN_ID"] = old_domain
        if old_localhost is None:
            os.environ.pop("ROS_LOCALHOST_ONLY", None)
        else:
            os.environ["ROS_LOCALHOST_ONLY"] = old_localhost
