"""ROS 2 Jazzy image-to-inspection node."""

from __future__ import annotations

from std_msgs.msg import Header
import threading
from pathlib import Path
from queue import Empty
import time

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tank_vision_interfaces.msg import DefectArray, SeamState, VisionQuality

from tank_vision.config import load_config
from tank_vision.outputs import create_overlay
from tank_vision import __version__
from tank_vision.inspection_pipeline import InspectionPipeline
from tank_vision.metrics import FrameMetrics
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.ros_adapter import (
    debug_image_subscribed,
    format_realtime_performance,
    ros_header_metadata,
)
from tank_vision.types import FrameData, VisionResult
from tank_vision_ros.conversions import defect_results_message, quality_message, seam_message


class TankVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("tank_vision_node")
        share = Path(get_package_share_directory("tank_vision_ros"))
        self.declare_parameter("config_file", str(share / "config" / "default.yaml"))
        self.declare_parameter("image_topic", "/vision/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("debug_image_topic", "/vision/debug_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("seam_topic", "/vision/seam_state")
        self.declare_parameter("defects_topic", "/vision/defects")
        self.declare_parameter("quality_topic", "/vision/quality")
        config_path = str(self.get_parameter("config_file").value)
        self.config = load_config(config_path)
        self.realtime_pipeline = RealtimeSeamPipeline(self.config)
        self.inspection_pipeline = InspectionPipeline(self.config)
        self.bridge = CvBridge()
        self.sequence = 0
        self.latest_camera_info: CameraInfo | None = None
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self._performance_input_frames = 0
        self._performance_result_frames = 0
        self._performance_first_result_ns: int | None = None
        self.seam_publisher = self.create_publisher(SeamState, str(self.get_parameter("seam_topic").value), 10)
        self.defects_publisher = self.create_publisher(
            DefectArray, str(self.get_parameter("defects_topic").value), 10
        )
        self.quality_publisher = self.create_publisher(
            VisionQuality, str(self.get_parameter("quality_topic").value), 10
        )
        self.debug_publisher = self.create_publisher(Image, str(self.get_parameter("debug_image_topic").value), 10)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_timer(0.01, self._publish_inspection_results)
        self.get_logger().info(f"Tank vision node started with config: {config_path}")

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self.latest_camera_info = message

    def _image_callback(self, message: Image) -> None:
        self._performance_input_frames += 1
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            timestamp_ns, frame_id = ros_header_metadata(
                int(message.header.stamp.sec), int(message.header.stamp.nanosec), message.header.frame_id
            )
            frame = FrameData(
                image=image,
                timestamp_ns=timestamp_ns,
                frame_id=frame_id,
                sequence_id=self.sequence,
                source_name=f"ros2:{self.image_topic}",
            )
            self.sequence += 1
            realtime = self.realtime_pipeline.process(frame)
            self._record_realtime_result(realtime)
            result = VisionResult(
                frame=frame,
                seam_result=realtime.seam_result,
                defect_results=[],
                quality_result=realtime.quality_result,
                processing_time_ms=realtime.metrics.timing.total_algorithm_ms,
                warnings=realtime.warnings,
                algorithm_version=__version__,
                config=self.config.to_dict(),
                metrics=FrameMetrics(timing=realtime.metrics.timing),
            )
            self.seam_publisher.publish(seam_message(result, message.header))
            self.quality_publisher.publish(quality_message(result, message.header))
            self.inspection_pipeline.submit(frame, self.realtime_pipeline.last_seam_mask)
            if debug_image_subscribed(self.publish_debug_image, self.debug_publisher):
                overlay = create_overlay(result)
                if overlay is not None:
                    debug_message = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                    debug_message.header = message.header
                    self.debug_publisher.publish(debug_message)
        except Exception as exc:
            self.get_logger().error(f"Frame processing failed without stopping node: {type(exc).__name__}: {exc}")

    def _record_realtime_result(self, realtime: object) -> None:
        """Record and periodically report synchronous real-time results."""
        self._performance_result_frames += 1
        now_ns = time.perf_counter_ns()
        if self._performance_first_result_ns is None:
            self._performance_first_result_ns = now_ns
        if self._performance_result_frames % 100 != 0:
            return
        summary = self.realtime_pipeline.stats.summary()
        elapsed_sec = max(0.0, (now_ns - self._performance_first_result_ns) / 1_000_000_000.0)
        output_hz = self._performance_result_frames / elapsed_sec if elapsed_sec > 0 else 0.0
        self.get_logger().info(
            format_realtime_performance(
                self._performance_input_frames,
                self._performance_result_frames,
                int(summary["dropped_frames"]),
                summary,
                output_hz,
            )
        )

    def _publish_inspection_results(self) -> None:
        while True:
            try:
                result = self.inspection_pipeline.get_result(timeout=0)
            except Empty:
                return
            header = Header()
            header.frame_id = result.frame.frame_id
            header.stamp.sec, header.stamp.nanosec = divmod(result.frame.timestamp_ns, 1_000_000_000)
            self.defects_publisher.publish(defect_results_message(result.defect_results, header))

    def destroy_node(self) -> bool:
        # The bounded output needs a consumer while normal shutdown drains inputs.
        draining = threading.Event()
        def drain():
            while not draining.is_set():
                self._publish_inspection_results()
                draining.wait(.01)
        consumer = threading.Thread(target=drain, daemon=True)
        consumer.start()
        try:
            self.inspection_pipeline.close()
            self._publish_inspection_results()
            self.realtime_pipeline.close()
        finally:
            draining.set()
            consumer.join(1)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TankVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
