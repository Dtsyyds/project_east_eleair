"""Low-rate, failure-isolated ROS 2 to Rerun bridge."""

from __future__ import annotations

import importlib
import math
from pathlib import Path
import time

import numpy as np
import rerun as rr
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMessage
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_msgs.msg import TFMessage

from .image_adapter import image_to_numpy
from .imu_adapter import imu_values
from .inspection_adapter import diagnostic_level, eddy_channels, fusion_scalars, ultrasound_channels
from .pointcloud_adapter import pointcloud_to_numpy
from .rerun_session import RerunSession
from .tf_adapter import clean_frame_id, transform_values
from .timestamp_utils import RateLimiter, TimelineTracker, message_timestamp_ns, stamp_to_nanoseconds


class RerunBridgeNode(Node):
    def __init__(self):
        super().__init__("rerun_bridge")
        defaults = {
            "output_mode": "spawn", "recording_id": "tank_inspection",
            "save_path": "output/rerun/inspection.rrd",
            "connect_url": "rerun+http://127.0.0.1:9876/proxy",
            "image_topic": "/camera/front/image_raw",
            "camera_info_topic": "/camera/front/camera_info",
            "pointcloud_topic": "/livox/lidar", "imu_topic": "/livox/imu",
            "tf_topic": "/tf", "tf_static_topic": "/tf_static",
            "fusion_index_topic": "/derived/inspection/fusion_index",
            "eddy_topic": "/inspection/eddy_current/raw",
            "ultrasound_topic": "/inspection/ultrasound/raw",
            "probe_state_topic": "/inspection/probe_state",
            "robot_state_topic": "/robot/state", "odometry_topic": "/robot/odometry",
            "path_topic": "/robot/path", "pose_topic": "/robot/pose",
            "diagnostics_topic": "/diagnostics",
            "bev_image_topic": "/derived/inspection/bev/image",
            "bev_coverage_topic": "/derived/inspection/bev/coverage",
            "bev_defects_topic": "/derived/inspection/bev/defects",
            "image_max_hz": 2.0, "pointcloud_max_hz": 1.0,
            "signal_max_hz": 2.0, "max_point_count": 100000,
            "point_stride": 1, "log_imu": True, "log_tf": True,
            "log_diagnostics": True, "log_bev": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._timeline = TimelineTracker()
        self._destroying = False
        self._limiters = {
            "image": RateLimiter(float(self._param("image_max_hz"))),
            "pointcloud": RateLimiter(float(self._param("pointcloud_max_hz"))),
            "signals": RateLimiter(float(self._param("signal_max_hz"))),
            "bev": RateLimiter(float(self._param("image_max_hz"))),
        }
        self._warned = set()
        self._last_error_log_ns = 0
        self._session = RerunSession(
            str(self._param("output_mode")), str(self._param("recording_id")),
            str(self._param("save_path")), str(self._param("connect_url")))
        # Do not shadow rclpy.Node's private subscription bookkeeping.
        self._bridge_subscriptions = []
        self._create_standard_subscriptions()
        self._create_optional_inspection_subscriptions()
        self._create_optional_vision_subscriptions()
        self.get_logger().info(
            f"Rerun bridge ready in {self._param('output_mode')} mode; visualization is not a data source")

    def _param(self, name):
        return self.get_parameter(name).value

    @staticmethod
    def _sensor_qos():
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

    @staticmethod
    def _reliable_qos(depth=5, transient=False):
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)

    def _subscribe(self, message_type, parameter, callback, qos):
        topic = str(self._param(parameter))
        if not topic:
            return
        self._bridge_subscriptions.append(self.create_subscription(message_type, topic, callback, qos))

    def _create_standard_subscriptions(self):
        sensor_qos = self._sensor_qos()
        self._subscribe(Image, "image_topic", self._image_callback, sensor_qos)
        self._subscribe(CameraInfo, "camera_info_topic", self._camera_info_callback, sensor_qos)
        self._subscribe(PointCloud2, "pointcloud_topic", self._pointcloud_callback, sensor_qos)
        if bool(self._param("log_imu")):
            self._subscribe(Imu, "imu_topic", self._imu_callback, sensor_qos)
        if bool(self._param("log_tf")):
            self._subscribe(TFMessage, "tf_topic", lambda msg: self._tf_callback(msg, False), sensor_qos)
            self._subscribe(TFMessage, "tf_static_topic", lambda msg: self._tf_callback(msg, True), self._reliable_qos(1, True))
        if bool(self._param("log_diagnostics")):
            self._subscribe(DiagnosticArray, "diagnostics_topic", self._diagnostics_callback, self._reliable_qos())
        self._subscribe(Odometry, "odometry_topic", self._odometry_callback, sensor_qos)
        self._subscribe(PathMessage, "path_topic", self._path_callback, sensor_qos)
        self._subscribe(PoseStamped, "pose_topic", self._pose_callback, sensor_qos)
        if bool(self._param("log_bev")):
            self._subscribe(Image, "bev_image_topic", lambda msg: self._image_callback(msg, True), sensor_qos)
            self._subscribe(Image, "bev_coverage_topic", lambda msg: self._image_callback(msg, True, "/world/bev/coverage"), sensor_qos)

    def _optional_message(self, module_name, class_name):
        try:
            return getattr(importlib.import_module(module_name), class_name)
        except (ImportError, AttributeError):
            self._warn_once(
                f"missing:{module_name}:{class_name}",
                f"Optional message {module_name}.{class_name} is unavailable; its topic will not be subscribed")
            return None

    def _create_optional_inspection_subscriptions(self):
        specs = [
            ("EddyCurrentFrame", "eddy_topic", self._eddy_callback),
            ("UltrasoundFrame", "ultrasound_topic", self._ultrasound_callback),
            ("ProbeState", "probe_state_topic", self._probe_callback),
            ("RobotState", "robot_state_topic", self._robot_callback),
            ("FusionIndex", "fusion_index_topic", self._fusion_callback),
        ]
        for class_name, parameter, callback in specs:
            message_type = self._optional_message("inspection_interfaces.msg", class_name)
            if message_type is not None:
                self._subscribe(message_type, parameter, callback, self._sensor_qos())

    def _create_optional_vision_subscriptions(self):
        defect_type = self._optional_message("tank_vision_interfaces.msg", "DefectArray")
        if defect_type is not None:
            self._subscribe(defect_type, "bev_defects_topic", self._defects_callback, self._sensor_qos())

    def _warn_once(self, key, message):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warning(message)

    def _begin(self, stream, message, explicit_ns=None):
        receive_ns = self.get_clock().now().nanoseconds
        timestamp_ns, used_receive = message_timestamp_ns(message, receive_ns) if explicit_ns is None else (int(explicit_ns), False)
        sequence, regressed = self._timeline.observe(stream, timestamp_ns)
        if used_receive:
            self._warn_once(f"zero_stamp:{stream}", f"{stream} has zero/no header stamp; using receive time")
        if regressed:
            now = time.monotonic_ns()
            if now - self._last_error_log_ns > 5_000_000_000:
                self._last_error_log_ns = now
                self.get_logger().warning(f"Timestamp regression detected on {stream}")
        self._session.set_time(timestamp_ns, sequence)
        return timestamp_ns

    def _guard(self, name, callback):
        try:
            callback()
        except Exception as exc:
            now = time.monotonic_ns()
            if now - self._last_error_log_ns > 2_000_000_000:
                self._last_error_log_ns = now
                self.get_logger().error(f"Rerun {name} adapter failed without affecting acquisition: {type(exc).__name__}: {exc}")

    def _image_callback(self, message, bev=False, path=None):
        def work():
            timestamp_ns, _ = message_timestamp_ns(message, self.get_clock().now().nanoseconds)
            limiter = self._limiters["bev" if bev else "image"]
            if not limiter.allow(timestamp_ns):
                return
            self._begin("bev_image" if bev else "camera_image", message)
            pixels, _ = image_to_numpy(message)
            self._session.log(path or ("/world/bev/image" if bev else "/world/camera/image"), rr.Image(pixels))
        self._guard("image", work)

    def _camera_info_callback(self, message):
        def work():
            self._begin("camera_info", message)
            matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
            if not np.any(matrix):
                self._warn_once("zero_camera_info", "CameraInfo intrinsics are all zero; no Pinhole projection will be logged")
                return
            self._session.log("/world/camera", rr.Pinhole(
                image_from_camera=matrix, resolution=[int(message.width), int(message.height)]))
        self._guard("camera_info", work)

    def _pointcloud_callback(self, message):
        def work():
            timestamp_ns, _ = message_timestamp_ns(message, self.get_clock().now().nanoseconds)
            if not self._limiters["pointcloud"].allow(timestamp_ns):
                return
            self._begin("pointcloud", message)
            points, intensity = pointcloud_to_numpy(
                message, int(self._param("point_stride")), int(self._param("max_point_count")))
            colors = None
            if intensity is not None and intensity.size:
                low, high = float(np.min(intensity)), float(np.max(intensity))
                scaled = np.zeros_like(intensity, dtype=np.uint8) if high <= low else np.clip((intensity - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
                colors = np.column_stack([scaled, scaled, scaled])
            self._session.log("/world/lidar", rr.Points3D(points, colors=colors))
            self._session.text("/world/lidar/metadata", f"frame_id={message.header.frame_id}; displayed_points={len(points)}")
        self._guard("pointcloud", work)

    def _imu_callback(self, message):
        def work():
            self._begin("imu", message)
            acceleration, angular, orientation = imu_values(message)
            for axis, value in zip("xyz", acceleration): self._session.scalar(f"/signals/imu/acceleration/{axis}", value)
            for axis, value in zip("xyz", angular): self._session.scalar(f"/signals/imu/angular_velocity/{axis}", value)
            if orientation is not None:
                for axis, value in zip("xyzw", orientation): self._session.scalar(f"/signals/imu/orientation/{axis}", value)
        self._guard("imu", work)

    def _tf_callback(self, message, is_static):
        for transform in message.transforms:
            def work(transform=transform):
                timestamp_ns = stamp_to_nanoseconds(transform.header.stamp) or self.get_clock().now().nanoseconds
                self._begin("tf_static" if is_static else "tf", transform, timestamp_ns)
                parent, child, xyz, xyzw = transform_values(transform)
                self._session.log(
                    f"/world/frames/{child}",
                    rr.Transform3D(translation=xyz, quaternion=rr.Quaternion(xyzw=xyzw), parent_frame=parent, child_frame=child),
                    static=is_static)
            self._guard("tf", work)

    def _odometry_callback(self, message):
        self._guard("odometry", lambda: self._log_pose(message, message.pose.pose, "/world/robot"))

    def _pose_callback(self, message):
        self._guard("pose", lambda: self._log_pose(message, message.pose, "/world/robot"))

    def _log_pose(self, message, pose, path):
        self._begin("robot_pose", message)
        q = pose.orientation
        values = [float(q.x), float(q.y), float(q.z), float(q.w)]
        norm = math.sqrt(sum(value * value for value in values))
        if norm <= 1.0e-12:
            raise ValueError("robot pose has invalid quaternion")
        self._session.log(path, rr.Transform3D(
            translation=[pose.position.x, pose.position.y, pose.position.z],
            quaternion=rr.Quaternion(xyzw=[value / norm for value in values])))

    def _path_callback(self, message):
        def work():
            self._begin("robot_path", message)
            points = [[pose.pose.position.x, pose.pose.position.y, pose.pose.position.z] for pose in message.poses]
            if points: self._session.log("/world/robot/path", rr.LineStrips3D([points]))
        self._guard("path", work)

    def _diagnostics_callback(self, message):
        def work():
            self._begin("diagnostics", message)
            for status in message.status:
                numeric_level = diagnostic_level(status.level)
                level = "ERROR" if numeric_level >= 2 else "WARN" if numeric_level == 1 else "INFO"
                name = str(status.name).replace("/", "_") or "unnamed"
                self._session.text(f"/system/diagnostics/{name}", f"{status.message}; hardware={status.hardware_id}", level)
        self._guard("diagnostics", work)

    def _eddy_callback(self, message):
        def work():
            timestamp_ns, _ = message_timestamp_ns(message, self.get_clock().now().nanoseconds)
            if not self._limiters["signals"].allow(timestamp_ns): return
            self._begin("eddy", message)
            signal_i, signal_q = eddy_channels(message)
            for channel in range(signal_i.shape[0]):
                sample_index = np.arange(signal_i.shape[1], dtype=np.float32)
                self._session.log(
                    f"/signals/eddy/channel_{channel}/i",
                    rr.LineStrips2D([np.column_stack([sample_index, signal_i[channel]])]))
                self._session.log(
                    f"/signals/eddy/channel_{channel}/q",
                    rr.LineStrips2D([np.column_stack([sample_index, signal_q[channel]])]))
            self._session.scalar("/signals/eddy/quality", message.quality_score)
            self._session.scalar("/signals/eddy/lift_off_mm", message.lift_off_mm)
        self._guard("eddy", work)

    def _ultrasound_callback(self, message):
        def work():
            timestamp_ns, _ = message_timestamp_ns(message, self.get_clock().now().nanoseconds)
            if not self._limiters["signals"].allow(timestamp_ns): return
            self._begin("ultrasound", message)
            channels = ultrasound_channels(message)
            sample_index = np.arange(channels.shape[1], dtype=np.float32)
            for channel in range(channels.shape[0]):
                self._session.log(
                    f"/signals/ultrasound/channel_{channel}",
                    rr.LineStrips2D([np.column_stack([sample_index, channels[channel]])]))
            self._session.scalar("/signals/ultrasound/coupling", message.coupling_score)
        self._guard("ultrasound", work)

    def _fusion_callback(self, message):
        def work():
            self._begin("fusion_index", message)
            values = fusion_scalars(message)
            self._session.scalar("/synchronization/camera_delta", values["camera_delta_ms"])
            self._session.scalar("/synchronization/lidar_delta", values["lidar_delta_ms"])
            self._session.scalar("/synchronization/imu_delta", values["imu_window_ms"])
            self._session.log("/synchronization/status", rr.AnyValues(
                camera_valid=values["camera_valid"], lidar_valid=values["lidar_valid"],
                imu_valid=values["imu_valid"], pose_valid=values["pose_valid"],
                invalid_reason=values["invalid_reason"], anchor_topic=str(message.anchor_topic),
                anchor_sequence=int(message.anchor_sequence), quality_flags=int(message.quality_flags)))
        self._guard("fusion_index", work)

    def _probe_callback(self, message):
        def work():
            self._begin("probe", message)
            self._session.scalar("/world/probe/contact_force_n", message.contact_force_n)
            self._session.scalar("/world/probe/lift_off_mm", message.lift_off_mm)
            self._session.scalar("/world/probe/coupling", message.coupling_score)
        self._guard("probe", work)

    def _robot_callback(self, message):
        def work():
            self._begin("robot_state", message)
            self._session.scalar("/world/robot/linear_speed_m_s", message.linear_speed_m_s)
            self._session.scalar("/world/robot/angular_speed_rad_s", message.angular_speed_rad_s)
            self._session.scalar("/world/robot/slip_ratio", message.slip_ratio)
        self._guard("robot_state", work)

    def _defects_callback(self, message):
        def work():
            self._begin("bev_defects", message)
            for defect in message.defects:
                self._session.log(f"/world/camera/detections/{defect.defect_id}", rr.AnyValues(
                    class_name=str(defect.class_name), confidence=float(defect.confidence),
                    bbox_xyxy=list(defect.bbox_xyxy), area_px=float(defect.area_px)))
        self._guard("defects", work)

    def destroy_node(self):
        if self._destroying:
            return False
        self._destroying = True
        try:
            self._session.close()
        except KeyboardInterrupt:
            pass
        except Exception as exc:
            self.get_logger().error(f"Rerun shutdown failed locally: {exc}")
        try:
            return super().destroy_node()
        except KeyboardInterrupt:
            # ros2 launch can forward a second SIGINT while rclpy is already
            # destroying subscriptions. The process is exiting either way.
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RerunBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
