"""ROS 2 node for the independent fixed-ROI stable seam tracker."""

from __future__ import annotations

import copy
import math
from pathlib import Path
from queue import Empty, Full, Queue
import shutil
from collections import deque
from threading import BoundedSemaphore, Event, Lock, Thread
import time

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point32
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tank_vision_interfaces.msg import SeamGeometry, SeamState

from tank_vision.latest_frame import LatestFrameSlot
from tank_vision.seam_tracking_types import StableSeamResult
from tank_vision.sources import MindVisionCameraSource, SourceError
from tank_vision.stable_seam_tracker import StableSeamTracker, draw_stable_seam_overlay
from tank_vision.snapshot_writer import SnapshotTask, write_snapshot
from tank_vision.failure_event_recorder import (FailureEventTask, LiveSessionRecorder,
                                                gate_evidence, human_readable_reason)
from tank_vision.roi_validation_recorder import ROIValidationRecorder
from tank_vision.geometry_provenance_recorder import GeometryProvenanceRecorder, ProvenanceTask
from tank_vision.types import FrameData


SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)


class InputPacket:
    """One successfully received frame; the packet is the only latest-slot item."""

    def __init__(self, header: Header, frame: FrameData, captured_index: int,
                 image_callback_entry_ns: int | None = None,
                 image_conversion_ms: float | None = None) -> None:
        self.header = header
        self.frame = frame
        self.captured_index = captured_index
        self.host_timestamp_ns = time.time_ns()
        self.image_callback_entry_ns = image_callback_entry_ns
        self.image_callback_exit_ns: int | None = None
        self.image_conversion_ms = image_conversion_ms
        self.callback_total_ms: float | None = None


def _bool_parameter(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)


def np_isfinite(value: object) -> bool:
    try:
        return bool(np.isfinite(float(value)))
    except (TypeError, ValueError):
        return False


def np_round_points(points: list[tuple[float, float]]) -> np.ndarray:
    return np.round(np.asarray(points, dtype=np.float32)).astype(np.int32).reshape(-1, 1, 2)


def _display_value(value: object) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "N/A"
    return "N/A" if not math.isfinite(number) else f"{number:.2f}"


def _session_disk_usage(path: Path) -> int:
    total = 0
    if not path.exists():
        return total
    for item in path.rglob("*"):
        if item.is_file():
            try:
                total += item.stat().st_size
            except OSError:
                pass
    return total


def _distribution(values: deque[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    data = np.asarray(values, dtype=np.float64)
    return {"count": len(data), "p50": float(np.percentile(data, 50)),
            "p95": float(np.percentile(data, 95)), "max": float(np.max(data))}


PERFORMANCE_TIMING_FIELDS = (
    "image_conversion_ms", "callback_total_ms", "packet_queue_wait_ms",
    "tracker_process_ms", "provenance_second_pass_ms", "provenance_task_build_ms",
    "snapshot_task_build_ms", "publish_geometry_ms", "publish_state_ms",
    "callback_to_output_ms", "process_packet_wall_ms",
)


def _finite(value: float | None) -> float:
    return float(value) if value is not None else math.nan


def _json_float(value: float) -> float | None:
    value = float(value)
    return value if math.isfinite(value) else None


def seam_geometry_message(result: StableSeamResult, header: Header, input_frames: int,
                          processed_frames: int, dropped_frames: int) -> SeamGeometry:
    message = SeamGeometry()
    message.header = header
    message.detected = result.detected
    message.predicted_only = result.predicted_only
    message.tracking_state = int(result.tracking_state)
    message.confidence = result.confidence
    x, y, width, height = result.roi_original_px
    message.roi.x_offset, message.roi.y_offset = x, y
    message.roi.width, message.roi.height = width, height
    message.left_boundary = [Point32(x=float(px), y=float(py), z=0.0) for px, py in result.left_boundary_original_px]
    message.right_boundary = [Point32(x=float(px), y=float(py), z=0.0) for px, py in result.right_boundary_original_px]
    message.centerline = [Point32(x=float(px), y=float(py), z=0.0) for px, py in result.centerline_original_px]
    message.median_width_px = _finite(result.median_width_px)
    message.lateral_error_px = _finite(result.lateral_error_px)
    message.heading_error_deg = _finite(result.heading_error_deg)
    message.coverage = result.coverage
    message.effective_span = result.effective_span
    message.fit_residual_px = _finite(result.fit_residual_px)
    message.processing_ms = result.processing_ms
    message.input_frames, message.processed_frames, message.dropped_frames = input_frames, processed_frames, dropped_frames
    message.failure_reasons = result.failure_reasons
    return message


def compatibility_seam_message(result: StableSeamResult, header: Header) -> SeamState:
    message = SeamState()
    message.header = header
    message.detected = result.detected
    message.confidence = result.confidence
    message.centerline_points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in result.centerline_original_px]
    message.lateral_error_px = _finite(result.lateral_error_px)
    message.heading_error_deg = _finite(result.heading_error_deg)
    message.seam_width_px = _finite(result.median_width_px)
    message.lost_reason = ";".join(result.failure_reasons) if not result.detected else ""
    message.processing_time_ms = result.processing_ms
    return message


class StableSeamNode(Node):
    def __init__(self) -> None:
        super().__init__("stable_seam_node")
        share = Path(get_package_share_directory("tank_vision_ros"))
        self.declare_parameter("input_mode", "ros_topic")
        self.declare_parameter("config_file", str(share / "config/s100p_stable_seam_4096x3072.yaml"))
        self.declare_parameter("reference_file", str(share / "config/seam_width_reference_s100p_v1.json"))
        self.declare_parameter("image_topic", "/camera/front/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/front/camera_info")
        self.declare_parameter("geometry_topic", "/vision/front/seam_geometry")
        self.declare_parameter("seam_state_topic", "/vision/front/seam_state")
        self.declare_parameter("debug_image_topic", "/vision/front/debug_image")
        self.declare_parameter("publish_debug_image", False)
        self.declare_parameter(
            "snapshot_output_dir",
            "/media/mcap_data/vision/stable_seam_failure_snapshots",
        )
        self.declare_parameter(
            "live_session_output_root",
            "/media/mcap_data/vision/stable_seam_live_sessions",
        )
        self.declare_parameter("auto_save_failures", True)
        self.declare_parameter("failure_snapshot_cooldown_sec", 2.0)
        self.declare_parameter("failure_snapshot_queue_size", 4)
        self.declare_parameter("max_failure_snapshots_per_session", 100)
        self.declare_parameter("save_on_failure_reason_change", True)
        self.declare_parameter("save_on_state_transition", True)
        self.declare_parameter("save_on_new_worst_metric", True)
        self.declare_parameter("auto_save_full_frame", False)
        self.declare_parameter("auto_save_context_jpeg", True)
        self.declare_parameter("context_jpeg_quality", 90)
        self.declare_parameter("save_lossless_roi", True)
        self.declare_parameter("auto_save_policy", "event")
        self.declare_parameter("failure_continuous_sec", 2.0)
        self.declare_parameter("per_frame_metrics_log", True)
        self.declare_parameter("roi_validation_recording_enabled", False)
        self.declare_parameter("roi_validation_output_root", "/media/mcap_data/runs/roi_validation")
        self.declare_parameter("roi_validation_queue_size", 8)
        self.declare_parameter("roi_validation_max_duration_sec", 0.0)
        self.declare_parameter("geometry_provenance_sampling_enabled", False)
        self.declare_parameter("geometry_provenance_output_root", "/media/mcap_data/runs/geometry_provenance")
        self.declare_parameter("geometry_provenance_queue_size", 8)
        self.declare_parameter("geometry_provenance_sample_period_sec", 5.0)
        self.declare_parameter("geometry_provenance_max_events", 30)
        self.mode = str(self.get_parameter("input_mode").value)
        if self.mode not in {"ros_topic", "mindvision_direct"}:
            raise ValueError("input_mode must be ros_topic or mindvision_direct")
        config_path = str(self.get_parameter("config_file").value)
        reference_path = str(self.get_parameter("reference_file").value)
        self.tracker = StableSeamTracker(config_path, reference_path)
        self._tuning_lock = Lock()
        self._tunable_parameter_paths = {
            "tune_minimum_point_count": ("geometry", "minimum_point_count"),
            "tune_minimum_coverage": ("geometry", "minimum_coverage"),
            "tune_minimum_effective_span": ("geometry", "minimum_effective_span"),
            "tune_minimum_inlier_ratio": ("geometry", "minimum_inlier_ratio"),
            "tune_maximum_fit_residual_px": ("geometry", "maximum_fit_residual_px"),
            "tune_minimum_path_confidence": ("geometry", "minimum_path_confidence"),
            "tune_geometry_maximum_missing_run": ("geometry", "maximum_missing_run"),
            "tune_edge_min_response": ("processing", "edge_min_response"),
            "tune_edge_percentile": ("processing", "edge_percentile"),
            "tune_edge_min_separation_px": ("processing", "edge_min_separation_px"),
            "tune_guided_search_radius_width_scale": ("guided_search", "local_search_radius_width_scale"),
        }
        self._tunable_parameter_limits = {
            "tune_minimum_point_count": (1, 10000, int),
            "tune_minimum_coverage": (0.0, 1.0, float),
            "tune_minimum_effective_span": (0.0, 1.0, float),
            "tune_minimum_inlier_ratio": (0.0, 1.0, float),
            "tune_maximum_fit_residual_px": (0.0, 1000.0, float),
            "tune_minimum_path_confidence": (0.0, 1.0, float),
            "tune_geometry_maximum_missing_run": (0, 10000, int),
            "tune_edge_min_response": (0.0, 255.0, float),
            "tune_edge_percentile": (0.0, 100.0, float),
            "tune_edge_min_separation_px": (0, 1000, int),
            "tune_guided_search_radius_width_scale": (0.0, 10.0, float),
        }
        self._unavailable_tunable_parameters = {}
        for parameter_name, path in list(self._tunable_parameter_paths.items()):
            target = self.tracker.config
            try:
                for key in path:
                    target = target[key]
            except KeyError:
                # Stable configs do not have guided-anchor-only tuning fields.
                # Do not invent a value or change the algorithm configuration.
                self._unavailable_tunable_parameters[parameter_name] = path
                del self._tunable_parameter_paths[parameter_name]
                continue
            self.declare_parameter(parameter_name, target)
        self.add_on_set_parameters_callback(self._tuning_parameter_callback)
        self.bridge = CvBridge()
        self.slot: LatestFrameSlot[InputPacket] = LatestFrameSlot()
        self.stop_event = Event()
        self.captured_frames = 0
        self._accept_snapshot_requests = True
        self._snapshot_lock = Lock()
        self._snapshot_capacity = BoundedSemaphore(4)
        self._snapshot_requests: Queue[float] = Queue(maxsize=4)
        self._snapshot_tasks: Queue[SnapshotTask | None] = Queue(maxsize=4)
        self._snapshot_writer = Thread(target=self._snapshot_writer_loop, name="stable-seam-snapshot-writer", daemon=True)
        self._snapshot_writer.start()
        self.processed_frames = 0
        self.detected_frames = 0
        self.capture_errors = 0
        self.timeout_errors = 0
        self.state_counts: dict[str, int] = {"LOST": 0, "ACQUIRING": 0, "TRACKING": 0, "DEGRADED": 0}
        self.failure_reason_counts: dict[str, int] = {}
        self._state_history: list[str] = []
        self._previous_state = "LOST"
        self._previous_failure = False
        self._previous_failure_reasons: tuple[str, ...] = ()
        self._failure_started_at: float | None = None
        self._last_failure_snapshot_at = -float("inf")
        self._worst_joint_inlier = float("inf")
        self._worst_residual = -float("inf")
        self._auto_snapshot_requested = 0
        self._auto_snapshot_rejected = 0
        self._manual_snapshot_requested = 0
        self._manual_snapshot_rejected = 0
        self._manual_snapshot_enqueued = 0
        self._manual_snapshot_saved = 0
        self._last_frame_packet: InputPacket | None = None
        self._provenance_capacity_guard_skips = 0
        self._provenance_second_pass_count = 0
        self._provenance_task_build_count = 0
        self._processing_samples: deque[float] = deque(maxlen=4096)
        self._performance_samples: dict[str, deque[float]] = {
            field: deque(maxlen=4096) for field in PERFORMANCE_TIMING_FIELDS
        }
        # Keep a bounded timestamp audit trail.  The final input/output rates
        # are computed on their common active window, excluding startup and
        # shutdown gaps.  200k samples cover more than five hours at 10 Hz.
        self._input_host_timestamps_ns: deque[int] = deque(maxlen=200_000)
        self._output_host_timestamps_ns: deque[int] = deque(maxlen=200_000)
        self._processing_started_ns: int | None = None
        self._processing_ended_ns: int | None = None
        self._capture_started_ns: int | None = None
        self._capture_ended_ns: int | None = None
        self._shutdown_started_ns: int | None = None
        self._camera_info_received = False
        self._camera_info_valid = False
        self._camera_telemetry_lock = Lock()
        self._last_camera_telemetry: dict[str, object] = {}
        self._live_recorder = LiveSessionRecorder(
            str(self.get_parameter("live_session_output_root").value), config_path, reference_path,
            {name: self.get_parameter(name).value for name in (
                "auto_save_failures", "failure_snapshot_cooldown_sec", "failure_snapshot_queue_size",
                "max_failure_snapshots_per_session", "auto_save_full_frame", "auto_save_context_jpeg",
                "save_lossless_roi", "auto_save_policy")},
            snapshot_queue_size=int(self.get_parameter("failure_snapshot_queue_size").value),
            max_snapshots=int(self.get_parameter("max_failure_snapshots_per_session").value),
        )
        self.get_logger().info(f"live session: {self._live_recorder.session}")
        if not self._live_recorder.startup_space_ok:
            self.get_logger().warning("less than 1 GB free; automatic failure images are disabled, metrics continue")
        self._roi_validation_recorder: ROIValidationRecorder | None = None
        if _bool_parameter(self.get_parameter("roi_validation_recording_enabled").value):
            self._roi_validation_recorder = ROIValidationRecorder(
                self.get_parameter("roi_validation_output_root").value,
                config_path,
                reference_path,
                queue_size=int(self.get_parameter("roi_validation_queue_size").value),
                max_duration_sec=float(self.get_parameter("roi_validation_max_duration_sec").value),
            )
            self.get_logger().info(f"ROI validation recorder: {self._roi_validation_recorder.session_directory}")
        self._geometry_provenance_recorder: GeometryProvenanceRecorder | None = None
        if _bool_parameter(self.get_parameter("geometry_provenance_sampling_enabled").value):
            self._geometry_provenance_recorder = GeometryProvenanceRecorder(
                self.get_parameter("geometry_provenance_output_root").value,
                config_path, reference_path,
                queue_size=int(self.get_parameter("geometry_provenance_queue_size").value),
                sample_period_sec=float(self.get_parameter("geometry_provenance_sample_period_sec").value),
                max_events=int(self.get_parameter("geometry_provenance_max_events").value),
            )
            self.get_logger().info(f"geometry provenance recorder: {self._geometry_provenance_recorder.session_directory}")
        self.geometry_publisher = self.create_publisher(SeamGeometry, str(self.get_parameter("geometry_topic").value), 10)
        self.seam_publisher = self.create_publisher(SeamState, str(self.get_parameter("seam_state_topic").value), 10)
        self.debug_publisher = self.create_publisher(Image, str(self.get_parameter("debug_image_topic").value), SENSOR_QOS)
        self.diagnostics_subscription = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._diagnostics_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10))
        # Keep the public name explicit so launch namespaces/remappings cannot
        # accidentally move the service away from the documented endpoint.
        self.snapshot_service = self.create_service(Trigger, "/stable_seam_node/save_snapshot", self._save_snapshot_callback)
        self.get_logger().info("snapshot service ready: /stable_seam_node/save_snapshot")
        if self.mode == "ros_topic":
            self.subscription = self.create_subscription(Image, str(self.get_parameter("image_topic").value), self._image_callback, SENSOR_QOS)
            self.camera_info_subscription = self.create_subscription(
                CameraInfo, str(self.get_parameter("camera_info_topic").value),
                self._camera_info_callback, SENSOR_QOS)
            target = self._topic_worker
        else:
            self.get_logger().info(
                "mindvision_direct owns the camera; do not run mindvision_camera_node concurrently. "
                "Full-resolution raw image publication is disabled.")
            # Camera acquisition is started below.  The common processing
            # worker only drains the latest-frame slot for both input modes.
            target = self._topic_worker
        self.worker = Thread(target=target, name="stable-seam-processing-worker", daemon=True)
        self.worker.start()
        self.capture_worker: Thread | None = None
        if self.mode == "mindvision_direct":
            self.capture_worker = Thread(target=self._direct_capture_worker, name="stable-seam-camera-worker", daemon=True)
            self.capture_worker.start()

    def _save_snapshot_callback(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        with self._snapshot_lock:
            accepting = self._accept_snapshot_requests and not self.stop_event.is_set()
            rejection_reason = "snapshot request queue full or node is stopping"
            if accepting and shutil.disk_usage(self._live_recorder.session).free < 500 * 1024 * 1024:
                accepting = False
                rejection_reason = "low_disk_space_below_500MB"
            if accepting:
                try:
                    if not self._snapshot_capacity.acquire(blocking=False):
                        accepting = False
                        rejection_reason = "snapshot writer queue full"
                    else:
                        self._snapshot_requests.put_nowait(time.time())
                except Full:
                    self._snapshot_capacity.release()
                    accepting = False
                    rejection_reason = "snapshot request queue full"
            self._manual_snapshot_requested += 1
            if not accepting:
                self._manual_snapshot_rejected += 1
        response.success = bool(accepting)
        response.message = "queued" if accepting else rejection_reason
        if accepting:
            self.get_logger().info("snapshot request queued")
        else:
            self.get_logger().warning(response.message)
        return response

    def _image_callback(self, message: Image) -> None:
        callback_entry_ns = time.time_ns()
        conversion_start_ns = time.perf_counter_ns()
        try:
            # cv_bridge keeps the ROS message buffer alive through the
            # returned ndarray view.  Avoiding a second 37.7 MB full-frame
            # copy is important on RK3588; the worker consumes the latest
            # packet and never mutates this array.
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            image_conversion_ms = (time.perf_counter_ns() - conversion_start_ns) / 1e6
            header = copy.deepcopy(message.header)
            timestamp_ns = int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)
            frame = FrameData(image=image, timestamp_ns=timestamp_ns, frame_id=str(header.frame_id),
                              sequence_id=self.captured_frames, source_name="ros_topic")
            self._enqueue_packet(InputPacket(
                header, frame, self.captured_frames + 1,
                image_callback_entry_ns=callback_entry_ns,
                image_conversion_ms=image_conversion_ms,
            ))
        except Exception as exc:
            self.get_logger().error(f"image conversion failed: {exc}")

    def _camera_info_callback(self, message: CameraInfo) -> None:
        self._camera_info_received = True
        self._camera_info_valid = bool(
            len(message.k) == 9 and any(abs(float(value)) > 1e-12 for value in message.k)
            and len(message.p) == 12 and any(abs(float(value)) > 1e-12 for value in message.p)
        )
        if not self._camera_info_valid:
            self.get_logger().warning(
                "CameraInfo is present but uncalibrated (K/P are zero); pixel tracking only",
                throttle_duration_sec=30.0,
            )

    def _diagnostics_callback(self, message: DiagnosticArray) -> None:
        received_ns = time.time_ns()
        for status in message.status:
            name = str(status.name).lower()
            if "mindvision" not in name and "camera" not in name:
                continue
            values = {str(item.key): str(item.value) for item in status.values}
            if not any(key in values for key in ("exposure_us", "analog_gain", "analog_gain_raw", "camera_model")):
                continue
            stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            with self._camera_telemetry_lock:
                self._last_camera_telemetry = {
                    "diagnostic_name": str(status.name), "values": values,
                    "telemetry_stamp_ns": stamp_ns, "telemetry_received_host_ns": received_ns,
                }

    def _tuning_parameter_callback(self, parameters) -> SetParametersResult:
        updates: list[tuple[tuple[str, ...], object]] = []
        for parameter in parameters:
            if parameter.name not in self._tunable_parameter_paths:
                continue
            lower, upper, value_type = self._tunable_parameter_limits[parameter.name]
            try:
                value = value_type(parameter.value)
                if not math.isfinite(float(value)) or not lower <= value <= upper:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{parameter.name} must be in [{lower}, {upper}]",
                    )
            except (TypeError, ValueError, OverflowError):
                return SetParametersResult(successful=False, reason=f"invalid value for {parameter.name}")
            updates.append((self._tunable_parameter_paths[parameter.name], value))
        with self._tuning_lock:
            for path, value in updates:
                target = self.tracker.config
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = value
        if updates:
            self.get_logger().info(
                "runtime tuning updated: " + ", ".join(f"{path[-1]}={value}" for path, value in updates))
        return SetParametersResult(successful=True, reason="accepted")

    def _enqueue_packet(self, packet: InputPacket) -> None:
        if self.stop_event.is_set():
            return
        self.captured_frames += 1
        self._capture_started_ns = packet.host_timestamp_ns if self._capture_started_ns is None else self._capture_started_ns
        self._capture_ended_ns = packet.host_timestamp_ns
        self._input_host_timestamps_ns.append(packet.host_timestamp_ns)
        packet.captured_index = self.captured_frames
        if packet.image_callback_entry_ns is not None:
            packet.image_callback_exit_ns = time.time_ns()
            packet.callback_total_ms = (packet.image_callback_exit_ns - packet.image_callback_entry_ns) / 1e6
        self.slot.put(packet)

    def _publish(self, header: Header, image: object) -> None:
        """Compatibility entry point: enqueue work; never run tracker inline."""
        frame = FrameData(image=image, timestamp_ns=int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec),
                          frame_id=str(header.frame_id), sequence_id=self.captured_frames,
                          source_name="compatibility_input")
        self._enqueue_packet(InputPacket(header, frame, self.captured_frames + 1))

    def _process_packet(self, packet: InputPacket) -> None:
        packet_wall_start_ns = time.perf_counter_ns()
        process_entry_ns = time.time_ns()
        header, frame = packet.header, packet.frame
        image = frame.image
        want_debug = (_bool_parameter(self.get_parameter("publish_debug_image").value)
                      and self.debug_publisher.get_subscription_count() > 0)
        pre_state = self.tracker.state.name
        provenance_recorder = self._geometry_provenance_recorder
        provenance_capacity = provenance_recorder is not None and provenance_recorder.has_capacity()
        if provenance_recorder is not None and not provenance_capacity:
            self._provenance_capacity_guard_skips += 1
        periodic_provenance = provenance_recorder.periodic_due() if provenance_capacity else False
        pre_temporal_state = (
            self.tracker.snapshot_temporal_state()
            if provenance_capacity and provenance_recorder is not None else None
        )
        trace_reasons: list[str] = ["periodic_5s"] if periodic_provenance else []
        traced_result = None
        traced_provenance = None
        traced_diagnostics = None
        tracker_process_start_ns = time.perf_counter_ns()
        tracker_process_ms = 0.0
        provenance_second_pass_ms = 0.0
        with self._tuning_lock:
            result = self.tracker.process(image, diagnostics=False, provenance=False)
            tracker_process_ms = (time.perf_counter_ns() - tracker_process_start_ns) / 1e6
            if provenance_recorder is not None and provenance_recorder.has_capacity():
                trace_reasons.extend(provenance_recorder.extreme_reasons(result.fit_inlier_ratio, result.fit_residual_px))
                if trace_reasons:
                    post_temporal_state = self.tracker.snapshot_temporal_state()
                    self.tracker.restore_temporal_state(pre_temporal_state)
                    second_pass_start_ns = time.perf_counter_ns()
                    traced_result = self.tracker.process(image, diagnostics=True, provenance=True)
                    self._provenance_second_pass_count += 1
                    provenance_second_pass_ms = (time.perf_counter_ns() - second_pass_start_ns) / 1e6
                    traced_provenance = self.tracker.last_provenance
                    traced_diagnostics = self.tracker.last_diagnostics
                    self.tracker.restore_temporal_state(post_temporal_state)
        self.processed_frames += 1
        self._processing_samples.append(float(result.processing_ms))
        output_host_timestamp_ns = time.time_ns()
        self._output_host_timestamps_ns.append(output_host_timestamp_ns)
        self._processing_started_ns = output_host_timestamp_ns if self._processing_started_ns is None else self._processing_started_ns
        self._processing_ended_ns = output_host_timestamp_ns
        dropped = self.slot.overwritten_count
        geometry = seam_geometry_message(result, header, self.captured_frames, self.processed_frames, dropped)
        publish_geometry_start_ns = time.perf_counter_ns()
        self.geometry_publisher.publish(geometry)
        publish_geometry_ms = (time.perf_counter_ns() - publish_geometry_start_ns) / 1e6
        publish_state_start_ns = time.perf_counter_ns()
        self.seam_publisher.publish(compatibility_seam_message(result, header))
        publish_state_ms = (time.perf_counter_ns() - publish_state_start_ns) / 1e6
        row = self._live_metric_row(packet, pre_state, result, geometry, dropped)
        row.update({
            "image_callback_entry_ns": packet.image_callback_entry_ns,
            "image_callback_exit_ns": packet.image_callback_exit_ns,
            "image_conversion_ms": packet.image_conversion_ms,
            "callback_total_ms": packet.callback_total_ms,
            "packet_queue_wait_ms": (
                (process_entry_ns - packet.image_callback_exit_ns) / 1e6
                if packet.image_callback_exit_ns is not None else None
            ),
            "tracker_process_ms": tracker_process_ms,
            "provenance_second_pass_ms": provenance_second_pass_ms,
            "provenance_capacity_available": provenance_capacity,
            "provenance_capacity_guard_skipped": bool(provenance_recorder is not None and not provenance_capacity),
            "provenance_task_build_ms": 0.0,
            "snapshot_task_build_ms": 0.0,
            "publish_geometry_ms": publish_geometry_ms,
            "publish_state_ms": publish_state_ms,
            "callback_to_output_ms": (
                (output_host_timestamp_ns - packet.image_callback_entry_ns) / 1e6
                if packet.image_callback_entry_ns is not None else None
            ),
            "output_host_timestamp_ns": output_host_timestamp_ns,
        })
        if provenance_recorder is not None and trace_reasons and traced_provenance is not None and traced_diagnostics is not None:
            task_build_start_ns = time.perf_counter_ns()
            x, y, width_px, height_px = result.roi_original_px
            roi_bgr = image[y:y + height_px, x:x + width_px]
            if roi_bgr.ndim == 2:
                roi_bgr = cv2.cvtColor(roi_bgr, cv2.COLOR_GRAY2BGR)
            else:
                roi_bgr = np.ascontiguousarray(roi_bgr[:, :, :3])
            telemetry = self._telemetry(packet.frame)
            metadata = {
                "header_stamp_ns": int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec),
                "frame_id": str(header.frame_id), "captured_index": int(packet.captured_index),
                "processed_index": int(self.processed_frames), "host_timestamp_ns": int(packet.host_timestamp_ns),
                "device_timestamp_ns": int(packet.frame.timestamp_ns), "state": result.tracking_state.name,
                "detected": bool(result.detected), "predicted_only": bool(result.predicted_only),
                "failure_reasons": list(result.failure_reasons), "roi_original_px": list(result.roi_original_px),
                "config_sha256": provenance_recorder.config_sha256, "reference_sha256": provenance_recorder.reference_sha256,
                "joint_inlier_ratio": float(result.fit_inlier_ratio), "fit_residual_px": result.fit_residual_px,
                "stage_timings_ms": dict(result.stage_timings_ms), "processing_ms": float(result.processing_ms),
                "telemetry": telemetry, "trace_reasons": trace_reasons,
            }
            task = ProvenanceTask(roi_bgr=roi_bgr.copy(), roi=tuple(result.roi_original_px), metadata=metadata,
                                  geometry=self._ros_message_to_dict(geometry), provenance=traced_provenance,
                                  diagnostics=traced_diagnostics, result=traced_result,
                                  labels=list(trace_reasons))
            self._provenance_task_build_count += 1
            row["provenance_task_build_ms"] = (time.perf_counter_ns() - task_build_start_ns) / 1e6
            extreme_labels = [item for item in trace_reasons if item != "periodic_5s"]
            if extreme_labels:
                provenance_recorder.commit_extreme(task, extreme_labels)
            if periodic_provenance:
                provenance_recorder.submit_periodic(task)
        if self._roi_validation_recorder is not None:
            roi_metadata = {
                "header_stamp": {"sec": int(header.stamp.sec), "nanosec": int(header.stamp.nanosec)},
                "frame_id": str(header.frame_id),
                "captured_index": int(packet.captured_index),
                "processed_index": int(self.processed_frames),
                "host_timestamp_ns": int(packet.host_timestamp_ns),
                "device_timestamp_ns": int(packet.frame.timestamp_ns),
                "config_sha256": self._live_recorder.config_sha256,
                "reference_sha256": self._live_recorder.reference_sha256,
                "state": result.tracking_state.name,
                "detected": bool(result.detected),
                "predicted_only": bool(result.predicted_only),
                "failure_reasons": list(result.failure_reasons),
                "roi_original_px": list(result.roi_original_px),
                "geometry": self._ros_message_to_dict(geometry),
                "stage_timings_ms": dict(result.stage_timings_ms),
                "crossing": {
                    "selected_pair_crossing_count": int(result.selected_pair_crossing_count),
                    "fitted_output_crossing_count": int(result.fitted_output_crossing_count),
                    "minimum_fitted_width_px": result.minimum_fitted_width_px,
                },
                "processing_ms": float(result.processing_ms),
                "end_to_end_ms": float(row["end_to_end_ms"]),
            }
            self._roi_validation_recorder.submit(image, tuple(result.roi_original_px), roi_metadata)
        # Explicit/manual requests are consumed first, giving them priority
        # over an automatic event generated by the same processed frame.
        self._capture_snapshot_if_requested(header, image, result, geometry, row)
        snapshot_saved, snapshot_path = self._maybe_failure_snapshot(packet, result, geometry, row)
        row["snapshot_saved"] = snapshot_saved
        row["snapshot_path"] = snapshot_path
        if want_debug:
            overlay = self._runtime_overlay(image, result, row, (1280, 960))
            debug = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            debug.header = header
            self.debug_publisher.publish(debug)
        row["process_packet_wall_ms"] = (time.perf_counter_ns() - packet_wall_start_ns) / 1e6
        self._record_performance_timings(row)
        if _bool_parameter(self.get_parameter("per_frame_metrics_log").value):
            self._live_recorder.submit_metrics(row)

    def _record_performance_timings(self, row: dict[str, object]) -> None:
        for field in PERFORMANCE_TIMING_FIELDS:
            value = row.get(field)
            if value is None:
                continue
            try:
                if math.isfinite(float(value)):
                    self._performance_samples[field].append(float(value))
            except (TypeError, ValueError):
                continue

    def _live_metric_row(self, packet: InputPacket, pre_state: str,
                         result: StableSeamResult, geometry: SeamGeometry,
                         dropped: int) -> dict[str, object]:
        gate = gate_evidence(self.tracker.config, result)
        reasons = list(result.failure_reasons)
        for reason in reasons:
            self.failure_reason_counts[reason] = self.failure_reason_counts.get(reason, 0) + 1
        state = result.tracking_state.name
        self.state_counts[state] = self.state_counts.get(state, 0) + 1
        self._state_history.append(state)
        self._state_history = self._state_history[-60:]
        if result.detected:
            self.detected_frames += 1
        telemetry = self._telemetry(packet.frame)
        return {
            "frame_index": int(packet.captured_index), "captured_index": int(packet.captured_index),
            "host_timestamp_ns": int(packet.host_timestamp_ns), "device_timestamp_ns": int(packet.frame.timestamp_ns),
            "output_host_timestamp_ns": int(time.time_ns()),
            "pre_state": pre_state, "post_state": state, "detected": bool(result.detected),
            "predicted_only": bool(result.predicted_only), "tracking_state": state,
            "confidence": float(result.confidence), "coverage": float(result.coverage),
            "effective_span": float(result.effective_span), "point_count": int(result.accepted_scan_bands),
            "maximum_missing_run": int(result.maximum_missing_run),
            "dropped_processed_frames": int(dropped),
            "abrupt_left_jump_count": int(result.abrupt_left_jump_count),
            "abrupt_right_jump_count": int(result.abrupt_right_jump_count),
            "selected_pair_crossing_count": int(result.selected_pair_crossing_count),
            "fitted_output_crossing_count": int(result.fitted_output_crossing_count),
            "minimum_fitted_width_px": result.minimum_fitted_width_px,
            "crossing_count": int(result.fitted_output_crossing_count),
            "path_confidence": None, "center_inlier_ratio": None, "width_inlier_ratio": None,
            "joint_inlier_ratio": float(result.fit_inlier_ratio), "fit_residual_px": result.fit_residual_px,
            "median_width_px": result.median_width_px, "lateral_error_px": result.lateral_error_px,
            "heading_error_deg": result.heading_error_deg, "processing_ms": float(result.processing_ms),
            "failure_reasons": reasons, "human_readable_reason": human_readable_reason(self.tracker.config, result),
            "gate_truth": gate, "metrics_unavailable_fields": ["path_confidence", "center_inlier_ratio",
                "width_inlier_ratio"], "roi_original_px": list(result.roi_original_px),
            "exposure_mode": telemetry["exposure_mode"], "exposure_us": telemetry["exposure_us"],
            "analog_gain": telemetry["analog_gain"], "analog_gain_raw": telemetry["analog_gain_raw"],
            "gain_db": telemetry["gain_db"], "telemetry_unavailable_fields": telemetry["telemetry_unavailable_fields"],
            "stage_timings_ms": dict(result.stage_timings_ms),
            "end_to_end_ms": (int(time.time_ns()) - int(packet.host_timestamp_ns)) / 1e6,
            "snapshot_saved": False, "snapshot_path": None,
            "geometry_message_detected": bool(geometry.detected),
            "camera_info_received": bool(self._camera_info_received),
            "camera_info_valid": bool(self._camera_info_valid),
        }

    def _telemetry(self, frame: FrameData) -> dict[str, object]:
        gain_db = frame.gain_db
        analog_gain = None if gain_db is None else float(10.0 ** (float(gain_db) / 20.0))
        with self._camera_telemetry_lock:
            diagnostic = dict(self._last_camera_telemetry)
        values = diagnostic.get("values", {}) if isinstance(diagnostic.get("values"), dict) else {}
        if frame.exposure_us is None:
            try:
                frame_exposure = float(values.get("exposure_us"))
            except (TypeError, ValueError):
                frame_exposure = None
        else:
            frame_exposure = frame.exposure_us
        if frame.analog_gain_raw is None:
            try:
                frame_gain_raw = int(float(values.get("analog_gain_raw")))
            except (TypeError, ValueError):
                frame_gain_raw = None
        else:
            frame_gain_raw = frame.analog_gain_raw
        if analog_gain is None:
            try:
                analog_gain = float(values.get("analog_gain"))
            except (TypeError, ValueError):
                analog_gain = None
        telemetry_received_ns = diagnostic.get("telemetry_received_host_ns")
        unavailable: list[str] = []
        if frame.exposure_mode is None: unavailable.append("exposure_mode_sdk_unavailable")
        if frame_exposure is None: unavailable.append("exposure_us")
        if analog_gain is None: unavailable.append("analog_gain")
        if frame_gain_raw is None: unavailable.append("analog_gain_raw")
        if gain_db is None: unavailable.append("gain_db")
        age_ms = None if telemetry_received_ns is None else (time.time_ns() - int(telemetry_received_ns)) / 1e6
        return {"exposure_mode": frame.exposure_mode or "unavailable:sdk_does_not_expose_mode",
                "exposure_us": frame_exposure, "analog_gain": analog_gain, "analog_gain_raw": frame_gain_raw,
                "gain_db": gain_db, "telemetry_stamp_ns": diagnostic.get("telemetry_stamp_ns"),
                "telemetry_age_ms": age_ms, "telemetry_unavailable_fields": sorted(set(unavailable))}

    @staticmethod
    def _ros_message_to_dict(message: SeamGeometry) -> dict[str, object]:
        return {
            "header": {
                "stamp": {"sec": int(message.header.stamp.sec), "nanosec": int(message.header.stamp.nanosec)},
                "frame_id": str(message.header.frame_id),
            },
            "detected": bool(message.detected),
            "predicted_only": bool(message.predicted_only),
            "tracking_state": int(message.tracking_state),
            "confidence": float(message.confidence),
            "roi": {"x_offset": int(message.roi.x_offset), "y_offset": int(message.roi.y_offset),
                    "height": int(message.roi.height), "width": int(message.roi.width)},
            "left_boundary": [{"x": float(point.x), "y": float(point.y), "z": float(point.z)} for point in message.left_boundary],
            "right_boundary": [{"x": float(point.x), "y": float(point.y), "z": float(point.z)} for point in message.right_boundary],
            "centerline": [{"x": float(point.x), "y": float(point.y), "z": float(point.z)} for point in message.centerline],
            "median_width_px": _json_float(message.median_width_px),
            "lateral_error_px": _json_float(message.lateral_error_px),
            "heading_error_deg": _json_float(message.heading_error_deg),
            "coverage": float(message.coverage),
            "effective_span": float(message.effective_span),
            "fit_residual_px": _json_float(message.fit_residual_px),
            "processing_ms": float(message.processing_ms),
            "input_frames": int(message.input_frames),
            "processed_frames": int(message.processed_frames),
            "dropped_frames": int(message.dropped_frames),
            "failure_reasons": [str(value) for value in message.failure_reasons],
        }

    def _maybe_failure_snapshot(self, packet: InputPacket, result: StableSeamResult,
                                geometry: SeamGeometry, row: dict[str, object]) -> tuple[bool, str | None]:
        """Select sparse failure events without copying an ordinary frame."""
        row["snapshot_task_build_ms"] = 0.0
        enabled = _bool_parameter(self.get_parameter("auto_save_failures").value)
        policy = str(self.get_parameter("auto_save_policy").value).strip().lower()
        if not enabled or policy == "disabled":
            return False, None
        now = time.monotonic()
        state = result.tracking_state.name
        reasons = tuple(result.failure_reasons)
        failure = not result.detected
        if failure and self._failure_started_at is None:
            self._failure_started_at = now
        if not failure:
            self._failure_started_at = None
        triggers: list[str] = []
        if failure and self._previous_state != "LOST" and state == "LOST":
            triggers.append("state_transition_to_lost")
        if failure and not self._previous_failure:
            triggers.append("first_failure")
        if failure and reasons != self._previous_failure_reasons and self._previous_failure_reasons:
            if _bool_parameter(self.get_parameter("save_on_failure_reason_change").value):
                triggers.append("failure_reason_change")
        if _bool_parameter(self.get_parameter("save_on_state_transition").value) and state != self._previous_state:
            if state in {"DEGRADED", "LOST"} or (self._previous_state == "LOST" and state == "ACQUIRING") or (self._previous_state == "ACQUIRING" and state == "TRACKING"):
                triggers.append(f"state_transition_{self._previous_state}_to_{state}")
        joint = float(result.fit_inlier_ratio)
        if _bool_parameter(self.get_parameter("save_on_new_worst_metric").value) and np_isfinite(joint):
            if joint < self._worst_joint_inlier - 0.02:
                triggers.append("new_worst_joint_inlier")
                self._worst_joint_inlier = joint
        residual = result.fit_residual_px
        if _bool_parameter(self.get_parameter("save_on_new_worst_metric").value) and np_isfinite(residual):
            if float(residual) > self._worst_residual + 1.0:
                triggers.append("new_worst_fit_residual")
                self._worst_residual = float(residual)
        if failure and self._failure_started_at is not None and now - self._failure_started_at >= float(self.get_parameter("failure_continuous_sec").value):
            triggers.append("continuous_failure")
        if policy == "every_failure" and failure:
            triggers.append("every_failure")
        triggers = sorted(set(triggers))
        ordinary = all(trigger in {"continuous_failure", "every_failure"} for trigger in triggers)
        cooldown = float(self.get_parameter("failure_snapshot_cooldown_sec").value)
        if ordinary and now - self._last_failure_snapshot_at < cooldown:
            triggers = []
        if not triggers:
            self._previous_state, self._previous_failure_reasons, self._previous_failure = state, reasons, failure
            return False, None
        trigger = "+".join(triggers)
        self._auto_snapshot_requested += 1
        config_path = str(self.get_parameter("config_file").value)
        reference_path = str(self.get_parameter("reference_file").value)

        def make_task(event_id: str) -> FailureEventTask:
            task_build_start_ns = time.perf_counter_ns()
            task = FailureEventTask(
                image=packet.frame.image.copy(), header=copy.deepcopy(packet.header), result=copy.deepcopy(result),
                geometry=self._ros_message_to_dict(geometry), gate=copy.deepcopy(row["gate_truth"]),
                metrics=copy.deepcopy(row), trigger_reason=trigger, automatic=True, source_mode=self.mode,
                config_path=config_path, reference_path=reference_path,
                config_sha256=self._live_recorder.config_sha256, reference_sha256=self._live_recorder.reference_sha256,
                save_full_frame=_bool_parameter(self.get_parameter("auto_save_full_frame").value),
                save_context_jpeg=_bool_parameter(self.get_parameter("auto_save_context_jpeg").value),
                context_jpeg_quality=int(self.get_parameter("context_jpeg_quality").value),
                save_lossless_roi=_bool_parameter(self.get_parameter("save_lossless_roi").value), event_id=event_id)
            row["snapshot_task_build_ms"] = (time.perf_counter_ns() - task_build_start_ns) / 1e6
            return task

        accepted, event_id, reason = self._live_recorder.enqueue_snapshot(make_task, automatic=True)
        if accepted:
            self._last_failure_snapshot_at = now
            self._state_history = self._state_history[-60:]
            self._previous_state, self._previous_failure_reasons, self._previous_failure = state, reasons, failure
            return True, event_id
        self._auto_snapshot_rejected += 1
        if reason:
            self.get_logger().warning(f"automatic failure snapshot rejected: {reason}")
        self._previous_state, self._previous_failure_reasons, self._previous_failure = state, reasons, failure
        return False, None

    def _runtime_overlay(self, image: object, result: StableSeamResult,
                         row: dict[str, object], max_size: tuple[int, int] | None = None) -> object:
        """Build debug-only presentation; it is never used for geometry output."""
        overlay = draw_stable_seam_overlay(image, result, None)
        if not result.detected:
            for points in (result.left_boundary_original_px, result.right_boundary_original_px, result.centerline_original_px):
                if len(points) >= 2:
                    cv2.polylines(overlay, [np_round_points(points)], False, (0, 0, 255), 3, cv2.LINE_AA)
        gate = row["gate_truth"]["checks"]
        lines = [
            ("VALID TRACK" if result.detected and result.tracking_state.name == "TRACKING" else "REJECTED / DIAGNOSTIC ONLY"),
            f"frame={row['frame_index']} state={row['pre_state']}->{row['post_state']} detected={result.detected} predicted={result.predicted_only}",
            f"conf={result.confidence:.3f} process={result.processing_ms:.1f}ms",
            f"coverage {float(result.coverage):.3f} / {gate['coverage']['threshold']:.3f} | span {float(result.effective_span):.3f} / {gate['effective_span']['threshold']:.3f}",
            f"points {result.accepted_scan_bands} / {gate['point_count']['threshold']} | missing {result.maximum_missing_run} / {gate['missing_run']['threshold']}",
            f"inlier {result.fit_inlier_ratio:.3f} / {gate['joint_inlier']['threshold']:.3f} | residual {_display_value(result.fit_residual_px)} / {gate['fit_residual']['threshold']:.1f}",
            f"center_inlier=N/A width_inlier=N/A crossing selected/fitted={row['selected_pair_crossing_count']}/{row['fitted_output_crossing_count']} min_width={_display_value(row.get('minimum_fitted_width_px'))} | path_conf={_display_value(row.get('path_confidence'))}",
            f"jumps L/R={result.abrupt_left_jump_count}/{result.abrupt_right_jump_count}",
            f"exposure={_display_value(row.get('exposure_us'))}us gain={_display_value(row.get('analog_gain'))} raw={_display_value(row.get('analog_gain_raw'))}",
            row["human_readable_reason"],
        ]
        colors = (0, 190, 0) if result.detected and result.tracking_state.name == "TRACKING" else (0, 210, 255) if result.tracking_state.name == "ACQUIRING" else (0, 0, 255)
        for index, line in enumerate(lines):
            y = 45 + index * 30
            cv2.putText(overlay, str(line)[:150], (25, y), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(overlay, str(line)[:150], (25, y), cv2.FONT_HERSHEY_SIMPLEX, .65, colors if index == 0 else (255, 255, 255), 1, cv2.LINE_AA)
        base_y = max(overlay.shape[0] - 42, 20)
        for index, state in enumerate(self._state_history[-60:]):
            palette = {"TRACKING": (0, 190, 0), "ACQUIRING": (0, 210, 255), "DEGRADED": (0, 140, 255), "LOST": (0, 0, 220)}
            cv2.line(overlay, (25 + index * 8, base_y), (31 + index * 8, base_y), palette.get(state, (180, 180, 180)), 5)
        if max_size is not None:
            scale = min(1.0, max_size[0] / overlay.shape[1], max_size[1] / overlay.shape[0])
            if scale < 1:
                overlay = cv2.resize(overlay, (round(overlay.shape[1] * scale), round(overlay.shape[0] * scale)), interpolation=cv2.INTER_AREA)
        return overlay

    def _capture_snapshot_if_requested(self, header: Header, image: object,
                                       result: StableSeamResult, geometry: SeamGeometry,
                                       row: dict[str, object] | None = None) -> None:
        try:
            request_wall_time = self._snapshot_requests.get_nowait()
        except Empty:
            return
        try:
            # This is the only full-resolution copy made for a snapshot.  It
            # occurs after processing and only when a one-shot request exists.
            task_build_start_ns = time.perf_counter_ns()
            frame_copy = image.copy()
            task = SnapshotTask(
                image=frame_copy,
                header=copy.deepcopy(header),
                result=copy.deepcopy(result),
                geometry=self._ros_message_to_dict(geometry),
                request_wall_time=float(request_wall_time),
                capture_wall_time=time.time(),
                source_mode=self.mode,
                input_shape=tuple(int(value) for value in frame_copy.shape),
            )
            self._snapshot_tasks.put_nowait(task)
            self._manual_snapshot_enqueued += 1
            if row is not None:
                row["snapshot_task_build_ms"] = (time.perf_counter_ns() - task_build_start_ns) / 1e6
            if row is not None:
                event_gate = row.get("gate_truth", gate_evidence(self.tracker.config, result))
                config_path = str(self.get_parameter("config_file").value)
                reference_path = str(self.get_parameter("reference_file").value)

                def make_manual_event(event_id: str) -> FailureEventTask:
                    return FailureEventTask(
                        # Reuse the already-made manual copy; this keeps the
                        # explicit request priority without a second copy.
                        image=frame_copy, header=copy.deepcopy(header), result=copy.deepcopy(result),
                        geometry=self._ros_message_to_dict(geometry), gate=copy.deepcopy(event_gate),
                        metrics=copy.deepcopy(row), trigger_reason="manual_request", automatic=False,
                        source_mode=self.mode, config_path=config_path, reference_path=reference_path,
                        config_sha256=self._live_recorder.config_sha256,
                        reference_sha256=self._live_recorder.reference_sha256,
                        save_full_frame=True, save_context_jpeg=True,
                        context_jpeg_quality=int(self.get_parameter("context_jpeg_quality").value),
                        save_lossless_roi=True, event_id=event_id)

                accepted, _, reject_reason = self._live_recorder.enqueue_snapshot(make_manual_event, automatic=False)
                if not accepted and reject_reason:
                    self.get_logger().warning(f"manual live event rejected: {reject_reason}")
            self.get_logger().info("snapshot frame handed to writer")
        except Full:
            self._snapshot_capacity.release()
            self.get_logger().error("snapshot writer queue full; request was not saved")
        except Exception as exc:  # noqa: BLE001 - evidence capture must not stop tracking
            self._snapshot_capacity.release()
            self.get_logger().error(f"snapshot task creation failed: {exc}")

    def _snapshot_writer_loop(self) -> None:
        config_path = str(self.get_parameter("config_file").value)
        reference_path = str(self.get_parameter("reference_file").value)
        output_dir = str(self.get_parameter("snapshot_output_dir").value)
        while True:
            task = self._snapshot_tasks.get()
            try:
                if task is None:
                    return
                directory = write_snapshot(task, output_dir, config_path, reference_path)
                self._manual_snapshot_saved += 1
                self.get_logger().info(f"snapshot written: {directory}")
            except Exception as exc:  # noqa: BLE001 - writer exceptions are isolated
                self.get_logger().error(f"snapshot writer failed: {exc}")
            finally:
                if task is not None:
                    self._snapshot_capacity.release()
                self._snapshot_tasks.task_done()

    def _topic_worker(self) -> None:
        while not self.stop_event.is_set():
            item = self.slot.get(0.1)
            if item is not None:
                try:
                    self._process_packet(item)
                except Exception as exc:  # noqa: BLE001 - stop cleanly during context shutdown
                    if self.stop_event.is_set() or not rclpy.ok():
                        break
                    self.get_logger().error(f"stable seam topic worker stopped: {exc}")
                    break

    def _direct_capture_worker(self) -> None:
        camera = self.tracker.config["camera"]
        source = None
        try:
            source = MindVisionCameraSource({
                "sdk_root": camera["sdk_root"], "resolution_index": camera["resolution_index"],
                "width": camera["expected_width"], "height": camera["expected_height"],
                "exposure_mode": camera["exposure_mode"], "exposure_us": camera["exposure_us"], "timeout_ms": camera["timeout_ms"],
                "ae_target": camera["ae_target"], "ae_exposure_min_us": camera["ae_exposure_min_us"],
                "ae_exposure_max_us": camera["ae_exposure_max_us"], "gain_min_raw": camera["gain_min_raw"], "gain_max_raw": camera["gain_max_raw"],
            })
            for _ in range(int(camera["warmup_frames"])):
                if self.stop_event.is_set():
                    return
                try:
                    next(source)
                except SourceError as exc:
                    if "timeout" in str(exc).lower():
                        self.timeout_errors += 1
                    else:
                        self.capture_errors += 1
                        self.get_logger().error(f"MindVision warmup capture error: {exc}")
            while not self.stop_event.is_set():
                try:
                    frame = next(source)
                    if self.stop_event.is_set() or not rclpy.ok():
                        break
                    header = Header()
                    header.stamp.sec = frame.timestamp_ns // 1_000_000_000
                    header.stamp.nanosec = frame.timestamp_ns % 1_000_000_000
                    header.frame_id = frame.frame_id
                    self._enqueue_packet(InputPacket(header, frame, self.captured_frames + 1))
                except SourceError as exc:
                    if "timeout" in str(exc).lower():
                        self.timeout_errors += 1
                    else:
                        self.capture_errors += 1
                        self.get_logger().error(str(exc))
                except Exception as exc:  # noqa: BLE001 - avoid publishing after SIGINT context shutdown
                    if self.stop_event.is_set() or not rclpy.ok():
                        break
                    self.capture_errors += 1
                    self.get_logger().error(str(exc))
        except Exception as exc:
            self.capture_errors += 1
            self.get_logger().error(f"MindVision direct worker stopped: {exc}")
        finally:
            if source is not None:
                source.close()
            if rclpy.ok():
                self.get_logger().info(
                    f"MindVision direct capture stopped: timeouts={self.timeout_errors}, "
                        f"capture_errors={self.capture_errors}, processed={self.processed_frames}")

    def destroy_node(self) -> bool:
        self._shutdown_started_ns = time.time_ns()
        with self._snapshot_lock:
            self._accept_snapshot_requests = False
        self.stop_event.set()
        self.slot.close()
        if self.capture_worker is not None:
            self.capture_worker.join(timeout=3.0)
        if hasattr(self, "worker"):
            self.worker.join(timeout=3.0)
        roi_validation_summary = None
        if self._roi_validation_recorder is not None:
            roi_validation_summary = self._roi_validation_recorder.close()
        geometry_provenance_summary = None
        if self._geometry_provenance_recorder is not None:
            geometry_provenance_summary = self._geometry_provenance_recorder.close()
        try:
            self._snapshot_tasks.put(None, timeout=3.0)
        except Full:
            self.get_logger().error("snapshot writer queue did not accept shutdown marker")
        self._snapshot_writer.join(timeout=15.0)
        try:
            input_stamps = list(self._input_host_timestamps_ns)
            output_stamps = list(self._output_host_timestamps_ns)
            input_first = input_stamps[0] if input_stamps else None
            input_last = input_stamps[-1] if input_stamps else None
            output_first = output_stamps[0] if output_stamps else None
            output_last = output_stamps[-1] if output_stamps else None
            common_start = max(value for value in (input_first, output_first) if value is not None) if input_first is not None and output_first is not None else None
            common_end = min(value for value in (input_last, output_last) if value is not None) if input_last is not None and output_last is not None else None
            common_input = [value for value in input_stamps if common_start is not None and common_end is not None and common_start <= value <= common_end]
            common_output = [value for value in output_stamps if common_start is not None and common_end is not None and common_start <= value <= common_end]
            common_duration = ((common_end - common_start) / 1e9
                               if common_start is not None and common_end is not None and common_end > common_start else 0.0)
            input_fps = ((len(common_input) - 1) / common_duration
                         if len(common_input) > 1 and common_duration > 0 else 0.0)
            output_fps = ((len(common_output) - 1) / common_duration
                          if len(common_output) > 1 and common_duration > 0 else 0.0)
            counters = {
                "captured_frames": self.captured_frames,
                "processed_frames": self.processed_frames,
                "detected_frames": self.detected_frames,
                "lost_frames": self.state_counts.get("LOST", 0),
                "state_counts": self.state_counts,
                "failure_reason_counts": self.failure_reason_counts,
                "preview_skipped_frames": self.slot.overwritten_count,
                "auto_snapshot_requested": self._auto_snapshot_requested,
                "auto_snapshot_rejected": self._auto_snapshot_rejected,
                "manual_snapshot_requested": self._manual_snapshot_requested,
                "manual_snapshot_enqueued": self._manual_snapshot_enqueued,
                "manual_snapshot_saved": self._manual_snapshot_saved,
                "manual_snapshot_rejected": self._manual_snapshot_rejected,
                "provenance_capacity_guard_skips": self._provenance_capacity_guard_skips,
                "provenance_second_pass_count": self._provenance_second_pass_count,
                "provenance_task_build_count": self._provenance_task_build_count,
                "session_disk_usage_bytes": _session_disk_usage(self._live_recorder.session),
                "processing_ms": _distribution(self._processing_samples),
                "performance_timing": {
                    field: _distribution(self._performance_samples[field])
                    for field in PERFORMANCE_TIMING_FIELDS
                },
                "camera_active_duration_sec": ((self._capture_ended_ns - self._capture_started_ns) / 1e9
                                                if self._capture_started_ns is not None and self._capture_ended_ns is not None else 0.0),
                "tracker_active_duration_sec": ((self._processing_ended_ns - self._processing_started_ns) / 1e9
                                                 if self._processing_started_ns is not None and self._processing_ended_ns is not None else 0.0),
                "input_first_host_timestamp_ns": input_first,
                "input_last_host_timestamp_ns": input_last,
                "output_first_host_timestamp_ns": output_first,
                "output_last_host_timestamp_ns": output_last,
                "common_active_window_start_ns": common_start,
                "common_active_window_end_ns": common_end,
                "common_active_duration_sec": common_duration,
                "common_input_frames": len(common_input),
                "common_output_frames": len(common_output),
                "input_fps": input_fps,
                "output_fps": output_fps,
                "warmup_duration_sec": ((output_first - input_first) / 1e9
                                         if input_first is not None and output_first is not None and output_first >= input_first else 0.0),
                "shutdown_duration_sec": 0.0,
            }
            # Preserve the old field names for consumers, but make them use
            # the corrected common-window rates.
            counters["camera_fps"] = input_fps
            counters["tracker_fps"] = output_fps
            counters["camera_info_received"] = self._camera_info_received
            counters["camera_info_valid"] = self._camera_info_valid
            counters["roi_validation"] = roi_validation_summary
            counters["geometry_provenance"] = geometry_provenance_summary
            counters["shutdown_duration_sec"] = ((time.time_ns() - self._shutdown_started_ns) / 1e9
                                                  if self._shutdown_started_ns is not None else 0.0)
            self._live_recorder.finalize(counters, graceful=True)
        except Exception as exc:  # session evidence is best effort during shutdown
            self.get_logger().error(f"live session finalize failed: {exc}")
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StableSeamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # A launch-level SIGINT can arrive while rclpy destroys the last
            # publisher.  Session evidence is finalized before this call;
            # do not turn a normal stop into a failed process.
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
