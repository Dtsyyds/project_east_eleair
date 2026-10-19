"""ROS 2 Image/CameraInfo publisher for MindVision mono and color cameras."""

from __future__ import annotations

from collections import deque

import ctypes
import array
import fcntl
import importlib.util
import os
from pathlib import Path
import sys
import time
from typing import Any

import cv2
import numpy as np
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from .buffer import copy_uint8_buffer
from .camera_settings import configure_exposure
from .clock_mapper import DeviceClockMapper
from .frame_layout import FrameLayout, frame_layout, raw_bayer_gb8_layout, validate_data_length
from .isp_settings import AppliedSettings, apply_isp_settings
from .qos_settings import validate_qos


def _sdk_library_dir(root: Path) -> Path:
    machine = os.uname().machine.lower()
    architecture = "arm64" if machine in {"aarch64", "arm64"} else "x64" if machine in {"x86_64", "amd64"} else ""
    if not architecture:
        raise RuntimeError(f"unsupported MindVision host architecture: {machine}")
    return root / "lib" / architecture


def _candidate_sdk_roots(configured: str):
    if configured:
        yield Path(configured).expanduser()
    environment = os.environ.get("MINDVISION_SDK_ROOT")
    if environment:
        yield Path(environment).expanduser()
    workspace = os.environ.get("EAST_ELEAIR_ROOT")
    if workspace:
        yield Path(workspace).expanduser() / "MindVisionSDK_v2.1.0.50" / "sdk"
    prefix = Path(get_package_prefix("mindvision_camera_driver"))
    for parent in (prefix, *prefix.parents):
        yield parent / "MindVisionSDK_v2.1.0.50" / "sdk"
    # Legacy fallbacks remain available, but only after the current SDK.
    if workspace:
        yield Path(workspace).expanduser() / "MindVisionlinuxSDK"
    yield Path("/opt/MindVisionlinuxSDK")
    yield Path("/usr/local/share/MindVisionlinuxSDK")


def load_mvsdk(configured_root: str = "") -> tuple[Any, Path, Path]:
    checked: list[str] = []
    for candidate in _candidate_sdk_roots(configured_root):
        root = candidate.resolve()
        checked.append(str(root))
        binding_dir = root / "demo" / "python_demo"
        binding = binding_dir / "mvsdk.py"
        library = _sdk_library_dir(root) / "libMVSDK.so"
        if binding.is_file() and library.is_file():
            library_handle = ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
            module_name = f"_mindvision_mvsdk_{abs(hash(str(root)))}"
            spec = importlib.util.spec_from_file_location(module_name, binding)
            if spec is None or spec.loader is None:
                raise RuntimeError(f"could not load MindVision Python binding: {binding}")
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            # mvsdk.py initially opens by SONAME. Replace its handle with the
            # absolute library selected above so all subsequent calls are pinned
            # to the same SDK root even when an older SONAME is globally visible.
            module._sdk = library_handle
            return module, root, library
    raise RuntimeError("MindVision SDK root not found; checked: " + ", ".join(checked))


def _ros_time(nanoseconds: int) -> Time:
    value = int(nanoseconds)
    return Time(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


class MindVisionCameraNode(Node):
    """Publish the SDK's verified Mono8 or BGR8 ISP output without fake calibration."""

    def __init__(self):
        super().__init__("camera")
        self.declare_parameter("sdk_root", "")
        self.declare_parameter("expected_model", "")
        self.declare_parameter("serial_number", "")
        self.declare_parameter("frame_id", "mindvision_optical_frame")
        self.declare_parameter("image_topic", "~/image_raw")
        self.declare_parameter("camera_info_topic", "~/camera_info")
        self.declare_parameter("preview_topic", "~/preview/image_raw")
        self.declare_parameter("raw_bayer_topic", "")
        self.declare_parameter("preview_width", 640)
        self.declare_parameter("preview_fps", 10.0)
        self.declare_parameter("pixel_format", "auto")
        self.declare_parameter("resolution_index", -1)
        self.declare_parameter("width", 4096)
        self.declare_parameter("height", 3072)
        self.declare_parameter("auto_exposure", True)
        self.declare_parameter("exposure_time_us", 0.0)
        self.declare_parameter("analog_gain", 0.0)
        self.declare_parameter("ae_target", -1)
        self.declare_parameter("rgb_gain_r", -1)
        self.declare_parameter("rgb_gain_g", -1)
        self.declare_parameter("rgb_gain_b", -1)
        self.declare_parameter("saturation", -1)
        self.declare_parameter("gamma", -1)
        self.declare_parameter("contrast", -1)
        self.declare_parameter("white_balance", "unchanged")
        self.declare_parameter("color_temperature_index", -1)
        self.declare_parameter("frame_speed_index", -1)
        self.declare_parameter("target_fps", 0.0)
        # Backwards-compatible aliases used by the SUF1205 profiles.
        self.declare_parameter("exposure_mode", "")
        self.declare_parameter("exposure_us", -1.0)
        self.declare_parameter("ae_min_exposure_us", 0.0)
        self.declare_parameter("ae_max_exposure_us", 0.0)
        self.declare_parameter("ae_min_analog_gain_raw", 0)
        self.declare_parameter("ae_max_analog_gain_raw", 0)
        self.declare_parameter("warmup_frames", 60)
        self.declare_parameter("publish_rate_hz", -1.0)
        self.declare_parameter("timeout_ms", 2000)
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("qos_reliability", "best_effort")

        self._sdk: Any | None = None
        self._handle: Any | None = None
        self._buffer: Any = 0
        self._playing = False
        self._closed = False
        self._is_mono = False
        self._layout: FrameLayout | None = None
        self._clock_mapper = DeviceClockMapper()
        self._timer = None
        self._sequence = 0
        self._warmup_completed = 0
        self._timeout_count = 0
        self._capture_error_count = 0
        self._device_ticks = deque(maxlen=2048)
        self._host_receive_ns = deque(maxlen=2048)
        self._header_stamp_ns = deque(maxlen=2048)
        self._device_time_ns = deque(maxlen=2048)
        self._exposures_us = deque(maxlen=2048)
        self._analog_gain_raw = deque(maxlen=2048)
        self._f_analog_gain = deque(maxlen=2048)
        self._ae_exposure_range: tuple[float, float] | None = None
        self._ae_gain_range: tuple[int, int] | None = None
        self._device = None
        self._diagnostics_pub = None
        self._device_lock = None
        self._applied_settings: AppliedSettings | None = None
        self._sdk_library: Path | None = None
        self._preview_last_ns = 0
        self._preview_count = 0
        self._raw_bayer_pub = None

        try:
            self._configure_qos()
            self._open_camera()
            legacy_rate = float(self._value("publish_rate_hz"))
            publish_rate_hz = legacy_rate if legacy_rate >= 0.0 else float(self._value("target_fps"))
            if publish_rate_hz < 0.0:
                raise RuntimeError("publish_rate_hz must be non-negative")
            timer_period = 0.001 if publish_rate_hz == 0.0 else 1.0 / publish_rate_hz
            self._timer = self.create_timer(timer_period, self._capture_once)
            description = "camera maximum" if publish_rate_hz == 0.0 else f"{publish_rate_hz:g} Hz"
            self.get_logger().info(f"MindVision publish rate: {description}")
        except Exception:
            self.close()
            raise

    def _value(self, name: str) -> Any:
        return self.get_parameter(name).value

    def _configure_qos(self) -> None:
        depth, reliability_name = validate_qos(self._value("qos_depth"), self._value("qos_reliability"))
        if reliability_name == "best_effort":
            reliability = ReliabilityPolicy.BEST_EFFORT
        elif reliability_name == "reliable":
            reliability = ReliabilityPolicy.RELIABLE
        else:
            raise RuntimeError("qos_reliability must be best_effort or reliable")
        self._qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=reliability,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._image_pub = self.create_publisher(Image, str(self._value("image_topic")), self._qos)
        self._info_pub = self.create_publisher(CameraInfo, str(self._value("camera_info_topic")), self._qos)
        self._preview_pub = self.create_publisher(Image, str(self._value("preview_topic")), self._qos)
        raw_bayer_topic = str(self._value("raw_bayer_topic")).strip()
        if raw_bayer_topic:
            self._raw_bayer_pub = self.create_publisher(Image, raw_bayer_topic, self._qos)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)

        preview_width = int(self._value("preview_width"))
        preview_fps = float(self._value("preview_fps"))
        if preview_width < 16 or preview_width > 1920:
            raise RuntimeError("preview_width must be between 16 and 1920")
        if preview_fps <= 0.0 or preview_fps > 30.0:
            raise RuntimeError("preview_fps must be greater than zero and at most 30")

    def _select_device(self, devices: list[Any]) -> Any:
        serial = str(self._value("serial_number"))
        if serial.startswith("sn_"):
            serial = serial[3:]
        if serial:
            for device in devices:
                if device.GetSn() == serial:
                    return device
            known = ", ".join(device.GetSn() for device in devices)
            raise RuntimeError(f"MindVision serial {serial!r} not found; available: {known}")
        if len(devices) != 1:
            raise RuntimeError(f"expected exactly one MindVision camera, found {len(devices)}")
        return devices[0]

    def _select_resolution(self, capability: Any) -> Any:
        count = int(capability.iImageSizeDesc)
        supported: list[tuple[int, int, int, str]] = []
        for index in range(count):
            item = capability.pImageSizeDesc[index]
            description = item.GetDescription() if hasattr(item, "GetDescription") else ""
            supported.append((index, int(item.iWidth), int(item.iHeight), str(description)))
            self.get_logger().info(
                f"MindVision supported resolution index={index} width={item.iWidth} height={item.iHeight} description={description}"
            )
        requested = int(self._value("resolution_index"))
        if requested < -1:
            raise RuntimeError("resolution_index must be -1 or a non-negative index")
        if requested >= 0:
            if requested >= count:
                raise RuntimeError(f"MindVision resolution index {requested} is unavailable")
        else:
            target = (int(self._value("width")), int(self._value("height")))
            matches = [item for item in supported if item[1:3] == target]
            if not matches:
                available = ", ".join(f"{i}:{w}x{h}" for i, w, h, _ in supported)
                raise RuntimeError(f"no exact MindVision resolution {target[0]}x{target[1]}; available: {available}")
            requested = matches[0][0]
        resolution = capability.pImageSizeDesc[requested]
        status = self._sdk.CameraSetImageResolution(self._handle, resolution)
        if status != self._sdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError(f"could not select MindVision resolution index {requested}")
        self.get_logger().info(
            f"Selected resolution index {requested}: {resolution.GetDescription()} ({resolution.iWidth}x{resolution.iHeight})"
        )
        return resolution

    def _configure_exposure(self) -> None:
        legacy_mode = str(self._value("exposure_mode")).strip().lower()
        auto_exposure = bool(self._value("auto_exposure"))
        if legacy_mode:
            if legacy_mode not in {"auto", "manual"}:
                raise RuntimeError("exposure_mode must be auto or manual")
            auto_exposure = legacy_mode == "auto"
        legacy_exposure = float(self._value("exposure_us"))
        exposure_time_us = (
            legacy_exposure if legacy_exposure >= 0.0
            else float(self._value("exposure_time_us"))
        )
        self._ae_exposure_range, self._ae_gain_range = configure_exposure(
            self._sdk,
            self._handle,
            "auto" if auto_exposure else "manual",
            exposure_time_us,
            self._value("ae_min_exposure_us"),
            self._value("ae_max_exposure_us"),
            self._value("ae_min_analog_gain_raw"),
            self._value("ae_max_analog_gain_raw"),
        )
        if auto_exposure:
            self.get_logger().info(
                f"MindVision AE enabled exposure_range_us={self._ae_exposure_range} "
                f"analog_gain_raw_range={self._ae_gain_range}"
            )
        rgb_values = tuple(int(self._value(name)) for name in ("rgb_gain_r", "rgb_gain_g", "rgb_gain_b"))
        if any(value >= 0 for value in rgb_values) and not all(value >= 0 for value in rgb_values):
            raise RuntimeError("rgb_gain_r, rgb_gain_g and rgb_gain_b must be configured together")
        self._applied_settings = apply_isp_settings(
            self._sdk, self._handle, self._capability,
            auto_exposure=auto_exposure,
            exposure_time_us=exposure_time_us,
            analog_gain=float(self._value("analog_gain")),
            ae_target=int(self._value("ae_target")),
            rgb_gain=rgb_values if all(value >= 0 for value in rgb_values) else None,
            saturation=int(self._value("saturation")),
            gamma=int(self._value("gamma")),
            contrast=int(self._value("contrast")),
            white_balance=str(self._value("white_balance")),
            color_temperature_index=int(self._value("color_temperature_index")),
            frame_speed_index=int(self._value("frame_speed_index")),
            log=self.get_logger().info,
        )

    def _acquire_device_lock(self, serial: str) -> None:
        path = Path("/tmp") / f"mindvision_camera_{serial}.lock"
        lock = path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            lock.close()
            raise RuntimeError(f"MindVision camera {serial} is already owned by another driver process") from exc
        lock.seek(0)
        lock.truncate()
        lock.write(f"pid={os.getpid()}\n")
        lock.flush()
        self._device_lock = lock

    def _open_camera(self) -> None:
        configured_serial = str(self._value("serial_number"))
        if configured_serial.startswith("sn_"):
            configured_serial = configured_serial[3:]
        # When a serial is configured, take ownership before even initializing
        # the vendor SDK.  CameraSdkInit/CameraEnumerateDevice can otherwise
        # touch an already streaming USB device before the later lock check.
        if configured_serial:
            self._acquire_device_lock(configured_serial)
        self._sdk, sdk_root, self._sdk_library = load_mvsdk(str(self._value("sdk_root")))
        init_status = self._sdk.CameraSdkInit(1)
        if init_status != self._sdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError(f"MindVision CameraSdkInit failed with error {init_status}")
        devices = list(self._sdk.CameraEnumerateDevice())
        if not devices:
            raise RuntimeError("MindVision SDK found no camera")
        self._device = self._select_device(devices)
        if self._device_lock is None:
            self._acquire_device_lock(str(self._device.GetSn()))
        expected_model = str(self._value("expected_model")).strip()
        friendly_name = str(self._device.GetFriendlyName())
        if expected_model and expected_model.lower() not in friendly_name.lower():
            raise RuntimeError(
                f"MindVision model mismatch: expected {expected_model!r}, enumerated {friendly_name!r}"
            )
        self._handle = self._sdk.CameraInit(self._device, -1, -1)
        capability = self._sdk.CameraGetCapability(self._handle)
        self._capability = capability
        self._select_resolution(capability)
        self._is_mono = bool(capability.sIspCapacity.bMonoSensor)
        requested_format = str(self._value("pixel_format")).lower()
        if requested_format == "auto":
            requested_format = "mono8" if self._is_mono else "bgr8"
        if requested_format not in {"mono8", "bgr8"}:
            raise RuntimeError("pixel_format must be auto, mono8 or bgr8")
        if self._is_mono and requested_format != "mono8":
            raise RuntimeError("monochrome MindVision sensor requires pixel_format=mono8")
        formats = {"mono8": self._sdk.CAMERA_MEDIA_TYPE_MONO8,
                   "bgr8": self._sdk.CAMERA_MEDIA_TYPE_BGR8}
        output_format = formats[requested_format]
        encoding = requested_format
        if self._sdk.CameraSetIspOutFormat(self._handle, output_format) != self._sdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError(f"could not select MindVision {encoding} output")
        if self._sdk.CameraSetTriggerMode(self._handle, 0) != self._sdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError("could not enable MindVision continuous acquisition")
        self._configure_exposure()
        if self._sdk.CameraPlay(self._handle) != self._sdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError("could not start MindVision acquisition")
        self._playing = True
        channels = 1 if encoding == "mono8" else 3
        max_width = int(capability.sResolutionRange.iWidthMax)
        max_height = int(capability.sResolutionRange.iHeightMax)
        self._max_width = max_width
        self._max_height = max_height
        self._buffer = self._sdk.CameraAlignMalloc(max_width * max_height * channels, 16)
        if not self._buffer:
            raise RuntimeError("CameraAlignMalloc returned a null buffer")
        self._warmup()
        self.get_logger().info(
            f"MindVision ready model={self._device.GetFriendlyName()} serial={self._device.GetSn()} "
            f"sensor={'mono' if self._is_mono else 'color'} encoding={encoding} "
            f"max_buffer={max_width * max_height * channels} sdk={sdk_root} "
            f"sdk_library={self._sdk_library} sdk_version={self._sdk.CameraSdkGetVersionString()} "
            "timestamp_source=software_anchored_device_tick hardware_synchronized=false tick_period_ns=100000"
        )

    def _acquire_processed(self) -> tuple[Any, FrameLayout, Any, int, tuple[FrameLayout, Any] | None] | None:
        raw_buffer: Any | None = None
        try:
            try:
                raw_buffer, head = self._sdk.CameraGetImageBuffer(self._handle, int(self._value("timeout_ms")))
            except Exception as exc:  # noqa: BLE001 - vendor boundary
                if getattr(exc, "error_code", None) == self._sdk.CAMERA_STATUS_TIME_OUT:
                    self._timeout_count += 1
                    return None
                self._capture_error_count += 1
                raise
            raw_frame = None
            raw_bayer_pub = getattr(self, "_raw_bayer_pub", None)
            if raw_bayer_pub is not None and raw_bayer_pub.get_subscription_count() > 0:
                raw_layout = raw_bayer_gb8_layout(
                    self._sdk, head, self._max_width, self._max_height)
                raw_data = copy_uint8_buffer(raw_buffer, raw_layout.data_size)
                validate_data_length(raw_data, raw_layout)
                raw_frame = (raw_layout, raw_data)
            if self._sdk.CameraImageProcess(self._handle, raw_buffer, self._buffer, head) != self._sdk.CAMERA_STATUS_SUCCESS:
                self._capture_error_count += 1
                raise RuntimeError("MindVision CameraImageProcess failed")
            max_width = self._max_width
            max_height = self._max_height
            # Capability maxima are enforced by the allocated buffer; the selected
            # profile dimensions are also checked here before copying any bytes.
            layout = frame_layout(self._sdk, head, self._is_mono, max_width, max_height)
            data = copy_uint8_buffer(self._buffer, layout.data_size)
            validate_data_length(data, layout)
            return head, layout, data, time.monotonic_ns(), raw_frame
        except Exception:
            if raw_buffer is not None:
                # The finally block below owns the single release. Keep this
                # branch only to make the exception path visible to callers.
                pass
            raise
        finally:
            if raw_buffer is not None:
                active_exception = sys.exc_info()[0] is not None
                status = self._sdk.CameraReleaseImageBuffer(self._handle, raw_buffer)
                if not active_exception and status not in (None, self._sdk.CAMERA_STATUS_SUCCESS):
                    self._capture_error_count += 1
                    raise RuntimeError(f"MindVision CameraReleaseImageBuffer failed with status {status}")

    def _warmup(self) -> None:
        requested = int(self._value("warmup_frames"))
        if requested < 0:
            raise RuntimeError("warmup_frames must be non-negative")
        attempts = 0
        maximum_attempts = max(requested * 3 + 10, requested)
        while self._warmup_completed < requested and attempts < maximum_attempts:
            attempts += 1
            if self._acquire_processed() is not None:
                self._warmup_completed += 1
        if self._warmup_completed < requested:
            raise RuntimeError(
                f"MindVision warmup acquired {self._warmup_completed}/{requested} frames "
                f"after {attempts} attempts"
            )
        self.get_logger().info(f"MindVision warmup complete: {self._warmup_completed} frames discarded")

    def _camera_info(self, stamp: Time, width: int, height: int) -> CameraInfo:
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = str(self._value("frame_id"))
        message.width = width
        message.height = height
        # No calibration has been performed. Leave all calibration arrays at
        # ROS defaults; in particular K[0] remains zero.
        message.distortion_model = ""
        return message

    def _capture_once(self) -> None:
        if self._closed or not rclpy.ok(context=self.context):
            return
        read_started_ns = time.monotonic_ns()
        try:
            acquired = self._acquire_processed()
            if acquired is None:
                if rclpy.ok(context=self.context):
                    self._publish_error_diagnostics("frame_timeout", DiagnosticStatus.WARN)
                return
            # SIGINT can invalidate the ROS context while the blocking SDK read
            # is in progress.  Releasing the SDK frame is already guaranteed by
            # _acquire_processed(); do not attempt to publish into that context.
            if self._closed or not rclpy.ok(context=self.context):
                return
            head, layout, data, receive_ns, raw_frame = acquired
            raw_tick = int(head.uiTimeStamp) & 0xFFFFFFFF
            stamp_ns = self._clock_mapper.map(raw_tick, self.get_clock().now().nanoseconds)
            previous_stamp = self._header_stamp_ns[-1] if self._header_stamp_ns else None
            if previous_stamp is not None and stamp_ns <= previous_stamp:
                raise RuntimeError(f"non-monotonic ROS image timestamp {stamp_ns} after {previous_stamp}")
            raw_gain = int(self._sdk.CameraGetAnalogGain(self._handle))
            stamp = _ros_time(stamp_ns)
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = str(self._value("frame_id"))
            image.height = layout.height
            image.width = layout.width
            image.encoding = layout.encoding
            image.is_bigendian = 0
            image.step = layout.step
            image.data = data
            if len(image.data) != layout.data_size:
                raise RuntimeError(f"ROS Image data length {len(image.data)} != {layout.data_size}")
            self._image_pub.publish(image)
            if raw_frame is not None and self._raw_bayer_pub is not None:
                raw_layout, raw_data = raw_frame
                raw_image = Image()
                raw_image.header = image.header
                raw_image.height = raw_layout.height
                raw_image.width = raw_layout.width
                raw_image.encoding = raw_layout.encoding
                raw_image.is_bigendian = 0
                raw_image.step = raw_layout.step
                raw_image.data = raw_data
                self._raw_bayer_pub.publish(raw_image)
            self._info_pub.publish(self._camera_info(stamp, layout.width, layout.height))
            self._publish_preview(image, data, receive_ns)
            self._sequence += 1
            self._device_ticks.append(raw_tick)
            self._device_time_ns.append(int(self._clock_mapper.last_device_ns))
            self._header_stamp_ns.append(stamp_ns)
            self._host_receive_ns.append(receive_ns)
            self._exposures_us.append(int(head.uiExpTime))
            self._analog_gain_raw.append(raw_gain)
            self._f_analog_gain.append(float(head.fAnalogGain))
            self._publish_runtime_diagnostics(
                stamp_ns=stamp_ns, encoding=layout.encoding,
                width=layout.width, height=layout.height,
                exposure_us=int(head.uiExpTime), analog_gain=float(head.fAnalogGain),
                analog_gain_raw=raw_gain,
                source_read_ms=(time.monotonic_ns() - read_started_ns) / 1_000_000.0,
            )
            if self._sequence == 1:
                self.get_logger().info(
                    f"Publishing MindVision frames: {layout.width}x{layout.height} {layout.encoding} "
                    f"step={layout.step} bytes={layout.data_size}"
                )
            if self._sequence % 100 == 0:
                self._log_stats()
        except Exception as exc:  # noqa: BLE001 - timer callback boundary
            if self._closed or not rclpy.ok(context=self.context):
                return
            self._capture_error_count += 1
            if self._sdk is not None and getattr(exc, "error_code", None) == self._sdk.CAMERA_STATUS_TIME_OUT:
                self.get_logger().warning("MindVision frame timeout", throttle_duration_sec=5.0)
            else:
                self.get_logger().error(f"MindVision capture failed: {exc}")
            self._publish_error_diagnostics(str(exc), DiagnosticStatus.ERROR)

    def _publish_preview(self, source: Image, data: array.array, receive_ns: int) -> None:
        preview_fps = float(self._value("preview_fps"))
        interval_ns = int(1_000_000_000 / preview_fps)
        if self._preview_last_ns and receive_ns - self._preview_last_ns < interval_ns:
            return
        image = np.frombuffer(data, dtype=np.uint8).reshape((source.height, source.width, 3))
        width = min(int(self._value("preview_width")), int(source.width))
        height = max(1, int(round(source.height * width / source.width)))
        # Preview is visualization-only. INTER_LINEAR avoids making full-rate
        # acquisition wait on the much more expensive large-ratio AREA filter.
        resized = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
        preview = Image()
        preview.header = source.header
        preview.height = height
        preview.width = width
        preview.encoding = "bgr8"
        preview.is_bigendian = 0
        preview.step = width * 3
        # Assign array('B') directly. ROS generated setters expand bytes one
        # element at a time (about 130 ms even for this preview), while the
        # array buffer takes the zero-conversion path used by the full frame.
        preview.data = array.array("B", resized.tobytes())
        self._preview_pub.publish(preview)
        self._preview_last_ns = receive_ns
        self._preview_count += 1

    def _publish_error_diagnostics(self, reason: str, level: int) -> None:
        if self._diagnostics_pub is None:
            return
        status = DiagnosticStatus()
        status.name = "mindvision_camera"
        status.hardware_id = str(self._device.GetSn()) if self._device is not None else "unavailable"
        # Humble's generated uint8 constants can be represented as a one-byte
        # bytes object.  Normalize both that form and ordinary integers.
        status.level = level[0] if isinstance(level, bytes) else int(level)
        status.message = reason
        status.values = [
            KeyValue(key="streaming", value="false"),
            KeyValue(key="timeout_count", value=str(self._timeout_count)),
            KeyValue(key="capture_error_count", value=str(self._capture_error_count)),
            KeyValue(key="last_good_frame_count", value=str(self._sequence)),
            KeyValue(key="sdk_library", value=str(self._sdk_library)),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics_pub.publish(message)

    def _publish_runtime_diagnostics(
        self, *, stamp_ns: int, encoding: str, width: int, height: int,
        exposure_us: int, analog_gain: float, analog_gain_raw: int,
        source_read_ms: float,
    ) -> None:
        """Publish measured metadata; SDK-unavailable values remain explicit."""
        if self._diagnostics_pub is None:
            return
        status = DiagnosticStatus()
        status.name = "mindvision_camera"
        status.hardware_id = str(self._device.GetSn()) if self._device is not None else "unavailable"
        status.level = DiagnosticStatus.OK
        status.message = "streaming"
        values = {
            "header_stamp_ns": str(int(stamp_ns)), "width": str(int(width)),
            "height": str(int(height)), "encoding": str(encoding),
            "camera_model": str(self._device.GetFriendlyName()) if self._device is not None else "unavailable",
            "exposure_us": str(int(exposure_us)), "analog_gain": str(float(analog_gain)),
            "analog_gain_raw": str(int(analog_gain_raw)),
            "gain_db": "unavailable:sdk_does_not_expose_db",
            "source_read_ms": str(float(source_read_ms)),
            "timeout_count": str(int(self._timeout_count)),
            "capture_error_count": str(int(self._capture_error_count)),
            "statistics_basis": "last_2048_frames",
            "frame_count": str(int(self._sequence)), "camera_info_valid": "false",
            "preview_frame_count": str(int(self._preview_count)),
            "preview_topic": str(self._value("preview_topic")),
            "raw_bayer_topic": str(self._value("raw_bayer_topic")),
            "streaming": "true",
            "sdk_library": str(self._sdk_library),
            "sdk_version": str(self._sdk.CameraSdkGetVersionString()),
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = _ros_time(stamp_ns)
        message.status = [status]
        self._diagnostics_pub.publish(message)

    def _log_stats(self) -> None:
        device_fps = 0.0
        if len(self._device_time_ns) > 1 and self._device_time_ns[-1] > self._device_time_ns[0]:
            device_fps = (len(self._device_time_ns) - 1) / ((self._device_time_ns[-1] - self._device_time_ns[0]) / 1_000_000_000)
        host_fps = 0.0
        if len(self._host_receive_ns) > 1 and self._host_receive_ns[-1] > self._host_receive_ns[0]:
            host_fps = (len(self._host_receive_ns) - 1) / ((self._host_receive_ns[-1] - self._host_receive_ns[0]) / 1_000_000_000)
        exposure_first_last = (
            f"{self._exposures_us[0]}..{self._exposures_us[-1]}"
            if self._exposures_us else "n/a"
        )
        gain_first_last = (
            f"{self._analog_gain_raw[0]}..{self._analog_gain_raw[-1]}"
            if self._analog_gain_raw else "n/a"
        )
        self.get_logger().info(
            f"MindVision stats frames={self._sequence} timeout_count={self._timeout_count} "
            f"capture_error_count={self._capture_error_count} raw_tick={self._clock_mapper.last_raw_tick} "
            f"exposure_us_first_last={exposure_first_last} "
            f"analog_gain_raw_first_last={gain_first_last} "
            f"device_timestamp_fps={device_fps:.3f} host_receive_fps={host_fps:.3f} "
            f"timestamp_source=software_anchored_device_tick hardware_synchronized=false tick_period_ns=100000"
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._timer is not None:
            self._timer.cancel()
        try:
            if self._sdk is not None and self._handle is not None and self._playing:
                try:
                    status = self._sdk.CameraPause(self._handle)
                    if status not in (None, self._sdk.CAMERA_STATUS_SUCCESS):
                        self.get_logger().error(f"MindVision CameraPause failed with status {status}")
                except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - cleanup must continue
                    self.get_logger().error(f"MindVision CameraPause failed: {exc}")
                self._playing = False
        finally:
            try:
                if self._sdk is not None and self._buffer:
                    try:
                        self._sdk.CameraAlignFree(self._buffer)
                    except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - cleanup must continue
                        self.get_logger().error(f"MindVision CameraAlignFree failed: {exc}")
                    finally:
                        self._buffer = 0
            finally:
                if self._sdk is not None and self._handle is not None:
                    try:
                        status = self._sdk.CameraUnInit(self._handle)
                        if status not in (None, self._sdk.CAMERA_STATUS_SUCCESS):
                            self.get_logger().error(f"MindVision CameraUnInit failed with status {status}")
                    except (Exception, KeyboardInterrupt) as exc:  # noqa: BLE001 - cleanup must continue
                        self.get_logger().error(f"MindVision CameraUnInit failed: {exc}")
                    finally:
                        self._handle = None
                if getattr(self, "_device_lock", None) is not None:
                    try:
                        fcntl.flock(self._device_lock.fileno(), fcntl.LOCK_UN)
                    finally:
                        self._device_lock.close()
                        self._device_lock = None

    def destroy_node(self):
        self.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MindVisionCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
