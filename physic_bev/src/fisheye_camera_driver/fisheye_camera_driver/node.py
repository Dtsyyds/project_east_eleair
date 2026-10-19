"""ROS 2 V4L2 fisheye publisher with bounded buffering and reconnect."""

from __future__ import annotations

from array import array
from collections import deque
import threading
import time

import cv2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .configuration import load_camera_info, resolve_device, validate_capture


class FisheyeCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("fisheye")
        self.declare_parameter("device", "")
        self.declare_parameter("device_id_contains", "icSpring_icspring_camera")
        self.declare_parameter("width", 1920)
        self.declare_parameter("height", 1080)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("pixel_format", "MJPG")
        self.declare_parameter("frame_id", "fisheye_optical_frame")
        self.declare_parameter("camera_info_url", "")
        self.declare_parameter("reconnect_interval_sec", 2.0)
        width, height, fps, pixel_format = validate_capture(
            int(self.get_parameter("width").value), int(self.get_parameter("height").value),
            float(self.get_parameter("fps").value), str(self.get_parameter("pixel_format").value))
        reconnect = float(self.get_parameter("reconnect_interval_sec").value)
        if reconnect <= 0.0 or reconnect > 60.0:
            raise ValueError("reconnect_interval_sec must be within (0, 60]")
        self._requested = (width, height, fps, pixel_format)
        self._reconnect_interval = reconnect
        self._device = resolve_device(str(self.get_parameter("device").value),
                                      str(self.get_parameter("device_id_contains").value))
        self._calibration = load_camera_info(str(self.get_parameter("camera_info_url").value))
        self._capture = None
        self._capture_lock = threading.Lock()
        self._frame_lock = threading.Lock()
        self._stop = threading.Event()
        self._latest_frame = None
        self._capture_sequence = 0
        self._published_sequence = 0
        self._publish_period_ns = int(1e9 / fps)
        self._next_publish_ns = 0
        self._next_open = 0.0
        self._frames = 0
        self._read_errors = 0
        self._reconnects = 0
        self._shape_mismatches = 0
        self._host_times = deque(maxlen=256)
        self._capture_times = deque(maxlen=256)
        self._last_error = "not_opened"
        self._image_pub = self.create_publisher(Image, "~/image_raw", qos_profile_sensor_data)
        self._info_pub = self.create_publisher(CameraInfo, "~/camera_info", qos_profile_sensor_data)
        self._diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name="fisheye-v4l2-capture", daemon=True)
        self._capture_thread.start()
        # Poll faster than the requested output rate.  The UVC source can run
        # near the same frequency as the publisher; equal-frequency timers can
        # repeatedly observe the previous sequence and alias 15 Hz to 7.5 Hz.
        # The deadline below remains the sole output-rate limiter and frames
        # are still latest-only (never duplicated).
        poll_hz = min(120.0, max(30.0, fps * 4.0))
        self._timer = self.create_timer(1.0 / poll_hz, self._publish_latest)
        self._diagnostic_timer = self.create_timer(1.0, self._publish_diagnostics)
        self.get_logger().info(
            f"Fisheye configured device={self._device} {width}x{height} {pixel_format} target={fps:g}Hz")

    def _close_capture(self) -> None:
        with self._capture_lock:
            capture, self._capture = self._capture, None
        if capture is not None:
            capture.release()

    def _open_capture(self) -> bool:
        now = time.monotonic()
        if now < self._next_open:
            return False
        self._next_open = now + self._reconnect_interval
        if not self._device.exists():
            self._last_error = f"device_not_found:{self._device}"
            return False
        width, height, fps, pixel_format = self._requested
        capture = cv2.VideoCapture(str(self._device), cv2.CAP_V4L2)
        # Do not force CAP_PROP_BUFFERSIZE=1 for this UVC camera.  On the
        # icSpring device that setting changes the effective delivery rate
        # from about 16.6 Hz to 8.6 Hz.  The capture thread drains V4L2
        # continuously and the application keeps only _latest_frame, so the
        # default V4L2/OpenCV queue does not create a ROS-side frame backlog.
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)
        if not capture.isOpened():
            capture.release()
            self._last_error = "v4l2_open_failed"
            return False
        actual = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                  int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if actual != (width, height):
            capture.release()
            self._last_error = f"capture_mode_mismatch:requested={width}x{height}:actual={actual[0]}x{actual[1]}"
            self._shape_mismatches += 1
            return False
        with self._capture_lock:
            self._capture = capture
        self._reconnects += 1
        self._last_error = ""
        return True

    def _camera_info(self, stamp, width: int, height: int) -> CameraInfo:
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = str(self.get_parameter("frame_id").value)
        message.width = width
        message.height = height
        if self._calibration is None:
            return message
        calibration = self._calibration
        if (int(calibration["image_width"]), int(calibration["image_height"])) != (width, height):
            raise RuntimeError("camera calibration resolution does not match captured frame")
        message.distortion_model = str(calibration.get("distortion_model", "plumb_bob"))
        message.d = [float(value) for value in calibration["distortion_coefficients"]["data"]]
        message.k = [float(value) for value in calibration["camera_matrix"]["data"]]
        message.r = [float(value) for value in calibration["rectification_matrix"]["data"]]
        message.p = [float(value) for value in calibration["projection_matrix"]["data"]]
        return message

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            with self._capture_lock:
                capture = self._capture
            if capture is None:
                if not self._open_capture():
                    self._stop.wait(min(0.1, self._reconnect_interval))
                continue
            ok, frame = capture.read()
            receipt_ns = self.get_clock().now().nanoseconds
            if not ok or frame is None:
                self._read_errors += 1
                self._last_error = "v4l2_read_failed"
                self._close_capture()
                continue
            if frame.ndim != 3 or frame.shape[2] != 3:
                self._read_errors += 1
                self._last_error = "decoded_frame_is_not_bgr8"
                continue
            height, width = frame.shape[:2]
            expected_width, expected_height, _, _ = self._requested
            if (width, height) != (expected_width, expected_height):
                self._shape_mismatches += 1
                self._last_error = "decoded_frame_shape_mismatch"
                continue
            with self._frame_lock:
                self._capture_sequence += 1
                self._latest_frame = (self._capture_sequence, receipt_ns, frame)
                self._capture_times.append(time.monotonic_ns())
            self._last_error = ""

    def _publish_latest(self) -> None:
        if not rclpy.ok(context=self.context):
            return
        now_ns = time.monotonic_ns()
        if self._next_publish_ns and now_ns < self._next_publish_ns:
            return
        with self._frame_lock:
            latest = self._latest_frame
        if latest is None or latest[0] == self._published_sequence:
            return
        sequence, receipt_ns, frame = latest
        height, width = frame.shape[:2]
        stamp = rclpy.time.Time(nanoseconds=receipt_ns).to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = str(self.get_parameter("frame_id").value)
        image.height = height
        image.width = width
        image.encoding = "bgr8"
        image.is_bigendian = 0
        image.step = width * 3
        # sensor_msgs' Python setter validates generic byte sequences one item
        # at a time.  Supplying its native array type avoids two full Python
        # scans of every 640x480x3 frame and keeps the capture thread schedulable.
        image.data = array("B", frame.tobytes())
        self._image_pub.publish(image)
        self._info_pub.publish(self._camera_info(stamp, width, height))
        self._published_sequence = sequence
        if not self._next_publish_ns:
            self._next_publish_ns = now_ns + self._publish_period_ns
        else:
            self._next_publish_ns += self._publish_period_ns
            if self._next_publish_ns <= now_ns:
                missed = (now_ns - self._next_publish_ns) // self._publish_period_ns + 1
                self._next_publish_ns += missed * self._publish_period_ns
        self._frames += 1
        self._host_times.append(time.monotonic_ns())
        self._last_error = ""

    def _publish_diagnostics(self) -> None:
        if not rclpy.ok(context=self.context):
            return
        fps = 0.0
        if len(self._host_times) > 1 and self._host_times[-1] > self._host_times[0]:
            fps = (len(self._host_times) - 1) / ((self._host_times[-1] - self._host_times[0]) / 1e9)
        capture_fps = 0.0
        with self._frame_lock:
            capture_count = self._capture_sequence
            capture_times = tuple(self._capture_times)
        if len(capture_times) > 1 and capture_times[-1] > capture_times[0]:
            capture_fps = (len(capture_times) - 1) / ((capture_times[-1] - capture_times[0]) / 1e9)
        status = DiagnosticStatus()
        status.name = "fisheye_camera"
        status.hardware_id = str(self._device)
        streaming = self._capture is not None and bool(self._host_times)
        status.level = DiagnosticStatus.OK if streaming else DiagnosticStatus.ERROR
        status.message = "streaming" if streaming else (self._last_error or "no_data")
        values = {"device": str(self._device), "connection_state": "connected" if self._capture else "disconnected",
                  "running_state": "running" if self._capture else "reconnecting",
                  "data_state": "streaming" if streaming else "no_data", "actual_fps": f"{fps:.6f}",
                  "capture_fps": f"{capture_fps:.6f}", "capture_count": str(capture_count),
                  "frame_count": str(self._frames), "read_errors": str(self._read_errors),
                  "shape_mismatches": str(self._shape_mismatches), "open_count": str(self._reconnects),
                  "last_error": self._last_error,
                  "camera_info_valid": str(self._calibration is not None).lower(),
                  "timestamp_source": "ros_clock_at_v4l2_read_completion"}
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics_pub.publish(message)

    def destroy_node(self):
        self._stop.set()
        self._close_capture()
        if self._capture_thread.is_alive():
            self._capture_thread.join(timeout=3.0)
        if self._capture_thread.is_alive():
            self.get_logger().error("fisheye capture thread did not exit within 3 seconds")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FisheyeCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # Launch may invalidate the ROS context immediately before a Python
        # timer enters publish(). Treat only that shutdown race as normal.
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
