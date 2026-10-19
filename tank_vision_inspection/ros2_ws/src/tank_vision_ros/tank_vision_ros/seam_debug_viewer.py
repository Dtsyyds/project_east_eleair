"""Optional GUI kept out of the production detector process."""

from __future__ import annotations

import time
from threading import Event

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class SeamDebugViewer(Node):
    def __init__(self) -> None:
        super().__init__("seam_debug_viewer")
        self.bridge = CvBridge()
        self.stop_requested = Event()
        self.snapshot_client = self.create_client(Trigger, "/stable_seam_node/save_snapshot")
        self.parameter_client = self.create_client(SetParameters, "/stable_seam_node/set_parameters")
        self.auto_save_enabled = True
        self.status_text = "S: save snapshot | A: auto events | Q/ESC: quit"
        self.status_until = 0.0
        self.create_subscription(Image, "/vision/front/debug_image", self._show, qos_profile_sensor_data)
        self.create_timer(0.05, self._poll_key)

    def _set_status(self, text: str, seconds: float = 2.5) -> None:
        self.status_text = text
        self.status_until = time.monotonic() + seconds

    def _show(self, message: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(message, "bgr8")
        scale = min(1.0, 1280 / image.shape[1], 960 / image.shape[0])
        if scale < 1:
            image = cv2.resize(image, (round(image.shape[1] * scale), round(image.shape[0] * scale)))
        label = self.status_text if time.monotonic() < self.status_until else "S: save snapshot | A: auto events | Q/ESC: quit"
        cv2.putText(image, label, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, label, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow("stable seam debug", image)

    def _poll_key(self) -> None:
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            self.stop_requested.set()
        elif key in (ord("s"), ord("S")):
            if not self.snapshot_client.wait_for_service(timeout_sec=0.0):
                self._set_status("SNAPSHOT SERVICE UNAVAILABLE", 2.5)
                self.get_logger().warning("snapshot service unavailable")
                return
            future = self.snapshot_client.call_async(Trigger.Request())
            future.add_done_callback(self._snapshot_response)
            self._set_status("SNAPSHOT REQUESTED", 1.5)
        elif key in (ord("a"), ord("A")):
            if not self.parameter_client.wait_for_service(timeout_sec=0.0):
                self._set_status("PARAMETER SERVICE UNAVAILABLE", 2.5)
                return
            self.auto_save_enabled = not self.auto_save_enabled
            request = SetParameters.Request()
            request.parameters = [Parameter("auto_save_failures", Parameter.Type.BOOL,
                                             self.auto_save_enabled).to_parameter_msg()]
            future = self.parameter_client.call_async(request)
            future.add_done_callback(self._parameter_response)
            self._set_status(f"AUTO FAILURE EVENTS: {self.auto_save_enabled}", 2.5)

    def _parameter_response(self, future) -> None:
        try:
            results = future.result().results
            if not results or not all(item.successful for item in results):
                self._set_status("AUTO EVENT PARAMETER REJECTED", 3.0)
        except Exception as exc:  # noqa: BLE001 - viewer must remain usable
            self._set_status("PARAMETER SERVICE ERROR", 3.0)
            self.get_logger().error(f"parameter service call failed: {exc}")

    def _snapshot_response(self, future) -> None:
        try:
            response = future.result()
            if response.success:
                self._set_status("SNAPSHOT QUEUED", 2.5)
                self.get_logger().info("snapshot queued")
            else:
                self._set_status(f"SNAPSHOT REJECTED: {response.message}", 3.0)
                self.get_logger().warning(f"snapshot rejected: {response.message}")
        except Exception as exc:  # noqa: BLE001 - GUI must remain usable if service disappears
            self._set_status("SNAPSHOT SERVICE ERROR", 3.0)
            self.get_logger().error(f"snapshot service call failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeamDebugViewer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.stop_requested.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.stop_requested.set()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        cv2.destroyAllWindows()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
