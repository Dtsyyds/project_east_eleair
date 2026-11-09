#!/usr/bin/env python3
"""Small live tuning UI for StableSeamNode ROS parameters and debug output.

This tool never starts a camera and never edits the YAML/reference files.  It
updates only runtime ``tune_*`` parameters on an already running stable seam
node, so closing the UI leaves the configured defaults untouched.
"""

from __future__ import annotations

import argparse
import base64
import math
from pathlib import Path
import threading
import tkinter as tk
from tkinter import ttk

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tank_vision_interfaces.msg import SeamGeometry
import yaml


CONTROL_SPECS = (
    ("tune_minimum_point_count", "geometry.minimum_point_count", "最少有效点", 1, 200, 1),
    ("tune_minimum_coverage", "geometry.minimum_coverage", "最小覆盖率", 0.0, 1.0, 0.01),
    ("tune_minimum_effective_span", "geometry.minimum_effective_span", "最小有效跨度", 0.0, 1.0, 0.01),
    ("tune_minimum_inlier_ratio", "geometry.minimum_inlier_ratio", "最小joint inlier", 0.0, 1.0, 0.01),
    ("tune_maximum_fit_residual_px", "geometry.maximum_fit_residual_px", "最大残差(px)", 0.0, 100.0, 0.5),
    ("tune_minimum_path_confidence", "geometry.minimum_path_confidence", "最小路径置信度", 0.0, 1.0, 0.01),
    ("tune_geometry_maximum_missing_run", "geometry.maximum_missing_run", "最大连续缺失", 0, 30, 1),
    ("tune_edge_min_response", "processing.edge_min_response", "最小边缘响应", 0.0, 20.0, 0.1),
    ("tune_edge_percentile", "processing.edge_percentile", "边缘百分位", 0.0, 100.0, 0.5),
    ("tune_edge_min_separation_px", "processing.edge_min_separation_px", "边缘最小间距(px)", 0, 100, 1),
    ("tune_guided_search_radius_width_scale", "guided_search.local_search_radius_width_scale", "guided搜索半径(宽度倍数)", 0.0, 3.0, 0.05),
)


def nested_value(document: dict, dotted_path: str):
    value = document
    for key in dotted_path.split("."):
        value = value[key]
    return value


class TuningNode(Node):
    def __init__(self, node_name: str, debug_topic: str) -> None:
        super().__init__("seam_tuning_ui")
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image_b64: str | None = None
        self.latest_metrics: dict[str, object] = {}
        self.last_error = ""
        self.parameter_send_lock = threading.Lock()
        self.parameter_client = self.create_client(SetParameters, f"/{node_name}/set_parameters")
        # Debug images are published with sensor-data QoS (BEST_EFFORT).  A
        # RELIABLE subscription is incompatible with that publisher in ROS 2
        # and would leave the UI showing "waiting for debug_image" while the
        # geometry metrics continue to update.
        self.create_subscription(Image, debug_topic, self._debug_callback, qos_profile_sensor_data)
        self.create_subscription(SeamGeometry, "/vision/front/seam_geometry", self._geometry_callback, 10)

    def _debug_callback(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            scale = min(1.0, 1280 / image.shape[1], 800 / image.shape[0])
            if scale < 1.0:
                image = cv2.resize(image, (round(image.shape[1] * scale), round(image.shape[0] * scale)), interpolation=cv2.INTER_AREA)
            # Tk PhotoImage does not reliably decode JPEG bytes on all target
            # images.  PNG is supported by the Tk build used on the board and
            # keeps the debug overlay lossless enough for threshold tuning.
            ok, encoded = cv2.imencode(".png", image)
            if ok:
                with self.lock:
                    self.latest_image_b64 = base64.b64encode(encoded.tobytes()).decode("ascii")
        except Exception as exc:  # UI should remain alive if one frame is malformed
            with self.lock:
                self.last_error = f"debug image: {exc}"

    def _geometry_callback(self, message: SeamGeometry) -> None:
        with self.lock:
            self.latest_metrics = {
                "state": {0: "LOST", 1: "ACQUIRING", 2: "TRACKING", 3: "DEGRADED"}.get(int(message.tracking_state), "UNKNOWN"),
                "detected": bool(message.detected),
                "confidence": float(message.confidence),
                "coverage": float(message.coverage),
                "span": float(message.effective_span),
                "residual": float(message.fit_residual_px),
                "processing_ms": float(message.processing_ms),
                "points": len(message.centerline),
                "reasons": ", ".join(str(value) for value in message.failure_reasons),
                "reason_list": [str(value) for value in message.failure_reasons],
            }

    def submit_parameters(self, values: dict[str, object]) -> None:
        if not values:
            return
        if not self.parameter_client.service_is_ready():
            with self.lock:
                self.last_error = "参数服务发现中，正在重试…"
            threading.Thread(target=self._submit_after_service_discovery, args=(dict(values),), daemon=True).start()
            return
        self._send_parameters(values)

    def _submit_after_service_discovery(self, values: dict[str, object]) -> None:
        if not self.parameter_client.wait_for_service(timeout_sec=2.0):
            with self.lock:
                self.last_error = "参数服务未连接（2秒内未发现 /set_parameters）"
            return
        self._send_parameters(values)

    def _send_parameters(self, values: dict[str, object]) -> None:
        # Serialize requests so rapid slider movements cannot race each other
        # while the service graph is being discovered.
        with self.parameter_send_lock:
            if not self.parameter_client.service_is_ready():
                with self.lock:
                    self.last_error = "参数服务暂时不可用，未发送本次调节"
                return
            request = SetParameters.Request()
            request.parameters = [Parameter(name=name, value=value).to_parameter_msg() for name, value in values.items()]
            future = self.parameter_client.call_async(request)
            future.add_done_callback(self._parameter_result)

    def _parameter_result(self, future) -> None:
        try:
            results = future.result().results
            failed = [item.reason for item in results if not item.successful]
            with self.lock:
                self.last_error = "; ".join(failed) if failed else ""
        except Exception as exc:  # service may disappear during shutdown
            with self.lock:
                self.last_error = f"参数服务错误: {exc}"


class TuningUi:
    def __init__(self, root: tk.Tk, node: TuningNode, config: dict) -> None:
        self.root = root
        self.node = node
        self.defaults: dict[str, object] = {}
        self.variables: dict[str, tk.DoubleVar] = {}
        self.pending: dict[str, object] = {}
        self.flush_scheduled = False
        self.photo: tk.PhotoImage | None = None
        self.gate_status_labels: dict[str, tk.Label] = {}
        self.root.title("Stable Seam 实时调参")
        self.root.geometry("1500x960")
        self._build(config)
        self.root.after(100, self._refresh)

    def _build(self, config: dict) -> None:
        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True)
        image_frame = ttk.Frame(main, padding=8)
        control_frame = ttk.Frame(main, padding=8)
        main.add(image_frame, weight=4)
        main.add(control_frame, weight=2)
        self.image_label = ttk.Label(image_frame, text="等待debug_image…")
        self.image_label.pack(fill=tk.BOTH, expand=True)
        self.status_label = ttk.Label(image_frame, text="等待SeamGeometry…", justify=tk.LEFT)
        self.status_label.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(control_frame, text="运行时阈值（仅覆盖当前节点，不写回配置文件）").pack(anchor=tk.W, pady=(0, 8))
        ttk.Label(control_frame, text="当前帧：OK=满足，FAIL=不满足，N/A=当前消息未提供直接观测").pack(anchor=tk.W, pady=(0, 6))
        for parameter_name, path, label, minimum, maximum, resolution in CONTROL_SPECS:
            default = nested_value(config, path)
            self.defaults[parameter_name] = default
            variable = tk.DoubleVar(value=float(default))
            self.variables[parameter_name] = variable
            row = ttk.Frame(control_frame)
            row.pack(fill=tk.X, pady=3)
            ttk.Label(row, text=label, width=22).pack(side=tk.LEFT)
            scale = tk.Scale(row, variable=variable, from_=minimum, to=maximum, resolution=resolution,
                             orient=tk.HORIZONTAL, showvalue=False, length=230,
                             command=lambda value, name=parameter_name: self._changed(name, value))
            scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
            value_label = ttk.Label(row, width=8)
            value_label.pack(side=tk.LEFT)
            gate_status = tk.Label(row, text="当前帧:—", width=21, anchor=tk.W, fg="#777777")
            gate_status.pack(side=tk.LEFT, padx=(6, 0))
            self.gate_status_labels[parameter_name] = gate_status
            variable.trace_add("write", lambda *_args, var=variable, label_widget=value_label: label_widget.configure(text=f"{var.get():.3g}"))
            value_label.configure(text=f"{variable.get():.3g}")
        buttons = ttk.Frame(control_frame)
        buttons.pack(fill=tk.X, pady=12)
        ttk.Button(buttons, text="应用全部", command=self._apply_all).pack(side=tk.LEFT, padx=(0, 6))
        ttk.Button(buttons, text="恢复配置默认", command=self._reset).pack(side=tk.LEFT)
        self.error_label = ttk.Label(control_frame, foreground="red", wraplength=430)
        self.error_label.pack(fill=tk.X, pady=8)

    def _changed(self, name: str, value: str) -> None:
        default = self.defaults[name]
        self.pending[name] = int(round(float(value))) if isinstance(default, int) else float(value)
        if not self.flush_scheduled:
            self.flush_scheduled = True
            self.root.after(120, self._flush)

    def _flush(self) -> None:
        self.flush_scheduled = False
        values, self.pending = self.pending, {}
        self.node.submit_parameters(values)

    def _apply_all(self) -> None:
        self.node.submit_parameters({name: int(round(var.get())) if isinstance(self.defaults[name], int) else float(var.get())
                                     for name, var in self.variables.items()})

    def _reset(self) -> None:
        for name, value in self.defaults.items():
            self.variables[name].set(float(value))
        self._apply_all()

    @staticmethod
    def _gate_status(parameter_name: str, metrics: dict[str, object], threshold: float,
                     reasons: set[str]) -> tuple[str, str]:
        """Report only checks supported by the current SeamGeometry message."""
        detected = bool(metrics.get("detected", False))
        reason_by_parameter = {
            "tune_minimum_point_count": "insufficient_points",
            "tune_minimum_coverage": "coverage_below_minimum",
            "tune_minimum_effective_span": "effective_span_below_minimum",
            "tune_minimum_inlier_ratio": "inlier_ratio_below_minimum",
            "tune_maximum_fit_residual_px": "fit_residual_above_maximum",
            "tune_minimum_path_confidence": "path_confidence_below_minimum",
            "tune_geometry_maximum_missing_run": "missing_run_too_long",
        }
        reason = reason_by_parameter.get(parameter_name)
        if reason in reasons:
            if parameter_name == "tune_minimum_coverage":
                return f"FAIL {float(metrics.get('coverage', 0.0)):.3f}<{threshold:.3f}", "#c62828"
            if parameter_name == "tune_minimum_effective_span":
                return f"FAIL {float(metrics.get('span', 0.0)):.3f}<{threshold:.3f}", "#c62828"
            if parameter_name == "tune_maximum_fit_residual_px":
                residual = metrics.get("residual")
                if residual is not None and math.isfinite(float(residual)):
                    return f"FAIL {float(residual):.1f}>{threshold:.1f}", "#c62828"
                return "FAIL fit unavailable", "#c62828"
            return "FAIL current reason", "#c62828"

        direct_values = {
            "tune_minimum_coverage": (metrics.get("coverage"), lambda value: value >= threshold, "{:.3f}"),
            "tune_minimum_effective_span": (metrics.get("span"), lambda value: value >= threshold, "{:.3f}"),
            "tune_maximum_fit_residual_px": (metrics.get("residual"), lambda value: value <= threshold, "{:.1f}"),
        }
        if parameter_name in direct_values:
            observed, predicate, formatter = direct_values[parameter_name]
            if observed is not None and math.isfinite(float(observed)):
                observed = float(observed)
                if predicate(observed):
                    return f"OK {formatter.format(observed)}", "#2e7d32"
                return f"FAIL {formatter.format(observed)}", "#c62828"

        # A detected frame passed all tracker gates.  For a rejected frame,
        # only an explicit failure reason is safe to attribute.  The current
        # SeamGeometry message does not publish edge-response or search-radius
        # observations, so those controls remain explicitly N/A.
        if detected and parameter_name in reason_by_parameter:
            return "OK gate passed", "#2e7d32"
        return "N/A not published", "#777777"

    def _update_gate_status(self, metrics: dict[str, object]) -> None:
        reasons = set(str(value) for value in metrics.get("reason_list", []))
        for parameter_name, label in self.gate_status_labels.items():
            threshold = float(self.variables[parameter_name].get())
            text, colour = self._gate_status(parameter_name, metrics, threshold, reasons)
            label.configure(text=text, fg=colour)

    def _refresh(self) -> None:
        with self.node.lock:
            image_b64 = self.node.latest_image_b64
            metrics = dict(self.node.latest_metrics)
            error = self.node.last_error
        if image_b64:
            try:
                photo = tk.PhotoImage(data=image_b64, format="png")
                # The PanedWindow leaves less space for the image when the
                # tuning controls are visible.  Tk labels do not scale an
                # image automatically, so fit it to the current viewport
                # before assigning it; otherwise the right/bottom part is
                # simply clipped.
                viewport_width = self.image_label.winfo_width() - 8
                viewport_height = self.image_label.winfo_height() - 8
                if viewport_width > 100 and viewport_height > 100:
                    scale = max(photo.width() / viewport_width, photo.height() / viewport_height)
                    reduction = max(1, math.ceil(scale))
                    if reduction > 1:
                        photo = photo.subsample(reduction, reduction)
                self.photo = photo
                self.image_label.configure(image=self.photo, text="")
            except tk.TclError as exc:
                self.error_label.configure(text=f"图像显示错误: {exc}")
        if metrics:
            self._update_gate_status(metrics)
            self.status_label.configure(text=(
                f"state={metrics.get('state')} detected={metrics.get('detected')} conf={metrics.get('confidence', 0):.3f}\n"
                f"coverage={metrics.get('coverage', 0):.3f} span={metrics.get('span', 0):.3f} "
                f"residual={metrics.get('residual')} ms={metrics.get('processing_ms', 0):.2f} "
                f"points={metrics.get('points')}\nreasons={metrics.get('reasons') or 'none'}"))
        self.error_label.configure(text=error)
        self.root.after(100, self._refresh)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="same tracker YAML used by stable_seam_node")
    parser.add_argument("--node-name", default="stable_seam_node")
    parser.add_argument("--debug-topic", default="/vision/front/debug_image")
    args = parser.parse_args(argv)
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    rclpy.init()
    node = TuningNode(args.node_name, args.debug_topic)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    root = tk.Tk()
    TuningUi(root, node, config)
    root.protocol("WM_DELETE_WINDOW", root.destroy)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
