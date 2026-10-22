"""Multi-signal platform and optional-runtime detection."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import importlib.util
import os
from pathlib import Path
import platform
import sys

from tank_vision.sources.mindvision_probe import probe_mindvision_sdk


def _module_available(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ModuleNotFoundError, ValueError):
        return False


def _text_if_present(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").lower()
    except OSError:
        return ""


@dataclass(slots=True)
class PlatformInfo:
    platform_name: str
    machine_architecture: str
    operating_system: str
    python_version: str
    ros_distro: str | None
    mindvision_sdk: bool
    onnxruntime: bool
    hbm_runtime: bool
    hobot_dnn: bool
    rknn_runtime: bool
    recommended_backend: str
    unavailable_reasons: dict[str, str] = field(default_factory=dict)
    evidence: list[str] = field(default_factory=list)
    override: str | None = None

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def detect_platform(override: str | None = None) -> PlatformInfo:
    machine = platform.machine().lower()
    ros_distro = os.environ.get("ROS_DISTRO")
    hbm_runtime = _module_available("hbm_runtime")
    hobot_dnn = _module_available("hobot_dnn")
    rknn_runtime = _module_available("rknnlite.api") or _module_available("rknn.api")
    onnxruntime = _module_available("onnxruntime")
    model_text = " ".join(
        _text_if_present(path)
        for path in (Path("/proc/device-tree/model"), Path("/sys/firmware/devicetree/base/model"), Path("/etc/board-release"))
    )
    evidence = [f"machine={machine}"]
    if "rk3588" in model_text:
        detected = "rk3588"
        evidence.append("device_tree_contains_rk3588")
    elif any(token in model_text for token in ("s100p", "nash")) or hbm_runtime or hobot_dnn:
        detected = "s100p"
        evidence.append("s100p_board_identity_or_bpu_runtime")
    elif machine in {"x86_64", "amd64"}:
        detected = "x86"
        evidence.append("x86_architecture")
    else:
        detected = "unknown"
        evidence.append("unrecognized_arm_or_other_host")
    explicit = override or os.environ.get("TANK_VISION_PLATFORM")
    if explicit:
        explicit = explicit.lower()
        if explicit not in {"x86", "s100p", "rk3588"}:
            raise ValueError("TANK_VISION_PLATFORM must be x86, s100p, or rk3588")
        detected = explicit
        evidence.append(f"explicit_override={explicit}")
    reasons: dict[str, str] = {}
    expected_arch = {"x86": {"x86_64", "amd64"}, "s100p": {"aarch64", "arm64"}, "rk3588": {"aarch64", "arm64"}}
    if detected in expected_arch and machine not in expected_arch[detected]:
        reasons["platform"] = f"{detected} expects {sorted(expected_arch[detected])}, current machine is {machine}"
    if not onnxruntime:
        reasons["onnx"] = "onnxruntime is not installed"
    if not (hbm_runtime or hobot_dnn):
        reasons["d_robotics_bpu"] = "neither hbm_runtime nor hobot_dnn is installed"
    if not rknn_runtime:
        reasons["rknn"] = "neither rknnlite.api nor rknn.api is installed"
    camera_probe = probe_mindvision_sdk()
    if not camera_probe.available:
        reasons["mindvision"] = camera_probe.reason
    if detected == "s100p" and (hbm_runtime or hobot_dnn) and "platform" not in reasons:
        backend = "d_robotics_bpu"
    elif detected == "rk3588" and rknn_runtime and "platform" not in reasons:
        backend = "rknn"
    elif detected == "x86" and onnxruntime:
        backend = "onnx"
    else:
        backend = "opencv"
    return PlatformInfo(
        platform_name=detected,
        machine_architecture=machine,
        operating_system=platform.platform(),
        python_version=".".join(str(value) for value in sys.version_info[:3]),
        ros_distro=ros_distro,
        mindvision_sdk=camera_probe.available,
        onnxruntime=onnxruntime,
        hbm_runtime=hbm_runtime,
        hobot_dnn=hobot_dnn,
        rknn_runtime=rknn_runtime,
        recommended_backend=backend,
        unavailable_reasons=reasons,
        evidence=evidence,
        override=explicit,
    )
