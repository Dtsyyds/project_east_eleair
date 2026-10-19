from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np
import pytest
import yaml

from tank_vision.config import load_config
from tank_vision.inference import DRoboticsBpuBackend, RknnBackend, TensorSpec
from tank_vision.platforms import detect_platform
from tank_vision.sources.mindvision_probe import probe_mindvision_sdk


class FakeBpuAdapter:
    input_spec = (TensorSpec("image", (1, 3, 4, 4), "float32", "nchw", "rgb"),)
    output_spec = (TensorSpec("mask", (1, 1, 4, 4), "float32"),)

    def __init__(self) -> None:
        self.loaded = False
        self.closed = False

    def load_model(self, path: Path, metadata: Mapping[str, Any]) -> None:
        assert path.is_file()
        assert metadata["target_arch"] == "nash_m"
        self.loaded = True

    def infer(self, inputs):
        assert self.loaded
        return [np.zeros((1, 1, 4, 4), dtype=np.float32)]

    def warmup(self, iterations: int) -> None:
        assert iterations >= 0

    def close(self) -> None:
        self.closed = True


def test_platform_detection_reports_current_host() -> None:
    info = detect_platform()
    assert info.machine_architecture
    assert info.python_version
    assert info.platform_name in {"x86", "s100p", "rk3588", "unknown"}
    assert info.recommended_backend in {"opencv", "onnx", "d_robotics_bpu", "rknn"}


def test_invalid_platform_override_is_rejected() -> None:
    with pytest.raises(ValueError, match="TANK_VISION_PLATFORM"):
        detect_platform("imaginary")


def test_s100p_backend_without_sdk_is_safely_unavailable() -> None:
    backend = DRoboticsBpuBackend()
    if backend.runtime_module is None:
        assert not backend.available
        assert backend.unavailable_reason and "hbm_runtime" in backend.unavailable_reason


def test_fake_bpu_adapter_exercises_contract_without_claiming_hardware(tmp_path: Path) -> None:
    model = tmp_path / "fake_test_only.hbm"
    model.write_bytes(b"not a real model")
    adapter = FakeBpuAdapter()
    backend = DRoboticsBpuBackend(adapter)
    backend.load_model(model, {"target_arch": "nash_m", "input_format": "rgb"})
    outputs = backend.infer(np.zeros((1, 3, 4, 4), dtype=np.float32))
    assert outputs[0].shape == (1, 1, 4, 4)
    backend.close()
    assert adapter.closed


def test_fake_bpu_rejects_wrong_rdk_target(tmp_path: Path) -> None:
    model = tmp_path / "wrong.hbm"
    model.write_bytes(b"test only")
    backend = DRoboticsBpuBackend(FakeBpuAdapter())
    with pytest.raises(ValueError, match="nash_m"):
        backend.load_model(model, {"target_arch": "x5", "input_format": "nv12"})


def test_rknn_without_runtime_is_safely_unavailable() -> None:
    backend = RknnBackend()
    if backend.runtime_module is None:
        assert not backend.available
        assert backend.unavailable_reason and "RKNN Runtime" in backend.unavailable_reason


def test_empty_platform_model_path_is_allowed_but_not_fabricated() -> None:
    root = Path(__file__).parents[1]
    s100p = load_config(root / "configs" / "platforms" / "s100p.yaml")
    assert s100p.inference.model_path is None
    assert s100p.inference.target_arch == "nash_m"


def test_wrong_platform_model_configuration_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "wrong.yaml"
    path.write_text(
        yaml.safe_dump({"platform": {"name": "s100p"}, "inference": {"backend": "rknn"}}),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="RKNN models"):
        load_config(path)


@pytest.mark.camera
def test_mindvision_sdk_or_camera_is_explicitly_skipped_without_sdk() -> None:
    probe = probe_mindvision_sdk()
    if not probe.available:
        pytest.skip(probe.reason)
    pytest.skip("SDK files found, but an attached camera and reviewed official adapter are required")
