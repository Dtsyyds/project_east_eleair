"""Safe S100P Nash-m BPU adapter boundary."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any, Mapping, Protocol

import numpy as np

from tank_vision.inference.base import InferenceBackend, TensorSpec


class BpuRuntimeAdapter(Protocol):
    input_spec: tuple[TensorSpec, ...]
    output_spec: tuple[TensorSpec, ...]

    def load_model(self, path: Path, metadata: Mapping[str, Any]) -> None: ...
    def infer(self, inputs: np.ndarray | Mapping[str, np.ndarray]) -> list[np.ndarray]: ...
    def warmup(self, iterations: int) -> None: ...
    def close(self) -> None: ...


class DRoboticsBpuBackend(InferenceBackend):
    """Does not guess vendor API symbols; an SDK-reviewed adapter is injected."""

    name = "d_robotics_bpu"

    def __init__(self, adapter: BpuRuntimeAdapter | None = None) -> None:
        self.adapter = adapter
        self.runtime_module = self._detect_runtime()
        self._reason = None if adapter is not None else self._default_reason()

    @staticmethod
    def _detect_runtime() -> str | None:
        for module in ("hbm_runtime", "hobot_dnn"):
            try:
                if importlib.util.find_spec(module) is not None:
                    return module
            except (ImportError, ValueError):
                continue
        return None

    def _default_reason(self) -> str:
        if self.runtime_module is None:
            return "S100P BPU runtime unavailable: neither hbm_runtime nor hobot_dnn is installed"
        return f"{self.runtime_module} detected, but an SDK-reviewed Nash-m adapter is not configured"

    @property
    def available(self) -> bool:
        return self.adapter is not None

    @property
    def unavailable_reason(self) -> str | None:
        return None if self.available else self._reason

    @property
    def input_spec(self) -> tuple[TensorSpec, ...]:
        return self.adapter.input_spec if self.adapter is not None else ()

    @property
    def output_spec(self) -> tuple[TensorSpec, ...]:
        return self.adapter.output_spec if self.adapter is not None else ()

    def load_model(self, model_path: str | Path, metadata: Mapping[str, Any] | None = None) -> None:
        if self.adapter is None:
            raise RuntimeError(self.unavailable_reason)
        path = Path(model_path)
        if not path.is_file():
            raise FileNotFoundError(f"S100P Nash-m model does not exist: {path}")
        model_metadata = dict(metadata or {})
        if model_metadata.get("target_arch") != "nash_m":
            raise ValueError("S100P requires model metadata target_arch=nash_m; X5 and other RDK models cannot be mixed")
        if str(model_metadata.get("input_format", "")).lower() not in {"bgr", "rgb", "nv12", "gray"}:
            raise ValueError("S100P model metadata must declare BGR, RGB, NV12, or gray input_format")
        self.adapter.load_model(path, model_metadata)

    def infer(self, inputs: np.ndarray | Mapping[str, np.ndarray]) -> list[np.ndarray]:
        if self.adapter is None:
            raise RuntimeError(self.unavailable_reason)
        return [np.asarray(item) for item in self.adapter.infer(inputs)]

    def warmup(self, iterations: int = 1) -> None:
        if self.adapter is None:
            raise RuntimeError(self.unavailable_reason)
        self.adapter.warmup(max(0, iterations))

    def close(self) -> None:
        if self.adapter is not None:
            self.adapter.close()

