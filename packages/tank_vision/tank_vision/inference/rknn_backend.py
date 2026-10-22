"""Safe optional RK3588 RKNN runtime boundary."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any, Mapping, Protocol

import numpy as np

from tank_vision.inference.base import InferenceBackend, TensorSpec


class RknnRuntimeAdapter(Protocol):
    input_spec: tuple[TensorSpec, ...]
    output_spec: tuple[TensorSpec, ...]
    def load_model(self, path: Path, metadata: Mapping[str, Any]) -> None: ...
    def infer(self, inputs: np.ndarray | Mapping[str, np.ndarray]) -> list[np.ndarray]: ...
    def warmup(self, iterations: int) -> None: ...
    def close(self) -> None: ...


class RknnBackend(InferenceBackend):
    name = "rknn"

    def __init__(self, adapter: RknnRuntimeAdapter | None = None) -> None:
        self.adapter = adapter
        self.runtime_module = self._detect_runtime()

    @staticmethod
    def _detect_runtime() -> str | None:
        for module in ("rknnlite.api", "rknn.api"):
            try:
                if importlib.util.find_spec(module) is not None:
                    return module
            except (ImportError, ModuleNotFoundError, ValueError):
                continue
        return None

    @property
    def available(self) -> bool:
        return self.adapter is not None

    @property
    def unavailable_reason(self) -> str | None:
        if self.available:
            return None
        if self.runtime_module is None:
            return "RKNN Runtime is not installed; RK3588 acceleration is unavailable"
        return f"{self.runtime_module} detected, but an SDK-reviewed RK3588 adapter is not configured"

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
            raise FileNotFoundError(f"RKNN model does not exist: {path}")
        model_metadata = dict(metadata or {})
        if model_metadata.get("target_arch") != "rk3588":
            raise ValueError("RKNN model metadata must declare target_arch=rk3588")
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

