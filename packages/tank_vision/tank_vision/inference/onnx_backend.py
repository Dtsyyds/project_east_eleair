"""ONNX Runtime CPU backend with explicit tensor validation."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import cv2
import numpy as np

from tank_vision.inference.base import InferenceBackend, TensorSpec


def _shape_tuple(shape: list[Any]) -> tuple[int | str | None, ...]:
    return tuple(item if isinstance(item, (int, str)) or item is None else str(item) for item in shape)


class OnnxBackend(InferenceBackend):
    name = "onnxruntime_cpu"
    backend_name = name

    def __init__(self, model_path: str | Path | None = None) -> None:
        self.session: Any = None
        self.model_path: Path | None = None
        self._unavailable_reason: str | None = None
        self._input_spec: tuple[TensorSpec, ...] = ()
        self._output_spec: tuple[TensorSpec, ...] = ()
        self.metadata: dict[str, Any] = {}
        if model_path:
            self.load_model(model_path)
        else:
            self._unavailable_reason = "ONNX model_path is not configured"

    @property
    def error(self) -> str | None:
        """Compatibility alias for the original MVP backend."""

        return self._unavailable_reason

    @property
    def available(self) -> bool:
        return self.session is not None

    @property
    def unavailable_reason(self) -> str | None:
        return None if self.available else self._unavailable_reason

    @property
    def input_spec(self) -> tuple[TensorSpec, ...]:
        return self._input_spec

    @property
    def output_spec(self) -> tuple[TensorSpec, ...]:
        return self._output_spec

    def load_model(self, model_path: str | Path, metadata: Mapping[str, Any] | None = None) -> None:
        self.close()
        self.model_path = Path(model_path)
        self.metadata = dict(metadata or {})
        if not self.model_path.is_file():
            self._unavailable_reason = f"ONNX model does not exist: {self.model_path}"
            return
        try:
            import onnxruntime as ort  # type: ignore[import-not-found]

            self.session = ort.InferenceSession(str(self.model_path), providers=["CPUExecutionProvider"])
            self._input_spec = tuple(
                TensorSpec(item.name, _shape_tuple(item.shape), item.type, self.metadata.get("layout"), self.metadata.get("input_format"))
                for item in self.session.get_inputs()
            )
            self._output_spec = tuple(
                TensorSpec(item.name, _shape_tuple(item.shape), item.type) for item in self.session.get_outputs()
            )
            self._unavailable_reason = None
        except ImportError:
            self._unavailable_reason = "onnxruntime is not installed; install requirements/x86.txt"
        except Exception as exc:
            self._unavailable_reason = f"Could not load ONNX model: {exc}"
            self.session = None

    @staticmethod
    def _expected_dtype(type_name: str) -> np.dtype[Any] | None:
        mapping = {
            "tensor(float)": np.dtype(np.float32),
            "tensor(float16)": np.dtype(np.float16),
            "tensor(uint8)": np.dtype(np.uint8),
            "tensor(int8)": np.dtype(np.int8),
            "tensor(int64)": np.dtype(np.int64),
        }
        return mapping.get(type_name)

    @staticmethod
    def _validate_shape(actual: tuple[int, ...], expected: tuple[int | str | None, ...], name: str) -> None:
        if expected and len(actual) != len(expected):
            raise ValueError(f"Input {name} rank {len(actual)} does not match model rank {len(expected)}")
        for index, (actual_dim, expected_dim) in enumerate(zip(actual, expected)):
            if isinstance(expected_dim, int) and expected_dim > 0 and actual_dim != expected_dim:
                raise ValueError(f"Input {name} dimension {index} is {actual_dim}, expected {expected_dim}")

    def infer(self, inputs: np.ndarray | Mapping[str, np.ndarray]) -> list[np.ndarray]:
        if not self.available:
            raise RuntimeError(self.unavailable_reason or "ONNX backend is unavailable")
        if isinstance(inputs, np.ndarray):
            if len(self._input_spec) != 1:
                raise ValueError("A mapping is required for a multi-input ONNX model")
            feed = {self._input_spec[0].name: inputs}
        else:
            feed = dict(inputs)
        for spec in self._input_spec:
            if spec.name not in feed:
                raise ValueError(f"Missing model input: {spec.name}")
            tensor = feed[spec.name]
            if not isinstance(tensor, np.ndarray):
                raise TypeError(f"Input {spec.name} must be a NumPy array")
            expected_dtype = self._expected_dtype(spec.dtype)
            if expected_dtype is not None and tensor.dtype != expected_dtype:
                raise TypeError(f"Input {spec.name} dtype is {tensor.dtype}, expected {expected_dtype}")
            self._validate_shape(tensor.shape, spec.shape, spec.name)
        return [np.asarray(output) for output in self.session.run(None, feed)]

    def run_raw(self, image: np.ndarray, input_size: tuple[int, int] | None = None) -> list[np.ndarray]:
        """Legacy explicit BGR-to-NCHW float preprocessing helper."""

        if not isinstance(image, np.ndarray) or image.size == 0:
            raise ValueError("Cannot run ONNX on an empty image")
        prepared = cv2.resize(image, input_size, interpolation=cv2.INTER_LINEAR) if input_size else image
        prepared = cv2.cvtColor(prepared, cv2.COLOR_GRAY2RGB if prepared.ndim == 2 else cv2.COLOR_BGR2RGB)
        tensor = np.transpose(prepared.astype(np.float32) / 255.0, (2, 0, 1))[None, ...]
        return self.infer(tensor)

    def warmup(self, iterations: int = 1) -> None:
        if not self.available or not self._input_spec:
            raise RuntimeError(self.unavailable_reason or "ONNX backend is unavailable")
        arrays: dict[str, np.ndarray] = {}
        for spec in self._input_spec:
            if any(not isinstance(dim, int) or dim <= 0 for dim in spec.shape):
                raise ValueError("Cannot warm up a dynamic-shape model without concrete input data")
            dtype = self._expected_dtype(spec.dtype)
            if dtype is None:
                raise TypeError(f"Unsupported ONNX warmup dtype: {spec.dtype}")
            arrays[spec.name] = np.zeros(tuple(int(dim) for dim in spec.shape), dtype=dtype)
        for _ in range(max(0, iterations)):
            self.infer(arrays)

    def close(self) -> None:
        self.session = None
        self._input_spec = ()
        self._output_spec = ()


ONNXBackend = OnnxBackend

