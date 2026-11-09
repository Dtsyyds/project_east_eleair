"""Optional OpenCV DNN raw inference backend."""

from __future__ import annotations

from pathlib import Path
from typing import Mapping, Any

import cv2
import numpy as np

from tank_vision.inference.base import InferenceBackend, TensorSpec


class OpenCvDnnBackend(InferenceBackend):
    name = "opencv_dnn_cpu"

    def __init__(self) -> None:
        self.net: cv2.dnn.Net | None = None
        self._reason: str | None = "model is not loaded"
        self._input_spec: tuple[TensorSpec, ...] = ()
        self._output_spec: tuple[TensorSpec, ...] = ()

    @property
    def available(self) -> bool:
        return self.net is not None

    @property
    def unavailable_reason(self) -> str | None:
        return None if self.available else self._reason

    @property
    def input_spec(self) -> tuple[TensorSpec, ...]:
        return self._input_spec

    @property
    def output_spec(self) -> tuple[TensorSpec, ...]:
        return self._output_spec

    def load_model(self, model_path: str | Path, metadata: Mapping[str, Any] | None = None) -> None:
        path = Path(model_path)
        if not path.is_file():
            self._reason = f"OpenCV DNN model does not exist: {path}"
            return
        try:
            self.net = cv2.dnn.readNet(str(path))
            meta = dict(metadata or {})
            shape = tuple(meta.get("shape", ()))
            self._input_spec = (TensorSpec(str(meta.get("input_name", "input")), shape, str(meta.get("dtype", "float32")), meta.get("layout"), meta.get("input_format")),)
            self._reason = None
        except cv2.error as exc:
            self.net = None
            self._reason = f"Could not load OpenCV DNN model: {exc}"

    def infer(self, inputs: np.ndarray | Mapping[str, np.ndarray]) -> list[np.ndarray]:
        if not self.available or self.net is None:
            raise RuntimeError(self.unavailable_reason or "OpenCV DNN backend is unavailable")
        if not isinstance(inputs, np.ndarray):
            raise TypeError("OpenCV DNN backend currently supports one explicit input tensor")
        self.net.setInput(inputs)
        names = self.net.getUnconnectedOutLayersNames()
        outputs = self.net.forward(names) if names else [self.net.forward()]
        return [np.asarray(item) for item in outputs]

    def warmup(self, iterations: int = 1) -> None:
        if not self._input_spec or not self._input_spec[0].shape:
            raise ValueError("OpenCV warmup requires concrete shape metadata")
        shape = self._input_spec[0].shape
        if any(not isinstance(dim, int) or dim <= 0 for dim in shape):
            raise ValueError("OpenCV warmup requires a static input shape")
        tensor = np.zeros(tuple(int(dim) for dim in shape), dtype=np.float32)
        for _ in range(max(0, iterations)):
            self.infer(tensor)

    def close(self) -> None:
        self.net = None

