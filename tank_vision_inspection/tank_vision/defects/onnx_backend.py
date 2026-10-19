"""Backward-compatible imports for inference backends."""

from tank_vision.inference.onnx_backend import ONNXBackend, OnnxBackend
from tank_vision.inference.rknn_backend import RknnBackend


class RK3588NPUBackend(RknnBackend):
    """Original MVP class name retained for callers."""


__all__ = ["ONNXBackend", "OnnxBackend", "RK3588NPUBackend", "RknnBackend"]
