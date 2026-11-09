"""SDK-neutral inference backends with optional vendor runtimes."""

from tank_vision.inference.base import InferenceBackend, TensorSpec
from tank_vision.inference.d_robotics_bpu_backend import DRoboticsBpuBackend
from tank_vision.inference.onnx_backend import OnnxBackend
from tank_vision.inference.opencv_backend import OpenCvDnnBackend
from tank_vision.inference.rknn_backend import RknnBackend

__all__ = [
    "DRoboticsBpuBackend",
    "InferenceBackend",
    "OnnxBackend",
    "OpenCvDnnBackend",
    "RknnBackend",
    "TensorSpec",
]
