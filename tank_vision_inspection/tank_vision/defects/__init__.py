"""Surface-anomaly detection and future model backends."""

from .classical_baseline import ClassicalDefectBaseline
from .detector import DefectDetector
from .onnx_backend import ONNXBackend
from .tiled_inference import Tile, TiledInference

__all__ = ["ClassicalDefectBaseline", "DefectDetector", "ONNXBackend", "Tile", "TiledInference"]

