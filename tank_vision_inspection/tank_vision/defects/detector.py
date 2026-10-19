"""Defect detector contract."""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from tank_vision.types import DefectResult, FrameData


class DefectDetector(ABC):
    backend_name: str
    last_warning: str | None = None

    @abstractmethod
    def detect(self, frame: FrameData, exclusion_mask: np.ndarray | None = None) -> list[DefectResult]:
        """Return conservative anomaly candidates, never invented semantic classes."""

