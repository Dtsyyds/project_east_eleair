"""Seam detector contract."""

from __future__ import annotations

from abc import ABC, abstractmethod

from tank_vision.types import FrameData, SeamResult


class SeamDetector(ABC):
    @abstractmethod
    def detect(self, frame: FrameData) -> SeamResult:
        """Detect a seam while preserving ``frame.timestamp_ns``."""

