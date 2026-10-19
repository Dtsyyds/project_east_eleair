"""Generic OpenCV camera source for development and fallback acquisition."""

from __future__ import annotations

import time

import cv2

from tank_vision.sources.base import FrameSource, SourceError
from tank_vision.metrics import FrameMetrics, StageTiming
from tank_vision.types import FrameData


class OpenCVCameraSource(FrameSource):
    def __init__(
        self,
        device: int | str = 0,
        width: int | None = None,
        height: int | None = None,
        frame_id: str = "camera",
    ) -> None:
        self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            raise SourceError(f"Could not open OpenCV camera device: {device}")
        if width:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.device = device
        self.sequence = 0
        self.frame_id = frame_id
        self.source_name = f"opencv_camera:{device}"

    def __next__(self) -> FrameData:
        source_started_ns = time.perf_counter_ns()
        ok, image = self.capture.read()
        source_elapsed_ns = time.perf_counter_ns() - source_started_ns
        # This is captured immediately after read; a vendor hardware timestamp should replace it.
        timestamp_ns = time.time_ns()
        if not ok or image is None:
            raise SourceError(f"Failed to acquire frame from {self.source_name}")
        timing = StageTiming()
        timing.add_ns("source_read_ms", source_elapsed_ns)
        frame = FrameData(
            image=image,
            timestamp_ns=timestamp_ns,
            frame_id=self.frame_id,
            sequence_id=self.sequence,
            source_name=self.source_name,
            metrics=FrameMetrics(timing=timing),
        )
        self.sequence += 1
        return frame

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
