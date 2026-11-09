"""Video-file input source with timestamps derived from the media timeline."""

from __future__ import annotations

from pathlib import Path
import time

import cv2

from tank_vision.sources.base import FrameSource, SourceError
from tank_vision.metrics import FrameMetrics, StageTiming
from tank_vision.types import FrameData


class VideoSource(FrameSource):
    def __init__(self, path: str | Path, frame_id: str = "camera") -> None:
        self.path = Path(path)
        if not self.path.is_file():
            raise SourceError(f"Video file does not exist: {self.path}")
        self.capture = cv2.VideoCapture(str(self.path))
        if not self.capture.isOpened():
            raise SourceError(f"Could not open video: {self.path}")
        self.base_timestamp_ns = self.path.stat().st_mtime_ns
        self.sequence = 0
        self.frame_id = frame_id
        self.source_name = f"video:{self.path}"

    def __next__(self) -> FrameData:
        source_started_ns = time.perf_counter_ns()
        ok, image = self.capture.read()
        source_elapsed_ns = time.perf_counter_ns() - source_started_ns
        if not ok or image is None:
            self.close()
            raise StopIteration
        position_ms = max(0.0, float(self.capture.get(cv2.CAP_PROP_POS_MSEC)))
        timestamp_ns = self.base_timestamp_ns + int(position_ms * 1_000_000)
        timing = StageTiming()
        timing.add_ns("source_read_ms", source_elapsed_ns)
        frame = FrameData(
            image=image,
            timestamp_ns=timestamp_ns,
            frame_id=self.frame_id,
            sequence_id=self.sequence,
            source_name=self.source_name,
            original_filename=self.path.name,
            metrics=FrameMetrics(timing=timing),
        )
        self.sequence += 1
        return frame

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
