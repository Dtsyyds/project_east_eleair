"""Timestamp-preserving folder image source."""

from __future__ import annotations

import logging
from pathlib import Path
import time

import cv2

from tank_vision.sources.base import FrameSource, SourceError
from tank_vision.metrics import FrameMetrics, StageTiming
from tank_vision.types import FrameData

LOGGER = logging.getLogger(__name__)
IMAGE_SUFFIXES = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


class FolderSource(FrameSource):
    def __init__(self, folder: str | Path, recursive: bool = False, frame_id: str = "camera") -> None:
        self.folder = Path(folder)
        if not self.folder.is_dir():
            raise SourceError(f"Input folder does not exist: {self.folder}")
        iterator = self.folder.rglob("*") if recursive else self.folder.glob("*")
        self.paths = sorted(path for path in iterator if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES)
        self.index = 0
        self.sequence = 0
        self.frame_id = frame_id
        self.source_name = f"folder:{self.folder}"
        self.errors: list[str] = []

    def __next__(self) -> FrameData:
        while self.index < len(self.paths):
            path = self.paths[self.index]
            self.index += 1
            decode_started_ns = time.perf_counter_ns()
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            decode_elapsed_ns = time.perf_counter_ns() - decode_started_ns
            if image is None or image.size == 0:
                message = f"Unreadable image skipped: {path}"
                self.errors.append(message)
                LOGGER.warning(message)
                continue
            source_started_ns = time.perf_counter_ns()
            try:
                timestamp_ns = path.stat().st_mtime_ns
            except OSError:
                timestamp_ns = 0
                self.errors.append(f"Could not read file timestamp: {path}")
            source_elapsed_ns = time.perf_counter_ns() - source_started_ns
            timing = StageTiming()
            timing.add_ns("source_read_ms", source_elapsed_ns)
            timing.add_ns("decode_ms", decode_elapsed_ns)
            frame = FrameData(
                image=image,
                timestamp_ns=timestamp_ns,
                frame_id=self.frame_id,
                sequence_id=self.sequence,
                source_name=self.source_name,
                original_filename=path.name,
                metrics=FrameMetrics(timing=timing),
            )
            self.sequence += 1
            return frame
        raise StopIteration
