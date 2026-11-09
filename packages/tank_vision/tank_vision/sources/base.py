"""Acquisition source contracts."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Iterator

from tank_vision.types import FrameData


class SourceError(RuntimeError):
    """A recoverable or fatal acquisition-source error."""


class FrameSource(ABC, Iterator[FrameData]):
    """Iterator that returns frames with acquisition metadata."""

    source_name: str

    def __iter__(self) -> "FrameSource":
        return self

    @abstractmethod
    def __next__(self) -> FrameData:
        raise StopIteration

    def close(self) -> None:
        """Release source resources."""

    def __enter__(self) -> "FrameSource":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

