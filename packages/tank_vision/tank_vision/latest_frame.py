"""Bounded latest-frame-only exchange used by the stable ROS node."""

from __future__ import annotations

from threading import Condition
from typing import Generic, TypeVar

T = TypeVar("T")


class LatestFrameSlot(Generic[T]):
    def __init__(self) -> None:
        self._condition = Condition()
        self._item: T | None = None
        self._closed = False
        self.input_count = 0
        self.overwritten_count = 0

    def put(self, item: T) -> bool:
        with self._condition:
            if self._closed:
                return False
            self.input_count += 1
            overwritten = self._item is not None
            self.overwritten_count += int(overwritten)
            self._item = item
            self._condition.notify()
            return not overwritten

    def get(self, timeout: float | None = None) -> T | None:
        with self._condition:
            if self._item is None and not self._closed:
                self._condition.wait(timeout)
            item, self._item = self._item, None
            return item

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()

    @property
    def closed(self) -> bool:
        with self._condition:
            return self._closed
