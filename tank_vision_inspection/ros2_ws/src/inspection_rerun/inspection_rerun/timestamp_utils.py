"""Integer-nanosecond ROS timestamp handling and visualization rate limiting."""

from dataclasses import dataclass
from typing import Any


NSEC_PER_SEC = 1_000_000_000


def stamp_to_nanoseconds(stamp: Any) -> int:
    if stamp is None:
        return 0
    return int(stamp.sec) * NSEC_PER_SEC + int(stamp.nanosec)


def message_timestamp_ns(message: Any, receive_time_ns: int) -> tuple[int, bool]:
    header = getattr(message, "header", None)
    stamp_ns = stamp_to_nanoseconds(getattr(header, "stamp", None))
    if stamp_ns > 0:
        return stamp_ns, False
    return int(receive_time_ns), True


@dataclass
class RateLimiter:
    max_hz: float
    last_ns: int = 0

    def allow(self, timestamp_ns: int) -> bool:
        if self.max_hz <= 0.0:
            return True
        period_ns = max(1, int(NSEC_PER_SEC / self.max_hz))
        timestamp_ns = int(timestamp_ns)
        if self.last_ns == 0 or timestamp_ns < self.last_ns or timestamp_ns - self.last_ns >= period_ns:
            self.last_ns = timestamp_ns
            return True
        return False


class TimelineTracker:
    """Tracks per-stream regression while maintaining a global sequence timeline."""

    def __init__(self) -> None:
        self._last_by_stream: dict[str, int] = {}
        self._sequence = 0

    def observe(self, stream: str, timestamp_ns: int) -> tuple[int, bool]:
        timestamp_ns = int(timestamp_ns)
        previous = self._last_by_stream.get(stream)
        regressed = previous is not None and timestamp_ns < previous
        self._last_by_stream[stream] = timestamp_ns
        sequence = self._sequence
        self._sequence += 1
        return sequence, regressed
