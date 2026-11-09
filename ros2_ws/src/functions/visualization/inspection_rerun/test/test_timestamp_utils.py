from inspection_rerun.timestamp_utils import RateLimiter, TimelineTracker, message_timestamp_ns, stamp_to_nanoseconds
from inspection_rerun.inspection_adapter import diagnostic_level


class Stamp:
    def __init__(self, sec=0, nanosec=0): self.sec, self.nanosec = sec, nanosec


class Header:
    def __init__(self, stamp): self.stamp = stamp


class Message:
    def __init__(self, stamp): self.header = Header(stamp)


def test_integer_nanosecond_conversion():
    assert stamp_to_nanoseconds(Stamp(123, 456789012)) == 123456789012


def test_zero_stamp_uses_receive_time():
    assert message_timestamp_ns(Message(Stamp()), 987654321) == (987654321, True)


def test_regression_and_sequence_are_independent():
    tracker = TimelineTracker()
    assert tracker.observe("camera", 100) == (0, False)
    assert tracker.observe("imu", 50) == (1, False)
    assert tracker.observe("camera", 99) == (2, True)


def test_rate_limiter_drops_until_period_and_recovers_on_replay_seek():
    limiter = RateLimiter(2.0)
    assert limiter.allow(1_000_000_000)
    assert not limiter.allow(1_100_000_000)
    assert limiter.allow(1_500_000_000)
    assert limiter.allow(100)


def test_humble_uint8_diagnostic_level_can_be_bytes_or_int():
    assert diagnostic_level(2) == 2
    assert diagnostic_level(b"\x02") == 2
