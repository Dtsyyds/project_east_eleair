"""Map the MindVision device tick counter into the active ROS clock domain."""


class DeviceClockMapper:
    """Map a wrapping 32-bit 0.1 ms counter to nanoseconds.

    The first observed device tick is anchored to the ROS time sampled after that
    frame arrived. Subsequent stamps retain the device clock spacing and remain
    comparable with the other ROS sensor headers.
    """

    TICK_NS = 100_000
    WRAP = 1 << 32
    HALF_WRAP = 1 << 31

    def __init__(self):
        self._last_raw = None
        self._wraps = 0
        self._offset_ns = None
        self._last_device_ns = None

    @property
    def last_raw_tick(self):
        return self._last_raw

    @property
    def last_device_ns(self):
        return self._last_device_ns

    @property
    def wrap_count(self):
        return self._wraps

    def map(self, raw_tick, ros_now_ns):
        raw = int(raw_tick) & 0xFFFFFFFF
        if self._last_raw is not None and raw < self._last_raw:
            if self._last_raw - raw > self.HALF_WRAP:
                self._wraps += 1
            else:
                raise ValueError(
                    f"non-monotonic MindVision device tick: {raw} after {self._last_raw}"
                )
        self._last_raw = raw
        device_ns = (raw + self._wraps * self.WRAP) * self.TICK_NS
        if self._offset_ns is None:
            self._offset_ns = int(ros_now_ns) - device_ns
        self._last_device_ns = device_ns
        return device_ns + self._offset_ns
