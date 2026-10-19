from mindvision_camera_driver.buffer import copy_uint8_buffer
from mindvision_camera_driver.clock_mapper import DeviceClockMapper

import ctypes


def test_maps_device_spacing_into_ros_epoch():
    mapper = DeviceClockMapper()
    first = mapper.map(100, 5_000_000_000)
    second = mapper.map(130, 9_000_000_000)
    assert first == 5_000_000_000
    assert second - first == 3_000_000


def test_unwraps_32_bit_counter():
    mapper = DeviceClockMapper()
    first = mapper.map(0xFFFFFFF0, 10_000_000_000)
    second = mapper.map(0x00000010, 10_010_000_000)
    assert second - first == 32 * mapper.TICK_NS


def test_small_out_of_order_value_does_not_fake_wrap():
    mapper = DeviceClockMapper()
    first = mapper.map(1000, 20_000_000_000)
    assert first == 20_000_000_000
    try:
        mapper.map(999, 20_000_000_000)
    except ValueError as exc:
        assert "non-monotonic" in str(exc)
    else:
        raise AssertionError("small device-tick regression must be rejected")


def test_copies_native_frame_into_ros_uint8_array():
    source = (ctypes.c_ubyte * 4)(1, 2, 254, 255)
    copied = copy_uint8_buffer(ctypes.addressof(source), 4)
    assert copied.typecode == "B"
    assert copied.tolist() == [1, 2, 254, 255]
