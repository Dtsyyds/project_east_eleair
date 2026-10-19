import numpy as np

from inspection_rerun.image_adapter import image_to_numpy


class Image:
    pass


def make_image(width, height, step, encoding, data, bigendian=0):
    msg = Image()
    msg.width, msg.height, msg.step = width, height, step
    msg.encoding, msg.data, msg.is_bigendian = encoding, data, bigendian
    return msg


def test_mono8_image():
    pixels, encoding = image_to_numpy(make_image(3, 2, 3, "mono8", bytes(range(6))))
    assert encoding == "mono8"
    np.testing.assert_array_equal(pixels, [[0, 1, 2], [3, 4, 5]])


def test_mono8_ignores_row_padding():
    data = bytes([1, 2, 3, 99, 99, 4, 5, 6, 88, 88])
    pixels, _ = image_to_numpy(make_image(3, 2, 5, "mono8", data))
    np.testing.assert_array_equal(pixels, [[1, 2, 3], [4, 5, 6]])


def test_bgr_is_converted_to_rgb():
    pixels, encoding = image_to_numpy(make_image(1, 1, 3, "bgr8", bytes([1, 2, 3])))
    assert encoding == "rgb8"
    np.testing.assert_array_equal(pixels, [[[3, 2, 1]]])


def test_big_endian_mono16_becomes_native_values():
    pixels, encoding = image_to_numpy(make_image(2, 1, 4, "mono16", bytes([0x01, 0x02, 0xA0, 0xB0]), 1))
    assert encoding == "mono16"
    assert pixels.dtype == np.uint16
    np.testing.assert_array_equal(pixels, [[0x0102, 0xA0B0]])


def test_d405_16uc1_depth_preserves_millimetres():
    pixels, encoding = image_to_numpy(
        make_image(2, 1, 4, "16UC1", bytes([0xE8, 0x03, 0xD0, 0x07])))
    assert encoding == "16uc1"
    assert pixels.dtype == np.uint16
    np.testing.assert_array_equal(pixels, [[1000, 2000]])
