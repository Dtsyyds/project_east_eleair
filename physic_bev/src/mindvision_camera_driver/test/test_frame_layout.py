from types import SimpleNamespace

import pytest

from mindvision_camera_driver.frame_layout import frame_layout, raw_bayer_gb8_layout, validate_data_length


class FakeSdk:
    CAMERA_MEDIA_TYPE_MONO8 = 1
    CAMERA_MEDIA_TYPE_BGR8 = 2
    CAMERA_MEDIA_TYPE_BAYGB8 = 3


def _head(width, height, media_type, size):
    return SimpleNamespace(iWidth=width, iHeight=height, uiMediaType=media_type, uBytes=size)


def test_gc_uses_bgr8_three_channels_and_exact_step_data():
    layout = frame_layout(FakeSdk(), _head(4096, 3072, 2, 4096 * 3072 * 3), False, 4096, 3072)
    assert layout.encoding == "bgr8"
    assert layout.channels == 3
    assert layout.step == 4096 * 3
    assert layout.data_size == 4096 * 3072 * 3
    validate_data_length(bytearray(layout.data_size), layout)


def test_gm_uses_mono8_one_channel():
    layout = frame_layout(FakeSdk(), _head(1920, 1200, 1, 1920 * 1200), True, 4096, 3072)
    assert layout.encoding == "mono8"
    assert layout.channels == 1
    assert layout.step == 1920
    assert layout.data_size == 1920 * 1200


@pytest.mark.parametrize("head", [
    _head(2, 2, 1, 12),
    _head(2, 2, 2, 4),
])
def test_media_type_and_ubytes_are_strict(head):
    with pytest.raises(ValueError):
        frame_layout(FakeSdk(), head, False, 4096, 3072)


def test_dimensions_are_bounded():
    with pytest.raises(ValueError, match="maximum"):
        frame_layout(FakeSdk(), _head(4097, 3072, 2, 4097 * 3072 * 3), False, 4096, 3072)


def test_sus1207gc_raw_bayer_gb8_layout_is_explicit_and_lossless():
    layout = raw_bayer_gb8_layout(
        FakeSdk(), _head(4096, 3072, 3, 4096 * 3072), 4096, 3072)
    assert layout.encoding == "bayer_gbrg8"
    assert layout.step == 4096
    assert layout.data_size == 4096 * 3072


def test_raw_bayer_rejects_unverified_media_type():
    with pytest.raises(ValueError, match="raw media type"):
        raw_bayer_gb8_layout(FakeSdk(), _head(4096, 3072, 2, 4096 * 3072), 4096, 3072)
