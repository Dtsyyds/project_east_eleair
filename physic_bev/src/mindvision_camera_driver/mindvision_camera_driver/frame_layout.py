"""Pure-Python validation of MindVision ISP frame layouts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class FrameLayout:
    width: int
    height: int
    channels: int
    encoding: str
    step: int
    data_size: int
    media_type: int


def frame_layout(sdk: Any, frame_head: Any, is_mono: bool, max_width: int, max_height: int) -> FrameLayout:
    width = int(frame_head.iWidth)
    height = int(frame_head.iHeight)
    if width <= 0 or height <= 0 or width > int(max_width) or height > int(max_height):
        raise ValueError(f"invalid MindVision frame dimensions {width}x{height}; maximum is {max_width}x{max_height}")
    channels = 1 if is_mono else 3
    encoding = "mono8" if is_mono else "bgr8"
    expected_media_type = sdk.CAMERA_MEDIA_TYPE_MONO8 if is_mono else sdk.CAMERA_MEDIA_TYPE_BGR8
    media_type = int(frame_head.uiMediaType)
    if media_type != int(expected_media_type):
        raise ValueError(
            f"unexpected MindVision media type 0x{media_type:08x}; "
            f"expected 0x{int(expected_media_type):08x} for {encoding}"
        )
    step = width * channels
    data_size = step * height
    if int(frame_head.uBytes) != data_size:
        raise ValueError(
            f"unexpected MindVision uBytes={frame_head.uBytes}; expected {data_size} "
            f"for {width}x{height} {encoding}"
        )
    if step != width * channels or data_size != height * step:
        raise ValueError("MindVision frame step/data-size invariant failed")
    return FrameLayout(width, height, channels, encoding, step, data_size, media_type)


def validate_data_length(data: Any, layout: FrameLayout) -> None:
    if len(data) != layout.data_size:
        raise ValueError(f"image data length {len(data)} does not match expected {layout.data_size}")


def raw_bayer_gb8_layout(sdk: Any, frame_head: Any, max_width: int, max_height: int) -> FrameLayout:
    """Validate the unprocessed SUS1207GC sensor buffer as Bayer GB 8-bit."""
    width = int(frame_head.iWidth)
    height = int(frame_head.iHeight)
    if width <= 0 or height <= 0 or width > int(max_width) or height > int(max_height):
        raise ValueError(
            f"invalid MindVision raw frame dimensions {width}x{height}; "
            f"maximum is {max_width}x{max_height}"
        )
    media_type = int(frame_head.uiMediaType)
    expected = int(sdk.CAMERA_MEDIA_TYPE_BAYGB8)
    if media_type != expected:
        raise ValueError(
            f"unexpected MindVision raw media type 0x{media_type:08x}; "
            f"expected 0x{expected:08x} for Bayer GB8"
        )
    data_size = width * height
    if int(frame_head.uBytes) != data_size:
        raise ValueError(
            f"unexpected MindVision raw uBytes={frame_head.uBytes}; expected {data_size} "
            f"for {width}x{height} Bayer GB8"
        )
    return FrameLayout(width, height, 1, "bayer_gbrg8", width, data_size, media_type)
