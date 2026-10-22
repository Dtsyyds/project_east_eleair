"""ROS Image conversion with explicit row-stride handling."""

import numpy as np


_ENCODINGS = {
    "mono8": (np.dtype("u1"), 1),
    "mono16": (np.dtype("u2"), 1),
    # REP-118 depth images from RealSense cameras commonly use 16UC1.  Keep
    # the integer millimetre values intact; the Rerun adapter supplies the
    # units-per-metre metadata when it constructs rr.DepthImage.
    "16uc1": (np.dtype("u2"), 1),
    "rgb8": (np.dtype("u1"), 3),
    "bgr8": (np.dtype("u1"), 3),
    "rgba8": (np.dtype("u1"), 4),
    "bgra8": (np.dtype("u1"), 4),
}


def image_to_numpy(message):
    encoding = str(message.encoding).lower()
    if encoding not in _ENCODINGS:
        raise ValueError(f"unsupported image encoding: {message.encoding!r}")
    dtype, channels = _ENCODINGS[encoding]
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    bytes_per_pixel = dtype.itemsize * channels
    used_row_bytes = width * bytes_per_pixel
    if step < used_row_bytes:
        raise ValueError(f"image step {step} is smaller than row data {used_row_bytes}")
    raw = memoryview(message.data)
    required = height * step
    if raw.nbytes < required:
        raise ValueError(f"image buffer has {raw.nbytes} bytes, expected at least {required}")
    rows = np.frombuffer(raw, dtype=np.uint8, count=required).reshape(height, step)
    packed = np.ascontiguousarray(rows[:, :used_row_bytes])
    if dtype.itemsize == 2:
        endian = ">" if bool(message.is_bigendian) else "<"
        pixels = packed.reshape(-1).view(np.dtype(endian + "u2"))
    else:
        pixels = packed.reshape(-1).view(dtype)
    shape = (height, width) if channels == 1 else (height, width, channels)
    pixels = pixels.reshape(shape)
    if dtype.itemsize == 2:
        pixels = pixels.astype(np.uint16, copy=False)
    if encoding == "bgr8":
        pixels = np.ascontiguousarray(pixels[:, :, ::-1])
        encoding = "rgb8"
    elif encoding == "bgra8":
        pixels = np.ascontiguousarray(pixels[:, :, [2, 1, 0, 3]])
        encoding = "rgba8"
    return pixels, encoding
