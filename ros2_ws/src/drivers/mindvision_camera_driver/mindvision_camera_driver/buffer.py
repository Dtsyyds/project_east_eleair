"""Low-overhead native frame-buffer helpers."""

import array
import ctypes


def copy_uint8_buffer(address, size):
    """Copy a native SDK buffer directly into ROS's required array('B') storage."""
    output = array.array("B", [0]) * int(size)
    ctypes.memmove(output.buffer_info()[0], address, int(size))
    return output
