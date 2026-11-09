import ctypes
from types import SimpleNamespace

import pytest

from mindvision_camera_driver.node import MindVisionCameraNode
from mindvision_camera_driver.qos_settings import validate_qos


def test_qos_is_bounded_and_best_effort_by_profile():
    assert validate_qos(1, "best_effort") == (1, "best_effort")
    assert validate_qos(2, "reliable") == (2, "reliable")
    with pytest.raises(ValueError):
        validate_qos(10, "reliable")


class _Timeout(Exception):
    error_code = -12


class _CaptureSdk:
    CAMERA_STATUS_SUCCESS = 0
    CAMERA_STATUS_TIME_OUT = -12
    CAMERA_MEDIA_TYPE_BGR8 = 2
    CAMERA_MEDIA_TYPE_MONO8 = 1

    def __init__(self):
        self.calls = []
        self.frames = ["timeout", "frame"]
        self.released = 0
        self.buffer = ctypes.create_string_buffer(12)

    def CameraGetImageBuffer(self, _handle, _timeout):
        action = self.frames.pop(0)
        if action == "timeout":
            raise _Timeout()
        return object(), SimpleNamespace(iWidth=2, iHeight=2, uiMediaType=2, uBytes=12)

    def CameraImageProcess(self, _handle, _raw, output, _head):
        ctypes.memmove(output, bytes(range(12)), 12)
        return self.CAMERA_STATUS_SUCCESS

    def CameraReleaseImageBuffer(self, _handle, _raw):
        self.released += 1
        return self.CAMERA_STATUS_SUCCESS


def test_timeout_has_no_release_and_success_has_one_release():
    sdk = _CaptureSdk()
    node = object.__new__(MindVisionCameraNode)
    node._sdk = sdk
    node._handle = 1
    node._buffer = ctypes.addressof(sdk.buffer)
    node._is_mono = False
    node._max_width = 2
    node._max_height = 2
    node._timeout_count = 0
    node._capture_error_count = 0
    node._value = lambda name: 1000 if name == "timeout_ms" else None
    assert node._acquire_processed() is None
    assert sdk.released == 0 and node._timeout_count == 1
    acquired = node._acquire_processed()
    assert acquired is not None
    assert len(acquired[2]) == 12
    assert sdk.released == 1


def test_close_pauses_then_frees_then_uninitializes_once():
    class Sdk:
        CAMERA_STATUS_SUCCESS = 0

        def __init__(self):
            self.calls = []

        def CameraPause(self, _handle):
            self.calls.append("pause")
            return 0

        def CameraAlignFree(self, _buffer):
            self.calls.append("free")

        def CameraUnInit(self, _handle):
            self.calls.append("uninit")
            return 0

    sdk = Sdk()
    node = object.__new__(MindVisionCameraNode)
    node._sdk = sdk
    node._handle = 1
    node._buffer = 2
    node._playing = True
    node._closed = False
    node._timer = None
    node.close()
    node.close()
    assert sdk.calls == ["pause", "free", "uninit"]


def test_close_cancels_timer_before_releasing_camera():
    class Timer:
        def __init__(self):
            self.cancelled = False

        def cancel(self):
            self.cancelled = True

    class Sdk:
        CAMERA_STATUS_SUCCESS = 0

        def CameraPause(self, _handle):
            assert timer.cancelled
            return 0

        def CameraAlignFree(self, _buffer):
            pass

        def CameraUnInit(self, _handle):
            return 0

    timer = Timer()
    node = object.__new__(MindVisionCameraNode)
    node._sdk = Sdk()
    node._handle = 1
    node._buffer = 2
    node._playing = True
    node._closed = False
    node._timer = timer
    node.close()
    assert timer.cancelled
