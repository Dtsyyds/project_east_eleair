from __future__ import annotations

import os
from pathlib import Path
import ctypes
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from tank_vision.sources import FolderSource, MindVisionCameraSource, SourceError
from tank_vision import cli
from tank_vision.config import load_config
from tank_vision.synthetic import seam_image


def test_folder_source_sorts_skips_corrupt_and_preserves_mtime(tmp_path: Path) -> None:
    corrupt = tmp_path / "00_corrupt.jpg"
    corrupt.write_bytes(b"not an image")
    valid = tmp_path / "01_valid.png"
    assert cv2.imwrite(str(valid), seam_image(160, 120))
    expected_timestamp = 1_700_000_000_123_456_789
    os.utime(valid, ns=(expected_timestamp, expected_timestamp))
    source = FolderSource(tmp_path)
    frames = list(source)
    assert len(frames) == 1
    assert frames[0].timestamp_ns == expected_timestamp
    assert frames[0].sequence_id == 0
    assert frames[0].original_filename == valid.name
    assert source.errors and "Unreadable" in source.errors[0]


def test_empty_folder_is_valid_source(tmp_path: Path) -> None:
    assert list(FolderSource(tmp_path)) == []


class _FakeMindVisionDevice:
    def GetSn(self) -> str:
        return "MV-123"


class _FakeResolution:
    def __init__(self, width: int, height: int) -> None:
        self.iWidth = width
        self.iHeight = height


class _FakeMindVisionSdk:
    CAMERA_STATUS_SUCCESS = 0
    CAMERA_STATUS_TIME_OUT = -12
    CAMERA_MEDIA_TYPE_MONO8 = 1
    CAMERA_MEDIA_TYPE_BGR8 = 2

    def __init__(self) -> None:
        self.device = _FakeMindVisionDevice()
        self.resolutions = [_FakeResolution(4, 3), _FakeResolution(2, 2)]
        self.buffer = None
        self.released = 0
        self.freed = False
        self.uninitialized = False
        self.selected_resolution = None
        self.ae_state = None
        self.exposure_us = None

    def CameraEnumerateDevice(self):
        return [self.device]

    def CameraInit(self, *_args):
        return 7

    def CameraGetCapability(self, _handle):
        capability = type("Capability", (), {})()
        capability.sIspCapacity = type("Isp", (), {"bMonoSensor": 1})()
        capability.sResolutionRange = type("Range", (), {"iWidthMax": 4, "iHeightMax": 3})()
        capability.iImageSizeDesc = len(self.resolutions)
        capability.pImageSizeDesc = self.resolutions
        return capability

    def CameraSetImageResolution(self, _handle, resolution):
        self.selected_resolution = resolution
        return self.CAMERA_STATUS_SUCCESS

    def CameraSetIspOutFormat(self, *_args):
        return self.CAMERA_STATUS_SUCCESS

    def CameraSetTriggerMode(self, *_args):
        return self.CAMERA_STATUS_SUCCESS

    def CameraSetAeState(self, _handle, state):
        self.ae_state = state
        return self.CAMERA_STATUS_SUCCESS

    def CameraSetExposureTime(self, _handle, exposure_us):
        self.exposure_us = exposure_us
        return self.CAMERA_STATUS_SUCCESS

    def CameraPlay(self, *_args):
        return self.CAMERA_STATUS_SUCCESS

    def CameraAlignMalloc(self, size, _alignment):
        self.buffer = ctypes.create_string_buffer(size)
        return ctypes.addressof(self.buffer)

    def CameraGetImageBuffer(self, _handle, _timeout_ms):
        header = type("Header", (), {})()
        header.iWidth = 2
        header.iHeight = 2
        header.uBytes = 4
        header.uiMediaType = self.CAMERA_MEDIA_TYPE_MONO8
        header.uiTimeStamp = 42
        header.uiExpTime = 12_345
        header.fAnalogGain = 4.0
        return object(), header

    def CameraImageProcess(self, _handle, _raw, buffer, _header):
        ctypes.memmove(buffer, bytes((1, 2, 3, 4)), 4)
        return self.CAMERA_STATUS_SUCCESS

    def CameraReleaseImageBuffer(self, *_args):
        self.released += 1

    def CameraAlignFree(self, _buffer):
        self.freed = True

    def CameraUnInit(self, _handle):
        self.uninitialized = True


def test_mindvision_source_preserves_hardware_metadata(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    sdk = _FakeMindVisionSdk()
    monkeypatch.setattr(MindVisionCameraSource, "_load_sdk", classmethod(lambda _cls, _root: sdk))
    source = MindVisionCameraSource(
        {"sdk_root": str(tmp_path), "width": 2, "height": 2, "timeout_ms": 50, "frame_id": "mv_frame"}
    )
    frame = next(source)
    source.close()
    assert sdk.selected_resolution is sdk.resolutions[1]
    assert sdk.ae_state == 1
    assert sdk.released == 1 and sdk.freed and sdk.uninitialized
    assert frame.timestamp_ns == 4_200_000
    assert frame.exposure_us == 12_345.0
    assert frame.gain_db == pytest.approx(12.0412, abs=0.001)
    assert frame.frame_id == "mv_frame"
    assert frame.source_name == "mindvision:MV-123"
    assert frame.image.ndim == 2
    assert np.array_equal(frame.image, np.array([[1, 2], [3, 4]], dtype=np.uint8))


def test_mindvision_source_rejects_ambiguous_devices(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    sdk = _FakeMindVisionSdk()
    sdk.CameraEnumerateDevice = lambda: [sdk.device, _FakeMindVisionDevice()]
    monkeypatch.setattr(MindVisionCameraSource, "_load_sdk", classmethod(lambda _cls, _root: sdk))
    with pytest.raises(SourceError, match="More than one MindVision camera"):
        MindVisionCameraSource({"sdk_root": str(tmp_path)})


def test_mindvision_timeout_is_clear_and_close_still_releases_resources(
        monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    class TimeoutErrorFromSdk(Exception):
        error_code = -12

    sdk = _FakeMindVisionSdk()
    sdk.CameraGetImageBuffer = lambda *_args: (_ for _ in ()).throw(TimeoutErrorFromSdk())
    monkeypatch.setattr(MindVisionCameraSource, "_load_sdk", classmethod(lambda _cls, _root: sdk))
    source = MindVisionCameraSource({"sdk_root": str(tmp_path), "width": 2, "height": 2, "timeout_ms": 50})
    with pytest.raises(SourceError, match="timeout after 50 ms"):
        next(source)
    source.close()
    assert sdk.released == 0 and sdk.freed and sdk.uninitialized


def test_cli_passes_camera_configuration_to_mindvision_source(monkeypatch: pytest.MonkeyPatch) -> None:
    captured = {}

    def fake_source(options):
        captured.update(options)
        return object()

    monkeypatch.setattr(cli, "MindVisionCameraSource", fake_source)
    config = load_config(Path(__file__).parents[1] / "configs" / "camera_mv_suf1205gm_usb2.yaml")
    source = cli._make_source(
        SimpleNamespace(source="mindvision", mindvision_sdk_root="/sdk/from/cli", input=None, camera_device="0"), config
    )
    assert source is not None
    assert captured["sdk_root"] == "/sdk/from/cli"
    assert captured["resolution_index"] == 3
    assert (captured["width"], captured["height"]) == (1920, 1200)
