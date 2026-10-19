from __future__ import annotations

import ctypes
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "test_mindvision_camera.py"
SPEC = importlib.util.spec_from_file_location("test_mindvision_camera_script", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
camera_test = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(camera_test)


class _Timeout(Exception):
    def __init__(self) -> None:
        self.error_code = -12


class _SdkError(Exception):
    def __init__(self) -> None:
        self.error_code = -34


class _Device:
    def GetSn(self) -> str:
        return "046081820650"

    def GetFriendlyName(self) -> str:
        return "MV-SUF1205GC"

    def GetPortType(self) -> str:
        return "USB3.0"


class _Resolution:
    iWidth = 2
    iHeight = 2

    def GetDescription(self) -> str:
        return "2x2"


class _FakeSdk:
    CAMERA_STATUS_SUCCESS = 0
    CAMERA_STATUS_TIME_OUT = -12
    CAMERA_MEDIA_TYPE_MONO8 = 1
    CAMERA_MEDIA_TYPE_BGR8 = 2

    def __init__(self, mono: bool = False, failures: list[str] | None = None) -> None:
        self.device = _Device()
        self.mono = mono
        self.failures = list(failures or [])
        self.frame_number = 0
        self.ae_state: int | None = None
        self.exposure_us = None
        self.set_format = None
        self.release_count = 0
        self.align_free_count = 0
        self.uninit_count = 0
        self.buffer_object = None
        self.buffer_address = 0
        self.ae_exposure_range = (0.0, 10_000_000.0)
        self.ae_gain_range = (2, 16)
        self.call_order: list[str] = []

    def CameraEnumerateDevice(self):
        return [self.device]

    def CameraInit(self, *_args):
        return 7

    def CameraGetCapability(self, _handle):
        return SimpleNamespace(
            sIspCapacity=SimpleNamespace(bMonoSensor=int(self.mono)),
            sResolutionRange=SimpleNamespace(iWidthMax=2, iHeightMax=2),
            iImageSizeDesc=1,
            pImageSizeDesc=[_Resolution()],
        )

    def CameraSetImageResolution(self, *_args):
        return self.CAMERA_STATUS_SUCCESS

    def CameraSetIspOutFormat(self, _handle, value):
        self.set_format = value
        return self.CAMERA_STATUS_SUCCESS

    def CameraSetTriggerMode(self, *_args):
        return self.CAMERA_STATUS_SUCCESS

    def CameraSetAeState(self, _handle, value):
        self.call_order.append("CameraSetAeState")
        self.ae_state = value
        return self.CAMERA_STATUS_SUCCESS

    def CameraGetAeState(self, _handle):
        assert self.ae_state is not None
        return self.ae_state

    def CameraSetExposureTime(self, _handle, value):
        self.exposure_us = value
        return self.CAMERA_STATUS_SUCCESS

    def CameraGetAeExposureRange(self, _handle):
        self.call_order.append("CameraGetAeExposureRange")
        return self.ae_exposure_range

    def CameraSetAeExposureRange(self, _handle, minimum, maximum):
        self.call_order.append("CameraSetAeExposureRange")
        self.ae_exposure_range = (float(minimum), float(maximum))
        return self.CAMERA_STATUS_SUCCESS

    def CameraGetAeAnalogGainRange(self, _handle):
        self.call_order.append("CameraGetAeAnalogGainRange")
        return self.ae_gain_range

    def CameraSetAeAnalogGainRange(self, _handle, minimum, maximum):
        self.call_order.append("CameraSetAeAnalogGainRange")
        self.ae_gain_range = (int(minimum), int(maximum))
        return self.CAMERA_STATUS_SUCCESS

    def CameraPlay(self, *_args):
        self.call_order.append("CameraPlay")
        return self.CAMERA_STATUS_SUCCESS

    def CameraGetAnalogGain(self, _handle):
        return 10 if self.ae_state == 1 else 2

    def CameraAlignMalloc(self, size, _alignment):
        self.buffer_object = ctypes.create_string_buffer(size)
        self.buffer_address = ctypes.addressof(self.buffer_object)
        return self.buffer_address

    def CameraGetImageBuffer(self, _handle, _timeout_ms):
        if self.failures:
            failure = self.failures.pop(0)
            if failure == "timeout":
                raise _Timeout()
            if failure == "sdk":
                raise _SdkError()
            if failure == "keyboard":
                raise KeyboardInterrupt()
        self.frame_number += 1
        channels = 1 if self.mono else 3
        media_type = self.CAMERA_MEDIA_TYPE_MONO8 if self.mono else self.CAMERA_MEDIA_TYPE_BGR8
        value = self.frame_number * 10
        return object(), SimpleNamespace(
            iWidth=2,
            iHeight=2,
            uBytes=4 * channels,
            uiMediaType=media_type,
            uiTimeStamp=1000 + self.frame_number * 100,
            uiExpTime=30000 if self.ae_state == 0 else 100000,
            fAnalogGain=1.0 if self.ae_state == 0 else 8.0,
            _value=value,
        )

    def CameraImageProcess(self, _handle, _raw, output, header):
        channels = 1 if self.mono else 3
        ctypes.memset(output, header._value, 4 * channels)
        return self.CAMERA_STATUS_SUCCESS

    def CameraReleaseImageBuffer(self, _handle, _raw):
        self.release_count += 1
        return self.CAMERA_STATUS_SUCCESS

    def CameraAlignFree(self, _buffer):
        self.align_free_count += 1
        ctypes.memset(self.buffer_address, 0, 4 * (1 if self.mono else 3))

    def CameraUnInit(self, _handle):
        self.uninit_count += 1


def _config(tmp_path: Path, exposure_mode: str = "auto", exposure_us: str = "null", color: str = "true", pixel_format: str = "bgr8") -> Path:
    path = tmp_path / "camera.yaml"
    path.write_text(
        "camera:\n"
        "  model: MV-SUF1205GC-T\n"
        f"  color: {color}\n"
        f"  pixel_format: {pixel_format}\n"
        f"  exposure_mode: {exposure_mode}\n"
        f"  exposure_us: {exposure_us}\n"
        "  trigger_mode: continuous\n",
        encoding="utf-8",
    )
    return path


def _args(config: Path, tmp_path: Path, *extra: str):
    values = ["--config", str(config), "--sdk-root", str(tmp_path), *extra]
    args = camera_test.build_parser().parse_args(values)
    if args.warmup_frames is None:
        args.warmup_frames = 0
    return args


def test_auto_and_manual_exposure_are_explicit(tmp_path: Path) -> None:
    auto_sdk = _FakeSdk()
    auto_result = camera_test.capture(_args(_config(tmp_path), tmp_path, "--frames", "2"), lambda _root: auto_sdk)
    assert auto_sdk.ae_state == 1
    assert auto_result["requested_exposure_mode"] == "auto"
    assert auto_result["reported_exposure_mode"] == "auto"

    manual_sdk = _FakeSdk()
    manual_config = _config(tmp_path, exposure_mode="manual", exposure_us="null")
    manual_result = camera_test.capture(
        _args(manual_config, tmp_path, "--exposure-mode", "manual", "--exposure-us", "30000", "--frames", "2"),
        lambda _root: manual_sdk,
    )
    assert manual_sdk.ae_state == 0
    assert manual_sdk.exposure_us == 30000
    assert manual_result["requested_exposure_mode"] == "manual"
    assert manual_result["reported_exposure_us_first_last"] == [30000, 30000]


def test_bounded_auto_exposure_validates_sets_in_order_and_reports_raw_gain(tmp_path: Path) -> None:
    sdk = _FakeSdk()
    result = camera_test.capture(
        _args(
            _config(tmp_path),
            tmp_path,
            "--exposure-mode", "auto",
            "--ae-min-exposure-us", "100",
            "--ae-max-exposure-us", "50000",
            "--ae-min-analog-gain-raw", "2",
            "--ae-max-analog-gain-raw", "16",
            "--frames", "2",
        ),
        lambda _root: sdk,
    )
    assert sdk.call_order.index("CameraSetAeExposureRange") < sdk.call_order.index("CameraSetAeAnalogGainRange")
    assert sdk.call_order.index("CameraSetAeAnalogGainRange") < sdk.call_order.index("CameraSetAeState")
    assert sdk.call_order.index("CameraSetAeState") < sdk.call_order.index("CameraPlay")
    assert result["requested_ae_exposure_range_us"] == [100.0, 50000.0]
    assert result["requested_ae_analog_gain_range_raw"] == [2, 16]
    assert result["sdk_ae_exposure_range_us"] == [100.0, 50000.0]
    assert result["sdk_ae_analog_gain_range_raw"] == [2, 16]
    assert result["final_analog_gain_raw"] == 10
    assert result["reported_analog_gain_raw_first_last"] == [10, 10]


def test_ae_range_options_are_rejected_for_manual_mode(tmp_path: Path) -> None:
    sdk = _FakeSdk()
    with pytest.raises(ValueError, match="only be used"):
        camera_test.capture(
            _args(
                _config(tmp_path, exposure_mode="manual"),
                tmp_path,
                "--exposure-mode", "manual",
                "--exposure-us", "30000",
                "--ae-max-exposure-us", "50000",
                "--frames", "1",
            ),
            lambda _root: sdk,
        )
    assert sdk.call_order.count("CameraSetAeExposureRange") == 0


def test_warmup_last_frame_copy_and_statistics_are_json_safe(tmp_path: Path) -> None:
    sdk = _FakeSdk()
    image_path = tmp_path / "last.bmp"
    result = camera_test.capture(
        _args(_config(tmp_path), tmp_path, "--warmup-frames", "2", "--frames", "3", "--save-image", str(image_path)),
        lambda _root: sdk,
    )
    saved = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    assert result["warmup_frames"] == 2
    assert result["captured_frames"] == 3
    assert result["saved_frame_policy"] == "last"
    assert int(saved[0, 0, 0]) == 50
    assert sdk.release_count == 5
    assert sdk.align_free_count == 1 and sdk.uninit_count == 1
    json.dumps(result)
    assert set((result["saved_frame_statistics"] or {})) >= {
        "bgr_mean", "gray_mean", "gray_std", "gray_percentiles_p1_p5_p50_p95_p99",
        "underexposed_ratio", "saturated_ratio", "laplacian_variance",
    }


def test_mono8_reshape_and_timeout_continue(tmp_path: Path) -> None:
    sdk = _FakeSdk(mono=True, failures=["timeout"])
    result = camera_test.capture(
        _args(_config(tmp_path, color="false", pixel_format="mono8"), tmp_path, "--frames", "2", "--save-image", str(tmp_path / "mono.bmp")),
        lambda _root: sdk,
    )
    assert sdk.set_format == sdk.CAMERA_MEDIA_TYPE_MONO8
    assert result["timeout_count"] == 1
    assert result["captured_frames"] == 2
    assert (result["saved_frame_statistics"] or {})["channels"] == 1


def test_non_timeout_sdk_error_and_keyboard_interrupt_cleanup(tmp_path: Path) -> None:
    sdk = _FakeSdk(failures=["sdk"])
    with pytest.raises(_SdkError):
        camera_test.capture(_args(_config(tmp_path), tmp_path, "--frames", "1"), lambda _root: sdk)
    assert sdk.release_count == 0
    assert sdk.align_free_count == 1 and sdk.uninit_count == 1

    interrupted = _FakeSdk(failures=["keyboard"])
    with pytest.raises(KeyboardInterrupt):
        camera_test.capture(_args(_config(tmp_path), tmp_path, "--frames", "1"), lambda _root: interrupted)
    assert interrupted.release_count == 0
    assert interrupted.align_free_count == 1 and interrupted.uninit_count == 1


def test_no_preview_does_not_call_gui(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    called = []
    monkeypatch.setattr(camera_test.cv2, "namedWindow", lambda *_args: called.append("window"))
    monkeypatch.setattr(camera_test.cv2, "waitKey", lambda *_args: called.append("wait"))
    camera_test.capture(_args(_config(tmp_path), tmp_path, "--frames", "1"), lambda _root: _FakeSdk())
    assert called == []
