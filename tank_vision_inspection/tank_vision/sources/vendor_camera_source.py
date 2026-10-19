"""Real-time frame sources backed by reviewed vendor SDKs."""

from __future__ import annotations

from abc import abstractmethod
import ctypes
import importlib
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

from tank_vision.sources.base import FrameSource, SourceError
from tank_vision.metrics import FrameMetrics, StageTiming
from tank_vision.types import FrameData


class VendorCameraSource(FrameSource):
    """SDK-neutral contract; implement only after installing and reviewing the official SDK.

    Implementations must copy the vendor acquisition timestamp, exposure, and gain into
    :class:`FrameData`. No SDK symbols are guessed in this project.
    """

    source_name = "vendor_camera"

    def __init__(self, sdk_config: dict[str, Any] | None = None) -> None:
        self.sdk_config = sdk_config or {}
        if self.__class__ is VendorCameraSource:
            raise SourceError(
                "MV-SUF1205GC-T SDK adapter is unavailable. Install the official SDK and "
                "implement a subclass in tank_vision/sources/vendor_camera_source.py."
            )

    @abstractmethod
    def __next__(self) -> FrameData:
        raise StopIteration


class MindVisionCameraSource(VendorCameraSource):
    """Acquire frames with MindVision's official ``mvsdk.py`` binding.

    The SDK timestamp is reported in 0.1 ms ticks.  It is converted to the
    project's nanosecond timestamp unit without substituting host completion
    time.  The counter's epoch remains device-defined.
    """

    source_name = "mindvision"

    def __init__(self, sdk_config: dict[str, Any] | None = None) -> None:
        super().__init__(sdk_config)
        self._handle: Any | None = None
        self._buffer: Any = 0
        self._mvsdk: Any | None = None
        self._is_mono = False
        self._sequence = 0
        self._closed = False
        self._exposure_mode = "unknown"
        self.frame_id = str(self.sdk_config.get("frame_id") or "mindvision_optical_frame")
        self.timeout_ms = int(self.sdk_config.get("timeout_ms") or 1000)
        if self.timeout_ms <= 0:
            raise SourceError("MindVision timeout_ms must be positive")
        try:
            self._open()
        except SourceError:
            self.close()
            raise
        except Exception as exc:  # noqa: BLE001 - third-party SDK boundary
            self.close()
            raise SourceError(f"Could not initialize MindVision camera: {exc}") from exc

    @staticmethod
    def _library_dir(sdk_root: Path) -> Path:
        machine = os.uname().machine.lower()
        architecture = "x64" if machine in {"x86_64", "amd64"} else "arm64" if machine in {"aarch64", "arm64"} else ""
        if not architecture:
            raise SourceError(f"Unsupported MindVision SDK architecture: {machine}")
        directory = sdk_root / "lib" / architecture
        if not directory.is_dir():
            raise SourceError(f"MindVision SDK library directory is missing: {directory}")
        return directory

    @classmethod
    def _load_sdk(cls, root: Path) -> Any:
        binding_dir = root / "demo" / "python_demo"
        binding = binding_dir / "mvsdk.py"
        library = cls._library_dir(root) / "libMVSDK.so"
        if not binding.is_file() or not library.is_file():
            raise SourceError("MindVision SDK must contain demo/python_demo/mvsdk.py and lib/<arch>/libMVSDK.so")
        ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
        sys.path.insert(0, str(binding_dir))
        return importlib.import_module("mvsdk")

    def _sdk_root(self) -> Path:
        configured = self.sdk_config.get("sdk_root") or os.environ.get("MINDVISION_SDK_ROOT")
        if not configured:
            raise SourceError("Set camera.sdk_root or MINDVISION_SDK_ROOT to the official MindVision Linux SDK")
        root = Path(str(configured)).expanduser().resolve()
        if not root.is_dir():
            raise SourceError(f"MindVision SDK root does not exist: {root}")
        return root

    def _select_device(self, devices: list[Any]) -> Any:
        serial = self.sdk_config.get("serial_number")
        if serial:
            for device in devices:
                if device.GetSn() == serial:
                    return device
            known = ", ".join(device.GetSn() for device in devices)
            raise SourceError(f"MindVision serial {serial!r} was not found; available: {known}")
        if len(devices) != 1:
            raise SourceError("More than one MindVision camera is connected; set camera.serial_number")
        return devices[0]

    def _configure_resolution(self, capability: Any) -> None:
        assert self._mvsdk is not None and self._handle is not None
        requested_index = self.sdk_config.get("resolution_index")
        if requested_index is None:
            target = (int(self.sdk_config.get("width") or 0), int(self.sdk_config.get("height") or 0))
            for index in range(capability.iImageSizeDesc):
                resolution = capability.pImageSizeDesc[index]
                if (resolution.iWidth, resolution.iHeight) == target:
                    requested_index = index
                    break
        if requested_index is None:
            return
        index = int(requested_index)
        if not 0 <= index < capability.iImageSizeDesc:
            raise SourceError(f"MindVision resolution index {index} is unavailable")
        resolution = capability.pImageSizeDesc[index]
        if self._mvsdk.CameraSetImageResolution(self._handle, resolution) != self._mvsdk.CAMERA_STATUS_SUCCESS:
            raise SourceError(f"Could not select MindVision resolution index {index}")

    def _open(self) -> None:
        self._mvsdk = self._load_sdk(self._sdk_root())
        devices = list(self._mvsdk.CameraEnumerateDevice())
        if not devices:
            raise SourceError("Official MindVision SDK found no camera")
        device = self._select_device(devices)
        self._handle = self._mvsdk.CameraInit(device, -1, -1)
        capability = self._mvsdk.CameraGetCapability(self._handle)
        self._configure_resolution(capability)
        self._is_mono = bool(capability.sIspCapacity.bMonoSensor)
        output_format = self._mvsdk.CAMERA_MEDIA_TYPE_MONO8 if self._is_mono else self._mvsdk.CAMERA_MEDIA_TYPE_BGR8
        expected_pixel_format = "mono8" if self._is_mono else "bgr8"
        configured_pixel_format = str(self.sdk_config.get("pixel_format") or expected_pixel_format).lower()
        if configured_pixel_format != expected_pixel_format:
            raise SourceError(
                f"MindVision pixel_format={configured_pixel_format!r} disagrees with SDK output {expected_pixel_format!r}"
            )
        if self._mvsdk.CameraSetIspOutFormat(self._handle, output_format) != self._mvsdk.CAMERA_STATUS_SUCCESS:
            raise SourceError("Could not set MindVision output pixel format")
        if self.sdk_config.get("trigger_mode", "continuous") != "continuous":
            raise SourceError("MindVision source currently supports trigger_mode: continuous only")
        if self._mvsdk.CameraSetTriggerMode(self._handle, 0) != self._mvsdk.CAMERA_STATUS_SUCCESS:
            raise SourceError("Could not enable MindVision continuous acquisition")
        exposure_mode = self.sdk_config.get("exposure_mode")
        exposure_us = self.sdk_config.get("exposure_us")
        if exposure_mode is None:
            # Compatibility with older YAML: null means automatic exposure;
            # a numeric exposure means the old implicit manual mode.
            exposure_mode = "manual" if exposure_us is not None else "auto"
        if exposure_mode not in {"auto", "manual"}:
            raise SourceError("MindVision exposure_mode must be auto or manual")
        self._exposure_mode = str(exposure_mode)
        if exposure_mode == "auto":
            if exposure_us is not None:
                raise SourceError("MindVision auto exposure requires exposure_us=null")
            optional_ae_calls = (
                ("CameraSetAeTarget", (int(self.sdk_config["ae_target"]),), "AE target")
                if self.sdk_config.get("ae_target") is not None else None,
                ("CameraSetAeExposureRange", (float(self.sdk_config["ae_exposure_min_us"]), float(self.sdk_config["ae_exposure_max_us"])), "AE exposure range")
                if self.sdk_config.get("ae_exposure_min_us") is not None and self.sdk_config.get("ae_exposure_max_us") is not None else None,
                ("CameraSetAeAnalogGainRange", (int(self.sdk_config["gain_min_raw"]), int(self.sdk_config["gain_max_raw"])), "AE gain range")
                if self.sdk_config.get("gain_min_raw") is not None and self.sdk_config.get("gain_max_raw") is not None else None,
            )
            for call in optional_ae_calls:
                if call is None:
                    continue
                name, values, label = call
                function = getattr(self._mvsdk, name, None)
                if function is None or function(self._handle, *values) != self._mvsdk.CAMERA_STATUS_SUCCESS:
                    raise SourceError(f"Could not configure MindVision {label}")
            if self._mvsdk.CameraSetAeState(self._handle, 1) != self._mvsdk.CAMERA_STATUS_SUCCESS:
                raise SourceError("Could not enable MindVision auto exposure")
        else:
            if exposure_us is None or float(exposure_us) <= 0:
                raise SourceError("MindVision manual exposure requires a positive exposure_us")
            if self._mvsdk.CameraSetAeState(self._handle, 0) != self._mvsdk.CAMERA_STATUS_SUCCESS:
                raise SourceError("Could not disable MindVision auto exposure")
            if self._mvsdk.CameraSetExposureTime(self._handle, float(exposure_us)) != self._mvsdk.CAMERA_STATUS_SUCCESS:
                raise SourceError("Could not set MindVision exposure")
            requested_gain = self.sdk_config.get("analog_gain_raw")
            if requested_gain is not None:
                setter = getattr(self._mvsdk, "CameraSetAnalogGain", None)
                if setter is None or setter(self._handle, int(requested_gain)) != self._mvsdk.CAMERA_STATUS_SUCCESS:
                    raise SourceError("Could not set MindVision analog gain")
        if self._mvsdk.CameraPlay(self._handle) != self._mvsdk.CAMERA_STATUS_SUCCESS:
            raise SourceError("Could not start MindVision acquisition")
        channels = 1 if self._is_mono else 3
        self._buffer = self._mvsdk.CameraAlignMalloc(
            capability.sResolutionRange.iWidthMax * capability.sResolutionRange.iHeightMax * channels,
            16,
        )
        self.source_name = f"mindvision:{device.GetSn()}"

    def __next__(self) -> FrameData:
        if self._closed or self._mvsdk is None or self._handle is None or not self._buffer:
            raise SourceError("MindVision source is closed")
        started_ns = time.perf_counter_ns()
        raw_buffer: Any | None = None
        try:
            raw_buffer, frame_head = self._mvsdk.CameraGetImageBuffer(self._handle, self.timeout_ms)
            elapsed_ns = time.perf_counter_ns() - started_ns
            if self._mvsdk.CameraImageProcess(self._handle, raw_buffer, self._buffer, frame_head) != self._mvsdk.CAMERA_STATUS_SUCCESS:
                raise SourceError("MindVision CameraImageProcess failed")
            channels = 1 if self._is_mono else 3
            expected_size = int(frame_head.iWidth) * int(frame_head.iHeight) * channels
            if int(frame_head.uBytes) != expected_size:
                raise SourceError(f"Unexpected MindVision output size: {frame_head.uBytes} bytes for {frame_head.iWidth}x{frame_head.iHeight}")
            expected_media_type = self._mvsdk.CAMERA_MEDIA_TYPE_MONO8 if self._is_mono else self._mvsdk.CAMERA_MEDIA_TYPE_BGR8
            if int(frame_head.uiMediaType) != int(expected_media_type):
                raise SourceError(
                    f"Unexpected MindVision media type 0x{int(frame_head.uiMediaType):08x}; "
                    f"expected 0x{int(expected_media_type):08x}"
                )
            pixels = (ctypes.c_ubyte * expected_size).from_address(int(self._buffer))
            image = np.ctypeslib.as_array(pixels).copy()
            image = image.reshape((frame_head.iHeight, frame_head.iWidth) if self._is_mono else (frame_head.iHeight, frame_head.iWidth, 3))
            timing = StageTiming()
            timing.add_ns("source_read_ms", elapsed_ns)
            analog_gain = float(frame_head.fAnalogGain)
            gain_db = 20.0 * math.log10(analog_gain) if analog_gain > 0 else None
            frame = FrameData(
                image=image,
                timestamp_ns=int(frame_head.uiTimeStamp) * 100_000,
                frame_id=self.frame_id,
                sequence_id=self._sequence,
                source_name=self.source_name,
                exposure_us=float(frame_head.uiExpTime),
                gain_db=gain_db,
                analog_gain_raw=analog_gain,
                exposure_mode=self._exposure_mode,
                calibration_version=self.sdk_config.get("calibration_version"),
                metrics=FrameMetrics(timing=timing),
            )
            self._sequence += 1
            return frame
        except SourceError:
            raise
        except Exception as exc:  # noqa: BLE001 - third-party SDK boundary
            if (self._mvsdk is not None
                    and getattr(exc, "error_code", None) == getattr(self._mvsdk, "CAMERA_STATUS_TIME_OUT", object())):
                raise SourceError(f"MindVision acquisition timeout after {self.timeout_ms} ms") from exc
            raise SourceError(f"MindVision frame acquisition failed: {exc}") from exc
        finally:
            if raw_buffer is not None:
                active_exception = sys.exc_info()[0] is not None
                release_status = self._mvsdk.CameraReleaseImageBuffer(self._handle, raw_buffer)
                if not active_exception and release_status not in (None, self._mvsdk.CAMERA_STATUS_SUCCESS):
                    raise SourceError(f"MindVision CameraReleaseImageBuffer failed with status {release_status}")

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            if self._mvsdk is not None and self._buffer:
                self._mvsdk.CameraAlignFree(self._buffer)
                self._buffer = 0
        finally:
            if self._mvsdk is not None and self._handle is not None:
                try:
                    self._mvsdk.CameraUnInit(self._handle)
                finally:
                    self._handle = None
