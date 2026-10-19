#!/usr/bin/env python3
"""Bounded MindVision acquisition test with explicit exposure and save policy.

The script uses only the vendor supplied ``mvsdk.py`` binding. It keeps the
capture path usable without a display and pairs every successful
``CameraGetImageBuffer`` with exactly one release in the same ``finally``.
"""

from __future__ import annotations

import argparse
import ctypes
import importlib
import json
import os
from pathlib import Path
import statistics
import sys
import time
from typing import Any, Callable

import cv2
import numpy as np
import yaml


def _architecture_library_dir(root: Path) -> Path:
    machine = os.uname().machine.lower()
    name = "x64" if machine in {"x86_64", "amd64"} else "arm64" if machine in {"aarch64", "arm64"} else ""
    if not name:
        raise RuntimeError(f"Unsupported host architecture for MindVision smoke test: {machine}")
    directory = root / "lib" / name
    if not directory.is_dir():
        raise RuntimeError(f"SDK does not contain a {name} library directory: {directory}")
    return directory


def _load_official_binding(sdk_root: Path) -> Any:
    """Load the vendor Python binding from a verified SDK directory."""
    binding_dir = sdk_root / "demo" / "python_demo"
    if not (binding_dir / "mvsdk.py").is_file():
        raise RuntimeError(f"Official mvsdk.py binding not found under: {binding_dir}")
    library = _architecture_library_dir(sdk_root) / "libMVSDK.so"
    if not library.is_file():
        raise RuntimeError(f"Official SDK library not found: {library}")
    ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(binding_dir))
    return importlib.import_module("mvsdk")


def _select_device(mvsdk: Any, serial_number: str | None) -> Any:
    devices = list(mvsdk.CameraEnumerateDevice())
    if not devices:
        raise RuntimeError("Official SDK found no MindVision camera")
    if serial_number:
        for device in devices:
            if device.GetSn() == serial_number:
                return device
        known = ", ".join(device.GetSn() for device in devices)
        raise RuntimeError(f"Configured serial number {serial_number!r} was not found; available: {known}")
    if len(devices) != 1:
        raise RuntimeError("More than one camera found; set camera.serial_number in the YAML configuration")
    return devices[0]


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, int(len(ordered) * percentile) - 1))
    return float(ordered[index])


def _image_statistics(image: np.ndarray) -> dict[str, object]:
    if image.dtype != np.uint8:
        raise RuntimeError(f"Expected uint8 image for statistics, got {image.dtype}")
    if image.ndim == 2:
        gray = image
        channels = 1
        bgr_mean = [float(image.mean())]
    elif image.ndim == 3 and image.shape[2] == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        channels = 3
        bgr_mean = [float(value) for value in image.reshape(-1, 3).mean(axis=0)]
    else:
        raise RuntimeError(f"Expected Mono8 or BGR8 image, got shape {image.shape}")
    gray_values = gray.astype(np.float64, copy=False)
    percentiles = np.percentile(gray_values, [1, 5, 50, 95, 99])
    return {
        "shape": [int(value) for value in image.shape],
        "channels": channels,
        "dtype": str(image.dtype),
        "bgr_mean": [round(value, 4) for value in bgr_mean],
        "gray_mean": round(float(gray_values.mean()), 4),
        "gray_std": round(float(gray_values.std()), 4),
        "gray_percentiles_p1_p5_p50_p95_p99": [round(float(value), 4) for value in percentiles],
        "underexposed_ratio": round(float(np.mean(gray_values <= 12.0)), 6),
        "saturated_ratio": round(float(np.mean(gray_values >= 255.0)), 6),
        "laplacian_variance": round(float(cv2.Laplacian(gray, cv2.CV_64F).var()), 4),
    }


def _resolve_exposure(args: argparse.Namespace, camera_config: dict[str, object]) -> tuple[str, int | None]:
    config_mode = camera_config.get("exposure_mode")
    config_exposure = camera_config.get("exposure_us")
    if args.exposure_mode == "auto" and args.exposure_us is not None:
        raise ValueError("--exposure-mode auto cannot be combined with --exposure-us")
    if config_mode is not None and config_mode not in {"auto", "manual"}:
        raise ValueError("camera.exposure_mode must be 'auto' or 'manual'")
    if config_mode == "auto" and config_exposure is not None and args.exposure_mode not in {"auto", "manual"} and args.exposure_us is None:
        raise ValueError("camera.exposure_mode=auto requires camera.exposure_us=null")
    mode = args.exposure_mode or ("manual" if args.exposure_us is not None else config_mode)
    if mode is None:
        # Backwards compatibility: old YAML with exposure_us: null means AE.
        mode = "manual" if config_exposure is not None else "auto"
    exposure = None if args.exposure_mode == "auto" else (args.exposure_us if args.exposure_us is not None else config_exposure)
    if mode == "manual":
        if exposure is None:
            raise ValueError("manual exposure mode requires --exposure-us or camera.exposure_us")
        if isinstance(exposure, bool) or float(exposure) <= 0 or float(exposure) != int(float(exposure)):
            raise ValueError("manual exposure must be a positive integer number of microseconds")
        return mode, int(exposure)
    if exposure is not None:
        raise ValueError("auto exposure mode cannot specify exposure_us")
    return mode, None


def _read_ae_ranges(mvsdk: Any, handle: Any) -> tuple[tuple[float, float], tuple[int, int]]:
    if not hasattr(mvsdk, "CameraGetAeExposureRange") or not hasattr(mvsdk, "CameraGetAeAnalogGainRange"):
        raise RuntimeError("SDK binding does not provide the required auto-exposure range APIs")
    exposure_min, exposure_max = mvsdk.CameraGetAeExposureRange(handle)
    gain_min, gain_max = mvsdk.CameraGetAeAnalogGainRange(handle)
    return (float(exposure_min), float(exposure_max)), (int(gain_min), int(gain_max))


def _requested_ae_ranges(args: argparse.Namespace, mode: str) -> tuple[tuple[float, float] | None, tuple[int, int] | None]:
    exposure_values = (args.ae_min_exposure_us, args.ae_max_exposure_us)
    gain_values = (args.ae_min_analog_gain_raw, args.ae_max_analog_gain_raw)
    if mode != "auto" and any(value is not None for value in (*exposure_values, *gain_values)):
        raise ValueError("AE range options can only be used with --exposure-mode auto")
    if any(value is None for value in exposure_values) and any(value is not None for value in exposure_values):
        raise ValueError("--ae-min-exposure-us and --ae-max-exposure-us must be supplied together")
    if any(value is None for value in gain_values) and any(value is not None for value in gain_values):
        raise ValueError("--ae-min-analog-gain-raw and --ae-max-analog-gain-raw must be supplied together")
    exposure_range = None if exposure_values[0] is None else (float(exposure_values[0]), float(exposure_values[1]))
    gain_range = None if gain_values[0] is None else (int(gain_values[0]), int(gain_values[1]))
    return exposure_range, gain_range


def _configure_auto_exposure_ranges(
    mvsdk: Any,
    handle: Any,
    requested_exposure_range: tuple[float, float] | None,
    requested_gain_range: tuple[int, int] | None,
) -> tuple[tuple[float, float], tuple[int, int]]:
    sdk_exposure_range, sdk_gain_range = _read_ae_ranges(mvsdk, handle)
    if requested_exposure_range is not None:
        minimum, maximum = requested_exposure_range
        if minimum <= 0 or minimum > maximum or minimum < sdk_exposure_range[0] or maximum > sdk_exposure_range[1]:
            raise ValueError(
                f"requested AE exposure range [{minimum}, {maximum}] us is outside SDK range {sdk_exposure_range}"
            )
    if requested_gain_range is not None:
        minimum, maximum = requested_gain_range
        if minimum > maximum or minimum < sdk_gain_range[0] or maximum > sdk_gain_range[1]:
            raise ValueError(
                f"requested AE analog gain raw range [{minimum}, {maximum}] is outside SDK range {sdk_gain_range}"
            )
    if requested_exposure_range is not None:
        if mvsdk.CameraSetAeExposureRange(handle, *requested_exposure_range) != mvsdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError("Could not set auto-exposure time range")
    if requested_gain_range is not None:
        if mvsdk.CameraSetAeAnalogGainRange(handle, *requested_gain_range) != mvsdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError("Could not set auto analog-gain raw range")
    if requested_exposure_range is not None or requested_gain_range is not None:
        return _read_ae_ranges(mvsdk, handle)
    return sdk_exposure_range, sdk_gain_range


def _is_timeout(mvsdk: Any, exc: BaseException) -> bool:
    return getattr(exc, "error_code", None) == getattr(mvsdk, "CAMERA_STATUS_TIME_OUT", object())


def _frame_from_processed_buffer(mvsdk: Any, output_buffer: int, frame_head: Any, is_mono: bool) -> np.ndarray:
    width = int(frame_head.iWidth)
    height = int(frame_head.iHeight)
    channels = 1 if is_mono else 3
    expected_size = width * height * channels
    if width <= 0 or height <= 0:
        raise RuntimeError(f"Invalid frame dimensions {width}x{height}")
    if int(frame_head.uBytes) != expected_size:
        raise RuntimeError(
            f"Unexpected SDK output size: uBytes={frame_head.uBytes}, expected={expected_size} "
            f"for {width}x{height}x{channels}"
        )
    expected_media_type = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if is_mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    if int(frame_head.uiMediaType) != int(expected_media_type):
        raise RuntimeError(
            f"Unexpected SDK media type 0x{int(frame_head.uiMediaType):08x}; "
            f"expected 0x{int(expected_media_type):08x}"
        )
    pixels = (ctypes.c_ubyte * expected_size).from_address(int(output_buffer))
    image = np.ctypeslib.as_array(pixels).copy()
    return image.reshape((height, width) if is_mono else (height, width, 3))


def _saved_file_statistics(path: Path, expected_shape: tuple[int, ...], expected_channels: int) -> dict[str, object]:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"Saved image cannot be read by OpenCV: {path}")
    if image.dtype != np.uint8:
        raise RuntimeError(f"Saved image dtype is {image.dtype}, expected uint8")
    actual_channels = 1 if image.ndim == 2 else int(image.shape[2])
    if tuple(image.shape) != expected_shape or actual_channels != expected_channels:
        raise RuntimeError(
            f"Saved image shape/channels {image.shape}/{actual_channels} do not match "
            f"{expected_shape}/{expected_channels}"
        )
    stats = _image_statistics(image)
    stats["path"] = str(path.resolve())
    return stats


def _configure_camera(
    mvsdk: Any,
    handle: Any,
    capability: Any,
    args: argparse.Namespace,
    camera_config: dict[str, object],
) -> tuple[bool, str, int | None, dict[str, object] | None, tuple[float, float] | None, tuple[int, int] | None]:
    selected_resolution: dict[str, object] | None = None
    if args.resolution_index is not None:
        if not 0 <= args.resolution_index < capability.iImageSizeDesc:
            raise RuntimeError(
                f"Resolution index {args.resolution_index} is unavailable; "
                f"choose an index from 0 to {capability.iImageSizeDesc - 1}"
            )
        resolution = capability.pImageSizeDesc[args.resolution_index]
        if mvsdk.CameraSetImageResolution(handle, resolution) != mvsdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError(f"Could not select SDK resolution index {args.resolution_index}")
        selected_resolution = {
            "index": args.resolution_index,
            "description": resolution.GetDescription() if hasattr(resolution, "GetDescription") else None,
            "width": int(resolution.iWidth),
            "height": int(resolution.iHeight),
        }
    is_mono = bool(capability.sIspCapacity.bMonoSensor)
    configured_color = camera_config.get("color")
    if configured_color is not None and bool(configured_color) == is_mono:
        raise RuntimeError("Camera colour setting disagrees with the SDK-reported sensor type")
    output_format = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if is_mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    configured_pixel_format = str(camera_config.get("pixel_format") or ("mono8" if is_mono else "bgr8")).lower()
    expected_pixel_format = "mono8" if is_mono else "bgr8"
    if configured_pixel_format != expected_pixel_format:
        raise RuntimeError(
            f"Camera pixel_format={configured_pixel_format!r} disagrees with SDK sensor output {expected_pixel_format!r}"
        )
    if mvsdk.CameraSetIspOutFormat(handle, output_format) != mvsdk.CAMERA_STATUS_SUCCESS:
        raise RuntimeError(f"Could not set SDK output format to {expected_pixel_format}")
    if camera_config.get("trigger_mode", "continuous") != "continuous":
        raise RuntimeError("Smoke test only supports trigger_mode: continuous")
    if mvsdk.CameraSetTriggerMode(handle, 0) != mvsdk.CAMERA_STATUS_SUCCESS:
        raise RuntimeError("Could not enable continuous acquisition")
    mode, exposure_us = _resolve_exposure(args, camera_config)
    requested_ae_exposure_range, requested_ae_gain_range = _requested_ae_ranges(args, mode)
    sdk_ae_exposure_range: tuple[float, float] | None = None
    sdk_ae_gain_range: tuple[int, int] | None = None
    if mode == "auto":
        sdk_ae_exposure_range, sdk_ae_gain_range = _configure_auto_exposure_ranges(
            mvsdk, handle, requested_ae_exposure_range, requested_ae_gain_range
        )
        if mvsdk.CameraSetAeState(handle, 1) != mvsdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError("Could not enable auto exposure")
    else:
        if mvsdk.CameraSetAeState(handle, 0) != mvsdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError("Could not disable auto exposure")
        if hasattr(mvsdk, "CameraGetExposureTimeRange"):
            minimum, maximum, _step = mvsdk.CameraGetExposureTimeRange(handle)
            if exposure_us is None or not float(minimum) <= float(exposure_us) <= float(maximum):
                raise RuntimeError(
                    f"manual exposure {exposure_us} us is outside SDK range [{minimum}, {maximum}] us"
                )
        if mvsdk.CameraSetExposureTime(handle, exposure_us) != mvsdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError(f"Could not set exposure to {exposure_us} us")
    return is_mono, mode, exposure_us, selected_resolution, sdk_ae_exposure_range, sdk_ae_gain_range


def _configure_preview() -> bool:
    try:
        cv2.namedWindow("MindVision preview", cv2.WINDOW_NORMAL)
    except cv2.error as exc:
        raise RuntimeError(f"--preview requested but OpenCV GUI is unavailable: {exc}") from exc
    return True


def _show_preview(image: np.ndarray, width: int, exposure_us: int, gain: float, timestamp: int, fps: float) -> bool:
    height = max(1, int(round(image.shape[0] * width / image.shape[1])))
    display = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    text = f"exp={exposure_us} us gain={gain:.3g} ts={timestamp} fps={fps:.2f}"
    cv2.putText(display, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow("MindVision preview", display)
    try:
        key = cv2.waitKey(1) & 0xFF
    except cv2.error as exc:
        raise RuntimeError(f"OpenCV preview waitKey failed: {exc}") from exc
    return key not in {ord("q"), 27}


def capture(args: argparse.Namespace, sdk_loader: Callable[[Path], Any] | None = None) -> dict[str, object]:
    with open(args.config, "r", encoding="utf-8") as handle:
        camera_config = (yaml.safe_load(handle) or {}).get("camera", {})
    if not isinstance(camera_config, dict):
        raise RuntimeError("camera configuration must be a mapping")
    if args.frames <= 0 or args.timeout_ms <= 0 or args.warmup_frames < 0:
        raise ValueError("--frames and --timeout-ms must be positive; --warmup-frames cannot be negative")
    if args.preview_width <= 0:
        raise ValueError("--preview-width must be positive")
    mvsdk = (sdk_loader or _load_official_binding)(Path(args.sdk_root).expanduser().resolve())
    device = _select_device(mvsdk, camera_config.get("serial_number"))
    handle: Any | None = None
    output_buffer: Any = 0
    preview_open = False
    timeout_count = 0
    capture_error_count = 0
    warnings: list[str] = []
    read_ms: list[float] = []
    host_success_ns: list[int] = []
    device_timestamps: list[int] = []
    exposures: list[int] = []
    gains: list[float] = []
    analog_gain_raws: list[int] = []
    dimensions: set[tuple[int, int, int, int]] = set()
    first_saved_frame: np.ndarray | None = None
    last_saved_frame: np.ndarray | None = None
    first_saved_metadata: tuple[tuple[int, ...], int] | None = None
    last_saved_metadata: tuple[tuple[int, ...], int] | None = None
    saved_image_path: Path | None = None
    selected_resolution: dict[str, object] | None = None
    is_mono = False
    requested_mode = "auto"
    requested_exposure_us: int | None = None
    actual_mode: str | None = None
    requested_ae_exposure_range: tuple[float, float] | None = None
    requested_ae_gain_range: tuple[int, int] | None = None
    sdk_ae_exposure_range: tuple[float, float] | None = None
    sdk_ae_gain_range: tuple[int, int] | None = None
    warmup_successful = 0
    captured_frames = 0
    preview_exit = False
    saved_stats: dict[str, object] | None = None
    try:
        handle = mvsdk.CameraInit(device, -1, -1)
        capability = mvsdk.CameraGetCapability(handle)
        (
            is_mono,
            requested_mode,
            requested_exposure_us,
            selected_resolution,
            sdk_ae_exposure_range,
            sdk_ae_gain_range,
        ) = _configure_camera(
            mvsdk, handle, capability, args, camera_config
        )
        requested_ae_exposure_range, requested_ae_gain_range = _requested_ae_ranges(args, requested_mode)
        if hasattr(mvsdk, "CameraGetAeState"):
            actual_mode = "auto" if int(mvsdk.CameraGetAeState(handle)) else "manual"
        channels = 1 if is_mono else 3
        output_buffer = mvsdk.CameraAlignMalloc(
            int(capability.sResolutionRange.iWidthMax) * int(capability.sResolutionRange.iHeightMax) * channels,
            16,
        )
        if not output_buffer:
            raise RuntimeError("CameraAlignMalloc returned a null buffer")
        if args.preview:
            preview_open = _configure_preview()
        if mvsdk.CameraPlay(handle) != mvsdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError("Could not start acquisition")

        def acquire_one() -> tuple[np.ndarray, Any] | None:
            nonlocal timeout_count, capture_error_count
            started_ns = time.perf_counter_ns()
            raw_buffer: Any | None = None
            try:
                try:
                    raw_buffer, frame_head = mvsdk.CameraGetImageBuffer(handle, args.timeout_ms)
                except Exception as exc:  # noqa: BLE001 - classify only vendor timeout
                    if _is_timeout(mvsdk, exc):
                        timeout_count += 1
                        return None
                    capture_error_count += 1
                    raise
                read_ms.append((time.perf_counter_ns() - started_ns) / 1_000_000)
                if mvsdk.CameraImageProcess(handle, raw_buffer, output_buffer, frame_head) != mvsdk.CAMERA_STATUS_SUCCESS:
                    capture_error_count += 1
                    raise RuntimeError("CameraImageProcess failed")
                return _frame_from_processed_buffer(mvsdk, output_buffer, frame_head, is_mono), frame_head
            finally:
                if raw_buffer is not None:
                    release_status = mvsdk.CameraReleaseImageBuffer(handle, raw_buffer)
                    if release_status not in (None, mvsdk.CAMERA_STATUS_SUCCESS):
                        raise RuntimeError(f"CameraReleaseImageBuffer failed with status {release_status}")

        warmup_attempts = 0
        max_warmup_attempts = max(args.warmup_frames * 3 + 10, args.warmup_frames)
        while warmup_successful < args.warmup_frames and warmup_attempts < max_warmup_attempts:
            warmup_attempts += 1
            if acquire_one() is not None:
                warmup_successful += 1
        if warmup_successful < args.warmup_frames:
            warnings.append(
                f"warmup completed only {warmup_successful}/{args.warmup_frames} successful frames "
                f"after {warmup_attempts} attempts"
            )

        max_capture_attempts = max(args.frames * 3 + 10, args.frames)
        attempts = 0
        while captured_frames < args.frames and attempts < max_capture_attempts and not preview_exit:
            attempts += 1
            acquired = acquire_one()
            if acquired is None:
                continue
            frame, frame_head = acquired
            captured_frames += 1
            host_success_ns.append(time.perf_counter_ns())
            dimensions.add((int(frame_head.iWidth), int(frame_head.iHeight), int(frame_head.uBytes), int(frame_head.uiMediaType)))
            device_timestamps.append(int(frame_head.uiTimeStamp))
            exposures.append(int(frame_head.uiExpTime))
            gains.append(float(frame_head.fAnalogGain))
            if hasattr(mvsdk, "CameraGetAnalogGain"):
                analog_gain_raws.append(int(mvsdk.CameraGetAnalogGain(handle)))
            if args.save_image is not None:
                saved_frame = frame.copy()
                metadata = (tuple(int(value) for value in frame.shape), 1 if frame.ndim == 2 else int(frame.shape[2]))
                if args.save_frame == "first" and first_saved_frame is None:
                    first_saved_frame = saved_frame
                    first_saved_metadata = metadata
                if args.save_frame == "last":
                    last_saved_frame = saved_frame
                    last_saved_metadata = metadata
            if args.preview:
                current_fps = 0.0
                if len(host_success_ns) > 1 and host_success_ns[-1] > host_success_ns[0]:
                    current_fps = (len(host_success_ns) - 1) / ((host_success_ns[-1] - host_success_ns[0]) / 1_000_000_000)
                preview_exit = not _show_preview(
                    frame,
                    args.preview_width,
                    int(frame_head.uiExpTime),
                    float(frame_head.fAnalogGain),
                    int(frame_head.uiTimeStamp),
                    current_fps,
                )
        if captured_frames < args.frames and not preview_exit:
            warnings.append(f"captured only {captured_frames}/{args.frames} successful frames")
        if args.save_image is not None:
            destination = Path(args.save_image).expanduser()
            if not destination.parent.is_dir():
                raise RuntimeError(f"Image output directory does not exist: {destination.parent}")
            if args.save_frame == "first":
                frame_to_save, save_metadata = first_saved_frame, first_saved_metadata
            else:
                frame_to_save, save_metadata = last_saved_frame, last_saved_metadata
            if frame_to_save is None or save_metadata is None:
                raise RuntimeError("No valid frame is available to save")
            if not cv2.imwrite(str(destination), frame_to_save):
                raise RuntimeError(f"Could not save captured image: {destination}")
            saved_image_path = destination.resolve()
            saved_stats = _saved_file_statistics(destination, save_metadata[0], save_metadata[1])
    finally:
        try:
            if preview_open:
                cv2.destroyWindow("MindVision preview")
                cv2.destroyAllWindows()
        finally:
            try:
                if output_buffer:
                    mvsdk.CameraAlignFree(output_buffer)
                    output_buffer = 0
            finally:
                if handle is not None:
                    mvsdk.CameraUnInit(handle)

    if camera_config.get("gain_db") is not None:
        warnings.append("camera.gain_db is not applied by this test; actual fAnalogGain is reported per frame")
    if camera_config.get("model") and camera_config.get("model") != "MV-SUF1205GC":
        warnings.append(
            f"configuration model is {camera_config['model']!r}; SDK-reported model is expected to be MV-SUF1205GC "
            "for the connected color camera"
        )
    if saved_stats is not None and (
        float(saved_stats["underexposed_ratio"]) > 0.65
        or float(saved_stats["gray_mean"]) < 16.0
        or float(saved_stats["gray_percentiles_p1_p5_p50_p95_p99"][3]) < 32.0
    ):
        warnings.append(
            "saved image is severely underexposed by the configured threshold; this is an exposure/scene warning, "
            "not an automatic camera-fault diagnosis"
        )
    device_delta = (device_timestamps[-1] - device_timestamps[0]) if len(device_timestamps) >= 2 else 0
    device_monotonic = all(b > a for a, b in zip(device_timestamps, device_timestamps[1:]))
    device_fps = (len(device_timestamps) - 1) / (device_delta * 0.0001) if device_delta > 0 and len(device_timestamps) > 1 else 0.0
    host_delta_ns = host_success_ns[-1] - host_success_ns[0] if len(host_success_ns) >= 2 else 0
    host_fps = (len(host_success_ns) - 1) / (host_delta_ns / 1_000_000_000) if host_delta_ns > 0 else 0.0
    return {
        "camera_name": device.GetFriendlyName(),
        "serial_number": device.GetSn(),
        "port_type": device.GetPortType(),
        "configured_model": camera_config.get("model"),
        "sdk_reported_model": device.GetFriendlyName(),
        "selected_resolution": selected_resolution,
        "requested_exposure_mode": requested_mode,
        "requested_exposure_us": requested_exposure_us,
        "requested_ae_exposure_range_us": list(requested_ae_exposure_range) if requested_ae_exposure_range else None,
        "requested_ae_analog_gain_range_raw": list(requested_ae_gain_range) if requested_ae_gain_range else None,
        "sdk_ae_exposure_range_us": list(sdk_ae_exposure_range) if sdk_ae_exposure_range else None,
        "sdk_ae_analog_gain_range_raw": list(sdk_ae_gain_range) if sdk_ae_gain_range else None,
        "reported_exposure_mode": actual_mode,
        "reported_exposure_us_first_last": [exposures[0], exposures[-1]] if exposures else [None, None],
        "reported_analog_gain_first_last": [gains[0], gains[-1]] if gains else [None, None],
        "reported_analog_gain_raw_first_last": [analog_gain_raws[0], analog_gain_raws[-1]] if analog_gain_raws else [None, None],
        "final_analog_gain_raw": analog_gain_raws[-1] if analog_gain_raws else None,
        "sensor_mono": is_mono,
        "pixel_format": "mono8" if is_mono else "bgr8",
        "frame_formats": [list(values) for values in sorted(dimensions)],
        "warmup_frames": warmup_successful,
        "warmup_frames_requested": args.warmup_frames,
        "captured_frames": captured_frames,
        "timeout_count": timeout_count,
        "capture_error_count": capture_error_count,
        "device_timestamp_monotonic": device_monotonic,
        "device_timestamp_delta": device_delta,
        "device_timestamp_fps": round(device_fps, 6),
        "host_capture_fps": round(host_fps, 6),
        "source_read_ms": {
            "mean": round(statistics.mean(read_ms), 3) if read_ms else 0.0,
            "p95": round(_percentile(read_ms, 0.95), 3),
            "max": round(max(read_ms), 3) if read_ms else 0.0,
        },
        "saved_image": str(saved_image_path) if saved_image_path else None,
        "saved_frame_policy": args.save_frame,
        "saved_frame_statistics": saved_stats,
        "warnings": warnings,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MindVision official SDK hardware capture and exposure test")
    parser.add_argument("--config", default="configs/camera_mv_suf1205gc_t.yaml")
    parser.add_argument("--sdk-root", required=True, help="Official extracted Linux SDK root")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=None, help="Discard this many successfully processed frames first")
    parser.add_argument("--timeout-ms", type=int, default=1000)
    parser.add_argument("--resolution-index", type=int, help="Use an official SDK image-resolution index")
    parser.add_argument("--exposure-mode", choices=("auto", "manual"), help="Explicitly select auto or manual exposure")
    parser.add_argument("--exposure-us", type=int, help="Legacy-compatible manual exposure in microseconds")
    parser.add_argument("--ae-min-exposure-us", type=float, help="Auto-exposure lower bound in microseconds")
    parser.add_argument("--ae-max-exposure-us", type=float, help="Auto-exposure upper bound in microseconds")
    parser.add_argument("--ae-min-analog-gain-raw", type=int, help="Auto analog-gain lower bound in SDK raw units")
    parser.add_argument("--ae-max-analog-gain-raw", type=int, help="Auto analog-gain upper bound in SDK raw units")
    parser.add_argument("--save-frame", choices=("first", "last"), default="last")
    parser.add_argument("--save-image", help="Save a copied processed frame as a BMP")
    parser.add_argument("--preview", action="store_true", help="Show a live OpenCV preview")
    parser.add_argument("--preview-width", type=int, default=960)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.frames <= 0 or args.timeout_ms <= 0 or (args.exposure_us is not None and args.exposure_us <= 0):
        parser.error("--frames and --timeout-ms must be positive; --exposure-us must be positive")
    if args.warmup_frames is None:
        config_mode = None
        config_exposure = None
        try:
            with open(args.config, "r", encoding="utf-8") as handle:
                config = (yaml.safe_load(handle) or {}).get("camera", {})
            if isinstance(config, dict):
                config_mode = config.get("exposure_mode")
                config_exposure = config.get("exposure_us")
        except (OSError, yaml.YAMLError):
            pass
        effective_mode = args.exposure_mode or config_mode or ("manual" if args.exposure_us is not None or config_exposure is not None else "auto")
        args.warmup_frames = 60 if effective_mode == "auto" else 0
    if args.warmup_frames < 0:
        parser.error("--warmup-frames cannot be negative")
    try:
        print(json.dumps(capture(args), ensure_ascii=False, indent=2))
    except KeyboardInterrupt:
        print("MindVision camera capture interrupted; resources were cleaned up", file=sys.stderr)
        return 130
    except Exception as exc:  # noqa: BLE001 - boundary around third-party SDK and CLI validation
        print(f"MindVision camera capture failed: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
