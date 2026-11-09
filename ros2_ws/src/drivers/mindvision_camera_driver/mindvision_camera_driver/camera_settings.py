"""SDK-independent validation rules for MindVision exposure settings."""

from __future__ import annotations

from typing import Any


def optional_pair(first: Any, second: Any, name: str, integer: bool = False):
    first_unset = first is None or (int(first) == 0 if integer else float(first) == 0.0)
    second_unset = second is None or (int(second) == 0 if integer else float(second) == 0.0)
    if first_unset and second_unset:
        return None
    if first_unset or second_unset:
        raise ValueError(f"{name} minimum and maximum must be supplied together")
    return (int(first), int(second)) if integer else (float(first), float(second))


def configure_exposure(
    sdk: Any,
    handle: Any,
    mode: str,
    exposure_us: float,
    ae_min_exposure_us: Any,
    ae_max_exposure_us: Any,
    ae_min_analog_gain_raw: Any,
    ae_max_analog_gain_raw: Any,
) -> tuple[tuple[float, float] | None, tuple[int, int] | None]:
    mode = str(mode).lower()
    if mode not in {"auto", "manual"}:
        raise ValueError("exposure_mode must be auto or manual")
    requested_exposure_range = optional_pair(
        ae_min_exposure_us, ae_max_exposure_us, "AE exposure range"
    )
    requested_gain_range = optional_pair(
        ae_min_analog_gain_raw, ae_max_analog_gain_raw, "AE analog gain raw range", integer=True
    )
    if mode != "auto" and (requested_exposure_range is not None or requested_gain_range is not None):
        raise ValueError("AE range parameters are only valid with exposure_mode=auto")
    if mode == "auto":
        if float(exposure_us) > 0.0:
            raise ValueError("exposure_us must be 0 in auto mode; use exposure_mode=manual")
        sdk_exposure_range = tuple(float(value) for value in sdk.CameraGetAeExposureRange(handle))
        sdk_gain_range = tuple(int(value) for value in sdk.CameraGetAeAnalogGainRange(handle))
        if requested_exposure_range is not None:
            minimum, maximum = requested_exposure_range
            if minimum > maximum or minimum < sdk_exposure_range[0] or maximum > sdk_exposure_range[1]:
                raise ValueError(f"requested AE exposure range {requested_exposure_range} is outside {sdk_exposure_range}")
        if requested_gain_range is not None:
            minimum, maximum = requested_gain_range
            if minimum > maximum or minimum < sdk_gain_range[0] or maximum > sdk_gain_range[1]:
                raise ValueError(f"requested AE analog gain raw range {requested_gain_range} is outside {sdk_gain_range}")
        if requested_exposure_range is not None:
            if sdk.CameraSetAeExposureRange(handle, *requested_exposure_range) != sdk.CAMERA_STATUS_SUCCESS:
                raise RuntimeError("could not set MindVision AE exposure range")
        if requested_gain_range is not None:
            if sdk.CameraSetAeAnalogGainRange(handle, *requested_gain_range) != sdk.CAMERA_STATUS_SUCCESS:
                raise RuntimeError("could not set MindVision AE analog gain raw range")
        readback_exposure = tuple(float(value) for value in sdk.CameraGetAeExposureRange(handle))
        readback_gain = tuple(int(value) for value in sdk.CameraGetAeAnalogGainRange(handle))
        if sdk.CameraSetAeState(handle, 1) != sdk.CAMERA_STATUS_SUCCESS:
            raise RuntimeError("could not enable MindVision auto exposure")
        return readback_exposure, readback_gain
    if float(exposure_us) <= 0.0:
        raise ValueError("manual exposure_mode requires positive exposure_us")
    minimum, maximum, _step = sdk.CameraGetExposureTimeRange(handle)
    if not float(minimum) <= float(exposure_us) <= float(maximum):
        raise ValueError(f"manual exposure {exposure_us} us is outside SDK range [{minimum}, {maximum}] us")
    if sdk.CameraSetAeState(handle, 0) != sdk.CAMERA_STATUS_SUCCESS:
        raise RuntimeError("could not disable MindVision auto exposure")
    if sdk.CameraSetExposureTime(handle, float(exposure_us)) != sdk.CAMERA_STATUS_SUCCESS:
        raise RuntimeError(f"could not set MindVision exposure to {exposure_us} us")
    return None, None
