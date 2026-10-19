"""Validated application of MindVision exposure and ISP controls."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class AppliedSettings:
    auto_exposure: bool
    exposure_time_us: float
    analog_gain_raw: int
    analog_gain: float
    ae_target: int
    rgb_gain: tuple[int, int, int] | None
    saturation: int | None
    gamma: int | None
    contrast: int | None
    white_balance: str
    color_temperature_index: int | None
    frame_speed_index: int | None


def _set_checked(sdk: Any, name: str, handle: Any, *values: Any) -> None:
    status = getattr(sdk, name)(handle, *values)
    if status != sdk.CAMERA_STATUS_SUCCESS:
        raise RuntimeError(f"MindVision {name} failed with status {status}")


def _bounded(name: str, value: int, minimum: int, maximum: int) -> int:
    value = int(value)
    if value < int(minimum) or value > int(maximum):
        raise ValueError(f"{name}={value} is outside SDK range [{minimum}, {maximum}]")
    return value


def apply_isp_settings(
    sdk: Any,
    handle: Any,
    capability: Any,
    *,
    auto_exposure: bool,
    exposure_time_us: float,
    analog_gain: float,
    ae_target: int,
    rgb_gain: tuple[int, int, int] | None,
    saturation: int,
    gamma: int,
    contrast: int,
    white_balance: str,
    color_temperature_index: int,
    frame_speed_index: int,
    log: Callable[[str], None] | None = None,
) -> AppliedSettings:
    """Apply only supported controls and verify every configured value by readback.

    Negative integer values and non-positive analog gain mean "leave the SDK
    value unchanged". ``analog_gain`` is a multiplier; the SDK setter uses a
    raw integer whose multiplier is ``raw * fAnalogGainStep``.
    """
    log = log or (lambda _message: None)
    isp = capability.sIspCapacity
    expose = capability.sExposeDesc
    is_mono = bool(isp.bMonoSensor)

    if auto_exposure:
        if not bool(isp.bAutoExposure):
            raise RuntimeError("MindVision camera does not support auto exposure")
        _set_checked(sdk, "CameraSetAeState", handle, 1)
    else:
        if not bool(isp.bManualExposure):
            raise RuntimeError("MindVision camera does not support manual exposure")
        if float(exposure_time_us) <= 0.0:
            raise ValueError("manual exposure requires exposure_time_us > 0")
        minimum, maximum, _step = sdk.CameraGetExposureTimeRange(handle)
        if not float(minimum) <= float(exposure_time_us) <= float(maximum):
            raise ValueError(
                f"exposure_time_us={exposure_time_us} is outside SDK range [{minimum}, {maximum}]"
            )
        _set_checked(sdk, "CameraSetAeState", handle, 0)
        _set_checked(sdk, "CameraSetExposureTime", handle, float(exposure_time_us))

    if float(analog_gain) > 0.0:
        step = float(expose.fAnalogGainStep)
        if step <= 0.0:
            raise RuntimeError("MindVision reported invalid analog gain step")
        raw = int(round(float(analog_gain) / step))
        _bounded("analog_gain_raw", raw, expose.uiAnalogGainMin, expose.uiAnalogGainMax)
        if abs(raw * step - float(analog_gain)) > max(1e-6, step / 2.0):
            raise ValueError(f"analog_gain={analog_gain} cannot be represented with SDK step {step}")
        _set_checked(sdk, "CameraSetAnalogGain", handle, raw)

    if int(ae_target) >= 0:
        target = _bounded("ae_target", ae_target, expose.uiTargetMin, expose.uiTargetMax)
        _set_checked(sdk, "CameraSetAeTarget", handle, target)

    applied_rgb: tuple[int, int, int] | None = None
    applied_saturation: int | None = None
    applied_gamma: int | None = None
    applied_contrast: int | None = None
    wb = str(white_balance).strip().lower()
    if wb not in {"unchanged", "off", "auto", "once"}:
        raise ValueError("white_balance must be unchanged, off, auto or once")
    color_index: int | None = None

    if is_mono:
        configured_color = (
            rgb_gain is not None or int(saturation) >= 0 or int(gamma) >= 0
            or int(contrast) >= 0 or wb != "unchanged" or int(color_temperature_index) >= 0
        )
        if configured_color:
            raise RuntimeError("color ISP controls are not supported by a monochrome sensor")
    else:
        if rgb_gain is not None:
            ranges = capability.sRgbGainRange
            red = _bounded("rgb_gain_r", rgb_gain[0], ranges.iRGainMin, ranges.iRGainMax)
            green = _bounded("rgb_gain_g", rgb_gain[1], ranges.iGGainMin, ranges.iGGainMax)
            blue = _bounded("rgb_gain_b", rgb_gain[2], ranges.iBGainMin, ranges.iBGainMax)
            _set_checked(sdk, "CameraSetGain", handle, red, green, blue)
            applied_rgb = tuple(int(value) for value in sdk.CameraGetGain(handle))
            if applied_rgb != (red, green, blue):
                raise RuntimeError(f"MindVision RGB gain readback mismatch: {applied_rgb}")
        if int(saturation) >= 0:
            value = _bounded(
                "saturation", saturation,
                capability.sSaturationRange.iMin, capability.sSaturationRange.iMax,
            )
            _set_checked(sdk, "CameraSetSaturation", handle, value)
            applied_saturation = int(sdk.CameraGetSaturation(handle))
        if int(gamma) >= 0:
            value = _bounded("gamma", gamma, capability.sGammaRange.iMin, capability.sGammaRange.iMax)
            _set_checked(sdk, "CameraSetGamma", handle, value)
            applied_gamma = int(sdk.CameraGetGamma(handle))
        if int(contrast) >= 0:
            value = _bounded(
                "contrast", contrast,
                capability.sContrastRange.iMin, capability.sContrastRange.iMax,
            )
            _set_checked(sdk, "CameraSetContrast", handle, value)
            applied_contrast = int(sdk.CameraGetContrast(handle))
        if wb == "auto":
            if not bool(isp.bAutoWb):
                raise RuntimeError("MindVision camera does not support automatic white balance")
            _set_checked(sdk, "CameraSetWbMode", handle, 1)
        elif wb in {"off", "once"}:
            _set_checked(sdk, "CameraSetWbMode", handle, 0)
            if wb == "once":
                if not bool(isp.bWbOnce):
                    raise RuntimeError("MindVision camera does not support one-shot white balance")
                _set_checked(sdk, "CameraSetOnceWB", handle)
        if int(color_temperature_index) >= 0:
            if int(capability.iClrTempDesc) <= 0:
                raise RuntimeError("MindVision camera exposes no preset color temperatures")
            indices = {
                int(capability.pClrTempDesc[index].iIndex)
                for index in range(int(capability.iClrTempDesc))
            }
            if int(color_temperature_index) not in indices:
                raise ValueError(
                    f"color_temperature_index={color_temperature_index} is unavailable; "
                    f"SDK indices={sorted(indices)}"
                )
            _set_checked(sdk, "CameraSetPresetClrTemp", handle, int(color_temperature_index))
            color_index = int(sdk.CameraGetPresetClrTemp(handle))

    speed_index: int | None = None
    if int(frame_speed_index) >= 0:
        indices = {
            int(capability.pFrameSpeedDesc[index].iIndex)
            for index in range(int(capability.iFrameSpeedDesc))
        }
        if int(frame_speed_index) not in indices:
            raise ValueError(
                f"frame_speed_index={frame_speed_index} is unavailable; SDK indices={sorted(indices)}"
            )
        _set_checked(sdk, "CameraSetFrameSpeed", handle, int(frame_speed_index))
        speed_index = int(sdk.CameraGetFrameSpeed(handle))

    result = AppliedSettings(
        auto_exposure=bool(sdk.CameraGetAeState(handle)),
        exposure_time_us=float(sdk.CameraGetExposureTime(handle)),
        analog_gain_raw=int(sdk.CameraGetAnalogGain(handle)),
        analog_gain=float(sdk.CameraGetAnalogGain(handle)) * float(expose.fAnalogGainStep),
        ae_target=int(sdk.CameraGetAeTarget(handle)),
        rgb_gain=applied_rgb,
        saturation=applied_saturation,
        gamma=applied_gamma,
        contrast=applied_contrast,
        white_balance=wb,
        color_temperature_index=color_index,
        frame_speed_index=speed_index,
    )
    log(f"MindVision applied settings: {result}")
    return result
