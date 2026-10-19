from types import SimpleNamespace

import pytest

from mindvision_camera_driver.isp_settings import apply_isp_settings


class FakeSdk:
    CAMERA_STATUS_SUCCESS = 0

    def __init__(self):
        self.values = {
            "ae": 1, "exposure": 30000.0, "analog_raw": 16, "target": 60,
            "gain": (100, 100, 100), "saturation": 80, "gamma": 100,
            "contrast": 100, "wb": 0, "temperature": 0, "speed": 0,
        }

    def __getattr__(self, name):
        setters = {
            "CameraSetAeState": "ae", "CameraSetExposureTime": "exposure",
            "CameraSetAnalogGain": "analog_raw", "CameraSetAeTarget": "target",
            "CameraSetSaturation": "saturation", "CameraSetGamma": "gamma",
            "CameraSetContrast": "contrast", "CameraSetWbMode": "wb",
            "CameraSetPresetClrTemp": "temperature", "CameraSetFrameSpeed": "speed",
        }
        getters = {
            "CameraGetAeState": "ae", "CameraGetExposureTime": "exposure",
            "CameraGetAnalogGain": "analog_raw", "CameraGetAeTarget": "target",
            "CameraGetGain": "gain", "CameraGetSaturation": "saturation",
            "CameraGetGamma": "gamma", "CameraGetContrast": "contrast",
            "CameraGetPresetClrTemp": "temperature", "CameraGetFrameSpeed": "speed",
        }
        if name == "CameraGetExposureTimeRange":
            return lambda _handle: (10.0, 1000000.0, 1.0)
        if name == "CameraSetGain":
            def set_gain(_handle, red, green, blue):
                self.values["gain"] = (red, green, blue)
                return 0
            return set_gain
        if name == "CameraSetOnceWB":
            return lambda _handle: 0
        if name in setters:
            def setter(_handle, value):
                self.values[setters[name]] = value
                return 0
            return setter
        if name in getters:
            return lambda _handle: self.values[getters[name]]
        raise AttributeError(name)


def capability(*, auto_wb=0, once_wb=1, mono=0):
    return SimpleNamespace(
        sIspCapacity=SimpleNamespace(
            bMonoSensor=mono, bAutoExposure=1, bManualExposure=1,
            bAutoWb=auto_wb, bWbOnce=once_wb,
        ),
        sExposeDesc=SimpleNamespace(
            fAnalogGainStep=0.125, uiAnalogGainMin=16, uiAnalogGainMax=64,
            uiTargetMin=1, uiTargetMax=255,
        ),
        sRgbGainRange=SimpleNamespace(
            iRGainMin=0, iRGainMax=399, iGGainMin=0, iGGainMax=399,
            iBGainMin=0, iBGainMax=399,
        ),
        sSaturationRange=SimpleNamespace(iMin=0, iMax=200),
        sGammaRange=SimpleNamespace(iMin=0, iMax=200),
        sContrastRange=SimpleNamespace(iMin=0, iMax=200),
        iClrTempDesc=0,
        iFrameSpeedDesc=0,
    )


def test_fixed_sus1207_settings_are_applied_and_read_back():
    sdk = FakeSdk()
    result = apply_isp_settings(
        sdk, 1, capability(), auto_exposure=False, exposure_time_us=70877,
        analog_gain=4.0, ae_target=70, rgb_gain=(131, 110, 147),
        saturation=100, gamma=-1, contrast=-1, white_balance="off",
        color_temperature_index=-1, frame_speed_index=-1,
    )
    assert result.auto_exposure is False
    assert result.exposure_time_us == 70877
    assert result.analog_gain_raw == 32
    assert result.analog_gain == 4.0
    assert result.rgb_gain == (131, 110, 147)
    assert result.saturation == 100


def test_unsupported_auto_white_balance_fails_explicitly():
    with pytest.raises(RuntimeError, match="does not support automatic white balance"):
        apply_isp_settings(
            FakeSdk(), 1, capability(auto_wb=0), auto_exposure=True,
            exposure_time_us=0, analog_gain=0, ae_target=-1, rgb_gain=None,
            saturation=-1, gamma=-1, contrast=-1, white_balance="auto",
            color_temperature_index=-1, frame_speed_index=-1,
        )


def test_partial_or_out_of_range_color_values_fail():
    with pytest.raises(ValueError, match="rgb_gain_r"):
        apply_isp_settings(
            FakeSdk(), 1, capability(), auto_exposure=True, exposure_time_us=0,
            analog_gain=0, ae_target=-1, rgb_gain=(500, 100, 100),
            saturation=-1, gamma=-1, contrast=-1, white_balance="unchanged",
            color_temperature_index=-1, frame_speed_index=-1,
        )
