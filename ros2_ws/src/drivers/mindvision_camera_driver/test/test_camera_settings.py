from types import SimpleNamespace

import pytest

from mindvision_camera_driver.camera_settings import configure_exposure


class FakeSdk:
    CAMERA_STATUS_SUCCESS = 0

    def __init__(self):
        self.exposure_range = (0.0, 10_000_000.0)
        self.gain_range = (2, 16)
        self.calls = []

    def CameraGetAeExposureRange(self, _handle):
        self.calls.append("get_ae_exposure")
        return self.exposure_range

    def CameraGetAeAnalogGainRange(self, _handle):
        self.calls.append("get_ae_gain")
        return self.gain_range

    def CameraSetAeExposureRange(self, _handle, minimum, maximum):
        self.calls.append("set_ae_exposure")
        self.exposure_range = (minimum, maximum)
        return 0

    def CameraSetAeAnalogGainRange(self, _handle, minimum, maximum):
        self.calls.append("set_ae_gain")
        self.gain_range = (minimum, maximum)
        return 0

    def CameraSetAeState(self, _handle, value):
        self.calls.append(("set_ae", value))
        return 0

    def CameraGetExposureTimeRange(self, _handle):
        self.calls.append("get_manual_exposure")
        return (11.2963, 740302.7662, 11.2963)

    def CameraSetExposureTime(self, _handle, _value):
        self.calls.append("set_manual_exposure")
        return 0


def test_auto_range_validation_and_order():
    sdk = FakeSdk()
    exposure, gain = configure_exposure(sdk, 1, "auto", 0, 100, 50000, 2, 16)
    assert exposure == (100.0, 50000.0)
    assert gain == (2, 16)
    assert sdk.calls[:4] == ["get_ae_exposure", "get_ae_gain", "set_ae_exposure", "set_ae_gain"]
    assert sdk.calls[4:] == ["get_ae_exposure", "get_ae_gain", ("set_ae", 1)]


def test_manual_turns_ae_off_and_rejects_ae_ranges():
    sdk = FakeSdk()
    configure_exposure(sdk, 1, "manual", 30000, 0, 0, 0, 0)
    assert sdk.calls == ["get_manual_exposure", ("set_ae", 0), "set_manual_exposure"]
    with pytest.raises(ValueError, match="only valid"):
        configure_exposure(sdk, 1, "manual", 30000, 100, 50000, 2, 16)


def test_out_of_range_ae_request_fails_before_setters():
    sdk = FakeSdk()
    with pytest.raises(ValueError, match="outside"):
        configure_exposure(sdk, 1, "auto", 0, 100, 20_000_000, 2, 16)
    assert "set_ae_exposure" not in sdk.calls
