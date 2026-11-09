"""Validate SDK transaction, serial-scoped persistence and operation exclusion."""
import copy
import json
from types import SimpleNamespace as NS

import pytest

from inspection_runtime.camera_tuning import MindvisionSettings, load_saved, settings_path


class SDK:
    CAMERA_STATUS_SUCCESS = 0
    names = {'AeState': 'ae', 'ExposureTime': 'exposure', 'AnalogGain': 'gain', 'AeTarget': 'target',
             'Gain': 'rgb', 'WbMode': 'wb', 'Saturation': 'saturation', 'Gamma': 'gamma', 'Contrast': 'contrast'}

    def __init__(self):
        self.values = dict(ae=0, exposure=70877., gain=8, target=70, rgb=(131, 110, 147), wb=0,
                           saturation=100, gamma=100, contrast=100)
        self.fail = None
        self.calls = []

    def CameraGetExposureTimeRange(self, handle):
        return 10., 1000000., 1.

    def __getattr__(self, name):
        if name.startswith('CameraGet'):
            return lambda handle: self.values[self.names[name[9:]]]
        if name.startswith('CameraSet'):
            key = self.names[name[9:]]
            def set_value(handle, *args):
                self.calls.append((name, args))
                if self.fail == name:
                    self.fail = None
                    return -1
                self.values[key] = args if key == 'rgb' else args[0]
                return 0
            return set_value
        raise AttributeError(name)


@pytest.fixture
def camera(tmp_path, monkeypatch):
    monkeypatch.setenv('INSPECTION_CONFIG_ROOT', str(tmp_path))
    cap = NS(sIspCapacity=NS(bMonoSensor=0, bAutoExposure=1, bManualExposure=1, bAutoWb=1),
             sExposeDesc=NS(fAnalogGainStep=.5, uiAnalogGainMin=1, uiAnalogGainMax=32, uiTargetMin=1, uiTargetMax=255),
             sRgbGainRange=NS(iRGainMin=0, iRGainMax=399, iGGainMin=0, iGGainMax=399, iBGainMin=0, iBGainMax=399),
             sSaturationRange=NS(iMin=0, iMax=200), sGammaRange=NS(iMin=0, iMax=200), sContrastRange=NS(iMin=0, iMax=200))
    return MindvisionSettings(SDK(), 1, cap, 'fixture-001')


@pytest.mark.parametrize('patch', [{'analog_gain': 4.3}, {'gamma': 201}, {'exposure_time_us': float('nan')},
                                    {'auto_exposure': 'false'}, {'resolution_index': 3}, {'contrast': 20.5}])
def test_invalid_batch_never_partially_changes_camera(camera, patch):
    before = copy.deepcopy(camera.sdk.values)
    with pytest.raises(ValueError):
        camera.apply(dict(saturation=99, **patch), save=True)
    assert camera.sdk.values == before
    assert camera.sdk.calls == []
    assert load_saved(camera.serial) is None


def test_preview_save_restart_and_serial_isolation(camera):
    result = camera.apply({'saturation': 95})
    assert result['values']['saturation'] == 95 and not result['saved']
    result = camera.apply({'saturation': 94}, save=True)
    assert result['saved'] and load_saved(camera.serial)['values']['saturation'] == 94
    restarted = MindvisionSettings(SDK(), 1, camera.capability, camera.serial)
    restarted.restore()
    assert restarted.read_values()['saturation'] == 94
    other = MindvisionSettings(SDK(), 1, camera.capability, 'different-serial')
    assert other.restore() is None
    assert other.read_values()['saturation'] == 100


def test_sdk_failure_rolls_back_and_keeps_saved_profile(camera):
    camera.apply({}, save=True)
    original = settings_path(camera.serial).read_bytes()
    before = camera.read_values()
    camera.sdk.fail = 'CameraSetGamma'
    with pytest.raises(RuntimeError, match='CameraSetGamma'):
        camera.apply({'saturation': 50, 'gamma': 120}, save=True)
    assert camera.read_values() == before
    assert settings_path(camera.serial).read_bytes() == original


def test_disk_error_rolls_back_hardware(camera, monkeypatch):
    before = camera.read_values()
    def fail(values):
        raise OSError('disk full')
    monkeypatch.setattr(camera, 'persist', fail)
    with pytest.raises(OSError, match='disk full'):
        camera.apply({'saturation': 90}, save=True)
    assert camera.read_values() == before
    assert load_saved(camera.serial) is None


def test_automatic_values_do_not_invalidate_saved_policy(camera):
    camera.apply({'auto_exposure': True, 'white_balance': 'auto'}, save=True)
    revision = camera.snapshot()['settings_revision']
    camera.sdk.values.update(exposure=12345., gain=10, rgb=(133, 111, 148))
    assert camera.snapshot()['saved']
    assert camera.snapshot()['settings_revision'] == revision


def test_wrong_saved_serial_is_rejected(camera):
    camera.apply({}, save=True)
    path = settings_path(camera.serial)
    record = json.loads(path.read_text())
    record['serial_number'] = 'other'
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='序列号'):
        camera.restore()


def test_rollback_failure_is_reported_until_successful_recovery(camera, monkeypatch):
    camera.apply({}, save=True)
    original = settings_path(camera.serial).read_bytes()
    with monkeypatch.context() as failure:
        failure.setattr(camera.sdk, 'CameraSetGamma', lambda *args: -1)
        with pytest.raises(RuntimeError, match='参数回滚失败'):
            camera.apply({'saturation': 80, 'gamma': 120}, save=True)
    assert camera.snapshot()['fault']
    assert settings_path(camera.serial).read_bytes() == original
    camera.restore()
    assert camera.snapshot()['fault'] is None
