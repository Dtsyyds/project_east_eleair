"""Shared MindVision exposure/ISP tuning, with verified device readback.

No device is opened here. Both camera adapters use their existing SDK handle.
Resolution and acquisition timing remain the responsibility of each task.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time

from .paths import config_path

FIELDS = ("auto_exposure", "exposure_time_us", "analog_gain", "ae_target",
          "white_balance", "rgb_gain_r", "rgb_gain_g", "rgb_gain_b",
          "saturation", "gamma", "contrast")


def settings_path(serial):
    if not isinstance(serial, str) or not serial or not all(c.isalnum() or c in '-_' for c in serial):
        raise ValueError("Invalid camera serial")
    return config_path("camera/tuning") / (serial + ".json")


def load_saved(serial):
    path = settings_path(serial)
    if not path.exists():
        return None
    record = json.loads(path.read_text(encoding='utf-8'))
    if record.get('schema_version') != 1 or record.get('serial_number') != serial:
        raise ValueError("相机保存配置的版本或序列号不匹配")
    if not isinstance(record.get('values'), dict) or set(record['values']) != set(FIELDS):
        raise ValueError("相机保存配置字段不完整")
    return record


def _equivalent(current, wanted, exposure_step=1.):
    ignored = set()
    if wanted['auto_exposure']:
        ignored.update(('exposure_time_us', 'analog_gain'))
    if wanted['white_balance'] == 'auto':
        ignored.update(('rgb_gain_r', 'rgb_gain_g', 'rgb_gain_b'))
    for name, value in wanted.items():
        if name in ignored:
            continue
        actual = current[name]
        if name == 'exposure_time_us':
            if not math.isclose(actual, value, rel_tol=1e-5, abs_tol=max(1., exposure_step)):
                return False
        elif isinstance(value, float):
            if not math.isclose(actual, value, rel_tol=1e-6, abs_tol=1e-6):
                return False
        elif actual != value:
            return False
    return True


class MindvisionSettings:
    def __init__(self, sdk, handle, capability, serial):
        self.sdk, self.handle, self.capability = sdk, handle, capability
        self.serial = str(serial)
        self.fault = None

    def read_values(self):
        s, h = self.sdk, self.handle
        rgb = s.CameraGetGain(h)
        return dict(auto_exposure=bool(s.CameraGetAeState(h)),
                    exposure_time_us=float(s.CameraGetExposureTime(h)),
                    analog_gain=float(s.CameraGetAnalogGain(h)) * float(self.capability.sExposeDesc.fAnalogGainStep),
                    ae_target=int(s.CameraGetAeTarget(h)),
                    white_balance='auto' if s.CameraGetWbMode(h) else 'off',
                    rgb_gain_r=int(rgb[0]), rgb_gain_g=int(rgb[1]), rgb_gain_b=int(rgb[2]),
                    saturation=int(s.CameraGetSaturation(h)), gamma=int(s.CameraGetGamma(h)),
                    contrast=int(s.CameraGetContrast(h)))

    def ranges(self):
        cap = self.capability
        low, high, step = self.sdk.CameraGetExposureTimeRange(self.handle)
        gain_step = float(cap.sExposeDesc.fAnalogGainStep)
        ranges = {'exposure_time_us': [float(low), float(high), float(step)],
                  'analog_gain': [cap.sExposeDesc.uiAnalogGainMin * gain_step,
                                  cap.sExposeDesc.uiAnalogGainMax * gain_step, gain_step],
                  'ae_target': [int(cap.sExposeDesc.uiTargetMin), int(cap.sExposeDesc.uiTargetMax), 1]}
        for name, attr in [('saturation', 'sSaturationRange'), ('gamma', 'sGammaRange'), ('contrast', 'sContrastRange')]:
            r = getattr(cap, attr)
            ranges[name] = [int(r.iMin), int(r.iMax), 1]
        for name, letter in [('rgb_gain_r', 'R'), ('rgb_gain_g', 'G'), ('rgb_gain_b', 'B')]:
            r = cap.sRgbGainRange
            ranges[name] = [int(getattr(r, 'i'+letter+'GainMin')), int(getattr(r, 'i'+letter+'GainMax')), 1]
        return ranges

    def validate(self, values):
        if set(values) != set(FIELDS) or type(values['auto_exposure']) is not bool:
            raise ValueError('相机参数字段或自动曝光类型无效')
        isp = self.capability.sIspCapacity
        if bool(isp.bMonoSensor):
            raise ValueError('当前调参入口仅支持彩色工业相机')
        if not (isp.bAutoExposure if values['auto_exposure'] else isp.bManualExposure):
            raise ValueError('设备不支持所选曝光模式')
        if values['white_balance'] not in ('off', 'auto'):
            raise ValueError('白平衡必须为 off 或 auto')
        if values['white_balance'] == 'auto' and not isp.bAutoWb:
            raise ValueError('设备不支持自动白平衡')
        for name, (low, high, step) in self.ranges().items():
            value = values[name]
            if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError(f'{name} 必须在 [{low}, {high}] 范围内')
            if name not in ('exposure_time_us', 'analog_gain') and int(value) != value:
                raise ValueError(f'{name} 必须为整数')
            if name == 'analog_gain' and (step <= 0 or not math.isclose(value / step, round(value / step), abs_tol=1e-6)):
                raise ValueError(f'analog_gain 必须按 {step} 的步长设置')

    def _write(self, values):
        def set_value(name, *args):
            status = getattr(self.sdk, name)(self.handle, *args)
            if status != self.sdk.CAMERA_STATUS_SUCCESS:
                raise RuntimeError(f'{name} failed: {status}')
        set_value('CameraSetAeState', int(values['auto_exposure']))
        if not values['auto_exposure']:
            set_value('CameraSetExposureTime', float(values['exposure_time_us']))
            step = float(self.capability.sExposeDesc.fAnalogGainStep)
            set_value('CameraSetAnalogGain', int(round(values['analog_gain'] / step)))
        set_value('CameraSetAeTarget', int(values['ae_target']))
        set_value('CameraSetWbMode', int(values['white_balance'] == 'auto'))
        if values['white_balance'] == 'off':
            set_value('CameraSetGain', *(int(values[k]) for k in ('rgb_gain_r', 'rgb_gain_g', 'rgb_gain_b')))
        for field, api in [('saturation', 'Saturation'), ('gamma', 'Gamma'), ('contrast', 'Contrast')]:
            set_value('CameraSet'+api, int(values[field]))

    def snapshot(self):
        values = self.read_values()
        saved = load_saved(self.serial)
        ranges = self.ranges()
        stable = dict(values)
        if stable['auto_exposure']:
            stable.pop('exposure_time_us')
            stable.pop('analog_gain')
        if stable['white_balance'] == 'auto':
            for key in ('rgb_gain_r', 'rgb_gain_g', 'rgb_gain_b'):
                stable.pop(key)
        revision = hashlib.sha256(json.dumps(stable, sort_keys=True).encode()).hexdigest()
        return {'serial_number': self.serial, 'values': values, 'ranges': ranges,
                'settings_revision': revision,
                'supports_auto_white_balance': bool(self.capability.sIspCapacity.bAutoWb),
                'saved': saved is not None and _equivalent(values, saved['values'], ranges['exposure_time_us'][2]),
                'saved_at': saved.get('saved_at') if saved else None,
                'saved_path': str(settings_path(self.serial)), 'fault': self.fault}

    def persist(self, values):
        path = settings_path(self.serial)
        path.parent.mkdir(parents=True, exist_ok=True)
        record = {'schema_version': 1, 'serial_number': self.serial, 'values': values,
                  'saved_at': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
        text = json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(text)
                stream.flush()
                os.fsync(stream.fileno())
            temporary.replace(path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def apply(self, patch, *, save=False):
        if not isinstance(patch, dict) or set(patch) - set(FIELDS):
            raise ValueError('包含不支持的相机参数')
        before = self.read_values()
        wanted = dict(before, **patch)
        self.validate(wanted)  # Validate the entire request before touching the SDK.
        try:
            self._write(wanted)
            actual = self.read_values()
            if not _equivalent(actual, wanted, self.ranges()['exposure_time_us'][2]):
                raise RuntimeError('相机参数回读与请求不一致')
            if save:
                self.persist(actual)
            self.fault = None
        except Exception as error:
            try:
                self._write(before)
                if not _equivalent(self.read_values(), before, self.ranges()['exposure_time_us'][2]):
                    raise RuntimeError('回读不一致')
            except Exception as rollback_error:
                self.fault = f'参数回滚失败：{rollback_error}'
                raise RuntimeError(f'{error}；{self.fault}') from error
            raise
        return self.snapshot()

    def restore(self):
        saved = load_saved(self.serial)
        if saved is not None:
            self.apply(saved['values'])
        return saved
