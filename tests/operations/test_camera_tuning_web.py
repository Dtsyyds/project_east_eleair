"""Explicit real-camera/Firefox tuning acceptance; restore original values."""
import importlib.util
import json
import os
from pathlib import Path
import time

import pytest

from inspection_runtime.paths import project_root, runtime_root

spec = importlib.util.spec_from_file_location('camera_browser_helpers', project_root() / 'tests/system/test_websocket_console.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
browser = helpers.browser
pytestmark = pytest.mark.skipif(os.environ.get('SYSTEM_CAMERA_TUNING_TEST') != '1', reason='explicit real camera tuning test')


def test_real_camera_apply_save_and_refresh(browser):
    from websockets.sync.client import connect
    browser.open('http://127.0.0.1:8080')
    browser.wait("window.tankConsole.snapshot().connected", timeout=15)
    browser.wait("window.tankConsole.snapshot().capabilities?.operations?.['camera.settings.get']?.supported", timeout=15)
    browser.click('camera-tuning-toggle')
    browser.wait("window.CameraTuning.snapshot()?.values && !document.getElementById('camera-tuning-apply').disabled", timeout=15)
    original = browser.js('return window.CameraTuning.snapshot()')
    changed = original['values']['saturation'] - 1
    if changed < original['ranges']['saturation'][0]:
        changed += 2
    def call(ws, action, parameters):
        ident = 'camera-test-' + str(time.time_ns())
        ws.send(json.dumps({'header': {'robot_id': 'tank_inspection_robot', 'msg_type': 'command'},
                            'payload': {'action': action, 'request_id': ident, 'client_id': 'camera-acceptance', 'parameters': parameters}}))
        while True:
            payload = json.loads(ws.recv(timeout=15))['payload']
            if payload.get('request_id') == ident and payload.get('status') in ('completed', 'failed'):
                assert payload['status'] == 'completed', payload
                return payload['camera_settings']
    try:
        frames = browser.js("return window.tankConsole.snapshot().streams.mindvision.displayed")
        browser.js("const e=document.getElementById('mv-saturation');e.value=arguments[0];e.dispatchEvent(new Event('input'));", changed)
        browser.click('camera-tuning-apply')
        browser.wait(f"window.CameraTuning.snapshot()?.values.saturation==={changed} && !document.getElementById('camera-tuning-save').disabled", timeout=15)
        browser.wait(f"window.tankConsole.snapshot().streams.mindvision.displayed>{frames}", timeout=10)
        browser.click('camera-tuning-save')
        browser.wait("window.CameraTuning.snapshot()?.saved===true && !document.getElementById('camera-tuning-save').disabled", timeout=15)
        applied = browser.js('return window.CameraTuning.snapshot()')
        path = Path(applied['saved_path'])
        assert json.loads(path.read_text())['values']['saturation'] == changed
        browser.open('http://127.0.0.1:8080')
        browser.wait("window.tankConsole.snapshot().capabilities?.operations?.['camera.settings.get']?.supported", timeout=15)
        browser.click('camera-tuning-toggle')
        browser.wait(f"window.CameraTuning.snapshot()?.values.saturation==={changed}", timeout=15)
        assert browser.js("return Number(document.getElementById('mv-saturation').value)") == changed
        # Browser-native bounds reject the bad draft before a mutation is sent.
        sent = browser.js('return window.tankConsole.snapshot().counts.sent')
        browser.js("const e=document.getElementById('mv-gamma');e.value=Number(e.max)+1;e.dispatchEvent(new Event('input'));document.getElementById('camera-tuning-apply').click();")
        assert browser.js('return window.tankConsole.snapshot().counts.sent') == sent
        screenshot = browser.call('WebDriver:TakeScreenshot', {})
        import base64
        out = runtime_root() / 'validation/camera_tuning'
        out.mkdir(parents=True, exist_ok=True)
        (out/'panel.png').write_bytes(base64.b64decode(screenshot))
        (out/'browser_acceptance.json').write_text(json.dumps({'passed': True, 'original': original,
             'applied': applied, 'preview_continued': True, 'refresh_restored': True, 'invalid_draft_blocked': True}, ensure_ascii=False, indent=2))
    finally:
        with connect('ws://127.0.0.1:9001', proxy=None) as ws:
            current = call(ws, 'camera.settings.get', {})
            restored = call(ws, 'camera.settings.save', {'serial_number': current['serial_number'],
                            'expected_revision': current['settings_revision'], 'values': original['values']})
            assert restored['saved']
            assert restored['values']['saturation'] == original['values']['saturation']
