"""Delayed results must be displayed on their own cached source frame."""
import asyncio
import copy
import subprocess
import sys
import time
from urllib.request import Request, urlopen

import pytest
import test_visual_console as ui

browser = ui.browser


def serve(port):
    from aiohttp import web
    from mock_console_server import MockConsole, make_app

    class DelayedMock(MockConsole):
        mode = 'normal'

        async def broadcast(self, kind, payload):
            if self.mode == 'frozen' and payload.get('video', {}).get('camera_id') == 'depth_camera':
                return
            if kind == 'status_sensor' and 'tracking_result' in payload:
                payload = copy.deepcopy(payload)
                result = payload.pop('tracking_result')
                mode = self.mode
                if mode != 'off':
                    async def later():
                        await asyncio.sleep(1.8 if mode == 'expired' else .45)
                        if self.mode != mode: return
                        if mode == 'mismatch': result['frame_ref']['frame_id'] += '-wrong'
                        if mode == 'dimensions': result['image_width'] += 1
                        if mode in ('lost', 'lost_no_frame'):
                            result.update(status='LOST', vision_2d_valid=False)
                        if mode == 'lost_no_frame':
                            result['frame_ref']['frame_id'] += '-no-cached-jpeg'
                        await MockConsole.broadcast(self, 'status_sensor', {'tracking_result': result})
                    asyncio.create_task(later())
            await super().broadcast(kind, payload)

        async def change(self, request):
            self.mode = request.match_info['name']
            return web.json_response({'mode': self.mode})

    mock = DelayedMock()
    app = make_app(mock)
    app.router.add_post('/test/mode/{name}', mock.change)
    web.run_app(app, host='127.0.0.1', port=port, print=None)


@pytest.fixture
def server(tmp_path, browser):
    port = ui.helpers.free_port()
    base = f'http://127.0.0.1:{port}'
    with (tmp_path/'server.log').open('w') as log:
        proc = subprocess.Popen([sys.executable, __file__, '--serve', str(port)], stdout=log, stderr=log)
        try:
            deadline = time.monotonic()+10
            while True:
                try: ui.get_json(base+'/mock/state'); break
                except OSError:
                    assert proc.poll() is None and time.monotonic()<deadline, (tmp_path/'server.log').read_text()
                    time.sleep(.05)
            yield base
            assert ui.get_json(base+'/mock/state')['chassis_requests'] == 0
        finally:
            browser.js("document.getElementById('disconnect')?.click()")
            proc.terminate()
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired: proc.kill(); proc.wait()


def mode(server, name):
    with urlopen(Request(server+'/test/mode/'+name, method='POST'), timeout=5): pass


def test_delayed_exact_frame_visible_and_raw_fallback(browser, server):
    ui.open_console(browser, server)
    browser.wait("window.tankConsole.snapshot().streams.depth_camera.matched_displays>4")
    data = browser.js("""
      const s=window.tankConsole.snapshot().streams.depth_camera;
      const c=document.getElementById('depth_camera-overlay');
      return {s,pixel:[...c.getContext('2d').getImageData(31,24,1,1).data],
        left:[...c.getContext('2d').getImageData(21,24,1,1).data],
        right:[...c.getContext('2d').getImageData(41,24,1,1).data],
        text:document.getElementById('depth_camera-tracking-display').textContent};
    """)
    assert data['s']['presentation'] == 'tracking'
    assert data['s']['overlay_match'] == 'matched'
    assert int(data['s']['last_displayed_frame']['stamp_ns']) < int(data['s']['last_received_frame']['stamp_ns'])
    assert data['pixel'] == [255,64,64,255]
    assert data['left'] == [51,153,255,255]
    assert data['right'] == [50,209,125,255]
    assert '蓝线：左边线拟合' in data['text'] and '绿线：右边线拟合' in data['text'] and '红线：二维中心线' in data['text']
    assert data['s']['cached_frames'] <= 48 and data['s']['cached_bytes'] <= 8*1024*1024

    mode(server, 'frozen')
    browser.wait("document.getElementById('depth_camera-tracking-display').textContent.includes('画面已过期')", timeout=7)
    assert browser.js("return [...document.getElementById('depth_camera-overlay').getContext('2d').getImageData(0,0,64,48).data].every(v=>v===0)")
    mode(server, 'normal')
    browser.wait("window.tankConsole.snapshot().streams.depth_camera.overlay_match==='matched'")

    for negative in ('off', 'mismatch', 'dimensions', 'expired'):
        mode(server, negative)
        time.sleep(2.1 if negative == 'expired' else .65)
        browser.wait("window.tankConsole.snapshot().streams.depth_camera.presentation==='raw'", timeout=5)
        assert browser.js("return document.getElementById('depth_camera-overlay').getContext('2d').getImageData(31,24,1,1).data[3]") == 0

    mode(server, 'normal')
    browser.wait("window.tankConsole.snapshot().streams.depth_camera.presentation==='tracking'")
    browser.js("document.querySelector('[data-overlay=depth_camera]').click()")
    browser.wait("window.tankConsole.snapshot().streams.depth_camera.presentation==='raw'")
    assert '叠加已隐藏' in browser.js("return document.getElementById('depth_camera-tracking-display').textContent")
    browser.js("document.querySelector('[data-overlay=depth_camera]').click()")
    browser.wait("window.tankConsole.snapshot().streams.depth_camera.presentation==='tracking'")

    mode(server, 'lost')
    browser.wait("document.getElementById('depth_camera-tracking-display').textContent.includes('未检出')")
    assert browser.js("return document.getElementById('depth_camera-overlay').getContext('2d').getImageData(31,24,1,1).data[3]") == 0
    mode(server, 'normal')
    browser.wait("document.getElementById('depth_camera-overlay').getContext('2d').getImageData(31,24,1,1).data[3]>0")
    mode(server, 'lost_no_frame')
    browser.wait("window.tankConsole.snapshot().tracking?.frame_ref?.frame_id.endsWith('-no-cached-jpeg')")
    assert browser.js("return document.getElementById('depth_camera-overlay').getContext('2d').getImageData(31,24,1,1).data[3]") == 0
    browser.click('disconnect')
    assert browser.js("return window.tankConsole.snapshot().streams.depth_camera.cached_frames") == 0
    assert not browser.js('return window.uiErrors')


if __name__ == '__main__':
    serve(int(sys.argv[2]))
