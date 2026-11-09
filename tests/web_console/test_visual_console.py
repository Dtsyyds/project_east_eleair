"""Real Firefox UI regression against the existing isolated, non-ROS fixture.

Run with PYTHONPATH=packages/inspection_runtime/src python3 -m pytest -q -s
 tests/web_console/test_visual_console.py. Requires existing pytest/Firefox and
 test-only aiohttp. Never opens devices or invokes a real actuator service.
Set UI_SOAK_SECONDS=120 (or longer) for the optional bounded-refresh soak.
"""
import base64
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.request import Request, urlopen

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('ui_browser_helpers', ROOT / 'tests/system/test_websocket_console.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
browser = helpers.browser
OUT = ROOT / '.runtime/s100p/validation/ui_redesign'
VIEWS = ['realtime', 'collection', 'replay', 'events', 'devices']


def get_json(url):
    with urlopen(url, timeout=5) as response:
        return json.load(response)


@pytest.fixture
def server(tmp_path, browser):
    port = helpers.free_port()
    base = f'http://127.0.0.1:{port}'
    with (tmp_path / 'mock.log').open('w') as log:
        process = subprocess.Popen([sys.executable, str(ROOT / 'tests/web_console/mock_console_server.py'),
                                    '--host', '127.0.0.1', '--port', str(port)], stdout=log, stderr=log)
        try:
            deadline = time.monotonic() + 10
            while True:
                try:
                    get_json(base + '/mock/state')
                    break
                except OSError:
                    assert time.monotonic() < deadline and process.poll() is None, (tmp_path / 'mock.log').read_text()
                    time.sleep(.05)
            yield base
            assert get_json(base + '/mock/state')['chassis_requests'] == 0
        finally:
            # Close the test socket before aiohttp waits for WebSocket shutdown.
            browser.js("document.getElementById('disconnect')?.click()")
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)


def click(browser, selector):
    """Use a real WebDriver click, not JS click (checks occlusion/hit targets)."""
    element = browser.call('WebDriver:FindElement', {'using': 'css selector', 'value': selector})
    ident = element['element-6066-11e4-a52e-4f735466cecf']
    browser.call('WebDriver:ElementClick', {'id': ident})


def select_view(browser, name):
    # A new page starts at its heading, independent of the previous page scroll.
    click(browser, f'nav button[data-view="{name}"]')
    browser.wait(f"document.querySelector('.view.active').id==='view-{name}'")
    assert browser.js("return document.querySelectorAll('nav [aria-current=page]').length") == 1
    assert browser.js("return window.scrollY") == 0


def open_console(browser, server):
    browser.open(server + '/?ws=' + server.replace('http:', 'ws:') + '/ws')
    browser.wait('window.tankConsole.snapshot().connected && window.tankConsole.snapshot().mode === "mock"')
    browser.js("window.uiErrors=[];window.addEventListener('error',e=>uiErrors.push(e.message));window.addEventListener('unhandledrejection',e=>uiErrors.push(String(e.reason)))")
    assert browser.js('return !!window.ConsoleTheme && !!window.CameraTuning && !!window.TankReplay')


def capture(browser, name):
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / (name + '.png')).write_bytes(base64.b64decode(browser.call('WebDriver:TakeScreenshot', {})))


def assert_layout(browser):
    result = browser.js("""
      const visible = e => e.getClientRects().length && getComputedStyle(e).visibility !== 'hidden';
      return {
        viewport: document.documentElement.clientWidth,
        width: document.documentElement.scrollWidth,
        clipped: [...document.querySelectorAll('button,input,select,textarea,.badge,.video-title,.sensor-row')]
          .filter(visible).filter(e => e.scrollWidth > e.clientWidth + 2)
          .map(e => e.id || e.textContent),
        duplicates: [...document.querySelectorAll('[id]')].map(e=>e.id).filter((id,i,ids)=>ids.indexOf(id)!==i),
        errors: window.uiErrors
      };
    """)
    assert result['width'] <= result['viewport'] + 1, result
    assert not result['clipped'], result
    assert not result['duplicates'], result
    assert not result['errors'], result
    return result


def scenario(server, name):
    with urlopen(Request(server + '/mock/scenario/' + name, method='POST'), timeout=5) as response:
        assert response.status == 200


def test_responsive_pages_and_controls(browser, server):
    open_console(browser, server)
    report = []
    for width, height in [(1920,1080),(1440,900),(1366,768),(1024,768),(768,1024),(390,844)]:
        browser.call('WebDriver:SetWindowRect', {'width':width, 'height':height})
        for name in VIEWS:
            select_view(browser, name)
            result=assert_layout(browser)
            report.append({'window':[width,height], 'view':name, **result})
            if width in [1920,390]: capture(browser, f'{name}-{width}')
        select_view(browser, 'realtime')
        click(browser, '#camera-tuning-toggle')
        assert browser.js("return document.getElementById('camera-tuning-toggle').getAttribute('aria-expanded')") == 'true'
        assert_layout(browser)
        click(browser, '#camera-tuning-toggle')
    (OUT / 'responsive.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    assert browser.js("return document.querySelectorAll('.sensor-row').length") == 7
    # Unsupported write/actuator capabilities remain gated.
    select_view(browser, 'devices')
    assert browser.js("return document.getElementById('move').disabled && document.getElementById('stop').disabled")


def test_collection_evidence_alerts_and_preview(browser, server):
    browser.call('WebDriver:SetWindowRect', {'width':1440,'height':1000})
    open_console(browser, server)
    browser.wait('window.tankConsole.snapshot().streams.fisheye.displayed > 2')
    select_view(browser, 'realtime')
    for stream in ['mindvision','fisheye','depth_camera']:
        click(browser, f'[data-overlay="{stream}"]')
        assert browser.js('return document.getElementById(arguments[0]+"-screen").classList.contains("raw-only")',stream)
        click(browser, f'[data-overlay="{stream}"]')
    # Fullscreen and Escape retain the original screen/overlay geometry.
    click(browser, '[data-fullscreen="mindvision"]')
    browser.wait("document.fullscreenElement?.id==='mindvision-screen'")
    browser.js("document.exitFullscreen()")
    browser.wait("document.fullscreenElement===null")
    click(browser, '.video-secondary .frame-details summary')
    assert browser.js("return document.querySelector('.video-secondary .frame-details').open")
    select_view(browser, 'collection')
    browser.js("document.getElementById('task-name').value='UI regression';document.getElementById('profile').value='mock_all'")
    click(browser, '#preflight')
    browser.wait("window.tankConsole.snapshot().task?.status==='ready'")
    click(browser, '#start-recording')
    browser.wait("window.tankConsole.snapshot().task?.status==='recording'")
    assert browser.js("return document.getElementById('quick-start').disabled && !document.getElementById('quick-stop').disabled")
    click(browser, '#quick-stop')
    browser.wait("window.tankConsole.snapshot().task?.status==='complete'")
    select_view(browser, 'events')
    click(browser, '#save-next')
    browser.wait("window.tankConsole.snapshot().events.some(e=>e.status==='complete')")
    click(browser, '#add-marker')
    browser.wait("window.tankConsole.snapshot().events.length>=2")
    click(browser, '#query-history')
    browser.wait('window.tankConsole.snapshot().history.length>0')
    select_view(browser, 'realtime')
    scenario(server, 'lost')
    browser.wait("document.getElementById('tracking-summary').classList.contains('bad') && document.getElementById('top-alert').dataset.severity==='error'")
    capture(browser, 'alarm-lost')
    click(browser, '#alert-list button')
    assert browser.js("return document.querySelector('#alert-list button').disabled")
    scenario(server, 'disk')
    browser.wait("document.getElementById('ssd-badge').classList.contains('bad')")
    browser.wait("document.getElementById('top-alert').dataset.severity==='critical'")
    scenario(server, 'normal')
    browser.wait('window.tankConsole.snapshot().streams.depth_camera.displayed>5')
    scenario(server, 'corrupt')
    browser.wait('window.tankConsole.snapshot().streams.depth_camera.decode_errors>0')
    browser.wait("document.getElementById('depth_camera-state').classList.contains('stale')",timeout=8)
    scenario(server, 'normal')
    browser.wait("document.getElementById('depth_camera-state').classList.contains('hidden')")
    select_view(browser, 'devices')
    click(browser, '#disconnect')
    assert browser.js("return document.getElementById('quick-start').disabled && document.getElementById('move').disabled")
    select_view(browser, 'realtime')
    assert browser.js("return document.getElementById('fisheye-state').classList.contains('stale')")
    assert_layout(browser)
    assert get_json(server+'/mock/state')['chassis_requests'] == 0


def test_replay_presentations(browser, server):
    """Test-only packet fixtures exercise the existing renderer, not a real MCAP."""
    open_console(browser, server)
    select_view(browser, 'replay')
    # Feed only the public presentation API. No requests, live state or files are changed.
    browser.js("""
      window.TankReplay.receive({header:{msg_type:'status_update'},payload:{replay:{
        service_id:'ui-fixture',session_id:'visual-only',revision:1,status:'paused',
        sensors:[{id:'lidar',topic:'/test/cloud',count:3},{id:'eddy_raw',topic:'/test/eddy',count:3}],topics:[]
      }}});
      for(const [id,data] of Object.entries({
        lidar:{kind:'cloud',xyz:[0,0,0,1,0,1,0,1,0],point_count:3,displayed_points:3},
        eddy_raw:{kind:'waveform',channels:[[1,2,1],[2,1,2]],sample_count:3,unit:'V'}
      })) window.TankReplay.receive({header:{msg_type:'replay_sensor'},payload:{session_id:'visual-only',sensor_id:id,topic:'/test/'+id,
          stamp_ns:'1',timestamp_ns:'1',frame_id:'test',position_seconds:0,data}});
    """)
    browser.wait("window.TankReplay.snapshot().panels.lidar.displayed===1")
    for width in [1920,1366,390]:
        browser.call('WebDriver:SetWindowRect', {'width':width,'height':1000})
        assert_layout(browser)
        capture(browser, f'replay-panels-{width}')
    assert browser.js("return window.TankReplay.snapshot().panels.lidar.errors") == 0


@pytest.mark.skipif(not os.environ.get('UI_SOAK_SECONDS'), reason='opt-in duration; no hardware stability claim')
def test_bounded_refresh_soak(browser, server):
    duration = int(os.environ['UI_SOAK_SECONDS'])
    open_console(browser, server)
    browser.wait('window.tankConsole.snapshot().streams.fisheye.displayed>3')
    start=time.monotonic();samples=[]
    while time.monotonic()-start<duration:
        snap=browser.js('return window.tankConsole.snapshot()')
        sample={'elapsed':round(time.monotonic()-start,1), 'displayed':snap['streams']['fisheye']['displayed'],
                'dom_nodes':browser.js('return document.querySelectorAll("*").length'), 'logs':len(snap['logs'])}
        samples.append(sample)
        if len(samples)>1: assert sample['displayed']>samples[-2]['displayed']
        assert sample['logs']<=200
        assert sample['dom_nodes']<1500
        assert not browser.js('return window.uiErrors')
        time.sleep(5)
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/'soak.json').write_text(json.dumps({'duration_seconds':duration,'mock_only':True,'samples':samples},indent=2))
