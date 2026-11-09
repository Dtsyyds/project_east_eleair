"""Fixed-viewport Firefox checks; uses only loopback fixtures, never hardware.

UI_BASELINE_DIR optionally points at a saved copy of the previous static assets.
It enables equal-state before/after screenshots without rewriting production DOM.
"""
import base64
from contextlib import contextmanager
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import struct
import threading

import pytest

import test_visual_console as ui

browser = ui.browser
server = ui.server
OUT = ui.ROOT / '.runtime/s100p/validation/ui_refinement'


def viewport(browser, width, height):
    # WebDriver window size includes browser chrome; assert the actual CSS viewport.
    for _ in range(3):
        delta = browser.js('return [outerWidth-innerWidth,outerHeight-innerHeight]')
        browser.call('WebDriver:SetWindowRect', {'width':width+delta[0], 'height':height+delta[1]})
        if browser.js('return [innerWidth,innerHeight]') == [width,height]:
            break
    assert browser.js('return [innerWidth,innerHeight,devicePixelRatio,visualViewport.scale]') == [width,height,1,1]


def screenshot(browser, name):
    data = base64.b64decode(browser.call('WebDriver:TakeScreenshot', {'full':False}))
    dimensions = list(struct.unpack('>II', data[16:24]))
    assert dimensions == browser.js('return [innerWidth,innerHeight]'), dimensions
    OUT.mkdir(parents=True,exist_ok=True)
    (OUT/(name+'.png')).write_bytes(data)


def measurements(browser):
    return browser.js("""
      const box = selector => {const r=document.querySelector(selector).getBoundingClientRect();
        return {x:r.x,y:r.y,width:r.width,height:r.height,bottom:r.bottom};};
      const selectors=['.operations-dock','#mindvision-screen','.video-primary','.alarm-panel',
        '.tracking-panel','.sensor-panel','#quick-start','#quick-stop','#fisheye-screen','#depth_camera-screen'];
      const fonts=['.sensor-row','.sensor-status','.video-tools button','.video-title h2','.rate-pair',
        '.tracking-summary p','#tracking-age','.alarm-panel .head small','#connection'];
      return {viewport:[innerWidth,innerHeight],dpr:devicePixelRatio,zoom:visualViewport.scale,
        browser:navigator.userAgent,scroll:[scrollX,scrollY],
        boxes:Object.fromEntries(selectors.map(s=>[s,box(s)])),
        typography:Object.fromEntries(fonts.map(s=>{const c=getComputedStyle(document.querySelector(s));
          return [s,{size:c.fontSize,color:c.color}];})),
        state:window.tankConsole.snapshot()};
    """)


@contextmanager
def static_server(path):
    class QuietHandler(SimpleHTTPRequestHandler):
        def log_message(self, *_): pass
    http = ThreadingHTTPServer(('127.0.0.1',0),partial(QuietHandler,directory=str(path)))
    thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
    try: yield f'http://127.0.0.1:{http.server_port}'
    finally: http.shutdown();http.server_close();thread.join(timeout=2)


def test_fixed_viewport_comparison(browser):
    sources={'after':ui.ROOT/'apps/web_console'}
    if os.environ.get('UI_BASELINE_DIR'): sources={'before':Path(os.environ['UI_BASELINE_DIR']),**sources}
    report={}
    for phase,path in sources.items():
        with static_server(path) as base:
            for width,height in [(1920,1080),(1366,768)]:
                viewport(browser,width,height)
                # Both revisions connect only to this static test server (no real bridge).
                browser.open(base+'/?ws='+base.replace('http:', 'ws:')+'/no-websocket')
                browser.click('disconnect')
                browser.js('window.scrollTo(0,0); window.uiErrors=[]')
                assert browser.js("return document.getElementById('connection').textContent") == '已断开'
                data=measurements(browser)
                assert not data['state']['connected']
                assert all(v['displayed']==0 for v in data['state']['streams'].values())
                screenshot(browser,f'{phase}-disconnected-{width}x{height}')
                report[f'{phase}-{width}']=data
                if phase=='after':
                    ui.assert_layout(browser)
                    assert data['boxes']['#mindvision-screen']['y']<260
                    assert data['boxes']['.video-primary']['bottom']<=height
                    assert data['boxes']['.alarm-panel']['bottom']<height
                    assert data['boxes']['.tracking-panel']['bottom']<height
                    assert data['boxes']['#quick-stop']['bottom']<height
                    assert float(data['typography']['.sensor-row']['size'][:-2])>=14
                    assert float(data['typography']['.sensor-status']['size'][:-2])>=13
                    if width==1920:
                        assert data['boxes']['#depth_camera-screen']['bottom']<height
    (OUT/'fixed-viewport.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))


def test_fixed_viewport_alert_and_geometry(browser,server):
    ui.open_console(browser,server)
    browser.wait('window.tankConsole.snapshot().streams.depth_camera.displayed>3')
    for width,height in [(1920,1080),(1366,768)]:
        viewport(browser,width,height)
        ui.select_view(browser,'realtime')
        ui.assert_layout(browser)
        screenshot(browser,f'mock-streams-{width}x{height}')
        ui.scenario(server,'lost')
        browser.wait("document.getElementById('tracking-state').textContent==='巡迹丢失' && document.getElementById('top-alert').textContent.length>0 && document.querySelector('#alert-list .alert-item')!==null")
        screenshot(browser,f'mock-alarm-{width}x{height}')
        box=measurements(browser)['boxes']['.alarm-panel']
        assert 0<=box['y']<box['bottom']<=height
        assert browser.js("return !document.querySelector('#alert-list').closest('details')")
        ui.scenario(server,'normal')
        browser.wait("document.getElementById('tracking-state').textContent==='二维视觉'")
        # Existing exact-frame overlay must share the image's CSS transform and source dimensions.
        geometry=browser.js("""
          return ['fisheye','depth_camera'].map(id=>{
            const c=document.getElementById(id+'-canvas'),o=document.getElementById(id+'-overlay');
            const a=c.getBoundingClientRect(),b=o.getBoundingClientRect();
            return {source:[c.width,c.height],overlay:[o.width,o.height],
              css:[a.x,a.y,a.width,a.height],overlayCss:[b.x,b.y,b.width,b.height]};});
        """)
        for item in geometry:
            assert item['source']==item['overlay']
            assert item['css']==item['overlayCss']
            assert abs(item['css'][2]/item['css'][3]-item['source'][0]/item['source'][1])<.005
    ui.click(browser,'[data-fullscreen="depth_camera"]')
    browser.wait("document.fullscreenElement?.id==='depth_camera-screen'")
    screenshot(browser,'mock-fullscreen')
    browser.js('document.exitFullscreen()')
    browser.wait('document.fullscreenElement===null')
    assert not browser.js('return window.uiErrors')


def _serve_geometry_fixture(port, configure=None):
    """Test-only large source rasters pass through the real WebSocket decode path."""
    import copy
    import io
    from aiohttp import web
    from PIL import Image, ImageDraw, ImageFont
    from mock_console_server import MockConsole, make_app

    sizes={'mindvision':(1920,1200),'fisheye':(1280,720),'depth_camera':(640,480)}
    frames={}
    for name,(width,height) in sizes.items():
        image=Image.new('RGB',(width,height),(28,43,60));draw=ImageDraw.Draw(image)
        for x in range(0,width,100): draw.line((x,0,x,height),fill=(58,78,98),width=2)
        for y in range(0,height,100): draw.line((0,y,width,y),fill=(58,78,98),width=2)
        draw.rectangle((2,2,width-3,height-3),outline=(230,230,230),width=4)
        # Reference centerline matches the existing fixture's image_pixels result.
        cx=round(31*width/64)
        draw.line((cx,0,cx,height),fill=(220,220,220),width=5)
        try: font=ImageFont.truetype('DejaVuSans.ttf',max(16,width//50))
        except OSError: font=ImageFont.load_default()
        draw.text((20,20),f'UI TEST / NOT HARDWARE / {name} {width} x {height}',fill=(230,230,230),font=font)
        output=io.BytesIO();image.save(output,format='JPEG',quality=85)
        frames[name]=base64.b64encode(output.getvalue()).decode()

    class GeometryMock(MockConsole):
        def status_payload(self):
            payload=super().status_payload()
            payload['devices']['mindvision']={'status':'active','configured':True,'reason':'isolated UI fixture'}
            return payload

        async def broadcast(self,kind,payload):
            if kind!='status_sensor' or 'video' not in payload:
                return await super().broadcast(kind,payload)
            ids=[payload['video']['camera_id']]
            if ids==['fisheye']: ids.append('mindvision')
            for name in ids:
                value=copy.deepcopy(payload);video=value['video'];width,height=sizes[name]
                video.update(camera_id=name,frame_id=f'{name}-{self.frame}',width=width,height=height,data=frames[name])
                result=self.tracking(name,video['frame_id'],video['stamp_ns'])
                result.update(image_width=width,image_height=height)
                for field in ['left_boundary','right_boundary','centerline','reliable_coverage']:
                    result[field]=[[x*width/64,y*height/48] for x,y in result[field]]
                value['tracking_result']=result
                await super().broadcast(kind,value)
    mock=GeometryMock();app=make_app(mock)
    if configure is not None: configure(app,mock)
    web.run_app(app,host='127.0.0.1',port=port,print=None)


@pytest.fixture
def geometry_server(tmp_path,browser):
    import subprocess
    import sys
    import time
    port=ui.helpers.free_port();base=f'http://127.0.0.1:{port}'
    with (tmp_path/'geometry.log').open('w') as log:
        process=subprocess.Popen([sys.executable,__file__,'--geometry',str(port)],stdout=log,stderr=log)
        try:
            deadline=time.monotonic()+10
            while True:
                try: ui.get_json(base+'/mock/state');break
                except OSError:
                    assert process.poll() is None and time.monotonic()<deadline,(tmp_path/'geometry.log').read_text()
                    time.sleep(.05)
            yield base
            assert ui.get_json(base+'/mock/state')['chassis_requests']==0
        finally:
            browser.click('disconnect');process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill();process.wait(timeout=5)


def test_large_source_scaling_and_fullscreen(browser,geometry_server):
    ui.open_console(browser,geometry_server)
    browser.wait("['mindvision','fisheye','depth_camera'].every(id=>window.tankConsole.snapshot().streams[id].displayed>3)")
    def check():
        result=browser.js("""
          return ['mindvision','fisheye','depth_camera'].map(id=>{
            const c=document.getElementById(id+'-canvas'),o=document.getElementById(id+'-overlay');
            const a=c.getBoundingClientRect(),b=o.getBoundingClientRect(),r=c.parentElement.getBoundingClientRect();
            return {id,source:[c.width,c.height],overlay:[o.width,o.height],
              css:[a.x,a.y,a.width,a.height],overlayCss:[b.x,b.y,b.width,b.height],
              contained:a.left>=r.left && a.right<=r.right && a.top>=r.top && a.bottom<=r.bottom,
              centerAlpha:o.getContext('2d').getImageData(Math.floor(31*c.width/64),Math.floor(c.height/2),1,1).data[3],
              matched:window.tankConsole.snapshot().streams[id].overlay_match};});
        """)
        for item in result:
            assert item['source']==item['overlay']
            assert item['css']==item['overlayCss']
            assert item['contained']
            assert abs(item['css'][2]/item['css'][3]-item['source'][0]/item['source'][1])<.005
            assert item['matched']=='matched' and item['centerAlpha']>0
        return result
    report=[]
    for width,height in [(1920,1080),(1366,768),(1024,768)]:
        viewport(browser,width,height);browser.js('window.scrollTo(0,0)')
        ui.assert_layout(browser)
        report.append({'viewport':[width,height],'geometry':check()})
        screenshot(browser,f'large-mock-{width}x{height}')
    viewport(browser,1366,768)
    ui.click(browser,'[data-fullscreen="mindvision"]')
    browser.wait("document.fullscreenElement?.id==='mindvision-screen'")
    # Allow Firefox's native fullscreen viewport to settle before measuring.
    browser.wait('innerWidth===document.fullscreenElement.clientWidth && innerHeight===document.fullscreenElement.clientHeight')
    report.append({'fullscreen':True,'geometry':check()})
    screenshot(browser,'large-mock-fullscreen')
    browser.js('document.exitFullscreen()');browser.wait('document.fullscreenElement===null')
    report.append({'fullscreen':False,'geometry':check()})
    browser.js("document.querySelector('.tracking-panel details').open=true")
    prior=browser.js("return document.getElementById('tracking-core').textContent")
    browser.wait('document.getElementById("tracking-core").textContent!=='+json.dumps(prior))
    browser.js("document.querySelector('.instrument-grid').scrollIntoView()")
    browser.wait('document.getElementById("pose-x").textContent!=="—"')
    assert browser.js('return document.getElementById("eddy-legend").children.length')>0
    assert not browser.js('return window.uiErrors')
    (OUT/'scaled-geometry.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':
    import sys
    if len(sys.argv)==3 and sys.argv[1]=='--geometry': _serve_geometry_fixture(int(sys.argv[2]))
