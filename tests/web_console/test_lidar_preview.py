"""Real browser checks of the bounded live cloud renderer, with no real devices."""
import importlib.util
import base64
import json
from pathlib import Path
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from functools import partial
import threading
import time

import pytest
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('lidar_browser_helpers',ROOT/'tests/system/test_websocket_console.py')
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
browser=helpers.browser


def test_r6_asset_units_coordinate_reference_and_inverse():
    paths=[ROOT/'apps'/app/'vehicle_model.js' for app in ('web_console','web_console_chassis_tracking')]
    assert paths[0].read_bytes()==paths[1].read_bytes()
    model=json.loads(paths[0].read_text().split('Object.freeze(',1)[1].rsplit(');',1)[0])
    assert model['model_id']=='r6_full_assembly_20261107'
    assert model['source_frame']=='livox_frame' and model['estimated'] is True
    R=np.array(model['rotation']);t=np.array(model['translation_m'])
    B=np.array(model['rotation_display_from_assembly'])
    offset=np.array(model['translation_display_from_assembly_m'])
    T=np.array(model['coordinate_reference']['T_assembly_from_lidar'])
    assert np.allclose(R@R.T,np.eye(3),atol=1e-8)
    assert np.linalg.det(R)==pytest.approx(1)
    assert np.allclose(t,B@T[:3,3]+offset)
    accel=np.array(model['stationary_acceleration_m_s2'])
    assert np.allclose(R@accel,[0,0,np.linalg.norm(accel)])
    p_mm=T[:3,3]*1000
    assert np.allclose((np.linalg.inv(T)@np.r_[p_mm*.001,1])[:3],0)
    assert np.allclose(B@(p_mm*.001)+offset,t)
    points=np.array([[0,0,0],[1,2,3],[-.1,.2,-.3]])
    assert np.allclose(((points@R.T+t)-t)@R,points)
    bounds=np.array(model['bounds_m'])
    assert np.allclose(bounds[1]-bounds[0],[.47171994,.30860001,.24818817],atol=1e-6)
    assert model['ground_z_m']==bounds[0,2]==0
    assert bounds[0,2]<t[2]<bounds[1,2]
    part=model['parts'][0];v=np.array(part['vertices']);f=np.array(part['faces'])
    assert part['original_triangles']==654318
    assert 1000<len(f)<=12000 and np.isfinite(v).all()
    assert f.min()>=0 and f.max()<len(v)
    assert np.all(v.min(0)>=bounds[0]-1e-5) and np.all(v.max(0)<=bounds[1]+1e-5)


@pytest.mark.parametrize('app',['web_console','web_console_chassis_tracking'])
def test_full_console_loads_cloud_module_before_initial_render(browser,app):
    # Exercise each real HTML entry point, not just the renderer in isolation.
    # Both protocol endpoints are unavailable test ports, including the robot iframe.
    class OfflineHandler(SimpleHTTPRequestHandler):
        def log_message(self,*args): pass
        def do_GET(self):
            if self.path.split('?')[0]=='/connection_config.js':
                body=b'window.RobotConnectionDefaults={unified:true,inspection:"ws://127.0.0.1:9/ws/inspection",robot:"ws://127.0.0.1:9/ws/robot"};'
                self.send_response(200);self.send_header('Content-Type','application/javascript')
                self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
            else: super().do_GET()
    server=ThreadingHTTPServer(('127.0.0.1',0),partial(OfflineHandler,directory=str(ROOT/'apps'/app)))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        browser.open(f'http://127.0.0.1:{server.server_port}/?ws=ws://127.0.0.1:9/ws/inspection')
        browser.click('disconnect')
        assert browser.js('return typeof window.LidarPreview.receive')=='function'
        assert browser.js('return window.tankConsole.snapshot().connected') is False
        assert browser.js('return document.getElementById("connection").textContent')=='已断开'
        browser.wait('window.LidarPreview.snapshot().vehicle_model_loaded && window.LidarPreview.snapshot().vehicle_triangles>1000')
        assert browser.js('return window.LidarPreview.snapshot().vehicle_model_id')=='r6_full_assembly_20261107'
        browser.click('lidar-vehicle-show')
        browser.wait('window.LidarPreview.snapshot().vehicle_triangles===0')
        browser.click('lidar-vehicle-show')
        browser.click('lidar-vehicle-level')
        assert browser.js('return window.LidarPreview.snapshot().level_display') is False
        browser.click('lidar-vehicle-level');browser.click('lidar-vehicle-close')
        assert browser.js('return window.LidarPreview.snapshot().range_m')==.6
        up=browser.js('const m=window.VehiclePreviewModel;return m.rotation.map(r=>r.reduce((s,v,i)=>s+v*m.stationary_acceleration_m_s2[i],0));')
        assert abs(up[0])<1e-6 and abs(up[1])<1e-6 and 9<up[2]<11
        browser.js('window.LidarPreview.update(true,99);window.LidarPreview.receive({points:[[0,0,0]],source_points:1,frame_id:"other_sensor",stamp_ns:"1",position_unit:"m"});')
        browser.wait('window.LidarPreview.snapshot().vehicle_triangles===0')
        browser.js('window.LidarPreview.receive({points:[[0,0,0]],source_points:1,frame_id:"livox_frame",stamp_ns:"2",position_unit:"m"});')
        browser.wait('window.LidarPreview.snapshot().vehicle_triangles>1000')
        results=[]
        for width,height in [(1920,1080),(1366,768),(600,900)]:
            browser.call('WebDriver:SetWindowRect',{'width':width,'height':height})
            time.sleep(.15)
            layout=browser.js('''
              const selectors={depth:'[data-stream="depth_camera"]',fish:'[data-stream="fisheye"]',
                lidar:'#lidar-preview-panel',industrial:'[data-stream="mindvision"]'};
              const box=e=>{const b=e.getBoundingClientRect();return {x:b.x,y:b.y,w:b.width,h:b.height,bottom:b.bottom}};
              const ids=[...document.querySelectorAll('[id]')].map(e=>e.id);
              return {viewport:document.documentElement.clientWidth,scrollWidth:document.documentElement.scrollWidth,
                cards:Object.fromEntries(Object.entries(selectors).map(([k,s])=>[k,box(document.querySelector(s))])),
                screens:Object.fromEntries(Object.entries(selectors).map(([k,s])=>[k,box(document.querySelector(s+' .screen'))])),
                duplicates:ids.filter((id,i)=>ids.indexOf(id)!==i)};''')
            cards=layout['cards'];screens=layout['screens']
            assert not layout['duplicates'] and layout['scrollWidth']<=layout['viewport']+1,layout
            assert abs(cards['depth']['x']-cards['lidar']['x'])<1,layout
            assert cards['lidar']['y']>=cards['depth']['bottom'],layout
            if layout['viewport']>720:
                assert abs(cards['depth']['y']-cards['fish']['y'])<1,layout
                assert abs(cards['lidar']['y']-cards['industrial']['y'])<1,layout
                assert cards['fish']['x']>cards['depth']['x'],layout
                assert max(v['w'] for v in screens.values())-min(v['w'] for v in screens.values())<1,layout
                assert max(v['h'] for v in screens.values())-min(v['h'] for v in screens.values())<1,layout
            else:
                assert cards['lidar']['y']<cards['fish']['y']<cards['industrial']['y'],layout
            results.append(layout)
            out=ROOT/'.runtime/s100p/validation/four_window_restore'
            out.mkdir(parents=True,exist_ok=True)
            if width==1920:
                (out/(app+'.png')).write_bytes(base64.b64decode(browser.call('WebDriver:TakeScreenshot',{})))
        (out/(app+'.json')).write_text(json.dumps(results,indent=2))
    finally:
        server.shutdown();server.server_close();thread.join(timeout=2)


@pytest.mark.parametrize('app',['web_console','web_console_chassis_tracking'])
def test_cloud_render_stale_reconnect_and_bounds(browser,tmp_path,app):
    (tmp_path/'lidar_preview.js').write_bytes((ROOT/'apps'/app/'lidar_preview.js').read_bytes())
    (tmp_path/'index.html').write_text('''<!doctype html><meta charset="utf-8">
      <canvas id="lidar-preview" style="width:600px;height:360px"></canvas>
      <span id="lidar-preview-status"></span><p id="lidar-preview-meta"></p>
      <button id="lidar-preview-reset">reset</button><button id="lidar-preview-top">top</button>
      <script src="lidar_preview.js"></script>''')
    class Quiet(SimpleHTTPRequestHandler):
        def log_message(self,*args): pass
    server=ThreadingHTTPServer(('127.0.0.1',0),partial(Quiet,directory=str(tmp_path)))
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        browser.call('WebDriver:Navigate',{'url':f'http://127.0.0.1:{server.server_port}/'})
        browser.js('''window.LidarPreview.update(true,1);
          window.LidarPreview.receive({points:[[1,2,3],[-1,-2,-3]],source_points:100,
            frame_id:'fixture_lidar',stamp_ns:'123',position_unit:'m'});
          window.LidarPreview.update(true,1);''')
        browser.wait('window.LidarPreview.snapshot().displayed>0')
        assert browser.js('return window.LidarPreview.snapshot().fresh')
        assert '原始 100 点 / 预览 2 点' in browser.js('return document.getElementById("lidar-preview-meta").textContent')
        before=browser.js('return document.getElementById("lidar-preview").toDataURL()')
        browser.click('lidar-preview-top');time.sleep(.1)
        assert before!=browser.js('return document.getElementById("lidar-preview").toDataURL()')
        browser.js('''window.LidarPreview.receive({points:[[Infinity,0,0]],source_points:1,
            frame_id:'fixture_lidar',stamp_ns:'124',position_unit:'m'});
          window.LidarPreview.receive({points:Array(2001).fill([0,0,0]),source_points:2001,
            frame_id:'fixture_lidar',stamp_ns:'125',position_unit:'m'});''')
        assert browser.js('return window.LidarPreview.snapshot().invalid')==2
        time.sleep(2.6);browser.js('window.LidarPreview.update(true,1)')
        assert browser.js('return document.getElementById("lidar-preview-status").textContent')=='点云已过期'
        browser.js('window.LidarPreview.update(false,1)')
        assert browser.js('return window.LidarPreview.snapshot().points')==0
        browser.js('''window.LidarPreview.update(true,2);
          window.LidarPreview.receive({points:[[1,0,0]],source_points:1,
            frame_id:'fixture_lidar',stamp_ns:'1',position_unit:'m'});''')
        assert browser.js('return window.LidarPreview.snapshot().fresh')
    finally:
        server.shutdown();server.server_close();thread.join(timeout=2)
