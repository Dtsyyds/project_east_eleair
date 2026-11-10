"""Optional real Firefox/Marionette smoke test, without third-party browser packages.
SLAM_BROWSER_TEST=1 enables it; uses 8080/8082 and only simulated SLAM state.
"""
import asyncio
import base64
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import threading
import time

import pytest
from websockets.asyncio.server import serve

from slam_console.server import Server

pytestmark=pytest.mark.skipif(os.environ.get('SLAM_BROWSER_TEST')!='1' or not shutil.which('firefox'),reason='opt-in Firefox test, requires local sockets and headless browser')
ROOT=Path(__file__).resolve().parents[3]


class Browser:
    def __init__(self, tmp_path):
        profile=tmp_path/'profile';profile.mkdir()
        with socket.socket() as probe:
            probe.bind(('127.0.0.1',0));port=probe.getsockname()[1]
        (profile/'user.js').write_text(f'user_pref("marionette.port", {port});\nuser_pref("full-screen-api.ignore-widgets",true);\nuser_pref("browser.shell.checkDefaultBrowser",false);\nuser_pref("browser.startup.homepage_override.mstone","ignore");\n')
        self.log=(tmp_path/'firefox.log').open('w')
        self.process=subprocess.Popen(['firefox','--headless','--no-remote','--profile',str(profile),'--marionette','about:blank'],stdout=self.log,stderr=subprocess.STDOUT)
        self.serial=0
        for _ in range(200):
            try:self.socket=socket.create_connection(('127.0.0.1',port),timeout=1);break
            except OSError:time.sleep(.1)
        else:
            self.process.terminate();self.process.wait(timeout=10)
            raise AssertionError((tmp_path/'firefox.log').read_text())
        self.socket.settimeout(20)
        self.read()
        self.call('WebDriver:NewSession',{'capabilities':{}})
    def read(self):
        length=b''
        while not length.endswith(b':'):
            part=self.socket.recv(1)
            if not part:raise RuntimeError('Marionette disconnected')
            length+=part
        remaining=int(length[:-1]);data=b''
        while remaining:
            part=self.socket.recv(remaining);data+=part;remaining-=len(part)
        return json.loads(data)
    def call(self,method,params=None):
        self.serial+=1;data=json.dumps([0,self.serial,method,params or {}]).encode();self.socket.sendall(str(len(data)).encode()+b':'+data)
        while True:
            reply=self.read()
            if isinstance(reply,list) and reply[0]==1 and reply[1]==self.serial:break
        if reply[2]:raise AssertionError(reply[2])
        return reply[3]
    def script(self,code):return self.call('WebDriver:ExecuteScript',{'script':code,'args':[],'newSandbox':False,'sandbox':None})['value']
    def wait(self,code):
        for _ in range(100):
            if self.script(code):return
            time.sleep(.1)
        raise AssertionError('Browser condition timeout: '+code+'; fullscreen: '+str(self.script('return [document.getElementById("fullscreen-status")?.textContent,document.getElementById("cloud-fullscreen")?.disabled,document.fullscreenElement?.id,document.getElementById("cloud-fullscreen")?.textContent,window.fullscreenClicks]')))
    def close(self):
        try:self.call('Marionette:Quit',{'flags':['eForceQuit']})
        except (OSError,RuntimeError):pass
        self.socket.close()
        try:self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.log.close()


class SimulatedManager:
    def __init__(self):
        self.current={'session_id':None,'state':'idle'};self.bridge=self;self.starts=0
    def previews(self):
        if self.current['state']!='running':return {}
        return {'/cloud_effected':{'type':'cloud','topic':'/cloud_effected','frame_id':'camera_init','stamp':time.time(),'received':time.time(),'source_points':3,'points':[[0,0,0],[1,1,1],[2,0,1]],'error':None}}
    def status(self):return dict(self.current, output_roots={'mapping':'/simulated/PCD/priori','relocation':'/simulated/PCD/relocate'})
    def maps(self):return [{'name':'prior.pcd','path':'/simulated/maps/prior.pcd'}]
    def preflight(self, params):return dict(ok=False, checked_at=time.time(), errors=['模拟输入未启动'], checks=[
        dict(id='lidar.freshness', label='雷达输入新鲜度', status='fail', detail='/simulated/lidar', suggestion='请启动传感器。')])
    async def start(self,params,request_id=None):
        await asyncio.sleep(.35)
        self.starts+=1
        self.current=dict(session_id=f'20261110_120000_{self.starts:012x}',state='running',mode=params['mode'],save=params['save'],map=None,start_request_id=request_id,save_result=None)
        return self.status()
    async def stop(self,sid):
        assert sid==self.current['session_id']
        self.current.update(state='stopped',save_result='disabled')
        return self.status()
    def logs(self,sid,cursor):return dict(text='模拟日志\n' if not cursor else '',cursor=cursor,next_cursor=13,size=13)


@pytest.mark.parametrize('fault', ['none', 'context', 'shader', 'link'])
def test_direct_page_quick_cancel_refresh_and_iframe(tmp_path, fault):
    manager=SimulatedManager();app=Server(manager,['127.0.0.1']);loop=asyncio.new_event_loop();started=threading.Event();holder={}
    def http(connection, request):
        response = app.http(connection, request)
        if response and request.path.startswith('/console.js') and fault != 'none':
            # Inject at the real script boundary; exercises control startup despite
            # context creation, shader compilation or program linking failure.
            injected = '''
window.previewFault = FAULT;
HTMLCanvasElement.prototype.getContext = function() {
  if (window.previewFault === 'context') throw Error('injected context failure');
  return new Proxy({}, {get: (_, name) => {
    if (name === 'getShaderParameter') return () => window.previewFault !== 'shader';
    if (name === 'getProgramParameter') return () => window.previewFault !== 'link';
    if (name === 'getShaderInfoLog' || name === 'getProgramInfoLog') return () => 'injected ' + window.previewFault + ' failure';
    return () => ({});
  }});
};
'''.replace('FAULT', json.dumps(fault))
            response.body = injected.encode() + response.body
            del response.headers['Content-Length']
            response.headers['Content-Length'] = str(len(response.body))
        return response
    async def serving():
        async with serve(app.client,'127.0.0.1',8082,process_request=http):
            started.set();holder['stop']=asyncio.Event();await holder['stop'].wait()
    thread=threading.Thread(target=lambda:loop.run_until_complete(serving()),daemon=True);thread.start()
    assert started.wait(5),'port 8082 must be free'
    class QuietHandler(SimpleHTTPRequestHandler):
        def log_message(self,*args):pass
    main=ThreadingHTTPServer(('127.0.0.1',8080),partial(QuietHandler,directory=str(ROOT/'apps/web_console_chassis_tracking')))
    httpthread=threading.Thread(target=main.serve_forever,daemon=True);httpthread.start()
    browser=None
    try:
        browser=Browser(tmp_path)
        browser.call('WebDriver:SetWindowRect', {'width':1440,'height':1000})
        browser.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8082/'})
        browser.wait('return document.getElementById("toggle").textContent==="启动"')
        assert browser.script('return document.getElementById("connection").textContent.includes("状态已同步")')
        browser.script('document.getElementById("preflight").click();return true')
        browser.wait('return document.getElementById("preflight-summary").textContent.includes("检查未通过")')
        assert browser.script('return document.getElementById("preflight-results").textContent.includes("请启动传感器")')
        assert manager.starts == 0
        assert browser.script('return document.getElementById("save").closest(".save-location").contains(document.getElementById("save-path")) && document.getElementById("configuration").contains(document.getElementById("save"))')
        for value, expected in [('1000','100'), ('0','30'), ('4.7','4')]:
            browser.script(f'document.getElementById("frames").value={json.dumps(value)};document.getElementById("frames").dispatchEvent(new Event("change"));return true')
            assert browser.script('return document.getElementById("frames").value') == expected
        assert browser.script('return document.getElementById("save").closest("section")===document.getElementById("preflight").closest("section") && document.getElementById("preflight").getBoundingClientRect().top>=document.querySelector(".save-location").getBoundingClientRect().bottom && document.getElementById("preflight-details").closest("section")===document.getElementById("preflight").closest("section")')
        assert browser.script('return !document.getElementById("preflight-details").open && !document.getElementById("log-details").open')
        assert browser.script('return document.getElementById("logs").closest("section")===document.getElementById("toggle").closest("section")')
        browser.script('document.querySelector("#preflight-details > summary").click();document.querySelector("#log-details > summary").click();return true')
        assert browser.script('return document.getElementById("preflight-results").getBoundingClientRect().height>0 && document.getElementById("logs").getBoundingClientRect().height>0')
        browser.script('document.querySelector("#preflight-details > summary").click();document.querySelector("#log-details > summary").click();return true')
        assert browser.script('return document.getElementById("save-path").textContent==="/simulated/PCD/priori"')
        browser.script('document.getElementById("mode").value="relocation";document.getElementById("mode").dispatchEvent(new Event("change",{bubbles:true}));return true')
        assert browser.script('return document.getElementById("save-path").textContent==="/simulated/PCD/relocate"')
        browser.script('document.getElementById("mode").value="mapping";document.getElementById("mode").dispatchEvent(new Event("change",{bubbles:true}));return true')
        if fault != 'none':
            assert browser.script('return document.getElementById("cloud-info").textContent.includes("injected")')
            browser.script('document.getElementById("cloud").dispatchEvent(new Event("webglcontextlost",{cancelable:true}));return true')
            assert browser.script('return document.getElementById("cloud-info").textContent.includes("上下文丢失")')
            browser.script('window.previewFault="none";document.getElementById("cloud").dispatchEvent(new Event("webglcontextrestored"));return true')
            browser.wait('return document.getElementById("cloud-info").textContent.includes("等待")')
        palette=browser.script('return ["--bg","--panel","--line","--text"].map(k=>getComputedStyle(document.documentElement).getPropertyValue(k).trim())')
        assert browser.script('return document.getElementById("toggle").getBoundingClientRect().height>=64')
        browser.call('WebDriver:SetWindowRect', {'width':390,'height':844})
        browser.wait('return document.documentElement.scrollWidth<=document.documentElement.clientWidth')
        browser.call('WebDriver:SetWindowRect', {'width':1440,'height':1000})
        assert browser.script('return document.getElementById("map").disabled')
        browser.script('document.getElementById("toggle").click();document.getElementById("toggle").click();return true')
        browser.wait('return document.getElementById("state").textContent==="已停止"')
        assert manager.starts==1
        browser.script('document.getElementById("toggle").click();return true')
        browser.wait('return document.getElementById("state").textContent==="运行中"')
        assert browser.script('return document.getElementById("configuration").disabled')
        manager.current.update(state='starting', initialization=dict(elapsed=5, timeout=60, checks=[
            dict(id='tracking', label='本次有效跟踪', status='pass'),
            dict(id='odom', label='本次新鲜且时钟有效的里程计', status='fail')]))
        browser.wait('return !document.getElementById("initialization").hidden')
        assert browser.script('return document.getElementById("initialization-time").textContent.includes("5.0s / 超时上限 60s")')
        assert browser.script('return document.getElementById("initialization-checks").textContent.includes("未满足")')
        manager.current['state'] = 'running'
        manager.current['health'] = dict(state='tracking_invalid', inputs=[], checks=[])
        browser.wait('return document.getElementById("health").textContent==="定位：跟踪无效"')
        assert browser.script('return document.getElementById("state").textContent==="运行中" && !document.getElementById("toggle").disabled')
        if fault == 'none':
            original_status = manager.status
            manager.status = lambda: {'state': 'invalid'}
            browser.wait('return document.getElementById("toggle").disabled && document.getElementById("last-confirmed").textContent.includes("过期")')
            assert browser.script('return document.getElementById("health").textContent==="定位：未知"')
            assert manager.current['session_id'] in browser.script('return document.getElementById("session").textContent')
            manager.status = original_status
            browser.script('document.getElementById("retry").click();return true')
            browser.wait('return document.getElementById("state").textContent==="运行中" && !document.getElementById("toggle").disabled')
            assert manager.starts == 2
        browser.call('WebDriver:Refresh')
        browser.wait('return document.getElementById("state").textContent==="运行中"')
        assert manager.starts==2
        browser.script('document.getElementById("clear-log").click();return true')
        browser.call('WebDriver:Navigate',{'url':'http://127.0.0.1:8080/'})
        browser.wait('return document.getElementById("slam-link").textContent==="SLAM 已连接"')
        browser.script('document.querySelector("[data-view=slam]").click();return true')
        assert browser.script('return document.getElementById("view-slam").classList.contains("active")')
        assert browser.script('return document.getElementById("slam-frame").src')=='http://127.0.0.1:8082/'
        assert palette==browser.script('return ["--bg","--panel","--line","--text"].map(k=>getComputedStyle(document.documentElement).getPropertyValue(k).trim())')
        browser.wait('return document.getElementById("slam-frame").style.height!=="1400px"')
        assert browser.script('return document.getElementById("slam-link").classList.contains("good")')
        # Exercise user-activated fullscreen from the actual cross-origin iframe.
        frame=browser.script('return document.getElementById("slam-frame")')
        browser.call('WebDriver:SwitchToFrame', {'element': frame})
        assert browser.script('return document.fullscreenEnabled')
        button=browser.script('window.fullscreenClicks=0;document.getElementById("cloud-fullscreen").addEventListener("click",()=>window.fullscreenClicks++);return document.getElementById("cloud-fullscreen")')
        browser.script('document.getElementById("cloud-fullscreen").scrollIntoView({block:"center"});return true')
        time.sleep(.3)
        browser.call('WebDriver:ElementClick', {'id': button['element-6066-11e4-a52e-4f735466cecf']})
        browser.wait('return document.fullscreenElement===document.getElementById("cloud-panel")')
        assert browser.script('return document.getElementById("cloud-fullscreen").textContent==="退出全屏"')
        assert browser.script('return document.getElementById("cloud").getBoundingClientRect().height>480')
        browser.script('document.getElementById("cloud-fullscreen").scrollIntoView({block:"center"});return true')
        time.sleep(.3)
        browser.call('WebDriver:ElementClick', {'id': button['element-6066-11e4-a52e-4f735466cecf']})
        browser.wait('return !document.fullscreenElement')
        browser.call('WebDriver:SwitchToFrame', {'id': None})
        assert manager.starts==2
        screenshot=browser.call('WebDriver:TakeScreenshot',{'full':True})['value']
        suffix='' if fault=='none' else f'_{fault}'
        output=ROOT/f'.runtime/slam_console_browser{suffix}.png';output.write_bytes(base64.b64decode(screenshot))
        browser.call('WebDriver:SetWindowRect', {'width':390,'height':844})
        browser.wait('return document.getElementById("slam-frame").getBoundingClientRect().right<=document.documentElement.clientWidth')
        screenshot=browser.call('WebDriver:TakeScreenshot',{'full':True})['value']
        (ROOT/f'.runtime/slam_console_browser_mobile{suffix}.png').write_bytes(base64.b64decode(screenshot))
    finally:
        if browser:browser.close()
        main.shutdown();main.server_close();httpthread.join(5)
        loop.call_soon_threadsafe(holder['stop'].set);thread.join(10);loop.close()


def test_position_display_live_initial_stale_reset_and_fullscreen(tmp_path):
    """Use a free port so the live robot's 8080/8082 services remain untouched."""
    manager = SimulatedManager()
    app = Server(manager, ['127.0.0.1'])
    loop = asyncio.new_event_loop()
    started, holder = threading.Event(), {}

    async def serving():
        async with serve(app.client, '127.0.0.1', 0, process_request=app.http) as server:
            holder['port'] = port = server.sockets[0].getsockname()[1]
            app.hosts = {f'127.0.0.1:{port}'}
            app.origins = {f'http://127.0.0.1:{port}'}
            holder['stop'] = asyncio.Event()
            started.set()
            await holder['stop'].wait()

    thread = threading.Thread(target=lambda: loop.run_until_complete(serving()), daemon=True)
    thread.start()
    assert started.wait(5)
    browser = None
    try:
        browser = Browser(tmp_path)
        browser.call('WebDriver:SetWindowRect', {'width':1440, 'height':1000})
        browser.call('WebDriver:Navigate', {'url': f'http://127.0.0.1:{holder["port"]}/'})
        browser.wait('return document.getElementById("toggle").textContent==="启动"')
        assert browser.script('return document.getElementById("position-x").textContent') == '—'
        manager.current = dict(session_id='mapping-one', state='running', mode='mapping', save=False,
            health={'state':'valid', 'inputs':[]}, position=dict(frame_id='map', fresh=True, age=.1,
            current={'xyz':[1.2345,-2.3456,3]}, initial={'xyz':[0,0,0]}))
        browser.wait('return document.getElementById("position-x").textContent==="1.234" && document.getElementById("initial-x").textContent==="0.000"')
        assert browser.script('return document.getElementById("position-status").textContent') == '实时定位'
        manager.current['position']['current'] = {'xyz':[4,5,6]}
        browser.wait('return document.getElementById("position-x").textContent==="4.000"')
        assert browser.script('return document.getElementById("initial-x").textContent') == '0.000'
        manager.current['position'].update(fresh=False, age=5)
        browser.wait('return document.getElementById("position-status").textContent.includes("数据过期")')
        manager.current = dict(session_id='relocation-two', state='starting', mode='relocation', save=False,
            position=dict(frame_id='site_map', fresh=False, age=None, current=None, initial=None))
        browser.wait('return document.getElementById("initial-source").textContent.includes("等待本次 GICP")')
        assert browser.script('return document.getElementById("initial-x").textContent') == '—'
        manager.current.update(state='running', health={'state':'valid', 'inputs':[]})
        manager.current['position'].update(fresh=True, age=.1,
            current={'xyz':[10,20,30]}, initial={'xyz':[9,19,29]})
        browser.wait('return document.getElementById("initial-x").textContent==="9.000"')
        browser.call('WebDriver:Refresh')
        browser.wait('return document.getElementById("position-x").textContent==="10.000"')
        assert browser.script('return document.getElementById("initial-x").textContent') == '9.000'
        assert browser.script('return document.getElementById("position-frame").textContent.includes("site_map")')
        original_status = manager.status
        manager.status = lambda: {'state':'invalid'}
        browser.wait('return document.getElementById("position-status").textContent.includes("状态未知")')
        assert browser.script('return document.getElementById("initial-x").textContent') == '9.000'
        manager.status = original_status
        browser.script('document.getElementById("retry").click();return true')
        browser.wait('return document.getElementById("position-status").textContent==="实时定位"')
        button = browser.script('document.getElementById("cloud-fullscreen").scrollIntoView({block:"center"});return document.getElementById("cloud-fullscreen")')
        browser.call('WebDriver:ElementClick', {'id':button['element-6066-11e4-a52e-4f735466cecf']})
        browser.wait('return document.fullscreenElement===document.getElementById("cloud-panel")')
        assert browser.script('return document.getElementById("cloud-panel").contains(document.querySelector(".position-strip"))')
        assert browser.script('return document.querySelector(".position-strip").getBoundingClientRect().bottom<=document.getElementById("cloud").getBoundingClientRect().top')
        browser.script('document.exitFullscreen();return true')
        browser.wait('return !document.fullscreenElement')
        manager.current['state'] = 'stopped'
        browser.wait('return document.getElementById("position-status").textContent.includes("任务已停止")')
        screenshot = browser.call('WebDriver:TakeScreenshot', {'full':True})['value']
        (ROOT/'.runtime/slam_console_positions.png').write_bytes(base64.b64decode(screenshot))
        browser.call('WebDriver:SetWindowRect', {'width':390, 'height':844})
        browser.wait('return document.documentElement.scrollWidth<=document.documentElement.clientWidth')
        assert manager.starts == 0
    finally:
        if browser:
            browser.close()
        loop.call_soon_threadsafe(holder['stop'].set)
        thread.join(10)
        loop.close()
