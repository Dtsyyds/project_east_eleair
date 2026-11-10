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
        (profile/'user.js').write_text(f'user_pref("marionette.port", {port});\nuser_pref("browser.shell.checkDefaultBrowser",false);\nuser_pref("browser.startup.homepage_override.mstone","ignore");\n')
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
        raise AssertionError('Browser condition timeout: '+code)
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
    def status(self):return dict(self.current)
    def maps(self):return [{'name':'prior.pcd','path':'/simulated/maps/prior.pcd'}]
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


def test_direct_page_quick_cancel_refresh_and_iframe(tmp_path):
    manager=SimulatedManager();app=Server(manager,['127.0.0.1']);loop=asyncio.new_event_loop();started=threading.Event();holder={}
    async def serving():
        async with serve(app.client,'127.0.0.1',8082,process_request=app.http):
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
        assert manager.starts==2
        screenshot=browser.call('WebDriver:TakeScreenshot',{'full':True})['value']
        output=ROOT/'.runtime/slam_console_browser.png';output.write_bytes(base64.b64decode(screenshot))
        browser.call('WebDriver:SetWindowRect', {'width':390,'height':844})
        browser.wait('return document.getElementById("slam-frame").getBoundingClientRect().right<=document.documentElement.clientWidth')
        screenshot=browser.call('WebDriver:TakeScreenshot',{'full':True})['value']
        (ROOT/'.runtime/slam_console_browser_mobile.png').write_bytes(base64.b64decode(screenshot))
    finally:
        if browser:browser.close()
        main.shutdown();main.server_close();httpthread.join(5)
        loop.call_soon_threadsafe(holder['stop'].set);thread.join(10);loop.close()
