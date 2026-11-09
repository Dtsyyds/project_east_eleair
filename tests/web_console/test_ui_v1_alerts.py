"""UI V1: alarm provenance/retention and live long-text layout. No ROS/hardware."""
import asyncio
import base64
from pathlib import Path
import json
import os
import struct
import subprocess
import sys
import time
from urllib.request import Request, urlopen

import pytest
import test_visual_console as ui
import test_realtime_refinement as refinement

browser=ui.browser
OUT=ui.ROOT/'.runtime/s100p/validation/ui_v1'
LONG_ALARM=('隔离测试告警：相机同步检查需要操作员复核，当前回传的时间戳与参考帧关联尚未确认；'
            '请检查原始记录及设备连接，确认已读不表示故障恢复。'*4+
            ' DIAGNOSTIC_ID_'+ 'X'*100)


def _configure(app,mock):
    from aiohttp import web
    original_status=mock.status_payload
    async def emit(request):
        await mock.broadcast('status_update',await request.json())
        return web.json_response({'test_only':True})
    async def pause(_):
        if mock.video_task:
            mock.video_task.cancel()
            try: await mock.video_task
            except asyncio.CancelledError: pass
        caps=original_status()['capabilities'];caps['freshness_ms']['alerts']=750
        # Reconnect supplies capabilities only; it cannot manufacture recovery.
        mock.status_payload=lambda:{'capabilities':caps}
        await mock.broadcast('status_update',mock.status_payload())
        return web.json_response({'test_only':True})
    async def drop(_):
        await asyncio.gather(*(ws.close() for ws in tuple(mock.clients)))
        return web.json_response({'test_only':True})
    app.router.add_post('/ui-test/emit',emit)
    app.router.add_post('/ui-test/pause',pause)
    app.router.add_post('/ui-test/drop',drop)


@pytest.fixture
def server(tmp_path,browser):
    port=ui.helpers.free_port();base=f'http://127.0.0.1:{port}'
    with (tmp_path/'fixture.log').open('w') as log:
        process=subprocess.Popen([sys.executable,__file__,'--serve',str(port)],stdout=log,stderr=log)
        try:
            deadline=time.monotonic()+10
            while True:
                try: ui.get_json(base+'/mock/state');break
                except OSError:
                    assert process.poll() is None and time.monotonic()<deadline,(tmp_path/'fixture.log').read_text()
                    time.sleep(.05)
            yield base
            assert ui.get_json(base+'/mock/state')['chassis_requests']==0
        finally:
            browser.click('disconnect');process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill();process.wait(timeout=5)


def emit(base,payload=None,route='emit'):
    req=Request(base+'/ui-test/'+route,data=json.dumps(payload or {}).encode(),headers={'Content-Type':'application/json'},method='POST')
    with urlopen(req,timeout=5) as response: assert response.status==200


def capture(browser,name):
    OUT.mkdir(parents=True,exist_ok=True)
    data=base64.b64decode(browser.call('WebDriver:TakeScreenshot',{'full':False}))
    assert list(struct.unpack('>II',data[16:24]))==browser.js('return [innerWidth,innerHeight]')
    (OUT/(name+'.png')).write_bytes(data)


def alerts(browser):
    return browser.js("return document.getElementById('alert-list').textContent")


def reconnect(browser):
    browser.click('connect');browser.wait('window.tankConsole.snapshot().connected')


def test_alarm_knowledge_and_retention(browser,server):
    ui.open_console(browser,server)
    emit(server,route='pause')
    browser.wait("document.getElementById('alert-list').textContent.includes('状态待确认')")
    # Neither a connected socket nor an empty incremental update is all-clear.
    emit(server,{'alerts':[]})
    assert '状态待确认' in alerts(browser) and '当前无活动告警' not in alerts(browser)
    emit(server,{'alerts':[{'alert_id':'fixture-A','active':True,'severity':'error','message':'隔离测试：未恢复的告警 A'}]})
    browser.wait("document.getElementById('top-alert').textContent.includes('未恢复的告警 A')")
    browser.click('disconnect')
    assert '未恢复的告警 A' in alerts(browser) and '状态待确认' in alerts(browser)
    reconnect(browser)
    emit(server,{'alerts':[]})
    assert '未恢复的告警 A' in alerts(browser)
    emit(server,{'alerts':[{'alert_id':'fixture-A','active':False,'recovered':True,'valid':False}]})
    assert '未恢复的告警 A' in alerts(browser)  # Invalid recovery is not clearance.
    emit(server,{'alerts':[{'alert_id':'fixture-A','active':False,'recovered':True,'valid':True}]})
    browser.wait("document.getElementById('alert-list').textContent.includes('已收告警已确认恢复')")
    assert '整体状态待确认' in alerts(browser)
    browser.wait("!document.getElementById('alert-list').textContent.includes('已收告警已确认恢复')",timeout=3)
    assert '状态待确认' in alerts(browser)  # Old recovery evidence cannot confirm current state.
    # Local warnings must survive automatic connection resets as well as manual disconnect.
    emit(server,{'storage':{'mounted':False,'valid':True},'tracking_result':{'status':'LOST','reason':'隔离测试巡迹丢失'}})
    browser.wait("document.getElementById('alert-list').textContent.includes('采集盘未挂载')")
    browser.click('disconnect');reconnect(browser)
    assert '采集盘未挂载' in alerts(browser) and '隔离测试巡迹丢失' in alerts(browser)
    connections=browser.js('return window.tankConsole.snapshot().connection_count')
    emit(server,route='drop')
    browser.wait(f'window.tankConsole.snapshot().connected && window.tankConsole.snapshot().connection_count>{connections}')
    assert '采集盘未挂载' in alerts(browser) and '隔离测试巡迹丢失' in alerts(browser)
    emit(server,{'storage':{'mounted':True,'valid':False},'tracking_result':{'status':'VISION_2D','valid':False}})
    browser.wait('window.tankConsole.snapshot().tracking?.valid===false')
    assert '采集盘未挂载' in alerts(browser) and '隔离测试巡迹丢失' in alerts(browser)
    emit(server,{'storage':{'mounted':True,'valid':True},'tracking_result':{'status':'VISION_2D','valid':True}})
    browser.wait("document.querySelectorAll('#alert-list .alert-item').length===0")
    assert '状态待确认' in alerts(browser)
    browser.click('disconnect')
    assert '连接已断开' in alerts(browser)
    assert ui.get_json(server+'/mock/state')['chassis_requests']==0


def geometry(browser):
    values=browser.js("""
      return ['mindvision','fisheye','depth_camera'].map(id=>{
        const c=document.getElementById(id+'-canvas'),o=document.getElementById(id+'-overlay');
        const a=c.getBoundingClientRect(),b=o.getBoundingClientRect(),p=c.parentElement.getBoundingClientRect();
        return {id,source:[c.width,c.height],overlay:[o.width,o.height],
          rect:[a.x,a.y,a.width,a.height],overlayRect:[b.x,b.y,b.width,b.height],
          centerPixel:[...o.getContext('2d').getImageData(Math.round(31*c.width/64),Math.floor(c.height/2),1,1).data],
          outsideAlpha:o.getContext('2d').getImageData(2,2,1,1).data[3],
          contained:a.left>=p.left&&a.right<=p.right&&a.top>=p.top&&a.bottom<=p.bottom,
          match:window.tankConsole.snapshot().streams[id].overlay_match};});
    """)
    for value in values:
        assert value['source']==value['overlay'] and value['rect']==value['overlayRect']
        assert value['contained'] and value['match']=='matched'
        # D405 uses a red centerline; the other previews retain green.
        color = [255,64,64] if value['id'] == 'depth_camera' else [112,221,177]
        assert all(abs(a-b)<=1 for a,b in zip(value['centerPixel'][:3],color))
        assert value['centerPixel'][3]==255 and value['outsideAlpha']==0
        assert abs(value['rect'][2]/value['rect'][3]-value['source'][0]/value['source'][1])<.005
    return values


def reveal(browser,selector):
    # Scroll under the sticky operations strip; verify the target is actually clickable.
    result=browser.js("""
      const e=document.querySelector(arguments[0]),dock=document.querySelector('.operations-dock');
      const top=getComputedStyle(dock).position==='sticky'?dock.getBoundingClientRect().bottom:0;
      window.scrollBy(0,e.getBoundingClientRect().top-top-12);
      const r=e.getBoundingClientRect(),y=Math.min(innerHeight-8,r.bottom-2),x=r.x+r.width/2;
      return {top:r.top,bottom:r.bottom,dock:top,hit:e.contains(document.elementFromPoint(x,y))};
    """,selector)
    assert result['top']>=result['dock'] and result['bottom']<=browser.js('return innerHeight') and result['hit'],result


def test_live_long_alarm_layout(browser,server):
    ui.open_console(browser,server)
    browser.wait("['mindvision','fisheye','depth_camera'].every(id=>window.tankConsole.snapshot().streams[id].displayed>3)")
    report=[]
    for width,height in [(1366,768),(1920,1080)]:
        refinement.viewport(browser,width,height);browser.js('window.scrollTo(0,0)')
        emit(server,{'alerts':[{'alert_id':'long-test','active':True,'severity':'error','message':LONG_ALARM,'freshness_ms':300000}]})
        browser.wait("document.querySelector('#alert-list .alert-item')!==null")
        ui.assert_layout(browser)
        assert LONG_ALARM in alerts(browser)
        assert browser.js("return document.querySelector('#alert-list .alert-item').scrollWidth<=document.querySelector('#alert-list .alert-item').clientWidth+2")
        frames=browser.js('return window.tankConsole.snapshot().streams.mindvision.displayed')
        browser.wait(f'window.tankConsole.snapshot().streams.mindvision.displayed>{frames+3}')
        view_report={'viewport':[width,height],'geometry':geometry(browser),'text':alerts(browser),'fullscreen':[]}
        report.append(view_report)
        capture(browser,f'live-long-alarm-{width}x{height}')
        for camera in ['mindvision','fisheye','depth_camera']:
            reveal(browser,f'[data-fullscreen="{camera}"]')
            ui.click(browser,f'[data-fullscreen="{camera}"]')
            browser.wait(f"document.fullscreenElement?.id==='{camera}-screen'")
            browser.wait('innerWidth===document.fullscreenElement.clientWidth && innerHeight===document.fullscreenElement.clientHeight')
            view_report['fullscreen'].append({'camera':camera,'actual_viewport':browser.js('return [innerWidth,innerHeight]'),'geometry':geometry(browser)})
            capture(browser,f'fullscreen-from-{width}x{height}-{camera}')
            browser.js('document.exitFullscreen()');browser.wait('document.fullscreenElement===null')
            geometry(browser)
        for camera in ['fisheye','depth_camera']:
            reveal(browser,f'[data-overlay="{camera}"]')
            ui.click(browser,f'[data-overlay="{camera}"]')
            assert browser.js('return document.getElementById(arguments[0]+"-screen").classList.contains("raw-only")',camera)
            ui.click(browser,f'[data-overlay="{camera}"]')
            reveal(browser,f'#{camera}-screen')
            capture(browser,f'scrolled-{camera}-{width}')
        for selector,name in [('#trajectory','trajectory'),('#eddy-chart','eddy')]:
            reveal(browser,selector)
            if name=='eddy':
                assert browser.js("""const c=document.getElementById('eddy-chart');
                  const pixels=c.getContext('2d').getImageData(0,0,c.width,c.height).data;
                  return pixels.some((value,i)=>i%4===3 && value>0);"""), 'Received eddy samples must produce visible marks, including short/constant series'
            capture(browser,f'scrolled-{name}-{width}')
        pose=browser.js("return document.getElementById('pose-x').textContent")
        browser.wait('document.getElementById("pose-x").textContent!=='+json.dumps(pose))
        assert browser.js("return document.getElementById('eddy-legend').childElementCount")>0
        emit(server,{'alerts':[{'alert_id':'long-test','active':True,'severity':'error','message':'隔离测试：告警文案已更新，尚未恢复'}]})
        browser.wait("document.getElementById('alert-list').textContent.includes('告警文案已更新')")
        assert LONG_ALARM not in alerts(browser)
        ui.assert_layout(browser)
    assert not browser.js('return window.uiErrors')
    (OUT/'live-layout.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))


def test_disconnected_v1_comparison(browser):
    sources={'after':ui.ROOT/'apps/web_console'}
    if os.environ.get('UI_V1_BASELINE_DIR'): sources={'before':Path(os.environ['UI_V1_BASELINE_DIR']),**sources}
    report={}
    for phase,path in sources.items():
        with refinement.static_server(path) as base:
            for width,height in [(1366,768),(1920,1080)]:
                refinement.viewport(browser,width,height)
                browser.open(base+'/?ws='+base.replace('http:', 'ws:')+'/no-websocket');browser.click('disconnect')
                browser.js('window.scrollTo(0,0);window.uiErrors=[]')
                capture(browser,f'{phase}-disconnected-{width}x{height}')
                data=refinement.measurements(browser);data['alert_text']=alerts(browser)
                report[f'{phase}-{width}']=data
                if phase=='after': assert '状态待确认' in alerts(browser)
    (OUT/'v1-comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))


def test_constant_eddy_comparison(browser,server):
    # Identical two-point data, paused fixture, fresh page: no random/live timing differences.
    emit(server,route='pause')
    sources={'after':ui.ROOT/'apps/web_console'}
    if os.environ.get('UI_V1_BASELINE_DIR'): sources={'before':Path(os.environ['UI_V1_BASELINE_DIR']),**sources}
    sample={'eddy_mean':{'channel_mean':[100,105],'unit':'UI_TEST_raw_count','stamp_ns':'1800000000125000000'}}
    report=[]
    for phase,path in sources.items():
        with refinement.static_server(path) as base:
            for width,height in [(1366,768),(1920,1080)]:
                refinement.viewport(browser,width,height)
                browser.open(base+'/?ws='+server.replace('http:', 'ws:')+'/ws')
                browser.wait('window.tankConsole.snapshot().connected')
                emit(server,sample);emit(server,sample)
                browser.wait("document.getElementById('eddy-meta').textContent.includes('UI_TEST_raw_count')")
                reveal(browser,'#eddy-chart')
                ink=browser.js("""const c=document.getElementById('eddy-chart');
                  return c.getContext('2d').getImageData(0,0,c.width,c.height).data.some((value,i)=>i%4===3&&value>0);""")
                assert ink==(phase=='after')
                capture(browser,f'{phase}-constant-eddy-{width}x{height}')
                report.append({'phase':phase,'viewport':[width,height],'sample':sample,'sample_count':2,'visible_marks':ink})
                browser.click('disconnect')
    (OUT/'constant-eddy.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__' and len(sys.argv)==3 and sys.argv[1]=='--serve':
    refinement._serve_geometry_fixture(int(sys.argv[2]),_configure)
