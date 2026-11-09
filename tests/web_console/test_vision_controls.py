"""Firefox workflow against loopback-only experimental controls, no ROS/hardware."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest
import test_visual_console as ui
import test_realtime_refinement as fixed

browser=ui.browser
OUT=Path(os.environ.get('D405_UI_TEST_OUTPUT', ui.ROOT/'.runtime/s100p/validation/vision_controls/browser'))


def serve(port):
    from aiohttp import web
    from mock_console_server import MockConsole, make_app
    class VisionMock(MockConsole):
        def __init__(self):
            super().__init__();self.vision={'status':'idle','diagnostics_fresh':True,'busy':False,'saved_count':0}
            self.actions=[];self.revision=0;self.candidate_sets=[];self.selection_requests=[];self.selected=None
        @property
        def capabilities(self):
            value=super().capabilities
            for name in ['vision.start','vision.stop','vision.save_sample','vision.status','vision.select_candidate','vision.clear_selection']:
                value['operations'][name]=True;value['permissions'][name]=True
            return value
        def status_payload(self):
            self.revision+=1
            return dict(super().status_payload(),vision=dict(self.vision,revision=self.revision,service_id='isolated-ui-test'))
        def tracking(self,source,frame_id,stamp_ns):
            value=super().tracking(source,frame_id,stamp_ns)
            value.update(session_id='mock-vision',selection={'state':'TRACKING' if self.selected else 'AUTO'},candidates=[])
            if self.scenario=='lost' and self.vision['status']=='running':
                if self.selected:
                    value.update(status='CENTER_2D',vision_2d_valid=True,left_boundary=[],right_boundary=[],
                                 centerline=[[31,5],[31,42]])
                else:
                    set_id='mock-set-'+str(self.frame)
                    self.candidate_sets=(self.candidate_sets+[set_id])[-32:]
                    value.update(candidate_set_id=set_id,candidates=[dict(candidate_id=str(i+1),rank=i+1,score=.45-i*.05,
                        coverage=.8,confidence=.4,centerline=[[x,5],[x,42]]) for i,x in enumerate([14,31,49])])
            return value
        async def handle_action(self,ws,payload):
            action=payload.get('action','')
            if not action.startswith('vision.'):return await super().handle_action(ws,payload)
            self.actions.append(action)
            reply={'action':action,'request_id':payload['request_id']}
            if action!='vision.status':
                self.vision['busy']=True
                await self.packet(ws,'response',dict(reply,status='accepted',**self.status_payload()))
                await asyncio.sleep(.15)
            if action=='vision.start':
                self.vision.update(status='running',session_id='mock-vision',directory='MOCK_ONLY/vision-experiment',
                    diagnostics={'candidate_selection_supported':True,'selection':{'state':'AUTO'},'tracking_error_reference': {'image_width':640,'image_height':480,
                        'reference_x':320.,'reference_y':360.,'reference_dx_dy':0.,
                        'heading_y_min':336.,'heading_y_max':384.}})
            elif action=='vision.stop':self.vision.update(status='stopped')
            elif action=='vision.select_candidate':
                p=payload['parameters'];self.selection_requests.append(p)
                assert p['session_id']=='mock-vision' and p['candidate_set_id'] in self.candidate_sets
                assert p['candidate_id'] in ('1','2','3')
                self.selected=p
                self.vision['diagnostics']['selection']={'state':'TRACKING',**p}
            elif action=='vision.clear_selection':
                self.selected=None;self.vision['diagnostics']['selection']={'state':'AUTO'}
            elif action=='vision.save_sample':
                self.vision.update(saved_count=self.vision['saved_count']+1,sample={'status':'completed','message':'隔离测试样本完成（非硬件证据）','path':'MOCK_ONLY/sample/raw.npz'})
            self.vision['busy']=False
            await self.packet(ws,'response',dict(reply,status='completed',**self.status_payload()))
        async def inspect(self,request):
            return web.json_response({'chassis_requests':self.chassis_requests,'actions':self.actions,'task':self.task,'vision':self.vision,
                                      'selection_requests':self.selection_requests})
    web.run_app(make_app(VisionMock()),host='127.0.0.1',port=port,print=None)


@pytest.fixture
def server(browser,tmp_path):
    base=f'http://127.0.0.1:{ui.helpers.free_port()}'
    with (tmp_path/'mock.log').open('w') as log:
        proc=subprocess.Popen([sys.executable,__file__,'--serve',base.rsplit(':',1)[1]],stdout=log,stderr=log)
        try:
            deadline=time.monotonic()+10
            while True:
                try:ui.get_json(base+'/mock/state');break
                except OSError:
                    assert proc.poll() is None and time.monotonic()<deadline
                    time.sleep(.05)
            yield base
            assert ui.get_json(base+'/mock/state')['chassis_requests']==0
        finally:
            browser.click('disconnect');proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.kill();proc.wait()


def test_three_controls_and_recording_independence(browser,server):
    fixed.OUT=OUT;ui.open_console(browser,server)
    assert browser.js("return window.tankConsole.snapshot().task?.status") != 'recording'
    for w,h in [(1366,768),(1920,1080)]:
        fixed.viewport(browser,w,h);ui.assert_layout(browser)
        browser.wait("!document.getElementById('vision-start').disabled")
        assert browser.js("return document.getElementById('vision-save').disabled")
        ui.click(browser,'#vision-start')
        browser.wait("window.VisionControls.snapshot()?.status==='running'")
        assert browser.js("return window.tankConsole.snapshot().task?.status") != 'recording'
        ui.click(browser,'#vision-save')
        browser.wait("window.VisionControls.snapshot()?.sample?.status==='completed'")
        browser.wait("!document.getElementById('vision-save').disabled")
        browser.js('window.scrollTo(0,0)')
        fixed.screenshot(browser,f'controls-running-{w}x{h}')
        ui.assert_layout(browser)
        # Start a continuous mock recording, then stop only visual tracking.
        ui.select_view(browser,'collection');ui.click(browser,'#preflight')
        browser.wait("window.tankConsole.snapshot().task?.status==='ready'")
        ui.click(browser,'#start-recording')
        browser.wait("window.tankConsole.snapshot().task?.status==='recording'")
        ui.select_view(browser,'realtime');ui.click(browser,'#vision-stop')
        browser.wait("window.VisionControls.snapshot()?.status==='stopped'")
        assert browser.js("return window.tankConsole.snapshot().task.status")=='recording'
        ui.click(browser,'#quick-stop');browser.wait("window.tankConsole.snapshot().task?.status==='complete'")
    ui.click(browser,'#vision-start');browser.wait("window.VisionControls.snapshot()?.status==='running'")
    before=ui.get_json(server+'/mock/state')['actions'].count('vision.start')
    browser.click('disconnect')
    assert browser.js("return ['vision-start','vision-stop','vision-save'].every(id=>document.getElementById(id).disabled)")
    browser.click('connect');browser.wait("window.tankConsole.snapshot().connected && window.VisionControls.snapshot()?.status==='running'")
    assert ui.get_json(server+'/mock/state')['actions'].count('vision.start')==before
    assert not browser.js('return window.uiErrors')
    (OUT/'workflow.json').write_text(json.dumps(ui.get_json(server+'/mock/state'),ensure_ascii=False,indent=2))


def test_reference_uses_active_config_and_survives_lost_measurement(browser,server):
    ui.open_console(browser,server)
    browser.wait("!document.getElementById('vision-start').disabled")
    browser.click('vision-start')
    browser.wait("document.getElementById('depth_camera-reference-display').textContent.includes('(320, 360)')")
    browser.js("document.getElementById('depth_camera-screen').scrollIntoView({block:'center'})")
    fixed.OUT=OUT;fixed.screenshot(browser,'reference-overlay')
    # 640x480 config is mapped onto the bridge's 64x48 fixture preview.
    pixel="return [...document.getElementById('depth_camera-reference').getContext('2d').getImageData(32,36,1,1).data]"
    assert browser.js(pixel)[3]>0
    ui.scenario(server,'lost')
    browser.wait("window.tankConsole.snapshot().tracking?.status==='LOST'")
    assert browser.js(pixel)[3]>0
    # A changed active reference must move and rotate the line, without using
    # hard-coded defaults or changing the detected seam overlay.
    result=browser.js("""
      const value=structuredClone(window.VisionControls.snapshot());value.revision+=100;
      Object.assign(value.diagnostics.tracking_error_reference,{reference_x:480,reference_dx_dy:.5});
      window.VisionControls.receive({payload:{vision:value}});
      window.VisionControls.renderReference({width:64,height:48},false,true);
      const c=document.getElementById('depth_camera-reference'),ctx=c.getContext('2d');
      return {text:document.getElementById('depth_camera-reference-display').textContent,
        target:[...ctx.getImageData(48,36,1,1).data],
        line:[...ctx.getImageData(38,16,1,1).data],
        old:[...ctx.getImageData(32,16,1,1).data]};
    """)
    assert '(480, 360)' in result['text'] and '26.6°' in result['text']
    assert result['target'][3]>0 and result['line'][3]>0 and result['old'][3]==0
    assert browser.js("""
      window.VisionControls.renderReference({width:64,height:48},false,false);
      return [...document.getElementById('depth_camera-reference').getContext('2d').getImageData(0,0,64,48).data].every(v=>v===0);
    """)
    assert browser.js("""
      window.VisionControls.renderReference({width:64,height:48},true,true);
      return [...document.getElementById('depth_camera-reference').getContext('2d').getImageData(0,0,64,48).data].every(v=>v===0);
    """)
    browser.click('disconnect')
    assert browser.js(pixel)[3]==0
    assert not browser.js('return window.uiErrors')


def test_lost_candidates_select_exact_displayed_frame_and_resume(browser,server):
    ui.open_console(browser,server)
    browser.wait("!document.getElementById('vision-start').disabled")
    browser.click('vision-start')
    browser.wait("window.VisionControls.snapshot()?.status==='running'")
    ui.scenario(server,'lost')
    browser.wait("!document.getElementById('vision-candidate-2').disabled")
    assert browser.js("return [1,2,3].every(i=>!document.getElementById('vision-candidate-'+i).hidden)")
    assert browser.js("return document.getElementById('depth_camera-overlay').getContext('2d').getImageData(14,8,1,1).data[3]")>0
    ui.click(browser,'#vision-candidate-2')
    browser.wait("window.tankConsole.snapshot().tracking?.selection?.state==='TRACKING'")
    data=ui.get_json(server+'/mock/state')
    assert len(data['selection_requests'])==1 and data['selection_requests'][0]['candidate_id']=='2'
    assert data['selection_requests'][0]['candidate_set_id'].startswith('mock-set-')
    browser.wait("document.getElementById('vision-candidate-message').textContent.includes('正在沿选中目标跟踪')")
    assert browser.js("return window.tankConsole.snapshot().tracking.vision_2d_valid")
    browser.wait("!document.getElementById('vision-clear-selection').disabled")
    ui.click(browser,'#vision-clear-selection')
    browser.wait("!document.getElementById('vision-candidate-1').disabled")
    # Hidden or unmatched candidate drawings must not remain selectable.
    browser.js("document.querySelector('[data-overlay=depth_camera]').click()")
    browser.wait("document.getElementById('vision-candidate-1').disabled")
    assert not browser.js('return window.uiErrors')


if __name__=='__main__' and len(sys.argv)==3:serve(int(sys.argv[2]))
