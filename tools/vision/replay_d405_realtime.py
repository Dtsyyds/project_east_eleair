#!/usr/bin/env python3
"""Disk RGB -> single network -> bounded latest-frame queue -> real WebSocket UI.
No ROS imports, camera interfaces, movement transport, or cached predictions as input.
"""
import argparse, asyncio, base64, csv, hashlib, json, signal, sys, time, uuid
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread
from urllib.parse import urlsplit
import cv2
import numpy as np
import torch
import yaml
from websockets.asyncio.server import serve

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'packages/d405_seam_tracking/src'))
from d405_seam_tracking.round1_a import Round1AVisual
from d405_seam_tracking.learning import preprocess
from d405_seam_tracking.geometry import roi_bounds
from d405_seam_tracking.center_2d import decode
from d405_seam_tracking.inference_backend import CPUInference
ANA = ROOT/'.runtime/s100p/analysis'
SESSIONS = ['WS-20261102_214559-d94ac71b', 'WS-20261102_214902-f634c958']
BASE = ANA/'d405_midline_residual_trial_20261103'

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def now(): return time.perf_counter_ns()
def stats(values):
    return {'p50_ms': float(np.percentile(values,50)), 'p95_ms': float(np.percentile(values,95))} if values else None

class Replay:
    def __init__(self, args):
        self.args = args
        self.config = yaml.safe_load((ROOT/'config/d405/round1_a.yaml').read_text())
        self.bounds = roi_bounds((480,640,3),self.config)
        assert self.bounds == (128,168,512,456)
        assert self.config['min_coverage']==.55 and self.config['probability_threshold']==.55
        assert self.config['path_max_gap_rows']==2 and self.config['max_fit_residual_pixels']==5
        cv2.setNumThreads(2); torch.set_num_threads(2); torch.use_deterministic_algorithms(True)
        self.visual = Round1AVisual(self.config)
        self.inference = CPUInference(self.visual.model, args.inference_backend)
        for _ in range(8):
            warm,_=preprocess(np.zeros((480,640,3),np.uint8),self.visual.size,rgb=True)
            self.inference.probability(warm)
        self.clients = {}; self.running=False; self.stop_requested=False; self.active=None
        self.last_output=None; self.task=None; self.current_run=None; self.history={}

    def infer(self, item):
        t0=now(); x,_=preprocess(item['rgb'],self.visual.size,rgb=True); t1=now()
        p=self.inference.probability(x)
        t2=now(); p=cv2.resize(p,(640,480),interpolation=cv2.INTER_LINEAR)
        result=decode(p,self.config,self.bounds,enabled=self.args.center2d); t3=now()
        ok,jpg=cv2.imencode('.jpg',cv2.cvtColor(item['rgb'],cv2.COLOR_RGB2BGR),[cv2.IMWRITE_JPEG_QUALITY,90]); assert ok
        return result, {'preprocess_ms':(t1-t0)/1e6,'inference_ms':(t2-t1)/1e6,'postprocess_ms':(t3-t2)/1e6,'total_compute_ms':(t3-t0)/1e6,'jpeg_ms':(now()-t3)/1e6}, base64.b64encode(jpg).decode()

    def broadcast(self,msg):
        for c in self.clients.values():
            if c['queue'].full(): c['queue'].get_nowait(); c['replaced']+=1
            c['queue'].put_nowait(msg)

    async def websocket(self,ws):
        cid=uuid.uuid4().hex[:10]
        c={'queue':asyncio.Queue(1),'replaced':0,'peer':str(ws.remote_address),'display_acks':[]}
        self.clients[cid]=c
        async def sender():
            while True:
                msg=await c['queue'].get()
                await asyncio.wait_for(ws.send(json.dumps(msg,separators=(',',':'))),timeout=1)
        send=asyncio.create_task(sender())
        try:
            async for raw in ws:
                d=json.loads(raw)
                if d.get('type')!='paint_ack': continue
                run=self.history.get(d.get('run_id'))
                if not run: continue
                record=run['lookup'].get((d.get('session'),d.get('frame')))
                if not record or str(d.get('stamp_ns'))!=record['stamp_ns']:continue
                ack={'client_id':cid,'peer':c['peer'],'board_ack_ns':now(), **d}
                run['acks'].append(ack)
                with (run['out']/'browser_paint_acks.jsonl').open('a') as f:f.write(json.dumps(ack)+'\n')
        finally:
            send.cancel(); self.clients.pop(cid,None)

    def load(self, session):
        audit=ANA/'d405_video_ab_20261102/audit'/session
        frames=json.loads((audit/'frames.json').read_text())
        frames=sorted([f for f in frames if session==SESSIONS[1] or 1090<=int(f['frame'])<=1495],key=lambda f:int(f['stamp_ns']))
        raw=[]
        for f in frames:
            rgb=cv2.cvtColor(cv2.imread(str(audit/'rgb'/f'{int(f["frame"]):06d}.png')),cv2.COLOR_BGR2RGB)
            assert hashlib.sha256(rgb.tobytes()).hexdigest()==f['rgb_sha256']
            raw.append(dict(f,rgb=rgb))
        baseline=BASE/('per_frame.csv' if session==SESSIONS[0] else 'second_session_214902/per_frame.csv')
        with baseline.open(encoding='utf-8-sig') as f: expected={int(r['frame']):r for r in csv.DictReader(f)}
        assert len(expected)==len(raw)
        return raw,expected

    async def clip(self, session, run):
        frames,expected=await asyncio.to_thread(self.load,session)
        q=asyncio.Queue(1); injected=[]; records=[]; overwritten=[]; injection_done=False
        source0=int(frames[0]['stamp_ns']); clock0=now(); self.active=session
        self.broadcast({'type':'begin','run_id':run['id'],'session':session,'center2d':self.args.center2d})
        async def producer():
            nonlocal injection_done
            for item in frames:
                target=clock0+int(item['stamp_ns'])-source0
                await asyncio.sleep(max(0,(target-now())/1e9))
                if self.stop_requested:break
                item['injected_ns']=now();item['target_ns']=target
                injected.append({k:v for k,v in item.items() if k!='rgb'})
                if q.full():old=q.get_nowait();overwritten.append(int(old['frame']))
                q.put_nowait(item)
            injection_done=True
        async def worker():
            while not injection_done or not q.empty():
                try:item=await asyncio.wait_for(q.get(),.1)
                except asyncio.TimeoutError:continue
                started=now(); result,timing,jpg=await asyncio.to_thread(self.infer,item)
                finished=now(); f=int(item['frame']); ref=expected[f]
                assert str(item['stamp_ns'])==ref['stamp_ns'] and item['rgb_sha256']==ref['rgb_sha256']
                a=result['A']; actual=np.asarray(result['center_points']).reshape(-1,2)
                if session==SESSIONS[0]:target_points=json.loads(ref['trial_center_points'])
                elif ref['trial_kind']=='A_UNCHANGED': target_points=[[p[0],(p[1]+p[2])/2] for p in json.loads(ref['A_points'])]
                else:target_points=json.loads(ref['points'])
                # Disabled mode compares A; for a valid A its geometry is unchanged.
                target_kind=ref['trial_kind'] if self.args.center2d else ('A_UNCHANGED' if ref['A_valid']=='True' else 'LOST')
                if not self.args.center2d and target_kind=='LOST':target_points=[]
                target_points=np.asarray(target_points).reshape(-1,2)
                delta=float(np.max(np.abs(actual-target_points))) if actual.size and actual.shape==target_points.shape else (0. if actual.shape==target_points.shape else None)
                same=(a['valid']==(ref['A_valid']=='True') and ';'.join(a['reasons'])==ref['A_reason'] and result['kind']==target_kind and delta is not None and delta<=1e-6)
                row={'session':session,'frame':f,'stamp_ns':str(item['stamp_ns']),'rgb_sha256':item['rgb_sha256'],
                     'injected_ns':item['injected_ns'],'started_ns':started,'finished_ns':finished,'queue_wait_ms':(started-item['injected_ns'])/1e6,
                     'board_inject_to_publish_ms':(finished-item['injected_ns'])/1e6,'injection_lateness_ms':(item['injected_ns']-item['target_ns'])/1e6,
                     'kind':result['kind'],'A_valid':a['valid'],'A_reason':';'.join(a['reasons']),'reason':result['reason'],
                     'center_points':result['center_points'],'A_points':a.get('points',[]),'baseline_match':same,'max_center_delta_px':delta,**timing}
                records.append(row);run['lookup'][(session,f)]=row
                with (run['out']/'processed.jsonl').open('a') as fp:fp.write(json.dumps(row)+'\n')
                self.broadcast({'type':'result','run_id':run['id'],'session':session,'frame':f,'stamp_ns':row['stamp_ns'],
                                'kind':result['kind'],'reason':result['reason'],'center_points':result['center_points'],
                                'A_points':a.get('points',[]),'roi':list(self.bounds),'jpeg':jpg})
                if not same:
                    self.stop_requested=True
                    raise RuntimeError('offline parity mismatch '+session+' #'+str(f))
        await asyncio.gather(producer(),worker())
        end=now()
        run['clips'][session]={'injected':injected,'records':records,'overwritten':overwritten,'elapsed_s':(end-clock0)/1e9,
            'source_span_s':(int(frames[-1]['stamp_ns'])-source0)/1e9,'input_queue_peak':1 if injected else 0,'inflight_peak':1 if records else 0}
        with (run['out']/(session+'_input.json')).open('w') as fp:json.dump({'injected':injected,'overwritten_frames':overwritten,'not_injected':[int(f['frame']) for f in frames if 'injected_ns' not in f]},fp)
        self.broadcast({'type':'end','run_id':run['id'],'session':session})
        self.summarize(run)
        print(json.dumps(self.summarize(run)['sessions'][-1],ensure_ascii=False),flush=True)

    def summarize(self,run):
        summary={'run_id':run['id'],'inference_backend':self.args.inference_backend,'center2d':self.args.center2d,'clock_basis':'injection monotonic; source timestamp only for scheduling and identity; no PC clock synchronization',
                 'PC_measurement':'visible browser canvas paint acknowledgements; board ACK latency includes return path and is not one-way display latency','sessions':[]}
        for session,c in run['clips'].items():
            rs=c['records']; inj=c['injected'];acks=[a for a in run['acks'] if a['session']==session and a.get('visible')]
            clients={}
            for a in acks:clients.setdefault(a['client_id'],{})[a['frame']]=a
            display=[]
            for cid,byframe in clients.items():
                aa=list(byframe.values());times=[a['paint_ms'] for a in aa]
                display.append({'client_id':cid,'peer':aa[0]['peer'],'unique_frames':len(aa),'fps_observed_window':(len(aa)-1)*1000/(max(times)-min(times)) if len(aa)>1 else None,
                    'receive_to_paint':stats([a['receive_to_paint_ms'] for a in aa]),
                    'board_inject_to_ack_roundtrip_upper_bound':stats([(a['board_ack_ns']-run['lookup'][(session,a['frame'])]['injected_ns'])/1e6 for a in aa])})
            def fps(ts):return (len(ts)-1)*1e9/(ts[-1]-ts[0]) if len(ts)>1 else None
            summary['sessions'].append({'session':session,'input_frames':len(inj),'input_fps':fps([x['injected_ns'] for x in inj]),
                'processed_frames':len(rs),'processed_fps':fps([r['finished_ns'] for r in rs]),'source_span_s':c['source_span_s'],'run_elapsed_s':c['elapsed_s'],
                'overwritten_unprocessed_frames':len(c['overwritten']),'processed_no_measurement':sum(r['kind']=='LOST' for r in rs),
                'A_complete':sum(r['kind']=='A_UNCHANGED' for r in rs),'CENTER_2D':sum(r['kind']=='CENTER_2D' for r in rs),
                'queue_peak':c['input_queue_peak'],'inflight_peak':c['inflight_peak'],'baseline_matched':sum(r['baseline_match'] for r in rs),
                'max_center_delta_px':max((r['max_center_delta_px'] or 0 for r in rs),default=0),
                'timing':{k:stats([r[k] for r in rs]) for k in ['preprocess_ms','inference_ms','postprocess_ms','total_compute_ms','jpeg_ms','queue_wait_ms','board_inject_to_publish_ms','injection_lateness_ms']},
                'PC_display':display or None,'PC_one_way_total_display_latency':None})
        (run['out']/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        return summary

    async def start(self):
        if self.running: return {'error':'replay already running'}
        self.running=True;self.stop_requested=False
        ident=time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
        out=self.args.output/ident;out.mkdir(parents=True)
        run={'id':ident,'out':out,'clips':{},'acks':[],'lookup':{}};self.history[ident]=run;self.current_run=run
        manifest={'inference_backend':self.args.inference_backend,'center2d_enabled':self.args.center2d,'queue_policy':'one in flight plus latest single pending; overwrite counted as unprocessed','model_sha256':self.visual.model_sha256,
                  'hashes':{str(p.relative_to(ROOT)):sha(p) for p in [ROOT/'config/d405/round1_a.yaml',ROOT/'packages/d405_seam_tracking/src/d405_seam_tracking/geometry.py',ROOT/'packages/d405_seam_tracking/src/d405_seam_tracking/center_2d.py',ROOT/'packages/d405_seam_tracking/src/d405_seam_tracking/inference_backend.py',Path(__file__)]},
                  'no_hardware':True,'no_ROS_or_motion_transport':True,'raw_RGB_only':True,'network_calls_per_processed_frame':1}
        (out/'frozen_manifest.json').write_text(json.dumps(manifest,indent=2))
        async def execute():
            try:
                for session in SESSIONS:
                    if self.stop_requested:break
                    await self.clip(session,run)
                    await asyncio.sleep(.5)
            except Exception as e:
                (out/'error.txt').write_text(repr(e));print(repr(e),flush=True)
            finally:
                await asyncio.sleep(1)
                self.summarize(run);self.running=False;self.active=None
                self.broadcast({'type':'end','run_id':ident})
        self.task=asyncio.create_task(execute())
        return {'run_id':ident,'output':str(out)}

async def main(args):
    app=Replay(args);loop=asyncio.get_running_loop()
    class HTTP(BaseHTTPRequestHandler):
        def reply(self,obj,code=200):
            b=json.dumps(obj,ensure_ascii=False).encode();self.send_response(code);self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
        def do_GET(self):
            path=urlsplit(self.path).path
            if path=='/':
                b=(ROOT/'apps/web_console/d405_realtime_replay.html').read_bytes().replace(b'__WS_PORT__',str(args.ws_port).encode());self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
            elif path=='/api/status':
                async def status():
                    return {'running':app.running,'session':app.active,'center2d':args.center2d,'clients':len(app.clients),'summary':app.summarize(app.current_run) if app.current_run else None}
                self.reply(asyncio.run_coroutine_threadsafe(status(),loop).result(5))
            else:self.reply({'error':'not found'},404)
        def do_POST(self):
            if self.path=='/api/start': self.reply(asyncio.run_coroutine_threadsafe(app.start(),loop).result(5))
            elif self.path=='/api/stop':loop.call_soon_threadsafe(setattr,app,'stop_requested',True);self.reply({'stopping':True})
            else:self.reply({'error':'no movement or other commands supported'},404)
        def log_message(self,*args):pass
    http=ThreadingHTTPServer((args.bind,args.http_port),HTTP)
    Thread(target=http.serve_forever,daemon=True).start()
    async with serve(app.websocket,args.bind,args.ws_port,max_size=2**20,write_limit=65536):
        print('READY http://%s:%s center2d=%s'%(args.bind,args.http_port,args.center2d),flush=True)
        if args.run:await app.start()
        await asyncio.Future()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--center2d',action='store_true',help='opt-in; omission restores A-only')
    parser.add_argument('--inference-backend',choices=['cpu','cpu_channels_last'],default='cpu');parser.add_argument('--run',action='store_true');parser.add_argument('--bind',default='192.168.127.10');parser.add_argument('--http-port',type=int,default=8772);parser.add_argument('--ws-port',type=int,default=8773)
    parser.add_argument('--output',type=Path,default=ANA/'d405_realtime_replay_20261103')
    asyncio.run(main(parser.parse_args()))
