#!/usr/bin/env python3
"""Read-only subscriber and static-test journal. No movement publishers exist."""
from pathlib import Path
import argparse,json,time,socket,sys,threading
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'packages/d405_seam_tracking/src'))
from d405_seam_tracking.receiver_hold import ErrorHold

def main():
 import rclpy
 from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
 from duct_robot_msgs.msg import WeldTrackingError
 from std_msgs.msg import Bool
 from sensor_msgs.msg import Image
 from d405_seam_tracking.dataset import image_array,stamp_ns
 import cv2,hashlib
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--bind',default='192.168.127.10');ap.add_argument('--port',type=int,default=8774);ap.add_argument('--node-name',default='rdk_weld_error_readonly_observer');ap.add_argument('--timeout',type=float,default=.5);ap.add_argument('--duration',type=float,default=0);args=ap.parse_args()
 args.output.mkdir(parents=True,exist_ok=False);fp=(args.output/'received.jsonl').open('x',buffering=1);lock=threading.Lock();data={'phase':'unmarked','state':None,'last_message':None,'count':0,'phase_count':0};image_data={'jpeg':None,'stamp_ns':None,'sha':None};h=ErrorHold(args.timeout)
 def log(kind,extra):
  with lock:
   row={'event':kind,'received_monotonic_ns':time.monotonic_ns(),'received_wall_ns':time.time_ns(),'phase':data['phase'],**extra};fp.write(json.dumps(row,ensure_ascii=False)+'\n')
 def message(m):
  d={'frame_seq':m.frame_seq,'stamp_ns':m.header.stamp.sec*10**9+m.header.stamp.nanosec,'frame_id':m.header.frame_id,'valid':m.valid,'lateral_error_norm':m.lateral_error_norm,'heading_error_rad':m.heading_error_rad,'confidence':m.confidence}
  # This board's Humble executor supplies only the message to callbacks.
  pubs=node.get_publishers_info_by_topic('/weld_tracking/error')
  if len(pubs)!=1:
   h.invalidate('publisher_missing_or_ambiguous');state=h.snapshot()
  else:state=h.receive(d,time.monotonic(),bytes(pubs[0].endpoint_gid).hex())
  with lock:data.update(state=state,last_message=d,count=data['count']+1)
  log('error_message',{'message':d,'receiver_state':state})
 def alive(m):
  old=h.state;state=h.source_alive(m.data)
  with lock:data['state']=state
  if not m.data and old!='LOST':log('source_lost',{'receiver_state':state})
 def tick():
  old=(h.state,h.reason);state=h.tick(time.monotonic())
  with lock:data['state']=state
  if old!=(h.state,h.reason):log('watchdog',{'receiver_state':state})
 def image(m):
  # Preview only, throttled; no model or overlay is evaluated here.
  if time.monotonic()-image.last<.5:return
  image.last=time.monotonic();rgb=image_array(m);bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR);cv2.rectangle(bgr,(128,168),(511,455),(0,220,255),1);ok,jpg=cv2.imencode('.jpg',bgr)
  if ok:
   with lock:image_data.update(jpeg=jpg.tobytes(),stamp_ns=stamp_ns(m),sha=hashlib.sha256(rgb.tobytes()).hexdigest())
 image.last=0
 rclpy.init();node=rclpy.create_node(args.node_name);q=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
 node.create_subscription(WeldTrackingError,'/weld_tracking/error',message,q);node.create_subscription(Bool,'/weld_tracking/source_alive',alive,q);node.create_subscription(Image,'/d405/camera/color/image_raw',image,q);node.create_timer(.02,tick)
 manifest={'hostname':socket.gethostname(),'node':args.node_name,'role':'RDK observation only; NOT proof of chassis receipt','domain':77,'movement_publishers':[],'message_timeout_sec':args.timeout,'reference':json.loads('{}')}
 (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2))
 class HTTP(BaseHTTPRequestHandler):
  def reply(self,obj,code=200):
   b=json.dumps(obj,ensure_ascii=False).encode();self.send_response(code);self.send_header('Content-Type','application/json');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b)
  def do_GET(self):
   if self.path.startswith('/image'):
    with lock:b=image_data['jpeg']
    if b is None:return self.reply({'waiting':True},503)
    self.send_response(200);self.send_header('Content-Type','image/jpeg');self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(b)));self.end_headers();self.wfile.write(b);return
   if self.path=='/api/status':
    with lock:v=dict(data)
    return self.reply(v)
   html='''<!doctype html><meta charset="utf-8"><title>D405 静态短测记录</title><style>body{font:18px sans-serif;margin:20px}button{padding:15px;margin:5px}img{max-width:100%}pre{white-space:pre-wrap}</style><h2>静态短测：手动移动画面中的焊缝，底盘不动</h2><p>先把焊缝放到对应位置，再点按钮并保持约 3 秒。按钮只写入人工阶段标签，不操作任何设备。</p><img id="rgb" width="640"><p id="buttons"></p><pre id="state"></pre><script>for(const p of ['左','中','右','完全离开 ROI','重新进入 ROI']){const b=document.createElement('button');b.textContent=p;b.onclick=async()=>{await fetch('/api/phase',{method:'POST',body:JSON.stringify({phase:p})})};document.getElementById('buttons').append(b)}setInterval(async()=>{document.getElementById('rgb').src='/image?t='+Date.now();document.getElementById('state').textContent=JSON.stringify(await(await fetch('/api/status')).json(),null,2)},500)</script>'''.encode();self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Content-Length',str(len(html)));self.end_headers();self.wfile.write(html)
  def do_POST(self):
   if self.path!='/api/phase':return self.reply({'error':'unsupported'},404)
   d=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
   if d.get('phase') not in ['左','中','右','完全离开 ROI','重新进入 ROI']:return self.reply({'error':'invalid phase'},400)
   with lock:
    data['phase']=d['phase'];data['phase_count']+=1;snapshot=dict(image_data);count=data['phase_count']
   log('human_phase',{'label_source':'operator button, no automatic labels','rgb_stamp_ns':snapshot['stamp_ns'],'rgb_sha256':snapshot['sha']})
   if snapshot['jpeg']:(args.output/f'phase_{count:02d}.jpg').write_bytes(snapshot['jpeg'])
   self.reply({'saved_phase_count':count,'file':str(args.output/'received.jsonl')})
  def log_message(self,*args):pass
 server=ThreadingHTTPServer((args.bind,args.port),HTTP);threading.Thread(target=server.serve_forever,daemon=True).start();print('READY static journal http://%s:%s'%(args.bind,args.port),flush=True)
 started=time.monotonic();last_graph=0
 try:
  while rclpy.ok() and (not args.duration or time.monotonic()-started<args.duration):
   rclpy.spin_once(node,timeout_sec=.05)
   if time.monotonic()-last_graph>2:
    last_graph=time.monotonic()
    pubs=node.get_publishers_info_by_topic('/weld_tracking/error');subs=node.get_subscriptions_info_by_topic('/weld_tracking/error')
    log('endpoints',{'publishers':[{'node':i.node_name,'namespace':i.node_namespace,'gid':bytes(i.endpoint_gid).hex()} for i in pubs],'subscribers':[{'node':i.node_name,'namespace':i.node_namespace,'gid':bytes(i.endpoint_gid).hex()} for i in subs]})
    if len(pubs)!=1:h.invalidate('publisher_missing_or_ambiguous')
 except KeyboardInterrupt:pass
 finally:server.shutdown();fp.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
