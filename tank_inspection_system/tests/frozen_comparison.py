from pathlib import Path
import sys,json,time,resource,hashlib,dataclasses
import numpy as np
import cv2
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'tank_vision_inspection'))
report=root/'tank_inspection_system/reports/refactor_v1'
mode=sys.argv[1]
if mode=='baseline':
 import tank_vision.metrics as metrics
 p=report/'backups/tank_vision_inspection/tank_vision/metrics.py'
 exec(compile(p.read_text(),str(p),'exec'),metrics.__dict__)
from tank_vision.stable_seam_tracker import StableSeamTracker
from tank_vision.config import load_config
from tank_vision.pipeline import VisionPipeline
from tank_vision.synthetic import seam_image
from tank_vision.types import FrameData
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.inspection_pipeline import InspectionPipeline
if mode=='baseline':
 import tank_vision.realtime_pipeline as rt
 import tank_vision.inspection_pipeline as ip
 for module,name in [(rt,'realtime_pipeline'),(ip,'inspection_pipeline')]:
  p=report/f'backups/tank_vision_inspection/tank_vision/{name}.py'
  exec(compile(p.read_text(),str(p),'exec'),module.__dict__)
 RealtimeSeamPipeline,InspectionPipeline=rt.RealtimeSeamPipeline,ip.InspectionPipeline

def norm(value):
 if dataclasses.is_dataclass(value): return norm(dataclasses.asdict(value))
 if isinstance(value,np.ndarray): return value.tolist()
 if isinstance(value,np.generic): return value.item()
 if isinstance(value,dict): return {k:norm(v) for k,v in value.items() if k not in {'processing_ms','stage_timings_ms','processing_time_ms','metrics','timing','elapsed_ms'}}
 if isinstance(value,(tuple,list)): return [norm(x) for x in value]
 if hasattr(value,'name'): return value.name
 return value

def digest(value): return hashlib.sha256(json.dumps(norm(value),sort_keys=True,allow_nan=False).encode()).hexdigest()
config=root/'tank_vision_inspection/configs/s100p_guided_anchor_seam_4096x3072.yaml'
ref=root/'tank_vision_inspection/configs/seam_geometry_reference_s100p_live_v1.json'
image=cv2.imread(str(root/'tank_vision_inspection/output/s1_real_seam_images/seam_center_lit_quality_01.bmp'))
assert image is not None
images=[image,np.zeros_like(image),image]
records=[]; timings={}; started=time.perf_counter(); cpu=time.process_time()
for tracing in [False,True]:
 tracker=StableSeamTracker(config,ref)
 times=[]
 for index in range(33):
  input_image=images[index%3]
  t=time.perf_counter();result=tracker.process(input_image,diagnostics=tracing,provenance=tracing);ms=(time.perf_counter()-t)*1000
  if index>=3: times.append(ms)
  diag=tracker.last_diagnostics
  records.append({'trace':tracing,'index':index,'result':norm(result.to_dict()),
                  'pairs':norm(diag.all_pairs) if tracing else None,
                  'selected_path':norm(diag.selected_pairs) if tracing else None,
                  'provenance':norm(tracker.last_provenance) if tracing else None})
 timings['trace_on' if tracing else 'trace_off']={'p50_ms':float(np.percentile(times,50)),
 'p95_ms':float(np.percentile(times,95)),'throughput_hz':1000/float(np.mean(times)),'frames':len(times)}
config=load_config(root/'tank_vision_inspection/configs/default.yaml')
config.pipeline.thumbnail_max_dimension=0
pipelines=[RealtimeSeamPipeline(config),InspectionPipeline(config),VisionPipeline(config)]
for p in pipelines:
 for i in range(8):
  frame=FrameData(seam_image(160,120),i+1,'fixture',i,'frozen')
  result=p.process(frame)
  records.append({'pipeline':type(p).__name__,'index':i,'result':norm(result.to_dict()) if hasattr(result,'to_dict') else {'seam':norm(getattr(result,'seam_result',None)),'defects':norm(getattr(result,'defect_results',None)),'quality':norm(getattr(result,'quality_result',None))}})
json.dump(records,(report/f'{mode}_business_outputs.json').open('w'),ensure_ascii=False,sort_keys=True,indent=2,allow_nan=False)
summary={'business_sha256':digest(records),'timings':timings,'wall_sec':time.perf_counter()-started,
         'cpu_sec':time.process_time()-cpu,'peak_rss_bytes':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss*1024,
         'input_sha256':hashlib.sha256(image.tobytes()).hexdigest(),'input_frames':len(records),'queue_drop':0,
         'environment':{'python':sys.version,'opencv':cv2.__version__,'numpy':np.__version__}}
json.dump(summary,(report/f'{mode}_performance.json').open('w'),indent=2)
print(summary)
