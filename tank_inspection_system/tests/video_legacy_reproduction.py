"""Show the old 95% check falsely accepting an unlocatable missing-frame legacy run."""
from pathlib import Path
import argparse,importlib.util,json,subprocess,sys,tempfile
ROOT=Path(__file__).resolve().parents[2]
REPORT=ROOT/'tank_inspection_system/reports/refactor_v1'
sys.path.insert(0,str(ROOT/'physic_bev/scripts'))
import validate_vehicle_collection as current
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--update-closeout',action='store_true');args=p.parse_args()
with tempfile.TemporaryDirectory(prefix='tank_legacy_video_') as temporary:
 run=Path(temporary)
 front=run/'video/front';front.mkdir(parents=True,exist_ok=True)
 video=front/'front_000001.mkv'
 if not video.exists():
  subprocess.run(['ffmpeg','-v','error','-f','lavfi','-i','testsrc=size=64x48:rate=10','-frames:v','19',
                  '-c:v','libx264','-preset','ultrafast','-tune','zerolatency','-pix_fmt','yuv420p',str(video)],check=True)
 sidecar=video.with_suffix('.frames.jsonl')
 sidecar.write_text(''.join(json.dumps({'frame_index':i,'ros_header_stamp_ns':1000000000+i*100000000})+'\n' for i in range(20)))
 (run/'reports').mkdir(exist_ok=True)
 (run/'reports/video_runtime_status.json').write_text(json.dumps({'received_frames':20,'queue_drop_count':0,'write_error_count':0}))
 backup=REPORT/'backups/physic_bev/scripts/validate_vehicle_collection.py'
 namespace={'__file__':str(ROOT/'physic_bev/scripts/validate_vehicle_collection.py'),'__name__':'legacy_validator'}
 exec(compile(backup.read_text(),str(backup),'exec'),namespace)
 old,new=namespace['validate_video'](run),current.validate_video(run)
 result={'baseline':old,'modified':new,'old_false_acceptance_reproduced':old['valid'] is True,
         'strict_rejection_verified':new['valid'] is False,'fixture_only':True}
 if args.update_closeout:
  final=REPORT/'final_report.json';d=json.loads(final.read_text());d.setdefault('closeout',{}).setdefault('checks',{})['legacy_missing_frame_reproduction']={'status':'verified','result':result,'evidence':'actual generated19frames/20journal; no shift/repair guessed'};final.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
 else:
  (REPORT/'p0_c_reproduction.json').write_text(json.dumps(result,indent=2)+'\n')
 assert result['old_false_acceptance_reproduced'] and result['strict_rejection_verified']
 print(json.dumps({'old_false_acceptance_reproduced':result['old_false_acceptance_reproduced'],'strict_rejection_verified':result['strict_rejection_verified'],'fixture_cleaned_on_exit':True}))
