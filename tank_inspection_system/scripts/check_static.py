import argparse
import sys
from pathlib import Path
import subprocess,json,hashlib
parser=argparse.ArgumentParser();parser.add_argument('--update-closeout',action='store_true');args=parser.parse_args()
root=Path(__file__).resolve().parents[2];report=root/'tank_inspection_system/reports/refactor_v1'
def cmd(argv):
 r=subprocess.run(argv,capture_output=True,text=True);return {'argv':argv,'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr}
checks=[]
for rel in ['tank_vision_inspection/tank_vision','tank_vision_inspection/scripts','tank_vision_inspection/tests','tank_vision_inspection/ros2_ws/src','physic_bev/scripts','physic_bev/src/inspection_tools','physic_bev/src/mindvision_camera_driver','physic_bev/src/system_websocket','tank_inspection_system/scripts','tank_inspection_system/tests']:
 checks.append(cmd(['python3','-m','compileall','-q',str(root/rel)]))
for p in list((root/'physic_bev/scripts').glob('*.sh'))+list((root/'tank_inspection_system/scripts').glob('*.sh')):
 checks.append(cmd(['bash','-n',str(p)]))
for rel in ['tank_vision_inspection','physic_bev/src']:checks.append(cmd(['git','-C',str(root/rel),'diff','--check']))
# Unborn/untracked files also get whitespace validation against the pre-edit backup.
for p in (report/'backups').rglob('*'):
 if p.is_file():
  rel=str(p.relative_to(report/'backups')).removesuffix('.closeout_original')
  dest=root/rel
  if dest.is_file() and p.read_bytes()!=dest.read_bytes():
   checks.append(cmd(['git','diff','--no-index','--check',str(p),str(dest)]))
for rel in ['physic_bev/src/system_websocket/agv_bridge/include/agv_bridge/tank_inspection_robot.hpp','physic_bev/src/system_websocket/agv_bridge/src/tank_inspection_robot.cpp','physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml','physic_bev/src/system_websocket/agv_bridge/launch/tank_inspection.launch.py','physic_bev/src/system_websocket/agv_bridge/test/test_inspection_bridge.py']:
 checks.append(cmd(['git','diff','--no-index','--check','/dev/null',str(root/rel)]))
for c in checks:
 c['passed']=c['returncode']==0 or ('--no-index' in c['argv'] and c['returncode']==1 and not c['stdout'] and not c['stderr'])
value={'passed':all(c['passed'] for c in checks),'checks':checks,'note':'git diff--check on unborn repo alone has no tracked diff; backup no-index checks cover actual edits; returncode 1 with empty check output indicates content difference, raw codes retained'}
if args.update_closeout:
 d=json.loads((report/'final_report.json').read_text());d.setdefault('closeout',{}).setdefault('checks',{})['static']=value
 (report/'final_report.json').write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
else:
 (report/'static_checks.json').write_text(json.dumps(value,indent=2)+'\n')
print({'passed':value['passed'],'checks':len(checks),'failed':[c for c in checks if not c['passed']]})
