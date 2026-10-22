#!/usr/bin/env python3
"""Safe migration smoke checks: no node, SDK initialization, bag replay or devices."""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]

def main():
    records=[]
    def run(label,argv,expected=0,env=None):
        result=subprocess.run(argv,cwd='/tmp',env=env,capture_output=True,text=True,timeout=60)
        records.append(dict(check=label,command=argv,returncode=result.returncode,expected=expected,status='passed' if result.returncode==expected else 'failed',stdout=result.stdout,stderr=result.stderr))
    for entry in ['build','env','run','preflight','collect','stop','replay','validate','handoff']:
        run(entry+' help',['bash',str(ROOT/'deploy'/f'{entry}.sh'),'--help'])
    run('CLI help',[str(ROOT/'apps/inspection_cli/inspection'),'--help'])
    run('web help',[str(ROOT/'apps/web_console/run.sh'),'--help'])
    for target in ['s100p','x86','rk3588']:
        run(target+' build plan',['bash',str(ROOT/'deploy/build.sh'),'--platform',target,'--plan'])
    env=dict(os.environ)
    for key in ['PYTHONPATH','AMENT_PREFIX_PATH','CMAKE_PREFIX_PATH','COLCON_PREFIX_PATH','LD_LIBRARY_PATH','VIRTUAL_ENV']:env.pop(key,None)
    env['EAST_ELEAIR_ROOT']=str(ROOT)
    python=ROOT/'.runtime/s100p/metadata_venv/bin/python'
    probe='''
import importlib, json, sys
from pathlib import Path
class NoROS:
 def find_spec(self, fullname, path=None, target=None):
  if fullname.split('.')[0] in {'rclpy','rosbag2_py','sensor_msgs','inspection_interfaces'}: raise RuntimeError('Unexpected core ROS import: '+fullname)
sys.meta_path.insert(0, NoROS())
names=['inspection_runtime.operations','inspection_runtime.replay','inspection_runtime.supervisor','inspection_runtime.data.vehicle_collection_manager','inspection_runtime.data.validate_vehicle_collection','inspection_runtime.data.build_role_handoff','emat_core.emat_serial_once','emat_core.measure_thickness','tank_vision','d405_seam_tracking','d405_seam_tracking.cli']
modules={name:importlib.import_module(name).__file__ for name in names}
from inspection_runtime.paths import load_system_config, config_path, project_root
assert all(Path(p).is_relative_to(project_root()) for p in modules.values()),modules
from tank_vision.stable_seam_tracker import StableSeamTracker
tracker=StableSeamTracker(config_path('vision/s100p_stable_seam_4096x3072.yaml'))
assert load_system_config()['emat']['sampling_rate_hz']==40000000.0
import yaml
print(json.dumps({'modules':modules,'yaml_version':yaml.__version__,'reference_loaded':True,'ROS_imports':'forbidden'},indent=2))
'''
    run('core imports and reference without ROS',[str(python),'-c',probe],env=env)
    for name in ['inspection','inspection-web','tank-vision','d405-seam']:
        run('installed '+name+' help',[str(python.parent/name),'--help'],env=env)
    run('safe preflight reports missing new overlay',['bash',str(ROOT/'deploy/preflight.sh'),'--json'],expected=2)
    output=ROOT/'docs/validation/migration_smoke_results.json'
    output.write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps([{k:r[k] for k in ['check','status','returncode']} for r in records],ensure_ascii=False,indent=2))
    return any(r['status']=='failed' for r in records)
if __name__=='__main__':raise SystemExit(main())
