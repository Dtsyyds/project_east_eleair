import argparse
from pathlib import Path
import importlib.util,hashlib,json,subprocess,sys,os
parser=argparse.ArgumentParser();parser.add_argument('--update-closeout',action='store_true');args=parser.parse_args()
root=Path(__file__).resolve().parents[2];report=root/'tank_inspection_system/reports/refactor_v1'
os.environ.setdefault('ROS_LOG_DIR',str(report/'launch_help_logs'))
pairs=[('tank_vision',root/'tank_vision_inspection/tank_vision'),('tank_vision_ros',root/'tank_vision_inspection/ros2_ws/src/tank_vision_ros/tank_vision_ros'),('inspection_rerun',root/'tank_vision_inspection/ros2_ws/src/inspection_rerun/inspection_rerun'),('inspection_tools',root/'physic_bev/src/inspection_tools/inspection_tools'),('mindvision_camera_driver',root/'physic_bev/src/mindvision_camera_driver/mindvision_camera_driver')]
entries=[]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name,source in pairs:
 spec=importlib.util.find_spec(name);target=Path(spec.origin).parent
 for f in source.rglob('*.py'):
  installed=target/f.relative_to(source)
  entries.append({'source':str(f.relative_to(root)),'installed':str(installed),'source_sha256':sha(f),'installed_sha256':sha(installed) if installed.is_file() else None,'matches':installed.is_file() and sha(f)==sha(installed)})
commands=[['tank-vision','--help'],['python3',str(root/'tank_vision_inspection/scripts/run_three_path_demo.py'),'--help'],['python3',str(root/'physic_bev/scripts/validate_vehicle_collection.py'),'--help'],['python3',str(root/'physic_bev/scripts/replay_vehicle_collection.py'),'--help'],['python3',str(root/'physic_bev/scripts/build_role_handoff.py'),'--help'],['ros2','launch','tank_vision_ros','stable_seam_topic.launch.py','--show-args'],['ros2','launch','inspection_bringup','visual_field_validation.launch.py','--show-args'],['ros2','launch','inspection_bringup','hardware_pipeline.launch.py','--show-args'],['ros2','pkg','executables','tank_vision_ros'],['ros2','pkg','executables','inspection_sync'],['ros2','pkg','executables','mindvision_camera_driver']]
commands += [['ros2','launch','agv_bridge','tank_inspection.launch.py','--show-args'],['ros2','pkg','executables','agv_bridge']]
results=[]
for cmd in commands:
 try:
  r=subprocess.run(cmd,capture_output=True,text=True,timeout=30)
  results.append({'argv':cmd,'returncode':r.returncode,'stdout':r.stdout,'stderr':r.stderr,'hardware_started':False})
 except subprocess.TimeoutExpired as e:results.append({'argv':cmd,'error':str(e),'returncode':124})
result={'code_files':entries,'all_match':all(e['matches'] for e in entries),'safe_cli_launch_checks':results,'all_safe_checks_passed':all(r['returncode']==0 for r in results),'hardware_started':False,'config_authority':'source owning repos; installation copies generated from package data_files'}
if args.update_closeout:
 d=json.loads((report/'final_report.json').read_text());d.setdefault('closeout',{}).setdefault('checks',{})['installation']=result
 (report/'final_report.json').write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')
else:
 (report/'installation_correspondence.json').write_text(json.dumps(result,indent=2)+'\n')
print({'files':len(entries),'all_match':result['all_match'],'checks':len(results),'all_cli':result['all_safe_checks_passed'],'failed':[r['argv'] for r in results if r['returncode']]})
