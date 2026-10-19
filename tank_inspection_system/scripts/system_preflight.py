"""Read-only software preflight. Never opens sensors, writes SSD benchmark or starts ROS nodes."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import platform
import subprocess

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path)
args=p.parse_args()
modules=['tank_vision','tank_vision_ros','inspection_tools','inspection_rerun','rosbag2_py','eddy_driver.msg','inspection_interfaces.msg']
checks={}
for name in modules:
    try:
        spec=importlib.util.find_spec(name)
        checks[name]={'available':spec is not None,'origin':str(spec.origin) if spec else None}
    except (ImportError,ValueError) as exc:checks[name]={'available':False,'error':str(exc)}
try:
    import rosbag2_py
    readers=sorted(rosbag2_py.get_registered_readers())
    writers=sorted(rosbag2_py.get_registered_writers())
except ImportError:
    readers=[]; writers=[]
checks['mcap_plugin']={'available':'mcap' in readers and 'mcap' in writers,
                       'registered_readers':readers,'registered_writers':writers,
                       'status':'available' if 'mcap' in readers and 'mcap' in writers else 'environment_blocked'}
data_root=Path(os.getenv('FIELD_DATA_ROOT','/media/mcap_data'))
checks['media_directory']={'available':data_root.is_dir(),'path':str(data_root),
                          'status':'available' if data_root.is_dir() else 'environment_blocked'}
result={'schema_version' :'tank_system_preflight_v1','hardware_executed':False,
        'machine':platform.machine(),'ros_distro':os.getenv('ROS_DISTRO'),
        'checks':checks,'source_roots':{'vision':os.getenv('VISION_ROOT'),'physic':os.getenv('FIELD_PHYSIC_ROOT')},
        'config_authority':'manifests/entrypoints.json and each owning repository',
        'mcap_plugin_status':checks['mcap_plugin']['status'],
        'hardware_revalidation_status':'not_executed'}
result['valid']=all(x['available'] for x in checks.values()) and result['machine']=='aarch64' and result['ros_distro']=='humble'
if args.output:
    if args.output.exists():p.error('refusing to overwrite an existing report')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
raise SystemExit(0 if result['valid'] else 2)
