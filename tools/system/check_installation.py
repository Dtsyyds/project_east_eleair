"""Report installed module origins; never starts nodes or edits old reports."""
import argparse, importlib.util, json
p=argparse.ArgumentParser(description=__doc__);p.parse_args()
result={}
for name in ["tank_vision","emat_core","inspection_runtime","d405_seam_tracking","inspection_operations","tank_vision_ros","inspection_tools"]:
    try:
        spec=importlib.util.find_spec(name);result[name]={"status":"available" if spec else "dependency_blocked","origin":spec.origin if spec else None}
    except (ImportError,ValueError) as e:result[name]={"status":"dependency_blocked","error":str(e)}
print(json.dumps(result,indent=2))
raise SystemExit(0 if all(v["status"]=="available" for v in result.values()) else 2)
