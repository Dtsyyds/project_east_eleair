"""Build core wheel from its authority source; installation staging is disposable."""
from pathlib import Path
import argparse,subprocess,json,zipfile,hashlib
root=Path(__file__).resolve().parents[2]
system=root/'tank_inspection_system';build=system/'build_core';source=root/'tank_vision_inspection'
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--wheel-only',action='store_true');p.add_argument('--venv',type=Path,default=system/'.runtime/core_env',help='explicit isolated install target, default relocatable deployment runtime');args=p.parse_args()
for path in [build/'metadata',build/'objects',build/'wheels',build/'bdist']:path.mkdir(parents=True,exist_ok=True)
command=['python3',str(source/'setup.py'),'egg_info','--egg-base',str(build/'metadata'),
         'build','--build-base',str(build/'objects'),'bdist_wheel','--bdist-dir',str(build/'bdist'),'--dist-dir',str(build/'wheels')]
subprocess.run(command,cwd=source,check=True)
wheels=list((build/'wheels').glob('tank_vision_inspection-*.whl'));wheel=max(wheels,key=lambda p:p.stat().st_mtime_ns)
with zipfile.ZipFile(wheel) as archive:
    files=[name for name in archive.namelist() if name.startswith('tank_vision/') and name.endswith('.py')]
    assert files and all(archive.read(name)==(source/name).read_bytes() for name in files)
if not args.wheel_only:
    venv=args.venv.resolve()
    if not venv.exists():subprocess.run(['python3','-m','venv','--system-site-packages',str(venv)],check=True)
    subprocess.run([str(venv/'bin/python'),'-m','pip','install','--no-index','--no-deps','--force-reinstall',str(wheel)],check=True)
print(json.dumps({'wheel':str(wheel),'source_files_verified':len(files),'installed':not args.wheel_only,'venv':str(args.venv) if not args.wheel_only else None}))
