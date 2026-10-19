"""Prepare baseline installation/entry artifacts without editing any working source."""
from pathlib import Path
import shutil,subprocess,json
SYSTEM=Path(__file__).resolve().parents[1]
ROOT=SYSTEM.parent
BACKUP=SYSTEM/'reports/refactor_v1/backups'
STAGING=SYSTEM/'build_core/baseline_staging'
STAGING.mkdir(parents=True,exist_ok=True)
for name in ['tank_vision','pyproject.toml','README.md','setup.py']:
    source=ROOT/'tank_vision_inspection'/name;dest=STAGING/name
    if source.is_dir():shutil.copytree(source,dest,dirs_exist_ok=True,ignore=shutil.ignore_patterns('__pycache__'))
    else:shutil.copy2(source,dest)
# Restore only captured pre-task files in the isolated baseline artifact.
for source in (BACKUP/'tank_vision_inspection/tank_vision').rglob('*.py'):
    rel=source.relative_to(BACKUP/'tank_vision_inspection')
    # A backup created while refining a task-new module is not a pre-task source.
    before=json.loads((SYSTEM/'reports/refactor_v1/source_sha256_before.json').read_text())
    if 'tank_vision_inspection/'+str(rel) in before:shutil.copy2(source,STAGING/rel)
venv=SYSTEM/'core_env_baseline'
if not venv.exists():subprocess.run(['python3','-m','venv','--system-site-packages',str(venv)],check=True)
subprocess.run([str(venv/'bin/python'),'-m','pip','install','--no-deps','--no-build-isolation',str(STAGING)],check=True)
# An entry overlay stores copies of edited original scripts and references all
# unaffected files. It is a rollback artifact, not another active source repo.
physic=SYSTEM/'rollback/physic_bev';physic.mkdir(parents=True,exist_ok=True)
for group in ['scripts','config']:
    target=physic/group;target.mkdir(exist_ok=True)
    for source in (ROOT/'physic_bev'/group).iterdir():
        dest=target/source.name
        if dest.exists() or dest.is_symlink():continue
        backup=BACKUP/'physic_bev'/group/source.name
        if backup.is_file():shutil.copy2(backup,dest)
        else:dest.symlink_to(source,target_is_directory=source.is_dir())
for group in ['install','src']:
    target=physic/group
    if not target.exists():target.symlink_to(ROOT/'physic_bev'/group,target_is_directory=True)
vision=SYSTEM/'rollback/tank_vision_inspection';vision.mkdir(parents=True,exist_ok=True)
for group in ['configs','ros2_ws']:
    target=vision/group
    if not target.exists():target.symlink_to(ROOT/'tank_vision_inspection'/group,target_is_directory=True)
package=venv/'lib/python3.10/site-packages/tank_vision'
target=vision/'tank_vision'
if not target.exists():target.symlink_to(package,target_is_directory=True)
print('Baseline installed. Source scripts/ROS installs unchanged. Use scripts/rollback.sh.')
