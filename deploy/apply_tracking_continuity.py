#!/usr/bin/env python3
"""Apply the versioned continuity patch to the separate chassis source package."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace',type=Path,help='robot_chassis_ros2ctrl directory')
    parser.add_argument('--check',action='store_true',help='verify only; never modify sources')
    parser.add_argument('--reverse',action='store_true',help='restore the recorded pre-optimization source version')
    args=parser.parse_args()
    directory=args.workspace.resolve()/'src/duct_robot_tracking'
    patches=Path(__file__).resolve().parent/'patches'
    manifest=json.loads((patches/'duct_robot_tracking_continuity.json').read_text())
    if args.reverse:
        manifest={name:dict(before=item['after'],after=item['before']) for name,item in manifest.items()}
    actual={name:hashlib.sha256((directory/name).read_bytes()).hexdigest() if (directory/name).exists() else None
            for name in manifest}
    if all(actual[n]==m['after'] for n,m in manifest.items()):
        print('Requested source version already present; hashes verified.');return
    mismatches=[n for n,m in manifest.items() if actual[n]!=m['before']]
    if mismatches:
        raise SystemExit('Source versions differ; refusing to overwrite: '+', '.join(mismatches))
    command=['patch','--batch','-p1','-i',str(patches/'duct_robot_tracking_continuity.patch')]
    if args.reverse: command.append('--reverse')
    subprocess.run(command+['--dry-run'],cwd=directory,check=True)
    if not args.check:
        subprocess.run(command,cwd=directory,check=True)
        for name,item in manifest.items():
            actual=hashlib.sha256((directory/name).read_bytes()).hexdigest() if (directory/name).exists() else None
            assert actual==item['after']
        print('Source patch applied. Build duct_robot_tracking before the next session.')


if __name__=='__main__': main()
