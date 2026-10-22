#!/usr/bin/env python3
"""Inspect setuptools resource declarations without installing ROS packages."""
import contextlib
import io
import json
import os
from pathlib import Path
import runpy
import setuptools

ROOT=Path(__file__).resolve().parents[2]

def main():
    records=[]
    for setup in sorted((ROOT/'ros2_ws/src').rglob('setup.py')):
        captured={}
        def capture(*args,**kwargs):captured.update(kwargs)
        original=setuptools.setup; cwd=Path.cwd()
        try:
            setuptools.setup=capture
            os.chdir(setup.parent)
            with contextlib.redirect_stdout(io.StringIO()):runpy.run_path(str(setup),run_name='__main__')
            absent=[]
            for destination,files in captured.get('data_files',[]):
                absent.extend(str(path) for path in files if not Path(path).is_file())
            records.append({'setup':str(setup.relative_to(ROOT)),'name':captured.get('name'),'missing_resources':absent,'entry_points':captured.get('entry_points',{})})
        finally:setuptools.setup=original;os.chdir(cwd)
    path=ROOT/'docs/validation/installation_metadata.json';path.write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'setups_checked':len(records),'missing_resources':sum(len(r['missing_resources']) for r in records)},indent=2))
    return any(r['missing_resources'] for r in records)
if __name__=='__main__':raise SystemExit(main())
