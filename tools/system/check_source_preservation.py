"""Validate migration source correspondence and static syntax."""
import argparse, subprocess, sys
from inspection_runtime.paths import project_root
p=argparse.ArgumentParser(description=__doc__);p.parse_args()
raise SystemExit(subprocess.call([sys.executable,str(project_root()/"tools/migration/validate.py")]))
