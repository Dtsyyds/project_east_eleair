"""Install canonical Python packages in the specified local virtual environment."""
import os
import argparse
import subprocess
from pathlib import Path
from inspection_runtime.paths import project_root, runtime_root
p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--venv", type=Path, default=runtime_root() / "venv")
p.add_argument("--wheel-only", action="store_true")
a = p.parse_args()
if not a.wheel_only and not (a.venv / "bin/python").exists(): subprocess.run(["python3", "-m", "venv", "--system-site-packages", str(a.venv)], check=True)
for rel in ["packages/emat_core", "packages/inspection_runtime", "packages/tank_vision", "packages/d405_seam_tracking"]:
    command = ["python3", "-m", "pip", "wheel", "--no-deps", "--no-build-isolation", "-w", str(runtime_root()/"wheels")] if a.wheel_only else [str(a.venv/"bin/python"), "-m", "pip", "install", "--no-index", "--no-deps", "--no-build-isolation", "-e"]
    subprocess.run(command + [str(project_root()/rel)], env={k:v for k,v in os.environ.items() if k != "PYTHONPATH"}, check=True)
