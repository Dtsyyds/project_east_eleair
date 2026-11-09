"""Create a separate revision checkout without rewriting current work."""
import argparse, subprocess
from inspection_runtime.paths import project_root
p=argparse.ArgumentParser(description=__doc__);p.add_argument("commit");p.add_argument("directory");a=p.parse_args()
subprocess.run([str(project_root()/"deploy/rollback/restore.sh"),a.commit,a.directory],check=True)
