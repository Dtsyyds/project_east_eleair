"""Verify pinned archives and extract debs only into a new local sysroot."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess

ROOT = Path(__file__).resolve().parents[1]
def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--platform", choices=["s100p", "rk3588", "x86"], default="s100p")
    p.add_argument("--check-only", action="store_true")
    args = p.parse_args()
    archive_root = ROOT / "third_party/offline/system"
    manifest = json.loads((archive_root / "dependency_manifest.json").read_text())
    for record in manifest["archives"]:
        path = archive_root / record["path"]
        if not path.is_file(): raise FileNotFoundError(path)
        if hashlib.sha256(path.read_bytes()).hexdigest() != record["sha256"]: raise ValueError("SHA mismatch: " + str(path))
    if not args.check_only:
        if args.platform == "x86" or platform.machine() not in ("aarch64", "arm64"):
            raise RuntimeError("Pinned debs are ARM64; provide matching x86 dependencies, never unpack ARM64 onto x86")
        for record in manifest["archives"]:
            path = archive_root / record["path"]
            if path.suffix == ".deb": subprocess.run(["dpkg-deb", "-x", str(path), str(ROOT / ".runtime" / args.platform / "sysroot")], check=True)
    print(json.dumps({"archives_verified": len(manifest["archives"]), "extracted": not args.check_only}))
if __name__ == "__main__": main()
