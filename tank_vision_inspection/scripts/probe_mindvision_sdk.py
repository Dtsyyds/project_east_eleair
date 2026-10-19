#!/usr/bin/env python3
"""Report possible SDK files and ELF dependency/architecture information."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision.sources.mindvision_probe import probe_mindvision_sdk


def _inspect_binary(path: str) -> dict[str, object]:
    result: dict[str, object] = {"path": path}
    for name, command in (("file", ["file", "-b", path]), ("ldd", ["ldd", path])):
        try:
            completed = subprocess.run(command, check=False, capture_output=True, text=True, timeout=10)
            result[name] = completed.stdout.strip() or completed.stderr.strip()
            result[f"{name}_returncode"] = completed.returncode
        except (OSError, subprocess.TimeoutExpired) as exc:
            result[name] = f"unavailable:{type(exc).__name__}"
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="Read-only MindVision SDK probe")
    parser.add_argument("--root", action="append", default=[], help="Additional SDK root to inspect")
    args = parser.parse_args()
    probe = probe_mindvision_sdk(args.root)
    report = probe.to_dict()
    report["library_inspection"] = [_inspect_binary(path) for path in probe.libraries if Path(path).is_file()]
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if probe.available else 2


if __name__ == "__main__":
    raise SystemExit(main())
