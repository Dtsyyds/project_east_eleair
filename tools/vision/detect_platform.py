#!/usr/bin/env python3
from __future__ import annotations
from inspection_runtime.paths import project_root, config_path, runtime_root, vision_evidence_root

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(project_root() / 'packages/tank_vision'))

from tank_vision.platforms import detect_platform


def main() -> int:
    parser = argparse.ArgumentParser(description="Detect tank vision platform capabilities")
    parser.add_argument("--platform", choices=("x86", "s100p", "rk3588"))
    args = parser.parse_args()
    print(json.dumps(detect_platform(args.platform).to_dict(), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
