#!/usr/bin/env python3
"""Convenience wrapper around the package CLI."""

from __future__ import annotations
from inspection_runtime.paths import project_root, config_path, runtime_root, vision_evidence_root

from pathlib import Path
import sys

sys.path.insert(0, str(project_root() / 'packages/tank_vision'))

from tank_vision.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
