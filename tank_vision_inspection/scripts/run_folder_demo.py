#!/usr/bin/env python3
"""Convenience wrapper around the package CLI."""

from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
