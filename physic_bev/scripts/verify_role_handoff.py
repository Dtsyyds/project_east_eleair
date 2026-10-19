#!/usr/bin/env python3
"""Verify a materialized role handoff without modifying it."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from build_role_handoff import HandoffError, verify_package


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-dir", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = verify_package(args.package_dir.resolve())
    except HandoffError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
