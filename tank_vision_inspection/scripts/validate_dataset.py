#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision.dataset_tools import load_manifest, validate_records


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate the tank inspection dataset")
    parser.add_argument("manifest")
    parser.add_argument("--dataset-root", default="dataset")
    parser.add_argument("--labels", default="dataset/schema/labels.yaml")
    parser.add_argument("--no-file-check", action="store_true")
    args = parser.parse_args()
    records = load_manifest(args.manifest)
    errors = validate_records(records, args.dataset_root, args.labels, not args.no_file_check)
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    print(f"Validated {len(records)} samples; no schema/path/group field errors found")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
