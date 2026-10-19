#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision.dataset_tools import create_grouped_split, load_manifest, split_leakage


def main() -> int:
    parser = argparse.ArgumentParser(description="Create session/tank/scan grouped splits")
    parser.add_argument("manifest")
    parser.add_argument("output")
    parser.add_argument("--train", type=float, default=0.7)
    parser.add_argument("--val", type=float, default=0.15)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    records = load_manifest(args.manifest)
    split = create_grouped_split(records, args.train, args.val, args.seed)
    errors = split_leakage(records, split)
    if errors:
        for error in errors:
            print(f"ERROR: {error}")
        return 1
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(split, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"Wrote leakage-free grouped split for {len(records)} samples to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
