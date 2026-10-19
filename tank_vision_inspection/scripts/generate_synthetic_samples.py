#!/usr/bin/env python3
"""Generate deterministic seam/anomaly/laser samples for the offline demo."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision.synthetic import generate_sample_set


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="sample_data/synthetic")
    args = parser.parse_args()
    paths = generate_sample_set(args.output)
    print(f"Generated {len(paths)} synthetic samples in {Path(args.output).resolve()}")
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
