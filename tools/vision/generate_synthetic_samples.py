#!/usr/bin/env python3
"""Generate deterministic seam/anomaly/laser samples for the offline demo."""

from __future__ import annotations
from inspection_runtime.paths import project_root, config_path, runtime_root, vision_evidence_root

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(project_root() / 'packages/tank_vision'))

from tank_vision.synthetic import generate_sample_set


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=str(project_root() / "experiments/vision/sample_data/synthetic"))
    args = parser.parse_args()
    paths = generate_sample_set(args.output)
    print(f"Generated {len(paths)} synthetic samples in {Path(args.output).resolve()}")
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
