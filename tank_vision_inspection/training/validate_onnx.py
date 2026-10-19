#!/usr/bin/env python3
"""Load an ONNX model and print raw tensor contracts; no decoder assumptions."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision.inference import OnnxBackend


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate ONNX load and tensor metadata")
    parser.add_argument("model")
    args = parser.parse_args()
    backend = OnnxBackend(args.model)
    if not backend.available:
        print(backend.unavailable_reason)
        return 2
    report = {
        "backend": backend.name,
        "inputs": [asdict(spec) for spec in backend.input_spec],
        "outputs": [asdict(spec) for spec in backend.output_spec],
        "decoder_validated": False,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2, default=str))
    backend.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
