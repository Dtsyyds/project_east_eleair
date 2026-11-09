#!/usr/bin/env python3
"""Invoke an explicitly selected project exporter without assuming a framework."""

from __future__ import annotations

import argparse
import importlib
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Framework-neutral ONNX export entry point")
    parser.add_argument("--exporter", required=True, help="Reviewed callable as package.module:function")
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    module_name, separator, function_name = args.exporter.partition(":")
    if not separator:
        parser.error("--exporter must use package.module:function")
    checkpoint = Path(args.checkpoint)
    if not checkpoint.is_file():
        parser.error(f"checkpoint does not exist: {checkpoint}")
    exporter = getattr(importlib.import_module(module_name), function_name)
    exporter(checkpoint, Path(args.output))
    if not Path(args.output).is_file():
        raise RuntimeError("Exporter returned without creating the requested ONNX file")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
