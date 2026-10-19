#!/usr/bin/env python3
"""Export the code-backed timestamp source audit from stability configuration."""

from __future__ import annotations
import argparse
import json
from pathlib import Path
import yaml


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
    document = {
        "schema_version": "tank_timestamp_audit_v1",
        "use_sim_time_required": False,
        "devices": config.get("timestamp_audit", {}),
        "comparison_warning": (
            "Device, ROS system, and monotonic clock values must not be compared until their "
            "epoch mapping is proven. Monotonic time is used only for intervals."),
        "physical_sync_status": "BLOCKED",
        "physical_sync_reason": (
            "No shared trigger/PTP/PPS evidence or repeated common physical event measurement is present."),
    }
    Path(args.output).write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
