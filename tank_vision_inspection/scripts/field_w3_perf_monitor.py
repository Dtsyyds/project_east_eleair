#!/usr/bin/env python3
"""Sample host/process resource state during one FIELD-W3 perf run."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import time


def _read_number(path: Path) -> float | None:
    try:
        return float(path.read_text().strip())
    except (OSError, ValueError):
        return None


def _temperatures() -> dict[str, float]:
    values: dict[str, float] = {}
    for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
        value = _read_number(path)
        if value is not None:
            values[path.parent.name] = value / 1000.0 if value > 200 else value
    return values


def _frequencies() -> dict[str, float]:
    values: dict[str, float] = {}
    for path in sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq")):
        value = _read_number(path)
        if value is not None:
            values[path.parent.parent.name] = value / 1000.0
    return values


def _processes() -> list[dict[str, object]]:
    try:
        completed = subprocess.run(
            ["ps", "-eo", "pid=,pcpu=,rss=,args="],
            capture_output=True, text=True, check=False,
        )
    except OSError:
        return []
    rows: list[dict[str, object]] = []
    needles = ("stable_seam_node", "mindvision_camera_node", "ros2 bag record")
    for line in completed.stdout.splitlines():
        parts = line.strip().split(None, 3)
        if len(parts) != 4 or not any(needle in parts[3] for needle in needles):
            continue
        try:
            rows.append({"pid": int(parts[0]), "cpu_percent": float(parts[1]),
                         "rss_kib": int(parts[2]), "command": parts[3]})
        except ValueError:
            continue
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--interval-sec", type=float, default=1.0)
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + max(0.0, args.duration_sec)
    with output.open("w", encoding="utf-8") as handle:
        while True:
            row = {
                "host_time_ns": time.time_ns(),
                "run_dir": str(Path(args.run_dir).resolve()),
                "processes": _processes(),
                "temperatures_c": _temperatures(),
                "cpu_frequencies_mhz": _frequencies(),
            }
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
            handle.flush()
            if time.monotonic() >= deadline:
                break
            time.sleep(max(0.05, args.interval_sec))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
