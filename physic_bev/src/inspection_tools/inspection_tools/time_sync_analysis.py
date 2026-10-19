#!/usr/bin/env python3
"""Analyze header intervals and cross-topic nearest-neighbor timestamp differences."""

from __future__ import annotations
import argparse
import bisect
import csv
import json
from pathlib import Path
from typing import Any
import yaml
from .stability_monitor import percentile


def _distribution(values: list[float]) -> dict[str, float | None]:
    absolute = [abs(value) for value in values]
    return {
        "count": len(values), "signed_mean_ms": sum(values) / len(values) if values else None,
        "absolute_p50_ms": percentile(absolute, 0.50),
        "absolute_p95_ms": percentile(absolute, 0.95),
        "absolute_p99_ms": percentile(absolute, 0.99),
        "absolute_max_ms": max(absolute) if absolute else None,
    }


def nearest_differences(left: list[int], right: list[int]) -> list[float]:
    output: list[float] = []
    if not right:
        return output
    for value in left:
        index = bisect.bisect_left(right, value)
        candidates = []
        if index < len(right):
            candidates.append(right[index])
        if index:
            candidates.append(right[index - 1])
        nearest = min(candidates, key=lambda candidate: abs(candidate - value))
        output.append((value - nearest) / 1e6)
    return output


def windowed_nearest_differences(left: list[tuple[int, int]], right: list[int],
                                 window_sec: float = 60.0) -> list[dict[str, Any]]:
    """Summarize timestamp spacing over monotonic receive-time windows."""
    if not left or not right:
        return []
    origin = left[0][1]
    buckets: dict[int, list[float]] = {}
    sorted_right = sorted(right)
    for stamp, receive_steady in left:
        difference = nearest_differences([stamp], sorted_right)[0]
        bucket = int(max(0, receive_steady - origin) / (window_sec * 1e9))
        buckets.setdefault(bucket, []).append(difference)
    return [{
        "window_start_sec": bucket * window_sec,
        "window_end_sec": (bucket + 1) * window_sec,
        **_distribution(values),
    } for bucket, values in sorted(buckets.items())]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--timing-csv", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
    stamps: dict[str, list[int]] = {}
    receives: dict[str, list[int]] = {}
    stamp_receive_pairs: dict[str, list[tuple[int, int]]] = {}
    with Path(args.timing_csv).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["header_timestamp_ns"]:
                stamp = int(row["header_timestamp_ns"])
                receive = int(row["receive_steady_timestamp_ns"])
                stamps.setdefault(row["topic"], []).append(stamp)
                stamp_receive_pairs.setdefault(row["topic"], []).append((stamp, receive))
            receives.setdefault(row["topic"], []).append(int(row["receive_steady_timestamp_ns"]))
    topics: dict[str, Any] = {}
    for topic, values in stamps.items():
        intervals = [(b - a) / 1e6 for a, b in zip(values, values[1:]) if b > a]
        topics[topic] = {
            "count": len(values), "regressions": sum(b < a for a, b in zip(values, values[1:])),
            "duplicates": sum(b == a for a, b in zip(values, values[1:])),
            "first_header_timestamp_ns": values[0],
            "last_header_timestamp_ns": values[-1],
            "header_interval_ms": _distribution(intervals),
        }
    pairs = {}
    for left, right in config.get("cross_device_pairs", []):
        comparable = bool(stamps.get(left) and stamps.get(right))
        differences = nearest_differences(stamps.get(left, []), sorted(stamps.get(right, []))) if comparable else []
        pairs[f"{left} <> {right}"] = {
            "status": "PASS" if comparable else "SKIP",
            "nearest_header_difference": _distribution(differences),
            "nearest_header_difference_by_60s_window": windowed_nearest_differences(
                stamp_receive_pairs.get(left, []), stamps.get(right, [])),
            "interpretation": (
                "Message timestamp nearest-neighbor spacing only; includes sampling-period quantization "
                "and is not a measurement of physical synchronization error."),
        }
    document = {
        "schema_version": "tank_time_sync_report_v1",
        "topics": topics, "cross_device_pairs": pairs,
        "physical_sync_accuracy": {"status": "BLOCKED", "measured": False,
            "reason": "No common hardware trigger/PTP/PPS or executed LED/common-motion event test."},
        "long_term_physical_drift": {"status": "BLOCKED", "measured": False,
            "reason": "Header monotonicity alone cannot establish physical clock drift."},
    }
    Path(args.output).write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n",
                                 encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
