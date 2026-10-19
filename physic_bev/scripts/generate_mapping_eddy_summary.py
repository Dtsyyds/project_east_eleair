#!/usr/bin/env python3
"""Generate FIELD-W6 storage, QoS, Eddy, and before/after performance reports."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import sys
from typing import Any
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import atomic_json, load_json, utc_now  # noqa: E402


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    values = []
    if not path.is_file():
        return values
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            import json
            values.append(json.loads(line))
        except ValueError:
            pass
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--summary-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    reports = run_dir / "reports"
    summary = args.summary_dir.resolve()
    summary.mkdir(parents=True, exist_ok=True)
    created = utc_now()
    mcap = load_json(reports / "mcap_validation.json", {}) or {}
    acceptance = load_json(reports / "system_acceptance.json", {}) or {}
    baseline = load_json(reports / "baseline_performance.json", {}) or {}
    preflight = load_json(reports / "preflight_report.json", {}) or {}
    inventory = load_json(run_dir / "topic_inventory.json", {}) or {}
    manifest = load_json(run_dir / "run_manifest.json", {}) or {}
    samples = _read_jsonl(reports / "runtime_samples.jsonl")
    bag_bytes = sum(int(item.get("size", 0)) for bag in mcap.get("bags", {}).values()
                    for item in bag.get("files", []))
    video_bytes = sum(path.stat().st_size for path in (run_dir / "video/front").glob("*.mkv"))
    spans = [float(topic.get("span_sec") or 0) for bag in mcap.get("bags", {}).values()
             for topic in bag.get("message_validation", {}).get("topics", {}).values()]
    span = max(spans, default=0.0)
    average_MBps = (bag_bytes + video_bytes) / span / 1e6 if span else None
    writer_peaks = {}
    for sample in samples:
        for name, value in sample.get("writers", {}).items():
            writer_peaks[name] = max(writer_peaks.get(name, 0.0), float(value.get("write_MBps", 0)))
    storage = {
        "schema_version": "field_w6_storage_bandwidth_report_v1", "created_at": created,
        "mount": preflight.get("mount"), "benchmark": preflight.get("storage_benchmark"),
        "mcap_bytes": bag_bytes, "video_bytes": video_bytes, "span_sec": span,
        "average_MBps": average_MBps, "writer_peak_MBps": writer_peaks,
        "mapping_storage_policy": manifest.get("mapping_capture_profile", {}).get(
            "storage_policy", "historical_raw_mapping_topics"
        ),
        "recorded_mapping_topics": list(
            manifest.get("mapping_capture_profile", {}).get("recorded_topics", {})
            or inventory.get("record_groups", {}).get("mapping", {})
        ),
    }
    atomic_json(reports / "storage_bandwidth_report.json", storage)

    recorded_rates = acceptance.get("rates_hz", {})
    baseline_topics = baseline.get("mapping_topics", {})
    comparisons = {}
    for topic in ("/cloud_registered", "/path", "/aft_mapped_to_init"):
        before = float(baseline_topics.get(topic, {}).get("receive_hz") or 0)
        runtime_values = [
            float(sample.get("topics", {}).get(topic, {}).get("receive_hz") or 0)
            for sample in samples
            if float(sample.get("topics", {}).get(topic, {}).get("receive_hz") or 0) > 0
        ]
        after = runtime_values[-1] if runtime_values else float(recorded_rates.get(topic) or 0)
        change = (after - before) / before * 100.0 if before > 0 else None
        comparisons[topic] = {"baseline_hz": before, "recording_hz": after,
                              "recording_source": ("runtime_monitor" if runtime_values
                                                   else "recorded_mcap"),
                              "change_percent": change, "passes_minus_10_percent":
                              change is not None and change >= -10.0}
    fast_livo_samples = [sample.get("processes", {}).get("external_fast_livo", {})
                         for sample in samples
                         if sample.get("processes", {}).get("external_fast_livo")]
    cpu_values = [float(value.get("cpu_percent_one_core_basis", 0)) for value in fast_livo_samples]
    performance = {
        "schema_version": "field_w6_recording_performance_v1", "created_at": created,
        "executed": bool(samples), "duration_sec": span,
        "mapping_rate_comparison": comparisons,
        "mapping_rate_drop_over_10_percent": any(
            value["change_percent"] is not None and value["change_percent"] < -10.0
            for value in comparisons.values()),
        "fast_livo": {
            "baseline": baseline.get("fast_livo"),
            "recording_cpu_percent_mean_one_core_basis":
                sum(cpu_values) / len(cpu_values) if cpu_values else None,
            "recording_samples": len(cpu_values),
        },
        "recorder_process_samples": [sample.get("processes", {}) for sample in samples[-3:]],
        "ssd_average_MBps": average_MBps, "ssd_writer_peak_MBps": writer_peaks,
        "temperatures": [sample.get("resources", {}).get("temperatures", []) for sample in samples],
        "diagnostic_rule": "a mapping topic frequency drop greater than 10% is a performance failure",
    }
    atomic_json(reports / "recording_performance.json", performance)
    qos_path = run_dir / "frozen_configs" / "qos_overrides.yaml"
    qos = yaml.safe_load(qos_path.read_text()) if qos_path.is_file() else {}
    atomic_json(reports / "qos_inventory.json", {
        "schema_version": "field_w6_qos_inventory_v1", "created_at": created,
        "generated_from_live_mapping_endpoints": True, "overrides": qos,
        "topic_info_verbose": inventory.get("topic_info_verbose", {}),
    })
    eddy_topics = {name: kind for name, kind in inventory.get("all_topics", {}).items()
                   if name.startswith("/eddy/")}
    atomic_json(reports / "eddy_topic_inventory.json", {
        "schema_version": "field_w6_eddy_topic_inventory_v1", "created_at": created,
        "topics": eddy_topics,
        "topic_info_verbose": {name: value for name, value in
                               inventory.get("topic_info_verbose", {}).items()
                               if name.startswith("/eddy/")},
        "recorded": inventory.get("record_groups", {}).get("eddy", {}),
        "device_timestamp_available": False,
        "header_timestamp_source": "host system_clock after complete UART frame parse",
    })
    for path in reports.glob("*.json"):
        shutil.copy2(path, summary / path.name)
    trajectory = reports / "compact_trajectory.jsonl"
    if trajectory.is_file():
        shutil.copy2(trajectory, summary / trajectory.name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
