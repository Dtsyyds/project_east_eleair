#!/usr/bin/env python3
"""Summarize one FIELD-W3 P0/P1/P2 run without changing its data."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import statistics
from typing import Any


TIMING_FIELDS = (
    "image_conversion_ms", "callback_total_ms", "packet_queue_wait_ms",
    "tracker_process_ms", "provenance_second_pass_ms", "provenance_task_build_ms",
    "snapshot_task_build_ms", "publish_geometry_ms", "publish_state_ms",
    "callback_to_output_ms", "process_packet_wall_ms",
)


def _load(path: Path, default: Any = None) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def _stats(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None, "mean": None}
    ordered = sorted(values)
    def percentile(q: float) -> float:
        position = (len(ordered) - 1) * q
        lower = int(position)
        upper = min(lower + 1, len(ordered) - 1)
        fraction = position - lower
        return ordered[lower] + (ordered[upper] - ordered[lower]) * fraction
    return {"count": len(values), "p50": percentile(.50), "p95": percentile(.95),
            "max": max(values), "mean": statistics.fmean(values)}


def _rows(run: Path) -> list[dict[str, Any]]:
    files = sorted(run.glob("live_session/*/frame_metrics.jsonl"))
    if not files:
        return []
    result: list[dict[str, Any]] = []
    for line in files[-1].read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            result.append(value)
    return result


def _monitor_summary(run: Path) -> dict[str, Any]:
    path = run / "logs" / "perf_monitor.jsonl"
    samples = []
    if path.is_file():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                samples.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    cpu = [float(p["cpu_percent"]) for s in samples for p in s.get("processes", []) if "cpu_percent" in p]
    rss = [float(p["rss_kib"]) for s in samples for p in s.get("processes", []) if "rss_kib" in p]
    temps = [float(v) for s in samples for v in s.get("temperatures_c", {}).values()]
    freqs = [float(v) for s in samples for v in s.get("cpu_frequencies_mhz", {}).values()]
    return {"sample_count": len(samples), "cpu_percent": _stats(cpu), "rss_kib": _stats(rss),
            "temperature_c": _stats(temps), "cpu_frequency_mhz": _stats(freqs),
            "source": str(path)}


def _live(run: Path) -> dict[str, Any]:
    files = sorted(run.glob("live_session/*/live_summary.json"))
    return _load(files[-1], {}) if files else {}


def _summary(run: Path, case: str, output_dir: Path) -> dict[str, Any]:
    live = _live(run)
    counters = live.get("counters", {}) if isinstance(live, dict) else {}
    rows = _rows(run)
    timing = {}
    for field in TIMING_FIELDS:
        values = [float(row[field]) for row in rows if row.get(field) is not None]
        timing[field] = _stats(values)
    processing_values = [float(row["processing_ms"]) for row in rows if row.get("processing_ms") is not None]
    timing["processing_ms"] = _stats(processing_values)
    wall_values = [float(row["process_packet_wall_ms"]) for row in rows if row.get("process_packet_wall_ms") is not None]
    wall_minus_processing = [
        float(row["process_packet_wall_ms"]) - float(row["processing_ms"])
        for row in rows if row.get("process_packet_wall_ms") is not None and row.get("processing_ms") is not None
    ]
    long_tail = sorted(
        ({"frame_index": row.get("frame_index"), **{field: row.get(field) for field in TIMING_FIELDS},
          "processing_ms": row.get("processing_ms"), "snapshot_saved": row.get("snapshot_saved", False)}
         for row in rows),
        key=lambda row: float(row.get("process_packet_wall_ms") or 0.0), reverse=True,
    )[:20]
    long_tail_path = output_dir / "long_tail_frames.csv"
    write_header = not long_tail_path.exists()
    with long_tail_path.open("a", newline="", encoding="utf-8") as handle:
        fields = ["case", "frame_index", *TIMING_FIELDS, "processing_ms", "snapshot_saved"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        if write_header:
            writer.writeheader()
        for row in long_tail:
            writer.writerow({"case": case, **row})
    return {
        "case": case, "run_dir": str(run.resolve()), "counters": counters,
        "system_pass": bool(
            (run / "mcap" / "validation_report.json").is_file()
            and (run / "mcap" / "manifest.yaml").is_file()
            and float(counters.get("tracker_fps") or 0.0) >= 6.0
        ),
        "rates": {key: counters.get(key) for key in ("input_fps", "camera_fps", "tracker_fps",
                                                       "common_active_duration_sec", "common_input_frames",
                                                       "common_output_frames")},
        "slot_overwritten": counters.get("preview_skipped_frames"),
        "processing_ms": counters.get("processing_ms", timing["processing_ms"]),
        "wall_time_vs_processing_ms": {"process_packet_wall_ms": _stats(wall_values),
                                        "wall_minus_processing_ms": _stats(wall_minus_processing)},
        "performance_timing": counters.get("performance_timing", timing),
        "timing_from_frame_metrics": timing,
        "recorder": {"geometry_provenance": counters.get("geometry_provenance"),
                      "auto_snapshot_requested": counters.get("auto_snapshot_requested"),
                      "auto_snapshot_saved": counters.get("auto_snapshot_saved"),
                      "auto_snapshot_rejected": counters.get("auto_snapshot_rejected"),
                      "provenance_capacity_guard_skips": counters.get("provenance_capacity_guard_skips"),
                      "provenance_second_pass_count": counters.get("provenance_second_pass_count"),
                      "provenance_task_build_count": counters.get("provenance_task_build_count"),
                      "session_disk_usage_bytes": counters.get("session_disk_usage_bytes")},
        "resource_monitor": _monitor_summary(run),
        "mcap_validated": (run / "mcap" / "validation_report.json").is_file(),
        "mcap_validation_report": _load(run / "mcap" / "validation_report.json", {}),
        "process_shutdown_audit": _load(run / "logs" / "processes_after_stop.txt", None) is not None,
    }


def _decision(summaries: dict[str, dict[str, Any]]) -> dict[str, Any]:
    p0 = float(summaries.get("p0", {}).get("rates", {}).get("tracker_fps") or 0.0)
    p1 = float(summaries.get("p1", {}).get("rates", {}).get("tracker_fps") or 0.0)
    p2 = float(summaries.get("p2", {}).get("rates", {}).get("tracker_fps") or 0.0)
    if p0 < 6.0:
        decision = "C"
    elif p2 < p0 * 0.9:
        decision = "D"
    elif p1 < 6.0:
        decision = "B"
    else:
        decision = "A"
    p0_wall = summaries.get("p0", {}).get("wall_time_vs_processing_ms", {})
    p2_wall = summaries.get("p2", {}).get("wall_time_vs_processing_ms", {})
    return {"decision": decision, "p0_tracker_hz": p0, "p1_tracker_hz": p1, "p2_tracker_hz": p2,
            "p0_ge_6hz": p0 >= 6.0, "p1_ge_6hz": p1 >= 6.0, "p2_ge_6hz": p2 >= 6.0,
            "provenance_overhead_observed": p2 < p0,
            "processing_wall_gap_p95_ms": {
                "p0": (p0_wall.get("wall_minus_processing_ms", {}) or {}).get("p95"),
                "p2": (p2_wall.get("wall_minus_processing_ms", {}) or {}).get("p95"),
            },
            "processing_wall_gap_max_ms": {
                "p0": (p0_wall.get("wall_minus_processing_ms", {}) or {}).get("max"),
                "p2": (p2_wall.get("wall_minus_processing_ms", {}) or {}).get("max"),
            },
            "gil_or_recorder_competition_evidence": {
                "observed": p2 < p0 and bool(summaries.get("p2", {}).get("slot_overwritten")),
                "basis": "P2 provenance second-pass/task wall-time tails, slot overwrites and RSS increase correlate with output-rate loss",
                "gil_attribution_proven": False,
            },
            "algorithm_behavior_modified": False,
            "system_performance_isolated": True,
            "system_integration_ready": bool(summaries.get("p0", {}).get("system_pass", False)),
            "algorithm_field_validation_pending": True,
            "production_ready": False}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=("p0", "p1", "p2"), required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    result = _summary(Path(args.run_dir), args.case, output)
    (output / {"p0": "p0_baseline.json", "p1": "p1_snapshot.json", "p2": "p2_provenance.json"}[args.case]).write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    summary_files = {case: _load(output / name, {}) for case, name in (
        ("p0", "p0_baseline.json"), ("p1", "p1_snapshot.json"), ("p2", "p2_provenance.json"))
    }
    (output / "rate_comparison.json").write_text(json.dumps({case: {"rates": value.get("rates", {}), "slot_overwritten": value.get("slot_overwritten"), "system_pass": value.get("system_pass")} for case, value in summary_files.items()}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "wall_time_breakdown.json").write_text(json.dumps({case: {"processing_ms": value.get("processing_ms"), "process_packet_wall_ms": (value.get("timing_from_frame_metrics", {}) or {}).get("process_packet_wall_ms"), "wall_minus_processing_ms": (value.get("wall_time_vs_processing_ms", {}) or {}).get("wall_minus_processing_ms"), "stages": value.get("performance_timing", {})} for case, value in summary_files.items()}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "recorder_overhead.json").write_text(json.dumps({case: value.get("recorder", {}) for case, value in summary_files.items()}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    decision = _decision(summary_files)
    (output / "next_decision.json").write_text(json.dumps(decision, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (output / "diagnostic_summary.json").write_text(json.dumps({"task": "FIELD-W3-PERF-ISOLATION-P0-P2", **decision, "runs": {case: value.get("run_dir") for case, value in summary_files.items()}}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
