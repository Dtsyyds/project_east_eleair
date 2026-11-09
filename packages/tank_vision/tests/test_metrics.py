from __future__ import annotations

import json

from tank_vision.metrics import PipelineStats, StageTiming, percentile


def test_stage_timing_serializes_all_required_fields() -> None:
    timing = StageTiming(quality_ms=1.25, total_algorithm_ms=3.5)
    decoded = json.loads(json.dumps(timing.to_dict()))
    assert decoded["quality_ms"] == 1.25
    assert decoded["total_algorithm_ms"] == 3.5
    assert "disk_write_ms" in decoded


def test_percentiles_are_interpolated() -> None:
    values = list(range(1, 101))
    assert percentile(values, 50) == 50.5
    assert percentile(values, 90) == 90.1
    assert percentile(values, 95) == 95.05


def test_pipeline_stats_reports_counts_fps_and_maximum() -> None:
    stats = PipelineStats()
    stats.record(StageTiming(total_algorithm_ms=10, total_end_to_end_ms=20), queue_depth=1)
    stats.record(StageTiming(total_algorithm_ms=20, total_end_to_end_ms=40), queue_depth=3)
    stats.mark_dropped()
    summary = stats.summary()
    assert summary["processed_frames"] == 2
    assert summary["dropped_frames"] == 1
    assert summary["max_queue_depth"] == 3
    assert summary["stages"]["total_algorithm_ms"]["p50"] == 15
    assert summary["average_algorithm_fps"] > 60
