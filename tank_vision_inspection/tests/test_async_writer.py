from __future__ import annotations

from pathlib import Path

from tank_vision.outputs import AsyncResultWriter
from tank_vision.pipeline import VisionPipeline
from tank_vision.synthetic import seam_image
from tank_vision.types import FrameData


def _result(config):
    pipeline = VisionPipeline(config)
    result = pipeline.process(FrameData(seam_image(), 1, "camera", 0, "test", original_filename="frame.png"))
    return pipeline, result


def test_async_writer_flushes_all_accepted_outputs(config, tmp_path: Path) -> None:
    config.storage.disk_low_space_threshold_mb = 0
    pipeline, result = _result(config)
    with AsyncResultWriter(tmp_path, config) as writer:
        assert writer.submit(result, pipeline.last_defect_mask)
        writer.flush()
    assert len(list((tmp_path / "json").glob("*.json"))) == 1
    assert (tmp_path / "summary.csv").read_text(encoding="utf-8").count("\n") == 2
    assert result.metrics.timing.disk_write_ms >= 0


def test_async_writer_failure_is_contained(config, tmp_path: Path) -> None:
    config.storage.disk_low_space_threshold_mb = 0
    _pipeline, result = _result(config)
    writer = AsyncResultWriter(tmp_path, config)

    def broken_write(*_args, **_kwargs):
        raise OSError("simulated write failure")

    writer._writer.write = broken_write  # type: ignore[method-assign]
    assert writer.submit(result)
    import pytest
    with pytest.raises(RuntimeError, match="failed"):
        writer.flush()
    with pytest.raises(RuntimeError, match="failed"):
        writer.close()
    assert writer.state == "FAILED"
    assert any(item.startswith("writer_failed:OSError") for item in result.warnings)
    assert writer.stats.summary()["failed_frames"] == 1


def test_closed_async_writer_rejects_new_work(config, tmp_path: Path) -> None:
    import pytest

    config.storage.disk_low_space_threshold_mb = 0
    _pipeline, result = _result(config)
    writer = AsyncResultWriter(tmp_path, config)
    writer.close()
    with pytest.raises(RuntimeError, match="closed"):
        writer.submit(result)


def test_abnormal_raw_policy_keeps_pre_and_post_event_window(config, tmp_path: Path) -> None:
    config.storage.disk_low_space_threshold_mb = 0
    config.storage.normal_frame_interval = 100
    config.storage.pre_event_frames = 2
    config.storage.post_event_frames = 1
    pipeline = VisionPipeline(config)
    results = []
    for sequence in (1, 2, 3, 4):
        result = pipeline.process(
            FrameData(seam_image(), sequence, "camera", sequence, "test", original_filename=f"{sequence}.png")
        )
        results.append(result)
    results[2].seam_result.detected = False
    with AsyncResultWriter(tmp_path, config) as writer:
        for result in results:
            assert writer.submit(result)
        writer.flush()
    saved = sorted(path.name for path in (tmp_path / "raw").glob("*.jpg"))
    assert len(saved) == 4
