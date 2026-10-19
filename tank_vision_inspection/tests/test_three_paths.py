from __future__ import annotations

from dataclasses import replace
import threading
import time

import numpy as np

from tank_vision.inspection_pipeline import InspectionPipeline
from tank_vision.metrics import FrameMetrics
from tank_vision.pipeline import VisionPipeline
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.synthetic import seam_image
from tank_vision.types import FrameData, InspectionResult, QualityResult, RealtimeResult, SeamResult


def _frame(sequence: int) -> FrameData:
    return FrameData(seam_image(160, 120), sequence, "camera", sequence, "test")


def _realtime_result(frame: FrameData) -> RealtimeResult:
    return RealtimeResult(frame, SeamResult(frame.timestamp_ns, True, 1.0), QualityResult(), FrameMetrics())


def test_realtime_pipeline_keeps_latest_queued_frame(config) -> None:
    entered = threading.Event()
    release = threading.Event()

    def processor(frame: FrameData) -> RealtimeResult:
        if frame.sequence_id == 0:
            entered.set()
            assert release.wait(2)
        return _realtime_result(frame)

    pipeline = RealtimeSeamPipeline(config, processor=processor)
    assert pipeline.submit(_frame(0))
    assert entered.wait(1)
    assert pipeline.submit(_frame(1))
    assert pipeline.submit(_frame(2))
    release.set()
    pipeline.flush()
    results = [pipeline.get_result(1).frame.sequence_id for _ in range(2)]
    pipeline.close()
    assert results == [0, 2]
    assert pipeline.stats.summary()["dropped_frames"] == 1


def test_drop_oldest_queue_policy_is_explicit(config) -> None:
    assert config.realtime_seam.queue_size == 1
    assert config.realtime_seam.drop_policy == "drop_oldest"
    assert config.realtime_seam.process_latest_only


def test_offline_compatibility_pipeline_never_drops(config) -> None:
    pipeline = VisionPipeline(config)
    results = [pipeline.process(_frame(index)) for index in range(4)]
    assert [item.frame.sequence_id for item in results] == [0, 1, 2, 3]
    assert pipeline.stats.summary()["dropped_frames"] == 0


def test_inspection_records_tile_coordinate_transforms(config) -> None:
    config.inspection.tile_size = 128
    config.inspection.overlap = 32
    image = np.zeros((150, 190, 3), dtype=np.uint8)
    result = InspectionPipeline(config).process(FrameData(image, 1, "camera", 0, "test"))
    assert len(result.tile_transforms) == 4
    assert max(item.x + item.valid_width for item in result.tile_transforms) == 190
    assert max(item.y + item.valid_height for item in result.tile_transforms) == 150


def test_slow_inspection_does_not_block_realtime_result(config) -> None:
    inspection_entered = threading.Event()
    inspection_release = threading.Event()

    def slow_inspection(frame: FrameData) -> InspectionResult:
        inspection_entered.set()
        assert inspection_release.wait(2)
        return InspectionResult(frame, [])

    realtime = RealtimeSeamPipeline(config, processor=_realtime_result)
    inspection = InspectionPipeline(config, processor=slow_inspection)
    frame = _frame(0)
    assert inspection.submit(frame, force=True)
    assert inspection_entered.wait(1)
    assert realtime.submit(frame)
    assert realtime.get_result(1).frame.sequence_id == 0
    inspection_release.set()
    inspection.flush()
    realtime.close()
    inspection.close()


def test_inspection_frame_interval_trigger(config) -> None:
    config.inspection.trigger_mode = "frame_interval"
    config.inspection.trigger_every_n_frames = 3
    pipeline = InspectionPipeline(config)
    assert pipeline.should_trigger(_frame(0))
    assert not pipeline.should_trigger(_frame(1))
    assert pipeline.should_trigger(_frame(3))


def test_disabled_inspection_does_not_start_or_process(config) -> None:
    config.inspection.enabled = False
    processed: list[int] = []

    def processor(frame: FrameData) -> InspectionResult:
        processed.append(frame.sequence_id)
        return InspectionResult(frame, [])

    pipeline = InspectionPipeline(config, processor=processor)
    pipeline.start()
    assert pipeline._thread is None
    assert not pipeline.submit(_frame(0))
    assert not pipeline.submit(_frame(1), force=True)
    assert pipeline.process(_frame(2)).defect_results == []
    pipeline.close()
    assert processed == []
    assert pipeline.stats.summary()["processed_frames"] == 0
