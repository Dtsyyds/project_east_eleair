"""Failure injection and deadline regressions; never opens hardware."""
import threading
import time
import pytest
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.inspection_pipeline import InspectionPipeline
from tank_vision.types import FrameData, RealtimeResult, InspectionResult, SeamResult, QualityResult
from tank_vision.metrics import FrameMetrics, PipelineStats, StageTiming
from tank_vision.synthetic import seam_image

@pytest.mark.parametrize("kind", [RealtimeSeamPipeline, InspectionPipeline])
def test_worker_failure_reaches_caller_and_abandons_pending(config, kind):
    entered, release = threading.Event(), threading.Event()
    def broken(frame):
        entered.set()
        release.wait(2)
        raise OSError("injected processor failure")
    p = kind(config, processor=broken)
    frame = FrameData(seam_image(32, 24), 1, "camera", 0, "fixture")
    p.submit(frame)
    assert entered.wait(1)
    p.submit(frame)
    release.set()
    # Wait without Queue.join on the baseline (the missing-task bug is bounded here).
    p._thread.join(1)
    assert getattr(p, "state", None) == "FAILED"
    with pytest.raises(RuntimeError, match="failed"):
        p.close(.1)
    assert p._input.unfinished_tasks == 0
    assert p.lifecycle_status["abandoned"] == 1
    with pytest.raises(RuntimeError):
        p.submit(frame)

@pytest.mark.parametrize("kind", [RealtimeSeamPipeline, InspectionPipeline])
def test_close_deadline_covers_queue_and_blocked_processor(config, kind):
    entered, release = threading.Event(), threading.Event()
    def blocked(frame):
        entered.set()
        release.wait(3)
        if kind is RealtimeSeamPipeline:
            return RealtimeResult(frame, SeamResult(1, True, 1.), QualityResult(), FrameMetrics())
        return InspectionResult(frame, [])
    p = kind(config, processor=blocked)
    frame = FrameData(seam_image(32, 24), 1, "camera", 0, "fixture")
    p.submit(frame)
    assert entered.wait(1)
    p.submit(frame)
    calls = []
    def close():
        started = time.monotonic()
        try:
            p.close(.05)
        except BaseException as exc:
            calls.append((exc, time.monotonic() - started))
    closer = threading.Thread(target=close, daemon=True)
    closer.start()
    closer.join(.3)
    try:
        assert not closer.is_alive(), "close ignored its deadline in Queue.join"
        assert isinstance(calls[0][0], TimeoutError)
        assert calls[0][1] < .3
        assert p.lifecycle_status["worker_alive"]
        assert p.lifecycle_status["incomplete"]
    finally:
        release.set()
        closer.join(1)
        p._thread.join(1)
    with pytest.raises((RuntimeError, TimeoutError)):
        p.close(.1)

def test_stats_storage_is_bounded_and_mean_is_all_time():
    p = PipelineStats()
    for i in range(10000):
        p.record(StageTiming(total_algorithm_ms=i))
    assert len(p._timings) <= 2048
    s = p.summary()
    assert s["processed_frames"] == 10000
    assert s["stages"]["total_algorithm_ms"]["mean"] == 4999.5
    assert s["quantile_basis"] == "last_2048_frames_linear_interpolation"


def test_blocked_writer_deadline_and_disk_status(config, tmp_path):
    import json
    from tank_vision.outputs import AsyncResultWriter
    from tank_vision.pipeline import VisionPipeline
    config.storage.disk_low_space_threshold_mb = 0
    config.storage.save_all_raw = False
    config.storage.save_abnormal_raw = False
    frame=FrameData(seam_image(32,24),1,"camera",1,"fixture")
    result=VisionPipeline(config).process(frame)
    p=AsyncResultWriter(tmp_path,config)
    entered,release=threading.Event(),threading.Event()
    def blocked(*args):
        entered.set(); release.wait(2)
    p._writer.write=blocked
    p.submit(result)
    assert entered.wait(1)
    p.submit(result)
    started=time.monotonic()
    try:
        with pytest.raises(TimeoutError): p.close(.05)
        assert time.monotonic()-started < .3
        status=json.loads((tmp_path/'writer_status.json').read_text())
        assert status['state']=='FAILED' and status['incomplete'] and status['worker_alive']
    finally:
        release.set(); p._thread.join(1)
    with pytest.raises(RuntimeError): p.close(.1)
    assert p.lifecycle_status['abandoned']==1


def test_normal_close_twice_preserves_results(config):
    def processor(frame):
        return InspectionResult(frame, [])
    p=InspectionPipeline(config,processor=processor)
    frame=FrameData(seam_image(32,24),1,"camera",1,"fixture")
    p.submit(frame,force=True)
    p.close(.5); p.close(.5)
    assert p.state=='CLOSED'
    assert p.get_result(0).frame.sequence_id==1
    assert p.lifecycle_status['accepted']==p.lifecycle_status['completed']==1


def test_long_live_result_storage_is_bounded(config):
    import gc
    import os
    import numpy as np
    def rss():
        return int(open('/proc/self/statm').read().split()[1])*os.sysconf('SC_PAGE_SIZE')
    def processor(frame):
        return RealtimeResult(frame, SeamResult(1,True,1),QualityResult(),FrameMetrics())
    p=RealtimeSeamPipeline(config,processor=processor)
    config.realtime_seam.drop_policy='drop_oldest'
    observations=[]
    for batch in range(2):
        for i in range(3000):
            p.submit(FrameData(np.full((128,160,3),i%255,np.uint8),i+1,'camera',batch*3000+i,'fixture'))
            p.flush(.5)
        gc.collect(); observations.append(rss())
    p.close(.5)
    assert p._output.qsize()==2
    assert p.output_frames==6000
    assert p.output_dropped_frames==5998
    assert p.stats.processed_frames==6000 and len(p.stats._timings)==2048
    assert observations[1]-observations[0] < 8*1024*1024
    report_root = os.environ.get("TANK_REFACTOR_REPORT_DIR")
    if report_root:
        from pathlib import Path
        import json
        Path(report_root, "bounded_memory.json").write_text(json.dumps({
            "input_frames": 6000, "processed_frames": p.stats.processed_frames,
            "saved_results_in_output_queue": p._output.qsize(), "output_dropped": p.output_dropped_frames,
            "rss_bytes_at_3000_and_6000": observations, "rss_growth_bytes": observations[1]-observations[0],
            "window_frames": len(p.stats._timings), "conservation_verified": True,
            "hardware_executed": False}, indent=2)+"\n")


def test_closing_status_io_is_included_in_whole_deadline(config,tmp_path,monkeypatch):
    import tank_vision.async_status as status_module
    from tank_vision.outputs import AsyncResultWriter
    config.storage.disk_low_space_threshold_mb=0
    p=AsyncResultWriter(tmp_path,config)
    entered,release=threading.Event(),threading.Event()
    original=status_module.write_status
    def blocked(path,status):
        entered.set();release.wait(2);original(path,status)
    monkeypatch.setattr(status_module,'write_status',blocked)
    started=time.monotonic()
    try:
        with pytest.raises(TimeoutError,match='metadata'):p.close(.08)
        assert time.monotonic()-started<.3
        assert entered.is_set() and p.state=='FAILED'
        assert p.lifecycle_status['metadata_worker_alive'] and p.lifecycle_status['incomplete']
    finally:
        release.set();p._lifecycle.metadata_thread.join(1)
    import json
    final=json.loads((tmp_path/'writer_status.json').read_text())
    assert final['state']=='FAILED' and final['incomplete']
    with pytest.raises(RuntimeError):p.close(.1)

@pytest.mark.parametrize('kind',['provenance','roi','live'])
def test_recorder_final_metadata_io_has_bounded_exit(tmp_path,monkeypatch,kind):
    from pathlib import Path
    from tank_vision.geometry_provenance_recorder import GeometryProvenanceRecorder
    from tank_vision.roi_validation_recorder import ROIValidationRecorder
    from tank_vision.failure_event_recorder import LiveSessionRecorder
    config_path=Path(__file__).parents[1]/'configs/s100p_guided_anchor_seam_4096x3072.yaml'
    reference=Path(__file__).parents[1]/'configs/seam_geometry_reference_s100p_live_v1.json'
    if kind=='provenance':
        recorder=GeometryProvenanceRecorder(tmp_path,config_path,reference)
        lifecycle=recorder._lifecycle
    elif kind=='roi':
        recorder=ROIValidationRecorder(tmp_path,config_path,reference)
        lifecycle=recorder._lifecycle
    else:
        recorder=LiveSessionRecorder(tmp_path,config_path,reference,{})
        lifecycle=recorder._metrics_lifecycle
    entered,release=threading.Event(),threading.Event()
    original=recorder._finalize_metadata
    def blocked(*args):entered.set();release.wait(2);return original(*args)
    monkeypatch.setattr(recorder,'_finalize_metadata',blocked)
    started=time.monotonic()
    try:
        summary=recorder.finalize({},timeout=.08) if kind=='live' else recorder.close(.08)
        assert time.monotonic()-started<.3 and entered.is_set()
        assert summary['complete'] is False and 'timed out' in summary['close_error']
        assert lifecycle.status['metadata_worker_alive'] and lifecycle.state=='FAILED'
    finally:
        release.set();lifecycle.metadata_thread.join(1)
    assert recorder._final_summary['complete'] is False if kind=='live' else recorder.summary['complete'] is False


def test_late_metadata_write_repairs_complete_flag_after_timeout(tmp_path,monkeypatch):
    from pathlib import Path
    import json
    from tank_vision.roi_validation_recorder import ROIValidationRecorder
    root=Path(__file__).parents[1]
    p=ROIValidationRecorder(tmp_path,root/'configs/s100p_guided_anchor_seam_4096x3072.yaml',root/'configs/seam_geometry_reference_s100p_live_v1.json')
    entered,release=threading.Event(),threading.Event()
    original=Path.write_text;calls=[]
    def blocked(path,*args,**kwargs):
        if path==p.complete_path and not calls:
            calls.append(True);entered.set();release.wait(2)
        return original(path,*args,**kwargs)
    monkeypatch.setattr(Path,'write_text',blocked)
    try:
        result=p.close(.08)
        assert entered.is_set() and result['complete'] is False
        assert p.summary['complete'] is False
    finally:
        release.set();p._lifecycle.metadata_thread.join(1)
    assert json.loads(p.complete_path.read_text())['complete'] is False
    assert p.close(.1)['complete'] is False
