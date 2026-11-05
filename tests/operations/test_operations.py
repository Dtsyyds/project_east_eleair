from inspection_runtime.paths import project_root
import copy
import json
import math
from pathlib import Path
import sys
import threading
import time

import pytest
import yaml

sys.path.insert(0, str(project_root() / 'packages/inspection_runtime/src/inspection_runtime'))
from inspection_runtime.operations import Operations, ROOT, mcap_closed, MCAP_MAGIC


@pytest.fixture
def controller(tmp_path, monkeypatch):
    config = yaml.safe_load((ROOT / "config/communication/system.yaml").read_text())
    config["data_root"] = str(tmp_path)
    events = []
    live = {name: time.monotonic() for name in config["required_topics"]}
    ops = Operations(config, events.append, lambda: dict(live))
    monkeypatch.setattr(ops, "storage", lambda: {"mounted": True, "free_bytes": 100 * 1024**3})
    yield ops, events, live
    ops.close()


def wait_result(events, ident):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        found = [p["payload"] for p in events if p["payload"].get("request_id") == ident and p["payload"].get("status") in ("completed", "failed")]
        if found:
            return found[-1]
        time.sleep(.01)
    raise AssertionError("no terminal response")


def request(action, ident="test", **params):
    return {"action": action, "request_id": ident, "client_id": "test-client", "parameters": params}


def test_start_checks_real_topic_freshness(controller):
    ops, events, live = controller
    live["/eddy/data"] = time.monotonic() - 30
    ops.handle(request("collection.start"))
    result = wait_result(events, "test")
    assert result["status"] == "failed"
    assert "/eddy/data" in result["error"]
    assert ops.recorder is None


def test_no_lidar_profile_still_requires_connected_inspection_sensors(controller):
    ops, events, live = controller
    assert ops.config["hardware"]["start_mid360"] is False
    assert not any(topic.startswith("/livox/") for topic in live)
    ops.preflight()
    live.pop("/d405/camera/depth/image_rect_raw")
    with pytest.raises(RuntimeError, match="/d405/camera/depth/image_rect_raw"):
        ops.preflight()


def test_emat_rejects_configured_eddy_serial_alias(controller, tmp_path):
    ops, events, live = controller
    device = tmp_path / "serial"
    device.touch()
    alias = tmp_path / "eddy-port"
    alias.symlink_to(device)
    ops.config["hardware"].update(start_eddy=True, eddy_device=str(alias))
    ops.config["emat"]["serial_port"] = str(device)
    ops.handle(request("emat.measure"))
    result = wait_result(events, "test")
    assert result["status"] == "failed"
    assert "串口冲突" in result["error"]


def test_preflight_and_duplicate_do_not_spawn_second_job(controller, monkeypatch):
    ops, events, live = controller
    calls = []
    monkeypatch.setattr(ops, "start", lambda params: calls.append(params))
    original = request("collection.start")
    ops.handle(original)
    assert wait_result(events, "test")["status"] == "completed"
    ops.handle(original)
    assert len(calls) == 1


def test_stop_wrong_run_does_not_cancel(controller):
    ops, events, live = controller
    ops.task = {"status": "recording", "run_id": "owned"}
    ops.handle(request("collection.stop", run_id="other"))
    assert wait_result(events, "test")["status"] == "failed"
    assert not ops.collection_cancel.is_set()


def test_unknown_action_and_profile_rejected(controller):
    ops, events, live = controller
    ops.handle(request("shell.run"))
    assert wait_result(events, "test")["status"] == "failed"
    ops.handle(request("collection.start", "profile", profile="arbitrary"))
    assert wait_result(events, "profile")["status"] == "failed"
    assert ops.recorder is None


def test_measure_protocol_and_algorithm(controller, tmp_path, monkeypatch):
    import serial
    ops, events, live = controller
    port_path = tmp_path / "emat"
    port_path.touch()
    ops.config["emat"].update(serial_port=str(port_path), sample_count=2000, read_interval_ms=0)
    data = [round(127 + 80 * math.sin(2 * math.pi * i / 12) *
                  sum(math.exp(-((i - center) / 20)**2) for center in range(300, 2000, 250))) for i in range(2000)]
    writes, published = [], []
    class Port:
        def __init__(self, *a, **kw):
            self.samples = iter(data)
        def __enter__(self):
            return self
        def __exit__(self, *_):
            pass
        def write(self, payload):
            writes.append(payload)
            self.command = payload
            return len(payload)
        def flush(self):
            pass
        def read(self, n):
            if self.command == b"\x81\x01":
                return b"\x01"
            if self.command == b"\x01":
                return b"\x02"
            return bytes([next(self.samples)])
    monkeypatch.setattr(serial, "Serial", Port)
    monkeypatch.setattr(ops.serial_api, "pause", lambda _: None)
    ops.publish_ultrasound = lambda samples, rate: published.append((samples, rate))
    ops.handle(request("emat.measure"))
    result = wait_result(events, "test")
    assert result["status"] == "completed", result
    measure = result["measurement"]
    assert abs(measure["thickness_mm"] - 10.125) < .1
    assert measure["echo_count"] >= 4
    assert writes[:2] == [b"\x81\x01", b"\x01"]
    assert writes[2:] == [b"\x00"] * 2000
    assert published[0][0] == data
    path = Path(measure["waveform_path"])
    assert ops.algorithm.read_signal(path) == data
    assert json.loads(path.with_name("result.json").read_text())["thickness_mm"] == measure["thickness_mm"]


def test_measure_failure_never_reuses_old_thickness(controller):
    ops, events, live = controller
    ops.config["emat"]["serial_port"] = "/nonexistent/inspection_emat"
    ops.measurement = {"status": "completed", "thickness_mm": 12.0}
    ops.handle(request("emat.measure"))
    result = wait_result(events, "test")
    assert result["status"] == "failed"
    assert "thickness_mm" not in result["measurement"]


def test_one_click_stop_cancels_measurement(controller, monkeypatch):
    ops, events, live = controller
    entered = threading.Event()
    def measuring():
        entered.set()
        ops.measure_cancel.wait(3)
        raise RuntimeError("测厚已停止")
    monkeypatch.setattr(ops, "measure", measuring)
    ops.handle(request("emat.measure", "measure"))
    assert entered.wait(2)
    ops.handle(request("collection.stop", "stop"))
    assert wait_result(events, "stop")["status"] == "completed"
    assert wait_result(events, "measure")["status"] == "failed"
    assert ops.measure_cancel.is_set()


def test_saved_waveform_result_is_labelled_and_does_not_fake_live_data(controller):
    ops, events, live = controller
    published = []
    ops.publish_ultrasound = lambda *args: published.append(args)
    ops.handle(request("emat.measure", source="saved"))
    result = wait_result(events, "test")
    assert result["status"] == "completed"
    assert result["measurement"]["source"] == "saved"
    assert result["measurement"]["thickness_mm"] == pytest.approx(5.265)
    assert published == []


def test_incomplete_mcap_is_not_complete(tmp_path):
    path = tmp_path / "record.mcap"
    path.write_bytes(MCAP_MAGIC + b"partial capture")
    assert not mcap_closed(path)
    path.write_bytes(MCAP_MAGIC + b"content" + MCAP_MAGIC)
    assert mcap_closed(path)


def test_unmounted_storage_stops_active_recording(controller, monkeypatch):
    ops, events, live = controller
    stops = threading.Event()
    class Process:
        def poll(self):
            return None
    ops.recorder = Process()
    ops.task = {"status": "recording"}
    monkeypatch.setattr(ops, "storage", lambda: {"mounted": False, "free_bytes": 100 * 1024**3})
    monkeypatch.setattr(ops, "stop_guarded", stops.set)
    ops.tick()
    assert stops.wait(2)
    assert ops.task["status"] == "stopping"
    assert "卸载" in ops.failure_reason
    ops.recorder = None


def test_disappeared_disk_reports_failure_without_crashing_status(controller, monkeypatch):
    ops, events, live = controller
    monkeypatch.setattr(ops, "storage", Operations.storage.__get__(ops))
    ops.config["data_root"] = str(Path(ops.config["data_root"]) / "missing-disk")
    status = ops.snapshot()
    assert status["storage"]["mounted"] is False
    assert status["storage"]["free_bytes"] == 0
    assert status["storage"]["error"]
    ops.handle(request("collection.preflight"))
    result = wait_result(events, "test")
    assert result["status"] == "failed"
    assert "采集盘" in result["error"]
    assert ops.recorder is None


def test_disk_io_error_still_reaches_automatic_stop(controller, monkeypatch):
    ops, events, live = controller
    stopped = threading.Event()
    monkeypatch.setattr(ops, "storage", Operations.storage.__get__(ops))
    def failed_usage(_):
        raise OSError("I/O error")
    monkeypatch.setattr("inspection_runtime.operations.shutil.disk_usage", failed_usage)
    monkeypatch.setattr(ops, "stop_guarded", stopped.set)
    ops.recorder = type("Recorder", (), {"poll": lambda self: None})()
    ops.task = {"status": "recording"}
    try:
        ops.tick()
        assert stopped.wait(2)
        assert ops.task["status"] == "stopping"
        assert events[-1]["payload"]["storage"]["error"] == "I/O error"
    finally:
        ops.recorder = None


def test_camera_profile_requires_all_six_streams(tmp_path):
    config = yaml.safe_load((ROOT / "config/communication/cameras.yaml").read_text())
    live = {topic: time.monotonic() for topic in config["required_topics"]}
    config["data_root"] = str(tmp_path)
    ops = Operations(config, lambda packet: None, lambda: live)
    ops.storage = lambda: {"mounted": True, "free_bytes": 100 * 1024**3}
    try:
        profiles = ops.snapshot()["capabilities"]["collection"]["profiles"]
        assert profiles == [{"id": "three_cameras", "label": "三相机采集"}]
        assert len(live) == 6
        assert "/eddy/data" not in live
        ops.preflight()
        for topic in list(live):
            last = live.pop(topic)
            with pytest.raises(RuntimeError, match=topic):
                ops.preflight()
            live[topic] = last
        with pytest.raises(ValueError, match="未知采集方案"):
            ops.start({"profile": "full_system"})
    finally:
        ops.close()


@pytest.mark.parametrize('stale', [False, True])
def test_one_frame_then_silent_cannot_finalize_complete(controller, tmp_path, stale):
    ops, events, live = controller
    ops.run_dir = tmp_path / 'run'
    bag = ops.run_dir / 'bag'
    bag.mkdir(parents=True)
    (bag / 'test.mcap').write_bytes(MCAP_MAGIC + b'fixture' + MCAP_MAGIC)
    (bag / 'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information': {
        'relative_file_paths': ['test.mcap'],
        'topics_with_message_count': [{'topic_metadata': {'name': topic}, 'message_count': 1}
                                     for topic in ops.config['required_topics']]}}))
    ops.recorder = type('Recorder', (), {'returncode': 0, 'poll': lambda self: 0})()
    if stale:
        live['/d405/camera/color/image_raw'] = time.monotonic() - 4
    ops.stop()
    assert ops.task['complete'] is (not stale)
    assert ops.task['mcap_valid'] is True
    if stale:
        assert ops.task['status'] == 'incomplete'
        assert '/d405/camera/color/image_raw' in ops.task['message']
    assert json.loads((ops.run_dir / 'run.json').read_text())['complete'] is (not stale)


def test_camera_tuning_is_blocked_during_recording(controller):
    ops, events, live = controller
    calls = []
    ops.camera_operation = lambda *args: calls.append(args) or {'values': {}}
    ops.task = {'status': 'recording'}
    for action in ('camera.settings.apply', 'camera.settings.save'):
        ops.handle(request(action, action, values={'saturation': 95}))
        assert wait_result(events, action)['status'] == 'failed'
    assert calls == []
    assert ops.task['status'] == 'recording'
    ops.handle(request('camera.settings.get', 'read'))
    assert wait_result(events, 'read')['status'] == 'completed'
    assert len(calls) == 1


def test_pending_tuning_blocks_collection_and_duplicate_save(controller, monkeypatch):
    ops, events, live = controller
    entered, release = threading.Event(), threading.Event()
    calls = []
    def apply(*args):
        calls.append(args)
        entered.set()
        release.wait(3)
        return {'saved': True, 'values': {'saturation': 95}}
    ops.camera_operation = apply
    try:
        command = request('camera.settings.save', 'save', values={'saturation': 95})
        ops.handle(command)
        assert entered.wait(2)
        ops.handle(command)
        ops.handle(request('collection.start', 'start'))
        assert wait_result(events, 'start')['status'] == 'failed'
        assert ops.recorder is None
    finally:
        release.set()
    assert wait_result(events, 'save')['status'] == 'completed'
    assert len(calls) == 1


def test_tuning_failure_does_not_fail_task_and_requires_readback(controller):
    ops, events, live = controller
    def fail(*args):
        raise RuntimeError('result unknown')
    ops.camera_operation = fail
    ops.handle(request('camera.settings.apply', 'apply', values={'saturation': 95}))
    assert wait_result(events, 'apply')['status'] == 'failed'
    assert ops.task['status'] == 'idle'
    ops.handle(request('collection.start', 'start'))
    assert wait_result(events, 'start')['status'] == 'failed'
    ops.camera_operation = lambda *args: {'values': {'saturation': 95}, 'fault': None}
    ops.handle(request('camera.settings.get', 'read'))
    assert wait_result(events, 'read')['status'] == 'completed'
    assert not ops.camera_uncertain


def test_collection_records_actual_camera_settings(controller, monkeypatch):
    ops, events, live = controller
    actual = {'serial_number': 'fixture-camera', 'saved': False,
              'values': {'exposure_time_us': 12345., 'saturation': 95}, 'fault': None}
    calls = []
    ops.camera_operation = lambda *args: calls.append(args) or copy.deepcopy(actual)

    def recorder(command, **kwargs):
        bag = Path(command[command.index('--output') + 1])
        bag.mkdir()
        (bag / 'fixture.mcap').touch()
        return type('Recorder', (), {'poll': lambda self: None})()

    monkeypatch.setattr('inspection_runtime.operations.subprocess.Popen', recorder)
    try:
        ops.handle(request('collection.start', 'record-settings'))
        assert wait_result(events, 'record-settings')['status'] == 'completed'
        assert calls == [('camera.settings.get', {})]
        record = json.loads((ops.run_dir / 'run.json').read_text())
        assert record['camera_settings'] == actual
        assert record['status'] == 'recording'
    finally:
        ops.recorder = None
        if ops.recorder_log:
            ops.recorder_log.close()
