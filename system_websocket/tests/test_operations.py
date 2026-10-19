import copy
import json
import math
from pathlib import Path
import sys
import threading
import time

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from operations import Operations, ROOT, mcap_closed, MCAP_MAGIC


@pytest.fixture
def controller(tmp_path, monkeypatch):
    config = yaml.safe_load((ROOT / "system_websocket/config/system.yaml").read_text())
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
