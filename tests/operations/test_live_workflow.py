"""Explicitly enabled hardware + real Firefox acceptance, against run.sh."""
from inspection_runtime.paths import project_root, runtime_root, load_system_config
import importlib.util
import json
import os
from pathlib import Path
import time
import urllib.request

import pytest

ROOT = project_root()
spec = importlib.util.spec_from_file_location("inspection_browser_helpers", project_root() / 'tests/system/test_websocket_console.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
browser = helpers.browser
pytestmark = pytest.mark.skipif(os.environ.get("SYSTEM_LIVE_TEST") != "1", reason="requires explicit hardware test")


@pytest.mark.skipif(not os.environ.get("SYSTEM_EXPECT_MISSING_TOPIC"), reason="requires a known disconnected stream")
def test_missing_camera_blocks_recording_and_keeps_other_previews(browser):
    missing = os.environ["SYSTEM_EXPECT_MISSING_TOPIC"]
    assert missing in load_system_config()["required_topics"]
    browser.open("http://127.0.0.1:8080")
    browser.wait("window.tankConsole.snapshot().connected", timeout=15)
    browser.wait("['mindvision','fisheye'].every(k=>window.tankConsole.snapshot().streams[k].displayed>2)", timeout=20)
    browser.wait("!document.getElementById('quick-start').disabled", timeout=10)
    browser.click("quick-start")
    browser.wait("window.tankConsole.snapshot().task?.status==='failed'", timeout=15)
    snapshot = browser.js("return window.tankConsole.snapshot()")
    assert missing in snapshot["task"]["error"]
    assert not snapshot["task"].get("run_id")
    assert not snapshot["task"].get("complete")
    assert any(check["label"] == missing and check["status"] == "failed"
               for check in snapshot["task"]["preflight_checks"])
    # A data fault must not bring down the operations service or live preview.
    frames = snapshot["streams"]["fisheye"]["displayed"]
    browser.wait("window.tankConsole.snapshot().streams.fisheye.displayed>" + str(frames), timeout=5)
    (runtime_root() / 'web_console/missing-camera-acceptance.json').write_text(
        json.dumps({"passed": True, "missing_topic": missing, "task": snapshot["task"],
                    "other_previews_alive": True}, ensure_ascii=False, indent=2))


@pytest.mark.skipif(os.environ.get("SYSTEM_EXPECT_STREAM_INTERRUPTION") != "1", reason="requires a known failing camera")
def test_camera_interruption_closes_recording_as_incomplete(browser):
    from websockets.sync.client import connect
    # Start at the first genuinely ready observation after a scoped restart.
    deadline = time.monotonic() + 30
    config = load_system_config()
    run_id = None
    with connect("ws://127.0.0.1:9001", proxy=None) as ws:
        def command(action, ident, parameters=None):
            ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "command"},
                                "payload": {"action": action, "request_id": ident, "client_id": "fault-acceptance",
                                            "parameters": parameters or {}}}))
            while True:
                payload = json.loads(ws.recv(timeout=5))["payload"]
                if payload.get("request_id") == ident and payload.get("status") in ("completed", "failed"):
                    return payload
        try:
            while time.monotonic() < deadline:
                result = command("collection.start", "fault-start-" + str(time.time_ns()))
                if result.get("task", {}).get("status") == "recording":
                    run_id = result["task"]["run_id"]
                    break
                time.sleep(.2)
            assert run_id is not None, result
            browser.open("http://127.0.0.1:8080")
            browser.wait("window.tankConsole.snapshot().task?.status==='incomplete'", timeout=30)
            task = browser.js("return window.tankConsole.snapshot().task")
            assert task["run_id"] == run_id
            assert task["complete"] is False
            assert task["writer_closed"] and task["mcap_valid"]
            assert "数据中断" in task["message"]
            assert any(c["status"] == "failed" for c in task["stream_checks_at_stop"])
            record = json.loads((Path(task["run_directory"]) / 'run.json').read_text())
            assert record["complete"] is False
            assert record["required_topics"] == config["required_topics"]
            assert record["collection_profile"] == config["collection_profile"]
            (runtime_root() / 'web_console/interruption-acceptance.json').write_text(
                json.dumps({"passed": True, "scope": "real camera fault handling", "task": task}, ensure_ascii=False, indent=2))
        finally:
            if run_id:
                command("collection.stop", "fault-cleanup-" + str(time.time_ns()), {"run_id": run_id})


def test_real_buttons_video_collection_reconnect_and_thickness(browser):
    from websockets.sync.client import connect
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        try:
            with opener.open("http://127.0.0.1:8080", timeout=.5):
                break
        except OSError:
            time.sleep(.2)
    browser.open("http://127.0.0.1:8080")
    browser.wait("window.tankConsole.snapshot().connected", timeout=15)
    browser.wait("!document.getElementById('quick-start').disabled", timeout=10)
    browser.wait("['mindvision','fisheye','depth_camera'].every(k=>window.tankConsole.snapshot().streams[k].displayed>2)", timeout=20)
    browser.click("quick-start")
    browser.wait("window.tankConsole.snapshot().task?.status==='recording'", timeout=20)
    run_id = browser.js("return window.tankConsole.snapshot().task.run_id")
    try:
        browser.click("measure-thickness")
        browser.wait("window.tankConsole.snapshot().measurement?.status==='completed' && window.tankConsole.snapshot().measurement?.source==='saved'", timeout=10)
        # Disconnecting/reconnecting the browser must preserve the owned recorder.
        browser.click("disconnect")
        browser.wait("!window.tankConsole.snapshot().connected")
        browser.click("connect")
        browser.wait("window.tankConsole.snapshot().connected && window.tankConsole.snapshot().task?.status==='recording'", timeout=12)
        browser.wait("!document.getElementById('measure-thickness').disabled", timeout=10)
        assert browser.js("return window.tankConsole.snapshot().task.run_id") == run_id
        thickness_text = browser.js("return document.getElementById('thickness-result').textContent")
        # A failed live acquisition must clear the preceding file-based result.
        sent = browser.js("return window.tankConsole.snapshot().counts.sent")
        click = browser.js("const b=document.getElementById('measure-thickness');const d=b.disabled;document.getElementById('measurement-source').value='live';b.click();return {disabled:d,notice:document.getElementById('notice').textContent,sent:window.tankConsole.snapshot().counts.sent};")
        assert click["sent"] == sent + 1, click
        browser.wait("['completed','failed'].includes(window.tankConsole.snapshot().measurement?.status) && window.tankConsole.snapshot().measurement?.source==='live'", timeout=65)
        live_measurement = browser.js("return window.tankConsole.snapshot().measurement")
        if live_measurement["status"] == "failed":
            assert browser.js("return document.getElementById('thickness-result').textContent") == "厚度：—"
        # Provide a meaningful recording duration after DDS discovery.
        time.sleep(15)
        before_stop = browser.js("return window.tankConsole.snapshot()")
        browser.click("quick-stop")
        browser.wait("['complete','incomplete','failed'].includes(window.tankConsole.snapshot().task?.status)", timeout=45)
        after_stop = browser.js("return window.tankConsole.snapshot()")
        assert after_stop["task"]["status"] == "complete", after_stop["task"]
        directory = Path(after_stop["task"]["run_directory"])
        run = json.loads((directory / "run.json").read_text())
        assert run["complete"] is True
        assert run["topic_counts"]["/inspection/emat/result"] >= 1
        bag_metadata = __import__("yaml").safe_load((directory / "bag/metadata.yaml").read_text())["rosbag2_bagfile_information"]
        seconds = bag_metadata["duration"]["nanoseconds"] / 1e9
        assert run["topic_counts"]["/camera/front/image_raw"] / seconds >= 6, run["topic_counts"]
        assert run["topic_counts"]["/camera/fisheye/image_raw"] / seconds >= 9, run["topic_counts"]
        config = load_system_config()
        d405_hz = float(config["hardware"]["d405_depth_profile"].split('x')[-1])
        for topic in config["required_topics"]:
            if topic.startswith('/d405/'):
                assert run["topic_counts"].get(topic, 0) / seconds >= .6 * d405_hz, (topic, run["topic_counts"])
        assert all(before_stop["streams"][k]["displayed"] > 0 for k in ("mindvision", "fisheye", "depth_camera"))
        assert before_stop["counts"]["invalid"] == 0
        report = {"passed": True, "required_topics": load_system_config()["required_topics"], "hardware": load_system_config()["hardware"], "run_id": run_id, "run_directory": str(directory), "thickness_display": thickness_text,
                  "topic_counts": run["topic_counts"], "browser_invalid_packets": before_stop["counts"]["invalid"],
                  "reconnect_preserved_recording": True,
                  "live_measurement_status": live_measurement["status"], "live_measurement_error": live_measurement.get("error"),
                  "video": {k: before_stop["streams"][k]["display_fps_2s"] for k in ("mindvision", "fisheye", "depth_camera")}}
        (runtime_root() / 'web_console/acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False))
    finally:
        # Scoped to the tested batch only, including assertion failures.
        with connect("ws://127.0.0.1:9001", proxy=None) as ws:
            ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "command"},
                                "payload": {"action": "collection.stop", "request_id": "acceptance-cleanup-" + str(time.time_ns()),
                                            "client_id": "acceptance", "parameters": {"run_id": run_id}}}))
