"""Explicitly enabled hardware + real Firefox acceptance, against run.sh."""
import importlib.util
import json
import os
from pathlib import Path
import time
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("inspection_browser_helpers", ROOT / "tank_inspection_system/tests/test_websocket_console.py")
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
browser = helpers.browser
pytestmark = pytest.mark.skipif(os.environ.get("SYSTEM_LIVE_TEST") != "1", reason="requires explicit hardware test")


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
        # Actual EMAT serial is present but not responding; the failure must be
        # explicit and must clear the preceding file-based thickness.
        sent = browser.js("return window.tankConsole.snapshot().counts.sent")
        click = browser.js("const b=document.getElementById('measure-thickness');const d=b.disabled;document.getElementById('measurement-source').value='live';b.click();return {disabled:d,notice:document.getElementById('notice').textContent,sent:window.tankConsole.snapshot().counts.sent};")
        assert click["sent"] == sent + 1, click
        browser.wait("['completed','failed'].includes(window.tankConsole.snapshot().measurement?.status) && window.tankConsole.snapshot().measurement?.source==='live'", timeout=65)
        live_measurement = browser.js("return window.tankConsole.snapshot().measurement")
        if live_measurement["status"] == "failed":
            assert browser.js("return document.getElementById('thickness-result').textContent") == "厚度：—"
        # Provide a meaningful recording duration after DDS discovery.
        time.sleep(8)
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
        assert all(before_stop["streams"][k]["displayed"] > 0 for k in ("mindvision", "fisheye", "depth_camera"))
        assert before_stop["counts"]["invalid"] == 0
        report = {"passed": True, "run_id": run_id, "run_directory": str(directory), "thickness_display": thickness_text,
                  "topic_counts": run["topic_counts"], "browser_invalid_packets": before_stop["counts"]["invalid"],
                  "reconnect_preserved_recording": True,
                  "live_measurement_status": live_measurement["status"], "live_measurement_error": live_measurement.get("error"),
                  "video": {k: before_stop["streams"][k]["display_fps_2s"] for k in ("mindvision", "fisheye", "depth_camera")}}
        (ROOT / "tank_inspection_system/.runtime/system_websocket/acceptance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False))
    finally:
        # Scoped to the tested batch only, including assertion failures.
        with connect("ws://127.0.0.1:9001", proxy=None) as ws:
            ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "command"},
                                "payload": {"action": "collection.stop", "request_id": "acceptance-cleanup-" + str(time.time_ns()),
                                            "client_id": "acceptance", "parameters": {"run_id": run_id}}}))
