"""Real Firefox + bridge + saved MCAP, explicitly enabled for acceptance."""
import base64
import importlib.util
import json
import os
from pathlib import Path
import time
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("replay_browser_helpers", ROOT / "tank_inspection_system/tests/test_websocket_console.py")
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
browser = helpers.browser
pytestmark = pytest.mark.skipif(os.environ.get("SYSTEM_REPLAY_TEST") != "1", reason="requires running inspection console")
RUN_ID = "WS-20261014_145556-53ab1041"


def wait_server():
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        try:
            with opener.open("http://127.0.0.1:8080", timeout=.5):
                return
        except OSError:
            time.sleep(.2)
    raise AssertionError("inspection console HTTP server is unavailable")


def test_actual_split_mcap_sensor_windows_controls_and_source_unchanged(browser):
    from websockets.sync.client import connect
    run = ROOT.parent / "runs" / RUN_ID
    source = {str(p): (p.stat().st_size, p.stat().st_mtime_ns) for p in (run / "bag").iterdir()}
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    deadline = time.monotonic() + 25
    while time.monotonic() < deadline:
        try:
            with opener.open("http://127.0.0.1:8080", timeout=.5):
                break
        except OSError:
            time.sleep(.2)
    browser.open("http://127.0.0.1:8080")
    browser.wait("!document.getElementById('quick-replay').disabled", timeout=12)
    browser.js("document.getElementById('replay-run').value=arguments[0];document.getElementById('replay-rate').value='2';", RUN_ID)
    browser.click("quick-replay")
    browser.wait("window.TankReplay.snapshot().state?.status==='playing'", timeout=15)
    active = ["mindvision", "fisheye", "d405_color", "d405_depth", "d405_infra1", "d405_infra2", "lidar", "imu", "imu_raw", "eddy_raw", "eddy_mean"]
    try:
        browser.wait("arguments[0].every(k=>window.TankReplay.snapshot().panels[k]?.displayed>0)".replace("arguments[0]", json.dumps(active)), timeout=20)
        initial = browser.js("return window.TankReplay.snapshot()")
        assert initial["state"]["run_id"] == RUN_ID
        assert initial["panels"]["emat_raw"]["count"] == 0
        assert initial["panels"]["emat_result"]["count"] == 0
        assert browser.js("return document.getElementById('replay-window-emat_raw').textContent.includes('本批无数据')")
        assert browser.js("return document.getElementById('quick-start').disabled")
        image = browser.call("WebDriver:TakeScreenshot", {})
        (ROOT / "tank_inspection_system/.runtime/system_websocket/replay-preview.png").write_bytes(base64.b64decode(image))
        browser.click("replay-pause")
        browser.wait("window.TankReplay.snapshot().state.status==='paused'", timeout=8)
        time.sleep(.6)
        frozen = browser.js("return window.TankReplay.snapshot()")
        time.sleep(.6)
        current = browser.js("return window.TankReplay.snapshot()")
        assert current["state"]["position_seconds"] == frozen["state"]["position_seconds"]
        assert all(current["panels"][k]["received"] == frozen["panels"][k]["received"] for k in active)
        browser.click("replay-resume")
        browser.wait("window.TankReplay.snapshot().state.status==='playing'", timeout=8)
        browser.wait("window.TankReplay.snapshot().panels.mindvision.received>" + str(frozen["panels"]["mindvision"]["received"]), timeout=8)
        browser.click("replay-stop")
        browser.wait("window.TankReplay.snapshot().state.status==='stopped'", timeout=12)
        browser.wait("!document.getElementById('replay-start').disabled", timeout=8)
        browser.click("replay-start")
        browser.wait("window.TankReplay.snapshot().state.status==='playing'", timeout=15)
        assert browser.js("return window.TankReplay.snapshot().session_id") != initial["session_id"]
        # Full EOF crosses the real 10-GiB split and verifies natural completion.
        browser.wait("window.TankReplay.snapshot().state.status==='complete'", timeout=160)
        final = browser.js("return window.TankReplay.snapshot()")
        assert len(final["state"]["files"]) == 2
        assert final["state"]["progress_percent"] == 100
        assert not final["state"].get("decode_errors"), final["state"]
        assert all(final["panels"][k]["errors"] == 0 for k in active)
        assert {str(p): (p.stat().st_size, p.stat().st_mtime_ns) for p in (run / "bag").iterdir()} == source
        report = {"passed": True, "run_id": RUN_ID, "pause_resume_stop_restart": True, "read_all_split_files": True,
                  "source_unchanged": True, "panels": final["panels"], "duration_seconds": final["state"]["duration_seconds"],
                  "decode_errors": final["state"].get("decode_errors", {})}
        (ROOT / "tank_inspection_system/.runtime/system_websocket/replay-acceptance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False))
    finally:
        with connect("ws://127.0.0.1:9001", proxy=None) as ws:
            ident = "replay-test-cleanup-" + str(time.time_ns())
            ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "command"},
                                "payload": {"action": "replay.stop", "request_id": ident, "client_id": "acceptance", "parameters": {}}}))
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                packet = json.loads(ws.recv(timeout=3))
                if packet["payload"].get("request_id") == ident and packet["payload"].get("status") in ("completed", "failed"):
                    break


def test_page_refresh_restores_paused_sensor_previews(browser):
    from websockets.sync.client import connect
    wait_server()
    browser.open("http://127.0.0.1:8080")
    browser.wait("!document.getElementById('quick-replay').disabled", timeout=12)
    browser.js("document.getElementById('replay-run').value=arguments[0];", RUN_ID)
    previous_session = browser.js("return window.TankReplay.snapshot().session_id")
    browser.click("quick-replay")
    active = ["mindvision", "fisheye", "d405_color", "d405_depth", "d405_infra1", "d405_infra2", "lidar", "imu", "imu_raw", "eddy_raw", "eddy_mean"]
    try:
        browser.wait("window.TankReplay.snapshot().state?.status==='playing' && window.TankReplay.snapshot().session_id!=="+json.dumps(previous_session), timeout=15)
        browser.wait("!document.getElementById('replay-pause').disabled", timeout=8)
        expression = json.dumps(active) + ".every(k=>window.TankReplay.snapshot().panels[k]?.displayed>0)"
        browser.wait(expression, timeout=20)
        browser.click("replay-pause")
        browser.wait("window.TankReplay.snapshot().state.status==='paused'", timeout=8)
        paused = browser.js("return window.TankReplay.snapshot()")
        browser.open("http://127.0.0.1:8080")
        browser.wait("window.TankReplay.snapshot().state?.status==='paused'", timeout=12)
        browser.wait(expression, timeout=12)
        restored = browser.js("return window.TankReplay.snapshot()")
        assert restored["session_id"] == paused["session_id"]
        assert restored["state"]["position_seconds"] == paused["state"]["position_seconds"]
        assert all(restored["panels"][k]["errors"] == 0 for k in active)
        browser.click("replay-stop")
        browser.wait("window.TankReplay.snapshot().state.status==='stopped'", timeout=12)
    finally:
        with connect("ws://127.0.0.1:9001", proxy=None) as ws:
            ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "command"},
                                "payload": {"action": "replay.stop", "request_id": "restore-cleanup-"+str(time.time_ns()), "client_id": "acceptance", "parameters": {}}}))
