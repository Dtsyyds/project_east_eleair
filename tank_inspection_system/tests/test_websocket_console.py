"""Actual offline Firefox DOM/WebSocket + installed bridge/DDS, no hardware.

Uses Firefox's built-in Marionette protocol, so no Selenium/npm/browser downloads.
Protocol reference: https://firefox-source-docs.mozilla.org/remote/marionette/
"""
import base64
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import threading
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]
PAGE = ROOT / "tank_inspection_system/websocket_console/index.html"


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class Browser:
    def __init__(self, sock):
        self.sock, self.counter = sock, 0

    def read(self):
        length = bytearray()
        while True:
            byte = self.sock.recv(1)
            if not byte:
                raise EOFError("Firefox Marionette closed")
            if byte == b":":
                break
            length.extend(byte)
        size = int(length)
        assert size < 20 * 1024 * 1024
        body = bytearray()
        while len(body) < size:
            chunk = self.sock.recv(size - len(body))
            if not chunk:
                raise EOFError("incomplete Marionette response")
            body.extend(chunk)
        return json.loads(body)

    def call(self, command, parameters):
        self.counter += 1
        body = json.dumps([0, self.counter, command, parameters]).encode()
        self.sock.sendall(str(len(body)).encode() + b":" + body)
        result = self.read()
        assert result[:2] == [1, self.counter], result
        assert result[2] is None, result[2]
        value = result[3]
        return value.get("value") if isinstance(value, dict) and "value" in value else value

    def js(self, script, *args):
        return self.call("WebDriver:ExecuteScript", {"script": script, "args": list(args), "newSandbox": True, "sandbox": None})

    def click(self, identifier):
        self.js("document.getElementById(arguments[0]).click();", identifier)

    def wait(self, expression, timeout=6):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.js("return " + expression):
                return
            time.sleep(.05)
        raise AssertionError("browser condition failed: " + expression + "\n" + json.dumps(self.js("return window.tankConsole.snapshot();"), ensure_ascii=False))

    def open(self, uri):
        self.call("WebDriver:Navigate", {"url": uri})
        self.wait("!!window.tankConsole")


@pytest.fixture
def browser(tmp_path):
    if not shutil.which("firefox"):
        pytest.skip("real Firefox unavailable; browser E2E not verified")
    profile = tmp_path / "firefox_profile"
    profile.mkdir()
    port = free_port()
    preferences = {
        "marionette.port": port, "browser.startup.homepage": "about:blank",
        "app.update.disabledForTesting": True, "browser.shell.checkDefaultBrowser": False,
        "browser.safebrowsing.downloads.enabled": False, "browser.safebrowsing.phishing.enabled": False,
        "browser.safebrowsing.malware.enabled": False, "datareporting.healthreport.uploadEnabled": False,
        "network.proxy.type": 1, "network.proxy.http": "127.0.0.1", "network.proxy.http_port": 9,
        "network.proxy.ssl": "127.0.0.1", "network.proxy.ssl_port": 9,
        "network.proxy.no_proxies_on": "localhost,127.0.0.1", "network.captive-portal-service.enabled": False,
        "network.connectivity-service.enabled": False,
    }
    (profile / "user.js").write_text("\n".join("user_pref(" + json.dumps(k) + "," + json.dumps(v) + ");" for k, v in preferences.items()))
    log_path = tmp_path / "firefox.log"
    with log_path.open("w") as log:
        proc = subprocess.Popen(["firefox", "--headless", "--no-remote", "--marionette", "--profile", str(profile), "about:blank"], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        sock = None
        try:
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                try:
                    sock = socket.create_connection(("127.0.0.1", port), timeout=.3)
                    break
                except OSError:
                    if proc.poll() is not None:
                        break
                    time.sleep(.05)
            assert sock is not None, log_path.read_text()
            sock.settimeout(10)
            client = Browser(sock)
            assert client.read()["marionetteProtocol"] == 3
            client.capabilities = client.call("WebDriver:NewSession", {})["capabilities"]
            client.call("WebDriver:SetWindowRect", {"width": 1280, "height": 1000})
            yield client
        finally:
            if sock is not None:
                sock.close()
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait(timeout=2)


@pytest.mark.parametrize("control", [False, True])
def test_offline_browser_actual_installed_bridge_and_ros(browser, tmp_path, control):
    # Reuse already-tested message types and stamp helper; no duplicate driver.
    spec = importlib.util.spec_from_file_location("console_wireless_fixture", ROOT / "physic_bev/src/system_websocket/agv_bridge/test/test_inspection_bridge.py")
    fixture = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fixture)
    prefix = Path(fixture.get_package_prefix("agv_bridge"))
    port = free_port()
    topics = {"fisheye": fixture.Image, "mindvision": fixture.Image, "depth": fixture.Image,
              "eddy": fixture.EddyFrame, "emat": fixture.UltrasoundFrame, "chassis": fixture.String,
              "lidar": fixture.PointCloud2, "pose": fixture.Odometry}
    params = {"robot_id": "tank_inspection_robot", "server_uri": "", "local_server_port": port,
              "tank_inspection.device_timeout_sec": 1.4, "tank_inspection.control_enabled": control,
              "tank_inspection.control_token": "fixture-browser-secret" if control else "",
              "tank_inspection.chassis_command_topic": "/console_fixture/cmd", "tank_inspection.eddy_mean_topic": ""}
    for name, short in {"fisheye": "fisheye", "mindvision": "mindvision", "depth_camera": "depth",
                        "eddy_raw": "eddy", "emat": "emat", "chassis_status": "chassis",
                        "lidar": "lidar", "lidar_pose": "pose"}.items():
        params["tank_inspection." + name + "_topic"] = "/console_fixture/" + short
    config = tmp_path / "params.yaml"
    config.write_text(fixture.yaml.safe_dump({"websocket_bridge_node": {"ros__parameters": params}}))
    fixture.rclpy.init()
    node = fixture.rclpy.create_node("websocket_console_fixture")
    pubs = {name: node.create_publisher(cls, "/console_fixture/" + name, fixture.qos_profile_sensor_data) for name, cls in topics.items()}
    motion = []
    node.create_subscription(fixture.Twist, "/console_fixture/cmd", motion.append, 10)
    counter = 0

    def publish():
        nonlocal counter
        counter += 1
        value = fixture.STAMP_BASE + counter * 50_000_000
        for key in ("fisheye", "mindvision"):
            pubs[key].publish(fixture.stamp(fixture.Image(height=8, width=8, encoding="rgb8", step=24, data=[90, 20, 10] * 64), value))
        pubs["depth"].publish(fixture.stamp(fixture.Image(height=8, width=8, encoding="rgb8", step=24, data=[180, 40, 20] * 64), value))
        pubs["eddy"].publish(fixture.stamp(fixture.EddyFrame(channel_count=8, samples_per_channel=20, data=list(range(160))), value))
        pubs["emat"].publish(fixture.stamp(fixture.UltrasoundFrame(channel_count=1, sample_count=2, sampling_rate_hz=1000., samples=[3, 4]), value))
        pubs["chassis"].publish(fixture.String(data="online"))
        pubs["lidar"].publish(fixture.stamp(fixture.PointCloud2(height=1, width=1, point_step=4, row_step=4, data=b"\0" * 4), value))
        pose = fixture.stamp(fixture.Odometry(), value)
        pose.pose.pose.position.x = 1.25
        pose.pose.pose.position.y = -.5
        pose.pose.pose.orientation.w = 1.
        pose.child_frame_id = "fixture_body"
        pubs["pose"].publish(pose)
        fixture.rclpy.spin_once(node, timeout_sec=.05)

    with (tmp_path / "bridge.log").open("w") as log:
        proc = subprocess.Popen([str(prefix / "lib/agv_bridge/websocket_bridge_node"), "--ros-args", "--params-file", str(config)], stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, ROS_LOG_DIR=str(tmp_path / "ros_logs")))
        try:
            browser.open(PAGE.as_uri())
            assert not browser.js("return window.tankConsole.snapshot().connected;")
            assert browser.js("return document.getElementById('move').disabled;")
            browser.js("document.getElementById('endpoint').value=arguments[0];", "ws://127.0.0.1:" + str(port))
            browser.click("connect")
            browser.wait("window.tankConsole.snapshot().connected")
            browser.wait("window.tankConsole.snapshot().last_response !== null")
            initial = browser.js("return window.tankConsole.snapshot();")
            assert initial["devices"]["emat"]["status"] == "unverified"
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and not all(p.get_subscription_count() for p in pubs.values()):
                fixture.rclpy.spin_once(node, timeout_sec=.05)
            assert all(p.get_subscription_count() for p in pubs.values())
            for _ in range(5):
                publish()
            browser.wait("['fisheye','mindvision','depth_camera','eddy_current','emat','chassis'].every(k=>window.tankConsole.snapshot().devices[k].status==='verified')")
            browser.wait("['fisheye','depth_camera'].every(k=>window.tankConsole.snapshot().streams[k].decoded>0)")
            browser.wait("document.getElementById('pose-x').textContent==='1.2500'")
            valid = browser.js("return window.tankConsole.snapshot();")
            assert valid["streams"]["depth_camera"]["last_source"]["source_encoding"] == "rgb8"
            pixel = browser.js("const c=document.createElement('canvas');c.width=8;c.height=8;const x=c.getContext('2d');x.drawImage(document.getElementById('depth_camera-image'),0,0);return Array.from(x.getImageData(4,4,1,1).data);")
            assert abs(pixel[0]-180)<12 and abs(pixel[1]-40)<12 and abs(pixel[2]-20)<12, pixel
            assert valid["devices"]["eddy_raw"]["status"] == "verified"
            assert valid["devices"]["eddy_mean"]["status"] == "unverified"
            assert valid["devices"]["eddy_mean"]["reason"] == "interface_unconfigured"
            for key in ("fisheye", "depth_camera"):
                assert valid["streams"][key]["last_source"]["stamp_ns"].startswith(str(fixture.STAMP_BASE)[:10])
                assert "data" not in valid["streams"][key]["last_source"]
            assert valid["pose"]["frame_id"] == "fixture_sensor" and valid["pose"]["position_unit"] == "m"
            assert not valid["hardware_revalidated"] and not valid["production_ready"]
            browser.click("stop-stream")
            browser.wait("window.tankConsole.snapshot().last_response.camera?.command==='stop_stream'")
            time.sleep(.3)
            before = browser.js("return window.tankConsole.snapshot().streams.fisheye.received;")
            publish()
            time.sleep(.3)
            assert browser.js("return window.tankConsole.snapshot().streams.fisheye.received;") == before
            browser.click("start-stream")
            browser.wait("window.tankConsole.snapshot().last_response.camera?.command==='start_stream'")
            browser.wait("window.tankConsole.snapshot().streams.fisheye.received>" + str(before))
            browser.click("enable-control")
            browser.js("document.getElementById('control-token').value='fixture-browser-secret';document.getElementById('vx').value='0.1';")
            publish()
            browser.click("move")
            if control:
                browser.wait("window.tankConsole.snapshot().last_response.agv?.execution==='published_not_hardware_acknowledged'")
                deadline = time.monotonic() + .7
                while time.monotonic() < deadline:
                    fixture.rclpy.spin_once(node, timeout_sec=.02)
                assert any(m.linear.x == .1 for m in motion)
                assert motion[-1].linear.x == 0.
            else:
                browser.wait("window.tankConsole.snapshot().last_response.agv?.reason==='control_disabled'")
            browser.click("stop")
            browser.wait("window.tankConsole.snapshot().last_response.agv?.command==='stop'")
            assert "fixture-browser-secret" not in browser.js("return JSON.stringify(window.tankConsole.snapshot());")
            fixture.rclpy.spin_once(node, timeout_sec=.1)
            if not control:
                assert not motion
            time.sleep(1.5)
            browser.wait("document.getElementById('device-fisheye').classList.contains('bad')")
            browser.wait("document.getElementById('pose-x').textContent==='—'")
            screenshot = browser.call("WebDriver:TakeScreenshot", {"full": True})
            (tmp_path / "console.png").write_bytes(base64.b64decode(screenshot))
            browser.click("disconnect")
            browser.wait("!window.tankConsole.snapshot().connected")
            count = browser.js("return window.tankConsole.snapshot().connection_count;")
            time.sleep(1.2)
            assert browser.js("return window.tankConsole.snapshot().connection_count;") == count
            browser.click("connect")
            browser.wait("window.tankConsole.snapshot().connection_count>" + str(count))
            browser.click("disconnect")
            print(json.dumps({"console_actual_e2e": {"browser": browser.capabilities["browserVersion"], "six_devices_verified": True, "two_jpeg_streams_decoded": True, "lidar_units_frame_stamp_verified": True, "stream_commands_verified": True, "fixture_control_enabled": control, "control_result": "published_in_isolated_fixture_with_watchdog_zero" if control else "control_disabled_rejection", "motion_publications": len(motion), "token_not_exported": True, "stale_pose_cleared": True, "manual_disconnect_cancels_reconnect": True, "hardware_revalidated": False}}, ensure_ascii=False))
        finally:
            browser.click("disconnect")
            node.destroy_node()
            fixture.rclpy.shutdown()
            proc.send_signal(signal.SIGINT)
            try:
                exit_code = proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.terminate()
                proc.wait(timeout=2)
                raise
            assert exit_code == 0, (tmp_path / "bridge.log").read_text()


def test_browser_reconnect_malformed_security_and_bounded_export(browser):
    from websockets.sync.server import serve
    from websockets.exceptions import ConnectionClosed
    connections, requests, errors = [], [], []

    def handler(ws):
        connections.append(True)
        try:
            request = json.loads(ws.recv(timeout=5))
            requests.append(request)
            assert request == {"header": {"robot_id": "tank_inspection_robot", "msg_type": "check"}, "payload": {}}
            if len(connections) == 1:
                ws.close(1000, "fixture_reconnect")
                return
            ws.send("{bad")
            ws.send(json.dumps({"header": {}, "payload": []}))
            ws.send(b"binary_not_supported")
            ws.send("x" * (1024 * 1024 + 1))
            ws.send(json.dumps({"header": {"robot_id": "different_robot", "msg_type": "response"}, "payload": {}}))
            for camera in ("unknown_camera", "fisheye"):
                ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "status_sensor"}, "payload": {"video": {"camera_id": camera, "encoding": "jpeg_base64", "width": 8, "height": 8, "data": "AAAA"}}}))
            for i in range(250):
                ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "fixture_unknown"}, "payload": {"index": i}}))
            ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "status_update"}, "payload": {"devices": {"fisheye": {"status": "unverified", "reason": "<img src=x onerror=window.XSS=1>", "topic": "/fixture", "received": 0, "accepted": 0, "invalid": 0}}}}))
            ws.send(json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": "response"}, "payload": {"echo": {"token": "remote-echo-secret", "items": list(range(10000))}}}))
            for message in ws:
                requests.append(json.loads(message))
        except ConnectionClosed:
            pass
        except BaseException as exc:
            errors.append(repr(exc))

    with serve(handler, "127.0.0.1", 0) as server:
        thread = threading.Thread(target=server.serve_forever)
        thread.start()
        try:
            browser.open(PAGE.as_uri() + "?ws=ws://127.0.0.1:" + str(server.socket.getsockname()[1]))
            browser.click("connect")
            browser.wait("window.tankConsole.snapshot().connection_count>=2", timeout=10)
            browser.wait("window.tankConsole.snapshot().counts.received>=254")
            browser.wait("window.tankConsole.snapshot().streams.fisheye.decode_errors===1")
            # Counted packets can precede the last status/response and DOM render.
            # Await the actual security payload, preserving all original checks.
            browser.wait("window.tankConsole.snapshot().devices.fisheye?.reason?.startsWith('<img') && document.querySelector('#device-fisheye .device-reason')?.textContent.startsWith('<img')")
            browser.wait("window.tankConsole.snapshot().last_response?.echo?.token==='[REDACTED]'")
            record = browser.js("return window.tankConsole.snapshot();")
            assert record["counts"]["invalid"] == 5 and record["counts"]["foreign"] == 1
            assert len(record["logs"]) == 200
            assert browser.js("return document.getElementById('log').children.length;") == 200
            assert not browser.js("return !!window.XSS || !!document.querySelector('#devices img');")
            assert browser.js("return document.querySelector('#device-fisheye .device-reason').textContent;").startswith("<img")
            assert "remote-echo-secret" not in json.dumps(record)
            assert len(json.dumps(record).encode()) < 200000
            assert all(r["header"]["msg_type"] == "check" for r in requests)
            browser.click("disconnect")
            time.sleep(1.2)
            assert len(connections) == 2 and not errors
            print(json.dumps({"console_malformed_e2e": {"connections": len(connections), "invalid_messages": record["counts"]["invalid"], "foreign_robot_filtered": record["counts"]["foreign"], "log_bound": 200, "xss_not_executed": True, "automatic_motion_commands": 0, "no_wan_required": True}}))
        finally:
            browser.click("disconnect")
            server.shutdown()
            thread.join(2)
            assert not thread.is_alive()
