#!/usr/bin/env python3
"""Single foreground owner of hardware, operations, bridge and local console."""
import fcntl
import functools
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]


class QuietHandler(SimpleHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def handle(self):
        try:
            super().handle()
        except (BrokenPipeError, ConnectionResetError):
            pass  # A browser can close a request while refreshing or leaving.


def stop_process(process, timeout=15):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout)
    except ProcessLookupError:
        return
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=10)


def main():
    config = yaml.safe_load((ROOT / "system_websocket/config/system.yaml").read_text())
    os.environ["ROS_DOMAIN_ID"] = str(config["ros_domain_id"])
    runtime = ROOT / "tank_inspection_system/.runtime/system_websocket"
    runtime.mkdir(parents=True, exist_ok=True)
    owner = (runtime / "owner.lock").open("a")
    try:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("系统已经运行，请直接打开上位机页面。", file=sys.stderr)
        return 2
    # Preflight both listening ports before starting any hardware.
    with socket.socket() as test:
        test.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            test.bind(("0.0.0.0", int(config["websocket_port"])))
        except OSError:
            print("WebSocket 端口已被占用，请先关闭已有桥接实例。", file=sys.stderr)
            return 2
    server = ThreadingHTTPServer(("0.0.0.0", int(config["http_port"])),
                                functools.partial(QuietHandler, directory=str(ROOT / "websocket_console")))
    server.daemon_threads = True
    # Never create a second device owner in the selected DDS domain.
    existing = subprocess.run(["ros2", "node", "list", "--no-daemon"], capture_output=True, text=True, timeout=12)
    if existing.returncode != 0:
        server.server_close()
        raise RuntimeError("无法检查 ROS 设备占用：" + existing.stderr.strip())
    names = existing.stdout.splitlines()
    owned_names = {"/camera/front", "/camera/fisheye", "/d405/camera", "/eddy_driver_node", "/livox_lidar_publisher", "/inspection_operations", "/websocket_bridge_node"}
    conflicts = sorted(owned_names.intersection(names))
    if conflicts:
        server.server_close()
        raise RuntimeError("已有设备进程正在运行，请关闭原启动终端后重试：" + "、".join(conflicts))
    bridge_config = yaml.safe_load((ROOT / "physic_bev/src/system_websocket/agv_bridge/config/tank_inspection.yaml").read_text())
    params = bridge_config["websocket_bridge_node"]["ros__parameters"]
    params.update({"local_server_port": int(config["websocket_port"]),
                   "tank_inspection.operations_topic": "/inspection/operations",
                   "tank_inspection.emat_topic": "/inspection/ultrasound/raw"})
    (runtime / "bridge.yaml").write_text(yaml.safe_dump(bridge_config), encoding="utf-8")
    processes, logs = {}, []
    server_started = False
    def launch(name, command):
        log = (runtime / (name + ".log")).open("wb")
        logs.append(log)
        processes[name] = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    stop = threading.Event()
    def request_stop(*_):
        stop.set()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        hardware = ["ros2", "launch", "inspection_bringup", "hardware_pipeline.launch.py", "record:=false"]
        hardware += [key + ":=" + (str(value).lower() if isinstance(value, bool) else str(value)) for key, value in config["hardware"].items()]
        launch("hardware", hardware)
        launch("operations", [sys.executable, str(ROOT / "system_websocket/src/operations_node.py")])
        launch("bridge", ["ros2", "launch", "agv_bridge", "tank_inspection.launch.py", "config:=" + str(runtime / "bridge.yaml")])
        threading.Thread(target=server.serve_forever, daemon=True).start()
        server_started = True
        addresses = subprocess.run(["ip", "-j", "-4", "address", "show"], capture_output=True, text=True, check=True)
        import json
        interfaces = json.loads(addresses.stdout)
        ips = [a["local"] for interface in interfaces if interface.get("operstate") == "UP"
               for a in interface.get("addr_info", []) if a.get("scope") == "global"]
        for address in ips or ["127.0.0.1"]:
            print(f"上位机：http://{address}:{config['http_port']}", flush=True)
        print("点击“一键启动采集 / 一键停止 / 一键测厚 / 一键回放”。Ctrl+C 关闭整套系统。", flush=True)
        while not stop.wait(.5):
            for name, process in processes.items():
                if process.poll() is not None:
                    raise RuntimeError(f"{name} 进程退出，详情：{runtime / (name + '.log')}")
    finally:
        # Operations flushes its recorder before camera streams are stopped.
        errors = []
        for name in ("operations", "bridge", "hardware"):
            if name in processes:
                try:
                    stop_process(processes[name], 45 if name == "operations" else 20)
                except (OSError, subprocess.TimeoutExpired) as error:
                    errors.append(f"{name} 关闭失败：{error}")
        if server_started:
            server.shutdown()
        server.server_close()
        for log in logs:
            log.close()
        owner.close()
        if errors:
            raise RuntimeError("；".join(errors))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(2)
