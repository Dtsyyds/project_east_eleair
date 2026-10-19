"""Read-only playback controller. Its child never publishes recorded ROS topics."""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import uuid

import yaml

ROOT = Path(__file__).resolve().parents[2]
ACTIVE = {"starting", "playing", "paused", "stopping"}
SENSORS = {
    "mindvision": ("工业相机", "/camera/front/image_raw", "image"),
    "fisheye": ("鱼眼相机", "/camera/fisheye/image_raw", "image"),
    "d405_color": ("D405 彩色", "/d405/camera/color/image_raw", "image"),
    "d405_depth": ("D405 深度", "/d405/camera/depth/image_rect_raw", "image"),
    "d405_infra1": ("D405 红外 1", "/d405/camera/infra1/image_rect_raw", "image"),
    "d405_infra2": ("D405 红外 2", "/d405/camera/infra2/image_rect_raw", "image"),
    "lidar": ("MID-360 点云", "/livox/lidar", "cloud"),
    "imu": ("IMU", "/livox/imu", "imu"),
    "imu_raw": ("IMU 原始数据", "/livox/imu_raw", "imu"),
    "eddy_raw": ("涡流原始波形", "/eddy/data", "waveform"),
    "eddy_mean": ("涡流通道均值", "/eddy/channel_mean", "means"),
    "emat_raw": ("电磁超声波形", "/inspection/ultrasound/raw", "waveform"),
    "emat_result": ("测厚结果", "/inspection/emat/result", "result"),
}


def manifest(data_root, run_id):
    if not isinstance(run_id, str) or not re.fullmatch(r"WS-[A-Za-z0-9_-]{1,100}", run_id):
        raise ValueError("请选择有效的采集批次")
    runs = (Path(data_root) / "runs").resolve()
    directory = (runs / run_id).resolve()
    if not directory.is_relative_to(runs):
        raise ValueError("采集目录越界")
    registration = json.loads((directory / "run.json").read_text())
    if registration.get("status") not in ("complete", "incomplete", "failed"):
        raise ValueError("该批次尚未停止并关闭文件")
    bag = (directory / "bag").resolve()
    if not bag.is_relative_to(directory):
        raise ValueError("MCAP 目录越界")
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    if metadata.get("storage_identifier") != "mcap":
        raise ValueError("该批次不是 MCAP 数据")
    files = metadata.get("relative_file_paths", [])
    if not files:
        raise ValueError("该批次没有 MCAP 分片")
    for name in files:
        path = (bag / name).resolve()
        if not path.is_relative_to(bag):
            raise ValueError("MCAP 分片路径越界")
        with path.open("rb") as stream:
            if stream.read(8) != b"\x89MCAP0\r\n":
                raise ValueError("MCAP 分片格式错误：" + path.name)
            stream.seek(-8, os.SEEK_END)
            if stream.read(8) != b"\x89MCAP0\r\n":
                raise ValueError("MCAP 分片尚未正常关闭：" + path.name)
    topics = [{"topic": item["topic_metadata"]["name"], "type": item["topic_metadata"]["type"],
               "count": int(item["message_count"])} for item in metadata["topics_with_message_count"]]
    counts = {t["topic"]: t["count"] for t in topics}
    sensors = [{"id": ident, "label": label, "topic": topic, "kind": kind, "count": counts.get(topic, 0)}
               for ident, (label, topic, kind) in SENSORS.items()]
    return {"run_id": run_id, "bag_directory": str(bag), "duration_seconds": metadata["duration"]["nanoseconds"] / 1e9,
            "start_ns": metadata["starting_time"]["nanoseconds_since_epoch"], "files": files,
            "topics": topics, "sensors": sensors, "capture_complete": registration.get("complete", False)}


class ReplayController:
    def __init__(self, config, emit):
        self.config, self.emit = config, emit
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.state = {"status": "idle", "message": "选择历史批次后点击一键回放。"}
        self.latest = {}
        self.process = None
        self.thread = None
        self.stderr = None
        self.ready = threading.Event()
        self.closed = False

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def start(self, parameters):
        info = manifest(self.config["data_root"], parameters.get("run_id"))
        rate = parameters.get("rate", 1)
        if isinstance(rate, bool) or rate not in (.5, 1, 2):
            raise ValueError("请选择 0.5、1 或 2 倍速")
        with self.lock:
            if self.closed:
                raise RuntimeError("系统正在关闭")
            if self.process and self.process.poll() is None:
                raise RuntimeError("已有回放正在进行，请先停止回放")
            if self.stderr:
                self.stderr.close()
            session = uuid.uuid4().hex
            runtime = ROOT / "tank_inspection_system/.runtime/system_websocket"
            runtime.mkdir(parents=True, exist_ok=True)
            self.stderr = (runtime / ("replay-" + session + ".log")).open("wb")
            self.state = {**info, "session_id": session, "status": "starting", "rate": rate,
                          "position_seconds": 0, "progress_percent": 0, "preview_messages": 0,
                          "message": "正在打开采集文件…"}
            self.ready.clear()
            self.latest.clear()
            try:
                self.process = subprocess.Popen([sys.executable, str(ROOT / "system_websocket/src/replay_worker.py"),
                                                str(info["bag_directory"]), session, str(rate)],
                                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=self.stderr,
                                               text=True, bufsize=1, start_new_session=True)
            except OSError as error:
                self.state.update(status="failed", message="无法启动回放：" + str(error))
                self.stderr.close()
                self.stderr = None
                raise
            self.thread = threading.Thread(target=self.read, args=(self.process, session), daemon=True)
            self.thread.start()
        if not self.ready.wait(15):
            self.stop()
            raise RuntimeError("回放文件打开超时")
        state = self.snapshot()
        if state["status"] == "failed":
            raise RuntimeError(state["message"])

    def read(self, process, session):
        try:
            for line in process.stdout:
                if len(line) > 1024 * 1024:
                    raise RuntimeError("回放预览消息超出大小限制")
                if not line.startswith("@REPLAY@"):
                    continue
                packet = json.loads(line[len("@REPLAY@"):])
                with self.lock:
                    if self.state.get("session_id") != session:
                        return
                    if packet.get("kind") == "state":
                        self.state.update(packet["data"])
                        self.changed.notify_all()
                        self.ready.set()
                        continue
                    if packet.get("kind") != "sensor" or self.state["status"] == "stopping":
                        continue
                    self.state["preview_messages"] += 1
                    self.latest[packet["data"]["sensor_id"]] = packet["data"]
                    if len(self.latest) > 64:
                        self.latest.pop(next(iter(self.latest)))
                self.emit("replay_sensor", packet["data"])
            code = process.wait()
            with self.lock:
                if self.state.get("session_id") == session and self.state["status"] in ACTIVE:
                    self.state.update(status="failed" if code else "stopped",
                                      message="回放进程异常退出，请检查回放日志。" if code else "回放已停止。")
        except Exception as error:
            with self.lock:
                if self.state.get("session_id") == session:
                    self.state.update(status="failed", message=str(error))
        finally:
            process.stdout.close()
            self.ready.set()

    def command(self, command):
        with self.lock:
            if not self.process or self.process.poll() is not None:
                raise RuntimeError("当前没有正在回放的批次")
            if command == "pause" and self.state["status"] != "playing":
                raise RuntimeError("当前回放不能暂停")
            if command == "resume" and self.state["status"] != "paused":
                raise RuntimeError("当前回放没有暂停")
            self.process.stdin.write(json.dumps({"command": command}) + "\n")
            self.process.stdin.flush()
            expected = "paused" if command == "pause" else "playing"
            if not self.changed.wait_for(lambda: self.state["status"] == expected or self.process.poll() is not None, timeout=3):
                raise RuntimeError("回放控制应答超时")
            if self.state["status"] != expected:
                raise RuntimeError("回放已结束")

    def restore(self):
        with self.lock:
            previews = list(self.latest.values())
        for preview in previews:
            self.emit("replay_sensor", preview)

    def stop(self):
        with self.lock:
            process = self.process
            if not process:
                return
            self.state.update(status="stopping", message="正在停止回放…")
            if process.poll() is None:
                try:
                    process.stdin.write('{"command":"stop"}\n')
                    process.stdin.flush()
                except (BrokenPipeError, OSError):
                    pass
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        if self.thread:
            self.thread.join(timeout=3)
        with self.lock:
            process.stdin.close()
            self.process = None
            if self.stderr:
                self.stderr.close()
                self.stderr = None
            self.state.update(status="stopped", message="回放已停止。")

    def close(self):
        with self.lock:
            self.closed = True
        self.stop()
