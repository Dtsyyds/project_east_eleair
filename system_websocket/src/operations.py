"""Owned MCAP recorder and fresh EMAT measurement; no shell commands from clients."""
from __future__ import annotations

import concurrent.futures
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import threading
import time
import uuid
from collections import OrderedDict
from types import SimpleNamespace

import yaml
from replay import ReplayController, ACTIVE as REPLAY_ACTIVE

ROOT = Path(__file__).resolve().parents[2]
MCAP_MAGIC = b"\x89MCAP0\r\n"


def mcap_closed(path):
    try:
        with path.open("rb") as stream:
            if stream.read(8) != MCAP_MAGIC:
                return False
            stream.seek(-8, os.SEEK_END)
            return stream.read(8) == MCAP_MAGIC
    except OSError:
        return False


def load_module(name, path):
    import sys
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def save_json(path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


class Operations:
    ACTIONS = ("collection.preflight", "collection.start", "collection.stop", "collection.status", "emat.measure", "history.list",
               "replay.start", "replay.pause", "replay.resume", "replay.stop", "replay.status")

    def __init__(self, config, emit, live_topics, publish_ultrasound=None):
        self.config, self.emit, self.live_topics = config, emit, live_topics
        self.publish_ultrasound = publish_ultrasound
        self.lock = threading.RLock()
        self.collection_lock = threading.Lock()
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=3)
        self.requests = OrderedDict()
        self.revision = 0
        self.service_id = uuid.uuid4().hex
        self.task = {"status": "idle", "message": "设备预览已启动，点击一键启动采集。"}
        self.measurement = {"status": "idle", "message": "点击一键测厚，采集新波形并运行算法。"}
        self.recorder = None
        self.recorder_log = None
        self.started = None
        self.run_dir = None
        self.collection_cancel = threading.Event()
        self.measure_cancel = threading.Event()
        self.closed = False
        self.failure_reason = ""
        self.replay_start_pending = False
        self.replay = ReplayController(config, self.packet)
        self.serial_api = load_module("inspection_emat_serial", ROOT / "physic_bev/src/emat/emat_serial_once.py")
        self.algorithm = load_module("inspection_thickness_algorithm", ROOT / "physic_bev/src/emat/jiancepy/measure_thickness.py")

    def packet(self, kind, payload):
        self.emit({"header": {"robot_id": "tank_inspection_robot", "msg_type": kind}, "payload": payload})

    def storage(self):
        root = Path(self.config["data_root"])
        return {"mounted": root.is_mount(), "free_bytes": shutil.disk_usage(root).free,
                "minimum_free_bytes": self.config["minimum_free_bytes"], "path": str(root)}

    def snapshot(self):
        with self.lock:
            self.revision += 1
            task = copy.deepcopy(self.task)
            task.update(revision=self.revision, service_id=self.service_id)
            measurement = copy.deepcopy(self.measurement)
            measurement.update(revision=self.revision, service_id=self.service_id)
            replay = self.replay.snapshot()
            replay.update(revision=self.revision, service_id=self.service_id)
            if self.started is not None and task["status"] in ("recording", "stopping", "finalizing"):
                task["elapsed_seconds"] = time.monotonic() - self.started
            if self.run_dir:
                task["bytes_written"] = sum(p.stat().st_size for p in self.run_dir.glob("bag/*.mcap"))
            return {"capabilities": {"protocol_version": 3, "robot_id": "tank_inspection_robot", "mode": "hardware",
                    "operations": {name: {"supported": True, "timeout_ms": 120000} for name in self.ACTIONS},
                    "collection": {"profiles": [{"id": "full_system", "label": "整机采集"}]}},
                    "task": task, "storage": self.storage(), "measurement": measurement, "replay": replay}

    def response(self, request, status, **extra):
        result = {"request_id": request["request_id"], "client_id": request.get("client_id", ""),
                  "action": request["action"], "status": status, **extra}
        with self.lock:
            key = (request.get("client_id", ""), request["request_id"])
            self.requests[key] = result
            while len(self.requests) > 128:
                self.requests.popitem(last=False)
        self.packet("response", result)

    def handle(self, request):
        if not isinstance(request, dict):
            return
        ident, action = request.get("request_id"), request.get("action")
        if not isinstance(ident, str) or not 1 <= len(ident) <= 128 or not isinstance(action, str):
            return
        client = request.get("client_id", "")
        if not isinstance(client, str) or len(client) > 128:
            return
        parameters = request.get("parameters", {})
        if not isinstance(parameters, dict):
            self.response(request, "failed", error="参数格式错误")
            return
        with self.lock:
            previous = self.requests.get((client, ident))
            if previous:
                self.packet("response", previous)
                return
            if self.closed or action not in self.ACTIONS:
                self.response(request, "failed", error="不支持该操作")
                return
            if action in ("collection.status", "replay.status"):
                self.response(request, "completed", **self.snapshot())
                if action == "replay.status":
                    self.replay.restore()
                return
            if action == "history.list":
                files = sorted((Path(self.config["data_root"]) / "runs").glob("WS-*/run.json"), reverse=True)[:50]
                history = []
                for path in files:
                    try:
                        history.append(json.loads(path.read_text()))
                    except (OSError, ValueError):
                        continue
                self.response(request, "completed", history=history)
                return
            if action.startswith("replay."):
                if action == "replay.start" and self.task["status"] in ("preflighting", "starting", "recording", "stopping", "finalizing"):
                    self.response(request, "failed", error="请先停止采集再回放")
                    return
                if action == "replay.start":
                    if self.replay_start_pending or self.replay.snapshot()["status"] in REPLAY_ACTIVE:
                        self.response(request, "failed", error="已有回放正在进行，请先停止回放")
                        return
                    self.replay_start_pending = True
                self.response(request, "accepted")
                self.pool.submit(self.work, request)
                return
            if action == "emat.measure":
                if parameters.get("source", "live") not in ("live", "saved"):
                    self.response(request, "failed", error="未知测厚数据来源")
                    return
                if self.measurement["status"] in ("running", "stopping"):
                    self.response(request, "failed", error="测厚正在进行")
                    return
                self.measure_cancel.clear()
                self.measurement = {"status": "running", "progress_percent": 0, "message": "正在测厚…",
                                    "source": parameters.get("source", "live"), "request_id": ident}
            elif action == "collection.stop":
                supplied_run = parameters.get("run_id")
                if supplied_run and supplied_run != self.task.get("run_id"):
                    self.response(request, "failed", error="采集批次不匹配")
                    return
                self.collection_cancel.set()
                self.measure_cancel.set()
                if self.measurement["status"] == "running":
                    self.measurement.update(status="stopping", message="正在停止测厚…")
                if self.task["status"] in ("stopping", "finalizing"):
                    self.response(request, "failed", error="正在停止并封存")
                    return
                if self.task["status"] in ("starting", "recording"):
                    self.task.update(status="stopping", message="正在关闭记录并封存…")
            else:
                if self.replay_start_pending or self.replay.snapshot()["status"] in REPLAY_ACTIVE:
                    self.response(request, "failed", error="请先停止回放再启动采集")
                    return
                if self.recorder is not None:
                    self.response(request, "failed", error="上一批记录进程尚未关闭，请先点击一键停止")
                    return
                if self.task["status"] in ("starting", "recording", "stopping", "finalizing", "preflighting"):
                    self.response(request, "failed", error="已有采集任务正在运行")
                    return
                self.collection_cancel.clear()
                self.failure_reason = ""
                self.run_dir = None
                self.started = None
                self.task = {"status": "starting" if action == "collection.start" else "preflighting"}
            self.response(request, "accepted")
            self.pool.submit(self.work, request)

    def work(self, request):
        action = request["action"]
        try:
            if action.startswith("replay."):
                if action == "replay.start":
                    self.replay.start(request.get("parameters", {}))
                elif action == "replay.stop":
                    self.replay.stop()
                else:
                    self.replay.command(action.split(".")[1])
            elif action == "emat.measure":
                if request.get("parameters", {}).get("source") == "saved":
                    self.measure_saved()
                else:
                    self.measure()
            else:
                with self.collection_lock:
                    if action == "collection.stop":
                        self.stop()
                    else:
                        self.preflight()
                        if self.collection_cancel.is_set():
                            raise RuntimeError("采集启动已取消")
                        if action == "collection.start":
                            self.start(request.get("parameters", {}))
                        else:
                            with self.lock:
                                self.task.update(status="ready", message="预检通过，可一键启动采集。")
            self.response(request, "completed", **self.snapshot())
        except Exception as error:
            if action.startswith("replay."):
                self.response(request, "failed", error=str(error), **self.snapshot())
                return
            if action == "collection.start" and self.recorder is not None:
                self.stop_guarded()
            with self.lock:
                target = self.measurement if action == "emat.measure" else self.task
                target.update(status="failed", error=str(error), message=str(error))
                if action == "emat.measure":
                    target.pop("thickness_mm", None)
                if self.run_dir and action != "emat.measure":
                    self.persist()
            self.response(request, "failed", error=str(error), **self.snapshot())
        finally:
            if action == "replay.start":
                with self.lock:
                    self.replay_start_pending = False

    def preflight(self):
        storage = self.storage()
        now = time.monotonic()
        live = self.live_topics()
        checks = [{"label": "采集盘", "status": "passed" if storage["mounted"] else "failed"},
                  {"label": "剩余空间", "status": "passed" if storage["free_bytes"] >= self.config["minimum_free_bytes"] else "failed"}]
        checks += [{"label": topic, "status": "passed" if now - live.get(topic, -math.inf) < 3 else "failed"}
                   for topic in self.config["required_topics"]]
        with self.lock:
            self.task["preflight_checks"] = checks
        missing = [item["label"] for item in checks if item["status"] == "failed"]
        if missing:
            raise RuntimeError("采集预检失败：" + "、".join(missing))

    def start(self, parameters):
        profile = parameters.get("profile")
        if profile not in (None, "", "full_system"):
            raise ValueError("未知采集方案")
        run_id = "WS-" + time.strftime("%Y%m%d_%H%M%S") + "-" + uuid.uuid4().hex[:8]
        directory = Path(self.config["data_root"]) / "runs" / run_id
        directory.mkdir(parents=True)
        share = ROOT / "physic_bev/src/inspection_bringup/config"
        topics = yaml.safe_load((share / "record_topics.yaml").read_text())
        cmd = ["ros2", "bag", "record", "--storage", "mcap", "--output", str(directory / "bag"),
               "--max-bag-size", str(topics["production"]["max_bag_size"]), "--max-bag-duration", "900",
               "--max-cache-size", "268435456", "--storage-config-file", str(share / "mcap_writer_options.yaml"),
               "--qos-profile-overrides-path", str(share / "qos_overrides.yaml"), *topics["topics"],
               "/inspection/emat/result"]
        with self.lock:
            self.run_dir = directory
            self.started = None
            self.task.update(status="starting", run_id=run_id, run_directory=str(directory),
                             started_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"), message="正在创建采集文件…",
                             metadata={key: str(parameters.get(key, ""))[:500] for key in ("task_name", "asset_id", "seam_id", "operator_note")})
            if self.collection_cancel.is_set():
                raise RuntimeError("采集启动已取消")
            self.recorder_log = (directory / "recorder.log").open("wb")
            try:
                self.recorder = subprocess.Popen(cmd, stdout=self.recorder_log, stderr=subprocess.STDOUT, start_new_session=True)
            except Exception:
                self.recorder_log.close()
                raise
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if self.collection_cancel.is_set():
                self.stop()
                raise RuntimeError("采集启动已取消")
            if self.recorder.poll() is not None:
                raise RuntimeError("记录进程启动失败，请查看 " + str(directory / "recorder.log"))
            if list((directory / "bag").glob("*.mcap")):
                with self.lock:
                    self.started = time.monotonic()
                    self.task.update(status="recording", message="正在采集，数据目录：" + str(directory),
                                     mcap_status="recording", video_status="recorded_in_mcap")
                    self.persist()
                return
            self.collection_cancel.wait(.1)
        self.stop()
        raise RuntimeError("记录文件创建超时")

    def persist(self):
        if self.run_dir:
            record = copy.deepcopy(self.task)
            record["path_hint"] = str(self.run_dir)
            record["finalize_status"] = record["status"]
            save_json(self.run_dir / "run.json", record)

    def stop(self):
        with self.lock:
            process = self.recorder
            if process is None:
                return
            self.task.update(status="stopping", message="正在正常关闭 MCAP 写入…")
        forced = False
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                forced = True
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
        with self.lock:
            if self.recorder_log:
                self.recorder_log.close()
                self.recorder_log = None
            self.recorder = None
            self.task.update(status="finalizing", message="正在检查各设备的实际记录数量…",
                             elapsed_seconds=time.monotonic() - self.started if self.started else 0)
        metadata_path = self.run_dir / "bag/metadata.yaml"
        counts = {}
        files_ok = False
        if metadata_path.exists():
            metadata = yaml.safe_load(metadata_path.read_text())["rosbag2_bagfile_information"]
            counts = {item["topic_metadata"]["name"]: item["message_count"] for item in metadata["topics_with_message_count"]}
            files = metadata.get("relative_file_paths", [])
            files_ok = bool(files) and all(mcap_closed(self.run_dir / "bag" / p) for p in files)
        missing = [topic for topic in self.config["required_topics"] if counts.get(topic, 0) <= 0]
        complete = files_ok and not missing and not forced and process.returncode == 0 and not self.failure_reason
        with self.lock:
            self.task.update(status="complete" if complete else "incomplete", complete=complete,
                             topic_counts=counts, writer_closed=not forced, mcap_valid=files_ok,
                             mcap_validation="file_footer_and_topic_metadata",
                             mcap_status="closed" if files_ok else "invalid", video_status="recorded_in_mcap",
                             message="采集已停止，文件已保存：" + str(self.run_dir) if complete else
                             "采集已停止，数据未完整：" + (self.failure_reason or "、".join(missing) or "MCAP 未正常关闭"),
                             finalize_checks=[{"label": "writer 关闭", "status": "passed" if not forced and process.returncode == 0 else "failed"},
                                              {"label": "MCAP 文件", "status": "passed" if files_ok else "failed"}] +
                                             [{"label": t, "status": "passed" if counts.get(t, 0) > 0 else "failed"} for t in self.config["required_topics"]])
            self.persist()

    def measure(self):
        import serial
        settings = self.config["emat"]
        device = Path(settings["serial_port"])
        if not device.exists():
            raise RuntimeError("电磁超声串口未连接：" + str(device))
        eddy = Path("/dev/eddy")
        if eddy.exists() and device.resolve() == eddy.resolve():
            raise RuntimeError("电磁超声与涡流串口冲突")
        count = int(settings["sample_count"])
        if not 256 <= count <= 65536:
            raise ValueError("电磁超声采样数量超出范围")
        directory = Path(self.config["data_root"]) / "measurements" / (time.strftime("%Y%m%d_%H%M%S") + "-" + uuid.uuid4().hex[:8])
        storage = self.storage()
        if not storage["mounted"] or storage["free_bytes"] < self.config["minimum_free_bytes"]:
            raise RuntimeError("采集盘未挂载或空间不足")
        directory.mkdir(parents=True)
        args = SimpleNamespace(raw_hex_trace=False, write_timeout_ms=500, ack_timeout_ms=500, inter_command_delay_ms=800)
        data = bytearray()
        with serial.Serial(str(device), baudrate=int(settings["baudrate"]), timeout=.05, write_timeout=.5, exclusive=True) as port:
            try:
                self.serial_api.run_start(port, args)
            except self.serial_api.ReadTimeout as error:
                raise RuntimeError("电磁超声启动/清零应答超时，请检查设备供电和串口连接") from error
            interval = float(settings["read_interval_ms"]) / 1000
            if not math.isfinite(interval) or interval < 0:
                raise ValueError("读取间隔无效")
            previous = None
            last_report = 0
            try:
                for index in range(count):
                    if self.measure_cancel.is_set():
                        raise RuntimeError("测厚已停止")
                    if previous is not None:
                        self.measure_cancel.wait(max(0, previous + interval - time.monotonic()))
                    if self.measure_cancel.is_set():
                        raise RuntimeError("测厚已停止")
                    previous = time.monotonic()
                    self.serial_api.write_exactly(port, b"\x00", .5)
                    data.extend(self.serial_api.read_exactly(port, 1, .5))
                    if previous - last_report > .5:
                        with self.lock:
                            self.measurement.update(progress_percent=(index + 1) / count * 95,
                                                    message=f"正在读取波形 {index + 1}/{count}…")
                        last_report = previous
            except Exception:
                (directory / "waveform.partial.txt").write_text(" ".join(f"{v:02X}" for v in data), encoding="ascii")
                raise
        (directory / "waveform.txt").write_text(" ".join(f"{v:02X}" for v in data), encoding="ascii")
        if self.publish_ultrasound:
            self.publish_ultrasound(list(data), float(settings["sampling_rate_hz"]))
        self.analyse_waveform(list(data), directory / "waveform.txt", "live")

    def measure_saved(self):
        settings = self.config["emat"]
        # Last successfully acquired live waveform takes priority over the
        # configured project waveform. Never load partial data.
        captures = sorted((Path(self.config["data_root"]) / "measurements").glob("*/waveform.txt"),
                          key=lambda p: p.stat().st_mtime, reverse=True)
        path = captures[0] if captures else ROOT / settings["saved_waveform"]
        self.analyse_waveform(self.algorithm.read_signal(path), path, "saved")

    def analyse_waveform(self, data, path, source):
        settings = self.config["emat"]
        for key in ("sound_speed_m_s", "sampling_rate_hz"):
            if not math.isfinite(float(settings[key])) or float(settings[key]) <= 0:
                raise ValueError("声速或采样率无效")
        analysis = self.algorithm.analyse(list(data), float(settings["sound_speed_m_s"]),
                                          float(settings["sampling_rate_hz"]), float(settings["adc_zero"]), int(settings["start_sample"]))
        thickness = float(analysis["thickness_mm"])
        if not math.isfinite(thickness) or thickness <= 0:
            raise RuntimeError("测厚算法未得到有效结果")
        if self.measure_cancel.is_set():
            raise RuntimeError("测厚已停止")
        result = {"status": "completed", "thickness_mm": thickness, "unit": "mm", "progress_percent": 100,
                  "measurement_id": uuid.uuid4().hex,
                  "request_id": self.measurement.get("request_id"),
                  "sample_count": len(data), "sound_speed_m_s": settings["sound_speed_m_s"],
                  "sampling_rate_hz": settings["sampling_rate_hz"], "echo_count": len(analysis["echo_peaks"]),
                  "calibrated": bool(settings["calibrated"]), "waveform_path": str(path), "source": source,
                  "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                  "message": f"算法结果：{thickness:.4f} mm · " + ("实时采集" if source == "live" else "已有波形：" + path.name)}
        if source == "live":
            save_json(path.with_name("result.json"), result)
        else:
            storage = self.storage()
            if not storage["mounted"] or storage["free_bytes"] < self.config["minimum_free_bytes"]:
                raise RuntimeError("采集盘未挂载或空间不足")
            results = Path(self.config["data_root"]) / "measurements/analysis"
            results.mkdir(parents=True, exist_ok=True)
            save_json(results / (time.strftime("%Y%m%d_%H%M%S") + "-" + uuid.uuid4().hex[:8] + ".json"), result)
        with self.lock:
            self.measurement = result

    def tick(self):
        with self.lock:
            if self.recorder and self.task["status"] == "recording":
                reason = ""
                storage = self.storage()
                if self.recorder.poll() is not None:
                    reason = "记录进程异常退出"
                elif not storage["mounted"]:
                    reason = "采集盘已卸载，已自动停止"
                elif storage["free_bytes"] < self.config["minimum_free_bytes"]:
                    reason = "采集盘空间不足，已自动停止"
                else:
                    now = time.monotonic()
                    missing = [t for t in self.config["required_topics"] if now - self.live_topics().get(t, -math.inf) > 10]
                    if missing:
                        reason = "设备数据中断：" + "、".join(missing)
                if reason:
                    self.failure_reason = reason
                    self.task.update(status="stopping", message=reason)
                    self.pool.submit(self.stop_guarded)
        self.packet("status_update", self.snapshot())

    def stop_guarded(self):
        try:
            with self.collection_lock:
                self.stop()
        except Exception as error:
            with self.lock:
                self.task.update(status="failed", message=str(error))
                self.persist()

    def close(self):
        with self.lock:
            self.closed = True
            self.collection_cancel.set()
            self.measure_cancel.set()
        self.pool.shutdown(wait=True)
        self.replay.close()
        self.stop_guarded()
