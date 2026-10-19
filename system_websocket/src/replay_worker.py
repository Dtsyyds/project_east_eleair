#!/usr/bin/env python3
"""Stream bounded, timestamp-paced previews from a split MCAP bag over stdio."""
from __future__ import annotations

import base64
from array import array
from collections import Counter
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
import yaml

from replay import SENSORS
STDOUT_LOCK = threading.Lock()


def finite_list(values):
    return [float(v) if math.isfinite(float(v)) else None for v in values]


def bounded(value, depth=0):
    if depth > 6:
        return "…"
    if isinstance(value, dict):
        return {str(k): bounded(v, depth + 1) for k, v in list(value.items())[:40]}
    if isinstance(value, (list, tuple, np.ndarray, array)):
        return [bounded(v, depth + 1) for v in value[:48]]
    if isinstance(value, str):
        return value[:3000]
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def image_preview(msg):
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1,
                "8UC1": 1, "16UC1": 1, "mono16": 1, "32FC1": 1}
    if msg.encoding not in channels or msg.width <= 0 or msg.height <= 0:
        raise ValueError("不支持的图像编码：" + msg.encoding)
    depth = msg.encoding in ("16UC1", "mono16", "32FC1")
    dtype = np.dtype((">" if msg.is_bigendian else "<") + ("f4" if msg.encoding == "32FC1" else "u2" if depth else "u1"))
    count = channels[msg.encoding]
    if len(msg.data) != msg.step * msg.height or msg.step < msg.width * count * dtype.itemsize:
        raise ValueError("图像行宽或数据长度无效")
    pixels = np.ndarray((msg.height, msg.width, count), dtype=dtype, buffer=msg.data,
                        strides=(msg.step, count * dtype.itemsize, dtype.itemsize))
    if count == 1:
        pixels = pixels[:, :, 0]
    scale = min(1, 640 / msg.width)
    preview = cv2.resize(pixels.astype(dtype.newbyteorder("="), copy=False),
                         (max(1, round(msg.width * scale)), max(1, round(msg.height * scale))),
                         interpolation=cv2.INTER_NEAREST if depth else cv2.INTER_AREA)
    if depth:
        valid = np.isfinite(preview) & (preview > 0)
        if valid.any():
            low, high = np.percentile(preview[valid], (2, 98))
            scaled = np.clip((np.nan_to_num(preview.astype(np.float32)) - low) * 255 / max(float(high - low), 1), 0, 255).astype(np.uint8)
            preview = cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
            preview[~valid] = 0
        else:
            preview = np.zeros((*preview.shape, 3), dtype=np.uint8)
    elif msg.encoding == "rgb8":
        preview = cv2.cvtColor(preview, cv2.COLOR_RGB2BGR)
    elif msg.encoding == "rgba8":
        preview = cv2.cvtColor(preview, cv2.COLOR_RGBA2BGR)
    elif msg.encoding == "bgra8":
        preview = cv2.cvtColor(preview, cv2.COLOR_BGRA2BGR)
    ok, jpeg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if not ok or len(jpeg) > 600000:
        raise ValueError("图像预览编码失败或过大")
    return {"kind": "image", "jpeg": base64.b64encode(jpeg).decode("ascii"), "width": preview.shape[1],
            "height": preview.shape[0], "source_width": msg.width, "source_height": msg.height,
            "source_encoding": msg.encoding, "depth_visualization": depth}


def pointcloud_preview(msg):
    fields = {f.name: f for f in msg.fields}
    sizes = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}
    if not all(axis in fields for axis in ("x", "y", "z")) or msg.point_step <= 0:
        raise ValueError("点云缺少 XYZ 字段")
    if msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError("点云数据布局无效")
    names, formats, offsets = [], [], []
    for axis in ("x", "y", "z"):
        field = fields[axis]
        if field.datatype not in sizes or field.count != 1:
            raise ValueError("XYZ 字段类型不支持")
        names.append(axis)
        formats.append((">" if msg.is_bigendian else "<") + sizes[field.datatype])
        offsets.append(field.offset)
    dtype = np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step})
    raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data, strides=(msg.row_step, msg.point_step))
    stride = max(1, math.ceil(msg.width * msg.height / 5000))
    points = np.column_stack([raw[a].reshape(-1)[::stride] for a in names]).astype(np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    return {"kind": "cloud", "xyz": np.round(points, 4).ravel().tolist(), "point_count": msg.width * msg.height,
            "displayed_points": len(points), "unit": "m"}


def sensor_preview(sensor_id, msg):
    if sensor_id.startswith("d405_") or sensor_id in ("mindvision", "fisheye"):
        return image_preview(msg)
    if sensor_id == "lidar":
        return pointcloud_preview(msg)
    if sensor_id in ("imu", "imu_raw"):
        return {"kind": "imu", "acceleration": finite_list([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]),
                "gyro": finite_list([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]),
                "unit": "m/s² · rad/s" if sensor_id == "imu" else "驱动原始单位"}
    if sensor_id == "eddy_raw":
        if msg.channel_count != 8 or msg.samples_per_channel != 20:
            raise ValueError("涡流帧结构无效")
        values = np.asarray(msg.data).reshape(msg.samples_per_channel, msg.channel_count).T
        return {"kind": "waveform", "channels": values.tolist(), "unit": "mV", "sample_count": msg.samples_per_channel,
                "sequence": msg.sequence, "driver_dropped_frames": msg.dropped_frames,
                "excitation_amplitude": msg.excitation_amplitude, "excitation_frequency": msg.excitation_frequency}
    if sensor_id == "eddy_mean":
        return {"kind": "means", "values": finite_list(msg.channel_mean), "unit": "mV"}
    if sensor_id == "emat_raw":
        if not msg.sample_count or not msg.channel_count or len(msg.samples) != msg.sample_count * msg.channel_count:
            raise ValueError("电磁超声波形布局无效")
        raw = np.asarray(msg.samples).reshape(msg.channel_count, msg.sample_count)
        stride = max(1, math.ceil(msg.sample_count / 4096))
        return {"kind": "waveform", "channels": raw[:8, ::stride].tolist(), "sample_count": msg.sample_count,
                "sampling_rate_hz": msg.sampling_rate_hz, "unit": msg.sample_encoding or "ADC", "display_stride": stride}
    if sensor_id == "emat_result":
        try:
            return {"kind": "result", "result": bounded(json.loads(msg.data))}
        except ValueError:
            return {"kind": "result", "result": {"message": msg.data[:3000]}}
    return {"kind": "generic", "message": bounded(message_to_ordereddict(msg))}


class Playback:
    def __init__(self, bag, session, rate, output=None):
        self.bag, self.session, self.rate = Path(bag), session, rate
        self.output = output or self.stdout
        self.condition = threading.Condition()
        self.stop_requested = False
        self.paused = False
        self.pause_started = 0
        self.wall_origin = 0
        self.position = 0
        self.errors = Counter()

    @staticmethod
    def stdout(packet):
        # Native rosbag2 log lines use stderr; stdout is the IPC protocol only.
        with STDOUT_LOCK:
            print("@REPLAY@" + json.dumps(packet, ensure_ascii=False, allow_nan=False, separators=(",", ":")), flush=True)

    def state(self, status, **extra):
        self.output({"kind": "state", "data": {"status": status, "position_seconds": self.position,
                      "progress_percent": min(100, self.position / max(self.duration, .001) * 100),
                      "decode_errors": dict(self.errors), **extra}})

    def command(self, command):
        with self.condition:
            if command == "stop":
                self.stop_requested = True
            elif command == "pause" and not self.paused:
                self.paused = True
                self.pause_started = time.monotonic()
                self.state("paused", message="回放已暂停。")
            elif command == "resume" and self.paused:
                self.wall_origin += time.monotonic() - self.pause_started
                self.paused = False
                self.state("playing", message="正在回放历史采集数据。")
            self.condition.notify_all()

    def commands(self):
        for line in sys.stdin:
            try:
                self.command(json.loads(line).get("command"))
            except (ValueError, AttributeError):
                continue
        self.command("stop")

    def wait(self, seconds):
        while True:
            with self.condition:
                if self.stop_requested:
                    return False
                if self.paused:
                    self.state("paused", message="回放已暂停。")
                    self.condition.wait(.5)
                    continue
                remaining = self.wall_origin + seconds / self.rate - time.monotonic()
                if remaining <= 0:
                    return True
                self.condition.wait(min(remaining, .04))

    def run(self):
        metadata = yaml.safe_load((self.bag / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
        start_ns = metadata["starting_time"]["nanoseconds_since_epoch"]
        self.duration = metadata["duration"]["nanoseconds"] / 1e9
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(self.bag), storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
        types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        topic_ids = {spec[1]: ident for ident, spec in SENSORS.items()}
        classes, last_shown = {}, {}
        self.wall_origin = time.monotonic()
        self.state("playing", message="正在回放历史采集数据。")
        last_status = 0
        while reader.has_next():
            if self.stop_requested:
                break
            topic, serialized, timestamp = reader.read_next()
            seconds = max(0, (timestamp - start_ns) / 1e9)
            if not self.wait(seconds):
                break
            self.position = seconds
            ident = topic_ids.get(topic, topic)
            fps = 5 if SENSORS.get(ident, (None, None, "generic"))[2] in ("image", "cloud") else 10 if ident in ("eddy_raw", "eddy_mean", "imu", "imu_raw") else 1
            wall = time.monotonic()
            if wall - last_shown.get(topic, -math.inf) >= 1 / fps:
                last_shown[topic] = wall
                try:
                    if topic not in classes:
                        classes[topic] = get_message(types[topic])
                    message = deserialize_message(serialized, classes[topic])
                    data = sensor_preview(ident, message)
                    if not self.wait(seconds):
                        break
                    header = getattr(message, "header", None)
                    stamp_ns = str(header.stamp.sec * 10**9 + header.stamp.nanosec) if header else str(timestamp)
                    self.output({"kind": "sensor", "data": {"session_id": self.session, "sensor_id": ident, "topic": topic,
                                 "timestamp_ns": str(timestamp), "stamp_ns": stamp_ns, "frame_id": getattr(header, "frame_id", ""),
                                 "position_seconds": seconds, "preview_only": True, "data": data}})
                except Exception as error:
                    self.errors[topic] += 1
                    if self.errors[topic] == 1:
                        self.output({"kind": "sensor", "data": {"session_id": self.session, "sensor_id": ident, "topic": topic,
                                     "position_seconds": seconds, "data": {"kind": "error", "message": str(error)}}})
            if wall - last_status >= .25:
                with self.condition:
                    self.state("paused" if self.paused else "playing", message="回放已暂停。" if self.paused else "正在回放历史采集数据。")
                last_status = wall
        if self.stop_requested:
            self.state("stopped", message="回放已停止。")
        else:
            self.position = self.duration
            self.state("complete", message="本批数据回放结束。")


def main():
    playback = Playback(sys.argv[1], sys.argv[2], float(sys.argv[3]))
    signal.signal(signal.SIGINT, lambda *_: playback.command("stop"))
    signal.signal(signal.SIGTERM, lambda *_: playback.command("stop"))
    threading.Thread(target=playback.commands, daemon=True).start()
    try:
        playback.run()
        return 0
    except Exception as error:
        playback.output({"kind": "state", "data": {"status": "failed", "message": "回放失败：" + str(error)}})
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
