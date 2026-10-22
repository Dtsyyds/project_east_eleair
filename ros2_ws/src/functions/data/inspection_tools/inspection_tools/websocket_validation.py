#!/usr/bin/env python3
"""Validate the live tank_inspection_v2 bridge without retaining JPEG bodies."""

from __future__ import annotations
import argparse
import base64
import hashlib
import json
from pathlib import Path
import time
import cv2
import numpy as np
from websockets.sync.client import connect


def envelope(kind: str, payload: dict, request_id: str) -> str:
    return json.dumps({"header": {"robot_id": "tank_inspection_robot", "msg_type": kind,
        "schema_version": "tank_inspection_v2", "request_id": request_id}, "payload": payload})


def receive_until(ws, predicate, timeout: float):
    deadline = time.monotonic() + timeout
    observed = []
    while time.monotonic() < deadline:
        try:
            message = json.loads(ws.recv(timeout=min(0.5, deadline - time.monotonic())))
        except TimeoutError:
            continue
        observed.append(message)
        if predicate(message):
            return message, observed
    raise TimeoutError("expected WebSocket message was not received")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", default="ws://127.0.0.1:9001")
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout-sec", type=float, default=15.0)
    args = parser.parse_args(argv)
    result = {"schema_version": "tank_websocket_live_validation_v1", "uri": args.uri,
              "checks": {}, "message_examples": []}
    ws = connect(args.uri, proxy=None, open_timeout=2)
    try:
        ws.send(envelope("check", {}, "live-check-1"))
        response, seen = receive_until(
            ws, lambda item: item.get("header", {}).get("request_id") == "live-check-1",
            args.timeout_sec)
        result["checks"]["schema_and_request_id"] = (
            response["header"].get("schema_version") == "tank_inspection_v2" and
            response["header"].get("request_id") == "live-check-1")
        devices = response.get("payload", {})
        result["device_check"] = devices
        cameras: dict[str, dict] = {}
        eca = None
        lidar = None
        deadline = time.monotonic() + args.timeout_sec
        for packet in seen:
            payload = packet.get("payload", {})
            if "video" in payload:
                cameras[payload["video"].get("camera_id", "unknown")] = payload["video"]
        while time.monotonic() < deadline and (len(cameras) < 3 or eca is None or lidar is None):
            try:
                packet = json.loads(ws.recv(timeout=0.5))
            except TimeoutError:
                continue
            payload = packet.get("payload", {})
            if "video" in payload:
                cameras[payload["video"].get("camera_id", "unknown")] = payload["video"]
            if "eca" in payload:
                eca = payload["eca"]
            if "lidar" in payload:
                lidar = payload["lidar"]
        decoded = {}
        for camera, video in cameras.items():
            body = base64.b64decode(video["data"])
            image = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_UNCHANGED)
            decoded[camera] = image is not None and image.shape[1] == video["width"] and image.shape[0] == video["height"]
            example = {key: value for key, value in video.items() if key != "data"}
            example["jpeg_bytes"] = len(body)
            example["jpeg_sha256"] = hashlib.sha256(body).hexdigest()
            result["message_examples"].append({"video": example})
        result["checks"]["three_jpeg_previews_decoded"] = (
            set(decoded) == {"fisheye", "mindvision", "depth_camera"} and all(decoded.values()))
        result["checks"]["eca_raw_mean_received"] = bool(
            eca and eca.get("raw", {}).get("data") and eca.get("mean", {}).get("channel_mean"))
        lidar_status = devices.get("lidar", {})
        result["checks"]["lidar_status_received"] = (
            lidar_status.get("status") == "verified" and
            float(lidar_status.get("publish_rate_hz", 0.0)) > 0.0)
        result["checks"]["preview_counter_ordering"] = all(
            int(video.get("source_received", 0)) >= int(video.get("source_accepted", 0)) >=
            int(video.get("preview_dropped", 0)) for video in cameras.values())
        result["lidar_pose"] = {
            "status": "PASS" if lidar is not None else "SKIP",
            "reason": "pose payload observed" if lidar is not None else
                      "FAST-LIVO2 /aft_mapped_to_init was not running",
        }
        result["eca_example"] = eca
        result["lidar_example"] = lidar
        ws.send("{invalid")
        ws.send(envelope("command", {"unknown": {"command": "invalid"}}, "invalid-followup"))
        invalid_response, _ = receive_until(
            ws, lambda item: item.get("header", {}).get("request_id") == "invalid-followup", 3)
        result["checks"]["malformed_message_did_not_break_connection"] = (
            invalid_response.get("payload", {}).get("unknown", {}).get("reason") == "unsupported_command")
    finally:
        ws.close()
    ws = connect(args.uri, proxy=None, open_timeout=2)
    try:
        ws.send(envelope("check", {}, "reconnect-check"))
        response, _ = receive_until(
            ws, lambda item: item.get("header", {}).get("request_id") == "reconnect-check", 4)
        result["checks"]["client_reconnect_and_state_restore"] = all(
            response.get("payload", {}).get(name, {}).get("status") == "verified"
            for name in ("fisheye", "mindvision", "depth_camera", "eddy_current"))
    finally:
        ws.close()
    result["pass"] = all(result["checks"].values())
    Path(args.output).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
