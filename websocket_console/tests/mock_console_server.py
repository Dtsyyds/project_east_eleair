#!/usr/bin/env python3
"""Isolated UI mock. It never imports ROS or publishes a chassis topic."""
from __future__ import annotations

import argparse
import asyncio
import base64
import io
import json
from pathlib import Path
from typing import Any

from aiohttp import WSMsgType, web

ROOT = Path(__file__).resolve().parents[1]
ROBOT = "tank_inspection_robot"


def jpeg_bytes(color: tuple[int, int, int]) -> bytes:
    try:
        from PIL import Image
        image = Image.new("RGB", (64, 48), color)
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=75)
        return output.getvalue()
    except ImportError as exc:  # pragma: no cover - CI image includes Pillow
        raise RuntimeError("Pillow is required by the isolated mock") from exc


JPEG_FISHEYE = base64.b64encode(jpeg_bytes((28, 78, 112))).decode("ascii")
JPEG_D405 = base64.b64encode(jpeg_bytes((72, 45, 95))).decode("ascii")


class MockConsole:
    def __init__(self) -> None:
        self.clients: set[web.WebSocketResponse] = set()
        self.scenario = "normal"
        self.frame = 0
        self.run_counter = 0
        self.start_requests = 0
        self.stop_requests = 0
        self.evidence_requests = 0
        self.chassis_requests = 0
        self.task: dict[str, Any] = {"status": "idle", "mode": "mock"}
        self.video_task: asyncio.Task[None] | None = None

    @property
    def capabilities(self) -> dict[str, Any]:
        operations = {
            "collection.preflight": True,
            "collection.start": {"supported": True, "msg_type": "command", "timeout_ms": 3000},
            "collection.stop": {"supported": True, "msg_type": "command", "timeout_ms": 3000},
            "collection.status": True,
            "evidence.save_displayed": True,
            "evidence.save_next": True,
            "event.mark": True,
            "history.list": True,
            "chassis.move": True,
            "chassis.stop": True,
        }
        return {
            "protocol_version": 2,
            "server_version": "isolated-mock-1",
            "robot_id": ROBOT,
            "mode": "mock",
            "operations": operations,
            "permissions": {key: True for key in operations},
            "collection": {"profiles": [{"id": "mock_all", "label": "隔离 Mock 全量"}]},
            "snapshot": {"displayed_frame_cache": True, "queue_limit": 4},
            "control": {"axes": ["vx", "wz"], "limits": {"vx": 0.1, "wz": 0.2}, "units": {"vx": "m/s", "wz": "rad/s"}},
            "freshness_ms": {"fisheye": 1400, "depth_camera": 1400, "lidar_pose": 1200, "tracking": 1200, "control": 1200, "storage": 5000},
        }

    async def packet(self, ws: web.WebSocketResponse, msg_type: str, payload: dict[str, Any]) -> None:
        if not ws.closed:
            await ws.send_str(json.dumps({"header": {"robot_id": ROBOT, "msg_type": msg_type}, "payload": payload}, separators=(",", ":")))

    async def broadcast(self, msg_type: str, payload: dict[str, Any]) -> None:
        await asyncio.gather(*(self.packet(ws, msg_type, payload) for ws in tuple(self.clients)), return_exceptions=True)

    def status_payload(self) -> dict[str, Any]:
        storage = {"mounted": True, "writable": True, "mount_point_label": "MOCK_SSD", "free_bytes": 120 * 1024**3, "status": "active"}
        if self.scenario == "disk":
            storage = {"mounted": False, "writable": False, "status": "failed", "error": "mock mount missing"}
        task = dict(self.task)
        if self.scenario == "finalizing":
            task = {"status": "finalizing", "run_id": "mock-run-finalizing", "progress_percent": 72, "message": "mock writer closed; validating manifest", "writer_closed": True, "mcap_valid": None}
        return {
            "capabilities": self.capabilities,
            "devices": {
                "fisheye": {"status": "verified", "configured": True, "topic": "/mock/fisheye", "message_type": "sensor_msgs/Image", "received": self.frame, "accepted": self.frame, "invalid": 0, "expected_hz": 8, "reason": "mock data"},
                "depth_camera": {"status": "verified", "configured": True, "topic": "/mock/d405/color", "message_type": "sensor_msgs/Image", "received": self.frame, "accepted": self.frame, "invalid": 0, "expected_hz": 8, "reason": "mock data"},
                "mindvision": {"configured": False, "status": "unverified", "reason": "not enabled in mock profile"},
                "eddy_current": {"status": "active", "configured": True, "topic": "/mock/eddy_mean", "expected_hz": 10},
                "emat": {"configured": False, "status": "unverified", "reason": "not configured"},
                "chassis": {"status": "active", "configured": True, "reason": "mock read-only actuator"},
                "lidar": {"status": "active", "configured": True, "topic": "/mock/odom", "expected_hz": 20},
            },
            "streams": {
                "fisheye": {"status": "active", "actual_hz": 8.0, "calibration_status": "mock_reference", "sync_status": "not_required"},
                "depth_camera": {"status": "active", "actual_hz": 8.0, "calibration_status": "mock_reference", "sync_status": "mock_synchronized"},
                "lidar_pose": {"status": "active", "actual_hz": 20.0, "calibration_status": "not_registered_to_tank"},
                "eddy_mean": {"status": "active", "actual_hz": 10.0, "unit": "raw_count"},
            },
            "lidar": {"status": "active", "valid": True, "x": self.frame * 0.002, "y": 0.05, "z": 0.0, "angle": 0.02, "position_unit": "m", "angle_unit": "rad", "frame_id": "mock_odom", "child_frame_id": "mock_base", "axis_convention": "x-forward y-left", "asset_registration": "not_registered", "stamp_ns": str(1_800_000_000_000_000_000 + self.frame * 125_000_000)},
            "control": {"enabled": False, "status": "active", "reason": "mock blocks actuator writes", "watchdog_stops": 0},
            "storage": storage,
            "task": task,
            "bridge": {"command_dropped": 0, "transport_dropped": 0, "bounded_video_queue": 2},
        }

    def tracking(self, source: str, frame_id: str, stamp_ns: str) -> dict[str, Any]:
        status = "VISION_2D"
        reason = "mock 2D observation valid; 3D identity not confirmed"
        if self.scenario == "lost":
            status, reason = "LOST", "mock candidate count below threshold"
        elif self.scenario == "conflict":
            status, reason = "CONFLICT_2D", "mock left/right observations conflict"
        return {
            "status": status,
            "reason": reason,
            "source": "isolated_mock_tracker",
            "backend_version": "mock-1",
            "mode": "vision_only",
            "vision_2d_valid": status == "VISION_2D",
            "geometry_3d_valid": False,
            "identity_confirmed": False,
            "predicted": False,
            "coordinate_space": "image_pixels",
            "image_width": 64,
            "image_height": 48,
            "frame_ref": {"robot_id": ROBOT, "source_id": source, "stream_id": "preview", "frame_id": frame_id, "stamp_ns": stamp_ns},
            "left_boundary": [[20, 5], [22, 42]],
            "right_boundary": [[42, 5], [40, 42]],
            "centerline": [[31, 5], [31, 42]],
            "reliable_coverage": [[20, 10], [42, 10], [40, 38], [22, 38]],
            "metrics": {"candidate_count": 24 if status == "VISION_2D" else 2, "coverage": 0.82, "confidence": 0.74, "processing_ms": 6.2},
            "diagnostics": {"candidate_threshold": 8, "comparison": "candidate_count >= threshold", "stage": "mock_filter"},
        }

    async def video_loop(self) -> None:
        while True:
            await asyncio.sleep(0.125)
            self.frame += 1
            if self.scenario == "partial":
                await self.broadcast("status_update", {"devices": {"fisheye": {"status": "verified", "configured": True, "topic": "/mock/fisheye", "expected_hz": 8, "received": self.frame}}})
                continue
            if self.frame % 8 == 0:
                await self.broadcast("status_update", self.status_payload())
            for camera_id, image, content in (("fisheye", JPEG_FISHEYE, "rgb8"), ("depth_camera", JPEG_D405, "depth_colormap")):
                frame_id = f"{camera_id}-{self.frame}"
                stamp_ns = str(1_800_000_000_000_000_000 + self.frame * 125_000_000)
                data = "AAAA" if self.scenario == "corrupt" and camera_id == "depth_camera" else image
                width = 65 if self.scenario == "dimension" and camera_id == "depth_camera" else 64
                video = {"camera_id": camera_id, "stream_id": "preview", "encoding": "jpeg_base64", "source_encoding": content, "content_type": content, "width": width, "height": 48, "frame_id": frame_id, "stamp_ns": stamp_ns, "preview_dropped": 0, "data": data}
                payload: dict[str, Any] = {"video": video}
                if camera_id == "depth_camera":
                    payload["tracking_result"] = self.tracking(camera_id, frame_id, stamp_ns)
                    payload["eddy_mean"] = {"channel_mean": [100 + (self.frame % 5), 105 - (self.frame % 3)], "unit": "raw_count", "stamp_ns": stamp_ns}
                await self.broadcast("status_sensor", payload)

    async def handle_action(self, ws: web.WebSocketResponse, payload: dict[str, Any]) -> None:
        action, request_id = payload.get("action"), payload.get("request_id")
        if not isinstance(request_id, str):
            await self.packet(ws, "response", {"status": "failed", "error": "missing request_id"})
            return
        await self.packet(ws, "response", {"request_id": request_id, "status": "accepted", "action": action})
        if action == "collection.status":
            await self.packet(ws, "response", {"request_id": request_id, "status": "completed", "result": {"task": self.task}})
        elif action == "collection.preflight":
            self.task = {"status": "ready", "mode": "mock", "preflight_checks": [{"name": "mock mount", "status": "passed", "message": "isolated writable data root"}, {"name": "mock topics", "status": "passed", "message": "required mock streams active"}]}
            await self.packet(ws, "task_update", {"request_id": request_id, "status": "completed", "task": self.task})
        elif action == "collection.start":
            self.start_requests += 1
            if self.task.get("status") == "recording":
                await self.packet(ws, "response", {"request_id": request_id, "status": "completed", "result": {"task": self.task}, "idempotent_reuse": True})
            else:
                self.run_counter += 1
                self.task = {"status": "recording", "mode": "mock", "run_id": f"mock-run-{self.run_counter:03d}", "elapsed_seconds": 0.0, "bytes_written": 4096, "mcap_status": "writing", "video_status": "writing"}
                await self.packet(ws, "task_update", {"request_id": request_id, "status": "completed", "task": self.task})
        elif action == "collection.stop":
            self.stop_requests += 1
            run_id = self.task.get("run_id", "mock-run-unknown")
            self.task = {"status": "finalizing", "mode": "mock", "run_id": run_id, "progress_percent": 65, "writer_closed": True, "message": "mock validation running"}
            await self.packet(ws, "task_update", {"request_id": request_id, "status": "running", "job_id": "mock-finalize", "task": self.task})
            await asyncio.sleep(0.2)
            self.task = {"status": "complete", "mode": "mock", "run_id": run_id, "progress_percent": 100, "writer_closed": True, "mcap_valid": True, "source_sha_valid": True, "algorithm_accepted": None, "finalize_checks": [{"name": "writer", "status": "passed", "message": "closed"}, {"name": "manifest sha", "status": "passed", "message": "mock hash verified"}, {"name": "hardware field", "status": "unknown", "message": "not exercised by mock"}]}
            await self.packet(ws, "task_update", {"request_id": request_id, "status": "completed", "job_id": "mock-finalize", "task": self.task})
        elif action in {"evidence.save_displayed", "evidence.save_next", "event.mark"}:
            self.evidence_requests += 1
            event_id = f"mock-event-{self.evidence_requests:03d}"
            queued = {"event_id": event_id, "type": payload.get("parameters", {}).get("type"), "status": "queued", "stamp_ns": str(1_800_000_000_000_000_000 + self.frame * 125_000_000), "note": payload.get("parameters", {}).get("note")}
            await self.packet(ws, "event_update", {"request_id": request_id, "status": "running", "event": queued})
            await asyncio.sleep(0.1)
            failed = self.scenario == "event_failure"
            event = dict(queued, status="failed" if failed else "complete", error="mock disk write failure" if failed else None, evidence=[] if failed else [{"name": "image", "status": "complete"}, {"name": "metadata", "status": "complete", "sha256": "mock-sha256"}], missing=["raw_depth"])
            await self.packet(ws, "event_update", {"request_id": request_id, "status": "failed" if failed else "completed", "event": event})
        elif action == "history.list":
            history = [{"run_id": "mock-run-archived", "started_at": "2026-10-04T10:00:00Z", "sensors": ["mock_fisheye", "mock_d405"], "duration_seconds": 12.5, "size_bytes": 20480, "finalize_status": "complete", "validation_status": "mock_only", "hardware_field_verified": False, "path_hint": "use board-side handoff command from the real adapter"}]
            await self.packet(ws, "response", {"request_id": request_id, "status": "completed", "history": history})
        elif action in {"chassis.move", "chassis.stop"}:
            self.chassis_requests += 1
            await self.packet(ws, "response", {"request_id": request_id, "status": "failed", "error": "isolated mock never publishes actuator commands"})
        else:
            await self.packet(ws, "response", {"request_id": request_id, "status": "failed", "error": "unsupported mock action"})

    async def websocket(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(max_msg_size=1024 * 1024)
        await ws.prepare(request)
        self.clients.add(ws)
        await self.packet(ws, "status_update", self.status_payload())
        try:
            async for message in ws:
                if message.type != WSMsgType.TEXT:
                    continue
                try:
                    packet = json.loads(message.data)
                    payload = packet.get("payload", {})
                    msg_type = packet.get("header", {}).get("msg_type")
                except (ValueError, AttributeError):
                    continue
                if msg_type == "check":
                    await self.packet(ws, "response", {"fisheye": {"command": "check", "status": "verified", "configured": True, "reason": "legacy shared response"}})
                elif msg_type == "command" and isinstance(payload.get("action"), str):
                    await self.handle_action(ws, payload)
                elif msg_type == "command":
                    await self.packet(ws, "response", {"camera": {"command": payload.get("camera", {}).get("command"), "status": "published_mock_only"}})
        finally:
            self.clients.discard(ws)
        return ws

    async def set_scenario(self, request: web.Request) -> web.Response:
        name = request.match_info["name"]
        allowed = {"normal", "lost", "conflict", "disk", "finalizing", "event_failure", "corrupt", "dimension", "partial"}
        if name not in allowed:
            raise web.HTTPBadRequest(text="unknown scenario")
        self.scenario = name
        await self.broadcast("status_update", self.status_payload())
        return web.json_response({"scenario": name})

    async def inspect(self, _request: web.Request) -> web.Response:
        return web.json_response({"scenario": self.scenario, "clients": len(self.clients), "task": self.task, "start_requests": self.start_requests, "stop_requests": self.stop_requests, "evidence_requests": self.evidence_requests, "chassis_requests": self.chassis_requests, "frame": self.frame})


async def static_file(name: str) -> web.Response:
    path = ROOT / name
    content_type = "text/html" if name.endswith(".html") else "text/javascript"
    return web.Response(body=path.read_bytes(), content_type=content_type, charset="utf-8")


def make_app(mock: MockConsole) -> web.Application:
    app = web.Application(client_max_size=1024 * 1024)
    app.router.add_get("/", lambda _request: static_file("index.html"))
    app.router.add_get("/index.html", lambda _request: static_file("index.html"))
    app.router.add_get("/console.js", lambda _request: static_file("console.js"))
    app.router.add_get("/console_core.js", lambda _request: static_file("console_core.js"))
    app.router.add_get("/ws", mock.websocket)
    app.router.add_post("/mock/scenario/{name}", mock.set_scenario)
    app.router.add_get("/mock/state", mock.inspect)

    async def start(_app: web.Application) -> None:
        mock.video_task = asyncio.create_task(mock.video_loop())

    async def stop(_app: web.Application) -> None:
        if mock.video_task:
            mock.video_task.cancel()
            try:
                await mock.video_task
            except asyncio.CancelledError:
                pass

    app.on_startup.append(start)
    app.on_cleanup.append(stop)
    return app


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    web.run_app(make_app(MockConsole()), host=args.host, port=args.port, print=None)
