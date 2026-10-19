#!/usr/bin/env python3
"""Protocol-level check for the isolated mock workflow used by browser QA."""
from __future__ import annotations

import argparse
import asyncio
import json

import aiohttp


ROBOT = "tank_inspection_robot"


def packet(msg_type: str, payload: dict) -> str:
    return json.dumps({"header": {"robot_id": ROBOT, "msg_type": msg_type}, "payload": payload})


async def receive_until(ws: aiohttp.ClientWebSocketResponse, predicate, timeout: float = 4.0) -> dict:
    async def wait() -> dict:
        async for message in ws:
            if message.type != aiohttp.WSMsgType.TEXT:
                continue
            value = json.loads(message.data)
            if predicate(value):
                return value
        raise AssertionError("websocket closed before expected packet")

    return await asyncio.wait_for(wait(), timeout)


async def command(ws: aiohttp.ClientWebSocketResponse, action: str, request_id: str, parameters: dict | None = None) -> None:
    await ws.send_str(packet("command", {"action": action, "request_id": request_id, "client_id": "workflow-test", "parameters": parameters or {}}))


async def run(base: str) -> None:
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(base.replace("http", "ws", 1) + "/ws", max_msg_size=1024 * 1024) as ws:
            initial = await receive_until(ws, lambda p: p["header"]["msg_type"] == "status_update" and "capabilities" in p["payload"])
            capabilities = initial["payload"]["capabilities"]
            assert capabilities["mode"] == "mock"
            assert capabilities["snapshot"]["displayed_frame_cache"] is True
            assert initial["payload"]["control"]["enabled"] is False

            await ws.send_str(packet("check", {}))
            legacy = await receive_until(ws, lambda p: p["header"]["msg_type"] == "response" and "fisheye" in p["payload"])
            assert "request_id" not in legacy["payload"], "legacy response must remain uncorrelated"

            await command(ws, "collection.preflight", "preflight-1", {"profile": "mock_all"})
            ready = await receive_until(ws, lambda p: p["payload"].get("request_id") == "preflight-1" and p["payload"].get("status") == "completed")
            assert ready["payload"]["task"]["status"] == "ready"

            await command(ws, "collection.start", "start-1", {"profile": "mock_all"})
            started = await receive_until(ws, lambda p: p["payload"].get("request_id") == "start-1" and p["payload"].get("status") == "completed")
            run_id = started["payload"]["task"]["run_id"]
            assert started["payload"]["task"]["status"] == "recording"

            await command(ws, "collection.start", "start-2", {"profile": "mock_all"})
            reused = await receive_until(ws, lambda p: p["payload"].get("request_id") == "start-2" and p["payload"].get("status") == "completed")
            assert reused["payload"]["result"]["task"]["run_id"] == run_id
            assert reused["payload"]["idempotent_reuse"] is True

            await command(ws, "collection.stop", "stop-1", {"run_id": run_id})
            final = await receive_until(ws, lambda p: p["payload"].get("request_id") == "stop-1" and p["payload"].get("status") == "completed")
            assert final["payload"]["task"]["status"] == "complete"
            assert final["payload"]["task"]["source_sha_valid"] is True
            assert final["payload"]["task"]["algorithm_accepted"] is None

            await command(ws, "evidence.save_next", "event-1", {"type": "seam_lost", "note": "mock"})
            event = await receive_until(ws, lambda p: p["payload"].get("request_id") == "event-1" and p["payload"].get("status") == "completed")
            assert event["payload"]["event"]["status"] == "complete"
            assert "raw_depth" in event["payload"]["event"]["missing"]

            await command(ws, "history.list", "history-1")
            history = await receive_until(ws, lambda p: p["payload"].get("request_id") == "history-1" and p["payload"].get("status") == "completed")
            assert history["payload"]["history"][0]["hardware_field_verified"] is False

        async with session.get(base + "/mock/state") as response:
            server_state = await response.json()
        assert server_state["chassis_requests"] == 0, "automated workflow must never exercise actuator operations"
        print(json.dumps({"result": "passed", "run_id": run_id, "server": server_state}, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", default="http://127.0.0.1:18765")
    args = parser.parse_args()
    asyncio.run(run(args.base))
