"""Bounded gateway forwarding; simulated transports never publish robot commands."""
import asyncio
import json
from pathlib import Path
import sys

import pytest
from websockets.exceptions import ConnectionClosedError
from websockets.frames import Close
from websockets.legacy.client import connect
from websockets.legacy.server import serve, WebSocketServerProtocol

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools'))
import robot_gateway


class Endpoint:
    def __init__(self, delay=0):
        self.queue = asyncio.Queue()
        self.delay = delay
        self.sent = []
        self.attempts = 0
        self.closed = False
        self.close_code = None
        self.close_reason = None

    def __aiter__(self):
        return self

    async def __anext__(self):
        packet = await self.queue.get()
        if packet is None:
            raise StopAsyncIteration
        return packet

    async def send(self, packet):
        self.attempts += 1
        await asyncio.sleep(self.delay)
        self.sent.append(packet)

    async def close(self, code=1000, reason=''):
        self.closed = True
        self.close_code, self.close_reason = code, reason
        await self.queue.put(None)


def use_upstream(monkeypatch, upstream):
    async def connecting(*args, **kwargs):
        assert kwargs['max_queue'] == 1
        assert kwargs['ping_timeout'] == 3
        return upstream
    monkeypatch.setattr(robot_gateway, 'connect', connecting)


async def until(predicate, timeout=2):
    async def waiting():
        while not predicate():
            await asyncio.sleep(.01)
    await asyncio.wait_for(waiting(), timeout)


@pytest.mark.parametrize('route', ['/ws/inspection', '/', '/ws'])
def test_camera_send_over_old_deadline_does_not_disconnect(monkeypatch, capsys, route):
    async def scenario():
        gateway = robot_gateway.RobotGateway()
        browser, upstream = Endpoint(delay=.35), Endpoint()
        use_upstream(monkeypatch, upstream)
        await upstream.queue.put('video-private-payload')
        worker = asyncio.create_task(gateway.websocket(browser, route))
        try:
            await until(lambda: bool(browser.sent))
            assert browser.sent == ['video-private-payload']
            assert not worker.done() and not browser.closed
            # Downstream congestion doesn't serialize or delay the other direction.
            await browser.queue.put('private-control-token')
            await until(lambda: bool(upstream.sent))
            assert upstream.sent == ['private-control-token']
        finally:
            await browser.close()
            await asyncio.wait_for(worker, 2)
        assert upstream.closed and not gateway.clients
    asyncio.run(scenario())
    output = capsys.readouterr().out
    assert 'video-private-payload' not in output and 'private-control-token' not in output
    assert '"slow_sends": 1' in output
    assert '"reason": "browser_closed"' in output


@pytest.mark.parametrize('route', ['/ws/inspection', '/ws/robot'])
def test_commands_keep_short_deadline_and_are_never_replayed(monkeypatch, capsys, route):
    async def scenario():
        gateway = robot_gateway.RobotGateway()
        browser, upstream = Endpoint(), Endpoint(delay=10)
        use_upstream(monkeypatch, upstream)
        await browser.queue.put('command-one-secret')
        await browser.queue.put('command-two-secret')
        await asyncio.wait_for(gateway.websocket(browser, route), 1)
        assert upstream.attempts == 1 and upstream.sent == []
        assert browser.closed and upstream.closed
        assert browser.close_reason == 'upstream_send_timeout'
        assert browser.queue.qsize() == 2  # unsent second command and close sentinel
    asyncio.run(scenario())
    output = capsys.readouterr().out
    assert '"timeout_ms": 250' in output
    assert 'command-one-secret' not in output and 'command-two-secret' not in output


def test_chassis_downstream_deadline_is_unchanged(monkeypatch):
    async def scenario():
        gateway = robot_gateway.RobotGateway()
        browser, upstream = Endpoint(delay=10), Endpoint()
        use_upstream(monkeypatch, upstream)
        await upstream.queue.put('chassis-feedback')
        await asyncio.wait_for(gateway.websocket(browser, '/ws/robot'), 1)
        assert browser.close_reason == 'browser_send_timeout'
        assert not browser.sent and upstream.closed
    asyncio.run(scenario())


def test_camera_unresponsive_client_still_has_bounded_shutdown(monkeypatch, capsys):
    async def scenario():
        gateway = robot_gateway.RobotGateway()
        browser, upstream = Endpoint(delay=10), Endpoint()
        use_upstream(monkeypatch, upstream)
        await upstream.queue.put(b'large-frame')
        started = asyncio.get_running_loop().time()
        await asyncio.wait_for(gateway.websocket(browser, '/ws/inspection'), 3)
        assert 1.9 <= asyncio.get_running_loop().time() - started < 3
        assert browser.close_reason == 'browser_send_timeout'
        assert browser.closed and upstream.closed and not gateway.clients
    asyncio.run(scenario())
    output = capsys.readouterr().out
    assert '"timeout_ms": 2000' in output and '"packet_bytes": 11' in output


@pytest.mark.parametrize('code,reason,classification', [
    (1011, 'keepalive ping timeout', 'upstream_heartbeat_timeout'),
    (1009, 'peer-private-message', 'upstream_message_too_large'),
    (1011, 'peer-private-message', 'upstream_closed'),
])
def test_close_codes_are_classified_without_logging_peer_text(code, reason, classification):
    error = ConnectionClosedError(None, Close(code, reason), None)
    details = robot_gateway.connection_end('upstream', error)
    assert details['reason'] == classification and details['sent_code'] == code
    assert reason not in json.dumps(details)


def test_upstream_refusal_is_diagnosed_and_client_is_released(monkeypatch, capsys):
    async def connecting(*args, **kwargs):
        raise ConnectionRefusedError(111, 'secret-in-exception')
    monkeypatch.setattr(robot_gateway, 'connect', connecting)
    async def scenario():
        gateway, browser = robot_gateway.RobotGateway(), Endpoint()
        await gateway.websocket(browser, '/ws/inspection')
        assert browser.close_reason == 'transport_error'
        assert not gateway.clients
    asyncio.run(scenario())
    output = capsys.readouterr().out
    assert 'ConnectionRefusedError' in output and '"errno": 111' in output
    assert 'secret-in-exception' not in output


def test_real_proxy_roundtrip_and_browser_close_cleans_upstream():
    async def scenario():
        ended = asyncio.Event()
        async def backend(ws, path):
            try:
                assert path == '/'
                packet = await ws.recv()
                await ws.send(packet)
                await ws.send(b'binary-preview')
                await ws.wait_closed()
            finally:
                ended.set()
        async with serve(backend, '127.0.0.1', 0) as backend_server:
            gateway = robot_gateway.RobotGateway(camera_port=backend_server.sockets[0].getsockname()[1])
            async with serve(gateway.websocket, '127.0.0.1', 0, max_queue=1) as public_server:
                port = public_server.sockets[0].getsockname()[1]
                async with connect(f'ws://127.0.0.1:{port}/ws/inspection') as browser:
                    await browser.send('read-only-probe')
                    assert await browser.recv() == 'read-only-probe'
                    assert await browser.recv() == b'binary-preview'
                await asyncio.wait_for(ended.wait(), 2)
                await until(lambda: not gateway.clients)
    asyncio.run(scenario())


def video(sequence, camera='mindvision'):
    return json.dumps({'header': {'robot_id': 'tank_inspection_robot', 'msg_type': 'status_sensor',
                                 'schema_version': 'tank_inspection_v2'},
                       'payload': {'video': {'camera_id': camera, 'encoding': 'jpeg_base64',
                                             'visualization_only': True, 'sequence': sequence, 'data': 'x' * 4096}}})


def test_previews_replace_only_same_stream_and_replies_keep_fifo():
    async def scenario():
        stats = {}
        buffer = robot_gateway.InspectionBuffer(stats)
        for i in range(100):
            buffer.put(video(i))
            buffer.put(video(i, 'fisheye'))
        reply1 = json.dumps({'header': {'msg_type': 'response'}, 'payload': {'request_id': 'one'}})
        reply2 = json.dumps({'header': {'msg_type': 'event'}, 'payload': {'event_id': 'two'}})
        buffer.put(reply1)
        buffer.put(reply2)
        buffer.finish({'reason': 'upstream_closed', 'received_code': 1000})
        assert [item async for item in buffer] == [reply1, reply2, video(99), video(99, 'fisheye')]
        assert stats['previews_replaced'] == 198
        assert stats['peak_reliable_messages'] == 2
        assert buffer.bytes == stats['queued_bytes'] == 0
    asyncio.run(scenario())


@pytest.mark.parametrize('change', ['response', 'request_id', 'mixed_payload', 'unknown_schema', 'unknown_camera'])
def test_non_preview_or_mixed_packets_cannot_be_coalesced(change):
    packet = json.loads(video(1))
    if change == 'response':
        packet['header']['msg_type'] = 'response'
    elif change == 'request_id':
        packet['header']['request_id'] = 'do-not-drop'
    elif change == 'mixed_payload':
        packet['payload']['events'] = [{'event_id': 'do-not-drop'}]
    elif change == 'unknown_schema':
        packet['header']['schema_version'] = 'unknown'
    else:
        packet['payload']['video']['camera_id'] = 'unknown'
    assert robot_gateway.preview_key(json.dumps(packet)) is None


def test_buffer_limits_bytes_and_reliable_count(monkeypatch):
    async def scenario():
        buffer = robot_gateway.InspectionBuffer({})
        for i in range(robot_gateway.INSPECTION_RELIABLE_MESSAGES):
            buffer.put(f'response {i}')
        with pytest.raises(BufferError):
            buffer.put('one too many')
        buffer = robot_gateway.InspectionBuffer({})
        monkeypatch.setattr(robot_gateway, 'INSPECTION_BUFFER_BYTES', len(video(1).encode()) + 1)
        buffer.put(video(1))
        buffer.put(video(2))  # Replacement doesn't increase the occupied space.
        with pytest.raises(BufferError):
            buffer.put(video(1, 'fisheye'))
    asyncio.run(scenario())


def test_expired_previews_are_not_presented_as_fresh(monkeypatch):
    async def scenario():
        stats = {}
        buffer = robot_gateway.InspectionBuffer(stats)
        monkeypatch.setattr(robot_gateway, 'PREVIEW_MAX_AGE', .01)
        buffer.put(video(1))
        buffer.put('important response')
        await asyncio.sleep(.02)
        buffer.finish({'reason': 'upstream_closed'})
        assert [item async for item in buffer] == ['important response']
        assert stats['previews_expired'] == 1
    asyncio.run(scenario())


@pytest.mark.parametrize('legacy', [True, False])
def test_real_ping_reproduces_serial_reader_failure_and_verifies_fix(monkeypatch, legacy):
    """The backend actually answers Ping; only the serial reader loses the Pong.

    Scale heartbeat periods for this regression. The next test uses production
    5s/3s heartbeats across several cycles, with the same sustained overload.
    """
    async def scenario():
        pongs = []
        class Protocol(WebSocketServerProtocol):
            async def pong(self, data=b''):
                pongs.append(data)
                return await super().pong(data)
        async def backend(ws, path):
            sequence = 0
            try:
                while True:
                    await ws.send(video(sequence))
                    sequence += 1
                    await asyncio.sleep(.005)
            except ConnectionClosedError:
                pass
            except robot_gateway.ConnectionClosed:
                pass
        real_connect = robot_gateway.connect
        def fast_heartbeat(*args, **kwargs):
            kwargs.update(ping_interval=.2, ping_timeout=.25)
            return real_connect(*args, **kwargs)
        monkeypatch.setattr(robot_gateway, 'connect', fast_heartbeat)
        async with serve(backend, '127.0.0.1', 0, create_protocol=Protocol, ping_interval=None) as server:
            gateway = robot_gateway.RobotGateway(camera_port=server.sockets[0].getsockname()[1])
            if legacy:
                async def old_downstream(upstream, browser, stats, *_):
                    return await gateway.forward(upstream, browser, 'upstream', 'browser', 2, stats)
                monkeypatch.setattr(gateway, 'inspection_downstream', old_downstream)
            browser = Endpoint(delay=.08)
            worker = asyncio.create_task(gateway.websocket(browser, '/ws/inspection'))
            try:
                if legacy:
                    await asyncio.wait_for(worker, 5)
                    assert browser.close_reason == 'upstream_heartbeat_timeout'
                    assert pongs, 'the backend must have answered Ping before the false timeout'
                else:
                    await asyncio.sleep(1.5)
                    assert len(pongs) >= 3 and not worker.done()
                    assert json.loads(browser.sent[-1])['payload']['video']['sequence'] > 100
            finally:
                await browser.close()
                await asyncio.wait_for(worker, 3)
    asyncio.run(scenario())


def test_real_production_heartbeat_under_20_seconds_of_preview_overload():
    async def scenario():
        backend_pongs = []
        class Protocol(WebSocketServerProtocol):
            async def pong(self, data=b''):
                backend_pongs.append(data)
                return await super().pong(data)
        async def backend(ws, path):
            async def previews():
                sequence = 0
                while True:
                    await ws.send(video(sequence))
                    sequence += 1
                    await asyncio.sleep(.01)
            producer = asyncio.create_task(previews())
            try:
                async for message in ws:
                    await ws.send(json.dumps({'header': {'msg_type': 'response'}, 'payload': {'request_id': message}}))
            finally:
                producer.cancel()
                await asyncio.gather(producer, return_exceptions=True)
        async with serve(backend, '127.0.0.1', 0, create_protocol=Protocol, ping_interval=None) as server:
            gateway = robot_gateway.RobotGateway(camera_port=server.sockets[0].getsockname()[1])
            browser = Endpoint(delay=.08)
            worker = asyncio.create_task(gateway.websocket(browser, '/ws/inspection'))
            try:
                await asyncio.sleep(1)
                for i in range(5):
                    await browser.queue.put(f'read-only-{i}')
                def replies():
                    packets = [json.loads(packet) for packet in browser.sent]
                    return [packet['payload']['request_id'] for packet in packets if packet['header']['msg_type'] == 'response']
                await until(lambda: len(replies()) == 5)
                assert replies() == [f'read-only-{i}' for i in range(5)]
                await asyncio.sleep(20)
                assert not worker.done() and len(backend_pongs) >= 3
                previews = [json.loads(packet)['payload']['video']['sequence'] for packet in browser.sent
                            if json.loads(packet)['header']['msg_type'] == 'status_sensor']
                assert previews[-1] > 1000 and len(previews) < 400
            finally:
                await browser.close()
                await asyncio.wait_for(worker, 3)
    asyncio.run(scenario())
