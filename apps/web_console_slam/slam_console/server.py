"""Same-port static HTTP and bounded WebSocket control/preview transport."""
import asyncio
from collections import OrderedDict
import contextlib
from http import HTTPStatus
import json
from pathlib import Path
import signal
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from .session import SessionManager

APP = Path(__file__).resolve().parents[1]
VERSION = '20261111-position'
STATIC = {'/': ('index.html', 'text/html; charset=utf-8'),
          '/index.html': ('index.html', 'text/html; charset=utf-8'),
          '/console.js': ('console.js', 'text/javascript; charset=utf-8'),
          '/theme.css': ('theme.css', 'text/css; charset=utf-8')}


def authority(host):
    return f'[{host}]' if ':' in host else host


class Server:
    def __init__(self, manager, hosts):
        self.manager = manager
        self.hosts = {authority(h) + ':8082' for h in hosts}
        self.origins = {f'http://{authority(h)}:{p}' for h in hosts for p in (8080, 8082)}
        self.cache = OrderedDict()
        self.control_lock = asyncio.Lock()
        self.closing = False
        self.inflight = set()

    def http(self, connection, request):
        try:
            host = request.headers.get('Host')
            origin = request.headers.get('Origin')
        except Exception:
            return connection.respond(HTTPStatus.BAD_REQUEST, 'invalid headers')
        if host not in self.hosts or (origin is not None and origin not in self.origins):
            return connection.respond(HTTPStatus.FORBIDDEN, 'Host/Origin not allowed')
        path = request.path.split('?', 1)[0]
        if path == '/ws':
            if origin not in self.origins:
                return connection.respond(HTTPStatus.FORBIDDEN, 'Browser Origin required')
            return None
        if path not in STATIC:
            return connection.respond(HTTPStatus.NOT_FOUND, 'Not found')
        filename, mime = STATIC[path]
        response = connection.respond(HTTPStatus.OK, (APP / filename).read_text(encoding='utf-8'))
        # respond() supplies text/plain. websockets.Headers assignment APPENDS;
        # it doesn't replace like dict. Duplicate MIME types + nosniff can make
        # browsers reject CSS/JS and leave only the static "connecting" text.
        del response.headers['Content-Type']
        response.headers['Content-Type'] = mime
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Content-Security-Policy'] = ("default-src 'self'; script-src 'self'; style-src 'self'; "
            f"connect-src 'self' ws://{host}; "
            "frame-ancestors " + ' '.join(sorted(self.origins)))
        return response

    async def dispatch(self, request):
        if not isinstance(request, dict):
            raise ValueError('请求须为 JSON 对象')
        rid, op = request.get('request_id'), request.get('op')
        if not isinstance(rid, str) or not 1 <= len(rid) <= 128:
            raise ValueError('需要 1..128 字符 request_id')
        params = request.get('params', {})
        if not isinstance(params, dict):
            raise ValueError('params 须为对象')
        if op == 'logs.read':
            try:
                result = await asyncio.to_thread(self.manager.logs, request.get('session_id'), params.get('cursor', 0))
                return {'type': 'response', 'request_id': rid, 'ok': True, 'result': result, 'session_id': request.get('session_id')}
            except (ValueError, OSError) as error:
                return {'type': 'response', 'request_id': rid, 'ok': False, 'error': str(error), 'session_id': request.get('session_id')}
        cacheable = op in ('start', 'stop', 'preflight')
        # Serialize mutations across clients. Repeated IDs with different payloads are rejected.
        async with self.control_lock:
            signature = json.dumps(request, sort_keys=True, ensure_ascii=False)
            if cacheable and rid in self.cache:
                previous, response = self.cache[rid]
                if signature != previous:
                    raise ValueError('request_id 已被不同请求使用')
                return response
            try:
                if self.closing and op == 'start':
                    raise ValueError('服务正在退出')
                if op == 'status':
                    result = self.manager.status()
                    result['backend_version'] = VERSION
                    start_id = params.get('start_request_id')
                    if start_id:
                        cached = self.cache.get(start_id)
                        accepted = result.get('start_request_id') == start_id or (cached and cached[1]['ok'])
                        result['start_resolution'] = 'accepted' if accepted else 'not_accepted'
                elif op == 'maps.list':
                    result = self.manager.maps()
                elif op == 'preflight':
                    result = await asyncio.to_thread(self.manager.preflight, params)
                elif op == 'start':
                    result = await self.manager.start(params, request_id=rid)
                elif op == 'stop':
                    result = await self.manager.stop(request.get('session_id'))
                elif op == 'logs.read':
                    result = await asyncio.to_thread(self.manager.logs, request.get('session_id'), params.get('cursor', 0))
                else:
                    raise ValueError('未知操作')
                response = {'type': 'response', 'request_id': rid, 'ok': True, 'result': result,
                            'session_id': self.manager.current['session_id']}
            except (ValueError, OSError, KeyError, TypeError) as error:
                response = {'type': 'response', 'request_id': rid, 'ok': False, 'error': str(error),
                            'session_id': self.manager.current['session_id']}
            if cacheable:
                self.cache[rid] = (signature, response)
                while len(self.cache) > 256:
                    self.cache.popitem(last=False)
            return response

    async def client(self, websocket):
        # One bounded control queue; previews live in the bridge's two latest-value slots.
        responses = asyncio.Queue(maxsize=8)
        async def receive():
            async for data in websocket:
                try:
                    task = asyncio.create_task(self.dispatch(json.loads(data)))
                    self.inflight.add(task)
                    task.add_done_callback(self.inflight.discard)
                    response = await asyncio.shield(task)
                except (ValueError, TypeError) as error:
                    response = {'type': 'response', 'ok': False, 'error': str(error)}
                await responses.put(response)
        async def send(payload):
            await asyncio.wait_for(websocket.send(json.dumps(payload, ensure_ascii=False, allow_nan=False)), 3)
        async def output():
            seen, last_status = {}, 0
            while True:
                # Prioritize replies over new previews and periodic state.
                try:
                    response = await asyncio.wait_for(responses.get(), .05)
                except asyncio.TimeoutError:
                    response = None
                if response is not None:
                    await send(response)
                    continue
                now = asyncio.get_running_loop().time()
                if now - last_status >= .5:
                    await send({'type': 'status', 'backend_version': VERSION, **self.manager.status()})
                    last_status = now
                sid = self.manager.current['session_id']
                for topic, preview in self.manager.bridge.previews().items():
                    key = (sid, preview['received'])
                    if seen.get(topic) != key:
                        await send({**preview, 'session_id': sid, 'age': max(0, time.time() - preview['received'])})
                        seen[topic] = key
        tasks = [asyncio.create_task(receive()), asyncio.create_task(output())]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                with contextlib.suppress(ConnectionClosed, asyncio.TimeoutError):
                    task.result()
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await websocket.close()


async def run(config, hosts):
    from .ros_bridge import RosBridge
    bridge = RosBridge(config)
    manager = SessionManager(config, bridge)
    app = Server(manager, hosts)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    try:
        async with serve(app.client, '0.0.0.0', 8082, process_request=app.http,
                         origins=sorted(app.origins), max_size=16384, max_queue=8,
                         write_limit=65536, ping_interval=20, ping_timeout=20, close_timeout=3):
            print('SLAM 空闲；网页：' + ', '.join(f'http://{authority(h)}:8082/' for h in hosts), flush=True)
            await stop.wait()
            app.closing = True
            # Finish any accepted start before closing; then keep WS online during save.
            async with app.control_lock:
                await manager.close()
    finally:
        app.closing = True
        await manager.close()
        bridge.close()
