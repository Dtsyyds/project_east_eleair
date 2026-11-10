"""统一 HTTP/WebSocket 入口；固定路由，不提供任意地址代理或控制重放。

/ws/inspection 转发原相机协议，/ws/robot 转发底盘协议。
每条浏览器连接对应独立上游连接；断开时关闭上游，保留原控制租约的停车语义。
"""
import argparse
import asyncio
import contextlib
from collections import OrderedDict, deque
from datetime import datetime
from http import HTTPStatus
from itertools import count
import json
import mimetypes
from pathlib import Path
import signal
from urllib.parse import unquote, urlsplit

from websockets.exceptions import ConnectionClosed, InvalidHandshake
from websockets.legacy.client import connect
from websockets.legacy.server import serve

WEB_ROOT = Path(__file__).resolve().parents[1] / 'apps/web_console'
# Keep command forwarding and chassis feedback deadlines unchanged. Camera data
# may fill the TCP write buffer briefly; it must not inherit a command deadline.
CONTROL_SEND_TIMEOUT = .25
INSPECTION_SEND_TIMEOUT = 2.0
INSPECTION_BUFFER_BYTES = 8 * 1024 * 1024
INSPECTION_RELIABLE_MESSAGES = 64
INSPECTION_PREVIEW_STREAMS = 16
PREVIEW_MAX_AGE = 2.0


def preview_key(packet):
    """Only the existing, explicit visualization-only envelopes may be replaced.

    Responses, events, status snapshots, binary data and unknown schemas remain
    FIFO. Return the original packet unchanged; never rewrite image metadata.
    """
    if not isinstance(packet, str):
        return None
    try:
        value = json.loads(packet)
    except (ValueError, RecursionError):
        return None
    if not isinstance(value, dict) or set(value) != {'header', 'payload'}:
        return None
    header, payload = value['header'], value['payload']
    if (not isinstance(header, dict) or not isinstance(payload, dict) or
            set(header) != {'robot_id', 'msg_type', 'schema_version'} or
            header['schema_version'] != 'tank_inspection_v2'):
        return None
    robot = header['robot_id']
    if not isinstance(robot, str) or not 1 <= len(robot) <= 128:
        return None
    if set(payload) == {'video'} and header['msg_type'] == 'status_sensor':
        video = payload['video']
        if (isinstance(video, dict) and video.get('visualization_only') is True and
                video.get('encoding') == 'jpeg_base64' and
                video.get('camera_id') in ('fisheye', 'mindvision', 'depth_camera')):
            return (robot, 'video', video['camera_id'])
    for field, kind in (('lidar_cloud', 'status_sensor'), ('eca', 'sensor_data')):
        if (set(payload) == {field} and header['msg_type'] == kind and
                isinstance(payload[field], dict) and payload[field].get('visualization_only') is True):
            return (robot, field)
    return None


class InspectionBuffer:
    """Nonblocking upstream drain, bounded FIFO + latest preview per stream."""

    def __init__(self, stats):
        self.reliable = deque()
        self.previews = OrderedDict()
        self.bytes = 0
        self.ready = asyncio.Event()
        self.ending = None
        self.close_code = None
        self.stats = stats
        stats.update(received_messages=0, previews_replaced=0, previews_expired=0,
                     queued_bytes=0, peak_queued_bytes=0, peak_reliable_messages=0)

    def put(self, packet):
        size = len(packet.encode('utf-8')) if isinstance(packet, str) else len(packet)
        key = preview_key(packet)
        old = self.previews.get(key)
        new_bytes = self.bytes + size - (old[1] if old else 0)
        if (new_bytes > INSPECTION_BUFFER_BYTES or
                (key is None and len(self.reliable) >= INSPECTION_RELIABLE_MESSAGES) or
                (key is not None and old is None and len(self.previews) >= INSPECTION_PREVIEW_STREAMS)):
            raise BufferError('inspection queue full')
        item = (packet, size, asyncio.get_running_loop().time())
        if key is None:
            self.reliable.append(item)
        else:
            # Replacing a value keeps stream order, so a busy camera cannot
            # starve another camera. The current in-flight frame is never cut.
            self.previews[key] = item
            if old:
                self.stats['previews_replaced'] += 1
        self.bytes = new_bytes
        self.stats['received_messages'] += 1
        self.stats['queued_bytes'] = self.bytes
        self.stats['peak_queued_bytes'] = max(self.stats['peak_queued_bytes'], self.bytes)
        self.stats['peak_reliable_messages'] = max(self.stats['peak_reliable_messages'], len(self.reliable))
        self.ready.set()

    def finish(self, ending, discard=False):
        self.ending = ending
        self.close_code = ending.get('received_code')
        if discard:
            self.reliable.clear()
            self.previews.clear()
            self.bytes = self.stats['queued_bytes'] = 0
        self.ready.set()

    def __aiter__(self):
        return self

    async def __anext__(self):
        while True:
            if self.reliable or self.previews:
                reliable = bool(self.reliable)
                packet, size, received = self.reliable.popleft() if reliable else self.previews.popitem(last=False)[1]
                self.bytes -= size
                self.stats['queued_bytes'] = self.bytes
                if not reliable and asyncio.get_running_loop().time() - received > PREVIEW_MAX_AGE:
                    self.stats['previews_expired'] += 1
                    continue
                return packet
            if self.ending is not None:
                raise StopAsyncIteration
            self.ready.clear()
            await self.ready.wait()


def connection_end(endpoint, error=None):
    """Classify transport closure without logging peer-supplied text or payloads."""
    sent = getattr(error, 'sent', None)
    received = getattr(error, 'rcvd', None)
    reason = f'{endpoint}_closed'
    if sent and sent.code == 1011 and sent.reason == 'keepalive ping timeout':
        reason = f'{endpoint}_heartbeat_timeout'
    elif (sent and sent.code == 1009) or (received and received.code == 1009):
        reason = f'{endpoint}_message_too_large'
    return {'reason': reason, 'sent_code': sent.code if sent else None,
            'received_code': received.code if received else None}


class RobotGateway:
    """只允许两个固定本机上游；不缓存/重试运动命令，不合并两种状态字段。"""

    def __init__(self, camera_port=9003, robot_port=9002, http_port=8081,
                 web_root=WEB_ROOT, public_ws_port=9001):
        self.web_root = Path(web_root).resolve()
        self.public_ws_port = public_ws_port
        self.routes = {'/ws/inspection': f'ws://127.0.0.1:{camera_port}/',
                       '/ws/robot': f'ws://127.0.0.1:{robot_port}/ws'}
        # 统一入口的根路径和 /ws 仍可作为相机协议别名。
        self.routes['/'] = self.routes['/ws'] = self.routes['/ws/inspection']
        self.clients = set()
        self.connection_ids = count(1)
        # 原服务信任的本机网页 Origin；外部 Origin 在网关握手时单独校验。
        self.upstream_origin = f'http://127.0.0.1:{http_port}'

    def record(self, event, **fields):
        # Fixed fields only: no packets, request bodies, tokens or close reasons
        # from peers. Flush because robot_session redirects stdout to gateway.log.
        print('[gateway] ' + json.dumps({
            'time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
            'event': event, **fields}, ensure_ascii=False), flush=True)

    async def forward(self, source, destination, source_name, destination_name, timeout, stats):
        direction = f'{source_name}_to_{destination_name}'
        loop = asyncio.get_running_loop()
        try:
            async for packet in source:
                size = len(packet.encode('utf-8')) if isinstance(packet, str) else len(packet)
                started = loop.time()
                try:
                    await asyncio.wait_for(destination.send(packet), timeout=timeout)
                except asyncio.TimeoutError:
                    return {'reason': f'{destination_name}_send_timeout', 'direction': direction,
                            'packet_bytes': size, 'timeout_ms': round(timeout * 1000),
                            'send_ms': round((loop.time() - started) * 1000)}
                except ConnectionClosed as error:
                    return {**connection_end(destination_name, error), 'direction': direction}
                elapsed = loop.time() - started
                stats['messages'] += 1
                stats['bytes'] += size
                stats['max_send_ms'] = max(stats['max_send_ms'], round(elapsed * 1000))
                if elapsed > CONTROL_SEND_TIMEOUT:
                    stats['slow_sends'] += 1
        except ConnectionClosed as error:
            return {**connection_end(source_name, error), 'direction': direction}
        return {**connection_end(source_name), 'direction': direction,
                'received_code': source.close_code}

    async def inspection_downstream(self, upstream, browser, stats, connection_id, route):
        buffer = InspectionBuffer(stats)

        async def receive():
            last_report = asyncio.get_running_loop().time()
            try:
                async for packet in upstream:
                    buffer.put(packet)  # Never wait on browser I/O here: drain Pong too.
                    now = asyncio.get_running_loop().time()
                    if now - last_report >= 5:
                        self.record('inspection_flow', connection_id=connection_id, route=route,
                                    transfers=dict(stats))
                        last_report = now
            except BufferError:
                # Critical messages aren't silently dropped to make room for video.
                # A client that cannot keep up with even the FIFO must reconnect.
                buffer.finish({'reason': 'inspection_queue_overflow', 'direction': 'upstream_to_browser'}, discard=True)
                return
            except ConnectionClosed as error:
                buffer.finish({**connection_end('upstream', error), 'direction': 'upstream_to_browser'})
                return
            except OSError as error:
                buffer.finish({'reason': 'transport_error', 'direction': 'upstream_to_browser',
                               'errno': error.errno, 'error_type': type(error).__name__}, discard=True)
                return
            except Exception as error:
                buffer.finish({'reason': 'gateway_error', 'direction': 'upstream_to_browser',
                               'error_type': type(error).__name__}, discard=True)
                return
            buffer.finish({'reason': 'upstream_closed', 'direction': 'upstream_to_browser',
                           'received_code': upstream.close_code})

        reader = asyncio.create_task(receive())
        try:
            ending = await self.forward(buffer, browser, 'upstream', 'browser', INSPECTION_SEND_TIMEOUT, stats)
            return buffer.ending if ending['reason'] == 'upstream_closed' and buffer.ending else ending
        finally:
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)

    async def http_request(self, path, headers):
        """只读提供上位机静态文件，不暴露数据盘、配置、日志或源码目录。"""
        route = urlsplit(path).path
        if headers.get('Upgrade', '').lower() == 'websocket':
            if route in self.routes and path == route:
                return None
            return HTTPStatus.NOT_FOUND, [], b'Unknown WebSocket route\n'
        if route == '/connection_config.js':
            body = ('"use strict";(()=>{const h=location.hostname.includes(":")?"["+location.hostname+"]":location.hostname;'
                    'const b=(location.protocol==="https:"?"wss://":"ws://")+h+":"+'
                    + json.dumps(str(self.public_ws_port)) + ';'
                    'window.RobotConnectionDefaults=Object.freeze({unified:true,'
                    'inspection:b+"/ws/inspection",robot:b+"/ws/robot"});})();').encode()
            return HTTPStatus.OK, [('Content-Type', 'application/javascript'),
                                   ('Cache-Control', 'no-store')], body
        try:
            relative = unquote(route).lstrip('/') or 'index.html'
            target = (self.web_root / relative).resolve()
            target.relative_to(self.web_root)
            if (target.suffix.lower() not in ('.html', '.js', '.css', '.ico', '.png', '.svg')
                    or not target.is_file() or target.stat().st_size > 2 * 1024 * 1024):
                raise ValueError('not_a_static_asset')
            body = target.read_bytes()
        except (OSError, ValueError):
            return HTTPStatus.NOT_FOUND, [], b'Not found\n'
        kind = mimetypes.guess_type(str(target))[0] or 'application/octet-stream'
        return HTTPStatus.OK, [('Content-Type', kind), ('Cache-Control', 'no-store'),
                               ('X-Content-Type-Options', 'nosniff')], body

    async def websocket(self, browser, path):
        """任何一侧断开均关闭另一侧；无法连接时让页面重连，不排队旧命令。"""
        if path not in self.routes:
            await browser.close(code=1008, reason='unknown_route')
            return
        if len(self.clients) >= 8:
            self.record('rejected', route=path, reason='gateway_client_limit')
            await browser.close(code=1008, reason='gateway_client_limit')
            return
        self.clients.add(browser)
        connection_id = next(self.connection_ids)
        started = asyncio.get_running_loop().time()
        downstream_timeout = CONTROL_SEND_TIMEOUT if path == '/ws/robot' else INSPECTION_SEND_TIMEOUT
        totals = {direction: dict(messages=0, bytes=0, max_send_ms=0, slow_sends=0)
                  for direction in ('browser_to_upstream', 'upstream_to_browser')}
        ending = {'reason': 'gateway_shutdown'}
        tasks = []
        upstream = None
        try:
            # 用标准 asyncio 控制握手期限，避免依赖特定 websockets 版本的超时参数。
            upstream = await asyncio.wait_for(connect(
                self.routes[path], origin=self.upstream_origin, close_timeout=.2,
                compression=None, max_size=16 * 1024 * 1024, max_queue=1,
                write_limit=65536, ping_interval=5, ping_timeout=3), timeout=2)
            self.record('connected', connection_id=connection_id, route=path,
                        downstream_timeout_ms=round(downstream_timeout * 1000),
                        downstream_policy='direct' if path == '/ws/robot' else 'latest_preview',
                        ping_interval_s=5, ping_timeout_s=3)
            downstream = (self.forward(upstream, browser, 'upstream', 'browser',
                                       downstream_timeout, totals['upstream_to_browser']) if path == '/ws/robot'
                          else self.inspection_downstream(upstream, browser, totals['upstream_to_browser'], connection_id, path))
            tasks = [asyncio.create_task(self.forward(browser, upstream, 'browser', 'upstream',
                                                     CONTROL_SEND_TIMEOUT, totals['browser_to_upstream'])),
                     asyncio.create_task(downstream)]
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            endings = [task.result() for task in tasks if task in done]
            # Preserve a concrete timeout if the other direction closed concurrently.
            ending = next((item for item in endings if item['reason'].endswith('_timeout')), endings[0])
        except asyncio.TimeoutError:
            ending = {'reason': 'upstream_connect_timeout', 'timeout_ms': 2000}
        except InvalidHandshake as error:
            ending = {'reason': 'upstream_handshake_rejected', 'error_type': type(error).__name__}
        except OSError as error:
            ending = {'reason': 'transport_error', 'error_type': type(error).__name__, 'errno': error.errno}
        except ConnectionClosed as error:
            ending = connection_end('upstream', error)
        except Exception as error:
            ending = {'reason': 'gateway_error', 'error_type': type(error).__name__}
        finally:
            self.clients.discard(browser)
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            self.record('disconnected', connection_id=connection_id, route=path,
                        duration_ms=round((asyncio.get_running_loop().time() - started) * 1000),
                        **ending, transfers=totals)
            if upstream is not None:
                with contextlib.suppress(OSError, ConnectionClosed, asyncio.TimeoutError):
                    await upstream.close()
            await browser.close(code=1013, reason=ending['reason'])


async def run(args):
    gateway = RobotGateway(args.camera_port, args.robot_port, args.camera_http_port,
                           web_root=args.web_root, public_ws_port=args.port)
    stopped = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stopped.set)
    origins = [f'http://{host}:{args.http_port}' for host in (*args.hosts, '127.0.0.1', 'localhost')]
    try:
        async with contextlib.AsyncExitStack() as stack:
            for port in (args.http_port, args.port):
                await stack.enter_async_context(serve(
                    gateway.websocket, '0.0.0.0', port, process_request=gateway.http_request,
                    origins=list(set(origins)), max_size=16384, max_queue=1, write_limit=65536,
                    compression=None, ping_interval=5, ping_timeout=3, close_timeout=.2))
            for host in args.hosts:
                print(f'Console: http://{host}:{args.http_port}/; WebSocket: {args.port}', flush=True)
            await stopped.wait()
    finally:
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.remove_signal_handler(sig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=9001)
    parser.add_argument('--camera-port', type=int, default=9003)
    parser.add_argument('--robot-port', type=int, default=9002)
    parser.add_argument('--http-port', type=int, default=8080)
    parser.add_argument('--camera-http-port', type=int, default=8081)
    parser.add_argument('--host', action='append', help='可重复指定；默认同时放行 RDK 网线与无线网页地址')
    parser.add_argument('--web-root', type=Path, default=WEB_ROOT)
    args = parser.parse_args()
    args.hosts = list(dict.fromkeys(args.host or ['192.168.127.10', '192.168.1.148']))
    if any(not host or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-:'
                           for c in host) for host in args.hosts):
        parser.error('host 只接受 IP/主机名，不接受 URL、路径或凭据')
    ports = (args.port, args.camera_port, args.robot_port, args.http_port, args.camera_http_port)
    if len(set(ports)) != 5 or any(not 1 <= port <= 65535 for port in ports):
        parser.error('统一入口和后台端口必须有效且互不相同')
    asyncio.run(run(args))


if __name__ == '__main__':
    main()
