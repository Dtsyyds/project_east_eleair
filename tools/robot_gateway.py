"""统一 HTTP/WebSocket 入口；固定路由，不提供任意地址代理或控制重放。

/ws/inspection 转发原相机协议，/ws/robot 转发底盘协议。
每条浏览器连接对应独立上游连接；断开时关闭上游，保留原控制租约的停车语义。
"""
import argparse
import asyncio
import contextlib
from http import HTTPStatus
import json
import mimetypes
from pathlib import Path
import signal
from urllib.parse import unquote, urlsplit

from websockets.exceptions import ConnectionClosed, InvalidHandshake
from websockets.legacy.client import connect
from websockets.legacy.server import serve

WEB_ROOT = Path(__file__).resolve().parents[1] / 'apps/web_console'


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
        # 原服务信任的本机网页 Origin；外部 Origin 在网关握手时单独校验。
        self.upstream_origin = f'http://127.0.0.1:{http_port}'

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
        if len(self.clients) >= 8:
            await browser.close(code=1008, reason='gateway_client_limit')
            return
        self.clients.add(browser)
        tasks = []
        upstream = None
        try:
            # 用标准 asyncio 控制握手期限，避免依赖特定 websockets 版本的超时参数。
            upstream = await asyncio.wait_for(connect(
                self.routes[path], origin=self.upstream_origin, close_timeout=.2,
                compression=None, max_size=16 * 1024 * 1024, max_queue=1,
                write_limit=65536, ping_interval=5, ping_timeout=3), timeout=2)
            async def forward(source, destination):
                async for packet in source:
                    await asyncio.wait_for(destination.send(packet), timeout=.25)
            tasks = [asyncio.create_task(forward(browser, upstream)),
                     asyncio.create_task(forward(upstream, browser))]
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (OSError, KeyError, ConnectionClosed, InvalidHandshake, asyncio.TimeoutError):
            pass  # 不记录命令正文、Token 或完整握手信息。
        finally:
            self.clients.discard(browser)
            for task in tasks:
                task.cancel()
            for task in tasks:
                with contextlib.suppress(asyncio.CancelledError, ConnectionClosed, asyncio.TimeoutError):
                    await task
            if upstream is not None:
                await upstream.close()
            await browser.close(code=1013, reason='backend_closed_or_unavailable')


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
