import asyncio
from http import HTTPStatus
import json
from types import SimpleNamespace

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from websockets.exceptions import InvalidStatus
from websockets.datastructures import Headers
from websockets.server import ServerProtocol

from slam_console.server import Server


class Manager:
    current={'session_id':None,'state':'idle'}
    bridge=SimpleNamespace(previews=lambda:{})
    def status(self):return dict(self.current)
    def maps(self):return []


def test_http_origin_host_and_static_allowlist():
    app=Server(Manager(),['localhost'])
    class Connection:
        # Use the actual response/header semantics, including default text/plain.
        # A plain dict would miss duplicate Content-Type on CSS and JS.
        def respond(self,status,body):return ServerProtocol().reject(status,body)
    def request(path='/',host='localhost:8082',origin=None):
        headers=Headers({'Host':host})
        if origin:headers['Origin']=origin
        return app.http(Connection(),SimpleNamespace(path=path,headers=headers))
    result=request()
    assert result.status_code==200 and 'frame-ancestors http://localhost:8080 http://localhost:8082' in result.headers['Content-Security-Policy']
    for path, mime in [('/', 'text/html'), ('/index.html', 'text/html'),
                       ('/theme.css?v=regression', 'text/css'), ('/console.js?v=regression', 'text/javascript')]:
        result = request(path)
        assert result.headers.get_all('Content-Type') == [mime + '; charset=utf-8']
        assert result.headers['X-Content-Type-Options'] == 'nosniff'
        assert "connect-src 'self' ws://localhost:8082;" in result.headers['Content-Security-Policy']
    assert request('/config/slam.yaml').status_code==404
    assert request('/../../etc/passwd').status_code==404
    assert request(host='attacker:8082').status_code==403
    assert request('/ws',origin='http://attacker:8080').status_code==403
    assert request('/ws').status_code==403
    assert request('/ws',origin='http://localhost:8082') is None


def test_real_http_websocket_share_port_and_reconnect_without_start():
    async def scenario():
        app=Server(Manager(),['127.0.0.1'])
        async with serve(app.client,'127.0.0.1',0,process_request=app.http) as server:
            port=server.sockets[0].getsockname()[1]
            app.hosts={f'127.0.0.1:{port}'};origin=f'http://127.0.0.1:{port}';app.origins={origin}
            for path, mime, content in [('/', b'text/html', '定位与建图'.encode()),
                                        ('/theme.css?v=regression', b'text/css', b'--panel: #141f2d'),
                                        ('/console.js?v=regression', b'text/javascript', b'new WebSocket')]:
                reader,writer=await asyncio.open_connection('127.0.0.1',port)
                writer.write(f'GET {path} HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\nConnection: close\r\n\r\n'.encode());await writer.drain()
                response=await reader.read()
                headers, body = response.split(b'\r\n\r\n', 1)
                types = [line for line in headers.lower().split(b'\r\n') if line.startswith(b'content-type:')]
                assert types == [b'content-type: ' + mime + b'; charset=utf-8']
                assert b'200 OK' in headers and content in body
                assert b'cache-control: no-store' in headers.lower()
                writer.close();await writer.wait_closed()
            for _ in range(2):
                async with connect(f'ws://127.0.0.1:{port}/ws',origin=origin,proxy=None) as ws:
                    await ws.send(json.dumps({'op':'status','request_id':'status'}))
                    while True:
                        reply=json.loads(await ws.recv())
                        if reply.get('request_id')=='status':break
                    assert reply['ok'] and reply['result']['state']=='idle'
            with pytest.raises(InvalidStatus):
                async with connect(f'ws://127.0.0.1:{port}/ws',origin='http://attacker',proxy=None):pass
    asyncio.run(scenario())
