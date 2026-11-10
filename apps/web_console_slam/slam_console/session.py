"""Single owned session, durable logs and a pipe-bound process guardian.

Each launch guardian stays in an owned process group. Parent death closes its
stdin pipe and triggers the same graceful shutdown as an explicit stop. Linux
subreaping keeps grandchildren owned even if ros2 launch dies unexpectedly.
"""
import asyncio
import codecs
import contextlib
import ctypes
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import struct
import sys
import tempfile
import time
import uuid

import yaml

ACTIVE = {'starting', 'running', 'stopping', 'saving'}


def pcd_valid(path):
    """Validate a PCL XYZ PCD header and its declared payload, not just existence."""
    try:
        with path.open('rb') as stream:
            header = {}
            for _ in range(100):
                line = stream.readline(4096)
                if not line or len(line) >= 4096:
                    return False
                words = line.decode('ascii').strip().split()
                if not words or words[0].startswith('#'):
                    continue
                header[words[0].upper()] = words[1:]
                if words[0].upper() == 'DATA':
                    break
            points = int(header['POINTS'][0])
            if points <= 0 or not {'x', 'y', 'z'} <= set(header['FIELDS']):
                return False
            sizes = [int(x) for x in header['SIZE']]
            counts = [int(x) for x in header.get('COUNT', ['1'] * len(sizes))]
            fields, types = header['FIELDS'], header['TYPE']
            if not (len(fields) == len(set(fields)) == len(sizes) == len(counts) == len(types)):
                return False
            if any(s not in (1, 2, 4, 8) or c <= 0 or t not in ('F', 'I', 'U')
                   or (t == 'F' and s not in (4, 8)) for s, c, t in zip(sizes, counts, types)):
                return False
            if int(header['WIDTH'][0]) * int(header['HEIGHT'][0]) != points:
                return False
            if any(counts[fields.index(axis)] != 1 for axis in ('x', 'y', 'z')):
                return False
            payload_size = path.stat().st_size - stream.tell()
            kind = header['DATA'][0]
            if kind == 'binary':
                return payload_size == points * sum(s*c for s, c in zip(sizes, counts))
            if kind == 'binary_compressed':
                compressed, raw = struct.unpack('<II', stream.read(8))
                return compressed > 0 and payload_size == compressed + 8 and raw == points * sum(s*c for s, c in zip(sizes, counts))
            if kind == 'ascii':
                count, finite = 0, False
                offsets = [sum(counts[:fields.index(axis)]) for axis in ('x', 'y', 'z')]
                for line in stream:
                    if not line.strip():
                        continue
                    values = [float(value) for value in line.split()]
                    if len(values) != sum(counts):
                        return False
                    finite = finite or all(math.isfinite(values[i]) for i in offsets)
                    count += 1
                    if count > points:
                        return False
                return count == points and finite
    except (OSError, ValueError, KeyError, UnicodeError, IndexError, struct.error):
        pass
    return False


def group_members(pgid):
    members = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and int(entry.name) != os.getpid() and fields[0] != 'Z':
                members.append(int(entry.name))
        except (OSError, ValueError, IndexError):
            pass
    return members


def descendants(direct=False):
    """All descendants, including reparented children owned through subreaping."""
    parents = {}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            try:
                fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
                parents[int(entry.name)] = (int(fields[1]), fields[0])
            except (OSError, ValueError, IndexError):
                pass
    owned = {os.getpid()}
    while True:
        expanded = owned | {pid for pid, (parent, _) in parents.items() if parent in owned}
        if expanded == owned:
            break
        owned = expanded
    return [pid for pid in owned if pid != os.getpid() and parents[pid][1] != 'Z'
            and (not direct or parents[pid][0] == os.getpid())]


def guard(command, timeout, watch_stdin=True, death_signal=signal.SIGINT):
    # Subreaping also covers launch-internal process groups and service SIGKILL.
    if ctypes.CDLL(None).prctl(36, 1, 0, 0, 0) != 0:
        raise RuntimeError('无法启用 Linux child subreaper')
    stopped = False
    def request(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, request)
    signal.signal(signal.SIGTERM, request)
    parent_pid = os.getpid()
    def bind_parent():
        # This guardian is single-threaded. Bind before exec to close the fork/death race.
        ctypes.CDLL(None).prctl(1, death_signal, 0, 0, 0)  # PR_SET_PDEATHSIG
        if os.getppid() != parent_pid:
            os.kill(os.getpid(), death_signal)
    child = subprocess.Popen(command, stdin=subprocess.DEVNULL, start_new_session=True, preexec_fn=bind_parent)
    print(f'[guardian] child pid={child.pid} pgid={child.pid}', flush=True)
    while child.poll() is None and not stopped:
        readable, _, _ = select.select([sys.stdin.buffer] if watch_stdin else [], [], [], .1)
        if readable and not os.read(sys.stdin.fileno(), 1):
            stopped = True
    def send(pids, sig):
        for pid in pids:
            with contextlib.suppress(ProcessLookupError):
                os.kill(pid, sig)
    # A live launch/service gets one signal and manages its own children first.
    # After a service crash, signal adopted guardians only. Signalling every
    # descendant would interrupt a node's save twice (guardian + launch + node).
    send([child.pid] if child.poll() is None else descendants(direct=True), signal.SIGINT)
    deadline = time.monotonic() + timeout
    while descendants() and time.monotonic() < deadline:
        child.poll()
        time.sleep(.1)
    forced = bool(descendants())
    if forced:
        send(descendants(), signal.SIGTERM)
        end = time.monotonic() + 3
        while descendants() and time.monotonic() < end:
            child.poll()
            time.sleep(.1)
        send(descendants(), signal.SIGKILL)
    returncode = child.wait()
    # Reap adopted zombies before leaving the managed process tree.
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
            if not pid:
                break
        except ChildProcessError:
            break
    if forced:
        return 124
    return 0 if stopped and returncode in (0, -signal.SIGINT) else (returncode if returncode >= 0 else 128 - returncode)


class SessionManager:
    def __init__(self, config, bridge):
        self.config, self.bridge = config, bridge
        self.current = {'session_id': None, 'state': 'idle', 'save_result': None}
        self.processes = {}
        self.readers = []
        self.monitor = None
        self.stopping = None
        self.lock = asyncio.Lock()
        self.combined = None

    def status(self):
        return dict(self.current)

    def persist(self):
        path = self.directory / 'session.json'
        temporary = path.with_suffix('.tmp')
        try:
            temporary.write_text(json.dumps(self.current, ensure_ascii=False, indent=2), encoding='utf-8')
            temporary.replace(path)
        except OSError as error:
            self.current['error'] = f'会话记录写入失败：{error}'

    def maps(self):
        directory = self.config['prior_directory']
        return [{'name': p.name, 'path': str(p.resolve())} for p in sorted(directory.glob('*.pcd'))
                if p.is_file() and p.resolve().parent == directory.resolve()]

    def options(self, params):
        mode, name, save = params.get('mode'), params.get('map'), params.get('save')
        if mode not in ('mapping', 'relocation') or not isinstance(save, bool):
            raise ValueError('模式须为 mapping/relocation，save 须为布尔值')
        selected = None
        if mode == 'relocation':
            if not isinstance(name, str) or Path(name).name != name:
                raise ValueError('请选择先验地图文件名')
            selected = (self.config['prior_directory'] / name).resolve()
            if selected.parent != self.config['prior_directory'].resolve() or selected.suffix.lower() != '.pcd' or not selected.is_file():
                raise ValueError('地图不存在或超出允许目录')
            if not pcd_valid(selected):
                raise ValueError('先验地图不是含有效点的完整 XYZ PCD')
        return mode, selected, save

    def preflight(self, params):
        from .__main__ import static_check
        errors = []
        try:
            mode, _, _ = self.options(params)
            errors.extend(static_check(self.config, check_port=False))
            errors.extend(self.bridge.preflight())
            for directory in (self.config['runtime_directory'], self.config['algorithm_root'] / 'PCD' / ('priori' if mode == 'mapping' else 'relocate')):
                directory.mkdir(parents=True, exist_ok=True)
                with tempfile.TemporaryFile(dir=directory):
                    pass
        except (OSError, ValueError) as error:
            errors.append(str(error))
        return {'ok': not errors, 'errors': errors}

    async def start(self, params, request_id=None):
        from .__main__ import base_parameters
        async with self.lock:
            if self.current['state'] in ACTIVE:
                raise ValueError('已有活动 SLAM 会话')
            checked = await asyncio.to_thread(self.preflight, params)
            if not checked['ok']:
                raise ValueError('\n'.join(checked['errors']))
            mode, selected, save = self.options(params)
            sid = time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:12]
            self.directory = self.config['runtime_directory'] / sid
            self.directory.mkdir(parents=True)
            self.output = self.config['algorithm_root'] / 'PCD' / ('priori' if mode == 'mapping' else 'relocate') / sid
            self.output.mkdir(parents=True)
            data = base_parameters(self.config)
            p = data['/**']['ros__parameters']
            p.update(workspace_root=str(self.config['algorithm_root']), locate_in_prior_map=mode == 'relocation', prior_map_update_en=mode == 'relocation')
            if selected:
                p.update(prior_map_path=str(selected.parent), prior_map_name=selected.name)
            p.setdefault('common', {}).update(img_en=0, lid_topic=self.config['lidar_topic'], imu_topic=self.config['imu_topic'])
            p.setdefault('preprocess', {})['lidar_type'] = 1
            p.setdefault('publish', {})['pub_effect_point_en'] = True
            p.setdefault('pcd_save', {}).update(pcd_save_en=save, pcd_save_en_relocate=save, interval=-1,
                                               output_dir=str(self.output), output_dir_relocate=str(self.output))
            config_file = self.directory / 'config.yaml'
            config_file.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
            self.current = {'start_request_id': request_id, 'session_id': sid, 'state': 'starting', 'mode': mode, 'map': str(selected) if selected else None,
                            'save': save, 'started': time.time(), 'runtime_directory': str(self.directory),
                            'output_directory': str(self.output), 'save_result': None, 'error': None,
                            'processes': {}, 'node_failures': {}, 'save_error': None}
            self.bridge.begin(self.current['started'])
            self.combined = (self.directory / 'launch.log').open('ab', buffering=0)
            self.processes, self.readers, self.stopping = {}, [], None
            self.persist()
            try:
                launches = [('mapping', 'fast_livo', 'mapping_relocation.launch.py')]
                if mode == 'relocation':
                    launches.append(('relocation', 'relocalization', 'relocalization.launch.py'))
                for name, package, launch in launches:
                    command = ['ros2', 'launch', package, launch, f'config_file:={config_file}',
                               'sigterm_timeout:=120', 'sigkill_timeout:=5']
                    if name == 'mapping':
                        command.append('enable_rviz:=false')
                    process = await asyncio.create_subprocess_exec(sys.executable, '-m', 'slam_console.session',
                        '--guard', str(self.config['shutdown_timeout']), json.dumps(command),
                        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
                        start_new_session=True, cwd=self.config['algorithm_root'], limit=65536)
                    self.processes[name] = process
                    self.current['processes'][name] = {'pid': process.pid, 'pgid': process.pid, 'command': command}
                    for source, stream in [('stdout', process.stdout), ('stderr', process.stderr)]:
                        self.readers.append(asyncio.create_task(self.read_output(name, source, stream)))
                    self.persist()
                self.monitor = asyncio.create_task(self.watch())
            except Exception as error:
                self.current['error'] = f'启动失败：{error}'
                self.stopping = asyncio.create_task(self.finish())
            return self.status()

    async def read_output(self, name, source, stream):
        decoder = codecs.getincrementaldecoder('utf-8')('replace')
        diagnostic_tail = ''
        # Keep draining pipes even when the disk cannot create or write a log.
        # Otherwise a full pipe can prevent launch from finishing its save.
        with contextlib.ExitStack() as stack:
            raw = None
            try:
                raw = stack.enter_context((self.directory / f'{name}.{source}.log').open('ab', buffering=0))
            except OSError as error:
                self.current['error'] = f'原始日志创建失败：{error}'
                await self.stop(self.current['session_id'])
            while True:
                chunk = await stream.read(8192)
                if chunk and raw:
                    try:
                        raw.write(chunk)
                    except OSError as error:
                        self.current['error'] = f'原始日志写入失败：{error}'
                        await self.stop(self.current['session_id'])
                text = decoder.decode(chunk, final=not chunk)
                if text:
                    diagnostic_tail = (diagnostic_tail + text)[-16384:]
                    for died in re.finditer(r'process has died.*?exit code (-?\d+)', diagnostic_tail):
                        code = int(died.group(1))
                        if code:
                            self.current['node_failures'][name] = code
                            self.current['error'] = f'{name} 内部节点异常退出：{code}，详见日志'
                            if self.current['state'] in ('starting', 'running'):
                                await self.stop(self.current['session_id'])
                    if 'Final PCD save failed:' in diagnostic_tail:
                        self.current['save_error'] = '算法报告最终 PCD 写入失败，请查看日志'
                        self.current['error'] = self.current['save_error']
                    prefix = f'[{time.strftime("%Y-%m-%d %H:%M:%S")}] [{name}/{source}] '
                    try:
                        self.combined.write((prefix + text + ('' if text.endswith('\n') else '\n')).encode('utf-8'))
                    except OSError as error:
                        self.current['error'] = f'合并日志写入失败：{error}'
                        await self.stop(self.current['session_id'])
                if not chunk:
                    break

    async def watch(self):
        try:
            timeout = self.config['mapping_timeout' if self.current['mode'] == 'mapping' else 'relocation_timeout']
            while self.current['state'] in ('starting', 'running'):
                for name, process in self.processes.items():
                    if process.returncode is not None and (name == 'mapping' or process.returncode != 0):
                        self.current['error'] = f'{name} 异常退出：{process.returncode}；请查看启动日志（含时钟校验）'
                        await self.stop(self.current['session_id'])
                        return
                if self.current['state'] == 'starting':
                    if self.bridge.ready(self.current['mode']):
                        self.current['state'] = 'running'
                        self.persist()
                    elif time.time() - self.current['started'] > timeout:
                        self.current['error'] = f'初始化超时（{timeout:g}s），未收到本次有效里程计/跟踪/先验初始化证据'
                        await self.stop(self.current['session_id'])
                        return
                await asyncio.sleep(.2)
        except Exception as error:
            self.current['error'] = f'会话监控失败：{error}'
            await self.stop(self.current['session_id'])

    async def stop(self, sid):
        if not sid or sid != self.current['session_id']:
            raise ValueError('停止的 session_id 与当前任务不符')
        if self.current['state'] not in ACTIVE:
            return self.status()
        if self.stopping is None:
            self.current['state'] = 'stopping'
            self.persist()
            self.stopping = asyncio.create_task(self.finish())
        return self.status()

    async def finish(self):
        if self.monitor and not self.monitor.done():
            self.monitor.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.monitor
        forced = False
        for process in self.processes.values():
            if process.stdin:
                process.stdin.close()  # EOF is also the service-death fallback.
        self.current['state'] = 'saving' if self.current['save'] else 'stopping'
        self.persist()
        try:
            await asyncio.wait_for(asyncio.gather(*(p.wait() for p in self.processes.values())), self.config['shutdown_timeout'] + 5)
        except asyncio.TimeoutError:
            forced = True
        for process in self.processes.values():
            if process.returncode is None or group_members(process.pid):
                forced = True
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGKILL)
        await asyncio.gather(*(p.wait() for p in self.processes.values()))
        results = await asyncio.gather(*self.readers, return_exceptions=True)
        if any(isinstance(r, Exception) for r in results):
            self.current['error'] = self.current['error'] or '日志写入失败，请检查磁盘'
        if self.combined:
            self.combined.close()
        codes = {name: p.returncode for name, p in self.processes.items()}
        paths = sorted(self.output.glob('*.pcd'))
        valid = [str(p) for p in paths if pcd_valid(p)]
        if not self.current['save']:
            outcome = 'disabled'
        elif forced:
            outcome = 'unconfirmed'
        elif any(code == 124 or code < 0 for code in codes.values()):
            outcome = 'unconfirmed'
        elif (any(code != 0 for code in codes.values()) or
              self.current['node_failures'] or self.current['save_error']):
            outcome = 'failed'
        elif valid and len(valid) == len(paths):
            outcome = 'success'
        elif paths:
            outcome = 'failed'
        elif self.current['error']:
            outcome = 'unconfirmed'
        else:
            outcome = 'no_valid_map'
        self.current.update(state='failed' if self.current['error'] or forced or outcome == 'failed' else 'stopped',
                            save_result=outcome, maps=valid, returncodes=codes, ended=time.time())
        self.persist()

    def logs(self, sid, cursor=0):
        if not isinstance(sid, str) or not re.fullmatch(r'[0-9]{8}_[0-9]{6}_[a-f0-9]{12}', sid):
            raise ValueError('无效 session_id')
        if type(cursor) is not int or cursor < 0:
            raise ValueError('日志游标须为非负整数')
        directory = (self.config['runtime_directory'] / sid).resolve()
        if directory.parent != self.config['runtime_directory'].resolve():
            raise ValueError('会话路径越界')
        with (directory / 'launch.log').open('rb') as log:
            size = os.fstat(log.fileno()).st_size
            cursor = min(cursor, size)
            log.seek(cursor)
            data = log.read(65536)
            # Keep byte cursors exact and never split a trailing UTF-8 character.
            decoded = codecs.getincrementaldecoder('utf-8')('replace')
            text = decoded.decode(data, final=False)
            pending = decoded.getstate()[0]
            next_cursor = cursor + len(data) - len(pending)
        return {'session_id': sid, 'cursor': cursor, 'next_cursor': next_cursor, 'size': size, 'text': text}

    async def close(self):
        if self.current['state'] in ACTIVE:
            await self.stop(self.current['session_id'])
        if self.stopping:
            await self.stopping
        if self.monitor and not self.monitor.done():
            self.monitor.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self.monitor


if __name__ == '__main__':
    if len(sys.argv) != 4 or sys.argv[1] != '--guard':
        raise SystemExit('internal launch guardian only')
    raise SystemExit(guard(json.loads(sys.argv[3]), float(sys.argv[2])))
