"""Requested event snapshots only; shares installed bounded worker lifecycle."""
import copy
from pathlib import Path
from queue import Queue
from tank_vision.async_lifecycle import AsyncLifecycle
from .contracts import Frame, dump, sha256
from .dataset import save_sample


class EventWriter:
    def __init__(self, root, capacity=3, maximum=30, interval_sec=1., process=None):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.maximum, self.interval_ns = maximum, int(interval_sec * 1e9)
        self.last_stamp = None
        self.limited = 0
        self.entries = []
        dump(self.root / 'result_manifest.json', {'schema': 'd405_events_v1', 'complete': False, 'state': 'NEW'})
        self.worker = AsyncLifecycle(Queue(maxsize=capacity), process or self._write, 'd405-event-writer', self._finalize)

    def request(self, frame, result, diagnostics=None):
        self.worker.check()
        if self.worker.accepted >= self.maximum or (self.last_stamp is not None and 0 <= frame.image_stamp_ns - self.last_stamp < self.interval_ns):
            self.limited += 1; return False
        if self.worker.queue.full():
            with self.worker.lock: self.worker.dropped += 1
            return False
        # Copy only on an explicit accepted save request, never on the normal hot path.
        frozen = copy.copy(frame)
        frozen.image = frame.image.copy()
        frozen.depth = None if frame.depth is None else frame.depth.copy()
        frozen.raw_valid = None if frame.raw_valid is None else frame.raw_valid.copy()
        frozen.source = copy.deepcopy(frame.source)
        item = (frozen, copy.deepcopy(result), None if diagnostics is None else {k: None if v is None else v.copy() for k, v in diagnostics.items()})
        accepted = self.worker.submit(item, policy='drop_newest')
        if accepted: self.last_stamp = frame.image_stamp_ns
        return accepted

    def _write(self, item):
        import numpy as np
        frame, result, diagnostics = item
        name = f'event_{frame.sequence:08d}_{frame.image_stamp_ns}'
        directory = self.root / name
        directory.mkdir(exist_ok=False)
        record = save_sample(directory, frame, 'raw', frame.source.get('session', 'live'))
        dump(directory / 'result.json', result)
        paths = [directory / 'raw.npz', directory / 'result.json']
        if diagnostics is not None:
            np.savez_compressed(directory / 'diagnostics.npz', **{k: v for k, v in diagnostics.items() if v is not None})
            paths.append(directory / 'diagnostics.npz')
        meta = {'complete': True, 'source': frame.metadata(), 'sample': record, 'files': [{'path': p.name, 'sha256': sha256(p)} for p in paths]}
        # Metadata last; no valid metadata when any payload or hash failed.
        dump(directory / 'metadata.json', meta)
        self.entries.append({'path': name, 'metadata_sha256': sha256(directory / 'metadata.json')})

    def _finalize(self):
        status = self.worker.status
        complete = status['state'] != 'FAILED' and status['failed'] == status['abandoned'] == status['unfinished'] == 0
        if complete:
            status = dict(status, state='CLOSED', worker_alive=False, incomplete=False)
        dump(self.root / 'result_manifest.json', {'schema': 'd405_events_v1', 'complete': complete,
              'worker': status, 'rate_or_capacity_limited': self.limited, 'events': self.entries})

    def close(self, timeout=5.):
        self.worker.close(timeout)


class SampleCapture:
    """One explicit request; ACK is distinct from persisted metadata completion."""
    def __init__(self, writer, clock=None):
        import time
        self.writer = writer
        self.clock = clock or time.monotonic
        self.current = {'status': 'idle'}
        self.deadline = 0.

    def request(self):
        import uuid
        if self.poll()['status'] in ('queued', 'writing'):
            raise RuntimeError('已有样本正在保存')
        self.writer.worker.check()
        self.current = {'status': 'queued', 'request_id': uuid.uuid4().hex, 'message': '等待下一组配对输入'}
        self.deadline = self.clock() + 8.
        return dict(self.current)

    def capture(self, frame, result, diagnostics):
        if self.poll()['status'] != 'queued': return
        if not self.writer.request(frame, result, diagnostics):
            self.current.update(status='failed', message='样本保存被限流、队列已满或已达到本次 30 组上限')
            return
        self.current.update(status='writing', path=f'event_{frame.sequence:08d}_{frame.image_stamp_ns}',
                            message='正在写入并校验原始样本')

    def poll(self):
        import json
        if self.current['status'] == 'writing':
            metadata = self.writer.root / self.current['path'] / 'metadata.json'
            if metadata.exists():
                try:
                    value = json.loads(metadata.read_text())
                    if value.get('complete') is True:
                        self.current.update(status='completed', message='样本文件已写入', files=value['files'])
                except (OSError, ValueError, KeyError) as error:
                    self.current.update(status='failed', message='样本元数据无效：' + str(error))
            elif self.writer.worker.status['state'] == 'FAILED':
                self.current.update(status='failed', message='样本写入失败，请检查磁盘和实验日志')
        if self.current['status'] == 'queued' and self.clock() > self.deadline:
            self.current.update(status='failed', message='等待配对输入超时，没有保存样本')
        return dict(self.current)
