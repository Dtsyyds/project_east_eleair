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
