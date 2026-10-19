"""Shared bounded worker lifecycle. No thread is killed on timeout."""
from __future__ import annotations
from queue import Empty, Full, Queue
import threading
import time

class WorkerFailed(RuntimeError):
    pass

class AsyncLifecycle:
    def __init__(self, queue: Queue, process, name: str, finalize=None):
        if queue.maxsize <= 0:
            raise ValueError("worker queue must be bounded")
        self.queue, self.process, self.finalize = queue, process, finalize
        self.state = "NEW"
        self.error = None
        self.accepted = self.completed = self.abandoned = self.failed = self.dropped = 0
        self.metadata_thread = None
        self.lock = threading.RLock()
        self.submit_lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, name=name, daemon=True)

    def check(self):
        if self.state == "FAILED":
            raise WorkerFailed(f"worker failed: {self.error}") from self.error

    def start(self):
        with self.lock:
            self.check()
            if self.state in {"CLOSING", "CLOSED"}:
                raise RuntimeError("worker is closed")
            if self.state == "NEW":
                self.state = "RUNNING"
                self.thread.start()

    def submit(self, item, policy="block", on_drop=None, timeout=None):
        deadline = None if timeout is None else time.monotonic() + max(0., timeout)
        self.start()
        # Serialise producers, but never hold the state lock in a blocking put.
        with self.submit_lock:
            while True:
                with self.lock:
                    self.check()
                    if self.state != "RUNNING":
                        raise RuntimeError("worker is closed")
                    try:
                        self.queue.put_nowait(item)
                        self.accepted += 1
                        return True
                    except Full:
                        if policy == "drop_newest":
                            self.dropped += 1
                            if on_drop: on_drop()
                            return False
                        if policy == "drop_oldest":
                            try:
                                self.queue.get_nowait()
                            except Empty:
                                continue
                            self.queue.task_done()
                            self.dropped += 1
                            if on_drop: on_drop()
                            continue
                if deadline is not None and time.monotonic() >= deadline:
                    raise TimeoutError("worker enqueue timed out")
                time.sleep(.002)

    def _abandon(self):
        while True:
            try: self.queue.get_nowait()
            except Empty: return
            self.abandoned += 1
            self.queue.task_done()

    def _run(self):
        try:
            while True:
                with self.lock:
                    if self.state == "FAILED":
                        self._abandon()
                        break
                    if self.state == "CLOSING" and self.queue.empty():
                        break
                try: item = self.queue.get(timeout=.02)
                except Empty: continue
                try:
                    self.process(item)
                    self.completed += 1
                except BaseException as exc:
                    with self.lock:
                        self.error, self.state = exc, "FAILED"
                        self.failed += 1
                        self._abandon()
                    break
                finally:
                    self.queue.task_done()
        finally:
            try:
                if self.finalize: self.finalize()
            except BaseException as exc:
                with self.lock:
                    self.error, self.state = exc, "FAILED"
            with self.lock:
                if self.state != "FAILED": self.state = "CLOSED"

    def flush(self, timeout=10.0):
        deadline = time.monotonic() + max(0., timeout)
        with self.queue.all_tasks_done:
            while self.queue.unfinished_tasks:
                self.check()
                remaining = deadline - time.monotonic()
                if remaining <= 0: raise TimeoutError("worker flush timed out; output must be consumed concurrently")
                self.queue.all_tasks_done.wait(min(.02, remaining))
        self.check()

    def close(self, timeout=10.0):
        deadline = time.monotonic() + max(0., timeout)
        with self.lock:
            if self.state == "NEW":
                # Start the worker so finalization is included in the same deadline.
                self.state = "CLOSING"
                self.thread.start()
            elif self.state == "RUNNING": self.state = "CLOSING"
        self.thread.join(max(0., deadline - time.monotonic()))
        if self.thread.is_alive():
            with self.lock:
                self.error = TimeoutError("worker close timed out; worker is still alive, output incomplete")
                self.state = "FAILED"
            raise self.error
        self.check()

    def finalize_io(self, operation, deadline, on_failure=None):
        """Include closing metadata I/O in the caller's one deadline.

        A timed-out I/O thread is never killed. It repairs incomplete status
        after I/O becomes available; meanwhile state/error/counts stay visible
        in memory and existing incomplete manifests remain authoritative.
        """
        errors, results = [], []
        expired = threading.Event()
        def failed(exc):
            with self.lock:
                self.error, self.state = exc, "FAILED"
            if on_failure: on_failure(str(exc))
        def job():
            try:
                results.append(operation())
            except BaseException as exc:
                errors.append(exc)
                failed(exc)
            finally:
                if expired.is_set() or errors:
                    # Recompute status from FAILED state, never replay a cached
                    # complete=True payload after a delayed metadata operation.
                    try: operation()
                    except BaseException: pass
        if self.metadata_thread is not None and self.metadata_thread.is_alive():
            raise TimeoutError("closing metadata worker still alive; output incomplete")
        self.metadata_thread = threading.Thread(target=job, name=self.thread.name + "-metadata", daemon=True)
        self.metadata_thread.start()
        self.metadata_thread.join(max(0., deadline-time.monotonic()))
        if self.metadata_thread.is_alive():
            exc = TimeoutError("closing metadata I/O timed out; metadata worker still alive, output incomplete")
            expired.set()
            failed(exc)
            exc.worker_status = self.status
            raise exc
        if errors: raise errors[0]
        return results[0] if results else None

    @property
    def status(self):
        return {"state": self.state, "accepted": self.accepted, "completed": self.completed,
                "failed": self.failed, "dropped": self.dropped, "abandoned": self.abandoned,
                "unfinished": self.queue.unfinished_tasks, "worker_alive": self.thread.is_alive(),
                "metadata_worker_alive": bool(self.metadata_thread and self.metadata_thread.is_alive()),
                "incomplete": self.state != "CLOSED" or bool(self.failed or self.abandoned),
                "error": str(self.error) if self.error else None}

    def get_result(self, output, timeout=None):
        deadline = None if timeout is None else time.monotonic() + max(0., timeout)
        while True:
            self.check()
            remaining = .02 if deadline is None else min(.02, max(0., deadline-time.monotonic()))
            try:
                result = output.get(timeout=remaining)
                output.task_done()
                return result
            except Empty:
                self.check()
                if deadline is not None and time.monotonic() >= deadline: raise
                if self.state == "CLOSED": raise

    def publish(self, output, result, latest=False):
        if latest:
            self.check()
            try:
                output.put_nowait(result)
                return 0
            except Full:
                try:
                    output.get_nowait()
                    output.task_done()
                except Empty:
                    output.put_nowait(result)
                    return 0
                output.put_nowait(result)
                return 1
        while True:
            self.check()
            try:
                output.put(result, timeout=.02)
                return 0
            except Full:
                continue
