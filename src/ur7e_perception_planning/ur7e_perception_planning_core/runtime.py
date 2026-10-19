"""One GPU owner at a time in combined mode; ROS I/O and CPU guard stay live."""
from contextlib import contextmanager
from threading import Condition, get_ident


class MappingPriorityLock:
    """Mapping goes before queued optimizations; active GPU work is not preempted."""
    def __init__(self):
        self.condition = Condition()
        self.owner, self.depth, self.mapping_waiters = None, 0, 0

    def acquire(self, mapping=False):
        with self.condition:
            if self.owner == get_ident():
                self.depth += 1
                return
            if mapping:
                self.mapping_waiters += 1
            try:
                while self.owner is not None or (not mapping and self.mapping_waiters):
                    self.condition.wait()
                self.owner, self.depth = get_ident(), 1
            finally:
                if mapping:
                    self.mapping_waiters -= 1

    def release(self):
        with self.condition:
            if self.owner != get_ident():
                raise RuntimeError('GPU lock released by non-owner')
            self.depth -= 1
            if self.depth == 0:
                self.owner = None
                self.condition.notify_all()

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, *exc):
        self.release()

    @contextmanager
    def mapping(self):
        self.acquire(mapping=True)
        try:
            yield
        finally:
            self.release()


GPU_LOCK = MappingPriorityLock()
