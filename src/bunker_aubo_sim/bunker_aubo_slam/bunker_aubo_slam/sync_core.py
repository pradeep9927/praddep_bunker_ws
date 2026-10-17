from collections import deque
from dataclasses import dataclass
from typing import Any, Deque, List, Optional, Tuple

BLANK = 'blank'


@dataclass
class Buffered:
    stamp: float
    msg: Any
    rx: float


class SyncCore:
    def __init__(self, n_cameras: int, max_dt: float = 0.05, camera_timeout: float = 0.5,
                 pending_timeout: float = 0.3, buffer_size: int = 15):
        self.n = n_cameras
        self.max_dt = max_dt
        self.camera_timeout = camera_timeout
        self.pending_timeout = pending_timeout
        self.buffers: List[Deque[Buffered]] = [deque(maxlen=buffer_size) for _ in range(n_cameras)]
        self.last_rx: List[Optional[float]] = [None] * n_cameras
        self.seen: List[bool] = [False] * n_cameras
        self.last_emitted = float('-inf')
        self.blank_count = [0] * n_cameras

    def alive(self, i: int, now: float) -> bool:
        return self.last_rx[i] is not None and now - self.last_rx[i] <= self.camera_timeout

    def ready(self) -> bool:
        return all(self.seen)

    def reference(self, now: float) -> Optional[int]:
        for i in range(self.n):
            if self.alive(i, now):
                return i
        return None

    def add(self, i: int, stamp: float, msg: Any, now: float) -> List[Tuple[float, list]]:
        self.buffers[i].append(Buffered(stamp, msg, now))
        self.last_rx[i] = now
        self.seen[i] = True
        return self.flush(now)

    def _closest(self, j: int, stamp: float) -> Optional[Buffered]:
        best = None
        for b in self.buffers[j]:
            if abs(b.stamp - stamp) <= self.max_dt and (best is None or abs(b.stamp - stamp) < abs(best.stamp - stamp)):
                best = b
        return best

    def flush(self, now: float) -> List[Tuple[float, list]]:
        out: List[Tuple[float, list]] = []
        if not self.ready():
            return out
        ref = self.reference(now)
        if ref is None:
            return out
        for cand in sorted(self.buffers[ref], key=lambda b: b.stamp):
            if cand.stamp <= self.last_emitted:
                continue
            group: list = [None] * self.n
            group[ref] = cand.msg
            complete = True
            for j in range(self.n):
                if j == ref:
                    continue
                match = self._closest(j, cand.stamp)
                if match is not None:
                    group[j] = match.msg
                    continue
                newest = max((b.stamp for b in self.buffers[j]), default=float('-inf'))
                may_still_arrive = self.alive(j, now) and newest < cand.stamp + self.max_dt
                if may_still_arrive and now - cand.rx < self.pending_timeout:
                    complete = False
                    break
                group[j] = BLANK
            if not complete:
                break  # wait, keep stamp order
            for j, g in enumerate(group):
                if g is BLANK:
                    self.blank_count[j] += 1
            self.last_emitted = cand.stamp
            out.append((cand.stamp, group))
        return out
