import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

OK, STALE, NO_DATA = 'OK', 'STALE', 'NO DATA'


@dataclass
class TopicStatus:
    topic: str
    state: str
    rate_hz: float
    age_s: Optional[float]
    expected_hz: float
    count: int


class TopicMonitor:
    def __init__(self, window_s: float = 3.0, stale_factor: float = 5.0, min_stale_s: float = 1.0,
                 clock=time.monotonic):
        self._lock = threading.Lock()
        self._rx: Dict[str, Deque[float]] = {}
        self._count: Dict[str, int] = {}
        self._expected: Dict[str, float] = {}
        self.window_s = window_s
        self.stale_factor = stale_factor
        self.min_stale_s = min_stale_s
        self._clock = clock

    def add(self, topic: str, expected_hz: float):
        with self._lock:
            self._rx.setdefault(topic, deque(maxlen=2000))
            self._count.setdefault(topic, 0)
            self._expected[topic] = expected_hz

    def record(self, topic: str):
        now = self._clock()
        with self._lock:
            if topic not in self._rx:
                self._rx[topic] = deque(maxlen=2000)
                self._count[topic] = 0
                self._expected.setdefault(topic, 1.0)
            self._rx[topic].append(now)
            self._count[topic] += 1

    def status(self, topic: str) -> TopicStatus:
        now = self._clock()
        with self._lock:
            rx = list(self._rx.get(topic, ()))
            count = self._count.get(topic, 0)
            expected = self._expected.get(topic, 1.0)
        if not rx:
            return TopicStatus(topic, NO_DATA, 0.0, None, expected, 0)
        age = now - rx[-1]
        recent = [t for t in rx if now - t <= self.window_s]
        rate = 0.0
        if len(recent) >= 2 and recent[-1] > recent[0]:
            rate = (len(recent) - 1) / (recent[-1] - recent[0])
        stale_after = max(self.stale_factor / max(expected, 1e-3), self.min_stale_s)
        state = OK if age <= stale_after else STALE
        return TopicStatus(topic, state, rate, age, expected, count)

    def snapshot(self) -> List[TopicStatus]:
        with self._lock:
            topics = list(self._rx.keys())
        return [self.status(t) for t in topics]
