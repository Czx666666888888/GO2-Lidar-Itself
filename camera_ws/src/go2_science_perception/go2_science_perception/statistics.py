from dataclasses import dataclass
from typing import Optional


@dataclass
class StreamStats:
    """Small, ROS-independent accumulator used by the diagnostics node."""

    count: int = 0
    first_time: Optional[float] = None
    last_time: Optional[float] = None
    message_type: str = ""
    frame_id: str = ""
    width: Optional[int] = None
    height: Optional[int] = None
    encoding: str = ""

    def update(self, now: float) -> None:
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.count += 1

    @property
    def frequency_hz(self) -> float:
        if self.count < 2 or self.first_time is None or self.last_time is None:
            return 0.0
        elapsed = self.last_time - self.first_time
        return (self.count - 1) / elapsed if elapsed > 0.0 else 0.0

    def is_fresh(self, now: float, stale_after: float) -> bool:
        return self.last_time is not None and now - self.last_time <= stale_after
