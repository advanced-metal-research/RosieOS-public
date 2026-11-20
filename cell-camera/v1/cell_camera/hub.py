"""The newest frame, handed to whoever asks, with capture running only while someone does.

This is the service's behaviour with no camera in it: a producer publishes
frames, clients wait for one newer than the last they drew, and the producer
is told to stop once nobody has asked for a while. Keeping it free of
GStreamer is what lets it be tested without hardware.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, NamedTuple, Optional


class Frame(NamedTuple):
    sequence: int         # increases by one per published frame; a client's cursor
    at: float             # clock() when it was published
    data: bytes           # one complete JPEG
    source_age: float     # seconds the frame spent in the capture pipeline before that
    stages: tuple = ()    # ((name, seconds), ...): where source_age went, when the producer measured it


class FrameHub:
    """One camera's frames. `produce(hub)` captures until `hub.wanted()` is false."""

    def __init__(self, produce: Callable[["FrameHub"], None], idle_stop_seconds: float,
                 max_frame_age_seconds: float, clock: Callable[[], float] = time.monotonic) -> None:
        self._produce = produce
        self._idle_stop = idle_stop_seconds
        self._max_age = max_frame_age_seconds
        self._clock = clock
        self._cv = threading.Condition()
        self._thread: Optional[threading.Thread] = None
        self._last_request = 0.0
        self._latest: Optional[Frame] = None
        self._sequence = 0
        self._error: Optional[str] = None
        self._running = False
        self._fps = 0.0
        self._window_at = 0.0
        self._window_count = 0

    def touch(self) -> None:
        """Counts as a request: starts capture if it is stopped and keeps it running (video viewers)."""
        with self._cv:
            self._last_request = self._clock()
            if self._thread is None or not self._thread.is_alive():
                self._error = None
                self._thread = threading.Thread(target=self._run, name="cell-camera-capture", daemon=True)
                self._thread.start()

    def frame(self, after: int, timeout: float = 3.0) -> Optional[Frame]:
        """The newest frame after `after`, waiting up to `timeout`; None while the camera starts or fails."""
        with self._cv:
            self.touch()
            self._cv.wait_for(lambda: self._fresh(after) is not None or self._error is not None, timeout=timeout)
            return self._fresh(after)

    def _fresh(self, after: int) -> Optional[Frame]:
        latest = self._latest
        if latest and latest.sequence > after and self._clock() - latest.at < self._max_age:
            return latest
        return None

    def wanted(self) -> bool:
        """Whether a client asked for a frame recently enough to keep capturing."""
        with self._cv:
            return self._clock() - self._last_request <= self._idle_stop

    def publish(self, data: bytes, source_age: float = 0.0, stages: tuple = ()) -> None:
        """Called by the producer for each complete JPEG; older frames are dropped, never queued."""
        if source_age > self._max_age:
            return
        now = self._clock()
        with self._cv:
            self._sequence += 1
            self._latest = Frame(self._sequence, now, data, source_age, stages)
            self._window_count += 1
            if now - self._window_at >= 1:
                self._fps = self._window_count / (now - self._window_at)
                self._window_count, self._window_at = 0, now
            self._cv.notify_all()

    def _run(self) -> None:
        with self._cv:
            self._running, self._window_at, self._window_count = True, self._clock(), 0
        try:
            self._produce(self)
        except Exception as exc:  # the producer's failure is the camera's status, not a crash
            with self._cv:
                self._error = str(exc) or type(exc).__name__
        finally:
            with self._cv:
                self._running = False
                self._latest = None
                self._fps = 0.0
                self._cv.notify_all()

    def status(self) -> dict:
        with self._cv:
            return {"capture_fps": round(self._fps, 1), "active": self._running,
                    "error": self._error, "sequence": self._sequence}


class DepthFrame(NamedTuple):
    sequence: int         # increases by one per published depth frame; a client's cursor
    at: float             # clock() when it was published
    data: bytes           # width x height unsigned 16-bit depth units, row by row, in the host's byte order
    width: int
    height: int
    scale: float          # metres per depth unit (the camera's depth scale)
    source_age: float     # seconds since librealsense had the whole frame
    number: int           # librealsense's frame number
    timestamp_ms: float   # librealsense's timestamp for the frame, milliseconds


class DepthFrames:
    """A RealSense's newest raw depth frame, for whoever asks.

    Capture is the FrameHub's: the colour and the depth come from one
    librealsense session, which only one process may hold, so asking for depth
    `touch`es the hub and waits for the frame its producer publishes here.
    """

    def __init__(self, touch: Callable[[], None], max_frame_age_seconds: float,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._touch = touch
        self._max_age = max_frame_age_seconds
        self._clock = clock
        self._cv = threading.Condition()
        self._latest: Optional[DepthFrame] = None
        self._sequence = 0

    def publish(self, data: bytes, width: int, height: int, scale: float, source_age: float,
                number: int, timestamp_ms: float) -> None:
        """Called by the producer for each new depth frame; older frames are dropped, never queued."""
        if len(data) != width * height * 2:
            raise ValueError(f"depth frame is {len(data)} bytes, not {width}x{height} 16-bit units")
        if source_age > self._max_age:
            return
        with self._cv:
            self._sequence += 1
            self._latest = DepthFrame(self._sequence, self._clock(), data, width, height, scale,
                                      source_age, number, timestamp_ms)
            self._cv.notify_all()

    def frame(self, after: int, timeout: float = 3.0) -> Optional[DepthFrame]:
        """The newest depth frame after `after`, waiting up to `timeout`; None while the camera starts or fails."""
        self._touch()
        with self._cv:
            self._cv.wait_for(lambda: self._fresh(after) is not None, timeout=timeout)
            return self._fresh(after)

    def _fresh(self, after: int) -> Optional[DepthFrame]:
        latest = self._latest
        if latest and latest.sequence > after and self._clock() - latest.at < self._max_age:
            return latest
        return None

    def clear(self) -> None:
        """Forget the last frame when capture stops, so a stopped camera serves no stale depth."""
        with self._cv:
            self._latest = None

    def status(self) -> dict:
        with self._cv:
            return {"enabled": True, "sequence": self._sequence}
