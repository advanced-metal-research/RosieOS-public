"""The camera's low-data H.264 stream, for the WebRTC sender (cell-camera/v1/video, `cell-video`).

Why: the JPEG stream is about 235 KB a frame; one browser viewer used about
25 GB an hour and a Deck about 12.7 GB (a Rosie 1430 arm, 2026-10-16), and a conference
hotspot ran out of data while the cell was teleoperated. H.264 at 960x720 and
15 fps capped at 800 kbit/s is at most 0.36 GB an hour, and much less for a
mostly still cell.

How: with video configured, the capture pipeline (capture.py) splits after
the camera. One branch is the JPEG stream as before. The other drops to the
video frame rate, scales, and encodes once with x264; a valve keeps the
encoder idle while nothing reads the stream. This service only encodes: the
`cell-video` sender reads the stream on loopback and sends it to each viewer
over WebRTC (UDP over Tailscale), outside the SSH tunnel that carries jog
replies. WebRTC itself is not done here because GStreamer 1.20's Python
binding crashed reading webrtcbin's replies (2026-10-19, Ubuntu 22.04).

  GET  <prefix>/video.h264       the stream while the connection stays open: per access unit,
                                 a 4-byte big-endian length then the H.264 Annex-B bytes, the
                                 first one a key frame; 200 at once, and the stream ends with
                                 no unit when the camera gives none in FIRST_UNIT_SECONDS;
                                 X-Video-FPS is the rate the units are encoded at
  POST <prefix>/video/keyframe   asks the encoder for a key frame (a viewer lost packets or joined)

Measured on an Orin Nano Super relay (no hardware encoder), 2026-10-19,
x264 superfast zerolatency 960x720 15 fps capped at 800 kbit/s: 0.15 of a
core and 75 kbit/s for a simple moving scene, 0.33 of a core at the cap for
pure noise.
"""
from __future__ import annotations

import struct
import threading
import time
from typing import Callable, Optional

from .config import Config

# A stream reader that falls this many access units behind is dropped (it
# reconnects and starts again at a key frame) rather than buffering video.
MAX_QUEUED_UNITS = 30
# The sender waits this long for the first unit after connecting; capture
# start took 3.3 s on the first camera host.
FIRST_UNIT_SECONDS = 8.0


def video_branch(config: Config, nvidia: bool) -> str:
    """The pipeline fragment from the capture's tee `split` to the appsink `video_units`.

    The tee carries the camera's raw frames in system memory. Every element is
    named `video_*`, so the JPEG stage timings never probe it.
    """
    video = config.video
    assert video is not None
    gate = ("split. ! queue name=video_branch max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! "
            # drop-mode=1 forwards the stream's sticky events (caps, segment)
            # while the gate is closed, and the sink below does not wait for a
            # first buffer: with the default drop-all and an async sink, the
            # closed gate stalled the whole capture, JPEG frames included
            # (an Orin Nano Super relay, 2026-10-19: 60 frames in 3.5 s gate
            # open, none in 15 s closed; 3.5-3.7 s closed with either change).
            "valve name=video_gate drop=true drop-mode=1 ! "
            f"videorate name=video_rate drop-only=true ! video/x-raw,framerate={video.fps}/1 ! ")
    if nvidia:
        # nvvidconv crops, scales and converts on the VIC, from and to system memory.
        scale = (f"nvvidconv name=video_convert compute-hw=2 {video_crop(config, True)}! "
                 f"video/x-raw,format=I420,width={video.width},height={video.height} ! ")
    else:
        scale = (f"{video_crop(config, False)}videoconvert name=video_convert ! videoscale name=video_scale ! "
                 f"video/x-raw,format=I420,width={video.width},height={video.height} ! ")
    encode = (f"x264enc name=video_encoder speed-preset=superfast tune=zerolatency bitrate={video.kbps} "
              f"key-int-max={video.fps * 2} threads=2 ! "
              "video/x-h264,profile=constrained-baseline,stream-format=byte-stream,alignment=au ! "
              "h264parse name=video_parse config-interval=-1 ! "
              "appsink name=video_units sync=false async=false emit-signals=false max-buffers=4 drop=false")
    return gate + scale + encode


def video_crop(config: Config, nvidia: bool) -> str:
    """The crop part of video_branch for this camera (the same rectangle the JPEG keeps)."""
    if not config.crop:
        return ""
    crop_width, crop_height, left, top = config.crop
    if nvidia:
        return f"left={left} right={left + crop_width} top={top} bottom={top + crop_height} "
    width, height, _ = config.capture
    return (f"videocrop name=video_crop left={left} right={width - left - crop_width} "
            f"top={top} bottom={height - top - crop_height} ! ")


def frame_unit(unit: bytes) -> bytes:
    """One access unit as the stream carries it: 4-byte big-endian length, then the bytes."""
    return struct.pack(">I", len(unit)) + unit


def is_key_unit(unit: bytes) -> bool:
    """Whether an Annex-B access unit holds an IDR slice (NAL type 5)."""
    i = 0
    while True:
        i = unit.find(b"\x00\x00\x01", i)
        if i < 0 or i + 3 >= len(unit):
            return False
        if unit[i + 3] & 0x1F == 5:
            return True
        i += 3


class _Reader:
    def __init__(self) -> None:
        self.units: list = []
        self.cv = threading.Condition()
        self.started = False  # sends only from a key frame on
        self.closed = False


class VideoStream:
    """The encoded stream's readers. The capture producer attaches its pipeline while it runs.

    Readers get every access unit from the next key frame on. The encoder's
    valve is open while any reader is connected, and capture is kept running
    through `keep_capturing` (FrameHub.touch).
    """

    def __init__(self, config: Config, keep_capturing: Callable[[], None], counter: Callable[[str, int, int], None]) -> None:
        assert config.video is not None
        self.config, self.video = config, config.video
        self._keep_capturing = keep_capturing
        self._count = counter
        self._lock = threading.Lock()
        self._line = None
        self._gst = None
        self._readers: list = []
        self._units = 0
        self._ticker = threading.Thread(target=self._tick, name="cell-camera-video", daemon=True)
        self._ticker.start()

    # -- the producer's side ------------------------------------------------

    def attach(self, line, Gst) -> None:
        """Called by the producer once its pipeline plays."""
        with self._lock:
            self._line, self._gst = line, Gst
            self._gate()
        threading.Thread(target=self._pull, args=(line, Gst), name="cell-camera-video-pull", daemon=True).start()

    def detach(self) -> None:
        """Called by the producer before its pipeline stops: every reader's stream ends."""
        with self._lock:
            self._line = None
            readers = list(self._readers)
        for reader in readers:
            with reader.cv:
                reader.closed = True
                reader.cv.notify_all()

    def _gate(self) -> None:
        """Opens the encoder's valve while readers exist (caller holds the lock)."""
        if self._line is not None:
            gate = self._line.get_by_name("video_gate")
            if gate is not None:
                gate.set_property("drop", not self._readers)

    def _pull(self, line, Gst) -> None:
        sink = line.get_by_name("video_units")
        while True:
            with self._lock:
                if self._line is not line:
                    return
            sample = sink.emit("try-pull-sample", int(0.2 * Gst.SECOND))
            if sample is None:
                continue
            buffer = sample.get_buffer()
            unit = buffer.extract_dup(0, buffer.get_size())
            key = is_key_unit(unit)
            with self._lock:
                self._units += 1
                readers = list(self._readers)
            for reader in readers:
                with reader.cv:
                    if not reader.started and not key:
                        continue
                    reader.started = True
                    if len(reader.units) >= MAX_QUEUED_UNITS:
                        reader.closed = True  # too far behind: it reconnects at a key frame
                    else:
                        reader.units.append(unit)
                    reader.cv.notify_all()

    # -- the HTTP side ------------------------------------------------------

    def keyframe(self) -> bool:
        """Asks the encoder for a key frame; False while no pipeline runs."""
        with self._lock:
            line, Gst = self._line, self._gst
        if line is None:
            return False
        import gi
        gi.require_version("GstVideo", "1.0")
        from gi.repository import GstVideo
        encoder = line.get_by_name("video_encoder")
        return encoder.get_static_pad("src").send_event(
            GstVideo.video_event_new_upstream_force_key_unit(Gst.CLOCK_TIME_NONE, True, 0))

    def stream(self, write: Callable[[bytes], None]) -> None:
        """Writes framed access units to `write` until the reader falls behind, capture stops, or write fails."""
        reader = _Reader()
        self._keep_capturing()
        deadline = time.monotonic() + FIRST_UNIT_SECONDS
        with self._lock:
            self._readers.append(reader)
            self._gate()
        self.keyframe()
        try:
            while True:
                with reader.cv:
                    reader.cv.wait_for(lambda: reader.units or reader.closed, timeout=1.0)
                    units, reader.units = reader.units, []
                    closed = reader.closed
                if units:
                    data = b"".join(frame_unit(u) for u in units)
                    write(data)
                    self._count("video", len(data), 0)
                elif not reader.started and time.monotonic() > deadline:
                    return
                elif not reader.started:
                    self.keyframe()  # the pipeline may have attached after the first ask
                if closed:
                    return
        finally:
            with self._lock:
                if reader in self._readers:
                    self._readers.remove(reader)
                self._gate()
            self._count("video", 0, 1)

    def status(self) -> dict:
        with self._lock:
            return {"enabled": True, "width": self.video.width, "height": self.video.height, "fps": self.video.fps,
                    "kbps": self.video.kbps, "readers": len(self._readers), "units": self._units,
                    "encoding": self._line is not None and bool(self._readers)}

    def _tick(self) -> None:
        """Keeps capture running while the stream has readers."""
        while True:
            time.sleep(1.0)
            with self._lock:
                busy = bool(self._readers)
            if busy:
                self._keep_capturing()
