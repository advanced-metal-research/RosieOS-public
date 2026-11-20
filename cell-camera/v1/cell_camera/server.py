"""The camera's HTTP face: a status document, the newest frame, and a viewer page.

Contract (the OLP relay and the viewer page depend on it):
  GET <prefix>/status              JSON: camera, resolution, capture_fps, active, error, sequence, ...
  GET <prefix>/frame.jpg?after=N   the newest JPEG after sequence N, waiting up to 3 s, with
                                   X-Frame-Sequence and X-Source-Age-Ms; 503 while starting or failed.
                                   Server-Timing says where that age went, in ms: the pipeline's
                                   stages (capture, queue, convert, encode, sink), `held` (published
                                   until this reply) and `wait` (this request waiting for the frame)
  GET <prefix>/                    a view-only page
  GET <prefix>/depth.z16?after=N   with a RealSense's depth configured: the newest raw depth frame after
                                   sequence N, waiting up to 3 s: width x height unsigned 16-bit depth
                                   units, row by row, in the host's byte order (little-endian on arm64
                                   and x86_64), 0 where the camera has no depth.
                                   X-Frame-Sequence, X-Depth-Width, X-Depth-Height, X-Depth-Scale
                                   (metres per unit), X-Source-Age-Ms, X-Camera-Frame-Number and
                                   X-Camera-Timestamp-Ms; 503 while starting or failed, 404 without depth.
                                   Status `realsense.depth` has the intrinsics and the depth to colour
                                   extrinsics
  GET <prefix>/video.h264          with video configured: the H.264 stream for the WebRTC sender (video.py)
  POST <prefix>/video/keyframe     asks the video encoder for a key frame
Read-only: nothing here changes the camera or reaches anything else.

Status carries `served`: replies and bytes (body plus headers) sent since the
service started, per kind of reply, so a data budget can be checked against
what the camera actually sent (`profile`, steamdeck/real/v4). Clients reach
the camera through the OLP relay, which counts them per client.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

from .config import Config
from .hub import FrameHub

PAGE = Path(__file__).with_name("index.html")
# Unverified bound from the first service: eight requests at once is several
# viewers; more are dropped rather than queued behind slow clients.
MAX_REQUESTS = 8
SOCKET_TIMEOUT_SECONDS = 5
_CSP = ("default-src 'self'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; "
        "img-src 'self' blob:; connect-src 'self'; frame-ancestors 'self'")


def server_timing(stages) -> str:
    """((name, seconds), ...) as a Server-Timing header value, in milliseconds."""
    return ", ".join(f"{name};dur={seconds * 1000:.2f}" for name, seconds in stages)


class Server(ThreadingHTTPServer):
    """Serves a bounded number of requests at once; a request beyond that is closed, not queued."""

    daemon_threads = True
    request_queue_size = MAX_REQUESTS

    def __init__(self, config: Config, hub: FrameHub, facts: dict, video=None, depth=None) -> None:
        self.config, self.hub, self.facts, self.video, self.depth = config, hub, facts, video, depth
        self.page = PAGE.read_bytes()
        self.slots = threading.BoundedSemaphore(MAX_REQUESTS)
        self.started = time.time()
        self._served_lock = threading.Lock()
        self._served: dict = {}
        super().__init__(config.listen, Handler)

    def count(self, kind: str, nbytes: int, replies: int = 1) -> None:
        """Adds `replies` replies and `nbytes` bytes to the `served` counters under `kind`.

        Video counts its bytes per RTP packet as they go (replies 0) and one
        reply per session when it ends.
        """
        with self._served_lock:
            entry = self._served.setdefault(kind, {"replies": 0, "bytes": 0})
            entry["replies"] += replies
            entry["bytes"] += nbytes

    def served(self) -> dict:
        with self._served_lock:
            return {"since": round(self.started, 3), **{kind: dict(entry) for kind, entry in self._served.items()}}

    def process_request(self, request, address):
        if not self.slots.acquire(False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request, address)
        finally:
            self.slots.release()

    def status(self) -> dict:
        config = self.config
        return {"camera": config.name, "title": config.title, "resolution": list(config.output),
                "capture_resolution": list(config.capture[:2]), "configured_fps": config.capture[2],
                **self.hub.status(), "encoding": self.facts.get("encoding", config.encoder),
                "jpeg_quality": None if config.pixel_format == "MJPG" else config.jpeg_quality,
                "dropped_invalid_frames": self.facts.get("dropped_invalid_frames", 0), "idle_stop_seconds": config.idle_stop_seconds,
                "served": self.served(),
                "video": self.video.status() if self.video is not None else {"enabled": False},
                # A RealSense's model, serial number, firmware and USB link, once a capture has opened it.
                **({"realsense": self.facts.get("realsense"),
                    "depth": self.depth.status() if self.depth is not None else {"enabled": False}}
                   if config.source == "realsense" else {})}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server: Server

    def log_message(self, *args):  # a request per frame per viewer is not a log
        pass

    def setup(self):
        super().setup()
        self.connection.settimeout(SOCKET_TIMEOUT_SECONDS)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def reply(self, status: int, data: bytes, kind: str, headers=None, counted_as: str = "other") -> None:
        self.send_response(status)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", _CSP)
        for key, value in (headers or {}).items():
            self.send_header(key, str(value))
        header_bytes = sum(len(line) for line in getattr(self, "_headers_buffer", []))
        self.end_headers()
        self.wfile.write(data)
        self.server.count(counted_as, header_bytes + len(data))

    def do_GET(self):
        try:
            url = urlsplit(self.path)
            prefix = self.server.config.path_prefix
            if prefix and not (url.path == prefix or url.path.startswith(prefix + "/")):
                return self.reply(404, b"Not found", "text/plain")
            path = url.path[len(prefix):]
            if path in ("", "/", "/index.html"):
                return self.reply(200, self.server.page, "text/html; charset=utf-8")
            if path == "/status":
                return self.reply(200, json.dumps(self.server.status()).encode(), "application/json")
            if path == "/frame.jpg":
                try:
                    after = max(0, int(parse_qs(url.query).get("after", ["0"])[0]))
                except ValueError:
                    return self.reply(400, b"Invalid frame cursor", "text/plain")
                asked = time.monotonic()
                frame = self.server.hub.frame(after)
                if frame is None:
                    return self.reply(503, b"Camera unavailable; reconnecting automatically", "text/plain")
                now = time.monotonic()
                return self.reply(200, frame.data, "image/jpeg", counted_as="jpeg", headers={
                    "X-Frame-Sequence": frame.sequence,
                    "X-Source-Age-Ms": round((now - frame.at + frame.source_age) * 1000, 2),
                    "Server-Timing": server_timing(frame.stages + (("held", now - frame.at), ("wait", now - asked)))})
            if path == "/depth.z16":
                return self._depth(url)
            if path == "/video.h264" and self.server.video is not None:
                return self._stream_video()
            return self.reply(404, b"Not found", "text/plain")
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass

    def _depth(self, url) -> None:
        """Answers the newest raw depth frame after ?after=N (see the contract above)."""
        if self.server.depth is None:
            return self.reply(404, b"This camera serves no depth", "text/plain")
        try:
            after = max(0, int(parse_qs(url.query).get("after", ["0"])[0]))
        except ValueError:
            return self.reply(400, b"Invalid frame cursor", "text/plain")
        frame = self.server.depth.frame(after)
        if frame is None:
            return self.reply(503, b"Depth unavailable; reconnecting automatically", "text/plain")
        now = time.monotonic()
        return self.reply(200, frame.data, "application/octet-stream", counted_as="depth", headers={
            "X-Frame-Sequence": frame.sequence, "X-Depth-Width": frame.width, "X-Depth-Height": frame.height,
            "X-Depth-Scale": repr(frame.scale),
            "X-Source-Age-Ms": round((now - frame.at + frame.source_age) * 1000, 2),
            "X-Camera-Frame-Number": frame.number, "X-Camera-Timestamp-Ms": round(frame.timestamp_ms, 3)})

    def _stream_video(self) -> None:
        """Sends the framed H.264 stream until the reader goes away or the stream ends (video.py)."""
        self.send_response(200)
        self.send_header("Content-Type", "application/x-rosie-h264-units")
        # The rate the camera encodes (config.py sets it from the capture
        # rate): the one source a reader times its samples from.
        self.send_header("X-Video-FPS", str(self.server.video.video.fps))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True
        # The socket timeout stays: a reader that stops taking bytes for
        # SOCKET_TIMEOUT_SECONDS fails its write and frees the encoder and
        # this request slot. Waiting for units is not a socket operation.

        def write(data: bytes) -> None:
            self.wfile.write(data)
            self.wfile.flush()
        self.server.video.stream(write)

    def do_POST(self):
        try:
            prefix = self.server.config.path_prefix
            if self.server.video is not None and urlsplit(self.path).path == prefix + "/video/keyframe":
                ok = self.server.video.keyframe()
                return self.reply(200 if ok else 503, b"Asked" if ok else b"No pipeline", "text/plain")
            return self.reply(404, b"Not found", "text/plain")
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            pass
