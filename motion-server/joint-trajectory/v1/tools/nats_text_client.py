"""A minimal NATS client over the text protocol, for the tools beside the daemon.

Enough for a publisher and a subscriber that answers requests: CONNECT is
implicit (the server's INFO is ignored), PING is answered, subjects are
matched exactly. No reconnect, no queue groups, no JetStream: the bench and
the sim cell keeper run beside the broker they talk to, and a dropped
connection is a reason to restart them, not to hide.
"""

from __future__ import annotations

import socket
import threading
from typing import Callable

Callback = Callable[[str, str, bytes], None]  # (subject, reply_to, payload)


class NatsClient:
    """One connection; a callback per subscribed subject."""

    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(None)
        self.callbacks: dict[str, Callback] = {}
        self.sid = 0
        self.lock = threading.Lock()
        self.closed = threading.Event()
        threading.Thread(target=self._reader, daemon=True).start()

    def subscribe(self, subject: str, callback: Callback) -> None:
        self.sid += 1
        self.callbacks[subject] = callback
        with self.lock:
            self.sock.sendall(f"SUB {subject} {self.sid}\r\n".encode())

    def publish(self, subject: str, payload: bytes, reply_to: str = "") -> None:
        head = f"PUB {subject}" + (f" {reply_to}" if reply_to else "")
        with self.lock:
            self.sock.sendall(head.encode() + b" " + str(len(payload)).encode()
                              + b"\r\n" + payload + b"\r\n")

    def close(self) -> None:
        self.closed.set()
        try:
            self.sock.close()
        except OSError:
            pass

    def _reader(self) -> None:
        buffer = b""
        try:
            while True:
                chunk = self.sock.recv(65536)
                if not chunk:
                    break
                buffer += chunk
                while b"\r\n" in buffer:
                    line, rest = buffer.split(b"\r\n", 1)
                    text = line.decode("utf-8", "replace")
                    if text.startswith("MSG "):
                        # MSG <subject> <sid> [reply-to] <#bytes>
                        parts = text.split(" ")
                        subject = parts[1]
                        reply_to = parts[3] if len(parts) == 5 else ""
                        size = int(parts[-1])
                        while len(rest) < size + 2:
                            more = self.sock.recv(65536)
                            if not more:
                                return
                            rest += more
                        payload, rest = rest[:size], rest[size + 2:]
                        callback = self.callbacks.get(subject)
                        if callback:
                            callback(subject, reply_to, payload)
                    elif text == "PING":
                        with self.lock:
                            self.sock.sendall(b"PONG\r\n")
                    buffer = rest
        except OSError:
            pass
        finally:
            self.closed.set()
