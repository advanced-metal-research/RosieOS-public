#!/usr/bin/env python3
"""Profile the cell camera's latency from one vantage point.

Why this exists: "the camera feels slow" names no stage. A frame crosses the
capture pipeline, the camera service, the OLP relay, a tunnel and the client,
and only a per-stage number says which of them to change. This tool asks for
frames the way the pendant does (the newest frame after the last one, then at
most one request every --min-interval-ms) and records what each frame cost.

Run it at each hop and compare; the difference between two hops is the hop:
  on the cell host, at the camera service   http://127.0.0.1:8088/camera
  on the cell host, at the OLP relay        http://127.0.0.1:8794/api/offline-programming/v1/cell-camera
  on a client, through its tunnel           http://127.0.0.1:18794/api/offline-programming/v1/cell-camera

Output, in --out:
  <label>.trace.jsonl    one line per frame: client timings, sizes, the server's stage timings
  <label>.trace.json     the same frames as Chrome trace events (open in Perfetto or chrome://tracing)
  <label>.metrics.json   p50/p90/p99/max of every timing, frame rate, bit rate, skipped frames
  <label>.log            the run's settings, errors and the summary table

Standard library only, so it runs on a Jetson, a Steam Deck or a laptop as it is.
"""
from __future__ import annotations

import argparse
import http.client
import json
import socket
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional
from urllib.parse import urlsplit

# The pendant's bounds (steamdeck/real/v5/src/camera_view.hpp): the profile is
# only a picture of the pendant if it asks at the pendant's pace.
PENDANT_MIN_INTERVAL_MS = 66
REQUEST_TIMEOUT_SECONDS = 6.0
PING_EVERY_SECONDS = 1.0


def parse_server_timing(header: Optional[str]) -> Dict[str, float]:
    """`capture;dur=4.1, encode;dur=6` as {"capture": 4.1, "encode": 6.0}; unknown parts are skipped."""
    stages: Dict[str, float] = {}
    for part in (header or "").split(","):
        name, _, rest = part.strip().partition(";")
        for field in rest.split(";"):
            key, _, value = field.strip().partition("=")
            if name and key == "dur":
                try:
                    stages[name] = float(value)
                except ValueError:
                    pass
    return stages


def percentile(values: List[float], fraction: float) -> float:
    """Nearest-rank percentile of a non-empty list."""
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))]


def summarize(values: List[float]) -> Dict[str, float]:
    return {"n": len(values), "p50": round(percentile(values, .5), 2), "p90": round(percentile(values, .9), 2),
            "p99": round(percentile(values, .99), 2), "max": round(max(values), 2),
            "mean": round(sum(values) / len(values), 2)}


def derive(frame: dict, ping_ms: Optional[float]) -> None:
    """Add the numbers no single clock measures. Each states its assumption in the README table.

    network_ms       the request's time outside the servers: total - wait - relay overhead.
                     Needs the server's own `wait`; absent on a service without stage timings.
    age_at_receive   the frame's age when its last byte arrived: the server's age at send plus
                     the time the reply took to arrive, taken as the body download plus half a
                     round trip of a small request (the link is assumed symmetric).
    """
    stages = frame["server"]
    if "wait" in stages:
        relay_overhead = max(0.0, stages["relay"] - stages["wait"]) if "relay" in stages else 0.0
        frame["relay_overhead_ms"] = round(relay_overhead, 2)
        frame["network_ms"] = round(max(0.0, frame["total_ms"] - stages["wait"] - relay_overhead), 2)
    if frame.get("source_age_ms") is not None and ping_ms is not None:
        frame["age_at_receive_ms"] = round(frame["source_age_ms"] + frame["download_ms"] + ping_ms / 2, 2)


class Link:
    """One keep-alive connection, as the pendant's network stack keeps."""

    def __init__(self, url: str) -> None:
        parts = urlsplit(url)
        if parts.scheme != "http" or not parts.hostname:
            raise SystemExit(f"--url must be http://host[:port]/path, got {url!r}")
        self.host, self.port, self.base = parts.hostname, parts.port or 80, parts.path.rstrip("/")
        self.connection: Optional[http.client.HTTPConnection] = None

    def get(self, path: str):
        """Returns (status, headers, body, ms to headers, ms to last byte); reconnects once."""
        for attempt in (0, 1):
            try:
                if self.connection is None:
                    self.connection = http.client.HTTPConnection(self.host, self.port, timeout=REQUEST_TIMEOUT_SECONDS)
                    self.connection.connect()
                    self.connection.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                start = time.perf_counter()
                self.connection.request("GET", self.base + path, headers={"User-Agent": "camera-latency-profiler"})
                response = self.connection.getresponse()
                headers_at = time.perf_counter()
                body = response.read()
                done = time.perf_counter()
                return response.status, response.headers, body, (headers_at - start) * 1e3, (done - start) * 1e3
            except (OSError, http.client.HTTPException):
                self.close()
                if attempt:
                    raise
        raise AssertionError("unreachable")

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None


def run(args: argparse.Namespace) -> int:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    log_lines: List[str] = []

    def log(text: str) -> None:
        line = f"{time.strftime('%Y-%m-%dT%H:%M:%S%z')} {text}"
        log_lines.append(line)
        print(line, flush=True)

    frames_link, ping_link = Link(args.url), Link(args.url)
    if args.ping_path is None:
        # The camera service has <prefix>/status; the OLP relay answers its status at the base itself.
        args.ping_path = "" if args.url.rstrip("/").endswith("/cell-camera") else "/status"
    log(f"profile label={args.label} url={args.url} seconds={args.seconds} min_interval_ms={args.min_interval_ms} "
        f"host={socket.gethostname()}")
    try:
        status, _, body, _, _ = ping_link.get(args.ping_path)
        log(f"status {status}: {body[:600].decode('utf-8', 'replace')}")
    except (OSError, http.client.HTTPException) as error:
        log(f"status request failed: {error}")

    frames: List[dict] = []
    pings: List[float] = []
    errors = 0
    sequence, ping_ms, last_ping = 0, None, 0.0
    began = time.perf_counter()
    wall_began = time.time()
    while time.perf_counter() - began < args.seconds:
        now = time.perf_counter()
        if now - last_ping >= PING_EVERY_SECONDS:
            # A small request on its own connection: the link's round trip without a frame's wait or size.
            try:
                _, _, _, _, ping_ms = ping_link.get(args.ping_path)
                pings.append(ping_ms)
            except (OSError, http.client.HTTPException):
                ping_ms = None
            last_ping = now
        asked = time.perf_counter()
        try:
            status, headers, body, ttfb_ms, total_ms = frames_link.get(f"/frame.jpg?after={sequence}")
        except (OSError, http.client.HTTPException) as error:
            errors += 1
            log(f"frame request failed: {error}")
            time.sleep(1)
            continue
        if status != 200 or not body.startswith(b"\xff\xd8"):
            errors += 1
            log(f"frame refused: HTTP {status} {body[:200].decode('utf-8', 'replace')}")
            time.sleep(1)
            continue
        got = int(headers.get("X-Frame-Sequence") or sequence + 1)
        age = headers.get("X-Source-Age-Ms")
        frame = {
            "t_ms": round((asked - began) * 1e3, 2), "sequence": got,
            "skipped": max(0, got - sequence - 1) if sequence else 0,
            "bytes": len(body), "ttfb_ms": round(ttfb_ms, 2), "download_ms": round(total_ms - ttfb_ms, 2),
            "total_ms": round(total_ms, 2), "source_age_ms": float(age) if age else None,
            "ping_ms": round(ping_ms, 2) if ping_ms is not None else None,
            "server": parse_server_timing(headers.get("Server-Timing")),
        }
        if frames:
            frame["interval_ms"] = round(frame["t_ms"] + frame["total_ms"] - frames[-1]["t_ms"] - frames[-1]["total_ms"], 2)
        derive(frame, ping_ms)
        frames.append(frame)
        sequence = got
        pause = args.min_interval_ms / 1e3 - (time.perf_counter() - asked)
        if pause > 0:
            time.sleep(pause)
    elapsed = time.perf_counter() - began
    frames_link.close()
    ping_link.close()

    if not frames:
        log(f"no frames in {elapsed:.1f} s ({errors} errors)")
        (out / f"{args.label}.log").write_text("\n".join(log_lines) + "\n")
        return 1

    series: Dict[str, List[float]] = {}
    for frame in frames:
        for key in ("ttfb_ms", "download_ms", "total_ms", "interval_ms", "source_age_ms", "age_at_receive_ms",
                    "network_ms", "relay_overhead_ms", "bytes"):
            if frame.get(key) is not None:
                series.setdefault(key, []).append(frame[key])
        for name, value in frame["server"].items():
            series.setdefault(f"server.{name}_ms", []).append(value)
    if pings:
        series["ping_ms"] = pings
    total_bytes = sum(frame["bytes"] for frame in frames)
    metrics = {
        "label": args.label, "url": args.url, "host": socket.gethostname(),
        "started": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(wall_began)),
        "seconds": round(elapsed, 2), "frames": len(frames), "errors": errors,
        "fps": round(len(frames) / elapsed, 2), "megabits_per_second": round(total_bytes * 8 / elapsed / 1e6, 2),
        "source_frames_skipped": sum(frame["skipped"] for frame in frames),
        "has_stage_timings": any(frame["server"] for frame in frames),
        "timings": {key: summarize(values) for key, values in sorted(series.items())},
    }

    (out / f"{args.label}.trace.jsonl").write_text("".join(json.dumps(frame) + "\n" for frame in frames))
    (out / f"{args.label}.metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    (out / f"{args.label}.trace.json").write_text(json.dumps(chrome_trace(frames, args.label)) + "\n")

    log(f"frames={metrics['frames']} fps={metrics['fps']} mbit/s={metrics['megabits_per_second']} "
        f"errors={errors} source_frames_skipped={metrics['source_frames_skipped']} "
        f"stage_timings={'yes' if metrics['has_stage_timings'] else 'no (service predates them)'}")
    log(f"{'timing':24s} {'n':>5s} {'p50':>9s} {'p90':>9s} {'p99':>9s} {'max':>9s}")
    for key, row in metrics["timings"].items():
        log(f"{key:24s} {row['n']:5d} {row['p50']:9.2f} {row['p90']:9.2f} {row['p99']:9.2f} {row['max']:9.2f}")
    (out / f"{args.label}.log").write_text("\n".join(log_lines) + "\n")
    return 0


def pendant(args: argparse.Namespace) -> int:
    """Summarize the pendant's own trace (ROSIE_PENDANT_CAMERA_TRACE): the only vantage point with decode and paint.

    age_at_paint_ms is the frame's age when the pendant painted it:
      the service's age at send                           source_age_ms
      + the relay's overhead on the cell host             relay - wait
      + the reply's headers crossing the link             (headers_ms - relay) / 2, the link assumed symmetric
      + the body's download                               reply_ms - headers_ms
      + decode_ms + paint_ms
    """
    frames = []
    for line in Path(args.pendant_trace).read_text().splitlines():
        try:
            frame = json.loads(line)
        except ValueError:
            continue
        stages = parse_server_timing(frame.pop("server_timing", ""))
        frame["server"] = stages
        frame["download_ms"] = round(frame["reply_ms"] - max(0.0, frame["headers_ms"]), 2)
        if "relay" in stages and "wait" in stages and frame.get("source_age_ms") is not None and frame["headers_ms"] >= 0:
            frame["relay_overhead_ms"] = round(max(0.0, stages["relay"] - stages["wait"]), 2)
            frame["network_ms"] = round(max(0.0, frame["reply_ms"] - stages["relay"]), 2)
            frame["age_at_paint_ms"] = round(
                frame["source_age_ms"] + frame["relay_overhead_ms"] + max(0.0, frame["headers_ms"] - stages["relay"]) / 2
                + frame["download_ms"] + frame["decode_ms"] + frame["paint_ms"], 2)
        frames.append(frame)
    if not frames:
        print(f"no frames in {args.pendant_trace}", file=sys.stderr)
        return 1
    series: Dict[str, List[float]] = {}
    for frame in frames:
        for key in ("headers_ms", "reply_ms", "download_ms", "decode_ms", "paint_ms", "since_last_paint_ms", "source_age_ms",
                    "relay_overhead_ms", "network_ms", "age_at_paint_ms", "bytes"):
            if frame.get(key) is not None and frame[key] >= 0:
                series.setdefault(key, []).append(frame[key])
        for name, value in frame["server"].items():
            series.setdefault(f"server.{name}_ms", []).append(value)
    seconds = (frames[-1]["t_ms"] - frames[0]["t_ms"]) / 1e3
    metrics = {"label": args.label, "source": str(args.pendant_trace), "frames": len(frames), "seconds": round(seconds, 2),
               "fps": round((len(frames) - 1) / seconds, 2) if seconds > 0 else None,
               "frame": frames[-1].get("frame"), "drawn": frames[-1].get("drawn"),
               "timings": {key: summarize(values) for key, values in sorted(series.items())}}
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{args.label}.trace.jsonl").write_text("".join(json.dumps(frame) + "\n" for frame in frames))
    (out / f"{args.label}.metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    lines = [f"pendant trace {args.pendant_trace}: frames={len(frames)} seconds={metrics['seconds']} fps={metrics['fps']} "
             f"frame={metrics['frame']} drawn={metrics['drawn']}",
             f"{'timing':24s} {'n':>5s} {'p50':>9s} {'p90':>9s} {'p99':>9s} {'max':>9s}"]
    lines += [f"{key:24s} {row['n']:5d} {row['p50']:9.2f} {row['p90']:9.2f} {row['p99']:9.2f} {row['max']:9.2f}"
              for key, row in metrics["timings"].items()]
    (out / f"{args.label}.log").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


def chrome_trace(frames: List[dict], label: str) -> dict:
    """Each frame as nested spans on one track, with the server's stages laid end to end before the reply."""
    events = [{"name": "process_name", "ph": "M", "pid": 1, "tid": 1, "args": {"name": f"camera latency: {label}"}},
              {"name": "thread_name", "ph": "M", "pid": 1, "tid": 1, "args": {"name": "client request"}},
              {"name": "thread_name", "ph": "M", "pid": 1, "tid": 2, "args": {"name": "frame history (server stages)"}}]
    for frame in frames:
        start = frame["t_ms"] * 1e3
        common = {"pid": 1, "cat": "camera", "args": {key: value for key, value in frame.items() if key != "server"}}
        events.append({**common, "name": f"frame {frame['sequence']}", "ph": "X", "tid": 1, "ts": start, "dur": frame["total_ms"] * 1e3})
        events.append({**common, "name": "wait for headers", "ph": "X", "tid": 1, "ts": start, "dur": frame["ttfb_ms"] * 1e3})
        events.append({**common, "name": "download", "ph": "X", "tid": 1, "ts": start + frame["ttfb_ms"] * 1e3, "dur": frame["download_ms"] * 1e3})
        # The frame's life before the reply left, back from the moment its headers arrived.
        pipeline = [(name, frame["server"][name]) for name in ("capture", "queue", "convert", "encode", "sink", "held") if name in frame["server"]]
        cursor = start + frame["ttfb_ms"] * 1e3 - sum(value for _, value in pipeline) * 1e3
        for name, value in pipeline:
            events.append({"name": name, "ph": "X", "pid": 1, "tid": 2, "cat": "camera", "ts": cursor, "dur": value * 1e3,
                           "args": {"sequence": frame["sequence"]}})
            cursor += value * 1e3
    return {"traceEvents": events, "displayTimeUnit": "ms"}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--url", help="the camera's base URL: .../camera or .../v1/cell-camera")
    parser.add_argument("--pendant-trace", help="summarize a pendant's ROSIE_PENDANT_CAMERA_TRACE file instead of asking for frames")
    parser.add_argument("--label", required=True, help="names the output files, for example relay-service")
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--min-interval-ms", type=float, default=PENDANT_MIN_INTERVAL_MS,
                        help="least time between requests; the pendant uses 66. 0 asks as fast as frames come")
    parser.add_argument("--ping-path", default=None,
                        help="a small document under --url, for the link's round trip (default: the camera's status)")
    parser.add_argument("--out", default="camera-latency")
    args = parser.parse_args()
    if args.pendant_trace:
        return pendant(args)
    if not args.url:
        parser.error("--url or --pendant-trace is required")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
