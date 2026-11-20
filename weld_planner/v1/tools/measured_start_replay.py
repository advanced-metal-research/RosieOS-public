#!/usr/bin/env python
"""Plan program mixes from a measured start on the production motion server.

The composed proof for "every non-I/O node plans from the robot's measured
pose" (2026-10-11): each case is packed with the seam worker's own packer
(`write_request`, cell from `descriptor_of`) beside a `start_state.json` as
the OLP captures it, posted to `/api/motion/plan` with the OLP's query (seam
search, weld and connecting trajopt, the continuous verifier, dense export),
and judged on the dense trajectory in absolute units:

- the first dense sample equals the measured start to 1e-10 rad;
- every sample of every axis is inside the description's position limits
  (a Rosie 1430 arm's J5 upper 0.5934 rad, 1a4f24ac), and a case that asks
  for a waypoint past a limit is refused rather than clipped;
- the segments come back in authored order, every connecting leg and weld
  natively PASS (the dense export refuses anything else);
- each weld's samples match the same program planned from the model reset
  pose, modulo whole turns: the welds keep their verified trajectories and
  only the free-space legs move.

The robot is either a model compiled here by the OLP's `compile-context`
(`--context-dir`, for example a Rosie 1430 arm's) or a committed `.weldplan`
(`--source`). Weld cases need a part: `--weld-source` names a `.weldplan`
whose program, STEP and tooling supply the first weld block. Run it in the planner's environment
(`pixi run -e motion-cpu python tools/measured_start_replay.py ...`).
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import socket
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

V1 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(V1 / "python"))

from weldplan.plan_request import read_request, write_request  # noqa: E402
from weldplan.urdf import descriptor_of  # noqa: E402

RECORD = struct.Struct(">d9d9dB")
QUERY = ("run_weld_trajopt=true&run_connecting_trajopt=true&verify=true&dense=true"
         "&free_space_backend=bspline&manifest_revision=1&plan_revision=1&program_id={}")


def context_members(directory: Path) -> dict[str, bytes]:
    return {str(path.relative_to(directory)): path.read_bytes()
            for path in sorted(directory.rglob("*")) if path.is_file()}


def start_state(cell: dict, q: list[float]) -> dict:
    robot = cell["robot_cell"]
    return {"schema": "rosie.measured-planning-start.v1", "joint_names": list(cell["axes"]),
            "joint_values_rad": [float(v) for v in q],
            "robot_cell": {key: robot[key] for key in ("model_id", "robot_description_sha256",
                                                        "machine_planning_calibration_sha256")},
            "coordinate_identity": "replay:1:1", "observation_identity": "replay:replay",
            "observed_time_ns": "1"}


def move(node_id: str, q: list[float], cell: dict, motion: str = "joint") -> dict:
    pose = {"frame_id": "world", "xyz_m": [0., 0., 0.], "rpy_rad": [0., 0., 0.],
            "joint_names": list(cell["axes"]), "joint_values_rad": [float(v) for v in q]}
    robot = cell["robot_cell"]
    return {"id": node_id, "op": "move", "enabled": True, "label": node_id, "comment": "",
            "row_hash": f"pending:{node_id}", "motion": motion, "target": pose, "speed_scale": .5,
            "speed_mm_s": 100., "capture": {"source": "preview", "observed_at": "2026-10-11T00:00:00Z",
                                           "model_id": robot["model_id"],
                                           "robot_description_sha256": robot["robot_description_sha256"],
                                           "pose": pose}}


def event(node_id: str, op: str, **extra) -> dict:
    return {"id": node_id, "op": op, "enabled": True, "label": node_id, "comment": "",
            "row_hash": f"pending:{node_id}", **extra}


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def decode(blob: bytes) -> tuple[dict, list[list[tuple]]]:
    length = struct.unpack(">I", blob[:4])[0]
    header = json.loads(blob[4:4 + length])
    offset, segments = 4 + length, []
    for meta in header["segments"]:
        samples = [RECORD.unpack_from(blob, offset + i * RECORD.size) for i in range(meta["sample_count"])]
        offset += meta["sample_count"] * RECORD.size
        segments.append(samples)
    return header, segments


def plan(origin: str, data: bytes, program_id: str) -> tuple[dict, float]:
    began = time.monotonic()
    request = urllib.request.Request(f"{origin}/api/motion/plan?" + QUERY.format(program_id), data=data,
                                     headers={"Content-Type": "application/octet-stream"})
    try:
        body = urllib.request.urlopen(request, timeout=3600).read()
    except urllib.error.HTTPError as error:
        body = error.read()
    return json.loads(body), time.monotonic() - began


def judge(name: str, base: dict, nodes: list[dict], start_q: list[float] | None, origin: str,
          reference: dict | None, expect_refusal: str | None) -> dict:
    cell = base["cell"]
    program = copy.deepcopy(base["program"])
    program["nodes"] = nodes
    switches = any(n["op"] == "io" for n in nodes)
    data = write_request(request_id=name, program=program, cell=cell, step_bytes=base["step_bytes"],
                         step_filename="part.step", tooling=base["tooling"], fixtures=base["fixtures"],
                         context=base["context"],
                         start_state=None if start_q is None else start_state(cell, start_q),
                         io_outputs=base.get("io_outputs") if switches else None)
    result, seconds = plan(origin, data, name.replace("+", "-"))
    out = {"case": name, "start": "measured" if start_q is not None else "program start",
           "plan_s": round(seconds, 2), "timings_ms": result.get("timings_ms")}
    dense = result.get("dense")
    if dense is None:
        reason = result.get("dense_error") or result.get("connecting_trajectories_error") or result.get("error") or result
        out["refused"] = json.dumps(reason)[:600]
        out["ok"] = bool(expect_refusal and expect_refusal in out["refused"])
        return out
    if expect_refusal:
        out.update(ok=False, refused="planned, but a refusal naming %r was expected" % expect_refusal)
        return out
    blob = urllib.request.urlopen(f"{origin}/api/motion/dense/{dense['trajectory_digest']}").read()
    header, segments = decode(blob)
    names = list(cell["axes"])
    columns = [int(n[1:]) - 1 for n in names]
    limits = {j["name"]: j["limits_rad"] for j in cell["joints"] if j.get("limits_rad")}
    problems = []
    first = segments[0][0][1:10]
    if start_q is not None:
        worst = max(abs(first[c] - q) for c, q in zip(columns, start_q))
        out["first_sample_minus_start_rad"] = worst
        if worst > 1e-10:
            problems.append(f"first sample is {worst:.3g} rad from the measured start")
    margin = {}
    for name_, column in zip(names, columns):
        lo, hi = limits[name_]["lower"], limits[name_]["upper"]
        values = [s[1 + column] for seg in segments for s in seg]
        margin[name_] = round(min(min(values) - lo, hi - max(values)), 6)
        if min(values) < lo - 1e-12 or max(values) > hi + 1e-12:
            problems.append(f"{name_} leaves [{lo}, {hi}]: {min(values):.6f}..{max(values):.6f}")
    out["limit_margin_rad"] = margin
    out["segments"] = [f"{m['kind']}:{m['source_id']}" for m in header["segments"]]
    verdicts = [m.get("verdict", {}).get("verdict") for m in result.get("connecting_trajectories") or []]
    verdicts += [(s.get("trajectory") or {}).get("verdict", {}).get("verdict") for s in result.get("seams") or []]
    out["verdicts"] = {v: verdicts.count(v) for v in set(verdicts)}
    if any(v != "PASS" for v in verdicts):
        problems.append(f"verdicts {out['verdicts']}")
    out["samples"], out["duration_s"] = header["sampling"]["total_sample_count"], header["sampling"]["total_duration_s"]
    if switches:
        # The output schedule in seconds: each ON refreshed inside its expiry,
        # an explicit OFF last, the digest the OLP checks.
        from weldplan.io_schedule import markers_digest
        markers = header.get("process_markers") or []
        starts, at = [], 0
        for meta in header["segments"]:
            starts.append(at)
            at += meta["sample_count"] - 1
        dt = header["sampling"]["dt_s"]
        times = [round((starts[m["segment"]] + m["sample"]) * dt, 6) for m in markers]
        expiry = {o["name"]: o["expiry_ns"] * 1e-9 for o in base["io_outputs"]["outputs"]}
        gaps = {}
        for name in expiry:
            ons = [t for m, t in zip(markers, times) if m["action"] == name and m["value"]]
            gaps[name] = max((b - a for a, b in zip(ons, ons[1:])), default=0.)
            if gaps[name] >= expiry[name]:
                problems.append(f"{name} ON gap {gaps[name]:.3f} s reaches its {expiry[name]:.3f} s expiry")
        out["markers"] = {"count": len(markers), "first": markers[:1], "last": markers[-1:],
                          "max_on_gap_s": gaps, "digest_ok": header.get("process_markers_sha256") == markers_digest(markers)}
        if not markers or markers[-1]["value"] is not False or not out["markers"]["digest_ok"]:
            problems.append(f"output schedule {out['markers']}")
    if reference is not None:
        welds = {m["source_id"]: seg for m, seg in zip(header["segments"], segments) if m["kind"] == "weld"}
        diff = 0.
        for source, seg in welds.items():
            other = reference.get(source)
            if other is None or len(other) != len(seg):
                problems.append(f"weld {source} differs in length from the model-reset plan")
                continue
            for a, b in zip(seg, other):
                for c in columns:
                    delta = (a[1 + c] - b[1 + c] + math.pi) % (2 * math.pi) - math.pi
                    diff = max(diff, abs(delta))
        out["weld_vs_reset_max_rad_mod_2pi"] = diff
        if diff > 1e-9:
            problems.append(f"weld samples moved {diff:.3g} rad against the model-reset plan")
    out["problems"] = problems
    out["ok"] = not problems
    out["_welds"] = {m["source_id"]: seg for m, seg in zip(header["segments"], segments) if m["kind"] == "weld"}
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--context-dir", type=Path, help="a compile-context output directory (the robot)")
    parser.add_argument("--source", type=Path, help="a .weldplan whose cell, context and program are the robot")
    parser.add_argument("--weld-source", type=Path, help="a .weldplan supplying the part, tooling and first weld block")
    parser.add_argument("--start", type=float, nargs="+", required=True, help="measured start, radians, cell axis order")
    parser.add_argument("--waypoint", type=float, nargs="+", action="append", default=[],
                        help="a taught waypoint (repeat), radians, cell axis order")
    parser.add_argument("--over-limit", type=float, nargs="+", help="a waypoint past a limit; must be refused")
    parser.add_argument("--io-outputs", type=Path, help="a rosie.cell-io-outputs.v1 document (the cell's outputs)")
    parser.add_argument("--origin", help="a running motion server; default: start one here")
    parser.add_argument("--cases", default="taught+home,home-only,weld-only,weld-only+home,taught+weld,over-limit,taught+io")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    if args.context_dir:
        context = context_members(args.context_dir)
        cell = descriptor_of(context, fixtures=[])
        base = {"cell": cell, "context": context, "fixtures": [], "tooling": None, "step_bytes": None,
                "program": None}
    else:
        opened = read_request(args.source.read_bytes())
        base = {key: opened[key] for key in ("cell", "context", "fixtures", "tooling", "step_bytes", "program")}
        cell = base["cell"]
    model = cell["robot_cell"]["model_id"]
    base["io_outputs"] = json.loads(args.io_outputs.read_text()) if args.io_outputs else None
    gas = (base["io_outputs"] or {}).get("outputs", [{}])[0].get("name", "gas")
    weld = read_request(args.weld_source.read_bytes()) if args.weld_source else None
    if base["program"] is None:
        template = copy.deepcopy(weld["program"]) if weld else None
        if template is None:
            from weldplan.program_v2 import _minimal
            template = _minimal()
        base["program"] = template
    base["program"]["robot"] = {key: cell["robot_cell"][key] for key in ("model_id", "robot_description_sha256")}
    if isinstance(base["program"].get("placement"), dict):
        base["program"]["placement"]["cell_id"] = model
    block = []
    if weld:
        base.update(step_bytes=weld["step_bytes"], tooling=weld["tooling"])
        nodes = weld["program"]["nodes"]
        first = next(i for i, n in enumerate(nodes) if n["op"] == "weld")
        block = [nodes[first - 1], nodes[first], {**nodes[next(i for i, n in enumerate(nodes) if n["op"] == "retract")]}]
    waypoints = [move(f"P{i + 1}", q, cell) for i, q in enumerate(args.waypoint)]
    home = lambda tag: event(f"home-{tag}", "home")  # noqa: E731
    cases = {
        "taught+home": [home("in"), *waypoints[:1], event("pause", "dwell", duration_s=.25), *waypoints[1:], home("out")],
        "home-only": [home("only")],
        "weld-only": block,
        "weld-only+home": [home("in"), *block, home("out")],
        "taught+weld": [*waypoints[:1], *block, *[copy.deepcopy(w) | {"id": w["id"] + "-back", "row_hash": "pending:back"} for w in waypoints[:1]], home("out")],
        "over-limit": [move("past-limit", args.over_limit or [], cell)] if args.over_limit else [],
        "taught+io": [*waypoints[:1], event("out-on", "io", channel=gas, state=True), *waypoints[1:],
                      event("out-off", "io", channel=gas, state=False), home("out")] if base["io_outputs"] else [],
    }

    server = None
    origin = args.origin
    if origin is None:
        port = free_port()
        server = subprocess.Popen([sys.executable, "-m", "weld_motion_planner.server.motion_planner_server",
                                   "--host", "127.0.0.1", "--port", str(port), "--dense-store-dir", "/tmp/replay-dense"],
                                  env={**os.environ, "PYTHONPATH": str(V1 / "python")})
        origin = f"http://127.0.0.1:{port}"
        for _ in range(1200):
            try:
                urllib.request.urlopen(origin + "/api/motion/health", timeout=2)
                break
            except Exception:
                time.sleep(.25)
    report = {"model": model, "start": args.start, "threads": os.environ.get("OMP_NUM_THREADS", "default"),
              "cpus": os.cpu_count(), "cases": []}
    try:
        for name in [c for c in args.cases.split(",") if c]:
            nodes = cases[name]
            if not nodes:
                report["cases"].append({"case": name, "skipped": "needs --weld-source / --over-limit"})
                continue
            reference = None
            if any(n["op"] == "weld" for n in nodes):
                ref = judge(name + "-reset", base, nodes, None, origin, None, None)
                reference = ref.pop("_welds", None)
                report["cases"].append(ref)
            expect = "limit" if name == "over-limit" else None
            outcome = judge(name, base, nodes, args.start, origin, reference, expect)
            outcome.pop("_welds", None)
            report["cases"].append(outcome)
            print(json.dumps(outcome), flush=True)
    finally:
        if server is not None:
            server.terminate()
            server.wait(timeout=30)
    text = json.dumps(report, indent=1)
    if args.report:
        args.report.write_text(text)
    print(text)
    return 0 if all(c.get("ok", True) for c in report["cases"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
