"""The execution I/O schedule: a program's `io` rows as timed output markers.

A program.v2 `io` node (`channel`, `state`) switches one of the connected
cell's configured digital outputs, with the arm at rest between the motion
nodes around it. The planner turns the rows into rt-core dense
`process_markers` (`{segment, sample, action, value}`, rt-core
`docs/CELL_IO.md` and `APPLICATION_API.md`): the action names the output and
the marker is delivered at that exact source sample, bounded to one native
cycle. Nothing here switches anything; it decides when, on the plan's clock.

The cell, not the program, says which outputs exist: `io_outputs.json` in the
`.weldplan` (schema `rosie.cell-io-outputs.v1`) is the `outputs` list of the
connected cell's rt-core Describe `io` block, captured by the OLP at pack time
(name, class, expiry). Without it an `io` row is refused; a channel that is
not an output, or a torch-class output on a cell whose terminal is not
qualified, is refused by name. The failure this exists to prevent is an output
that a plan silently drops (before 2026-10-11 the weld-only connector did) or
one that stays on: every ON is refreshed inside the output's own expiry while
the program says ON, so a plan that stops advancing (Halt, Pause, a fault, a
lost client) lets the output expire within `expiry_ns`, and the last sample
carries an explicit OFF for every output still ON. Stop is rt-core's: it
writes every output OFF in its fence cycle.

Stdlib only, like the rest of `weldplan`.
"""
from __future__ import annotations

import hashlib
import json
import math

SCHEMA = "rosie.cell-io-outputs.v1"
CLASSES = ("process", "torch")

# The dwell an `io` row executes in: the marker sits on its first sample and
# the arm holds still for the rest. Five 10 ms samples keep two rows that
# switch outputs back to back 50 ms apart, many native exchanges (1 ms), so
# rt-core's "two markers for one output in one exchange" fault cannot be hit
# by authoring. Unverified on hardware: readback settles in three exchanges
# in simulation (CELL_IO.md), and a real terminal's lag is unmeasured.
IO_EVENT_S = 0.05

# An ON is re-sent when half its expiry has passed on the plan's clock. Half,
# so one refresh may land late by up to a quarter expiry and the output still
# holds; never closer than two samples, or two markers could share an
# exchange. Unverified against a real terminal's marker latency.
REFRESH_FRACTION = 0.5


def validate_outputs(doc):
    """The cell's outputs as `{name: {"class", "expiry_ns"}}`, or ValueError."""
    if not isinstance(doc, dict) or doc.get("schema") != SCHEMA:
        raise ValueError(f"io_outputs must be a {SCHEMA} document")
    outputs = doc.get("outputs")
    if not isinstance(outputs, list):
        raise ValueError("io_outputs.outputs must be a list")
    table = {}
    for entry in outputs:
        name = entry.get("name") if isinstance(entry, dict) else None
        if not isinstance(name, str) or not name or name in table:
            raise ValueError("every cell output needs a unique non-empty name")
        if entry.get("class") not in CLASSES:
            raise ValueError(f"output {name!r} class must be one of {', '.join(CLASSES)}")
        expiry = entry.get("expiry_ns")
        if type(expiry) is not int or not 1 <= expiry <= 1_000_000_000:
            raise ValueError(f"output {name!r} expiry_ns must be 1..1000000000 ns (rt-core's bound)")
        table[name] = {"class": entry["class"], "expiry_ns": expiry}
    if type(doc.get("torch_qualified")) is not bool:
        raise ValueError("io_outputs.torch_qualified must be stated")
    return table


def io_problems(nodes, doc):
    """Why the enabled `io` rows cannot be scheduled, as sentences naming
    each row; empty when they can."""
    rows = [n for n in nodes if n.get("enabled", True) and n.get("op") == "io"]
    if not rows:
        return []
    names = ", ".join(_node_name(n) for n in rows)
    if doc is None:
        return [f"{names} {'switches an output' if len(rows) == 1 else 'switch outputs'}: plan with the cell "
                "connected, so the plan knows the cell's outputs and their expiry"]
    try:
        table = validate_outputs(doc)
    except ValueError as error:
        return [f"the cell's I/O description is invalid: {error}"]
    known = ", ".join(sorted(table)) or "none"
    out = []
    monitors = set()
    for row in rows:
        channel, state = row.get("channel"), row.get("state")
        if type(state) is not bool:
            out.append(f"{_node_name(row)} needs an on/off state")
        if row.get("action") in ("wait", "monitor", "end_monitor"):
            if channel not in {entry.get("name") for entry in doc.get("inputs", [])}:
                out.append(f"{_node_name(row)} waits for {channel!r}, which is not an input of this cell")
            action = row.get("action")
            if action == "monitor":
                if channel in monitors: out.append(f"{_node_name(row)} starts an already active monitor")
                monitors.add(channel)
            elif action == "end_monitor":
                if channel not in monitors: out.append(f"{_node_name(row)} ends a monitor that is not active")
                monitors.discard(channel)
            continue
        if channel not in table:
            out.append(f"{_node_name(row)} switches {channel!r}, which is not an output of this cell (outputs: {known})")
        elif table[channel]["class"] == "torch" or channel == doc.get("weld_output"):
            out.append(f"{_node_name(row)} switches torch output {channel!r}, which is reserved for semantic weld start/stop "
                       "and cannot be commanded by a generic I/O row")
    if monitors: out.append("Missing End monitor for: " + ", ".join(sorted(monitors)))
    return out


def schedule(segments, events, doc, dt_s, pulse_ends=None, boundary_events=None):
    """The dense `process_markers` for a plan.

    `segments` is the sample count of each dense segment in order; segment
    k > 0's first sample is the same instant as segment k-1's last (rt-core
    collapses that endpoint). `events` maps a segment index to the
    `(output, value)` its first sample switches, in program order. Returns
    `[{segment, sample, action, value}]` sorted by time.
    """
    table = validate_outputs(doc)
    pulse_ends = pulse_ends or {}
    boundary_events = boundary_events or {}
    refresh = {}
    for name, spec in table.items():
        samples = math.floor(spec["expiry_ns"] * 1e-9 * REFRESH_FRACTION / dt_s + 1e-9)
        if samples < 2:
            raise ValueError(f"output {name!r} expires in {spec['expiry_ns']} ns, shorter than two refreshes on "
                             f"the {dt_s} s dense grid")
        refresh[name] = samples
    on, last, timed = {}, {}, {}

    def place(instant, index, sample, name, value):
        # One marker per output per instant: an event or the final OFF
        # replaces a refresh that fell on the same (shared) sample.
        key = (instant, name)
        timed.pop(key, None)
        timed[key] = {"segment": index, "sample": sample, "action": name, "value": value}

    base = 0
    for index, count in enumerate(segments):
        for sample in range(count):
            instant = base + sample
            if (index,sample) in boundary_events:
                for name,value in boundary_events[(index,sample)]:
                    place(instant,index,sample,name,value)
                    on[name],last[name]=value,instant
            if sample == count-1 and index in pulse_ends:
                name=pulse_ends[index]
                place(instant,index,sample,name,False)
                on[name],last[name]=False,instant
            if sample == 0 and index in events:
                for name, value in events[index]:
                    place(instant, index, 0, name, bool(value))
                    on[name], last[name] = bool(value), instant
                continue
            if index > 0 and sample == 0:
                continue  # the previous segment's last sample, already judged
            for name in sorted(n for n, v in on.items() if v):
                if instant - last[name] >= refresh[name]:
                    place(instant, index, sample, name, True)
                    last[name] = instant
        base += count - 1
    for name in sorted(n for n, v in on.items() if v):
        place(base, len(segments) - 1, segments[-1] - 1, name, False)
    return [marker for _, marker in sorted(timed.items(), key=lambda m: m[0][0])]


def markers_digest(markers):
    """`sha256:` of the canonical marker list. The v1 trajectory digest covers
    samples only, so this is what binds an output schedule to its plan."""
    canonical = json.dumps(markers, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return "sha256:" + hashlib.sha256(canonical).hexdigest()


def _node_name(node):
    op, label = str(node.get("op")), str(node.get("label") or "").strip()
    return op.upper() if not label or label.lower() == op.lower() else f"{label} ({op})"
