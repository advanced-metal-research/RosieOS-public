"""The execution I/O schedule (weldplan/io_schedule.py), in absolute time.

The contract: an `io` row switches a configured output on the first sample of
its own dwell; an ON is re-sent before its expiry runs out, measured in
seconds on the plan's 10 ms clock; every output still ON gets an explicit OFF
on the last sample; never two markers for one output on one instant (rt-core
faults on that); rows that cannot be scheduled are refused by name.
"""
from __future__ import annotations

import pytest

from weldplan import io_schedule

DT = 0.01
OUTPUTS = {"schema": io_schedule.SCHEMA, "torch_qualified": False, "outputs": [
    {"name": "gas", "class": "process", "expiry_ns": 200_000_000},
    {"name": "torch", "class": "torch", "expiry_ns": 200_000_000}]}


def _io(node_id, channel, state, label=None):
    return {"id": node_id, "op": "io", "label": label or node_id, "channel": channel, "state": state}


def test_rows_are_refused_by_name_unless_the_cell_can_switch_them():
    gas = _io("g", "gas", True, "Gas on")
    assert io_schedule.io_problems([gas], OUTPUTS) == []
    assert io_schedule.io_problems([{**gas, "enabled": False}], None) == []
    assert io_schedule.io_problems([gas], None) == [
        "Gas on (io) switches an output: plan with the cell connected, so the plan knows the cell's outputs and their expiry"]
    assert io_schedule.io_problems([_io("r", "revpi:1", True, "IO")], OUTPUTS) == [
        "IO switches 'revpi:1', which is not an output of this cell (outputs: gas, torch)"]
    torch = io_schedule.io_problems([_io("t", "torch", True, "Arc")], OUTPUTS)
    assert torch and "reserved for semantic weld start/stop" in torch[0]
    assert "reserved for semantic weld start/stop" in io_schedule.io_problems([_io("t", "torch", True)], {**OUTPUTS, "torch_qualified": True})[0]
    assert "on/off state" in io_schedule.io_problems([_io("g", "gas", "yes")], OUTPUTS)[0]
    with pytest.raises(ValueError, match="expiry_ns"):
        io_schedule.validate_outputs({**OUTPUTS, "outputs": [{"name": "gas", "class": "process", "expiry_ns": 0}]})


def _times(markers, segments):
    """Each marker's instant in seconds on the plan's clock (shared endpoints
    collapse: segment k's sample 0 is segment k-1's last sample)."""
    starts, base = [], 0
    for count in segments:
        starts.append(base)
        base += count - 1
    return [round((starts[m["segment"]] + m["sample"]) * DT, 9) for m in markers]


def test_an_on_is_refreshed_inside_its_expiry_and_ends_off():
    # Segments: move (101 samples, 1 s), gas ON row (6), move (301, 3 s), gas OFF row (6), move (51).
    segments = [101, 6, 301, 6, 51]
    markers = io_schedule.schedule(segments, {1: [("gas", True)], 3: [("gas", False)]}, OUTPUTS, DT)
    gas = [m for m in markers if m["action"] == "gas"]
    times = _times(gas, segments)
    assert gas[0] == {"segment": 1, "sample": 0, "action": "gas", "value": True} and times[0] == pytest.approx(1.0)
    assert gas[-1] == {"segment": 3, "sample": 0, "action": "gas", "value": False}
    on_times = [t for m, t in zip(gas, times) if m["value"]]
    gaps = [b - a for a, b in zip(on_times, on_times[1:])]
    # Half the 200 ms expiry: every gap is 0.10 s, and the OFF is within it.
    assert gaps and max(gaps) == pytest.approx(0.10, abs=1e-9)
    assert times[-1] - on_times[-1] <= 0.10 + 1e-9
    assert all(m["value"] is True for m in gas[:-1])
    assert len({(t, m["action"]) for m, t in zip(markers, _times(markers, segments))}) == len(markers)


def test_an_output_left_on_gets_an_explicit_off_on_the_last_sample():
    segments = [6, 151]
    markers = io_schedule.schedule(segments, {0: [("gas", True)]}, OUTPUTS, DT)
    assert markers[-1] == {"segment": 1, "sample": 150, "action": "gas", "value": False}
    times = _times(markers, segments)
    assert times[-1] == pytest.approx(1.55)
    # No refresh collides with the final OFF on the same instant.
    assert len({(t, m["action"]) for m, t in zip(markers, times)}) == len(markers)


def test_an_event_on_a_shared_endpoint_replaces_a_refresh_at_that_instant():
    # A refresh would fall on the last sample of segment 1 (instant 0.20 s),
    # which is also segment 2's first sample where the OFF row sits.
    segments = [6, 16, 6]
    markers = io_schedule.schedule(segments, {0: [("gas", True)], 2: [("gas", False)]}, OUTPUTS, DT)
    times = _times(markers, segments)
    assert len({(t, m["action"]) for m, t in zip(markers, times)}) == len(markers)
    assert markers[-1] == {"segment": 2, "sample": 0, "action": "gas", "value": False}


def test_an_expiry_shorter_than_two_samples_of_refresh_is_refused():
    short = {**OUTPUTS, "outputs": [{"name": "gas", "class": "process", "expiry_ns": 30_000_000}]}
    with pytest.raises(ValueError, match="shorter than two refreshes"):
        io_schedule.schedule([6, 11], {0: [("gas", True)]}, short, DT)


def test_markers_digest_is_canonical():
    a = [{"segment": 0, "sample": 0, "action": "gas", "value": True}]
    b = [{"value": True, "action": "gas", "sample": 0, "segment": 0}]
    assert io_schedule.markers_digest(a) == io_schedule.markers_digest(b)
    assert io_schedule.markers_digest(a) != io_schedule.markers_digest([{**a[0], "value": False}])


def test_full_dio_schedule_keeps_one_marker_per_channel_instant():
    names = [f"O{i}" for i in range(1, 15)]
    doc = {"schema": io_schedule.SCHEMA, "torch_qualified": False,
           "outputs": [{"name": name, "class": "process", "expiry_ns": 200_000_000} for name in names]}
    markers = io_schedule.schedule([10001], {0: [(name, True) for name in names]}, doc, DT)
    assert len(markers) == 14014
    assert len({(m["sample"], m["action"]) for m in markers}) == len(markers)
    assert all(m["sample"] == 10000 and not m["value"] for m in markers[-14:])
    assert all(m["value"] for m in markers[:-14])


def test_input_monitors_require_paired_scope_and_a_real_input():
    doc={"schema":io_schedule.SCHEMA,"torch_qualified":False,"outputs":[],"inputs":[{"name":"sensor"}]}
    start={"id":"m","op":"io","action":"monitor","channel":"sensor","state":True}
    end={**start,"id":"e","action":"end_monitor"}
    assert not io_schedule.io_problems([start,end],doc)
    assert io_schedule.io_problems([start],doc)
    assert io_schedule.io_problems([end,start],doc)
    assert io_schedule.io_problems([start,start,end],doc)
    assert io_schedule.io_problems([{**start,"channel":"missing"},end],doc)
