#!/usr/bin/env python
"""Turn detected joints into the seams you would actually weld.

A joint is a CAD fact: the whole boundary where two parts meet. A seam is weld
intent, and intent has to respect both what a torch can reach and where the
weld line turns. So one joint becomes **one seam per reachable run, cut again
at every sharp corner**.

Three consequences worth stating, because getting any of them backwards
produces a program that looks right and cannot be welded:

**Split at obstructions.** A seam is not a single stretch with holes in it. The
column-to-base-plate loop on `pipe_bracket` is interrupted twice by gussets, so it
is two runs of 81.48 mm, not one 182.83 mm seam with two gaps. A planner
reading a seam is entitled to assume the arc stays lit from `start_s` to
`end_s`.

**Split at corners.** At a C0 junction the tangent flips, the groove bisector
flips with it, and the torch cannot drive through without either stopping or
reorienting in place. `continuity.break_positions_s` finds those; whether one
is sharp enough to act on is the process judgement below.

**Combine where continuous.** A reachable run on a closed loop can straddle
s=0, because `chain_edges` starts the loop at an arbitrary edge. That is ONE
continuous run, not two fragments that happen to touch the parametrisation's
seam. Such a span is emitted with `start_s > end_s`, which is how a consumer
knows it wraps.

The two splits are **not interchangeable**, and `origin` says which is which.
Pieces cut at a corner touch end to end and can be re-merged; runs separated by
an obstruction have unweldable material between them and cannot. Both kinds of
sibling share `weld_joint_ref`, so that field alone cannot tell them apart --
`origin.group_id` can.

Order matters: reachability first, then continuity. A corner inside a blocked
stretch is not a corner anyone welds through, and cutting the other way round
would produce groups spanning unreachable arc.

Everything here is a *default*. The operator changes direction, trims spans,
merges a group back into one seam, adds passes, or deletes seams entirely; this
only makes the starting point one that can be welded.
"""

from __future__ import annotations

import argparse
import json
import sys
from itertools import pairwise
from pathlib import Path
from typing import Any

from seam_worker import paths
from seam_worker.seam.continuity import junction_positions_s
from seam_worker.seam.span import span_length_m

SCHEMA = "amr-weld-planner-v1.seams.v1"

# Below this a run is not worth emitting as a seam at all. Unlike the resolution
# floor in access.py this IS a process judgement, which is why it lives here in
# intent generation and not in the geometric analysis -- and why it is a
# parameter rather than a constant buried in a function.
DEFAULT_MIN_SEAM_LENGTH_M = 0.005

# Above this a corner is worth stopping the arc for. Also a process judgement,
# which is why it sits here and not next to continuity.py's TANGENT_TOL_DEG --
# that one answers "is this the same direction?", a question about geometry;
# this one answers "should the weld stop here?", a question about the torch.
#
# 5.0 because it is the `tolerance.work_angle_deg` this same function already
# emits below: under 5 degrees the torch reference direction turns by less than
# the angular tolerance the seam itself declares, so driving through the corner
# is already inside the intent that was stated.
DEFAULT_MAX_CORNER_ANGLE_DEG = 5.0

# 4.0 mm/s travel, down from the 8.0 the process defaults carried until
# 2026-09-07, because the cell's measured joint rates bind at 8: on
# rosie_1400_v3 (the nine-motor bench's pair revision 4, wrist capped at 0.2 rad/s by
# RTCore's runtime limit) the seam search's derived edge rule is 2.86 degrees
# per 0.25 s step on the wrist, and bracket_b_2x seam_0003 asks the electrode
# to turn 6.27 degrees over 2 mm of travel, so at 8 mm/s no path exists.
# Swept on that seam: it crosses at 5 mm/s with 26 nodes in its thinnest
# layer, at 4 mm/s with 6296 -- 4 is the first value with room to spare. A
# faster cell is a new profile number and a new value here.
DEFAULT_TRAVEL_SPEED_MM_S = 4.0

# How far a detected seam holds the torch off the work, until a procedure says
# otherwise. The cell's one number, set by the owner (2026-09-11); every other
# default standoff -- the torch fallback, the OLP's, the motion corpus -- is
# this one. Measured on bracket_b_2x through the motion planner with the part
# at its pivot-frame placement: 12 mm crossed 4 of 22 seams and 17 mm crossed
# 8; 22 mm also crossed 8. At 12 mm the tip sphere of the Ø30 gun baked into
# link_6 (r 14.6 mm) sat 0.2 mm inside the part against a 0 mm clearance floor.
# 17 is inside what both fitted guns are held at (Ø16 nozzle 6-25 mm,
# rosie_link6_gun 15-30 mm).
DEFAULT_STANDOFF_MM = 17.0

DEFAULT_PROCESS: dict[str, Any] = {
    "method": "GMAW",
    "iso_4063": 135,
    "travel_speed_mm_s": {"kind": "constant", "value": DEFAULT_TRAVEL_SPEED_MM_S},
    "arc": {
        "wire_feed_speed_mm_s": {"kind": "constant", "value": 105.0},
        "voltage_v": {"kind": "constant", "value": 19.5},
        "current_a": {"kind": "constant", "value": 150.0},
        "mode": "synergic",
        "polarity": "dcep",
    },
    "wire": {"classification": "ER70S-6", "diameter_mm": 1.0, "stickout_mm": DEFAULT_STANDOFF_MM},
    "gas": {"mixture": "Ar82/CO2-18", "flow_l_min": 15.0, "preflow_s": 0.3, "postflow_s": 1.0},
    "weave": {"shape": "none"},
    "start": {"ignition": "scratch", "run_in_s": 0.2, "hot_start_scale": 1.3},
    "end": {"crater_fill_s": 0.4, "burnback_s": 0.08},
}


def run_length_m(run: list[float], total_m: float) -> float:
    """Arc length of a run, handling the wrap case where start > end."""
    return span_length_m({"start_s": run[0], "end_s": run[1]}, total_m)


# Two positions are the same point if they agree to this. Reachable-run
# boundaries come out of a bisection in access.py and junction positions out of
# a sum of declared lengths, so equality is never exact.
_SAME_S = 1e-9

def _cut_run(run: list[float], breaks: list[float]) -> list[list[float]]:
    """Cut one reachable run at every break strictly inside it.

    Walking is always FORWARD from `run[0]`, through s=0 if the run wraps, so a
    wrapping run yields ordinary pieces plus at most one that still wraps. The
    order of the returned pieces is the order the torch would travel them.
    """
    lo, hi = run
    wraps = hi < lo

    if wraps:
        after = sorted(b for b in breaks if b > lo + _SAME_S)
        before = sorted(b for b in breaks if b < hi - _SAME_S)
        inside = after + before
    else:
        inside = sorted(b for b in breaks if lo + _SAME_S < b < hi - _SAME_S)

    boundaries = [lo, *inside, hi]
    pieces = []
    for start, end in pairwise(boundaries):
        if abs(end - start) > _SAME_S or end < start:
            pieces.append([start, end])
    return pieces or [[lo, hi]]


def _absorb_short_pieces(
    pieces: list[list[float]], total_m: float, min_length_m: float
) -> list[list[float]]:
    """Merge a too-short piece into its neighbour rather than dropping it.

    A run boundary is measured by bisection in `access.py` (~0.06 mm) and a
    corner by summing declared lengths, so where a gusset ends exactly at a
    corner the two disagree by a hair: on pipe_bracket that leaves a 0.031 mm stub
    at the end of four runs. Dropping those would quietly lose weldable arc and
    break conservation; the stub is measurement noise about where one boundary
    is, not a separate stretch of weld, so it is absorbed into the piece it
    touches.

    This mirrors `access._absorb_slivers`, which solves the same problem one
    stage earlier. A run whose TOTAL length is under the minimum still yields
    nothing -- that is the process floor doing its job, and the caller counts it.
    """
    if min_length_m <= 0.0 or not pieces:
        return pieces
    merged = [list(p) for p in pieces]
    while len(merged) > 1:
        lengths = [run_length_m(p, total_m) for p in merged]
        shortest = min(range(len(merged)), key=lambda i: lengths[i])
        if lengths[shortest] >= min_length_m:
            break
        # Into the previous piece where there is one, so absorption walks
        # forward and the group keeps its travel order.
        into = shortest - 1 if shortest > 0 else 1
        low, high = min(shortest, into), max(shortest, into)
        merged[low : high + 1] = [[merged[low][0], merged[high][1]]]
    return merged


def _continuity_at(junctions: list[dict[str, Any]], s: float, closed: bool) -> str | None:
    """The junction class at `s`, or None if no junction is there.

    On a closed curve s=1 and s=0 are the same point, so a piece ending at 1.0
    has to be matched against a junction recorded at 0.0.
    """
    for junction in junctions:
        if abs(junction["s"] - s) <= _SAME_S:
            return junction["klass"]
        if closed and abs(abs(junction["s"] - s) - 1.0) <= _SAME_S:
            return junction["klass"]
    return None


def seams_for_joint(
    joint: dict[str, Any],
    index_from: int = 1,
    min_length_m: float = DEFAULT_MIN_SEAM_LENGTH_M,
    work_angle_deg: float = 0.0,
    travel_angle_deg: float = 10.0,
    max_corner_angle_deg: float = DEFAULT_MAX_CORNER_ANGLE_DEG,
) -> list[dict[str, Any]]:
    """One seam per reachable run, cut again at every corner over threshold.

    Falls back to a single full-span run when accessibility has not been
    computed -- absence of analysis is not evidence of obstruction, and refusing
    to emit a seam would be worse than emitting the whole boundary. The corner
    split still applies to that fallback: a corner is a fact about the curve,
    not about reachability.
    """
    weld_line = joint["weld_line"]
    total_m = weld_line["length_m"]
    closed = bool(weld_line.get("closed"))
    access = joint.get("accessibility") or {}
    runs = access.get("reachable_runs")
    if runs is None:
        runs = [[0.0, 1.0]]
    # Did accessibility actually bound anything? One run covering the whole
    # curve means it did not, and a run ending at s=1 there is the curve's own
    # end rather than an obstruction.
    bounded_by_reachability = not (
        len(runs) == 1 and runs[0][0] <= _SAME_S and runs[0][1] >= 1.0 - _SAME_S
    )

    junctions = junction_positions_s(weld_line)
    breaks = [j["s"] for j in junctions if j["tangent_angle_deg"] > max_corner_angle_deg]

    rail = (joint.get("bisector_rail") or {}).get("curve")
    reference: dict[str, Any] = (
        {
            "mode": "rail_curve",
            "rail": rail,
            "correspondence": "normalized_arc_length",
            "semantics": "bisector",
        }
        if rail
        else {"mode": "fixed_vector", "fixed_direction_xyz": [0.0, 0.0, 1.0], "semantics": "custom"}
    )

    out: list[dict[str, Any]] = []
    for run_index, run in enumerate(runs):
        if run_length_m(run, total_m) < min_length_m:
            continue
        pieces = _absorb_short_pieces(_cut_run(run, breaks), total_m, min_length_m)
        group_id = f"{joint['id']}.run{run_index:02d}"

        for piece_index, piece in enumerate(pieces):
            length = run_length_m(piece, total_m)
            wraps = piece[1] < piece[0]
            # NOT ROUNDED. See `access._clear_runs` for the argument -- the
            # same one applies, and a piece boundary is a corner or a run
            # boundary, both of which are exact by the time they arrive.
            start_s = piece[0]
            end_s = piece[1]
            at_start = _continuity_at(junctions, piece[0], closed)
            at_end = _continuity_at(junctions, piece[1], closed)

            # Why this piece ends where it does.
            #
            # Reachability outranks continuity where both are true, and on a
            # closed loop they often are -- a gusset's blocked stretch begins
            # exactly at the corner it is welded into. "There is material
            # beyond this point that no torch can reach" is the stronger claim
            # and the one a planner must not miss; the corner is still recorded
            # in `boundary_continuity`, so nothing is hidden by preferring it.
            last_piece = piece_index == len(pieces) - 1
            at_corner = any(abs(b - piece[1]) <= _SAME_S for b in breaks) or (
                closed and any(abs(abs(b - piece[1]) - 1.0) <= _SAME_S for b in breaks)
            )
            if last_piece and bounded_by_reachability:
                split_reason = "reachability"
            elif at_corner:
                split_reason = "continuity"
            else:
                split_reason = "none"

            index = index_from + len(out)
            out.append(
                {
                    "id": f"seam_{index:04d}",
                    "label": (
                        f"{joint['id']} run {run_index + 1}/{len(runs)}"
                        + (f" piece {piece_index + 1}/{len(pieces)}" if len(pieces) > 1 else "")
                        + f" · {length * 1000:.1f} mm"
                        + (" · wraps" if wraps else "")
                    ),
                    "weld_joint_ref": joint["id"],
                    # Which split produced this seam, and what it can be merged
                    # back into. Pieces of one group touch end to end; separate
                    # groups have unweldable arc between them.
                    "origin": {
                        "group_id": group_id,
                        "index_in_group": piece_index,
                        "group_size": len(pieces),
                        "split_reason": split_reason,
                        "boundary_continuity": [at_start, at_end],
                    },
                    "pass": {"index": 0, "role": "single"},
                    "curve": {"from_weld_joint": True},
                    # A span with start_s > end_s wraps through s=0. That happens
                    # when a closed loop's reachable run straddles the arbitrary
                    # point `chain_edges` chose to start at, and it is one seam
                    # rather than two because it is one continuous stretch of weld.
                    "span": {"start_s": start_s, "end_s": end_s},
                    "direction": "forward",
                    "reference": reference,
                    "orientation": {
                        "work_angle_deg": {"kind": "constant", "value": work_angle_deg},
                        "travel_angle_deg": {"kind": "constant", "value": travel_angle_deg},
                        "spin": {"mode": "free"},
                    },
                    "process": json.loads(json.dumps(DEFAULT_PROCESS)),
                    "tolerance": {
                        "position_mm": 0.5,
                        "work_angle_deg": 5.0,
                        "travel_angle_deg": 5.0,
                    },
                }
            )
    return out


def default_seams(
    joints_doc: dict[str, Any],
    min_length_m: float = DEFAULT_MIN_SEAM_LENGTH_M,
    max_corner_angle_deg: float = DEFAULT_MAX_CORNER_ANGLE_DEG,
) -> dict[str, Any]:
    """Default seams for every joint, with a summary of what was dropped."""
    seams: list[dict[str, Any]] = []
    absorbed = 0
    empty_runs = 0
    for joint in joints_doc.get("weld_joints", []):
        produced = seams_for_joint(
            joint,
            index_from=len(seams) + 1,
            min_length_m=min_length_m,
            max_corner_angle_deg=max_corner_angle_deg,
        )
        # How many corner cuts the length floor swallowed. Absorption keeps the
        # arc, so nothing is lost -- but a group with fewer pieces than it has
        # corners should say so rather than look like the corners were not
        # there.
        uncut = seams_for_joint(joint, min_length_m=0.0, max_corner_angle_deg=max_corner_angle_deg)
        absorbed += max(0, len(uncut) - len(produced))
        runs = (joint.get("accessibility") or {}).get("reachable_runs") or []
        emitted_groups = {s["origin"]["group_id"] for s in produced}
        empty_runs += sum(
            1
            for index in range(len(runs))
            if f"{joint['id']}.run{index:02d}" not in emitted_groups
        )
        seams.extend(produced)
    return {
        "schema": SCHEMA,
        "summary": {
            "seam_count": len(seams),
            "joint_count": len(joints_doc.get("weld_joints", [])),
            "group_count": len({s["origin"]["group_id"] for s in seams}),
            "runs_below_min_length": empty_runs,
            "pieces_absorbed_below_min_length": absorbed,
            "min_seam_length_m": min_length_m,
            "max_corner_angle_deg": max_corner_angle_deg,
            "weldable_length_m": round(
                sum(
                    run_length_m(
                        [s["span"]["start_s"], s["span"]["end_s"]],
                        next(
                            j["weld_line"]["length_m"]
                            for j in joints_doc["weld_joints"]
                            if j["id"] == s["weld_joint_ref"]
                        ),
                    )
                    for s in seams
                ),
                9,
            ),
        },
        "seams": seams,
    }


def self_test() -> int:
    from seam_worker.cad.topology import extract_topology
    from seam_worker.reach.access import annotate
    from seam_worker.seam.detect import detect_joints

    step = paths.PIPE_BRACKET
    if not step.exists():
        print(f"seams self-test: missing fixture {step}", file=sys.stderr)
        return 1
    topology = extract_topology(step)
    joints = annotate(step, topology, detect_joints(topology))
    result = default_seams(joints)
    summary = result["summary"]
    # 5 joints -> 6 reachable runs (the column loop is interrupted twice) ->
    # 14 pieces once the gusset rectangles are cut at their corners.
    assert summary["group_count"] == 6, summary
    assert summary["seam_count"] == 14, summary
    total = sum(j["weld_line"]["length_m"] for j in joints["weld_joints"])
    blocked = joints["summary"]["blocked_length_m"]
    # 1 um. Every span here is `round(s, 6)` of a normalized parameter, which
    # is ~0.1 um per boundary on a 190 mm weld line; nanometre agreement was
    # never an invariant, only a coincidence that survived until a boundary
    # moved. See tests/programmer/test_seams.py for the same note.
    assert abs(summary["weldable_length_m"] - (total - blocked)) < 1e-6, summary
    # Cutting must not lose arc: absorbing a sliver keeps it, dropping it would not.
    assert default_seams(joints, max_corner_angle_deg=180.0)["summary"][
        "weldable_length_m"
    ] == summary["weldable_length_m"], "the corner split must be length-neutral"
    print(
        f"seams self-test ok: {summary['seam_count']} seams in "
        f"{summary['group_count']} groups from {summary['joint_count']} joints, "
        f"{summary['weldable_length_m'] * 1000:.2f} mm weldable of "
        f"{total * 1000:.2f} mm boundary"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", default="")
    parser.add_argument("--joints", default="")
    parser.add_argument("--min-length-mm", type=float, default=DEFAULT_MIN_SEAM_LENGTH_M * 1000)
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()

    if ns.joints:
        joints = json.loads(Path(ns.joints).read_text(encoding="utf-8"))
    elif ns.step:
        from seam_worker.cad.topology import extract_topology
        from seam_worker.reach.access import annotate
        from seam_worker.seam.detect import detect_joints

        step = Path(ns.step)
        topology = extract_topology(step)
        joints = annotate(step, topology, detect_joints(topology))
    else:
        raise ValueError("one of --step, --joints or --self-test is required")

    result = default_seams(joints, ns.min_length_mm / 1000.0)
    print(json.dumps(result["summary"] if ns.summary else result, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"seams: {exc}", file=sys.stderr)
        raise SystemExit(1)
