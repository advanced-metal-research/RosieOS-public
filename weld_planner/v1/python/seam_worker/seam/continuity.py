#!/usr/bin/env python
"""How smoothly one weld-line segment joins the next.

A weld line is chained from CAD edges by matching their **endpoints**
(`detect.chain_edges`, `CHAIN_TOL_M = 1e-9`). That guarantees the curve does not
jump. It says nothing about whether the curve *turns*, and turning is what
matters to a torch: at a corner the tangent flips direction, the groove bisector
flips with it, and the arc has to stop or the wrist has to reorient in place.
This module measures that, per junction.

**Only C0 and G1 are ever emitted.** The schema's enum also offers C1 and C2,
and they are left unused deliberately:

- `G1` is *geometric*: the tangent directions agree. That is a property of the
  shape, and it is what a welder means by "smooth".
- `C1`/`C2` are *parametric*: the derivative vectors agree, in magnitude as well
  as direction. Each segment here carries its source edge's own `knots_u`
  straight out of OCCT, so one may run u in [0, 0.2] and the next u in [0, 1].
  Their derivative magnitudes differ for reasons that have nothing to do with
  the geometry, and rescaling a knot vector -- which moves no point at all --
  would change the answer. Under the model's canonical parameter, normalized
  arc length, speed is 1 on both sides, so C1 collapses onto G1 and adds
  nothing while C2 becomes "the curvatures match".

So the curvature is *measured* rather than labelled: `curvature_a_1_per_m`,
`curvature_b_1_per_m` and their relative jump go into `evidence.junctions[]`,
and a consumer that wants to ramp speed into a curvature change gets the
magnitude instead of a boolean with a threshold this module picked.

The tolerance here is a **geometric** one -- "is this the same direction?".
Whether a measured corner is sharp enough to stop the arc for is a process
judgement and lives in `seams.py`, next to the minimum seam length, for the
same reason.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

from seam_worker import paths
from seam_worker.seam.curve import Segment

SCHEMA = "amr-weld-planner-v1.junctions.v1"

# "The same direction, to within what a B-spline evaluation can resolve."
# 1e-3 deg is 17 microradians. The observed distribution is bimodal by two
# orders of magnitude on both fixtures -- dog_box's dogbone lobes break at 90
# deg, pipe_bracket's rounded column is tangent by construction and breaks below
# float noise -- so any threshold across six orders of magnitude gives the same
# classification. The tight one is chosen because it makes "tangent" a real
# claim rather than a generous one, and `test_continuity.py` asserts that
# nothing on either fixture lands in the gap, so a genuinely marginal fixture
# fails loudly instead of being classified by luck.
TANGENT_TOL_DEG = 1e-3

# Below this the curvature comparison is not meaningful: on a straight segment
# kappa is 0 and the relative jump would divide by zero.
_STRAIGHT_1_PER_M = 1e-9


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 0.0 else vector


def _angle_between_deg(a: np.ndarray, b: np.ndarray) -> float:
    """Angle between two unit vectors, via atan2.

    Not `acos(a . b)`: near zero angle -- which is exactly the regime this
    module classifies in -- the dot product is 1 - theta^2/2 and acos loses
    half its significant digits. atan2 of the cross and dot products stays
    accurate at both ends.
    """
    return math.degrees(math.atan2(float(np.linalg.norm(np.cross(a, b))), float(np.dot(a, b))))


def junction(seg_a: dict[str, Any], seg_b: dict[str, Any]) -> dict[str, Any]:
    """Measure how segment `seg_a` meets segment `seg_b`.

    Tangents come from the two `Segment`s directly. `Curve.tangent(s)` cannot be
    used: it goes through `Curve.locate(s)`, which resolves to ONE segment, so
    it structurally cannot see a jump at the boundary between two.
    """
    a = Segment(seg_a)
    b = Segment(seg_b)

    point_a, d1_a, _ = a.evaluate(a.u1)
    point_b, d1_b, _ = b.evaluate(b.u0)

    tangent_a = _unit(d1_a)
    tangent_b = _unit(d1_b)
    angle = _angle_between_deg(tangent_a, tangent_b)

    kappa_a = a.curvature(a.u1)
    kappa_b = b.curvature(b.u0)
    scale = max(abs(kappa_a), abs(kappa_b))
    relative = abs(kappa_b - kappa_a) / scale if scale > _STRAIGHT_1_PER_M else 0.0

    return {
        "position_gap_m": float(np.linalg.norm(point_b - point_a)),
        "tangent_angle_deg": angle,
        "curvature_a_1_per_m": kappa_a,
        "curvature_b_1_per_m": kappa_b,
        "curvature_relative_jump": relative,
        "klass": "G1" if angle <= TANGENT_TOL_DEG else "C0",
    }


def annotate_curve(curve: dict[str, Any]) -> list[dict[str, Any]]:
    """Write `continuity_to_next` into each segment; return the measurements.

    On a **closed** curve the last segment's value describes the junction back
    to segment 0. On an **open** curve the last segment gets no value at all --
    an absent field, never `"C0"` standing in for "there is no next segment".
    """
    segments = curve.get("segments") or []
    closed = bool(curve.get("closed"))
    measurements: list[dict[str, Any]] = []
    if len(segments) < 2:
        return measurements

    pairs = list(range(len(segments) - 1))
    if closed:
        pairs.append(len(segments) - 1)

    for index in pairs:
        following = (index + 1) % len(segments)
        measured = junction(segments[index], segments[following])
        segments[index]["continuity_to_next"] = measured["klass"]
        measurements.append(
            {
                "from_segment_index": index,
                "to_segment_index": following,
                **{k: (round(v, 12) if isinstance(v, float) else v) for k, v in measured.items()},
            }
        )
    return measurements


def junction_positions_s(curve: dict[str, Any]) -> list[dict[str, Any]]:
    """Every junction, placed on the curve's normalized arc length.

    Positions are cumulative sums of the segments' **declared** `length_m`,
    which is what `access.py` sums when it places blocked spans and what
    `seams.run_length_m` assumes. Recomputing lengths through `curve.Curve`
    would be more precise and would put these positions on a slightly different
    s-scale than the reachable runs they have to compose with.
    """
    segments = curve.get("segments") or []
    closed = bool(curve.get("closed"))
    total = sum(float(s.get("length_m", 0.0)) for s in segments)
    if len(segments) < 2 or total <= 0.0:
        return []

    out: list[dict[str, Any]] = []
    travelled = 0.0
    for index, segment in enumerate(segments):
        travelled += float(segment.get("length_m", 0.0))
        last = index == len(segments) - 1
        if last and not closed:
            continue
        measured = junction(segment, segments[(index + 1) % len(segments)])
        out.append(
            {
                # A closed loop's final junction sits at s=1, which IS s=0.
                # Reporting it as 0 keeps positions in [0, 1) so a consumer
                # never has to decide whether two ends of the same point are
                # two junctions.
                "s": 0.0 if last else travelled / total,
                "from_segment_index": index,
                "klass": measured["klass"],
                "tangent_angle_deg": measured["tangent_angle_deg"],
            }
        )
    return sorted(out, key=lambda j: j["s"])


def break_positions_s(curve: dict[str, Any], max_angle_deg: float) -> list[float]:
    """Normalized arc lengths of junctions turning by more than `max_angle_deg`."""
    return sorted(
        {j["s"] for j in junction_positions_s(curve) if j["tangent_angle_deg"] > max_angle_deg}
    )


def self_test() -> int:
    from seam_worker.cad.topology import extract_topology
    from seam_worker.seam.detect import detect_joints

    step = paths.PIPE_BRACKET
    if not step.exists():
        print(f"continuity self-test: missing fixture {step}", file=sys.stderr)
        return 1
    joints = detect_joints(extract_topology(step))
    counts = {"C0": 0, "G1": 0}
    for joint in joints["weld_joints"]:
        for measured in annotate_curve(joint["weld_line"]):
            counts[measured["klass"]] += 1
    assert counts["G1"] > 0, "pipe_bracket's rounded column must have tangent junctions"
    assert counts["C0"] > 0, "pipe_bracket's gusset rectangles must have corners"
    print(f"continuity self-test ok: {counts['C0']} corners, {counts['G1']} tangent junctions")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", default="")
    parser.add_argument("--joints", default="")
    parser.add_argument("--max-angle-deg", type=float, default=5.0)
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()

    if ns.joints:
        joints = json.loads(Path(ns.joints).read_text(encoding="utf-8"))
    elif ns.step:
        from seam_worker.cad.topology import extract_topology
        from seam_worker.seam.detect import detect_joints

        joints = detect_joints(extract_topology(Path(ns.step)))
    else:
        raise ValueError("one of --step, --joints or --self-test is required")

    report = []
    for joint in joints["weld_joints"]:
        report.append(
            {
                "weld_joint": joint["id"],
                "closed": joint["weld_line"]["closed"],
                "junctions": annotate_curve(joint["weld_line"]),
                "break_positions_s": break_positions_s(joint["weld_line"], ns.max_angle_deg),
            }
        )
    print(json.dumps({"schema": SCHEMA, "max_angle_deg": ns.max_angle_deg, "joints": report}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"continuity: {exc}", file=sys.stderr)
        raise SystemExit(1)
