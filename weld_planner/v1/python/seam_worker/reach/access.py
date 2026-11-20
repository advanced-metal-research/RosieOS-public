#!/usr/bin/env python
"""Stage 3: which stretches of a weld line no torch can reach.

Annotates detected joints with an `accessibility` block: blocked spans in
normalized arc length, plus `open_fraction` as a scalar function.

**Tooling-independent by construction.** A point is blocked when *no* direction
at all escapes the assembly -- it sits in a sealed pocket. That is a fact about
the parts. "Reachable with a 16 mm nozzle leaning 5 degrees" is a fact about a
particular torch, belongs to the planner, and deliberately does not appear
here. There is no nozzle radius, no reach, and no approach angle in this file.

The consequence is that this is *liberal*: a stretch stays weldable unless it is
completely occluded. Marginal clearance is reported through `open_fraction`, so
a planner that knows its own torch can threshold it however it likes, rather
than having someone else's torch assumptions baked in upstream.

Measured on `fixtures/pipe_bracket.step`, the separation is clean:

    gusset centre  (25, 0,10)     0 escaping directions -> blocked
    gusset edge    (25, 5,10)  24.9% of hemisphere open
    3 mm clear     (25, 8,10)  26.6%
    clear of gusset(25,20,10)  47.5%   (the quarter-space of a 90 deg fillet)

Self-contained. No imports from `cadquery/v1/` or `tesseract/v1/`.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

from seam_worker import paths
from seam_worker.cad.topology import edge_geom_key, load_step, sorted_solids

SCHEMA = "amr-weld-planner-v1.joints.v1"

# Directions tested per sample. A Fibonacci hemisphere: even coverage without
# the pole clustering of a lat/long grid.
DEFAULT_RAY_COUNT = 256

# Most sampled points are wide open -- 25 to 47 percent of the hemisphere
# escapes on a 90 degree fillet -- so casting all 256 rays at every one of them
# is almost entirely wasted work. Cast every Nth direction first; escalate to
# the full set only when the coarse pass finds NOTHING, which is exactly the
# case where the extra confidence is needed. Correct by construction: the
# coarse set is a subset, so a point that escapes coarsely escapes fully, and a
# point that fails coarsely is still checked against all 256 before being
# called blocked.
COARSE_STRIDE = 8

# Arc-length spacing of samples along the weld line, in source units (mm).
DEFAULT_SAMPLE_STEP = 2.0

# Ignore intersections this close to the origin: the sample sits ON the two
# groove faces, so every ray would otherwise register an immediate hit.
SURFACE_SKIP = 0.05

INTERSECT_TOL = 1e-6

# Which intersector answers "does this ray escape". `bvh` is indexed triangles
# (`occlusion.py`), `occt` is the exact-surface oracle it replaced.
#
# The default changed because the exact path made this stage the slowest thing in
# the pipeline for no accuracy that survived measurement: on an exhaust manifold
# one pipe bore took 43.6 s through OCCT and 24.9 ms through the BVH, and across
# both joints measured the two agreed on the blocked verdict at every single
# sample (63/63 and 53/53), with `min_open_fraction` identical at the coarse ray
# count. `occt` stays selectable so that comparison stays runnable.
DEFAULT_ENGINE = "bvh"


def hemisphere(count: int, axis: tuple[float, float, float]) -> list[tuple[float, float, float]]:
    """Evenly spread unit directions in the hemisphere about `axis`."""
    golden = math.pi * (3.0 - math.sqrt(5.0))
    length = math.sqrt(sum(c * c for c in axis)) or 1.0
    ax = (axis[0] / length, axis[1] / length, axis[2] / length)

    # Build an orthonormal basis with `ax` as the pole.
    helper = (1.0, 0.0, 0.0) if abs(ax[0]) < 0.9 else (0.0, 1.0, 0.0)
    u = (
        ax[1] * helper[2] - ax[2] * helper[1],
        ax[2] * helper[0] - ax[0] * helper[2],
        ax[0] * helper[1] - ax[1] * helper[0],
    )
    un = math.sqrt(sum(c * c for c in u)) or 1.0
    u = (u[0] / un, u[1] / un, u[2] / un)
    v = (
        ax[1] * u[2] - ax[2] * u[1],
        ax[2] * u[0] - ax[0] * u[2],
        ax[0] * u[1] - ax[1] * u[0],
    )

    out = []
    for i in range(count):
        z = 1.0 - (i + 0.5) / count
        r = math.sqrt(max(0.0, 1.0 - z * z))
        theta = golden * i
        cu, cv = r * math.cos(theta), r * math.sin(theta)
        out.append(
            (
                ax[0] * z + u[0] * cu + v[0] * cv,
                ax[1] * z + u[1] * cu + v[1] * cv,
                ax[2] * z + u[2] * cu + v[2] * cv,
            )
        )
    return out


def _aim(inter: Any, origin: Any, direction: Any) -> None:
    """Point the loaded intersector along a line from `origin`.

    The curve-only `Init` takes an adaptor, so the line is built here; the
    three-argument form is the one that accepts a raw `gp_Lin`, and it reloads
    the shape on every call. Loading once and aiming per ray measures 12%
    faster (197 -> 173 us/ray on pipe_bracket).
    """
    from OCP.Geom import Geom_Line
    from OCP.GeomAdaptor import GeomAdaptor_Curve
    from OCP.gp import gp_Dir, gp_Lin, gp_Pnt

    inter.Init(GeomAdaptor_Curve(Geom_Line(gp_Lin(gp_Pnt(*origin), gp_Dir(*direction)))))


def any_hit(
    inter: Any, origin: Any, direction: Any, lo: float = SURFACE_SKIP, hi: float | None = None
) -> bool:
    """Is there an intersection at a distance in (lo, hi]? Stops at the first.

    Short-circuiting matters more than it looks: a line through this assembly
    can cross a dozen faces, and walking all of them to answer a yes/no question
    made accessibility 44% slower when it was first written that way.
    """
    _aim(inter, origin, direction)
    while inter.More():
        w = inter.W()
        if w > lo and (hi is None or w <= hi):
            return True
        inter.Next()
    return False


def hits_along(
    inter: Any, origin: Any, direction: Any, lo: float = SURFACE_SKIP, hi: float | None = None
) -> list[float]:
    """Every intersection distance in (lo, hi], sorted. Cannot short-circuit.

    The distance is the quantity a torch needs and a hemisphere does not. Asking
    "does anything escape" only ever needed a boolean -- but a cylinder standing
    off the work has to know *where* a hit is: a face 4 mm out is a collision
    for a body that starts at 10 mm only if the body reaches it, and a face
    300 mm out is nothing at all.

    Keeping the whole list rather than the nearest hit is what makes standoff
    free to search over: moving the torch back slides an interval along a line
    that was already cast. See `clearance.py`.

    No early exit is possible here, because OCCT does not return intersections
    in order of distance -- which is also why the result is sorted rather than
    assumed.
    """
    _aim(inter, origin, direction)
    out: list[float] = []
    while inter.More():
        w = inter.W()
        if w > lo and (hi is None or w <= hi):
            out.append(w)
        inter.Next()
    out.sort()
    return out


def _escapes(inter: Any, origin: Any, direction: Any) -> bool:
    """Nothing in the way, to any distance. The reachability question.

    The line is infinite in both directions, so hits behind the origin have
    negative `W` and are dropped by the same skip that drops the groove faces
    the sample sits on.
    """
    return not any_hit(inter, origin, direction)


class _OcctEngine:
    """The escape test against exact surfaces. The oracle, not the default.

    Kept reachable rather than deleted: `occlusion.py` is a faster answer to the
    same question, and "faster answer to the same question" is a claim that has
    to stay testable. `tests/programmer/test_occlusion.py` runs both.
    """

    name = "occt"

    def __init__(self, shape: Any):
        from OCP.BRepIntCurveSurface import BRepIntCurveSurface_Inter

        # Load the shape ONCE. `Init(shape, line, tol)` reloads it on every ray,
        # which is 22% of the cost of a ray (measured 158 -> 123 us).
        self._inter = BRepIntCurveSurface_Inter()
        self._inter.Load(shape.wrapped, INTERSECT_TOL)

    @property
    def provenance(self) -> dict[str, Any]:
        return {"engine": self.name}

    def escaping(self, origin: tuple[float, float, float], dirs: Any) -> int:
        return sum(1 for d in dirs if _escapes(self._inter, origin, d))

    def any_escape(self, origin: tuple[float, float, float], dirs: Any) -> bool:
        return any(_escapes(self._inter, origin, d) for d in dirs)


def make_engine(shape: Any, engine: str, key: Any = None, **index_kwargs: Any) -> Any:
    """The intersector for this run: indexed triangles by default.

    `bvh` is the default because it answers the same question about 1500 times
    faster -- 25 ms against 43 s on one pipe bore -- and agrees with `occt` on
    every blocked verdict measured. `occt` remains selectable so the oracle can
    be run rather than cited.

    `index_kwargs` are the tessellation tolerances, `None` meaning "the shipped
    default". They mean nothing to the exact intersector, which has no
    tessellation, so asking for one with `occt` is refused rather than ignored --
    a tolerance that silently did nothing is the quiet failure this module is
    built against.
    """
    wanted = {k: v for k, v in index_kwargs.items() if v is not None}
    if engine == "occt":
        if wanted:
            raise ValueError(
                f"the occt engine has no tessellation, so {sorted(wanted)} cannot apply to it"
            )
        return _OcctEngine(shape)
    if engine != "bvh":
        raise ValueError(f"unknown engine {engine!r}; expected 'bvh' or 'occt'")

    from seam_worker.cad import occlusion

    return occlusion.Engine(
        occlusion.index_for(shape, key, **wanted),
        SURFACE_SKIP,
    )


def _occt_edges_by_key(shape: Any) -> dict[str, Any]:
    """geom_key -> OCCT edge, using the same key function topology.py assigns.

    Mapping by content rather than replicating traversal order is the same
    approach tessellate.py takes, and for the same reason: a second ordering
    implementation is a second place to drift.
    """
    out: dict[str, Any] = {}
    for index, solid in enumerate(sorted_solids(shape)):
        solid_id = f"solid_{index}"
        for edge in solid.Edges():
            out.setdefault(edge_geom_key(solid_id, edge), edge)
    return out


def _to_point(occt_point: Any, xyz: list[float]) -> float:
    """Squared distance from an OCCT point to a plain triple, same units.

    Only ever compared against another of itself, so the square is never taken
    back out.
    """
    return (
        (occt_point.X() - xyz[0]) ** 2
        + (occt_point.Y() - xyz[1]) ** 2
        + (occt_point.Z() - xyz[2]) ** 2
    )


def _sample_weld_line(
    edges_by_key: dict[str, Any],
    key_by_edge_id: dict[str, str],
    joint: dict[str, Any],
    step_mm: float,
) -> list[tuple[tuple[float, float, float], float]]:
    """(point, normalized arc length, segment index) along the weld line.

    Uniform in arc length. The segment index rides along because the groove
    bisector -- and therefore where a torch would come from -- rotates from one
    segment to the next, and a single pole for the whole joint tests the wrong
    half of the sky on the far ones.

    Sampled from the ORIGINAL OCCT edges rather than by evaluating the emitted
    B-splines: the curve evaluator does not exist yet, and going back to the
    source is exact on arcs instead of approximating them by a control polygon.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.GCPnts import GCPnts_UniformAbscissa

    segments = joint["weld_line"]["segments"]
    total = sum(segment["length_m"] for segment in segments)
    if total <= 0.0:
        return []

    samples: list[tuple[tuple[float, float, float], float, int]] = []
    travelled = 0.0
    for index, segment in enumerate(segments):
        edge = edges_by_key.get(key_by_edge_id.get(segment.get("source_edge_id", ""), ""))
        if edge is None:
            travelled += segment["length_m"]
            continue
        adaptor = BRepAdaptor_Curve(edge.wrapped)
        divider = GCPnts_UniformAbscissa(adaptor, step_mm)
        count = divider.NbPoints() if divider.IsDone() else 0
        if count < 2:
            parameters = [adaptor.FirstParameter(), adaptor.LastParameter()]
        else:
            parameters = [divider.Parameter(i) for i in range(1, count + 1)]

        # WALK THE EDGE THE WAY THE WELD LINE DOES.
        #
        # `chain_edges` orders a joint's edges head to tail and REVERSES the
        # ones that need it, so the emitted segment runs whichever way the loop
        # does. `BRepAdaptor_Curve` knows nothing about that: it walks the
        # ORIGINAL edge in its own parameter direction. Sampling without
        # checking therefore records every flag on a reversed segment at a
        # MIRRORED `s`, and nothing downstream can tell.
        #
        # It hid because the boundary bisection is right. `_blocked_at`
        # evaluates the emitted CURVE, so a transition found between two
        # mirrored samples is then refined at true positions -- the boundary
        # settles between the mirrored guess and the truth, which reads as a
        # plausible blocked span slightly in the wrong place rather than as
        # nonsense.
        #
        # Measured on `bracket_b` before this: over half the samples on every
        # fixture landed in the wrong place, by up to 110 mm. A gusset's 10 mm
        # footprint came back as 7.9 mm shifted 2.5 mm along the weld, so
        # `seam_0005` was offered running 4.6 mm underneath the gusset standing
        # on it -- a seam no torch could reach, presented as weldable.
        declared = (segment.get("analytic") or {}).get("start_xyz_m") or (
            segment.get("control_points_xyz_m") or [None]
        )[0]
        if declared is not None and len(parameters) > 1:
            want = [float(c) * 1000.0 for c in declared]
            head = adaptor.Value(parameters[0])
            tail = adaptor.Value(parameters[-1])
            if _to_point(tail, want) < _to_point(head, want):
                parameters.reverse()

        for i, parameter in enumerate(parameters):
            point = adaptor.Value(parameter)
            along = segment["length_m"] * i / max(1, len(parameters) - 1)
            samples.append(
                ((point.X(), point.Y(), point.Z()), (travelled + along) / total, index)
            )
        travelled += segment["length_m"]
    return samples


# Bisections used to refine each blocked/clear transition. Five halvings of a
# 2 mm sample step locate a boundary to about 0.06 mm, for five extra ray fans
# per boundary -- and there are only a handful of boundaries per joint.
BOUNDARY_BISECTIONS = 5

# A reachable run shorter than this is not a measurement, it is the corner being
# treated as a point. At the exact corner of a blocked edge a ray can still slip
# out diagonally -- 32 of 256 directions escape at 0.00 mm, decaying to 0 by
# 0.05 mm -- which left a 0.03 mm "weldable" sliver at each end of every blocked
# stretch and made position 0 read as reachable while 0.03 to 9.97 read blocked.
#
# This is a RESOLUTION floor, not a process minimum. "A run under 5 mm is not
# worth welding" is a judgement about a torch and a procedure, and would not
# belong here; "a run under half a millimetre is below what this method
# resolves" is a statement about the method. Planners get `reachable_runs` with
# lengths and can apply their own process minimum on top.
MIN_RESOLVED_RUN_M = 0.0005


def _runs(flags: list[tuple[float, bool]]) -> list[tuple[int, int]]:
    """Index ranges (inclusive) of contiguous blocked samples."""
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for i, (_, blocked) in enumerate(flags):
        if blocked and start is None:
            start = i
        elif not blocked and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(flags) - 1))
    return runs


def _weld_curve(joint: dict[str, Any]) -> Any:
    """The analytic weld line, built ONCE per joint.

    Bisection is only worth doing if the midpoint it tests is the real midpoint,
    so refinement evaluates the curve rather than interpolating between samples.
    Constructing a `Curve` per bisection step made accessibility five times
    slower than the sampling it was refining -- each construction rebuilds the
    arc-length lookup table for every segment.
    """
    from seam_worker.seam.curve import Curve

    try:
        return Curve(joint["weld_line"])
    except Exception:  # noqa: BLE001 - a malformed curve is not fatal here
        return None


def _spans(
    flags: list[tuple[float, bool]],
    closed: bool = False,
    refine: Any = None,
) -> list[list[float]]:
    """Blocked runs as [start_s, end_s], with boundaries refined by bisection.

    Sampling alone gets this measurably wrong. A run is detected from its first
    BLOCKED sample to its first CLEAR sample, which opens the span late and
    closes it late -- the whole thing shifts forward by about one sample step.
    On the 10 mm gusset-to-column edge of pipe_bracket, whose true blocked extent
    is (0, 10) mm, 2 mm sampling reported [2, 10] mm: a 20% error at the leading
    edge, and precisely the end an operator notices because it is where the
    weld appears to start.

    `refine(a, b)` is asked to locate the transition between a clear `a` and a
    blocked `b`; without it the midpoint is used, which is at least symmetric.

    Closed curves get their wrap-around run merged: a stretch straddling s=0
    would otherwise be reported as two separate spans, one at each end.
    """
    if not flags:
        return []
    runs = _runs(flags)
    if not runs:
        return []

    def boundary(clear_index: int, blocked_index: int) -> float:
        if clear_index < 0 or clear_index >= len(flags):
            return flags[blocked_index][0]
        a, b = flags[clear_index][0], flags[blocked_index][0]
        return refine(a, b) if refine else 0.5 * (a + b)

    spans: list[list[float]] = []
    for first, last in runs:
        lo = 0.0 if first == 0 else boundary(first - 1, first)
        hi = 1.0 if last == len(flags) - 1 else boundary(last + 1, last)
        spans.append([lo, hi])

    # wrap-around on a closed loop: the run touching 1.0 continues into the one
    # touching 0.0, so they are one physical stretch, not two
    if (
        closed
        and len(spans) > 1
        and spans[0][0] <= 1e-9
        and spans[-1][1] >= 1.0 - 1e-9
    ):
        merged = [spans[-1][0], spans[0][1] + 1.0]
        spans = spans[1:-1] + [[round(merged[0], 6), 1.0], [0.0, round(spans[0][1], 6)]]
        spans.sort()
        return [[round(lo, 6), round(hi, 6)] for lo, hi in spans]

    return [[round(lo, 6), round(hi, 6)] for lo, hi in spans]


def _clear_runs(spans: list[list[float]], closed: bool) -> list[list[float]]:
    """Reachable runs, as [start_s, end_s]. The complement of the blocked spans.

    On a closed loop a run through s=0 is ONE run; it is reported as the pair
    that wraps, with start > end, so a consumer can tell.
    """
    runs: list[list[float]] = []
    cursor = 0.0
    for lo, hi in sorted(spans):
        if lo > cursor:
            runs.append([cursor, lo])
        cursor = max(cursor, hi)
    if cursor < 1.0:
        runs.append([cursor, 1.0])
    if not runs:
        return []
    if closed and len(runs) > 1 and runs[0][0] <= 1e-9 and runs[-1][1] >= 1.0 - 1e-9:
        wrapped = [runs[-1][0], runs[0][1]]
        runs = runs[1:-1] + [wrapped]
    # NOT ROUNDED, and that is the point.
    #
    # These are NORMALIZED positions, so a decimal here is a decimal of the
    # whole weld line -- six of them quantize a 190 mm curve to 0.19 um. Worse
    # than the coarseness is the DIRECTION: a boundary rounded down past a
    # corner hands the seam its first micrometre on the far face of a 90 degree
    # turn, and `_snap_to_corners` above exists precisely to put these
    # boundaries ON corners. Rounding afterwards took them straight back off,
    # and twelve decimals still landed 5e-14 short.
    #
    # Rounding was for byte-stable plan requests. float64 round-trips exactly
    # through `json.dumps`, and everything upstream is deterministic from the
    # STEP, so the bytes are stable without it. See docs/issues/0001.
    return [[a, b] for a, b in runs]


def _run_length(run: list[float], total: float) -> float:
    lo, hi = run
    span = (hi - lo) if hi >= lo else (1.0 - lo + hi)
    return span * total


def _snap_to_corners(
    spans: list[list[float]], weld_line: dict[str, Any], total_m: float,
    step_mm: float = 0.0,
) -> list[list[float]]:
    """Move a bisected boundary onto a corner it landed within its own error of.

    Not continuity deciding reachability -- the order the docs insist on is
    kept. This is a MEASUREMENT reporting the exact value when it lands inside
    its own error bar of one. A corner is summed from declared segment lengths
    and is exact; a blocked-span boundary is bisected here and is good to about
    0.06 mm. Where a blocked stretch begins at a corner, as it usually does
    because the geometry that blocks a torch is usually the part that makes the
    corner, the two disagree by a hair and the hair falls on whichever side the
    last halving stopped.

    Left alone, that hair is fatal downstream. `bracket_a`'s first reachable
    run began 27 um past its corner, `seams._cut_run` made a 27 um stub of it,
    `_absorb_short_pieces` merged the stub forward ACROSS the corner, and the
    seam that came out started on the far face of a 90 degree turn. A motion
    planner sampling it is asked to swing the electrode 60 degrees between two
    samples 2.6 mm apart, which no arm does. Five of the fifteen seams in the
    committed corpus were in that state and none of them could be planned.
    See `docs/issues/0001`.

    Snapping HERE and not in `seams.py` is what keeps the books. The blocked
    length and the reachable runs are both derived from these spans, so moving
    a boundary moves both and `weldable + blocked == the whole boundary` still
    holds exactly. Snapped in intent generation instead, the seam would shrink
    while the blocked total stayed put.
    """
    from seam_worker.seam.continuity import junction_positions_s

    if not spans:
        return spans
    corners = [j["s"] for j in junction_positions_s(weld_line)]
    if not corners or total_m <= 0:
        return spans
    # The precision this module claims for itself, one line down from the
    # constant that sets it: five halvings of the sample step.
    snap = ((step_mm or DEFAULT_SAMPLE_STEP) / 1000.0 / 2 ** BOUNDARY_BISECTIONS) / total_m
    out = []
    for lo, hi in spans:
        for corner in corners:
            if abs(corner - lo) <= snap:
                lo = corner
            if abs(corner - hi) <= snap:
                hi = corner
        out.append([lo, hi])
    return out


def _absorb_slivers(
    spans: list[list[float]], closed: bool, total: float, min_run_m: float | None = None
) -> list[list[float]]:
    """Merge blocked spans separated by a run shorter than `min_run_m`.

    Without this, every blocked stretch is bracketed by a 0.03 mm reachable
    sliver at each corner, which is what made s=0 report clear on an edge that
    is entirely unweldable.

    The floor is a parameter because two different callers need two different
    ones, and conflating them is the mistake this module's own comment warns
    about. Here it defaults to `MIN_RESOLVED_RUN_M`, a statement about what ray
    casting resolves. `clearance.py` passes a process minimum instead -- "a 2 mm
    window between two collisions is not somewhere you strike an arc" -- which
    is a judgement about a torch and would not belong in this file.
    """
    floor = MIN_RESOLVED_RUN_M if min_run_m is None else min_run_m
    if len(spans) < 1:
        return spans
    ordered = sorted(spans)
    merged: list[list[float]] = [ordered[0]]
    for lo, hi in ordered[1:]:
        gap = (lo - merged[-1][1]) * total
        if gap <= floor:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    # a sliver at the very start or end of a closed loop joins the wrap
    if closed and merged:
        if merged[0][0] * total <= floor:
            merged[0][0] = 0.0
        if (1.0 - merged[-1][1]) * total <= floor:
            merged[-1][1] = 1.0
    return [[round(lo, 6), round(hi, 6)] for lo, hi in merged]


def _longest_clear_run(spans: list[list[float]], closed: bool) -> float:
    """Where a closed loop should start welding.

    `chain_edges` begins a loop at whatever edge came first out of the face's
    exploration order, which is arbitrary and can land the start inside an
    unreachable pocket -- confusing to look at and a poor default for a
    planner. This reports the beginning of the longest reachable run instead.
    Advisory: the seam's own `span` and `direction` still decide.
    """
    if not spans:
        return 0.0
    clear: list[tuple[float, float]] = []
    cursor = 0.0
    for lo, hi in sorted(spans):
        if lo > cursor:
            clear.append((cursor, lo))
        cursor = max(cursor, hi)
    if cursor < 1.0:
        clear.append((cursor, 1.0))
    if not clear:
        return 0.0
    if closed and len(clear) > 1 and clear[0][0] <= 1e-9 and clear[-1][1] >= 1.0 - 1e-9:
        # the run through s=0 is one run, not two
        wrapped = (clear[-1][0], clear[0][1] + 1.0)
        clear = clear[1:-1] + [wrapped]
    # Rounded OUTWARD, away from the blocked span this run begins after.
    #
    # Plain `round` puts the start inside that span whenever the true boundary
    # needs more than six decimals: a run beginning at exactly 1/12 rounds to
    # 0.083333, which is less than 0.0833333..., and the advertised start is
    # then a position the same document calls unreachable. Harmless before
    # `_snap_to_corners` only because the boundary was never exact.
    start = max(clear, key=lambda r: r[1] - r[0])[0] % 1.0
    return min(math.ceil(start * 1e6) / 1e6, 1.0)


def engine_directions(directions: list[tuple[float, float, float]], engine: Any) -> Any:
    """Directions in the form this engine wants, sliceable either way.

    The BVH kernels take a contiguous float64 array; the OCCT path takes tuples
    it can hand to `gp_Dir`. Both have to survive `[::COARSE_STRIDE]`, which is
    why this returns an array rather than a generator.
    """
    if getattr(engine, "name", "") == "bvh":
        from seam_worker.cad import occlusion

        return occlusion.as_array(directions)
    return directions


def annotate_joint(
    shape: Any,
    edges_by_key: dict[str, Any],
    key_by_edge_id: dict[str, str],
    joint: dict[str, Any],
    ray_count: int,
    step_mm: float,
    engine: Any = None,
) -> dict[str, Any]:
    """Accessibility for one joint, against whichever intersector it is given.

    `engine` is an object with `escaping` and `any_escape`; `make_engine` builds
    either. The algorithm below does not know which it has, which is the point:
    swapping the intersector must not be able to change the statistics, and
    `tests/programmer/test_occlusion.py` checks that it does not.
    """
    if engine is None:
        engine = make_engine(shape, DEFAULT_ENGINE)
    samples = _sample_weld_line(edges_by_key, key_by_edge_id, joint, step_mm)
    if not samples:
        return {
            "method": "hemisphere_escape",
            "intersector": engine.provenance,
            "ray_count": ray_count,
            "samples": 0,
            "blocked_spans": [],
            "open_fraction": {"kind": "constant", "value": 1.0},
            "note": "no source edges resolved; treated as accessible",
        }

    # Rays are spread about the groove bisector, which is where a torch would
    # come from. Any direction may still escape -- the pole only sets the
    # sampling distribution, not the test.
    #
    # PER SEGMENT, not per joint. The bisector rotates along a weld line, and
    # this used to take `groove_faces[0]`'s for all of it. On the dog_box tees
    # and laps the later segments' bisectors are a full 90 degrees away, which
    # puts their escape directions on the equator of the sampled hemisphere --
    # the least densely covered place -- with roughly half of them outside the
    # sampled set altogether. The result is a false "blocked", and since seams
    # are now split at obstructions, a false blocked manufactures a split the
    # operator then has to argue with.
    #
    # `segment_index` comes off the groove face record, so this uses the same
    # data that produced the wrong pole rather than recomputing a bisector.
    poles: dict[int, tuple[float, float, float]] = {}
    for groove in joint.get("groove_faces") or []:
        a = groove["face_a"]["outward_normal_xyz"]
        b = groove["face_b"]["outward_normal_xyz"]
        candidate = (a[0] + b[0], a[1] + b[1], a[2] + b[2])
        if any(abs(c) > 1e-9 for c in candidate):
            poles[int(groove.get("segment_index", 0))] = candidate
    # A segment with no groove record falls back to the first one there is, and
    # to +Z if there are none. Both are worse than a real bisector; neither is
    # wrong, because the pole is a sampling choice and any direction may still
    # escape.
    fallback = next(iter(poles.values()), (0.0, 0.0, 1.0))

    # One hemisphere per SEGMENT, built once. There are a handful of segments
    # and hundreds of samples, so this is the difference between negligible and
    # dominating the run.
    fans: dict[int, tuple[Any, Any]] = {}

    def fan_for(index: int) -> tuple[Any, Any]:
        cached = fans.get(index)
        if cached is None:
            full = engine_directions(hemisphere(ray_count, poles.get(index, fallback)), engine)
            cached = (full, full[::COARSE_STRIDE])
            fans[index] = cached
        return cached

    # Cumulative normalized bounds, so the boundary bisection can find the
    # segment for an arbitrary s. Refining a boundary against the neighbouring
    # segment's pole would test a different sky either side of it, and the
    # bisection would converge on where the POLE changes rather than on where
    # the geometry does.
    lengths = [seg["length_m"] for seg in joint["weld_line"]["segments"]]
    span_total = sum(lengths) or 1.0
    edges_s: list[float] = []
    running = 0.0
    for length in lengths:
        running += length
        edges_s.append(running / span_total)

    def segment_at(s_value: float) -> int:
        for index, upper in enumerate(edges_s):
            if s_value <= upper + 1e-12:
                return index
        return max(0, len(edges_s) - 1)

    # Reported below. Every segment's fan holds the same COUNT of directions --
    # only their orientation differs -- so one of them is representative.
    coarse_ray_count = len(fan_for(0)[1])

    flags: list[tuple[float, bool]] = []
    knots: list[list[float]] = []
    worst = 1.0
    escalations = 0
    for coord, s, segment_index in samples:
        full, sparse = fan_for(segment_index)
        escaped = engine.escaping(coord, sparse)
        tested = len(sparse)
        if escaped == 0:
            # nothing escaped coarsely: confirm against every direction before
            # declaring a stretch unweldable
            escalations += 1
            escaped = engine.escaping(coord, full)
            tested = len(full)
        fraction = escaped / tested
        worst = min(worst, fraction)
        flags.append((s, escaped == 0))
        knots.append([round(s, 6), round(fraction, 6)])

    # Refine each transition by bisecting between the clear and blocked sample,
    # evaluating the same escape test at the midpoint. A handful of extra ray
    # fans buys boundaries good to ~0.06 mm instead of ~one sample step.
    weld_curve = _weld_curve(joint)

    def _blocked_at(s_value: float) -> bool:
        if weld_curve is None:
            return False
        clamped = min(max(s_value, 0.0), 1.0)
        point = weld_curve.point(clamped)
        coord = (float(point[0]) * 1000.0, float(point[1]) * 1000.0, float(point[2]) * 1000.0)
        full, sparse = fan_for(segment_at(clamped))
        if engine.any_escape(coord, sparse):
            return False
        return not engine.any_escape(coord, full)

    def refine(clear_s: float, blocked_s: float) -> float:
        lo, hi = clear_s, blocked_s
        for _ in range(BOUNDARY_BISECTIONS):
            mid = 0.5 * (lo + hi)
            if _blocked_at(mid):
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    closed = bool(joint["weld_line"].get("closed", False))
    total_m = joint["weld_line"]["length_m"]
    blocked = _snap_to_corners(
        _absorb_slivers(_spans(flags, closed=closed, refine=refine), closed, total_m),
        joint["weld_line"], total_m, step_mm,
    )
    blocked_length = sum(hi - lo for lo, hi in blocked) * total_m
    reachable = _clear_runs(blocked, closed)
    return {
        # The METHOD is hemisphere escape whichever intersector ran: the
        # algorithm, the statistics and the meaning of every field below are
        # identical, and only the ray/geometry test differs. That difference goes
        # in `intersector`, because a verdict computed against a 0.01 mm
        # tessellation must not be read back as if it were computed against the
        # exact surfaces.
        "method": "hemisphere_escape",
        "intersector": engine.provenance,
        "ray_count": ray_count,
        "coarse_ray_count": coarse_ray_count,
        "escalated_samples": escalations,
        "sample_step_mm": step_mm,
        "samples": len(samples),
        "blocked_spans": blocked,
        "blocked_length_m": round(blocked_length, 9),
        "min_open_fraction": round(worst, 6),
        # The stretches that ARE weldable, which is what a seam should cover.
        # A run with start > end wraps through s=0 on a closed loop.
        "reachable_runs": reachable,
        "reachable_lengths_m": [round(_run_length(r, total_m), 9) for r in reachable],
        # Where a closed loop should start welding. `chain_edges` starts a loop
        # at an arbitrary edge, which can put s=0 inside an unreachable pocket.
        "suggested_start_s": _longest_clear_run(blocked, closed),
        # Pure geometry, no torch in it: the fraction of directions that escape
        # the assembly. A planner thresholds this against its own tooling.
        "open_fraction": {"kind": "samples", "s": [k[0] for k in knots],
                          "values": [k[1] for k in knots], "interpolation": "linear"},
    }


def edge_key_map(topology_doc: dict[str, Any]) -> dict[str, str]:
    """The only thing this module needs from a topology document.

    Kept separate so a caller can farm joints out to a worker pool without
    pickling the whole topology once per joint -- on dog_box that is about a
    megabyte each time, for a map of ninety short strings.
    """
    return {e["id"]: e["geom_key"] for e in topology_doc["edges"]}


def annotate_one(
    step_path: Path,
    key_by_edge_id: dict[str, str],
    joint: dict[str, Any],
    ray_count: int = DEFAULT_RAY_COUNT,
    step_mm: float = DEFAULT_SAMPLE_STEP,
    engine: str = DEFAULT_ENGINE,
    deflection_mm: float | None = None,
    dilation_mm: float | None = None,
) -> dict[str, Any]:
    """Accessibility for a single joint. The unit of parallelism, and the unit
    of progress the operator sees.

    Keyed on the STEP path so the index is built once per worker process rather
    than once per joint -- see `occlusion.index_for`. Ten joints farmed across a
    pool would otherwise pay the tessellation ten times and hand most of the
    speedup back.

    The tessellation arguments have to reach here rather than being read from
    settings inside: this runs in a pool worker, and a worker that resolved its
    own settings could answer one joint against a mesh the next joint was not
    measured against, from nothing more than a file being saved mid-run.
    """
    shape = load_step(step_path)
    return annotate_joint(
        shape,
        _occt_edges_by_key(shape),
        key_by_edge_id,
        joint,
        ray_count,
        step_mm,
        make_engine(
            shape,
            engine,
            key=str(step_path),
            deflection_mm=deflection_mm,
            dilation_mm=dilation_mm,
        ),
    )


def annotate(
    step_path: Path,
    topology_doc: dict[str, Any],
    joints_doc: dict[str, Any],
    ray_count: int = DEFAULT_RAY_COUNT,
    step_mm: float = DEFAULT_SAMPLE_STEP,
    engine: str = DEFAULT_ENGINE,
    deflection_mm: float | None = None,
    dilation_mm: float | None = None,
) -> dict[str, Any]:
    shape = load_step(step_path)
    edges_by_key = _occt_edges_by_key(shape)
    key_by_edge_id = edge_key_map(topology_doc)
    # One engine for every joint: the index is the expensive part and it does not
    # depend on the joint.
    built = make_engine(
        shape, engine, key=str(step_path), deflection_mm=deflection_mm, dilation_mm=dilation_mm
    )
    blocked_total = 0.0
    for joint in joints_doc.get("weld_joints", []):
        joint["accessibility"] = annotate_joint(
            shape, edges_by_key, key_by_edge_id, joint, ray_count, step_mm, built
        )
        blocked_total += joint["accessibility"].get("blocked_length_m", 0.0)
    joints_doc.setdefault("summary", {})["blocked_length_m"] = round(blocked_total, 9)
    joints_doc["summary"]["accessibility_ray_count"] = ray_count
    joints_doc["summary"]["accessibility_intersector"] = built.provenance
    return joints_doc


def repo_root() -> Path:
    """The repository this module sits inside. See `paths.py`."""
    return paths.REPO_ROOT


def self_test() -> int:
    from seam_worker.cad.topology import extract_topology
    from seam_worker.seam.detect import detect_joints

    step = paths.PIPE_BRACKET
    if not step.exists():
        print(f"access self-test: missing fixture {step}", file=sys.stderr)
        return 1
    topology_doc = extract_topology(step)
    doc = annotate(step, topology_doc, detect_joints(topology_doc))
    blocked = [j for j in doc["weld_joints"] if j["accessibility"]["blocked_spans"]]
    assert blocked, "pipe_bracket must have occluded weld line; the fixture exists for this"
    print(
        f"access self-test ok: {len(blocked)}/{len(doc['weld_joints'])} joints have blocked "
        f"spans, {doc['summary']['blocked_length_m'] * 1000:.1f} mm total unreachable"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", default="")
    parser.add_argument("--joints", default="", help="joints JSON; recomputed if absent")
    parser.add_argument("--out", default="")
    parser.add_argument("--rays", type=int, default=DEFAULT_RAY_COUNT)
    parser.add_argument("--sample-step-mm", type=float, default=DEFAULT_SAMPLE_STEP)
    parser.add_argument(
        "--engine",
        default=DEFAULT_ENGINE,
        choices=("bvh", "occt"),
        help="bvh: indexed triangles (default). occt: the exact-surface oracle, ~1500x slower",
    )
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)

    if ns.self_test:
        return self_test()
    if not ns.step:
        raise ValueError("--step is required unless --self-test is set")

    from seam_worker.cad.topology import extract_topology
    from seam_worker.seam.detect import detect_joints

    step = Path(ns.step)
    topology_doc = extract_topology(step)
    doc = (
        json.loads(Path(ns.joints).read_text(encoding="utf-8"))
        if ns.joints
        else detect_joints(topology_doc)
    )
    doc = annotate(step, topology_doc, doc, ns.rays, ns.sample_step_mm, ns.engine)

    if ns.out:
        out = Path(ns.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        print(f"wrote joints with accessibility: {ns.out}")
    if ns.summary or not ns.out:
        rows = {
            j["id"]: {
                "blocked_spans": j["accessibility"]["blocked_spans"],
                "blocked_mm": round(j["accessibility"]["blocked_length_m"] * 1000, 2),
                "min_open_fraction": j["accessibility"]["min_open_fraction"],
            }
            for j in doc["weld_joints"]
        }
        print(json.dumps(rows if ns.summary else doc, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"access: {exc}", file=sys.stderr)
        raise SystemExit(1)
