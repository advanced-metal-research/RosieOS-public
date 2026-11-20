#!/usr/bin/env python
"""Does THIS torch, at THESE angles, actually fit?

`access.py` deliberately refuses to answer that. It asks whether *any* direction
escapes the assembly -- a fact about the parts, with no nozzle in it -- and
leaves `open_fraction` for something downstream to threshold. This is that
something. It is the tooling-dependent half, and it lives here rather than in
`access.py` because the two answers must never be confused: one says "nobody can
weld this", the other says "you cannot weld this with that gun held like that".

The line the repository draws is around the CONTRACT, not the module. A
clearance verdict travels in the project bundle, beside the placement and the
torch that produced it; what reaches `weld-program.v1` is only what was always
allowed there -- the chosen angles and the chosen spans.

## The test

The torch is `tooling.py`'s cylinder: radius R, length L, starting a standoff d
out along the electrode axis. At each sample the seam frame gives `{t, r}`, the
angles give `u = apply_angles(t, r, work, travel)` pointing from the weld toward
the torch body, and the barrel is swept by a **parallel bundle of rays along u**
-- not a cone from the tip, because the body is a cylinder and a cone would
narrow exactly where the collision happens.

Origins lie on the disc of radius R about the tip, in the basis

    e1 = normalize(t - (t.u) u)     along the seam, perpendicular to the axis
    e2 = u x e1                     across the groove

tied to the seam frame rather than to world axes, so the fan does not spin
arbitrarily as the seam turns.

## Two ways a ray reports a collision

Each ray keeps its whole sorted hit list -- not a boolean, not the nearest hit
-- and a standoff `d` is read off it afterwards:

1. **A hit in `(d, d + L]`.** Something is inside the barrel.
2. **An odd number of hits beyond `d + L`.** The far field is definitely
   outside the solid, so an odd crossing count means the barrel's far END is
   buried. Without this a torch driven entirely inside a block thicker than
   150 mm reports clear, because it never crosses anything within its own
   length.

Note what case 1 does NOT include: material within `(0, d)`, the standoff gap.
That volume belongs to the arc and the wire, not to the body, and a fillet's own
two faces are always in it.

**Keeping the distances rather than a boolean is what makes standoff free to
search over.** Moving the torch back slides the interval `(d, d+L]` along lines
that were already cast, so seven candidate standoffs cost seven list scans and
zero new rays. That is why `access.hits_along` exists.

## Sanity, in closed form

For an infinite wedge of dihedral theta with the axis on the bisector, the ray
from the disc edge at offset b exits the material at exactly `w = |b| / tan(theta/2)`,
so a collision occurs iff `R / tan(theta/2) > d`, i.e. iff
`theta < 2 * atan(R / d)` -- `tooling.fits_dihedral_deg`, recovered by ray
casting. `tests/programmer/test_clearance.py` checks it against a synthetic corner so the
agreement does not depend on any fixture staying the same file.

## Two limitations, stated rather than discovered

- **A plate thinner than the fan spacing, standing edge-on between two rays, is
  missed.** dog_box has 3 mm plates; the fine fan's neighbour spacing is about
  2 mm, chosen against exactly that, and `test_tooling.py` pins it.
- **The seam frame is the one from `frames.py`**, so this inherits whatever
  `bisector_rail` gives. That is on purpose: building `r` a second time here
  would make a second definition of what a work angle means, and the frame
  parity tests exist to prevent precisely that.

## Cost, and where it went

Measured on pipe_bracket, per sample: a frame is 0.33 ms, a 65-ray fan is 0.196 ms
through the BVH and 10.0 ms through OCCT. The fan is still worth casting once
and reading every standoff off -- `SeamGeometry` is built once and handed to the
caller, and one frame table serves every angle the optimizer will try.

Both halves of that used to be an order of magnitude worse, and the two causes
were unrelated:

- **The rays went through OCCT.** `BRepIntCurveSurface_Inter` intersects the
  exact surface of every face with no spatial index: 154 us per ray here, 6.3 ms
  per ray on the exhaust manifold. They now go through `occlusion.py`'s triangle
  BVH, the same index `access.py` uses, at 3.0 us. One weld's angle lattice was
  400,554 rays and 56.6 seconds; it is 0.050 seconds.
- **A frame cost 4.2 ms, and 59% of a whole check was `Curve.__init__`.**
  Building a segment's arc-length table called `curve.de_boor` about 12,000
  times at interpreter overhead each. Batched through `de_boor_many` it is two
  array operations, and constructing this seam's two curves went from 568 ms to
  18 ms.

A clearance check on pipe_bracket's longest weld was 1.54 s and is 0.039 s. The
verdict did not move: `clear_fraction` and every blocked span agree with the
OCCT path, which `tests/programmer/test_clearance_engines.py` keeps runnable rather than
quoting. **`--engine occt` is still the oracle**, and it is the reason the
number above is a measurement rather than a hope.

This never runs as a pipeline stage. It depends on the torch and on the
authored profiles, neither of which exists at analyze time, and a stage would
want to write into `joints.json`, which the tooling-token test guards.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from itertools import pairwise
from pathlib import Path
from typing import Any

import numpy as np

from seam_worker import paths
from seam_worker.reach.access import (
    DEFAULT_ENGINE,
    _absorb_slivers,
    _clear_runs,
    _spans,
    hits_along,
)
from seam_worker.reach.tooling import (
    DEFAULT_TORCH,
    barrel_offsets,
    fits_dihedral_deg,
    weld_standoff_mm,
)
from seam_worker.seam.curve import Curve
from seam_worker.seam.frames import apply_angles, frame_at
from seam_worker.seam.scalar import evaluate
from seam_worker.seam.span import run_fraction, s_at, span_fraction, span_length_m

SCHEMA = "amr-weld-planner-v1.clearance.v1"

# The readout fan: 32 rays on the rim and 33 inside. Every point is within
# about 2 mm of a neighbour, which is what lets it see dog_box's 3 mm plates
# edge-on, and the rim ring reproduces `tooling.fits_dihedral_deg` on a
# synthetic groove to better than half a degree.
FINE_RAY_COUNT = 65

# The lattice fan for the optimizer, `barrel_offsets(R, 65)[::2]` -- 33 rays,
# 16 of them on the rim, which behaves like a barrel 0.15 mm narrow: about a
# degree optimistic on the fit angle, at half the cost.
#
# The nesting matters and its POLARITY is the opposite of `access.COARSE_STRIDE`.
# There, the coarse set proving ESCAPE is conclusive, because a subset that
# escapes escapes fully. Here a subset that COLLIDES collides fully -- so the
# coarse fan can only ever under-report collisions. It is therefore optimistic,
# usable for ranking lattice cells, and never sufficient on its own: the
# optimizer re-verifies its winning path at the fine fan before reporting a
# number to anyone.
COARSE_STRIDE = 2

# Arc-length spacing of samples along the seam, in millimetres. Matches
# `access.DEFAULT_SAMPLE_STEP`, so a clearance boundary and a reachability
# boundary are resolved at the same scale.
DEFAULT_SAMPLE_STEP_MM = 2.0

# Three halvings, not the five `access.py` uses. Each costs a frame -- the
# expensive part -- and a clearance boundary is genuinely fuzzy at the scale of
# the torch radius, so resolving it to 0.06 mm would be false precision.
BOUNDARY_BISECTIONS = 3

# A clear window shorter than this is not somewhere anyone strikes an arc. A
# PROCESS floor, deliberately equal to `seams.DEFAULT_MIN_SEAM_LENGTH_M` and
# deliberately NOT merged with `access.MIN_RESOLVED_RUN_M`, which is a
# statement about what ray casting resolves rather than about welding.
MIN_CLEAR_RUN_M = 0.005

# Frames come out of `curve.py` in metres; the OCCT shape is in source units,
# which for every STEP this project reads is millimetres. One conversion, named,
# in one place -- `access.py` spells the same factor inline and the conservation
# test in `test_clearance.py` is the tripwire for both.
MM_PER_M = 1000.0


class ClearanceError(ValueError):
    """A weld this check refuses to answer for, with the reason."""


# --------------------------------------------------------------------------
# the geometry of one weld, built once
# --------------------------------------------------------------------------


class SeamGeometry:
    """Frames along one weld, plus the ability to make more at arbitrary points.

    Built once and reused: frames cost 12x what a ray does, and they do not
    depend on the angles, so one of these serves every cell of the optimizer's
    lattice. That single fact is what makes searching over orientation
    affordable at all.

    Everything here is indexed by **fraction along the SEAM**, not `s` along the
    joint. Angle profiles are functions of fraction -- a profile drawn over one
    weld means nothing on its neighbour -- and `span.s_at` does the conversion.
    """

    def __init__(self, joint: dict[str, Any], seam: dict[str, Any]):
        direction = seam.get("direction", "forward")
        if direction not in ("forward", "reverse"):
            raise ClearanceError(
                f"seam {seam.get('id', '?')!r} has direction {direction!r}; "
                "expected 'forward' or 'reverse'"
            )
        # `reverse` negates `t`, and that is the whole of it -- the schema says
        # so in as many words: "Travel direction along the curve. Sets the sign
        # of the tangent t." No convention is being invented here, which is what
        # this used to refuse to do.
        #
        # Everything else follows from `apply_angles` without a special case,
        # and each consequence is the physical one:
        #
        #   r' = R(t, work) r     negating t reverses the SENSE of the work
        #                         roll, so +6 degrees leans toward the other
        #                         plate -- correct, because "lean left" depends
        #                         on which way you are facing.
        #   r  itself is UNCHANGED: it is built by projecting the rail
        #                         direction perpendicular to t, and
        #                         (d.(-t))(-t) == (d.t)t. The datum does not
        #                         move because you turned around, which is
        #                         exactly what a datum is for.
        #   b' = t x r'           flips, so the travel tilt leans the other way
        #                         in space -- and since travel direction also
        #                         flipped, +travel is still DRAG. The readout
        #                         needs no special case either.
        self.reverse = direction == "reverse"
        self.joint = joint
        self.seam = seam
        self.span = dict(seam["span"])
        self.curve = Curve(joint["weld_line"])
        rail_curve = (joint.get("bisector_rail") or {}).get("curve")
        self.rail = Curve(rail_curve) if rail_curve else None
        reference = seam.get("reference") or {}
        fixed = reference.get("fixed_direction_xyz") or reference.get("seed_direction_xyz")
        self.fixed = np.asarray(fixed, dtype=float) if fixed else np.array([0.0, 0.0, 1.0])
        self.length_m = span_length_m(self.span, self.curve.total)
        # Is this weld a CLOSED LOOP -- fraction 0 and fraction 1 the same
        # point on the part?
        #
        # True only when the joint's weld line closes AND this weld covers the
        # whole of it. A piece of a loop has two real ends; the whole loop has
        # none, and treating it as though it did puts an arc start wherever the
        # fraction origin happened to land. On dog_box's 821 mm dogbone that
        # showed up as a split into eight welds where seven boundaries were
        # real corners and the eighth was the seam's own numbering.
        self.closed = bool(joint.get("weld_line", {}).get("closed")) and (
            abs(span_fraction(self.span) - 1.0) < 1e-9
        )
        self.warnings: list[str] = []

    def fractions(self, step_mm: float = DEFAULT_SAMPLE_STEP_MM) -> list[float]:
        """Sample positions along the seam, uniform in arc length, ends included."""
        count = max(2, math.ceil(self.length_m * MM_PER_M / max(step_mm, 1e-6)) + 1)
        return [i / (count - 1) for i in range(count)]

    def frame(self, fraction: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """`(point in MILLIMETRES, t, r)` at a fraction along the seam.

        `t` is the TRAVEL tangent, so it is already negated on a reversed seam.
        One place, so nothing downstream has to remember.
        """
        point, t, r, warning = frame_at(
            self.curve, s_at(self.span, fraction), self.rail, self.fixed
        )
        if warning:
            self.warnings.append(warning)
        if self.reverse:
            t = -t
        return np.asarray(point, dtype=float) * MM_PER_M, t, r


# --------------------------------------------------------------------------
# the fan
# --------------------------------------------------------------------------


class _OcctRays:
    """Hit lists from the exact surfaces. The oracle, and the slow path.

    Loaded once and aimed per ray: `Init(shape, line, tol)` reloads the shape
    every time and costs 22% of a ray.

    Kept for the same reason `access._OcctEngine` is: the indexed intersector is
    a faster answer to a question OCCT already answers exactly, and a fast
    answer with no oracle to check it against is how a wrong one ships.
    Measured against the BVH, it is 150.8 us per ray on `pipe_bracket` and 6.3 ms
    per ray on the exhaust manifold, where the BVH is 1.3 us and 17.4 us.
    """

    name = "occt"

    def __init__(self, shape: Any):
        from OCP.BRepIntCurveSurface import BRepIntCurveSurface_Inter

        from seam_worker.reach.access import INTERSECT_TOL

        self._inter = BRepIntCurveSurface_Inter()
        self._inter.Load(shape.wrapped, INTERSECT_TOL)

    @property
    def provenance(self) -> dict[str, Any]:
        return {"engine": self.name}

    def hits_along(
        self,
        origin: tuple[float, float, float],
        direction: tuple[float, float, float],
        lo: float | None = None,
    ) -> list[float]:
        from seam_worker.reach.access import SURFACE_SKIP

        return hits_along(
            self._inter, origin, direction, SURFACE_SKIP if lo is None else lo
        )


def intersector(shape: Any) -> Any:
    """A ray caster with the shape already loaded, exact-surface.

    Still the OCCT one, so a caller that wants the oracle keeps getting it.
    `ray_engine` is what the measurement path uses.
    """
    return _OcctRays(shape)


def ray_engine(
    shape: Any,
    engine: str = DEFAULT_ENGINE,
    key: Any = None,
    deflection_mm: float | None = None,
    dilation_mm: float | None = None,
) -> Any:
    """The intersector the barrel test casts through.

    `bvh` indexes the triangles once and answers in microseconds; `occt`
    intersects the exact surfaces and is the oracle `tests/programmer/test_clearance.py`
    checks it against. Both expose `hits_along`, so `cast_fan` never learns
    which it has.

    The tessellation arguments are the same ones `access.make_engine` takes and
    mean the same thing, so `settings.intersector_of` can hand one block to
    both. Asking for them alongside `occt` is refused rather than ignored: the
    exact intersector has no mesh, and quietly accepting a deflection for it
    would let a settings file look like it had been applied when it had not.
    """
    if engine == "occt":
        wanted = {"deflection_mm": deflection_mm, "dilation_mm": dilation_mm}
        stated = sorted(k for k, v in wanted.items() if v is not None)
        if stated:
            raise ClearanceError(
                f"the occt engine has no tessellation, so {stated} cannot apply to it"
            )
        return _OcctRays(shape)
    if engine != "bvh":
        raise ClearanceError(f"unknown engine {engine!r}; expected 'bvh' or 'occt'")

    from seam_worker.cad import occlusion
    from seam_worker.reach.access import SURFACE_SKIP

    tolerances = {
        name: value
        for name, value in (
            ("deflection_mm", deflection_mm),
            ("dilation_mm", dilation_mm),
        )
        if value is not None
    }
    return occlusion.Engine(occlusion.index_for(shape, key, **tolerances), SURFACE_SKIP)


def barrel_axes(t: np.ndarray, u: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The disc basis `(e1, e2)` for a barrel pointing along `u`.

    `e1` is the tangent with its axial part removed, so the disc is oriented by
    the seam rather than by whichever world axis happened to be least parallel.
    A fan tied to a world axis rotates relative to the groove as the seam turns,
    and the collision it finds would then depend on how the part was modelled.

    Degenerates only when the torch points straight down the seam, which is a
    travel angle of 90 degrees; there any perpendicular will do.
    """
    axial = float(np.dot(t, u))
    e1 = np.asarray(t, dtype=float) - axial * np.asarray(u, dtype=float)
    norm = float(np.linalg.norm(e1))
    if norm < 1e-9:
        helper = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        e1 = helper - float(np.dot(helper, u)) * u
        norm = float(np.linalg.norm(e1)) or 1.0
    e1 = e1 / norm
    return e1, np.cross(u, e1)


def cast_fan(
    rays: Any,
    pos_mm: np.ndarray,
    t: np.ndarray,
    r: np.ndarray,
    work_deg: float,
    travel_deg: float,
    radius_mm: float,
    ray_count: int,
) -> list[list[float]]:
    """Hit distances for every ray of the barrel, at one sample.

    Unbounded on purpose. `BRepIntCurveSurface_Inter` is given an infinite line
    either way -- the `hi` argument of `hits_along` filters results, it does not
    limit the work -- so collecting every hit costs nothing extra and is what
    lets one cast answer for every standoff.
    """
    u = apply_angles(t, r, work_deg, travel_deg)
    e1, e2 = barrel_axes(t, u)
    direction = (float(u[0]), float(u[1]), float(u[2]))
    out: list[list[float]] = []
    for a, b in barrel_offsets(radius_mm, ray_count):
        origin = pos_mm + a * e1 + b * e2
        out.append(rays.hits_along((origin[0], origin[1], origin[2]), direction))
    return out


def fan_collides(fan: list[list[float]], standoff_mm: float, body_length_mm: float) -> bool:
    """Does the barrel at this standoff intersect anything?

    Two conditions, per ray, both explained in the module docstring: a hit
    inside the body, or an odd number of hits beyond its far end, which means
    the end is buried.
    """
    far = standoff_mm + body_length_mm
    for hits in fan:
        beyond = 0
        for w in hits:
            if standoff_mm < w <= far:
                return True
            if w > far:
                beyond += 1
        if beyond % 2 == 1:
            return True
    return False


def free_standoffs(
    fan: list[list[float]], body_length_mm: float, candidates: list[float]
) -> list[float]:
    """Which of these standoffs clear, on rays already cast. No new casting.

    The whole reason `hits_along` keeps the list. F5's outer loop over standoff
    is this function, and it is why standoff is an outer loop rather than a
    third lattice axis.
    """
    return [d for d in candidates if not fan_collides(fan, d, body_length_mm)]


# --------------------------------------------------------------------------
# the check
# --------------------------------------------------------------------------


def _angle_pair(seam: dict[str, Any], fraction: float) -> tuple[float, float]:
    orientation = seam.get("orientation") or {}
    work = orientation.get("work_angle_deg") or {"kind": "constant", "value": 0.0}
    travel = orientation.get("travel_angle_deg") or {"kind": "constant", "value": 0.0}
    return evaluate(work, fraction), evaluate(travel, fraction)


def _sealed_fractions(geometry: SeamGeometry, fractions: list[float]) -> list[list[float]]:
    """The stretches no torch could fix, mapped onto the seam's own axis.

    Read from the joint's existing `accessibility` block rather than measured
    again: those spans are the tooling-INDEPENDENT verdict, and re-deriving them
    here with a torch in hand would be a second answer to a question that
    already has one. Reported separately because they are a different colour on
    screen and a different decision for the operator -- no angle change and no
    optimizer run will ever recover them.

    Sealed implies blocked: if no direction at all escapes, no torch fits. So
    these are a SUBSET of `blocked_spans` in meaning, though not exactly in
    arithmetic -- the two are refined by different bisections at different
    depths, so their boundaries land a fraction of a millimetre apart.
    """
    from seam_worker.seam.span import contains_s

    blocked = ((geometry.joint.get("accessibility") or {}).get("blocked_spans")) or []
    if not blocked:
        return []
    flags = [
        (
            f,
            any(
                contains_s({"start_s": lo, "end_s": hi}, s_at(geometry.span, f))
                for lo, hi in blocked
            ),
        )
        for f in fractions
    ]
    return _spans(flags, closed=geometry.closed)


def collides_at(
    rays: Any,
    geometry: SeamGeometry,
    torch: dict[str, Any],
    standoff_mm: float,
    fraction: float,
    ray_count: int,
) -> bool:
    """Does the torch collide at one point of the seam, at the authored angles?

    The whole measurement, for one sample. Both the sampling pass and the
    boundary bisection go through here rather than each casting their own fan:
    a bisection that tested something subtly different from the samples it was
    refining would move a boundary for no reason anyone could find.
    """
    point, t, r = geometry.frame(fraction)
    work, travel = _angle_pair(geometry.seam, fraction)
    fan = cast_fan(
        rays, point, t, r, work, travel, float(torch["body_radius_mm"]), ray_count
    )
    return fan_collides(fan, standoff_mm, float(torch["body_length_mm"]))


def clearance(
    shape: Any,
    joint: dict[str, Any],
    seam: dict[str, Any],
    torch: dict[str, Any] | None = None,
    *,
    ray_count: int = FINE_RAY_COUNT,
    sample_step_mm: float = DEFAULT_SAMPLE_STEP_MM,
    min_clear_run_m: float = MIN_CLEAR_RUN_M,
    standoff_mm: float,
    engine: str = DEFAULT_ENGINE,
    key: Any = None,
    deflection_mm: float | None = None,
    dilation_mm: float | None = None,
) -> dict[str, Any]:
    """How much of this weld the torch can actually reach, at the authored angles.

    Everything in the result is in **seam fraction**, not joint `s`: the spans
    are stretches of THIS weld, and the operator's trim and split decisions are
    made on the same axis the angle profiles are drawn on.

    **`standoff_mm` is required.** It used to default to
    `weld_standoff_mm(seam, torch)`, which made a geometry function reach for a
    procedure value and gave the torch a say in a number the procedure owns.
    Resolving it is the caller's job -- `api/welds.py` has the settings and the
    preset library in hand and this does not -- and a measurement whose gap was
    filled in somewhere out of sight is the provenance failure R-UI-2 is about.
    """
    torch = torch or DEFAULT_TORCH
    geometry = SeamGeometry(joint, seam)
    standoff = float(standoff_mm)

    rays = ray_engine(shape, engine, key, deflection_mm, dilation_mm)

    def collides(fraction: float) -> bool:
        return collides_at(rays, geometry, torch, standoff, fraction, ray_count)

    fractions = geometry.fractions(sample_step_mm)
    flags = [(f, collides(f)) for f in fractions]

    def refine(clear_f: float, blocked_f: float) -> float:
        """Where a clear stretch ends, biased to the side that is proven clear.

        `_spans` always passes the clear sample first, so `lo` is the last
        fraction known not to collide and `hi` the first known to. Returning the
        MIDPOINT is the better estimator of the true transition and the wrong
        answer here: it is unbiased, so half the time it falls inside the
        blocked region, and this boundary is not a statistic -- it becomes the
        start or the stop of a weld, and an arc is struck at it.

        Measured on pipe_bracket before this: 9 of 36 boundaries left up to 0.10 mm
        of colliding arc inside a run the operator had just split off as clear.
        Three halvings of a 2 mm step leave a 0.25 mm bracket, so a midpoint sits
        up to 0.125 mm the wrong side of the truth -- which is exactly the size
        of stub that was showing up.

        So `lo`: the boundary goes at the last fraction actually proven clear.
        Being wrong now costs up to a quarter millimetre of seam left unwelded
        at each end, which is visible and harmless. The other direction was
        invisible and a collision.
        """
        lo, hi = clear_f, blocked_f
        for _ in range(BOUNDARY_BISECTIONS):
            mid = 0.5 * (lo + hi)
            if collides(mid):
                hi = mid
            else:
                lo = mid
        return lo

    # A PIECE of a joint has two ends even when the joint is a loop; the whole
    # loop has none. `geometry.closed` distinguishes them, and it matters all
    # the way out to the split button: on a closed weld a run ending at 1.0 and
    # one starting at 0.0 are the same stretch of metal, and offering them as
    # two welds puts an arc start and an arc stop at the fraction origin, which
    # is a number in a file rather than a place on the part.
    closed = geometry.closed
    length_m = geometry.length_m
    blocked = _absorb_slivers(
        _spans(flags, closed=closed, refine=refine),
        closed,
        length_m,
        min_run_m=min_clear_run_m,
    )
    # `_absorb_slivers` closes the gaps BETWEEN blocked spans. The two at the
    # ends are its blind spot on an OPEN seam, and they are the ones an operator
    # would actually see: a 2 mm weldable stub before the first obstruction is
    # not somewhere anyone strikes an arc. Absorbed rather than discarded, so
    # clear + blocked still sums to the seam exactly -- the conservation test is
    # the tripwire for the metres/millimetres confusion this file lives with.
    #
    # Skipped on a closed weld, where there are no end stubs to absorb: 0.0 and
    # 1.0 are interior points, and snapping a span out to them would invent an
    # obstruction at the join.
    if blocked and not closed:
        if blocked[0][0] * length_m < min_clear_run_m:
            blocked[0][0] = 0.0
        if (1.0 - blocked[-1][1]) * length_m < min_clear_run_m:
            blocked[-1][1] = 1.0
    clear = _clear_runs(blocked, closed)

    blocked_fraction = sum(run_fraction(lo, hi) for lo, hi in blocked)
    clear_fraction = sum(run_fraction(lo, hi) for lo, hi in clear)
    return {
        "schema": SCHEMA,
        "method": "barrel_fan",
        "seam_id": seam.get("id"),
        "weld_joint_ref": seam.get("weld_joint_ref"),
        # Whether fraction 0 and fraction 1 are the same point. Reported
        # because a consumer splitting on the clear runs has to know that a run
        # with start > end is one stretch and not an error.
        "closed": closed,
        # R-UI-2: a coverage number without the torch and standoff that produced
        # it is exactly the provenance failure the UI rules exist against.
        "torch": {k: torch[k] for k in ("id", "body_radius_mm", "body_length_mm") if k in torch},
        "standoff_mm": round(standoff, 6),
        # Quoted back from `tooling`, not recomputed here: the whole point of
        # the ray fan agreeing with the closed form is that there is one closed
        # form to agree with.
        "fits_dihedral_deg": round(fits_dihedral_deg(torch, standoff), 3),
        # Which intersector produced this, and at what tessellation. Same reason
        # `access.py` records it: a verdict measured against a mesh must not be
        # read back as if it came from the exact surfaces.
        "intersector": rays.provenance,
        "ray_count": ray_count,
        "sample_step_mm": sample_step_mm,
        "samples": len(fractions),
        "seam_length_m": round(length_m, 9),
        # Everything below is in FRACTION ALONG THE SEAM.
        "blocked_spans": blocked,
        "clear_runs": clear,
        "clear_fraction": round(clear_fraction, 6),
        "clear_length_m": round(clear_fraction * length_m, 9),
        "blocked_length_m": round(blocked_fraction * length_m, 9),
        # No angle and no optimizer recovers these; they are a different colour
        # and a different decision.
        "sealed_spans": _sealed_fractions(geometry, fractions),
        "min_clear_run_m": min_clear_run_m,
        "warnings": sorted(set(geometry.warnings)),
    }


# --------------------------------------------------------------------------
# the oracle the optimiser searches
# --------------------------------------------------------------------------


def decision_knots(
    geometry: SeamGeometry, knot_spacing_mm: float
) -> tuple[list[float], list[float]]:
    """`(knot fractions, mm between consecutive knots)` along the seam.

    Coarser than the oracle's own sampling, and deliberately so -- see the
    `orient` docstring. At the oracle's 2 mm a rate limit permits less than one
    bin of change per step, so the search freezes; angles are smooth and do not
    need a knot every 2 mm.
    """
    length_mm = geometry.length_m * MM_PER_M
    count = max(2, round(length_mm / max(knot_spacing_mm, 1e-6)) + 1)
    fractions = [i / (count - 1) for i in range(count)]
    gap = length_mm / (count - 1)
    return fractions, [gap] * (count - 1)


# How far the search may move an angle FROM THE PROFILE THAT WAS AUTHORED, when
# nothing narrower is declared. A deviation, not an absolute band: the window
# follows a varying profile instead of standing still, which is the whole
# difference between "the work angle may be anywhere in +/-15" and "you may
# move my ramp by up to 15 degrees".
DEFAULT_WORK_DEVIATION_DEG = 15.0
DEFAULT_TRAVEL_DEVIATION_DEG = 40.0


def _limit_series(
    limits: dict[str, Any] | None, key: str, fractions: list[float]
) -> list[float] | None:
    if not limits or limits.get(key) is None:
        return None
    return [evaluate(limits[key], f) for f in fractions]


def _axis_inputs(
    axis: str,
    orientation: dict[str, Any],
    fractions: list[float],
    deviation_deg: float,
    bin_deg: float,
    symmetric: bool = False,
) -> tuple[list[float], list[float], dict[str, Any] | None, list[str], bool]:
    """`(bins, authored at each knot, the mask, warnings, rate declared)`.

    Two bounds of different kinds, and the admissible set is their
    INTERSECTION:

    - `min`/`max` say where the angle may be at all. Absolute, and the weld's
      own procedure -- a hard mask.
    - `max_deviation_deg` says how far the search may move it from the profile
      that was AUTHORED. The window follows a varying profile instead of
      standing still, which is the whole difference between "work may be
      anywhere in +/-15" and "you may move my ramp by up to 15 degrees".

    They coincide only when the profile is constant and centred in its band,
    which is why one cannot stand in for the other.
    """
    from seam_worker.reach.orient import bin_values

    authored = [
        evaluate(orientation.get(axis) or {"kind": "constant", "value": 0.0}, f)
        for f in fractions
    ]
    declared = (orientation.get("limits") or {}).get(axis) or {}
    lows = _limit_series(declared, "min", fractions)
    highs = _limit_series(declared, "max", fractions)
    rate = declared.get("max_rate_deg_per_mm")

    # The weld's own deviation beats the settings default, the same way every
    # declared limit does. Split by DIRECTION when the document splits it --
    # dragging further is often free where pushing at all fouls the nozzle --
    # and the symmetric field is the shorthand for documents that see no
    # reason to distinguish. When either split field is present, the split
    # wins and the shorthand is ignored, per the schema.
    plus_stated = declared.get("max_deviation_plus_deg")
    minus_stated = declared.get("max_deviation_minus_deg")
    if plus_stated is not None or minus_stated is not None:
        plus = float(plus_stated if plus_stated is not None else deviation_deg)
        minus = float(minus_stated if minus_stated is not None else deviation_deg)
    else:
        stated = declared.get("max_deviation_deg")
        plus = minus = float(stated if stated is not None else deviation_deg)
    if plus < 0 or minus < 0:
        raise ClearanceError(
            f"deviation limits must not be negative, got +{plus} / -{minus}")

    warnings: list[str] = []

    # A declared limit is a hard mask, so an authored profile outside it is not
    # something to widen around -- it is a weld that does not satisfy its own
    # procedure, which `/api/validate` already reports. Refused here rather
    # than quietly moved, because moving it would be the optimiser silently
    # choosing which of the two the operator meant.
    for i, value in enumerate(authored):
        low = lows[i] if lows is not None else -math.inf
        high = highs[i] if highs is not None else math.inf
        if not low - 1e-9 <= value <= high + 1e-9:
            raise ClearanceError(
                f"the authored {axis.replace('_deg', '').replace('_', ' ')} is "
                f"{value:.2f}° at s={fractions[i]:.3f}, outside its declared limit of "
                f"[{low:.2f}, {high:.2f}]. Fix the profile or the limit before optimising."
            )

    # Never narrower than half a bin. A grid anchored at zero always has a
    # multiple within half a step of any value, so this is what guarantees the
    # authored profile itself is always reachable -- including at deviation 0,
    # which means "do not move it" and must not mean "no legal cell exists".
    # The floor applies PER SIDE: a document that pins one direction to zero
    # still gets the half-bin it needs to hold the authored value itself.
    reach_up = max(plus, bin_deg / 2.0)
    reach_down = max(minus, bin_deg / 2.0)
    window_lo = [
        max(a - reach_down, lows[i] if lows is not None else -math.inf)
        for i, a in enumerate(authored)
    ]
    window_hi = [
        min(a + reach_up, highs[i] if highs is not None else math.inf)
        for i, a in enumerate(authored)
    ]

    # Rounded OUTWARD to whole bins, so every window is bracketed rather than
    # merely touched.
    lo = math.floor(min(window_lo) / bin_deg) * bin_deg
    hi = math.ceil(max(window_hi) / bin_deg) * bin_deg
    if symmetric:
        # Reversing the travel direction negates `t`, and `apply_angles` turns
        # that into exactly `u(-t, w, tr) == u(t, -w, -tr)` -- verified to zero
        # error, not argued. So the reversed seam's clearance is the forward
        # table read at the negated angles, and no ray has to be cast twice.
        #
        # The price is that the table has to HOLD those negated angles. A window
        # around a drag angle of +10 is not symmetric, so the range is widened
        # to its own mirror image. On the shipped defaults that is 21 travel
        # bins instead of 17; nothing at all on work, whose window is already
        # centred on zero.
        span = max(abs(lo), abs(hi))
        lo, hi = -span, span
    mask: dict[str, Any] = {"min": window_lo, "max": window_hi}
    if rate is not None:
        mask["max_rate_deg_per_mm"] = rate

    bins = bin_values(lo, hi, bin_deg)
    # Counted over the ADMISSIBLE bins, not the bin range. The range is rounded
    # outward to whole bins and can hold three where the window admits one, so
    # measuring it would have quietly stopped reporting a frozen axis.
    widest = max(
        sum(1 for b in bins if window_lo[i] - 1e-9 <= b <= window_hi[i] + 1e-9)
        for i in range(len(authored))
    )
    if widest <= 1:
        warnings.append(
            f"the {axis} window admits a single {bin_deg}° bin, so this axis is "
            "fixed and the search cannot move it"
        )
    return bins, authored, mask, warnings, rate is not None


def search_inputs(
    seam: dict[str, Any],
    knot_fractions: list[float],
    *,
    work_deviation_deg: float = DEFAULT_WORK_DEVIATION_DEG,
    travel_deviation_deg: float = DEFAULT_TRAVEL_DEVIATION_DEG,
    work_bin_deg: float | None = None,
    travel_bin_deg: float | None = None,
    both_directions: bool = False,
) -> dict[str, Any]:
    """A seam document reduced to the numbers `orient` searches over.

    The split is deliberate: this module knows what a seam is, `orient` knows
    what a lattice is, and neither has to learn the other's vocabulary. It is
    the same reason `orient` has no OCCT in it -- the search is checkable
    against brute force precisely because it takes arrays, not documents.

    The authored profiles are evaluated at the DECISION knots, not at the oracle
    samples: they are what the search ties toward, and it only ever decides at a
    knot.

    `both_directions` widens each angle range to its own mirror image, so the
    same table also answers for the seam welded the other way round. See the
    note in `_axis_inputs`: reversing negates `t`, and that is exactly a
    negation of both angles, so it needs no new rays -- only bins that reach.
    """
    from seam_worker.reach.orient import (
        DEFAULT_TRAVEL_BIN_DEG,
        DEFAULT_WORK_BIN_DEG,
    )

    orientation = seam.get("orientation") or {}
    work_bins, authored_work, work_limits, warn_w, rate_w = _axis_inputs(
        "work_angle_deg", orientation, knot_fractions, work_deviation_deg,
        work_bin_deg or DEFAULT_WORK_BIN_DEG, both_directions,
    )
    travel_bins, authored_travel, travel_limits, warn_t, rate_t = _axis_inputs(
        "travel_angle_deg", orientation, knot_fractions, travel_deviation_deg,
        travel_bin_deg or DEFAULT_TRAVEL_BIN_DEG, both_directions,
    )

    # ONE sentence, however many axes are missing a rate limit.
    #
    # Not an error: nothing was declared, so nothing is violated. But the search
    # takes a free rate limit literally -- on pipe_bracket's column fillet it
    # returned a travel profile stepping 20 -> 10 degrees inside one 10 mm knot,
    # because the step bought arc and J3 only breaks ties. That is the right
    # answer to the question asked and the wrong profile to weld.
    #
    # Emitted once because it was emitted twice first, and four lines of amber
    # under a two-line result made the caveat louder than the answer.
    loose = [name for name, declared in (("work", rate_w), ("travel", rate_t)) if not declared]
    rate_warning = (
        [
            (
                f"no rate limit on the {' or '.join(loose)} angle, so the search "
                "may return a profile that changes faster than a torch can follow"
            )
        ]
        if loose
        else []
    )
    return {
        "work_bins": work_bins,
        "travel_bins": travel_bins,
        "authored_work": authored_work,
        "authored_travel": authored_travel,
        "work_limits": work_limits,
        "travel_limits": travel_limits,
        "warnings": rate_warning + warn_w + warn_t,
    }


def chunk_ranges(samples: int, chunks: int) -> list[tuple[int, int]]:
    """Split `[0, samples)` into at most `chunks` balanced, contiguous pieces.

    Contiguous rather than strided so each piece walks a connected stretch of
    the seam. It makes no difference to the arithmetic, but it means a chunk
    that fails names a place on the part rather than a scatter of samples.
    """
    if samples <= 0:
        return []
    chunks = max(1, min(chunks, samples))
    edges = [round(i * samples / chunks) for i in range(chunks + 1)]
    return [(lo, hi) for lo, hi in pairwise(edges) if hi > lo]


def build_lattice(
    shape: Any,
    joint: dict[str, Any],
    seam: dict[str, Any],
    torch: dict[str, Any] | None = None,
    *,
    work_bins: list[float],
    travel_bins: list[float],
    standoffs_mm: list[float],
    sample_step_mm: float = DEFAULT_SAMPLE_STEP_MM,
    knot_spacing_mm: float = 10.0,
    ray_count: int | None = None,
    index_range: tuple[int, int] | None = None,
    progress: Any = None,
    engine: str = DEFAULT_ENGINE,
    key: Any = None,
    deflection_mm: float | None = None,
    dilation_mm: float | None = None,
) -> dict[str, Any]:
    """Millimetres of clear arc for every (knot, work, travel, standoff).

    This is the expensive half of the optimiser and the only part that touches
    OCCT. Two economies make it affordable, and both come from measurements in
    `clearance.py`'s own docstring:

    **A frame costs 12x a ray, and does not depend on the angles.** So the frame
    at each oracle sample is computed ONCE and reused across every cell of the
    lattice. Recomputing it per cell would dominate the cost by an order of
    magnitude and buy nothing -- `{t, r}` is a property of the seam.

    **A cast answers for every standoff at once.** `hits_along` keeps the whole
    hit list, so moving the torch back just slides the interval `(d, d+L]` along
    lines already cast. Seven candidate standoffs are seven scans of a list, not
    seven sets of rays -- which is what lets standoff be an outer loop in the
    search instead of a third lattice axis.

    Returned as `clear_mm[standoff][knot][work][travel]`, ready to hand to
    `orient.optimise` one standoff slice at a time.

    **`index_range` casts only part of the seam**, so the work can be split
    across pool workers and summed with `merge_lattices`. The arithmetic allows
    it because every cell is a SUM over independent samples -- a sample
    contributes its arc to exactly one knot and knows nothing about its
    neighbours -- so partitioning the sample axis partitions the sums. Each
    chunk still computes the full sample and knot layout, and only the frames
    inside its own range, so nothing is computed twice.
    """
    torch = torch or DEFAULT_TORCH
    geometry = SeamGeometry(joint, seam)
    rays = ray_count or (FINE_RAY_COUNT // COARSE_STRIDE + 1)
    radius = float(torch["body_radius_mm"])
    body = float(torch["body_length_mm"])

    knots, spacing = decision_knots(geometry, knot_spacing_mm)
    fractions = geometry.fractions(sample_step_mm)
    length_mm = geometry.length_m * MM_PER_M

    # Arc each oracle sample speaks for, and which knot it belongs to. The ends
    # get half a step because they sit ON the boundary rather than inside it;
    # without that the seam's total comes out one step long and every coverage
    # percentage is quietly inflated.
    per = length_mm / max(1, len(fractions) - 1)
    share = [per if 0 < i < len(fractions) - 1 else per / 2 for i in range(len(fractions))]
    owner = [min(range(len(knots)), key=lambda k: abs(knots[k] - f)) for f in fractions]

    lo, hi = index_range or (0, len(fractions))
    if not 0 <= lo <= hi <= len(fractions):
        raise ClearanceError(
            f"index_range {(lo, hi)} is outside the {len(fractions)} samples of this seam"
        )

    caster = ray_engine(shape, engine, key, deflection_mm, dilation_mm)
    clear = [
        [[[0.0 for _ in travel_bins] for _ in work_bins] for _ in knots]
        for _ in standoffs_mm
    ]

    if hasattr(caster, "barrel_lattice"):
        # The whole chunk in one call. Frames still come from `frames.py` -- the
        # kernel is given `{P, t, r}` and applies the angles itself, so there is
        # still exactly one definition of what a work angle means.
        frames_pt = np.empty((hi - lo, 3))
        frames_t = np.empty((hi - lo, 3))
        frames_r = np.empty((hi - lo, 3))
        for i in range(lo, hi):
            frames_pt[i - lo], frames_t[i - lo], frames_r[i - lo] = geometry.frame(
                fractions[i]
            )
        if progress is not None:
            progress(hi - lo, hi - lo)
        per_sample = caster.barrel_lattice(
            frames_pt, frames_t, frames_r,
            np.asarray(work_bins, float), np.asarray(travel_bins, float),
            np.asarray(standoffs_mm, float),
            np.asarray(barrel_offsets(radius, rays), float),
            body,
        )
        # Fold the sample axis onto the decision knots. A sample contributes its
        # arc to exactly one knot, so this is a scatter-add and not a race --
        # which is why the kernel is free to run the samples in parallel.
        weighted = per_sample * np.asarray(share[lo:hi])[:, None, None, None]
        for i in range(lo, hi):
            k = owner[i]
            for d in range(len(standoffs_mm)):
                for a in range(len(work_bins)):
                    for b in range(len(travel_bins)):
                        clear[d][k][a][b] += weighted[i - lo, d, a, b]
    else:
        for i in range(lo, hi):
            # ONE frame, every cell.
            point, t, r = geometry.frame(fractions[i])
            k = owner[i]
            for a, w in enumerate(work_bins):
                for b, tv in enumerate(travel_bins):
                    fan = cast_fan(caster, point, t, r, w, tv, radius, rays)
                    # ONE set of casts, every standoff.
                    for d, standoff in enumerate(standoffs_mm):
                        if not fan_collides(fan, standoff, body):
                            clear[d][k][a][b] += share[i]
            if progress is not None:
                progress(i + 1 - lo, hi - lo)

    arc_mm = [0.0] * len(knots)
    for i in range(lo, hi):
        arc_mm[owner[i]] += share[i]

    return {
        "knot_fractions": knots,
        "spacing_mm": spacing,
        "arc_mm": arc_mm,
        "seam_length_mm": length_mm,
        "work_bins": list(work_bins),
        "travel_bins": list(travel_bins),
        "standoffs_mm": list(standoffs_mm),
        "clear_mm": clear,
        "ray_count": rays,
        "sample_step_mm": sample_step_mm,
        "samples": len(fractions),
        # The part of the sample axis this table actually speaks for. A whole
        # lattice says [0, samples]; `merge_lattices` checks the chunks tile it.
        "index_range": [lo, hi],
        # What it cost, because the next question after "how long did that take"
        # is always "how much of it was rays".
        "rays_cast": (hi - lo) * len(work_bins) * len(travel_bins) * rays,
        "warnings": sorted(set(geometry.warnings)),
    }


# Metadata every chunk of one lattice must agree on. Comparing these is what
# turns "the caller submitted mismatched work" from a silently wrong sum -- the
# worst kind, because every number still looks plausible -- into a refusal.
_LATTICE_INVARIANTS = (
    "knot_fractions",
    "spacing_mm",
    "seam_length_mm",
    "work_bins",
    "travel_bins",
    "standoffs_mm",
    "ray_count",
    "sample_step_mm",
    "samples",
)


def merge_lattices(parts: list[dict[str, Any]]) -> dict[str, Any]:
    """Chunks of one lattice, summed back into the whole.

    Every cell is a sum over samples and every chunk owns a disjoint stretch of
    them, so merging is addition -- there is no interaction term to get wrong.
    What there IS to get wrong is the partition, so the ranges are required to
    tile `[0, samples]` exactly: a gap would report a hole in the seam as
    "blocked everywhere" and an overlap would count arc twice and push coverage
    over 100%. Both are checked rather than trusted.
    """
    if not parts:
        raise ClearanceError("no lattice chunks to merge")
    head = parts[0]
    for other in parts[1:]:
        for key in _LATTICE_INVARIANTS:
            if other.get(key) != head.get(key):
                raise ClearanceError(
                    f"lattice chunks disagree on {key!r}: "
                    f"{head.get(key)!r} vs {other.get(key)!r}"
                )

    ranges = sorted(tuple(p["index_range"]) for p in parts)
    at = 0
    for lo, hi in ranges:
        if lo != at:
            raise ClearanceError(
                f"lattice chunks do not tile the sample axis: expected the next "
                f"chunk to start at {at}, got {lo}"
            )
        at = hi
    if at != head["samples"]:
        raise ClearanceError(
            f"lattice chunks cover {at} of {head['samples']} samples"
        )

    merged = {k: head[k] for k in _LATTICE_INVARIANTS}
    knots, work, travel = head["knot_fractions"], head["work_bins"], head["travel_bins"]
    merged["clear_mm"] = [
        [
            [
                [sum(p["clear_mm"][d][k][a][b] for p in parts) for b in range(len(travel))]
                for a in range(len(work))
            ]
            for k in range(len(knots))
        ]
        for d in range(len(head["standoffs_mm"]))
    ]
    merged["arc_mm"] = [
        sum(p["arc_mm"][k] for p in parts) for k in range(len(knots))
    ]
    merged["index_range"] = [0, head["samples"]]
    merged["rays_cast"] = sum(p["rays_cast"] for p in parts)
    merged["warnings"] = sorted({w for p in parts for w in p.get("warnings", [])})
    return merged


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def synthetic_corner(
    dihedral_deg: float, half_mm: float = 60.0, seam_length_mm: float = 80.0
) -> tuple[Any, dict[str, Any], dict[str, Any]]:
    """A groove of exactly `dihedral_deg`, with a joint and a seam on its corner.

    `(shape, joint, seam)`, ready for `clearance()`. This exists so the
    agreement with `tooling.fits_dihedral_deg` can be checked against geometry
    whose dihedral is known by construction rather than measured off a fixture
    that might be re-exported one day.

    The corner runs along **+X** through the origin, the material fills a block
    everywhere except an open wedge symmetric about **+Z**, and the seam frame
    is pinned with `fixed_direction_xyz = +Z`, so at work 0 / travel 0 the
    torch axis is the bisector and the closed form applies directly:

        a ray from transverse offset b leaves the material at w = |b| / tan(theta/2)

    The block is deliberately solid below the corner. A torch that tips far
    enough over runs into it, which is the behaviour a real fillet has and a
    pair of thin plates would not.
    """
    import cadquery as cq

    half_angle = math.radians(dihedral_deg) / 2.0
    reach = 20.0 * half_mm  # so the cut covers the block for any usable angle
    wedge = (
        cq.Workplane("YZ")
        .polyline(
            [
                (0.0, 0.0),
                (reach * math.sin(half_angle), reach * math.cos(half_angle)),
                (-reach * math.sin(half_angle), reach * math.cos(half_angle)),
            ]
        )
        .close()
        .extrude(4 * half_mm)
        .translate((-2 * half_mm, 0.0, 0.0))
    )
    block = cq.Workplane("XY").box(2 * half_mm, 2 * half_mm, 2 * half_mm)
    shape = block.cut(wedge).val()

    reach_m = seam_length_mm / 2000.0
    joint = {
        "id": "weld_joint_synthetic",
        "type": "fillet",
        "weld_line": {
            "closed": False,
            "length_m": 2 * reach_m,
            "segments": [
                {
                    "degree": 1,
                    "knots_u": [0.0, 0.0, 1.0, 1.0],
                    "control_points_xyz_m": [[-reach_m, 0.0, 0.0], [reach_m, 0.0, 0.0]],
                    "length_m": 2 * reach_m,
                }
            ],
        },
    }
    seam = {
        "id": "seam_synthetic",
        "weld_joint_ref": joint["id"],
        "span": {"start_s": 0.0, "end_s": 1.0},
        "direction": "forward",
        # No rail: the bisector here is known exactly, and feeding it in
        # directly keeps the test measuring clearance rather than measuring
        # whatever `detect._bisector_rail` produced.
        "reference": {"mode": "fixed_vector", "fixed_direction_xyz": [0.0, 0.0, 1.0]},
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 0.0},
        },
        "process": {"wire": {"stickout_mm": 10.0}},
    }
    return shape, joint, seam


def self_test() -> int:
    from seam_worker.cad.topology import extract_topology, load_step
    from seam_worker.reach.access import annotate
    from seam_worker.seam.detect import detect_joints
    from seam_worker.seam.seams import default_seams

    step = paths.PIPE_BRACKET
    if not step.exists():
        print(f"clearance self-test: missing fixture {step}", file=sys.stderr)
        return 1

    topology = extract_topology(step)
    joints = annotate(step, topology, detect_joints(topology))
    joints["seams"] = default_seams(joints)["seams"]
    by_id = {j["id"]: j for j in joints["weld_joints"]}

    seam = max(
        joints["seams"],
        key=lambda s: span_fraction(s["span"])
        * by_id[s["weld_joint_ref"]]["weld_line"]["length_m"],
    )
    shape = load_step(step)
    result = clearance(
        shape,
        by_id[seam["weld_joint_ref"]],
        seam,
        standoff_mm=weld_standoff_mm(seam, DEFAULT_TORCH["default_standoff_mm"]),
    )

    total = result["clear_length_m"] + result["blocked_length_m"]
    assert abs(total - result["seam_length_m"]) < 1e-9, (total, result["seam_length_m"])
    print(
        f"clearance self-test ok: {seam['id']} {result['seam_length_m'] * 1000:.1f} mm, "
        f"{result['clear_fraction'] * 100:.1f}% clear with "
        f"Ø{2 * result['torch']['body_radius_mm']:.0f} at {result['standoff_mm']:.0f} mm "
        f"({result['samples']} samples, {result['ray_count']} rays)"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", default="")
    parser.add_argument("--seam", default="", help="seam id; the longest one by default")
    parser.add_argument("--rays", type=int, default=FINE_RAY_COUNT)
    parser.add_argument("--standoff-mm", type=float, default=None)
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.step:
        raise ValueError("--step is required unless --self-test is set")

    from seam_worker.cad.topology import extract_topology, load_step
    from seam_worker.reach.access import annotate
    from seam_worker.seam.detect import detect_joints
    from seam_worker.seam.seams import default_seams

    step = Path(ns.step)
    topology = extract_topology(step)
    joints = annotate(step, topology, detect_joints(topology))
    joints["seams"] = default_seams(joints)["seams"]
    by_id = {j["id"]: j for j in joints["weld_joints"]}
    seams = joints["seams"]
    seam = (
        next(s for s in seams if s["id"] == ns.seam)
        if ns.seam
        else max(
            seams,
            key=lambda s: span_fraction(s["span"])
            * by_id[s["weld_joint_ref"]]["weld_line"]["length_m"],
        )
    )
    print(
        json.dumps(
            clearance(
                load_step(step),
                by_id[seam["weld_joint_ref"]],
                seam,
                ray_count=ns.rays,
                standoff_mm=ns.standoff_mm,
            ),
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"clearance: {exc}", file=sys.stderr)
        raise SystemExit(1)
