#!/usr/bin/env python
"""The T2 verifier: the thing that says a trajectory is safe, and means it.

Everything upstream of this file is a SEARCH. The ESDF is a grid, the spheres
are a cover, the optimizer descends a smooth surrogate, and all three are
approximations chosen because they are fast and differentiable. Measured on
this cell the sphere-vs-field error is 0.023 mm on average and 1.6 mm at
worst -- the same order as the clearances a weld actually runs at. So nothing
the planner believes about collision is allowed to ship on the planner's own
authority.

This module answers on different evidence: the real triangles, at fp64, on
the CPU, with an independent forward kinematics. It shares no code with the
thing it judges, which is the entire point of building it before the planner
rather than after.

## Why sampling is not verification

Check the waypoints and you have proved something about the waypoints. In
between, the arm swings. `WEAVE-AND-VERIFICATION.md` measured the gap between
consecutive samples on real plans at 1.2-2.4 mm, which is enough to pass
clean through a 3 mm clamp and report two clear endpoints. Halving the gap
doubles the cost and never closes it; the argument has no bottom.

## The certificate

So the verifier does not sample. For a segment `q(t) = q0 + t*dq`, every point of
link `L` moves with speed at most

    S_L = sum over joints j on L's chain of |dq_j| * R_j

where `R_j` bounds the distance from joint `j`'s axis to any point of any link
distal to it. That is a Lipschitz constant, and it turns one distance query
into a statement about an interval: if at the midpoint of `[t1, t2]` link L
clears obstacle O by more than `S_L * (t2 - t1) / 2`, then no point of L
touches O ANYWHERE in that interval. If it does not clear by that much, the
interval is halved and the two halves are asked the same question.

What comes back is not "no collision was found at the samples I took" but
"no collision exists on this segment", or an interval and a pair where the
proof failed. The recursion is bounded, and hitting the bound is a REFUSAL,
never a pass -- see `MAX_DEPTH`.

## Why the query is a predicate and not a distance

The certificate never needs to know how far apart things are, only whether
they are closer than `m`. coal answers that with `security_margin`, and it is
16-26x faster than the exact distance because the traversal stops at the first
box pair that proves the gap (measured in `BENCHMARKS.md` section 8; at verifier
margins, 9-14 us against 228). The exact distance is computed once per
verdict, for the report, and never inside the loop.

## The same certificate, pointed at the seam

`certify_tracking` asks the second question a weld verifier owes an answer to:
is the electrode where it belongs BETWEEN the samples. Same argument, two
substitutions -- the pair of bodies becomes a pair of points (the tip, and
where the tip belongs), and the mesh predicate becomes a distance. The
trajectory between knots is a cubic Hermite, so `q̇` is a quadratic whose
extremes are arithmetic; `chain_reach` turns those into a speed bound for the
tip and for the target riding the positioner; `SeamTarget` bounds how fast
the target itself moves along the seam. One distance at a midpoint then
bounds the error over a whole interval.

Two things it does that the collision side does not need to. Intervals are
cut at the weld line's CORNERS first, because a torch standing off the work
has a target that genuinely jumps there and no Lipschitz constant describes
a jump. And once something is out of tolerance the recursion stops being a
yes/no test and becomes a search for the worst instant, because "it misses"
is not an actionable sentence and "it misses by 20 mm" is.

## What fails closed

A missing velocity limit is not a passed velocity check. This cell's profile
carries position limits and NOT velocity or acceleration -- deliberately, since
a URDF `<limit>` has no rate fields and inventing them would certify a
trajectory against numbers nobody measured. The verdict therefore reports
those checks as `unverifiable` and the overall result is not `PASS`. The
distinction the whole file exists to preserve is between "checked and clear"
and "not checked".

M6 approach, transit, and retract moves have no programmed seam tangent or weld
travel speed. Their dedicated entry point marks that weld-only check not
applicable, but only after selecting exact position, velocity, and acceleration
extrema of the emitted cubic Hermite curve. A missing or malformed rate input
still fails closed; not-applicable is not a shortcut around applicable checks.

The tracking certificate obeys the same rule and has its own instance of it:
the seam-target derivative bound is exact on a straight weld line and does
not yet exist for a curved one, so a curved seam comes back `unverifiable`
rather than clear. Every fixture in the corpus authors straight segments, so
nothing is refused today -- which is precisely when the rule is easy to
forget to write.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Iterable, Sequence
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Literal

import numpy as np

from weld_motion_planner.verifier.hermite_rates import (
    HermiteRateError,
    judge_hermite_rates,
)

# Halving a segment 10 times makes an interval of 1/1024 of it: on this
# planner's wire (161 samples over 3-8 s, so 20-50 ms a segment) that is
# 0.02-0.05 ms, in which a body at 1 m/s moves 0.02-0.05 mm -- under the
# 0.1 mm anyone here would call a distance, and under what the meshes
# resolve. Deeper than this is not a finer answer, it is a refusal that
# took longer to arrive at. (It was 40 -- 1e-12 of a segment -- which no
# real motion ever reached; the cap only ever bounds the pathological
# grazing case, and reaching it is a REFUSAL, never a pass.)
MAX_DEPTH = 10
# The verifier's own safety margin, on top of the certificate. Zero would still be
# sound; this is here because the meshes are the VISUAL meshes and the
# fixtures are hand-measured.
DEFAULT_VERIFY_MARGIN_M = 0.002
# How many tasks a trajectory becomes on a pool the CALLER lends `verify`.
# Fine enough that one long weld cannot pin the pool's wall time to its own
# (measured: one 120-segment seam was 3.9 s of a 4.5 s stage), coarse enough
# that the geometry -- pickled once per task, its BVHs rebuilt on arrival --
# is not rebuilt per segment.
POOL_TASKS = 4


class VerifyError(ValueError):
    """A trajectory or a cell this refuses to judge, with the reason."""


FreeSpaceMotion = Literal["approach", "transit", "retract"]
_FREE_SPACE_MOTIONS = frozenset(("approach", "transit", "retract"))


# --------------------------------------------------------------------------
# Kinematics, written again


def _rpy(rpy: Sequence[float]) -> np.ndarray:
    """Z-Y-X intrinsic, the URDF convention, in float64.

    Written here rather than imported from `fk.py` on purpose. A verifier that
    shares its kinematics with the planner cannot catch a kinematics bug, and
    a kinematics bug is exactly the kind that puts a link somewhere nobody
    checked. `tests/motion_planner/test_verify.py` holds this against the committed
    oracle so the independence does not become a second source of truth.
    """
    r, p, y = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues, float64."""
    a = np.asarray(axis, dtype=np.float64)
    a = a / np.linalg.norm(a)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * (k @ k)


@dataclass(frozen=True)
class Kinematics:
    """The joint tree, in the order a walk from the base visits it."""

    base: str
    axis_names: list[str]
    joints: list[dict[str, Any]]      # topologically ordered, parent before child
    parent_of: dict[str, str]
    links: list[str]

    @property
    def num_dof(self) -> int:
        return len(self.axis_names)

    def transforms(self, q: Sequence[float]) -> dict[str, np.ndarray]:
        """Every link's world pose at `q`, float64."""
        q = np.asarray(q, dtype=np.float64)
        if q.shape != (self.num_dof,):
            raise VerifyError(f"q has shape {q.shape}, this cell has {self.num_dof} axes")
        out: dict[str, np.ndarray] = {self.base: np.eye(4)}
        for joint in self.joints:
            parent, child = str(joint["parent"]), str(joint["child"])
            if parent not in out:
                raise VerifyError(f"joint {joint['name']!r} has parent {parent!r} before it is placed")
            local = np.eye(4)
            local[:3, :3] = _rpy(joint.get("rpy_rad") or (0, 0, 0))
            local[:3, 3] = np.asarray(joint.get("xyz_m") or (0, 0, 0), dtype=np.float64)
            kind = str(joint.get("type") or "fixed")
            if kind in ("revolute", "continuous"):
                value = float(q[self.axis_names.index(str(joint["name"]))])
                motion = np.eye(4)
                motion[:3, :3] = _axis_rotation(joint["axis_xyz"], value)
                local = local @ motion
            elif kind == "prismatic":
                value = float(q[self.axis_names.index(str(joint["name"]))])
                axis = np.asarray(joint["axis_xyz"], dtype=np.float64)
                motion = np.eye(4)
                motion[:3, 3] = axis / np.linalg.norm(axis) * value
                local = local @ motion
            elif kind != "fixed":
                raise VerifyError(f"joint {joint['name']!r} has unsupported type {kind!r}")
            out[child] = out[parent] @ local
        return out


def kinematics(cell: dict[str, Any], *, held: dict[str, float] | None = None) -> Kinematics:
    joints_in = list(cell.get("joints") or [])
    if not joints_in:
        raise VerifyError("this cell has no joints; there is nothing to verify")
    base = str((cell.get("frames") or {}).get("base") or "")
    if not base:
        raise VerifyError("this cell names no base frame")

    held = held or {}
    ordered: list[dict[str, Any]] = []
    placed = {base}
    remaining = list(joints_in)
    while remaining:
        progressed = False
        for joint in list(remaining):
            if str(joint["parent"]) in placed:
                ordered.append(joint)
                placed.add(str(joint["child"]))
                remaining.remove(joint)
                progressed = True
        if not progressed:
            orphans = sorted({str(j["child"]) for j in remaining})
            raise VerifyError(f"these links never connect to {base!r}: {orphans}")

    axis_names = [str(j["name"]) for j in ordered
                  if str(j.get("type") or "fixed") != "fixed" and str(j["name"]) not in held]
    # A held axis becomes part of the geometry rather than part of `q`. The
    # profile's `axes.held` is what says which -- J8 and J9 on this cell, the
    # bay-B tilt and the frame index, both of which the arm must not hit and
    # neither of which this planner moves.
    frozen = []
    for joint in ordered:
        name = str(joint["name"])
        if name in held and str(joint.get("type") or "fixed") != "fixed":
            baked = dict(joint)
            local = np.eye(4)
            local[:3, :3] = _rpy(joint.get("rpy_rad") or (0, 0, 0))
            local[:3, 3] = np.asarray(joint.get("xyz_m") or (0, 0, 0), dtype=np.float64)
            if str(joint["type"]) == "prismatic":
                axis = np.asarray(joint["axis_xyz"], dtype=np.float64)
                shift = np.eye(4)
                shift[:3, 3] = axis / np.linalg.norm(axis) * float(held[name])
                local = local @ shift
            else:
                rot = np.eye(4)
                rot[:3, :3] = _axis_rotation(joint["axis_xyz"], float(held[name]))
                local = local @ rot
            baked["type"] = "fixed"
            baked["rpy_rad"] = list(_rpy_from_matrix(local[:3, :3]))
            baked["xyz_m"] = [float(v) for v in local[:3, 3]]
            frozen.append(baked)
        else:
            frozen.append(joint)

    parent_of = {str(j["child"]): str(j["parent"]) for j in frozen}
    links = [base] + [str(j["child"]) for j in frozen]
    return Kinematics(base=base, axis_names=axis_names, joints=frozen,
                      parent_of=parent_of, links=links)


def _rpy_from_matrix(rot: np.ndarray) -> tuple[float, float, float]:
    """Inverse of `_rpy`, for baking a held joint back into a fixed one."""
    sp = -float(rot[2, 0])
    sp = max(-1.0, min(1.0, sp))
    p = math.asin(sp)
    if abs(sp) > 1.0 - 1e-9:  # gimbal: yaw and roll are not separable
        return (math.atan2(-rot[1, 2], rot[1, 1]), p, 0.0)
    return (math.atan2(rot[2, 1], rot[2, 2]), p, math.atan2(rot[1, 0], rot[0, 0]))


# --------------------------------------------------------------------------
# The Lipschitz constants the certificate is built from


def reach_radii(kin: Kinematics, geom: Geometry) -> dict[str, float]:
    """For each moving joint, a bound on how far its axis reaches.

    `R_j` must dominate the distance from joint `j`'s axis to any point of any
    link distal to `j`, at EVERY configuration of the joints in between --
    otherwise `|dp/dq_j| <= R_j` is false somewhere and the certificate is
    void. Computed by a walk up from the leaves:

        R_j = max over the bodies rigidly attached to j's child of |v|
              and over each child joint k of ( |t_(j->k)| + R_k )

    where `|v|` is measured in the child link's own frame and `t_(j->k)` is the
    fixed offset between the two joint origins. Distance to a LINE is at most
    distance to a point on it, so bounding by the ball is sound; it is loose by
    however far the link sits along the axis, which costs step length and never
    correctness.

    A joint whose subtree carries no geometry gets 0.0, which is correct rather
    than a special case: nothing distal to it can hit anything.

    Takes the GEOMETRY and not a link-keyed mesh dict, because a body's name is
    not its link's name -- the workpiece is called "workpiece" and rides
    "table_a". Keying by link silently gave J7 a reach of zero, and a reach of
    zero is a certificate that proves nothing while reporting success.
    """
    children: dict[str, list[dict[str, Any]]] = {}
    for joint in kin.joints:
        children.setdefault(str(joint["parent"]), []).append(joint)

    on_link: dict[str, list[str]] = {}
    for body, link in geom.attached_to.items():
        on_link.setdefault(link, []).append(body)

    def ball(link: str) -> float:
        """Radius of everything rigidly attached to `link`, about its origin."""
        best = 0.0
        for body in on_link.get(link, []):
            mesh = geom.meshes[body]
            if not len(mesh.vertices):
                continue
            local = geom.local[body]
            placed = np.asarray(mesh.vertices, dtype=np.float64) @ local[:3, :3].T + local[:3, 3]
            best = max(best, float(np.linalg.norm(placed, axis=1).max()))
        for joint in children.get(link, []):
            offset = float(np.linalg.norm(np.asarray(joint.get("xyz_m") or (0, 0, 0), dtype=np.float64)))
            if str(joint.get("type") or "fixed") == "fixed":
                # Rigid: the subtree's own ball, translated.
                best = max(best, offset + ball(str(joint["child"])))
            else:
                best = max(best, offset + ball(str(joint["child"])))
        return best

    radii: dict[str, float] = {}
    for joint in kin.joints:
        name = str(joint["name"])
        if name not in kin.axis_names:
            continue
        if str(joint["type"]) == "prismatic":
            radii[name] = 1.0  # a metre of joint travel is a metre of motion
        else:
            radii[name] = ball(str(joint["child"]))
    return radii


def speed_bound(kin: Kinematics, radii: dict[str, float], dq: np.ndarray,
                link: str) -> float:
    """Max speed of any point of `link` for a unit-time move of `dq`.

    Only the joints ON this link's chain contribute -- a wrist rotation does
    not move the shoulder, and charging every link the whole arm's motion
    would make the certificate true but useless.
    """
    total = 0.0
    node = link
    while node in kin.parent_of:
        joint = next(j for j in kin.joints if str(j["child"]) == node)
        name = str(joint["name"])
        if name in kin.axis_names:
            total += abs(float(dq[kin.axis_names.index(name)])) * radii.get(name, 0.0)
        node = kin.parent_of[node]
    return total


# --------------------------------------------------------------------------
# Geometry


@dataclass
class Geometry:
    """The real triangles, indexed by the link that carries them.

    `attached_to` is what makes the workpiece ordinary: it rides the work
    chain exactly as link_6 rides the arm, so the certificate does not need a
    special case for a moving obstacle.
    """

    meshes: dict[str, Any]
    attached_to: dict[str, str]       # body -> link it is rigid with
    local: dict[str, np.ndarray]      # body -> 4x4 in that link's frame
    pairs: list[tuple[str, str]]      # bodies to check against each other
    _objects: dict[str, Any] = field(default_factory=dict, repr=False)

    def objects(self) -> dict[str, Any]:
        """coal collision objects, built once per process.

        Built lazily and cached on the instance because a `ProcessPoolExecutor`
        pickles this across to each worker, and a BVH is not picklable -- each
        worker builds its own. That is also why the pool is `fork`-free-safe:
        nothing shared is mutated.
        """
        if not self._objects:
            import coal

            for name, mesh in self.meshes.items():
                verts = np.asarray(mesh.vertices, dtype=np.float64)
                faces = np.asarray(mesh.faces, dtype=np.int64)
                vs, ts = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
                for v in verts:
                    vs.append(v)
                for f in faces:
                    ts.append(coal.Triangle(int(f[0]), int(f[1]), int(f[2])))
                model = coal.BVHModelOBBRSS()
                model.beginModel(len(faces), len(verts))
                model.addSubModel(vs, ts)
                model.endModel()
                self._objects[name] = coal.CollisionObject(model, coal.Transform3s())
        return self._objects

    def __getstate__(self) -> dict[str, Any]:
        state = self.__dict__.copy()
        state["_objects"] = {}
        return state


def load_dae(path: Path) -> Any:
    """A COLLADA file as a `mesh.Mesh`, in metres.

    Read here rather than through trimesh: the motion environment carries
    pycollada and not trimesh, and this needs three fields out of the file.
    Unit scaling is honoured -- the cell's own .dae files are already in
    metres, and a file that is not would otherwise put a link a thousand times
    too far away without saying so.
    """
    import collada

    from weld_motion_planner.geometry.mesh import Mesh

    doc = collada.Collada(str(path), ignore=[collada.common.DaeUnsupportedError,
                                             collada.common.DaeBrokenRefError])
    scale = float(getattr(doc.assetInfo, "unitmeter", None) or 1.0)
    verts: list[np.ndarray] = []
    faces: list[np.ndarray] = []
    offset = 0
    for geom in doc.geometries:
        for prim in geom.primitives:
            tri = prim.triangleset() if hasattr(prim, "triangleset") else prim
            v = np.asarray(tri.vertex, dtype=np.float64)
            f = np.asarray(tri.vertex_index, dtype=np.int64).reshape(-1, 3)
            verts.append(v)
            faces.append(f + offset)
            offset += len(v)
    if not verts:
        raise VerifyError(f"{path.name} carries no triangles")
    return Mesh(vertices=np.concatenate(verts) * scale,
                faces=np.concatenate(faces).astype(np.int32))


def from_cell(cell: dict[str, Any], *, mesh_root: Path,
              workpiece: Any = None, work_to_piece: np.ndarray | None = None,
              ignore: Iterable[tuple[str, str]] = (),
              extra_static: Iterable[Any] = ()) -> Geometry:
    """Assemble the real cell: every link's own .dae plus the placed workpiece.

    `extra_static` are the operator's fixtures from the plan request, already
    in the world frame (`robot_cell.collision.fixtures`). They are attached
    to the base link and paired with the arm and the workpiece only: a
    fixture standing against the pedestal is not a collision, and a pair of
    two bodies that never move could only ever report one.

    The pair list comes from `pairs.adjacency` for the same reason the search
    side uses it -- links joined by a joint touch at the mating face by
    construction, so checking them reports a collision at every pose and the
    verifier would refuse everything. `ignore` adds pairs on top, for the ones
    adjacency cannot see.
    """
    from weld_motion_planner.robot_cell.collision.pairs import adjacency

    meshes: dict[str, Any] = {}
    attached: dict[str, str] = {}
    local: dict[str, np.ndarray] = {}
    for link in cell.get("links") or []:
        name = str(link["name"])
        path = mesh_root / f"{name}.dae"
        if not path.is_file():
            continue
        meshes[name] = load_dae(path)
        attached[name] = name
        local[name] = np.eye(4)

    if workpiece is not None:
        work = str((cell.get("frames") or {}).get("work") or "")
        if not work:
            raise VerifyError("a workpiece was given but this cell names no work frame")
        meshes["workpiece"] = workpiece
        attached["workpiece"] = work
        local["workpiece"] = (np.eye(4) if work_to_piece is None
                              else np.asarray(work_to_piece, dtype=np.float64))

    frames = cell.get("frames") or {}
    base = str(frames.get("base") or "world")
    # Fixtures are judged against the arm and the workpiece alone.
    #
    # "The arm" means the links that MOVE. The chain from the tool does not stop
    # at the first joint: on `rosie_1420_v1` it runs tool0 .. link_1 -> base ->
    # pedestal -> world, and the last two hang off the base frame by fixed
    # joints alone. Counting them made the pedestal an arm link, and the cell's
    # own placeholder stand (0.5 m square at x, y = 1.0) overlaps the pedestal
    # mesh (0..0.825 by 0..0.8) by a 75 x 50 mm corner -- two bodies that never
    # move, reported at every segment of every seam: 122 of 122 on
    # `f7a12ed2`, 2026-09-21. So the walk keeps a link only once a joint that
    # is not `fixed` lies between it and the base.
    joint_of = {str(j["child"]): j for j in (cell.get("joints") or [])}
    chain: list[str] = []
    walk = str(frames.get("tool") or "")
    while walk and walk != base and walk in joint_of:
        chain.append(walk)
        walk = str(joint_of[walk]["parent"])
    arm: set[str] = set()
    moved = False
    for link in reversed(chain):          # from the base outwards
        moved = moved or str(joint_of[link].get("type")) != "fixed"
        if moved:                         # and everything past it rides along:
            arm.add(link)                 # the tool frame is fixed to link_6
    fixture_bodies: set[str] = set()
    for index, mesh in enumerate(extra_static):
        name = f"fixture_{index}"
        meshes[name] = mesh
        attached[name] = base
        local[name] = np.eye(4)
        fixture_bodies.add(name)

    skip = {frozenset(p) for p in adjacency(cell)} | {frozenset(p) for p in ignore}
    bodies = sorted(meshes)

    def wanted(a: str, b: str) -> bool:
        if a in fixture_bodies or b in fixture_bodies:
            other = b if a in fixture_bodies else a
            return other not in fixture_bodies and (other in arm or other == "workpiece")
        return frozenset((attached[a], attached[b])) not in skip and attached[a] != attached[b]

    pairs = [(a, b) for i, a in enumerate(bodies) for b in bodies[i + 1:] if wanted(a, b)]
    return Geometry(meshes=meshes, attached_to=attached, local=local, pairs=pairs)


# --------------------------------------------------------------------------
# The verifier


@dataclass
class Verdict:
    """What the verifier found, and what it did not look at.

    `passed` is a conjunction over checks that RAN. `unverifiable` is the list
    of checks that could not, and it is not empty on this cell today. A caller
    that reads `passed` without reading `unverifiable` has a verifier that passes
    what nobody measured, which is why `ok` folds both.
    """

    passed: bool
    collisions: list[dict[str, Any]] = field(default_factory=list)
    limit_violations: list[dict[str, Any]] = field(default_factory=list)
    unverifiable: list[str] = field(default_factory=list)
    worst_clearance_m: float | None = None
    # What the screen established when nothing came inside it. "Further than
    # 50 mm" is the honest report; a `None` clearance with no floor would read
    # as "not measured".
    clearance_floor_m: float | None = None
    worst_at: dict[str, Any] | None = None
    # T1's numbers: mobility along the seam tangent, and whether the programmed
    # travel speed is deliverable inside the joint rate limits.
    travel: dict[str, Any] = field(default_factory=dict)
    segments: int = 0
    queries: int = 0

    @property
    def ok(self) -> bool:
        return self.passed and not self.unverifiable

    def summary(self) -> dict[str, Any]:
        return {
            "verdict": "PASS" if self.ok else ("REFUSED" if self.passed else "FAIL"),
            "collisions": len(self.collisions),
            # Of those, how many are contact rather than mere proximity to
            # the verifier's own pad. Zero here with a non-zero count above means
            # the trajectory never touches anything and the margin is what
            # failed it.
            "penetrating": sum(1 for c in self.collisions if c.get("penetrating")),
            "limit_violations": len(self.limit_violations),
            # Keep bounded evidence alongside the counts. A count alone loses
            # which bodies or joint limits prevented admission. Counts remain
            # authoritative when a long report is truncated here.
            "collision_details": self.collisions[:8],
            "limit_details": self.limit_violations[:16],
            "unverifiable": self.unverifiable,
            "worst_clearance_mm": (
                round(self.worst_clearance_m * 1000.0, 3) if self.worst_clearance_m is not None
                else (f">{self.clearance_floor_m * 1000:.0f}"
                      if self.clearance_floor_m is not None else None)),
            "worst_at": self.worst_at,
            "segments": self.segments,
            "queries": self.queries,
            "travel": self.travel,
        }


def replace_limit_findings(
        verdict: Verdict,
        violations: Sequence[dict[str, Any]],
        unverifiable: Sequence[str]) -> Verdict:
    """Replace a generic rate read with findings from the emitted curve.

    Collision and travel findings belong to the generic verifier and survive.
    Position, velocity, and acceleration findings are one coherent check and
    are replaced together so a secant result cannot sit beside the exact
    Hermite result for the same emitted motion.
    """
    superseded = {"position", "velocity", "acceleration"}
    retained = [
        finding
        for finding in verdict.limit_violations
        if finding.get("kind") not in superseded
    ]
    combined = [*violations, *retained]
    unknown = list(dict.fromkeys([*verdict.unverifiable, *unverifiable]))
    return replace(
        verdict,
        passed=not verdict.collisions and not combined,
        limit_violations=combined,
        unverifiable=unknown,
    )


def _place(obj, matrix: np.ndarray) -> None:
    import coal

    obj.setTransform(coal.Transform3s(np.ascontiguousarray(matrix[:3, :3], dtype=np.float64),
                                      np.ascontiguousarray(matrix[:3, 3], dtype=np.float64)))


def _closer_than(objects, a: str, b: str, margin: float) -> bool:
    """coal's margin predicate: true when the gap is below `margin`.

    The whole inner loop. Not `distance() < margin`, because the traversal can
    stop at the first box pair that proves the gap and does not have to find
    the closest one.
    """
    import coal

    request = coal.CollisionRequest()
    request.security_margin = float(margin)
    result = coal.CollisionResult()
    coal.collide(objects[a], objects[b], request, result)
    return bool(result.isCollision())


def _exact(objects, a: str, b: str) -> float:
    import coal

    result = coal.DistanceResult()
    return float(coal.distance(objects[a], objects[b], coal.DistanceRequest(), result))


def chain_weights(kin: Kinematics, radii: dict[str, float], link: str) -> np.ndarray:
    """`speed_bound`'s chain walk, precomputed as a vector over the axes.

    Same arithmetic -- only the joints on this link's chain contribute -- but
    a dot product instead of a walk, because once the motion between two
    waypoints is a CURVE the speed bound is no longer one number for the
    whole segment and has to be recomputed at every node of the bisection.
    """
    weights = np.zeros(kin.num_dof)
    node = link
    while node in kin.parent_of:
        joint = next(j for j in kin.joints if str(j["child"]) == node)
        name = str(joint["name"])
        if name in kin.axis_names:
            weights[kin.axis_names.index(name)] = radii.get(name, 0.0)
        node = kin.parent_of[node]
    return weights


def certify_segment(kin: Kinematics, geom: Geometry, radii: dict[str, float],
                    q0: np.ndarray, q1: np.ndarray, *,
                    d0: np.ndarray | None = None, d1: np.ndarray | None = None,
                    duration_s: float = 1.0,
                    margin_m: float = DEFAULT_VERIFY_MARGIN_M) -> dict[str, Any]:
    """Prove that no checked pair touches anywhere on `q0 -> q1`, or say where.

    Conservative advancement by bisection. Each interval is judged at its
    midpoint against the distance its bodies could have travelled from either
    end, so a clear answer covers the whole interval rather than the point.

    `d0`/`d1` are joint VELOCITIES at the two ends, and giving them changes
    what is being certified: the motion becomes the cubic Hermite through the
    pair, which is the curve M5 actually emits, instead of the straight line
    between the waypoints. That difference is not cosmetic -- measured on
    `bracket_a_placed`'s corner seams the Hermite leaves the chord by 4.4
    degrees of joint angle, which is of order 100 mm at the wrist, so a
    certificate over the chord is a certificate about a path the robot does
    not take. Omitting them sets the velocities to the chord's own slope, at
    which the Hermite IS the straight line, so the default is the old
    behaviour exactly rather than by approximation.
    """
    objects = geom.objects()
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dq = q1 - q0
    d0 = dq / duration_s if d0 is None else np.asarray(d0, dtype=np.float64)
    d1 = dq / duration_s if d1 is None else np.asarray(d1, dtype=np.float64)

    # The chain each body hangs off, as a vector, so a node costs a dot
    # product. Two bodies approaching each other close at no more than the
    # sum of their speeds.
    weights = {body: chain_weights(kin, radii, geom.attached_to[body])
               for body in geom.meshes}

    queries = 0

    # Memoized, because the pairs are certified independently and therefore
    # ask for the SAME times: every pair starts at t = 0.5, most stop there,
    # and the ones that bisect walk the same dyadic ladder. Without this the
    # verifier paid 54 forward kinematics for one instant of the trajectory, and
    # kinematics -- not the geometry queries -- was two thirds of its cost.
    cache: dict[float, dict[str, np.ndarray]] = {}

    def frames_at(t: float) -> dict[str, np.ndarray]:
        got = cache.get(t)
        if got is None:
            got = cache[t] = kin.transforms(_hermite(q0, q1, d0, d1, duration_s, t))
        return got

    def poses(t: float, bodies: Iterable[str]) -> None:
        """Place only what is about to be asked about."""
        frames = frames_at(t)
        for body in bodies:
            _place(objects[body], frames[geom.attached_to[body]] @ geom.local[body])

    # Each pair is certified independently: they have different speed bounds,
    # so a wrist pair that needs deep bisection should not drag a base pair
    # down with it.
    for a, b in geom.pairs:
        closing = weights[a] + weights[b]
        stack: list[tuple[float, float, int]] = [(0.0, 1.0, 0)]
        while stack:
            lo, hi, depth = stack.pop()
            mid = 0.5 * (lo + hi)
            # The fastest either body can be going ANYWHERE on [lo, hi], from
            # the closed-form extremes of the Hermite's quadratic derivative
            # -- a straight line has one speed for the whole segment and a
            # curve does not, so this narrows as the interval does and buys
            # back most of what the curve costs.
            rate = duration_s * _velocity_extreme(q0, q1, d0, d1, duration_s, lo, hi)
            # Anything on [lo, hi] is within this much of where it is at mid.
            travel = float(closing @ rate) * (hi - lo) * 0.5
            poses(mid, (a, b))
            queries += 1
            if not _closer_than(objects, a, b, travel + margin_m):
                continue
            if depth >= MAX_DEPTH:
                return {"clear": False, "reason": "undecided", "pair": [a, b],
                        "t": [lo, hi], "queries": queries,
                        "worst_clearance_m": None}
            # Still ambiguous. If the pair is ACTUALLY touching at the
            # midpoint we are done immediately; otherwise halve.
            if _closer_than(objects, a, b, margin_m):
                queries += 1
                gap = _exact(objects, a, b)
                return {"clear": False, "reason": "collision", "pair": [a, b],
                        "t": [mid, mid], "queries": queries,
                        "worst_clearance_m": gap,
                        # Touching and "inside my safety pad" are different
                        # findings and the verifier should not spell them the
                        # same way. A weld pass is the case that makes the
                        # difference matter: the gun body clears the part by
                        # a millimetre or two BY DESIGN, so at any positive
                        # margin a correct trajectory reports a collision it
                        # does not have, and an operator who cannot tell the
                        # two apart learns to ignore both.
                        "penetrating": bool(gap <= 0.0),
                        "margin_m": float(margin_m)}
            queries += 1
            stack.append((lo, mid, depth + 1))
            stack.append((mid, hi, depth + 1))

    return {"clear": True, "queries": queries}


DEFAULT_SCREEN_M = 0.05


def report_clearance(kin: Kinematics, geom: Geometry, waypoints: np.ndarray, *,
                     screen_m: float = DEFAULT_SCREEN_M
                     ) -> tuple[float | None, dict[str, Any] | None, int]:
    """The worst clearance along the trajectory, without paying for it.

    The verdict wants a number an operator can read, and the exact distance
    costs 200 us a pair against 10 for the margin predicate. So the pairs are
    SCREENED at `screen_m` first, and only the ones that trip the screen are
    measured exactly. On a trajectory with any real clearance that is zero
    exact queries, and the answer is reported as a floor rather than as a
    figure -- "further than 50 mm" is what was actually established.

    This used to run inside `certify_segment`, one exact query per pair per
    segment. On the real cell that was 54 exact queries a segment and about
    half the verifier's total cost, all of it for a line in the report.

    It samples the WAYPOINTS, and that is sound only because it is a report.
    When the trajectory is a curve the tightest instant may fall between two
    of them, so read this as "no worse than" and read `collisions` for the
    statement that covers every instant.
    """
    objects = geom.objects()
    queries = 0
    worst: float | None = None
    worst_at: dict[str, Any] | None = None
    for k, q in enumerate(np.asarray(waypoints, dtype=np.float64)):
        frames = kin.transforms(q)
        for body, link in geom.attached_to.items():
            _place(objects[body], frames[link] @ geom.local[body])
        for a, b in geom.pairs:
            queries += 1
            if not _closer_than(objects, a, b, screen_m):
                continue
            d = _exact(objects, a, b)
            queries += 1
            if worst is None or d < worst:
                worst, worst_at = d, {"pair": [a, b], "waypoint": k}
    return worst, worst_at, queries


# --------------------------------------------------------------------------
# Limits


_RATE_SPELLINGS = {
    "velocity": ("velocity_rad_s", "velocity", "max_velocity_rad_per_s"),
    "acceleration": ("acceleration_rad_s2", "acceleration",
                     "max_acceleration_rad_per_s2"),
}


def _rate(spec: dict[str, Any], kind: str) -> float | None:
    """One axis's stated `kind` limit, whichever way the document spells it.

    `velocity_rad_s` first: it is the key `weld_motion_planner.robot_cell.robot.profile`
    NORMALISES to, and a check that did not know it reported "no rate limits"
    for months over a profile that carried them -- the owner authored
    120/240/60 deg/s and nothing looked at the keys the loader writes. The
    older spellings stay for raw documents.

    Shared rather than repeated, because that bug was found and fixed in
    `check_limits` while `check_travel` kept its own copy of the wrong list
    and went on reporting T1 as unverifiable. One list is the fix; two is how
    the fix half-took.
    """
    for key in _RATE_SPELLINGS[kind]:
        value = spec.get(key)
        if value is not None:
            return float(value)
    return None


def check_limits(kin: Kinematics, cell: dict[str, Any], profile: dict[str, Any],
                 waypoints: np.ndarray, times_s: np.ndarray | None, *,
                 knot_velocities: np.ndarray | None = None
                 ) -> tuple[list[dict[str, Any]], list[str]]:
    """Position always; full-curve extrema when the caller selects Hermite.

    Position-only candidates retain endpoint position and secant-rate checks.
    A caller that supplies knot velocities selects the emitted cubic Hermite
    curve, whose position and derivative limits are judged from polynomial
    extrema.
    """
    violations: list[dict[str, Any]] = []
    unverifiable: list[str] = []

    bounds = {}
    for joint in cell.get("joints") or []:
        limits = joint.get("limits_rad") or {}
        if "lower" in limits and "upper" in limits:
            bounds[str(joint["name"])] = (float(limits["lower"]), float(limits["upper"]))

    for i, name in enumerate(kin.axis_names):
        if name not in bounds:
            unverifiable.append(f"{name}: no position limit in the cell descriptor")
            continue
        lo, hi = bounds[name]
        below = np.nonzero(waypoints[:, i] < lo)[0]
        above = np.nonzero(waypoints[:, i] > hi)[0]
        for k in list(below[:3]) + list(above[:3]):
            violations.append({"axis": name, "kind": "position", "index": int(k),
                               "value_rad": float(waypoints[k, i]),
                               "limit_rad": [lo, hi]})

    limits = profile.get("limits") or {}
    rates = {k: v for k, v in limits.items()
             if isinstance(v, dict) and _rate(v, "velocity") is not None}
    if not rates and knot_velocities is None:
        unverifiable.append(
            "velocity and acceleration: the cell profile carries no rate limits, "
            "so no trajectory can be certified against them")
        return violations, unverifiable
    if times_s is None:
        unverifiable.append("velocity and acceleration: the trajectory carries no timing")
        return violations, unverifiable

    if knot_velocities is not None:
        try:
            exact = judge_hermite_rates(
                kin.axis_names, waypoints, knot_velocities, times_s, limits,
                allow_missing_limits=True,
            )
        except HermiteRateError as exc:
            raise VerifyError(f"the emitted Hermite rate check refused: {exc}") from exc
        unverifiable.extend(exact["unverifiable"])
        for axis in exact["per_joint"]:
            name = str(axis["axis"])
            if name not in bounds:
                continue
            lo, hi = bounds[name]
            for point in axis["position"].values():
                value = float(point["value_rad"])
                # Endpoint positions were checked above. Only an interior
                # extremum is new evidence about the emitted Hermite.
                if not 0.0 < float(point["u"]) < 1.0:
                    continue
                if value < lo or value > hi:
                    violations.append({
                        "axis": name,
                        "kind": "position",
                        "value_rad": value,
                        "limit_rad": [lo, hi],
                        "segment": point["segment"],
                        "u": point["u"],
                        "time_s": point["time_s"],
                        "exact_hermite": True,
                    })
        for finding in exact["violations"]:
            kind = str(finding["kind"])
            if kind == "velocity":
                value = finding["signed_rad_s"]
                limit = finding["limit_rad_s"]
            else:
                value = finding["signed_rad_s2"]
                limit = finding["limit_rad_s2"]
            violations.append({
                "axis": finding["axis"],
                "kind": kind,
                "value": value,
                "limit": limit,
                "ratio": finding["ratio"],
                "segment": finding["segment"],
                "u": finding["u"],
                "time_s": finding["time_s"],
                "exact_hermite": True,
            })
        return violations, unverifiable

    dt = np.diff(times_s)
    if np.any(dt <= 0):
        raise VerifyError("the trajectory's times are not strictly increasing")
    velocity = np.diff(waypoints, axis=0) / dt[:, None]
    acceleration = np.diff(velocity, axis=0) / dt[:-1, None]
    for i, name in enumerate(kin.axis_names):
        spec = rates.get(name) or {}
        vmax = _rate(spec, "velocity") or 0.0
        amax = _rate(spec, "acceleration") or 0.0
        if vmax > 0:
            for k in np.nonzero(np.abs(velocity[:, i]) > vmax)[0][:3]:
                violations.append({"axis": name, "kind": "velocity", "index": int(k),
                                   "value": float(velocity[k, i]), "limit": vmax})
        else:
            unverifiable.append(f"{name}: no velocity limit")
        if amax > 0:
            for k in np.nonzero(np.abs(acceleration[:, i]) > amax)[0][:3]:
                violations.append({"axis": name, "kind": "acceleration", "index": int(k),
                                   "value": float(acceleration[k, i]), "limit": amax})
        else:
            unverifiable.append(f"{name}: no acceleration limit")
    return violations, unverifiable


# --------------------------------------------------------------------------
# T1: can the arm actually deliver the travel speed


def check_travel(cell: dict[str, Any], profile: dict[str, Any],
                 waypoints: np.ndarray, directions: np.ndarray | None,
                 travel_speed_m_s: np.ndarray | None,
                 *, tip_local: Any = None,
                 held: dict[str, float] | None = None
                 ) -> tuple[list[dict[str, Any]], list[str], dict[str, Any]]:
    """T1, as the constraint that survives weave being out of scope.

    `WEAVE-AND-VERIFICATION.md` introduces this index as `mu_b'`, along the
    weave's transverse axis, to keep an arm weave-capable. Weave is out
    (`ARCHITECTURE.md`), and this does not bring it back. What it checks is
    the constraint the WELD imposes on its own: a seam is run at a programmed
    travel speed along the tangent, and near a singularity in that direction
    the joints must slew hard to deliver a modest tip speed. Same arithmetic,
    direction taken from the program rather than from an oscillation.

    Every way this can decline to answer is a REFUSAL and not a pass:
    no directions, no travel speed, or no rate limits each come back in
    `unverifiable`.
    """
    unverifiable: list[str] = []
    if directions is None or travel_speed_m_s is None:
        unverifiable.append(
            "travel mobility: the trajectory carries no seam tangent or no travel "
            "speed, so the arm's ability to deliver it was not checked")
        return [], unverifiable, {}

    import torch

    from weld_motion_planner.robot_cell.robot import fk
    from weld_motion_planner.robot_cell.robot.mobility import (
        directional,
        jacobian,
        travel_margin,
    )

    tool = str((cell.get("frames") or {}).get("tool") or "")
    if not tool:
        raise VerifyError("this cell names no tool frame; there is no tip to move")

    kin = kinematics(cell, held=held)
    spec = fk.build(cell, [tool])
    # The chain the Jacobian knows is the ARM's; the trajectory is over every
    # axis the verifier models, positioner included. Mapping by name rather
    # than by position, because the two orders are not the same and a silent
    # mismatch here would compute a Jacobian for the wrong pose.
    try:
        columns = [kin.axis_names.index(name) for name in spec.axis_names]
    except ValueError as exc:
        raise VerifyError(f"the arm's axes are not all in the trajectory: {exc}") from exc

    # CPU, like everything else the verifier computes: the Jacobian here is a
    # few dozen poses of a six-axis chain, the card buys nothing, and a CUDA
    # call would also keep the judging out of a forked worker process --
    # which is how `weld_trajopt_main` fans the winners out.
    device = torch.device("cpu")
    q = torch.as_tensor(np.asarray(waypoints)[:, columns], device=device, dtype=torch.float64)
    # `tip_local` is the TCP offset from `tooling.json`. It belongs here and
    # not in the chain: the torch is what traces the seam, and a Jacobian taken
    # at the flange describes a point 300 mm from the one being welded.
    jac = jacobian(spec, spec.link_names.index(tool), q, tip_local=tip_local)
    mob = directional(jac, torch.as_tensor(np.asarray(directions), device=device,
                                           dtype=torch.float64))

    limits = (profile.get("limits") or {})
    rates = [_rate(limits.get(name) or {}, "velocity") for name in spec.axis_names]
    qdot_max = None
    if all(r is not None for r in rates) and rates:
        qdot_max = torch.as_tensor([float(r) for r in rates], device=device,
                                   dtype=torch.float64)
    else:
        unverifiable.append(
            "travel mobility: the cell profile carries no joint velocity limits, so "
            "no travel speed can be certified deliverable")

    speed = torch.as_tensor(np.asarray(travel_speed_m_s), device=device, dtype=torch.float64)
    margin = travel_margin(mob, speed, qdot_max)

    violations: list[dict[str, Any]] = []
    for k in np.nonzero(mob.singular.cpu().numpy())[0][:5]:
        violations.append({"kind": "singular", "index": int(k),
                           "detail": "the seam tangent is not in the arm's reachable "
                                     "velocity space at this pose"})
    if qdot_max is not None:
        broken = np.nonzero(~margin["feasible"].cpu().numpy())[0]
        for k in broken[:5]:
            if bool(mob.singular[int(k)]):
                continue
            violations.append({
                "kind": "travel_speed", "index": int(k),
                "mu_m_per_rad": round(float(mob.mu[int(k)]), 6),
                "headroom": round(float(margin["headroom"][int(k)]), 4)})

    report = {
        "mu_min_m_per_rad": round(float(mob.mu.min()), 6),
        "mu_median_m_per_rad": round(float(mob.mu.median()), 6),
        "singular_waypoints": int(mob.singular.sum()),
        "checked_against_limits": bool(qdot_max is not None),
    }
    if qdot_max is not None:
        report["worst_headroom"] = round(float(margin["headroom"].min()), 4)
    return violations, unverifiable, report


# --------------------------------------------------------------------------
# Tracking: the same certificate, pointed at the seam instead of the meshes


# How far the linearity guards may drift before the bound is refused rather
# than trusted. A degree-1 segment makes the reference datum affine in the
# fraction EXACTLY, so the residual here is float64 noise on metre-scale
# vectors; 1e-9 m is six orders above that noise and six below any tolerance.
LINEARITY_TOL_M = 1e-9
# Rodrigues' sensitivity to its own AXIS: |d/dn R(n, t) v| <= |sin t| +
# 2(1 - cos t) <= 4, over the whole angle range. Used rather than the
# per-angle value because looseness costs bisection depth, never soundness.
AXIS_SENSITIVITY = 4.0
# Once a trajectory is out of tolerance the question stops being "does it
# pass" and becomes "by how much", because that number is what a slowdown or
# a seam split is sized against. So the recursion keeps refining any interval
# that could still hold something worse than the worst found so far -- branch
# and bound on the supremum -- and stops when no interval can beat it by more
# than this fraction. Pure yes/no bisection stops at the FIRST witness, which
# on the corner seam reported 7.5 mm for a trajectory that is 20 mm out.
SUP_SLACK = 0.02


class SeamTarget:
    """Where tool0 is supposed to be, as a curve in the WORK frame.

    The optimizer's target and the verifier's target must be the same contract or
    the certificate is judging something else -- so this composes it exactly
    as `weld_trajopt.py` does (seam frames, authored angles, placement, standoff
    along +u) and then, unlike the optimizer, answers questions about its
    DERIVATIVE that no sampling could answer.

    The three things the certificate needs beyond the value:

    - `breakpoints`, the fractions where the target stops being smooth. Two
      kinds: a segment join of the weld line, where the tangent is genuinely
      two-valued and the target JUMPS by `2 * standoff * sin(corner/2)`, and
      a knot of a piecewise-linear angle profile, where the direction merely
      kinks. Certifying across either would be bounding a function by a
      constant it does not have, so intervals are cut here first.
    - `lipschitz_f`, a bound on `|dT/df|` valid over a whole sub-interval.
    - `extent`, how far the target can be from the work frame's origin, which
      is what turns the positioner's joint rate into a target speed.
    """

    def __init__(self, seam: Any, placement_4x4: np.ndarray):
        self.seam = seam
        self.place = np.asarray(placement_4x4, dtype=np.float64)

    def _standoff_m(self, fractions: np.ndarray) -> np.ndarray:
        """The seam's own authored standoff, re-evaluated here per fraction --
        the same source the lattice and the optimizer pose at, so the
        certificate judges the target they actually held."""
        from weld_motion_planner.planner.dp_seam_search.engine.nodes import (
            authored_standoff_m,
        )

        return authored_standoff_m(self.seam, fractions)

    def _standoff_bound_m(self) -> float:
        """`sup` of the standoff over the whole seam. A piecewise-linear
        profile attains it at a knot or an end, so those are the probes."""
        spec = (getattr(self.seam, "process", None) or {}).get("standoff_mm") or {}
        candidates = [0.0, 1.0] + [float(k[0]) for k in spec.get("knots") or []]
        return float(self._standoff_m(np.asarray(candidates)).max())

    # -- value ----------------------------------------------------------

    def local(self, fractions: Any) -> np.ndarray:
        """(N, 3) tool0 targets in the work frame, placement folded in."""
        import torch

        from weld_motion_planner.geometry.seam_frame import apply_angles
        from weld_motion_planner.planner.dp_seam_search.engine.nodes import (
            authored_angles,
        )

        f = np.atleast_1d(np.asarray(fractions, dtype=np.float64))
        ft = torch.as_tensor(f, dtype=torch.float64)
        fr = self.seam.frames(ft)
        work_deg, travel_deg = authored_angles(self.seam, f)
        u = apply_angles(fr["t"], fr["r"],
                         torch.as_tensor(work_deg, dtype=torch.float64),
                         torch.as_tensor(travel_deg, dtype=torch.float64)).numpy()
        point = fr["pos_m"].numpy()
        point = point @ self.place[:3, :3].T + self.place[:3, 3]
        u = u @ self.place[:3, :3].T
        return point + self._standoff_m(f)[:, None] * u

    # -- structure ------------------------------------------------------

    def breakpoints(self) -> list[float]:
        span = self.seam.span
        start = float(span.get("start_s", 0.0))
        end = float(span.get("end_s", 1.0))
        length = (1.0 - start + end) if end < start else (end - start)
        if length <= 0.0:
            return []
        found: set[float] = set()
        curves = [self.seam.curve]
        if self.seam.rail is not None and len(self.seam.rail.segments) != len(self.seam.curve.segments):
            # Matching segment counts means the rail is read PER SEGMENT of
            # the weld line, so its joins already sit on the weld line's. A
            # differing count reads it at the shared `s`, and then its own
            # joins are breakpoints in their own right.
            curves.append(self.seam.rail)
        for curve in curves:
            for join in curve.joins_s.tolist():
                for candidate in (join, join - 1.0, join + 1.0):
                    f = (candidate - start) / length
                    if 1e-9 < f < 1.0 - 1e-9:
                        found.add(f)
        profiles = [(self.seam.orientation or {}).get(key) or {}
                    for key in ("work_angle_deg", "travel_angle_deg")]
        # The standoff kinks where its profile does, exactly like the angles:
        # `T = P + s(f) u`, so a knot of `s` is a knot of `T`.
        profiles.append((getattr(self.seam, "process", None) or {})
                        .get("standoff_mm") or {})
        for spec in profiles:
            if str(spec.get("kind") or "") == "piecewise_linear":
                for knot in spec.get("knots") or []:
                    f = float(knot[0])
                    if 1e-9 < f < 1.0 - 1e-9:
                        found.add(f)
        return sorted(found)

    def extent(self) -> float:
        """A bound on `|T|` about the work frame's origin.

        From the control polygon, not from samples: a B-spline lies inside
        the convex hull of its control points, so their placed norms bound
        the weld line everywhere, and the torch stands `standoff` off it.
        """
        best = 0.0
        for segment in self.seam.curve.segments:
            homo = segment.homogeneous.detach().cpu().numpy()
            points = homo[:, :3] / homo[:, 3:4]
            placed = points @ self.place[:3, :3].T + self.place[:3, 3]
            best = max(best, float(np.linalg.norm(placed, axis=1).max()))
        return best + self._standoff_bound_m()

    # -- the bound ------------------------------------------------------

    def lipschitz_f(self, lo: float, hi: float) -> float | None:
        """`sup |dT/df|` on `[lo, hi]`, or None if this cannot bound it.

        `[lo, hi]` lies inside one smooth piece (`breakpoints` cut the
        trajectory first), so one weld-line segment, one rail segment, and one
        linear run of every profile. `T = P + standoff * u`, and:

        - The weld POINT contributes exactly the seam's length `L`. `s` is
          normalized ARC LENGTH, so `|dP/ds|` is the curve's total length by
          definition and no bound is involved at all -- true at any degree.
        - The standoff contributes its own slope and, at the piece's LARGEST
          standoff, the turning of `u`. `u` is built from the tangent `t`,
          the datum `r` and the two authored angles (SEAM-MODEL section 5):

              r' = R(t, work) r,  b' = t x r',  u = R(b', -travel) r'

          so with `A = AXIS_SENSITIVITY` and `tau = |dt/df|`,

              |dr'/df| <= work' + A tau + |dr/df|
              |du/df|  <= travel' + (1 + A) |dr'/df| + A tau

          On a STRAIGHT segment `tau` is zero and the datum is affine, so
          `|dr/df|` is closed-form (`_straight_datum_rate`) -- the bound this
          certificate was first written with, kept exactly as it was.

          On a CURVED segment `tau = kappa * L`, with `kappa` bounded from
          the derivative control polygons (`_curvature_bound`), and the datum
          is bounded by the Lipschitz argument in `_curved_datum_rate`.
          `docs/issues/0008` is that bound; before it a curved weld line was
          refused here.

        None -- a refusal that travels into `unverifiable`, never a pass --
        where an ingredient does not hold: a rational curved segment (the
        hull argument is for non-rational splines only), a datum that
        passes through the tangent, a profile that is not linear on the
        piece as its breakpoints promised.
        """
        import torch

        from weld_motion_planner.planner.dp_seam_search.engine.nodes import (
            authored_angles,
        )

        width = hi - lo
        if width <= 0.0:
            return 0.0
        piece = self._piece_segment(lo, hi)
        if piece is None:
            return None
        index, u_lo, u_hi = piece
        segment = self.seam.curve.segments[index]
        straight = segment.degree == 1 and (
            self.seam.rail is None or all(seg.degree == 1 for seg in self.seam.rail.segments))
        # Three INTERIOR probes. They are not a sample of the maximum -- the
        # quantities they read are affine on this piece by construction, so two
        # of them determine the line exactly and the third exists to CHECK
        # that claim rather than assume it. A piece that fails the check is
        # refused below.
        probes = np.array([lo + width * t for t in (0.25, 0.5, 0.75)])
        ft = torch.as_tensor(probes, dtype=torch.float64)
        fr = self.seam.frames(ft)
        if bool(fr["degenerate"].any()):
            return None
        tangent = fr["t"].numpy()
        direction = self.seam.reference_direction(ft).numpy()

        work_deg, travel_deg = authored_angles(self.seam, probes)
        slopes = []
        for values in (np.asarray(work_deg), np.asarray(travel_deg)):
            if abs(float(values[1] - 0.5 * (values[0] + values[2]))) > 1e-9:
                return None
            slopes.append(abs(float(values[2] - values[0])) * 2.0 / width)
        work_rate, travel_rate = (math.radians(s) for s in slopes)

        # The standoff is a profile now: `T = P + s(f) u`, so `dT/df` gains
        # `s'` outright and the turning term is paid at the piece's LARGEST
        # standoff. Affine on this piece by the same argument as the angles
        # (breakpoints cut at its knots), checked rather than assumed, and the
        # sup of an affine `s` is at an end -- extrapolated exactly as `w` is.
        standoff = self._standoff_m(probes)
        if abs(float(standoff[1] - 0.5 * (standoff[0] + standoff[2]))) > 1e-9:
            return None
        standoff_rate = abs(float(standoff[2] - standoff[0])) * 2.0 / width
        standoff_hi = max(1.5 * float(standoff[0]) - 0.5 * float(standoff[2]),
                          1.5 * float(standoff[2]) - 0.5 * float(standoff[0]))

        length = self.seam.length_m
        if straight:
            r_rate = _straight_datum_rate(tangent, direction, width)
            tau = 0.0
        else:
            kappa = _curvature_bound(segment, u_lo, u_hi)
            if kappa is None:
                return None
            tau = kappa * length
            r_rate = self._curved_datum_rate(index, tangent[1], direction[1], width, tau)
        if r_rate is None:
            return None

        rotated = work_rate + AXIS_SENSITIVITY * tau + r_rate
        turn = travel_rate + (1.0 + AXIS_SENSITIVITY) * rotated + AXIS_SENSITIVITY * tau
        return length + standoff_rate + standoff_hi * turn

    def _piece_segment(self, lo: float, hi: float) -> tuple[int, float, float] | None:
        """The weld-line segment `[lo, hi]` lies on, and its parameter range.

        The low end is read departing and the high end arriving, so a piece
        that starts or ends on a join is placed on the segment it actually
        runs along. Two different answers mean the piece straddles a join the
        breakpoints should have cut at, and nothing is bounded across that.
        """
        import torch

        ends = torch.as_tensor([lo, hi], dtype=torch.float64)
        s = self.seam.s_at(ends)
        index, u = self.seam.curve.locate(s, torch.tensor([False, True]))
        if int(index[0]) != int(index[1]):
            return None
        u_lo, u_hi = sorted((float(u[0]), float(u[1])))
        return int(index[0]), u_lo, u_hi

    def _curved_datum_rate(self, index: int, t_mid: np.ndarray, d_mid: np.ndarray,
                           width: float, tau: float) -> float | None:
        """`sup |dr/df|` on a piece whose tangent turns at no more than `tau`.

        `r = w / |w|` with `w = D - (D.t) t` and `D` the raw datum, so
        `|dr/df| <= |dw/df| / |w|`, and since `t'` is perpendicular to `t`,

            |dw/df| <= |D'| + |D| tau.

        `|D'|` is a length rate and closed-form for each datum: zero for a
        fixed vector; for a rail read per segment, the rail point moves at its
        segment's length over the weld segment's, times `L`, and the weld
        point at `L`; for a rail read at the shared `s`, at the rail's total
        length times the span's share of `s`. `|D|` and `|w|` are measured at
        the piece's middle and carried to its ends by those same rates --
        Lipschitz, not sampling. The planner rescues a datum within
        `DEGENERATE_DOT` of the tangent with a different one, so a piece that
        could come that close is refused rather than bounded as if it could
        not.
        """
        from weld_motion_planner.geometry.seam_frame import DEGENERATE_DOT

        length = self.seam.length_m
        rail = self.seam.rail
        if rail is None:
            d_rate = 0.0
        elif len(rail.segments) == len(self.seam.curve.segments):
            weld = self.seam.curve.segments[index].length
            if weld <= 0.0:
                return None
            d_rate = length * (1.0 + rail.segments[index].length / weld)
        else:
            share = length / max(self.seam.curve.total, 1e-30)
            d_rate = length + share * rail.total
        half = 0.5 * width
        d_sup = float(np.linalg.norm(d_mid)) + d_rate * half
        w_mid = d_mid - float(d_mid @ t_mid) * t_mid
        w_rate = d_rate + d_sup * tau
        w_inf = float(np.linalg.norm(w_mid)) - w_rate * half
        if d_sup <= 0.0 or w_inf / d_sup <= math.sqrt(1.0 - DEGENERATE_DOT ** 2):
            return None
        return w_rate / w_inf


def _straight_datum_rate(tangent: np.ndarray, direction: np.ndarray,
                         width: float) -> float | None:
    """`sup |dr/df|` on a degree-1 piece: the tangent is constant, so
    `w = D - (D.t) t` is affine in the fraction and `|dr/df| <= |w'| / min|w|`
    with both closed-form."""
    if float(np.abs(tangent - tangent[1]).max()) > 1e-9:
        return None  # a straight segment whose tangent moves: not this piece
    along = (direction * tangent).sum(axis=1, keepdims=True)
    w = direction - along * tangent
    if float(np.abs(w[1] - 0.5 * (w[0] + w[2])).max()) > LINEARITY_TOL_M:
        return None
    # Extrapolate the probes back to the piece's own ends, exactly.
    w_lo = 1.5 * w[0] - 0.5 * w[2]
    w_hi = 1.5 * w[2] - 0.5 * w[0]
    w_rate = (w_hi - w_lo) / width
    floor = _segment_distance_to_origin(w_lo, w_hi)
    if floor <= 1e-12:
        return None  # the datum passes through the tangent: r is undefined
    return float(np.linalg.norm(w_rate)) / floor


def _curvature_bound(segment: Any, u_lo: float, u_hi: float) -> float | None:
    """`sup kappa` over `[u_lo, u_hi]` of one B-spline segment, rational or not.

    Bounded from Bezier control points, not samples. On a stretch of one knot
    span the homogeneous curve `A = (w C, w)` is a polynomial, and with
    `C = A / w` both derivatives are polynomials over powers of `w`:

        C'  = N / w^2,   N = A' w - w' A
        C'' = M / w^3,   M = (A'' w - w'' A) w - 2 w' N

    so `kappa = |C' x C''| / |C'|^3 = w |N x M| / |N|^3 <= w |M| / |N|^2`.
    A polynomial lies in the convex hull of its Bezier control points on the
    stretch, which bounds `w` and `|M|` above by their coefficients and `|N|`
    below by `min_i N_i . v` for any unit `v`, taken as the direction of
    their mean. No term depends on where the origin is, and a non-rational
    segment is the case `w = 1`, `N = C'`, `M = C''`. Where the lower bound
    is not positive the stretch is halved, down to `CURVATURE_SPLIT_DEPTH`,
    because a hull tightens toward the curve as its stretch shrinks.

    None -- a refusal -- where a weight can reach zero (the curve is not
    defined there) or the derivative can (the curve may stop, and curvature
    has no bound). `docs/issues/0008` has the history: the weld planner once
    refused every curved weld line here.
    """
    worst = 0.0
    for lo, hi in _span_stretches(segment.knots, segment.degree, len(segment.homogeneous), u_lo, u_hi):
        bound = _curvature_on(segment, lo, hi, CURVATURE_SPLIT_DEPTH)
        if bound is None:
            return None
        worst = max(worst, bound)
    return worst


# How many times a stretch whose derivative hull reaches the origin is halved
# before the curvature bound is refused. Eight halvings take a whole span to
# 1/256 of itself; a derivative still not separated from zero there is, for
# any weld line a person drew, a cusp.
CURVATURE_SPLIT_DEPTH = 8


def _curvature_on(segment: Any, lo: float, hi: float, depth: int) -> float | None:
    import torch

    degree = segment.degree
    weights = _bezier_points(segment, degree, segment.knots, segment.homogeneous, lo, hi)[:, 3]
    if float(weights.min()) <= 0.0:
        return None
    # N and M at nodes, from the homogeneous curve and its derivatives; then
    # their Bezier coefficients at the degrees they can have (2p - 1, 3p - 2).
    def fit(order: int) -> np.ndarray:
        t = _chebyshev(order + 1)
        u = torch.as_tensor(lo + (hi - lo) * t, dtype=segment.homogeneous.dtype,
                            device=segment.homogeneous.device)
        a = segment._de_boor(u, degree, segment.knots, segment.homogeneous).detach().cpu().numpy()
        a1 = segment._de_boor(u, *segment._d1).detach().cpu().numpy()
        a2 = segment._de_boor(u, *segment._d2).detach().cpu().numpy()
        w, w1, w2 = a[:, 3:4], a1[:, 3:4], a2[:, 3:4]
        n = a1[:, :3] * w - w1 * a[:, :3]
        m = (a2[:, :3] * w - w2 * a[:, :3]) * w - 2.0 * w1 * n
        return t, n, m

    t_n, n_values, _ = fit(2 * degree - 1)
    n = _bernstein_fit(2 * degree - 1, t_n, n_values)
    mean = n.mean(axis=0)
    size = float(np.linalg.norm(mean))
    floor = float((n @ (mean / size)).min()) if size > 0.0 else 0.0
    if floor <= 0.0:
        if depth <= 0:
            return None
        mid = 0.5 * (lo + hi)
        left = _curvature_on(segment, lo, mid, depth - 1)
        right = None if left is None else _curvature_on(segment, mid, hi, depth - 1)
        return None if right is None else max(left, right)
    order = max(3 * degree - 2, 0)
    t_m, _, m_values = fit(order)
    m = _bernstein_fit(order, t_m, m_values)
    return float(weights.max()) * float(np.linalg.norm(m, axis=1).max()) / floor ** 2


def _chebyshev(count: int) -> np.ndarray:
    """`count` Chebyshev nodes strictly inside [0, 1]."""
    return 0.5 - 0.5 * np.cos((2.0 * np.arange(count) + 1.0) * math.pi / (2.0 * count))


def _bernstein_fit(degree: int, t: np.ndarray, values: np.ndarray) -> np.ndarray:
    """Bezier control points of the degree-`degree` polynomial through
    `values` at `t` (`degree + 1` of each)."""
    if degree == 0:
        return values[:1]
    basis = np.array([[math.comb(degree, i) * x ** i * (1.0 - x) ** (degree - i)
                       for i in range(degree + 1)] for x in t])
    return np.linalg.solve(basis, values)


def _span_stretches(knots: Any, degree: int, count: int, u_lo: float, u_hi: float):
    """`[u_lo, u_hi]` cut at the knots of a spline, one stretch per span, so
    each stretch is a single polynomial."""
    k = knots.detach().cpu().numpy()
    inner = sorted({float(x) for x in k[degree: count + 1] if u_lo < float(x) < u_hi})
    edges = [u_lo, *inner, u_hi]
    return [(a, b) for a, b in zip(edges[:-1], edges[1:]) if b > a]


def _bezier_points(segment: Any, degree: int, knots: Any, points: Any,
                   lo: float, hi: float) -> np.ndarray:
    """(degree + 1, 4) Bezier control points of a homogeneous spline on
    `[lo, hi]`, a stretch inside one knot span.

    By interpolation at interior Chebyshev nodes: the stretch is one
    polynomial of `degree`, so `degree + 1` values determine it, and nodes
    strictly inside keep de Boor on the span that owns the stretch rather
    than a neighbour meeting it at a knot. The Bernstein collocation matrix
    is well conditioned for the degrees a weld line carries.
    """
    import torch

    t = _chebyshev(degree + 1)
    u = torch.as_tensor(lo + (hi - lo) * t, dtype=points.dtype, device=points.device)
    values = segment._de_boor(u, degree, knots, points).detach().cpu().numpy()
    return _bernstein_fit(degree, t, values)


def _worse(found: dict[int, dict[str, Any]], segment: int, t_lo: float, t_hi: float,
           f_lo: float, f_hi: float, error: float, throughout: bool) -> None:
    """Keep the worst finding PER TRAJECTORY SEGMENT.

    The search for the supremum visits an interval many times on its way
    down, and every visit past the tolerance is a fresh witness of the same
    fault. What an operator needs is which segments miss and by how much, so
    the record is keyed by segment and only ever gets worse.
    """
    previous = found.get(segment)
    if previous is not None and previous["witnessed_m"] >= error:
        previous["throughout"] = previous["throughout"] or throughout
        return
    found[segment] = {
        "segment": segment,
        "t": [float(t_lo), float(t_hi)],
        "fraction": [float(f_lo), float(f_hi)],
        "witnessed_m": float(error),
        # True when the interval is out of tolerance at EVERY point of it,
        # not merely at the instant that was measured.
        "throughout": bool(throughout) or bool(previous and previous["throughout"]),
    }


def _segment_distance_to_origin(a: np.ndarray, b: np.ndarray) -> float:
    """`min |(1-t) a + t b|` over `[0, 1]`, closed form."""
    d = b - a
    dd = float(d @ d)
    t = 0.0 if dd <= 0.0 else float(np.clip(-(a @ d) / dd, 0.0, 1.0))
    return float(np.linalg.norm(a + t * d))


def chain_reach(kin: Kinematics, joint_name: str, link: str, extent_m: float) -> float | None:
    """How far a point near `link` can be from `joint_name`'s axis.

    `reach_radii` answers this for MESHES, by name, walking down from a joint
    to everything it carries. This answers it for a single point at a stated
    distance from one link's origin, walking UP from that link -- which is
    what the tracking bound needs, because the two points it follows are the
    electrode tip and the target on the workpiece, and neither is a body.

    Zero for a joint that is not an ancestor of `link`: it does not move the
    point, which is a fact and not an omission. None for a prismatic joint on
    the chain, whose travel is not bounded by the geometry.
    """
    total = float(extent_m)
    node = link
    while node in kin.parent_of:
        joint = next(j for j in kin.joints if str(j["child"]) == node)
        if str(joint["name"]) == joint_name:
            # The axis passes through this joint's CHILD origin, so the
            # accumulated offsets already measure from the axis. Adding this
            # joint's own offset would double-count the link it sits on.
            return total
        if str(joint.get("type") or "fixed") == "prismatic":
            return None
        total += float(np.linalg.norm(np.asarray(joint.get("xyz_m") or (0, 0, 0), dtype=np.float64)))
        node = kin.parent_of[node]
    return 0.0


def _hermite(q0: np.ndarray, q1: np.ndarray, d0: np.ndarray, d1: np.ndarray,
             h: float, u: float) -> np.ndarray:
    u2, u3 = u * u, u * u * u
    return ((2 * u3 - 3 * u2 + 1) * q0 + h * (u3 - 2 * u2 + u) * d0
            + (-2 * u3 + 3 * u2) * q1 + h * (u3 - u2) * d1)


def _velocity_extreme(q0: np.ndarray, q1: np.ndarray, d0: np.ndarray, d1: np.ndarray,
                      h: float, lo: float, hi: float) -> np.ndarray:
    """Per-axis `max |q̇|` on `[lo, hi]`, in closed form and not by sampling.

    A cubic Hermite's derivative is a QUADRATIC, so its extremes on any
    interval are the two ends and at most one vertex, all three of which are
    arithmetic. This is the half of the certificate that makes the tip's
    Lipschitz constant exact rather than estimated.
    """
    delta = (q0 - q1) / h
    a = 6.0 * delta + 3.0 * (d0 + d1)
    b = -6.0 * delta - 4.0 * d0 - 2.0 * d1
    c = d0

    def at(u: np.ndarray | float) -> np.ndarray:
        return a * u * u + b * u + c

    best = np.maximum(np.abs(at(lo)), np.abs(at(hi)))
    with np.errstate(divide="ignore", invalid="ignore"):
        vertex = np.where(np.abs(a) > 1e-30, -b / (2.0 * a), np.full_like(a, lo - 1.0))
    inside = np.isfinite(vertex) & (vertex > lo) & (vertex < hi)
    if inside.any():
        clamped = np.where(inside, vertex, lo)
        best = np.where(inside, np.maximum(best, np.abs(at(clamped))), best)
    return best


@dataclass
class TrackingCertificate:
    """What the verifier proved about the tip's distance from where it belongs.

    `bound_m` is an UPPER bound on the worst tracking error anywhere on the
    trajectory -- not the worst error at any sample, which is a different and
    weaker sentence. `violations` are proven from BELOW: an interval on which
    the error is greater than the tolerance at every point, so no refinement
    of the same trajectory can rescue it.
    """

    tolerance_m: float
    clear: bool
    bound_m: float
    # The worst error actually SEEN, at an instant the certificate evaluated.
    # It matters because `bound_m` goes coarse the moment a violation is
    # witnessed: the recursion stops there rather than refining a fact it
    # already has, so the bound over that leaf keeps its whole travel term.
    # A clear certificate reports a tight `bound_m` and this is the interior
    # detail; a failing one reports a loose `bound_m` and this is the number
    # to read.
    worst_witnessed_m: float = 0.0
    violations: list[dict[str, Any]] = field(default_factory=list)
    # `violations` is capped for readability; this is the real count of
    # trajectory segments that miss.
    segments_out_of_tolerance: int = 0
    undecided: list[dict[str, Any]] = field(default_factory=list)
    unverifiable: list[str] = field(default_factory=list)
    pieces: int = 0
    evaluations: int = 0

    @property
    def ok(self) -> bool:
        return self.clear and not self.unverifiable and not self.undecided

    def summary(self) -> dict[str, Any]:
        return {
            "verdict": ("PASS" if self.ok else
                        ("FAIL" if self.violations else "REFUSED")),
            "tolerance_mm": round(self.tolerance_m * 1000.0, 4),
            "bound_mm": round(self.bound_m * 1000.0, 4),
            "worst_seen_mm": round(self.worst_witnessed_m * 1000.0, 4),
            "segments_out": self.segments_out_of_tolerance,
            "undecided": len(self.undecided),
            "unverifiable": self.unverifiable,
            "pieces": self.pieces,
            "evaluations": self.evaluations,
        }


def certify_tracking(kin: Kinematics, cell: dict[str, Any], target: SeamTarget,
                     waypoints: np.ndarray, velocities: np.ndarray,
                     times_s: np.ndarray, fractions: np.ndarray, *,
                     tolerance_m: float,
                     max_depth: int = MAX_DEPTH) -> TrackingCertificate:
    """Prove the tip tracks the seam BETWEEN the knots, or say where it does not.

    The collision certificate's own pattern, with the pair of bodies replaced
    by a pair of points. Between two knots the trajectory is a cubic Hermite,
    so `q̇` is a quadratic with closed-form extremes; the per-link reach radii
    turn those into a bound on how fast the tip can move; the seam side is an
    exact curve whose own `|dT/df|` is bounded by `SeamTarget`. Together they
    give a closing speed `S` for a whole interval, and one distance measured
    at its midpoint then says something about every instant in it:

        e(t) <= e(mid) + S * (hi - lo) / 2      for all t in [lo, hi]
        e(t) >= e(mid) - S * (hi - lo) / 2

    The first line certifies a PASS over the interval, the second certifies a
    FAILURE over it, and an interval that gets neither is halved. The
    optimizer is allowed to sample its tracking cost -- it is not trusted
    anyway. This is, so it may not.
    """
    waypoints = np.asarray(waypoints, dtype=np.float64)
    velocities = np.asarray(velocities, dtype=np.float64)
    times_s = np.asarray(times_s, dtype=np.float64)
    fractions = np.asarray(fractions, dtype=np.float64)
    if waypoints.shape != velocities.shape:
        raise VerifyError("the knots and their velocities must have the same shape")
    if len(waypoints) != len(times_s) or len(waypoints) != len(fractions):
        raise VerifyError("every knot needs a time and a fraction")

    frames_map = cell.get("frames") or {}
    tool = str(frames_map.get("tool") or "")
    work = str(frames_map.get("work") or "")
    if not tool or not work:
        raise VerifyError("tracking needs both a tool frame and a work frame")

    unverifiable: list[str] = []
    tip_reach = np.zeros(kin.num_dof)
    work_reach = np.zeros(kin.num_dof)
    extent = target.extent()
    for i, name in enumerate(kin.axis_names):
        for store, link, near in ((tip_reach, tool, 0.0), (work_reach, work, extent)):
            got = chain_reach(kin, name, link, near)
            if got is None:
                unverifiable.append(
                    f"tracking: {name} is prismatic on the chain to {link!r} and its "
                    "travel is not bounded by the geometry")
                got = 0.0
            store[i] = got
    if unverifiable:
        return TrackingCertificate(tolerance_m=tolerance_m, clear=False,
                                   bound_m=float("inf"), unverifiable=unverifiable)

    cuts = target.breakpoints()
    violations: dict[int, dict[str, Any]] = {}
    undecided: list[dict[str, Any]] = []
    bound = 0.0
    seen = 0.0
    pieces = evaluations = 0

    # Every smooth piece of the whole trajectory, with the derivative bound
    # that governs it. Intervals are cut where the target stops being smooth
    # -- a bound taken across a corner would describe a function that jumps.
    #
    # ONE derivative bound per piece, reused by every node below it: a
    # supremum over a subset never exceeds the supremum over the set, so
    # inheriting is sound, and evaluating the bound costs several curve solves.
    smooth: list[dict[str, Any]] = []
    for k in range(len(waypoints) - 1):
        h = float(times_s[k + 1] - times_s[k])
        if h <= 0.0:
            raise VerifyError("the trajectory's times are not strictly increasing")
        f0 = float(fractions[k])
        df = float(fractions[k + 1]) - f0
        edges = [0.0, 1.0]
        if abs(df) > 0.0:
            edges += [u for u in ((cut - f0) / df for cut in cuts)
                      if 1e-12 < u < 1.0 - 1e-12]
        edges = sorted(set(edges))
        for piece_lo, piece_hi in zip(edges[:-1], edges[1:], strict=True):
            f_a, f_b = f0 + df * piece_lo, f0 + df * piece_hi
            grip = target.lipschitz_f(min(f_a, f_b), max(f_a, f_b))
            if grip is None:
                unverifiable.append(
                    f"tracking: no derivative bound exists for the seam target on "
                    f"segment {k} of the trajectory (fractions "
                    f"{min(f_a, f_b):.4f}..{max(f_a, f_b):.4f})")
                bound = float("inf")
                break
            smooth.append({"k": k, "h": h, "f0": f0, "df": df, "grip": grip,
                           "q0": waypoints[k], "q1": waypoints[k + 1],
                           "d0": velocities[k], "d1": velocities[k + 1]})
        if unverifiable:
            break

    # The bisection walks BY LEVEL rather than depth-first, so a whole
    # frontier's worth of midpoints reaches the seam evaluator in one call.
    # Point by point it ran a batched evaluator at batch 1 and cost 3.84 s of
    # a 4.4 s stage (`BENCHMARKS.md` 11.4). Nothing about the decision changes
    # -- each node is judged by the same three lines -- only the ORDER they
    # are visited in, which the supremum search notices: `seen` climbs on a
    # different schedule, so the node count moves a little either way. Any
    # covering of leaves is a valid certificate, so that is a cost difference
    # and not a soundness one.
    frontier = [(i, 0.0, 1.0, 0) for i in range(len(smooth))]
    while frontier:
        fractions_at = np.array(
            [smooth[i]["f0"] + smooth[i]["df"] * 0.5 * (lo + hi)
             for i, lo, hi, _ in frontier])
        aims = target.local(fractions_at)
        children: list[tuple[int, float, float, int]] = []
        for node, (i, lo, hi, depth) in enumerate(frontier):
            piece = smooth[i]
            k, h, f0, df = piece["k"], piece["h"], piece["f0"], piece["df"]
            mid = 0.5 * (lo + hi)
            f_lo, f_hi = f0 + df * lo, f0 + df * hi
            pieces += 1

            q_mid = _hermite(piece["q0"], piece["q1"], piece["d0"], piece["d1"], h, mid)
            transforms = kin.transforms(q_mid)
            evaluations += 1
            place = transforms[work]
            aim = place[:3, :3] @ aims[node] + place[:3, 3]
            error = float(np.linalg.norm(transforms[tool][:3, 3] - aim))
            seen = max(seen, error)

            rate = _velocity_extreme(piece["q0"], piece["q1"], piece["d0"],
                                     piece["d1"], h, lo, hi)
            speed = float(tip_reach @ rate) + float(work_reach @ rate)
            speed += piece["grip"] * abs(df) / h
            travel = speed * (hi - lo) * h * 0.5

            # Only LEAVES contribute to the reported bound. An interval that
            # gets bisected has said nothing yet -- its travel term covers
            # ground its children will cover more tightly, and counting it
            # would report the recursion's first guess as the answer
            # (measured: 61 mm on a seam that certifies at 0.4).
            # An interval is finished when it can hold nothing worse than the
            # tolerance AND nothing worse than the worst already found. The
            # second half is what turns a pass/fail bisection into a search
            # for the supremum; while everything tracks, `seen` stays under
            # the tolerance and the rule is exactly the tolerance test.
            done = error + travel <= max(tolerance_m, seen * (1.0 + SUP_SLACK))
            if done or depth >= max_depth:
                bound = max(bound, error + travel)
                if error > tolerance_m:
                    _worse(violations, k, times_s[k] + lo * h, times_s[k] + hi * h,
                           min(f_lo, f_hi), max(f_lo, f_hi), error,
                           error - travel > tolerance_m)
                elif not done:
                    undecided.append({
                        "segment": k,
                        "t": [float(times_s[k] + lo * h), float(times_s[k] + hi * h)],
                        "error_at_most_m": error + travel,
                    })
                continue
            children.append((i, lo, mid, depth + 1))
            children.append((i, mid, hi, depth + 1))
        frontier = children

    return TrackingCertificate(
        tolerance_m=tolerance_m,
        clear=not violations and not undecided and not unverifiable,
        bound_m=bound,
        worst_witnessed_m=seen,
        violations=sorted(violations.values(), key=lambda v: -v["witnessed_m"])[:10],
        segments_out_of_tolerance=len(violations),
        undecided=undecided[:10],
        unverifiable=unverifiable[:1],
        pieces=pieces,
        evaluations=evaluations,
    )


# --------------------------------------------------------------------------
# The entry point


def _certify_one(payload: tuple[Any, ...]) -> dict[str, Any]:
    kin, geom, radii, q0, q1, d0, d1, duration, margin, index = payload
    out = certify_segment(kin, geom, radii, q0, q1, d0=d0, d1=d1,
                          duration_s=duration, margin_m=margin)
    out["segment"] = index
    return out


def verify(cell: dict[str, Any], geom: Geometry, waypoints: np.ndarray, *,
           profile: dict[str, Any] | None = None,
           times_s: np.ndarray | None = None,
           # Joint velocities at the waypoints. With them the trajectory
           # between two waypoints is the cubic Hermite the synthesizer
           # actually emits; without them it is the straight line, which is
           # what a trajectory of bare waypoints means.
           velocities: np.ndarray | None = None,
           # T1's inputs: the seam tangent in the WORLD frame at each waypoint,
           # and the travel speed the program asks for there. Absent, T1 comes
           # back as unverifiable rather than as passed.
           directions: np.ndarray | None = None,
           travel_speed_m_s: np.ndarray | None = None,
           tip_local: Any = None,
           held: dict[str, float] | None = None,
           margin_m: float = DEFAULT_VERIFY_MARGIN_M,
           clearance: bool = True,
           screen_m: float = DEFAULT_SCREEN_M,
           workers: int = 0,
           # A pool LENT by the caller, for a caller judging many trajectories
           # at once: the segments fan out across it in `POOL_TASKS` chunks,
           # so one long trajectory shares the pool's width instead of holding
           # its wall time hostage. `workers` spawns a pool for this call
           # alone; `pool` outlives it, and wins.
           pool: Any = None,
           # Where the clearance REPORT reads, when that is not the knots: a
           # caller certifying a curve on its pieces still wants the worst
           # clearance read along the wire it emits. The certificate and the
           # limits judge `waypoints`; this only moves the report.
           clearance_waypoints: np.ndarray | None = None,
           _free_space_motion: FreeSpaceMotion | None = None) -> Verdict:
    """Judge a trajectory. Fails closed on everything it cannot check."""
    waypoints = np.asarray(waypoints, dtype=np.float64)
    if waypoints.ndim != 2 or len(waypoints) < 2:
        raise VerifyError("a trajectory is at least two waypoints of shape (N, dof)")

    kin = kinematics(cell, held=held)
    if waypoints.shape[1] != kin.num_dof:
        raise VerifyError(
            f"trajectory has {waypoints.shape[1]} axes, this cell has {kin.num_dof}: "
            f"{kin.axis_names}")
    if pool is not None:
        # A caller lending a pool is judging many trajectories at once, from
        # threads, over one `Geometry`. The coal objects cache poses in
        # place, so a shared instance would let two clearance reports place
        # each other's bodies; a private cache costs one BVH build and makes
        # concurrent calls sound.
        geom = Geometry(geom.meshes, geom.attached_to, geom.local, geom.pairs)
    radii = reach_radii(kin, geom)

    if velocities is not None:
        velocities = np.asarray(velocities, dtype=np.float64)
        if velocities.shape != waypoints.shape:
            raise VerifyError("the waypoints and their velocities must have the same shape")
        if times_s is None:
            raise VerifyError(
                "velocities without times say nothing: the curve between two "
                "waypoints depends on how long it takes to get there")
    elif _free_space_motion is not None:
        raise VerifyError(
            "free-space verification requires emitted Hermite knot velocities"
        )

    # Generic/M5 verification retains its current rate semantics. M6 is the
    # only caller that can turn an otherwise-unverifiable weld-only T1 check
    # into not-applicable, so it must also judge the emitted Hermite rates
    # before that transition can produce PASS.
    violations, unverifiable = check_limits(
        kin,
        cell,
        profile or {},
        waypoints,
        times_s,
        knot_velocities=velocities if _free_space_motion is not None else None,
    )

    travel_report: dict[str, Any]
    if _free_space_motion is None:
        travel_bad, travel_missing, travel_report = check_travel(
            cell, profile or {}, waypoints, directions, travel_speed_m_s,
            tip_local=tip_local, held=held)
        violations.extend(travel_bad)
        unverifiable.extend(travel_missing)
    else:
        if _free_space_motion not in _FREE_SPACE_MOTIONS:
            raise VerifyError(f"unknown free-space motion: {_free_space_motion!r}")
        if directions is not None or travel_speed_m_s is not None:
            raise VerifyError(
                "free-space verification cannot carry a seam direction or travel speed"
            )
        travel_report = {
            "status": "not_applicable",
            "motion": _free_space_motion,
        }

    spans = np.ones(len(waypoints) - 1)
    if times_s is not None and len(times_s) == len(waypoints):
        spans = np.diff(np.asarray(times_s, dtype=np.float64))
    elif velocities is not None:
        raise VerifyError("every waypoint needs a time before its velocity means anything")
    work = [(kin, geom, radii, waypoints[i], waypoints[i + 1],
             None if velocities is None else velocities[i],
             None if velocities is None else velocities[i + 1],
             float(spans[i]), margin_m, i)
            for i in range(len(waypoints) - 1)]
    if pool is not None and len(work) > 1:
        size = max(1, math.ceil(len(work) / POOL_TASKS))
        results = list(pool.map(_certify_one, work, chunksize=size))
    elif workers and workers > 1:
        with ProcessPoolExecutor(max_workers=workers) as own:
            results = list(own.map(_certify_one, work, chunksize=8))
    else:
        results = [_certify_one(item) for item in work]

    collisions = [r for r in results if not r["clear"]]
    queries = sum(int(r["queries"]) for r in results)

    worst_m = worst_at = None
    if clearance:
        reported = waypoints if clearance_waypoints is None else np.asarray(
            clearance_waypoints, dtype=np.float64)
        worst_m, worst_at, extra = report_clearance(kin, geom, reported,
                                                    screen_m=screen_m)
        queries += extra

    return Verdict(
        passed=not collisions and not violations,
        collisions=collisions,
        limit_violations=violations,
        unverifiable=unverifiable,
        worst_clearance_m=worst_m,
        clearance_floor_m=screen_m if (clearance and worst_m is None) else None,
        worst_at=worst_at,
        segments=len(work),
        queries=queries,
        travel=travel_report,
    )


def verify_free_space(
        cell: dict[str, Any], geom: Geometry, waypoints: np.ndarray, *,
        motion: FreeSpaceMotion,
        times_s: np.ndarray,
        velocities: np.ndarray,
        profile: dict[str, Any] | None = None,
        held: dict[str, float] | None = None,
        margin_m: float = DEFAULT_VERIFY_MARGIN_M,
        clearance: bool = True,
        screen_m: float = DEFAULT_SCREEN_M,
        workers: int = 0,
        pool: Any = None,
        clearance_waypoints: np.ndarray | None = None) -> Verdict:
    """Judge one M6 link with weld-only travel mobility not applicable.

    This is deliberately narrower than a public mode flag on ``verify``: only
    the three M6 move kinds can select it, and their emitted Hermite rates are
    checked exactly before the verdict can become PASS.
    """
    return verify(
        cell,
        geom,
        waypoints,
        profile=profile,
        times_s=times_s,
        velocities=velocities,
        held=held,
        margin_m=margin_m,
        clearance=clearance,
        screen_m=screen_m,
        workers=workers,
        pool=pool,
        clearance_waypoints=clearance_waypoints,
        _free_space_motion=motion,
    )


# --------------------------------------------------------------------------
# Self-test


BLADE_HALF_M = 0.002
PLATE_HALF_M = 0.002
PLATE_ANGLE_RAD = 0.75


def _demo() -> tuple[dict[str, Any], Geometry]:
    """A one-axis arm and a plate it can swing through.

    Deliberately small and analytic, so the certificate's answers can be
    checked against arithmetic rather than against another implementation of
    the same idea. But not deliberately EASY: both bodies are 4 mm thick and
    the plate stands edge-on to the sweep, which is the real geometry -- a
    torch nozzle and a clamp -- rather than a shape chosen to be easy to hit.
    The angular window in which they overlap is about
    `(4 mm + 4 mm) / 600 mm = 0.013 rad` out of a 1.1 rad swing, so uniform
    sampling needs of order 80 waypoints to land inside it. That gap between
    "80 samples" and "one certificate" is what this file is for.
    """
    from weld_motion_planner.robot_cell.collision.scene import box_mesh

    cell = {
        "frames": {"base": "world", "tool": "rod", "work": "world"},
        "links": [{"name": n} for n in ("world", "rod")],
        "joints": [
            {"name": "J1", "parent": "world", "child": "rod", "type": "revolute",
             "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0], "axis_xyz": [0, 0, 1],
             "limits_rad": {"lower": -3.2, "upper": 3.2}},
        ],
    }
    blade = box_mesh([0.0, -BLADE_HALF_M, -0.05], [1.0, BLADE_HALF_M, 0.05])
    # A plate standing along the ray at PLATE_ANGLE_RAD, thin across the sweep.
    plate = box_mesh([0.3, -PLATE_HALF_M, -0.2], [0.9, PLATE_HALF_M, 0.2])
    rot = _axis_rotation(np.array([0.0, 0.0, 1.0]), PLATE_ANGLE_RAD)
    plate = type(plate)(vertices=plate.vertices @ rot.T, faces=plate.faces)

    geom = Geometry(
        meshes={"blade": blade, "plate": plate},
        attached_to={"blade": "rod", "plate": "world"},
        local={"blade": np.eye(4), "plate": np.eye(4)},
        pairs=[("blade", "plate")],
    )
    return cell, geom


def self_test() -> int:
    try:
        import coal  # noqa: F401
    except ImportError:
        print("verify: coal is not installed; the verifier cannot run")
        return 1

    cell, geom = _demo()
    kin = kinematics(cell)
    radii = reach_radii(kin, geom)
    # The far CORNER of the rod, not the far face: 1 m along x and 30 mm off
    # in each of y and z.
    # The far CORNER of the blade, not the far face: 1 m along x, 2 mm off in
    # y and 50 mm in z.
    expect = math.sqrt(1.0 ** 2 + BLADE_HALF_M ** 2 + 0.05 ** 2)
    if abs(radii["J1"] - expect) > 1e-9:
        print(f"verify: FAIL, J1's reach is {radii['J1']:.6f}, expected {expect:.6f}")
        return 1
    print(f"verify: J1 reach {radii['J1'] * 1000:.1f} mm")

    # 1. A move that stays clear of the plate.
    clear = verify(cell, geom, np.array([[-1.5], [-0.2]]))
    print(f"  clear move:      {clear.summary()}")
    if not clear.passed:
        print("verify: FAIL, a clear move was refused")
        return 1

    # 2. THE test. The blade ends up on the far side of the plate having gone
    #    straight through it, and both endpoints are clear. Sampling the
    #    endpoints passes this; the verifier must not.
    # Asymmetric about the plate ON PURPOSE. Centred, the very first midpoint
    # query would land on the contact and the certificate would look like it
    # got lucky rather than like it bisected.
    a, b = PLATE_ANGLE_RAD - 0.22, PLATE_ANGLE_RAD + 0.88
    for angle in (a, b):
        one = verify(cell, geom, np.array([[angle], [angle + 1e-9]]))
        if not one.passed:
            print(f"verify: FAIL, the endpoint at {angle} rad is meant to be clear")
            return 1
    tunnel = verify(cell, geom, np.array([[a], [b]]))
    print(f"  through it:      {tunnel.summary()}")
    if tunnel.passed:
        print("verify: FAIL, the rod passed through the plate and the verifier said yes")
        return 1
    if tunnel.collisions[0]["reason"] != "collision":
        print(f"verify: FAIL, expected a collision, got {tunnel.collisions[0]}")
        return 1

    # 3. Sampling, for the record: how many uniform samples it takes to notice.
    objects = geom.objects()
    for n in (2, 8, 32, 128, 512):
        found = False
        for t in np.linspace(a, b, n).reshape(-1, 1):
            frames = kin.transforms(t)
            for body, link in geom.attached_to.items():
                _place(objects[body], frames[link] @ geom.local[body])
            if _closer_than(objects, "blade", "plate", 0.0):
                found = True
                break
        print(f"  {n:4d} samples:    {'catches it' if found else 'MISSES IT'}"
              f"   ({tunnel.queries} certificate queries caught it)")

    # 4. A position-limit violation.
    over = verify(cell, geom, np.array([[-1.5], [-4.0]]))
    print(f"  past the limit:  {over.summary()}")
    if over.passed or not any(v["kind"] == "position" for v in over.limit_violations):
        print("verify: FAIL, a move past the position limit was not caught")
        return 1

    # 5. Rates are not checkable on a cell with no rate limits, and that must
    #    show up as a refusal rather than as a pass.
    if not clear.unverifiable or clear.ok:
        print("verify: FAIL, missing rate limits were reported as verified")
        return 1
    print(f"  unverifiable:    {clear.unverifiable}")

    print("verify: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--json", action="store_true", help="emit the verdict as JSON")
    args = parser.parse_args(argv)
    if args.json:
        print(json.dumps({"self_test": self_test()}, indent=2))
        return 0
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"verify: {exc}", file=sys.stderr)
        raise SystemExit(1)
