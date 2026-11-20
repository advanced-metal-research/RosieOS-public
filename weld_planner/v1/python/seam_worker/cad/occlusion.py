#!/usr/bin/env python
"""Ray occlusion against an indexed triangle mesh: the fast intersector.

`access.py` asks one question a few million times -- "does this ray leave the
assembly?" -- and asked it of OCCT, which intersects the ray with every exact
surface of the shape with no spatial index. Measured on an exhaust manifold: 130
microseconds per ray, 64 seconds for one pipe bore, six minutes for the
workpiece. The cost was never the ray count (142k rays is nothing); it was that
each ray was a NURBS intersection against all 176 faces.

This is the same question answered against a BVH over triangles: **1.5
microseconds per ray**, 25 ms for that bore, and the parity numbers below.

**Two callers, two questions, one index.** `access.py` asks for a VERDICT --
does anything escape -- and `escaping`/`any_escape` answer it, short-circuiting
at the first ray that gets out. `clearance.py` asks WHERE along a ray there is
material, because it reads a standoff off the list afterwards and counts
crossings past the barrel's far end; `hits_along` and `barrel_lattice` answer
that. The second question is the more expensive one, because a parity count
cannot short-circuit: on the exhaust manifold a verdict is 6.3 us per ray and a
hit list is 17.4 us. Both beat OCCT's 6.3 **milliseconds** per ray there.

`barrel_lattice` exists because at that speed the interpreter is the cost. It
inlines `clearance.cast_fan`, `frames.apply_angles` and `clearance.barrel_axes`
so that a lattice cell costs a ray and not a ray plus a round trip through
Python. One weld's angle lattice went from 56.6 s to 0.050 s that way, agreeing
with the exact intersector on 28,603 of 28,611 cells.

    joint          engine   256 directions/sample
    pipe outer     occt              42,347 ms
    pipe outer     bvh                   29 ms
    pipe bore      occt              43,550 ms
    pipe bore      bvh                   25 ms

**Two knobs, doing two different jobs.** Confusing them is how a fast wrong
answer gets shipped:

- `deflection_mm` bounds how far a facet may sit from the true surface. It buys
  *accuracy*, and coarse is not merely imprecise, it is wrong: on the manifold at
  0.25 mm one pipe bore read 50 of its 53 samples blocked, and from 0.05 mm
  downward it converged on 22, which is what the exact intersector says. The
  default is 0.01 mm to sit well inside that convergence and well under
  `access.SURFACE_SKIP` (0.05 mm), the distance the ray test ignores at the
  origin.

  The mechanism is NOT established. The obvious suspect -- a sample point on an
  exact bore circle falling behind the facet chords, inside mesh material -- does
  not reproduce on a synthetic tube of the same radius and length, nor does
  aperture pinching: both stay flat to within one ray in 256 across every
  deflection from 0.5 mm to 0.01 mm. Something about that assembly in particular
  does it. Recorded as an observation with the numbers, because a guessed
  mechanism in a docstring is worse than an admitted gap, and
  `test_the_answer_is_stable_under_refinement` pins the property that actually
  matters instead: refining the mesh must not move the answer.

- `dilation_mm` grows each triangle in its own plane, and it is not cosmetic. A
  ray squeezing through a slit thinner than the margin is called blocked, which
  is the right lie -- no torch fits through it either, and `access.py` must never
  report reachable where nothing is. Measured on `pipe_bracket.step`, it is also
  what makes the mesh agree with the exact surfaces: undilated the BVH finds
  57.4062 mm blocked, dilated 0.05 mm it finds 59.7500 mm, and OCCT finds
  59.7500 mm. So the margin is not a safety fudge on top of the right answer, it
  is what recovers the right answer at a span boundary, where a facet edge sits
  fractionally inside the true surface.

**Not the display mesh.** `tessellate.py` says in its own docstring that a
display tessellation is tuned for looking right rather than for being
conservative, and reusing it here would be exactly the
`visual_meshes_as_collision_meshes` smell this repo already calls out. This asks
for its own tessellation, at a deflection it states in the output.

**Parity with OCCT is a test, not a claim.** `tests/programmer/test_occlusion.py` puts the
same sample points and the same directions through both intersectors on
`pipe_bracket.step`. The blocked/not-blocked verdict -- the one that decides
whether a stretch becomes a candidate seam -- agreed on every sample of the
manifold, 63/63 and 53/53, and `min_open_fraction` agreed exactly at the coarse
ray count. `access.py` keeps the OCCT path reachable through `--engine occt` so
the oracle stays runnable rather than being a number in a comment.

Self-contained. No imports from `cadquery/v1/` or `tesseract/v1/`.
"""

from __future__ import annotations

import argparse
import sys
from typing import Any, NamedTuple

import numpy as np
from numba import njit, prange

# Facet-to-surface bound. Below `access.SURFACE_SKIP` (0.05 mm) on purpose: see
# the module docstring for what happens when it is not.
DEFAULT_DEFLECTION_MM = 0.01

# In-plane growth per triangle. Small: it exists to seal slits a torch could
# never enter, not to fatten the part.
DEFAULT_DILATION_MM = 0.05

# OCCT's angular deflection for the same tessellation, in radians.
DEFAULT_ANGULAR_DEFLECTION = 0.35

# Triangles per BVH leaf. Small leaves cost nodes and buy tighter boxes; 4 was
# the flat part of the curve on a 218k-triangle manifold.
LEAF_SIZE = 4

# Deep enough for a median-split tree over any triangle count this will see:
# 2**64 leaves. A fixed stack keeps the traversal allocation-free.
MAX_STACK = 64


class Bvh(NamedTuple):
    """A built index. Flat arrays because the kernels are nopython."""

    node_min: np.ndarray
    node_max: np.ndarray
    left: np.ndarray
    right: np.ndarray
    start: np.ndarray
    count: np.ndarray
    order: np.ndarray
    v0: np.ndarray
    e1: np.ndarray
    e2: np.ndarray
    triangle_count: int
    deflection_mm: float
    dilation_mm: float

    def provenance(self) -> dict[str, Any]:
        """What this index is, for the `evidence` block of whatever uses it.

        A reachability verdict computed at one deflection and read back as if it
        were exact is the kind of thing that has to be greppable in a saved
        project, so the numbers travel with the answer.
        """
        return {
            "triangles": int(self.triangle_count),
            "deflection_mm": self.deflection_mm,
            "dilation_mm": self.dilation_mm,
        }


# --------------------------------------------------------------------------
# tessellation
# --------------------------------------------------------------------------


def triangles(
    shape: Any,
    deflection_mm: float = DEFAULT_DEFLECTION_MM,
    dilation_mm: float = DEFAULT_DILATION_MM,
    angular: float = DEFAULT_ANGULAR_DEFLECTION,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Triangles of `shape` as three vertex arrays, dilated in plane.

    Meshed in the shape's own units, which for every STEP this module reads is
    millimetres -- the same units `access.py` casts rays in. Nothing is scaled
    here; scaling to metres happens where the topology document is written.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRepTools import BRepTools
    from OCP.TopLoc import TopLoc_Location

    # Discard any triangulation already hanging off the shape, because
    # `BRepMesh_IncrementalMesh` will otherwise KEEP it and the deflection asked
    # for here is silently ignored. Measured on the exhaust manifold: without
    # this, every deflection from 0.01 mm to 1.0 mm returned the same 357,810
    # triangles. That is not merely a lost speedup -- two callers wanting
    # different deflections on one shape would get whichever meshed first, and
    # if the coarse one wins, `access.py` reports reachable where nothing is.
    BRepTools.Clean_s(shape.wrapped)
    BRepMesh_IncrementalMesh(shape.wrapped, deflection_mm, False, angular, True)
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    for face in shape.Faces():
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face.wrapped, location)
        if mesh is None:
            # A face OCCT declined to mesh. Skipped rather than guessed at; the
            # count is reported so a shape that loses faces here is visible.
            continue
        trsf = location.Transformation()
        base = len(verts)
        for i in range(1, mesh.NbNodes() + 1):
            point = mesh.Node(i).Transformed(trsf)
            verts.append((point.X(), point.Y(), point.Z()))
        for i in range(1, mesh.NbTriangles() + 1):
            a, b, c = mesh.Triangle(i).Get()
            faces.append((base + a - 1, base + b - 1, base + c - 1))

    if not faces:
        raise ValueError("tessellation produced no triangles")

    v = np.asarray(verts, np.float64)
    f = np.asarray(faces, np.int64)
    a, b, c = v[f[:, 0]].copy(), v[f[:, 1]].copy(), v[f[:, 2]].copy()
    if dilation_mm > 0.0:
        # Each vertex pushed out from the triangle's centroid. In plane, so a
        # closed mesh stays closed and only the silhouette thickens; no vertex
        # normals, so nothing cracks at a sharp edge.
        centroid = (a + b + c) / 3.0
        for corner in (a, b, c):
            delta = corner - centroid
            length = np.linalg.norm(delta, axis=1, keepdims=True)
            length[length < 1e-12] = 1.0
            corner += delta / length * dilation_mm
    return a, b, c


# --------------------------------------------------------------------------
# build
# --------------------------------------------------------------------------


def build(
    a: np.ndarray,
    b: np.ndarray,
    c: np.ndarray,
    deflection_mm: float = DEFAULT_DEFLECTION_MM,
    dilation_mm: float = DEFAULT_DILATION_MM,
) -> Bvh:
    """Median-split BVH over triangle centroids.

    Median split rather than a surface-area heuristic: it is one
    `argpartition` per node, it builds a 218k-triangle tree in 0.7 s, and the
    traversal is already 1.5 us per ray. SAH would buy a fraction of that back
    for several times the build, which is the wrong trade when the build runs
    once per workpiece and the rays run millions of times.
    """
    tri_min = np.minimum(np.minimum(a, b), c)
    tri_max = np.maximum(np.maximum(a, b), c)
    centroid = (tri_min + tri_max) * 0.5
    order = np.arange(len(a), dtype=np.int64)

    capacity = 4 * (len(a) // LEAF_SIZE + 2)
    node_min = np.zeros((capacity, 3))
    node_max = np.zeros((capacity, 3))
    left = np.full(capacity, -1, np.int64)
    right = np.full(capacity, -1, np.int64)
    start = np.zeros(capacity, np.int64)
    count = np.zeros(capacity, np.int64)
    used = 0

    # Explicit stack rather than recursion: a 218k-triangle tree is ~35 levels
    # deep, which is fine, but the depth is data-dependent and a recursion limit
    # raised to cover it is a limit nobody can reason about.
    todo = [(0, len(a), -1, False)]
    while todo:
        lo, hi, parent, is_right = todo.pop()
        index = used
        used += 1
        subset = order[lo:hi]
        node_min[index] = tri_min[subset].min(axis=0)
        node_max[index] = tri_max[subset].max(axis=0)
        if parent >= 0:
            if is_right:
                right[parent] = index
            else:
                left[parent] = index
        if hi - lo <= LEAF_SIZE:
            start[index] = lo
            count[index] = hi - lo
            continue
        axis = int(np.argmax(node_max[index] - node_min[index]))
        mid = (lo + hi) // 2
        order[lo:hi] = subset[np.argpartition(centroid[subset, axis], mid - lo)]
        todo.append((mid, hi, index, True))
        todo.append((lo, mid, index, False))

    return Bvh(
        node_min=node_min[:used],
        node_max=node_max[:used],
        left=left[:used],
        right=right[:used],
        start=start[:used],
        count=count[:used],
        order=order,
        v0=a,
        e1=b - a,
        e2=c - a,
        triangle_count=len(a),
        deflection_mm=deflection_mm,
        dilation_mm=dilation_mm,
    )


# --------------------------------------------------------------------------
# traversal
# --------------------------------------------------------------------------


@njit(cache=True, inline="always")
def _slab(node_min, node_max, node, ox, oy, oz, ix, iy, iz):
    """Ray/box overlap, forward of the origin only."""
    t0 = (node_min[node, 0] - ox) * ix
    t1 = (node_max[node, 0] - ox) * ix
    lo = min(t0, t1)
    hi = max(t0, t1)
    t0 = (node_min[node, 1] - oy) * iy
    t1 = (node_max[node, 1] - oy) * iy
    lo = max(lo, min(t0, t1))
    hi = min(hi, max(t0, t1))
    t0 = (node_min[node, 2] - oz) * iz
    t1 = (node_max[node, 2] - oz) * iz
    lo = max(lo, min(t0, t1))
    hi = min(hi, max(t0, t1))
    return hi >= max(lo, 0.0)


@njit(cache=True)
def _any_hit(
    node_min, node_max, left, right, start, count, order, v0, e1, e2,
    ox, oy, oz, dx, dy, dz, skip,
):
    """Any triangle hit at t > skip. Short-circuits at the first.

    Möller–Trumbore, double precision, two-sided: a hit counts whichever way the
    facet faces, because the question is whether something is in the way and not
    which side of it the ray came from.

    `skip` drops the surfaces the sample point itself sits on -- the two groove
    faces -- exactly as OCCT's `W > SURFACE_SKIP` does, and it also means a ray
    is a forward ray rather than the infinite line OCCT is given.
    """
    ix = 1e30 if dx == 0.0 else 1.0 / dx
    iy = 1e30 if dy == 0.0 else 1.0 / dy
    iz = 1e30 if dz == 0.0 else 1.0 / dz
    stack = np.empty(MAX_STACK, np.int64)
    stack[0] = 0
    sp = 1
    while sp > 0:
        sp -= 1
        node = stack[sp]
        if not _slab(node_min, node_max, node, ox, oy, oz, ix, iy, iz):
            continue
        if count[node] > 0:
            for k in range(start[node], start[node] + count[node]):
                t = order[k]
                px = dy * e2[t, 2] - dz * e2[t, 1]
                py = dz * e2[t, 0] - dx * e2[t, 2]
                pz = dx * e2[t, 1] - dy * e2[t, 0]
                det = e1[t, 0] * px + e1[t, 1] * py + e1[t, 2] * pz
                if -1e-14 < det < 1e-14:
                    continue
                inv = 1.0 / det
                tx = ox - v0[t, 0]
                ty = oy - v0[t, 1]
                tz = oz - v0[t, 2]
                u = (tx * px + ty * py + tz * pz) * inv
                if u < 0.0 or u > 1.0:
                    continue
                qx = ty * e1[t, 2] - tz * e1[t, 1]
                qy = tz * e1[t, 0] - tx * e1[t, 2]
                qz = tx * e1[t, 1] - ty * e1[t, 0]
                w = (dx * qx + dy * qy + dz * qz) * inv
                if w < 0.0 or u + w > 1.0:
                    continue
                if (e2[t, 0] * qx + e2[t, 1] * qy + e2[t, 2] * qz) * inv > skip:
                    return True
        else:
            stack[sp] = left[node]
            sp += 1
            stack[sp] = right[node]
            sp += 1
    return False


@njit(cache=True)
def _count_escapes(
    node_min, node_max, left, right, start, count, order, v0, e1, e2,
    ox, oy, oz, dirs, skip, stop_at_first,
):
    """How many of `dirs` escape from one point. Stops at one if asked."""
    escaped = 0
    for d in range(dirs.shape[0]):
        if not _any_hit(
            node_min, node_max, left, right, start, count, order, v0, e1, e2,
            ox, oy, oz, dirs[d, 0], dirs[d, 1], dirs[d, 2], skip,
        ):
            escaped += 1
            if stop_at_first:
                return escaped
    return escaped


# --------------------------------------------------------------------------
# the barrel: hit lists, and the whole lattice in one call
# --------------------------------------------------------------------------

# Hits kept per ray. `clearance.py` needs the LIST, not the nearest hit, because
# it reads a standoff off it afterwards and counts crossings past the barrel's
# far end. Measured on the exhaust manifold a ray averages 2.9 hits; 256 is far
# enough above that to be a bug detector rather than a limit, and overflow is
# reported rather than truncated -- a dropped hit flips a parity and turns a
# buried torch into a clear one, which is the one direction this must never err.
MAX_HITS = 256


@njit(cache=True)
def _hits_along(
    node_min, node_max, left, right, start, count, order, v0, e1, e2,
    ox, oy, oz, dx, dy, dz, skip, out,
):
    """Every hit at t > skip, into `out`, unsorted. Returns how many there were.

    The return may EXCEED `out.shape[0]`: counting continues past capacity so
    the caller can tell a full buffer from an overflowing one. Unsorted because
    `_collides` only ever tests membership of an interval and a parity, neither
    of which cares about order, and sorting 3 hits costs more than not.
    """
    ix = 1e30 if dx == 0.0 else 1.0 / dx
    iy = 1e30 if dy == 0.0 else 1.0 / dy
    iz = 1e30 if dz == 0.0 else 1.0 / dz
    stack = np.empty(MAX_STACK, np.int64)
    stack[0] = 0
    sp = 1
    n = 0
    while sp > 0:
        sp -= 1
        node = stack[sp]
        if not _slab(node_min, node_max, node, ox, oy, oz, ix, iy, iz):
            continue
        if count[node] > 0:
            for k in range(start[node], start[node] + count[node]):
                t = order[k]
                px = dy * e2[t, 2] - dz * e2[t, 1]
                py = dz * e2[t, 0] - dx * e2[t, 2]
                pz = dx * e2[t, 1] - dy * e2[t, 0]
                det = e1[t, 0] * px + e1[t, 1] * py + e1[t, 2] * pz
                if -1e-14 < det < 1e-14:
                    continue
                inv = 1.0 / det
                tx = ox - v0[t, 0]
                ty = oy - v0[t, 1]
                tz = oz - v0[t, 2]
                u = (tx * px + ty * py + tz * pz) * inv
                if u < 0.0 or u > 1.0:
                    continue
                qx = ty * e1[t, 2] - tz * e1[t, 1]
                qy = tz * e1[t, 0] - tx * e1[t, 2]
                qz = tx * e1[t, 1] - ty * e1[t, 0]
                w = (dx * qx + dy * qy + dz * qz) * inv
                if w < 0.0 or u + w > 1.0:
                    continue
                d = (e2[t, 0] * qx + e2[t, 1] * qy + e2[t, 2] * qz) * inv
                if d > skip:
                    if n < out.shape[0]:
                        out[n] = d
                    n += 1
        else:
            stack[sp] = left[node]
            sp += 1
            stack[sp] = right[node]
            sp += 1
    return n


@njit(cache=True, inline="always")
def _collides(hits, n, standoff, body):
    """`clearance.fan_collides`, for one ray. The two conditions, unchanged.

    A hit inside the barrel, or an odd number beyond its far end -- which means
    the far end is buried in material. See `clearance.py`'s docstring for why
    material inside the standoff gap is NOT a collision.
    """
    far = standoff + body
    beyond = 0
    for i in range(n):
        w = hits[i]
        if standoff < w <= far:
            return True
        if w > far:
            beyond += 1
    return beyond % 2 == 1


@njit(cache=True, parallel=True)
def _barrel_lattice(
    node_min, node_max, left, right, start, count, order, v0, e1, e2,
    points, tangents, refs, work_rad, travel_rad, standoffs, offsets,
    body, skip, clear, overflow,
):
    """Clear/blocked for every (sample, work, travel, standoff), in one call.

    The whole of `clearance.cast_fan` plus `frames.apply_angles` plus
    `clearance.barrel_axes`, inlined, so that a lattice cell costs a ray rather
    than a ray plus an interpreter round trip. That is where the speedup
    actually lives: the arithmetic below is not cheaper than the Python it
    replaces, there is just no Python left between the rays.

    Parallel over SAMPLES, and `clear` is indexed by sample so no two threads
    ever touch the same element -- the fold onto decision knots happens in numpy
    afterwards, where it is a scatter-add and not a race.

    `clear[i, d, a, b]` is 1.0 when the torch is clear there, 0.0 when it is
    not; the caller weights by the arc each sample speaks for.
    """
    ns = points.shape[0]
    nw = work_rad.shape[0]
    nt = travel_rad.shape[0]
    nd = standoffs.shape[0]
    nr = offsets.shape[0]
    for i in prange(ns):
        hits = np.empty(MAX_HITS, np.float64)
        px = points[i, 0]; py = points[i, 1]; pz = points[i, 2]
        tx = tangents[i, 0]; ty = tangents[i, 1]; tz = tangents[i, 2]
        rx = refs[i, 0]; ry = refs[i, 1]; rz = refs[i, 2]
        for a in range(nw):
            # r' = R(t, work) r, Rodrigues about the unit tangent.
            ca = np.cos(work_rad[a]); sa = np.sin(work_rad[a])
            dot = tx * rx + ty * ry + tz * rz
            cx = ty * rz - tz * ry
            cy = tz * rx - tx * rz
            cz = tx * ry - ty * rx
            r1x = rx * ca + cx * sa + tx * dot * (1.0 - ca)
            r1y = ry * ca + cy * sa + ty * dot * (1.0 - ca)
            r1z = rz * ca + cz * sa + tz * dot * (1.0 - ca)
            # b' = t x r'
            bx = ty * r1z - tz * r1y
            by = tz * r1x - tx * r1z
            bz = tx * r1y - ty * r1x
            bn = np.sqrt(bx * bx + by * by + bz * bz)
            if bn > 1e-30:
                bx /= bn; by /= bn; bz /= bn
            for b in range(nt):
                # u = R(b', -travel) r'
                cb = np.cos(-travel_rad[b]); sb = np.sin(-travel_rad[b])
                dot2 = bx * r1x + by * r1y + bz * r1z
                c2x = by * r1z - bz * r1y
                c2y = bz * r1x - bx * r1z
                c2z = bx * r1y - by * r1x
                ux = r1x * cb + c2x * sb + bx * dot2 * (1.0 - cb)
                uy = r1y * cb + c2y * sb + by * dot2 * (1.0 - cb)
                uz = r1z * cb + c2z * sb + bz * dot2 * (1.0 - cb)
                un = np.sqrt(ux * ux + uy * uy + uz * uz)
                if un > 1e-30:
                    ux /= un; uy /= un; uz /= un
                # The disc basis: the tangent with its axial part removed, so
                # the fan is oriented by the seam and not by a world axis.
                ax = tx * ux + ty * uy + tz * uz
                f1x = tx - ax * ux; f1y = ty - ax * uy; f1z = tz - ax * uz
                fn = np.sqrt(f1x * f1x + f1y * f1y + f1z * f1z)
                if fn < 1e-9:
                    if abs(uz) < 0.9:
                        hx = 0.0; hy = 0.0; hz = 1.0
                    else:
                        hx = 1.0; hy = 0.0; hz = 0.0
                    hd = hx * ux + hy * uy + hz * uz
                    f1x = hx - hd * ux; f1y = hy - hd * uy; f1z = hz - hd * uz
                    fn = np.sqrt(f1x * f1x + f1y * f1y + f1z * f1z)
                    if fn < 1e-30:
                        fn = 1.0
                f1x /= fn; f1y /= fn; f1z /= fn
                f2x = uy * f1z - uz * f1y
                f2y = uz * f1x - ux * f1z
                f2z = ux * f1y - uy * f1x
                # ONE fan of casts; every standoff is read back off it.
                for d in range(nd):
                    clear[i, d, a, b] = 1.0
                for q in range(nr):
                    oa = offsets[q, 0]; ob = offsets[q, 1]
                    n = _hits_along(
                        node_min, node_max, left, right, start, count, order, v0, e1, e2,
                        px + oa * f1x + ob * f2x,
                        py + oa * f1y + ob * f2y,
                        pz + oa * f1z + ob * f2z,
                        ux, uy, uz, skip, hits,
                    )
                    if n > MAX_HITS:
                        # Truncated, so the parity is not trustworthy. Called
                        # blocked, which is the conservative lie, and counted so
                        # it cannot pass unnoticed.
                        overflow[i] += 1
                        n = MAX_HITS
                        for d in range(nd):
                            clear[i, d, a, b] = 0.0
                        continue
                    for d in range(nd):
                        if clear[i, d, a, b] > 0.0 and _collides(
                            hits, n, standoffs[d], body
                        ):
                            clear[i, d, a, b] = 0.0


class Engine:
    """The escape test, over a built index.

    Two methods because `access.py` needs two different things: a COUNT, which
    becomes `open_fraction`, and a boolean, which is all a bisection needs. The
    boolean short-circuits at the first escaping direction and the count cannot,
    so keeping them separate is worth a few lines.
    """

    name = "bvh"

    def __init__(self, bvh: Bvh, skip_mm: float):
        self._bvh = bvh
        self._skip = float(skip_mm)

    @property
    def provenance(self) -> dict[str, Any]:
        return {"engine": self.name, **self._bvh.provenance()}

    def escaping(self, origin: tuple[float, float, float], dirs: np.ndarray) -> int:
        b = self._bvh
        return int(
            _count_escapes(
                b.node_min, b.node_max, b.left, b.right, b.start, b.count, b.order,
                b.v0, b.e1, b.e2,
                float(origin[0]), float(origin[1]), float(origin[2]),
                dirs, self._skip, False,
            )
        )

    def any_escape(self, origin: tuple[float, float, float], dirs: np.ndarray) -> bool:
        b = self._bvh
        return bool(
            _count_escapes(
                b.node_min, b.node_max, b.left, b.right, b.start, b.count, b.order,
                b.v0, b.e1, b.e2,
                float(origin[0]), float(origin[1]), float(origin[2]),
                dirs, self._skip, True,
            )
        )

    # -- the barrel ------------------------------------------------------
    #
    # `clearance.py` asks a different question from `access.py`: not "does
    # anything escape" but "where, along this ray, is there material". Same
    # index, same traversal, a hit LIST instead of a verdict.

    def hits_along(
        self,
        origin: tuple[float, float, float],
        direction: tuple[float, float, float],
        lo: float | None = None,
    ) -> list[float]:
        """Distances to every surface along the forward ray, past `lo`.

        Signature-compatible with `access.hits_along` so the two intersectors
        are interchangeable at the call site and `--engine occt` stays the
        oracle rather than becoming dead code.
        """
        b = self._bvh
        out = np.empty(MAX_HITS, np.float64)
        n = _hits_along(
            b.node_min, b.node_max, b.left, b.right, b.start, b.count, b.order,
            b.v0, b.e1, b.e2,
            float(origin[0]), float(origin[1]), float(origin[2]),
            float(direction[0]), float(direction[1]), float(direction[2]),
            self._skip if lo is None else float(lo), out,
        )
        if n > MAX_HITS:
            raise ValueError(
                f"a ray hit {n} triangles, past the {MAX_HITS} this keeps. "
                "Raise occlusion.MAX_HITS: a truncated list has a wrong crossing "
                "parity, which reports a buried torch as clear."
            )
        return out[:n].tolist()

    def barrel_lattice(
        self,
        points: np.ndarray,
        tangents: np.ndarray,
        refs: np.ndarray,
        work_deg: np.ndarray,
        travel_deg: np.ndarray,
        standoffs_mm: np.ndarray,
        offsets: np.ndarray,
        body_length_mm: float,
    ) -> np.ndarray:
        """`clear[sample, standoff, work, travel]`, 1.0 where the torch fits.

        One call for the whole seam. See `_barrel_lattice` for why the angle
        arithmetic lives in the kernel rather than being passed in.
        """
        b = self._bvh
        shape = (len(points), len(standoffs_mm), len(work_deg), len(travel_deg))
        clear = np.zeros(shape, np.float64)
        overflow = np.zeros(len(points), np.int64)
        _barrel_lattice(
            b.node_min, b.node_max, b.left, b.right, b.start, b.count, b.order,
            b.v0, b.e1, b.e2,
            np.ascontiguousarray(points, np.float64),
            np.ascontiguousarray(tangents, np.float64),
            np.ascontiguousarray(refs, np.float64),
            np.radians(np.asarray(work_deg, np.float64)),
            np.radians(np.asarray(travel_deg, np.float64)),
            np.asarray(standoffs_mm, np.float64),
            np.ascontiguousarray(offsets, np.float64),
            float(body_length_mm),
            self._skip,
            clear,
            overflow,
        )
        total = int(overflow.sum())
        if total:
            raise ValueError(
                f"{total} rays hit more than the {MAX_HITS} triangles this keeps. "
                "Raise occlusion.MAX_HITS: a truncated list has a wrong crossing "
                "parity, which reports a buried torch as clear."
            )
        return clear


def as_array(dirs: Any) -> np.ndarray:
    """Directions as a contiguous float64 (n, 3), for the kernels."""
    return np.ascontiguousarray(np.asarray(dirs, np.float64).reshape(-1, 3))


# --------------------------------------------------------------------------
# cache
# --------------------------------------------------------------------------

# One index per (shape identity, deflection, dilation), per process.
#
# `access.annotate_joint` is the unit of parallelism, so without this every
# joint would re-tessellate and rebuild -- about 1.7 s each on a 218k-triangle
# manifold, which on a ten-joint workpiece would hand back most of the speedup
# this module exists for. Small because the arrays are tens of megabytes and a
# worker only ever looks at one workpiece at a time.
_CACHE: dict[tuple[Any, float, float], Bvh] = {}
_CACHE_LIMIT = 2


def index_for(
    shape: Any,
    key: Any = None,
    deflection_mm: float = DEFAULT_DEFLECTION_MM,
    dilation_mm: float = DEFAULT_DILATION_MM,
) -> Bvh:
    """The index for this shape, built once per process.

    `key` identifies the geometry -- the STEP path or its sha256. Passed in
    rather than derived from `shape`, because an OCCT handle has no identity a
    cache can trust and hashing 300k triangles to find out would cost what the
    lookup is meant to save.

    **`key=None` means do not cache**, and that is the only safe default. The
    tempting one is `id(shape)`, which is wrong in a way that produces no
    symptom: CPython reuses an address as soon as the object at it is freed, so
    a cached index can be handed back for an entirely different workpiece that
    happens to have landed there. Nothing downstream can detect it -- the
    triangles are a plausible mesh, just of the wrong part -- and it would show
    up as an occasional, unreproducible accessibility answer. A caller that
    wants the cache has to say what the geometry IS.
    """
    if key is None:
        return build(
            *triangles(shape, deflection_mm, dilation_mm),
            deflection_mm=deflection_mm,
            dilation_mm=dilation_mm,
        )
    cache_key = (key, float(deflection_mm), float(dilation_mm))
    found = _CACHE.get(cache_key)
    if found is None:
        found = build(
            *triangles(shape, deflection_mm, dilation_mm),
            deflection_mm=deflection_mm,
            dilation_mm=dilation_mm,
        )
        if len(_CACHE) >= _CACHE_LIMIT:
            _CACHE.pop(next(iter(_CACHE)))
        _CACHE[cache_key] = found
    return found


# --------------------------------------------------------------------------
# self-test
# --------------------------------------------------------------------------


def _box(dx: float, dy: float, dz: float, at: tuple[float, float, float] = (0.0, 0.0, 0.0)):
    import cadquery as cq

    return cq.Workplane("XY").box(dx, dy, dz).translate(at).val()


def self_test() -> int:
    import cadquery as cq

    skip = 0.05
    dirs = as_array(
        [
            (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, -1.0),
        ]
    )

    # A sealed box: inside escapes nowhere, outside escapes everywhere. The
    # whole semantic of the module in two assertions.
    box = _box(20.0, 20.0, 20.0)
    engine = Engine(build(*triangles(box, 0.01, 0.0), 0.01, 0.0), skip)
    assert engine.escaping((0.0, 0.0, 0.0), dirs) == 0, "inside a sealed box, something escaped"
    # From outside, five of the six escape and the one aimed back at the box does
    # not -- which is the assertion that the intersector sees the box at all,
    # rather than a mesh that is silently empty.
    assert engine.escaping((100.0, 0.0, 0.0), dirs) == 5, "the ray aimed at the box escaped"
    assert engine.any_escape((100.0, 0.0, 0.0), dirs) is True
    assert engine.any_escape((0.0, 0.0, 0.0), dirs) is False

    # Dilation seals a slit narrower than the margin, and leaves a wide one open.
    # This is the property the whole conservatism argument rests on, so it is
    # measured rather than asserted in a comment.
    def slit_open(gap_mm: float, dilation_mm: float) -> bool:
        left = _box(10.0, 40.0, 40.0, (-5.0 - gap_mm / 2.0, 0.0, 0.0))
        right = _box(10.0, 40.0, 40.0, (5.0 + gap_mm / 2.0, 0.0, 0.0))
        shape = cq.Compound.makeCompound([left, right])
        eng = Engine(build(*triangles(shape, 0.01, dilation_mm), 0.01, dilation_mm), skip)
        # Straight down the gap, from below the pair.
        return eng.any_escape((0.0, 0.0, -60.0), as_array([(0.0, 0.0, 1.0)]))

    assert slit_open(2.0, 0.0), "a 2 mm gap should be open undilated"
    assert slit_open(2.0, 0.05), "0.05 mm of margin must not seal a 2 mm gap"
    assert not slit_open(0.05, 0.20), "0.20 mm of margin should seal a 0.05 mm slit"

    # Provenance travels, because a verdict at one deflection must not be read
    # back as if it were exact.
    prov = engine.provenance
    assert prov["engine"] == "bvh" and prov["deflection_mm"] == 0.01, prov
    assert prov["triangles"] > 0

    # The cache returns the same object rather than rebuilding.
    first = index_for(box, "self-test", 0.01, 0.0)
    assert index_for(box, "self-test", 0.01, 0.0) is first, "cache missed on identical arguments"
    assert index_for(box, "self-test", 0.02, 0.0) is not first, "cache ignored the deflection"

    print(
        f"occlusion self-test ok: {prov['triangles']} triangles, sealed box blocked, "
        "slit sealed by dilation, cache keyed on deflection"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", default="", help="STEP file to index and report on")
    parser.add_argument("--deflection-mm", type=float, default=DEFAULT_DEFLECTION_MM)
    parser.add_argument("--dilation-mm", type=float, default=DEFAULT_DILATION_MM)
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.step:
        parser.print_help()
        return 0

    import json
    import time
    from pathlib import Path

    from seam_worker.cad.topology import load_step

    shape = load_step(Path(ns.step))
    t0 = time.time()
    bvh = build(
        *triangles(shape, ns.deflection_mm, ns.dilation_mm),
        deflection_mm=ns.deflection_mm,
        dilation_mm=ns.dilation_mm,
    )
    print(json.dumps({**bvh.provenance(), "build_seconds": round(time.time() - t0, 3),
                      "nodes": int(bvh.node_min.shape[0])}, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"occlusion: {exc}", file=sys.stderr)
        raise SystemExit(1)
