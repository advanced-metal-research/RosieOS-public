#!/usr/bin/env python
"""Stage 2: weld joint detection from topology.

Finds where parts meet, by face adjacency across distinct solids. Not shape
recognition -- nothing here looks at whether an outline is a circle or a
dogbone. Two faces on different solids with antiparallel outward normals, no
separation, and overlapping extents are a joint, whatever shape they happen
to be.

Operates on the `amr-weld-planner-v1.topology.v1` document only. No OCCT, no
STEP, no geometry kernel -- which makes it fast, pure, and testable without a
CAD environment. The cost is that partial overlap (where neither face contains
the other) needs a 2D boolean and is reported as unsupported rather than
guessed at. See `_overlap_relation`.

Three things this produces that the prior CAD path could not:

- the **weld line**: the boundary of the contact region, as exact curves
  carried over from topology, not resampled;
- the **groove faces**: the two faces actually exposed AT that line, which are
  NOT the contact faces -- see `groove_faces_for_edge`;
- the **dihedral**, from those groove faces, which is what makes work angle
  derivable instead of hardcoded.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

from seam_worker import paths
from seam_worker.seam.continuity import annotate_curve

SCHEMA = "amr-weld-planner-v1.joints.v1"

# Faces are coincident below this separation. STEP from a CAD assembly mates
# exactly, so this is a floating-point tolerance, not a modelling one.
COINCIDENT_GAP_M = 1e-7

# Above COINCIDENT_GAP_M but below this, report as a near miss: probably a
# joint with a root gap, and the operator should see it rather than hunt for
# it. WEBUI.md section 9.
NEAR_MISS_GAP_M = 0.003

# Outward normals must oppose. cos of the angle between them.
ANTIPARALLEL_DOT = -0.999

# Below this the contact is a sliver, not a joint.
MIN_CONTACT_AREA_M2 = 1e-6

# A branch solid thinner than this fraction of its own footprint, measured
# along the contact normal, is lying flat -> lap. Thicker is standing on the
# face -> tee.
LAP_THICKNESS_RATIO = 0.25

# Dihedral is emitted as a constant if it varies less than this along the
# weld line, and as a piecewise_linear profile otherwise.
DIHEDRAL_CONSTANT_TOL_DEG = 0.5

DEFAULT_RAIL_OFFSET_M = 0.020

# How far an approximate rail may be wrong, as the work-angle error it induces.
#
# An angle rather than a length because that is what the rail is FOR: it is the
# work-angle reference, so a deviation e at offset R matters as atan(e/R) and a
# bare millimetre budget would mean different things at different offsets. At
# 0.1 degrees this is well inside `DIHEDRAL_CONSTANT_TOL_DEG` (0.5) and the
# shipped work-angle band, so a rail at tolerance cannot be what decides a weld.
RAIL_TOLERANCE_DEG = 0.1

# Refinement rounds allowed while chasing that tolerance, and the loop stops the
# moment it is inside -- so simple geometry never pays for this at all, and the
# exact planar and cylindrical paths never enter it.
#
# Seven because that is what the measurement said. On the full outer ellipse of a
# 30x20 tube at a 20 mm offset, uniform refinement roughly halves the error per
# round:
#
#     round   poles   deviation      work-angle error
#         3      28    0.32283 mm         0.925 deg
#         4      52    0.17356 mm         0.497 deg
#         5     100    0.07627 mm         0.219 deg
#         6     196    0.03573 mm         0.102 deg
#         7     388    0.01730 mm         0.050 deg   <- inside tolerance
#
# 388 poles for a 158 mm rail is heavy, and it is the price of refining every
# span equally. Splitting only the spans where the bisector actually turns would
# reach the same tolerance far cheaper; that is a worthwhile optimisation and not
# a correctness fix, so it is recorded here rather than half-done.
MAX_RAIL_REFINEMENTS = 7


# --------------------------------------------------------------------------
# small vector helpers (no numpy: keeps this importable anywhere)
# --------------------------------------------------------------------------


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _sub(a, b):
    return [x - y for x, y in zip(a, b)]


def _add(a, b):
    return [x + y for x, y in zip(a, b)]


def _scale(a, k):
    return [x * k for x in a]


def _norm(a):
    n = math.sqrt(_dot(a, a))
    return [x / n for x in a] if n > 1e-12 else [0.0, 0.0, 0.0]


def _bbox_overlap(a_min, a_max, b_min, b_max, tol=1e-9):
    return not (
        any(lo > hi + tol for lo, hi in zip(a_min, b_max))
        or any(lo > hi + tol for lo, hi in zip(b_min, a_max))
    )


def _bbox_contains(outer_min, outer_max, inner_min, inner_max, tol=1e-9):
    return all(o <= i + tol for o, i in zip(outer_min, inner_min)) and all(
        i <= o + tol for i, o in zip(inner_max, outer_max)
    )


# --------------------------------------------------------------------------
# pair analysis
# --------------------------------------------------------------------------


def _plane_separation(face_a: dict, face_b: dict) -> float:
    """Distance between two parallel planes, along a's normal."""
    delta = _sub(face_a["plane"]["origin_xyz_m"], face_b["plane"]["origin_xyz_m"])
    return abs(_dot(delta, face_a["outward_normal_xyz"]))


def _overlap_relation(face_a: dict, face_b: dict) -> str:
    """Which face contains which, by extent.

    Returns "a_contains_b", "b_contains_a", "partial", or "disjoint".

    Extent containment is exact for the convex outlines in the fixtures and an
    approximation in general -- a non-convex outline could bound another face's
    extents without containing its area. `partial` is reported rather than
    guessed at: resolving it needs a 2D boolean in the shared plane, which
    means OCCT, which this module deliberately does not depend on.
    """
    amin, amax = face_a["bbox_min_xyz_m"], face_a["bbox_max_xyz_m"]
    bmin, bmax = face_b["bbox_min_xyz_m"], face_b["bbox_max_xyz_m"]
    if not _bbox_overlap(amin, amax, bmin, bmax):
        return "disjoint"
    if _bbox_contains(amin, amax, bmin, bmax):
        return "a_contains_b"
    if _bbox_contains(bmin, bmax, amin, amax):
        return "b_contains_a"
    return "partial"


def analyze_pair(face_a: dict, face_b: dict) -> dict[str, Any]:
    """Classify one candidate face pair.

    Always returns a verdict with a reason. "Not a joint" is useless feedback
    for manual authoring; "normals are 47 deg apart" tells the operator whether
    the model is wrong or the threshold is. WEBUI.md section 9.
    """
    out: dict[str, Any] = {"face_a": face_a["id"], "face_b": face_b["id"]}

    if face_a["solid_id"] == face_b["solid_id"]:
        return {**out, "verdict": "rejected", "reason": "same solid"}
    if face_a["surface_type"] != "plane" or face_b["surface_type"] != "plane":
        # `curved` marks this for the solid-pair test in `detect_joints`: an
        # extent prefilter cannot tell a real curved contact from two bounding
        # boxes that merely overlap, so whether this is worth reporting depends
        # on something only that caller knows -- did anything get welded between
        # these two solids at all.
        return {
            **out,
            "verdict": "unsupported",
            "curved": True,
            "reason": f"non-planar contact ({face_a['surface_type']}/{face_b['surface_type']})",
        }

    dot = _dot(face_a["outward_normal_xyz"], face_b["outward_normal_xyz"])
    out["normal_dot"] = round(dot, 9)
    if dot > ANTIPARALLEL_DOT:
        angle = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        return {
            **out,
            "verdict": "rejected",
            "reason": f"normals {angle:.1f} deg apart, not antiparallel",
        }

    gap = _plane_separation(face_a, face_b)
    out["gap_m"] = round(gap, 12)
    if gap > NEAR_MISS_GAP_M:
        return {**out, "verdict": "rejected", "reason": f"planes {gap * 1000:.3f} mm apart"}

    relation = _overlap_relation(face_a, face_b)
    out["overlap_relation"] = relation
    if relation == "disjoint":
        return {**out, "verdict": "rejected", "reason": "extents do not overlap"}
    if relation == "partial":
        return {
            **out,
            "verdict": "unsupported",
            "reason": "partial overlap needs a 2D boolean in the shared plane",
        }

    contact_area = min(face_a["area_m2"], face_b["area_m2"])
    out["contact_area_m2"] = contact_area
    if contact_area < MIN_CONTACT_AREA_M2:
        return {**out, "verdict": "rejected", "reason": f"contact area {contact_area:.2e} m2 is a sliver"}

    # The contained face is the branch; the containing face is the base. The
    # weld line runs around the branch, because that is where the free
    # boundary is.
    if relation == "a_contains_b":
        out["base"], out["branch"] = face_a["id"], face_b["id"]
    else:
        out["base"], out["branch"] = face_b["id"], face_a["id"]

    out["verdict"] = "contact" if gap <= COINCIDENT_GAP_M else "near_miss"
    out["reason"] = "coincident" if gap <= COINCIDENT_GAP_M else f"root gap {gap * 1000:.3f} mm"
    return out


# --------------------------------------------------------------------------
# joint construction
# --------------------------------------------------------------------------


def classify_joint_type(topology: dict, branch_face: dict) -> tuple[str, dict[str, Any]]:
    """lap or tee, from how thick the branch solid is along the contact normal.

    A plate lying flat on a face is thin along that normal -> lap. A plate
    standing on its edge is thick along it -> tee. This is a heuristic on the
    solid's extents, so it carries its numbers as evidence.
    """
    solid = next(s for s in topology["solids"] if s["id"] == branch_face["solid_id"])
    extents = _sub(solid["bbox_max_xyz_m"], solid["bbox_min_xyz_m"])
    normal = branch_face["outward_normal_xyz"]
    along = sum(abs(n) * e for n, e in zip(normal, extents))
    across = max(e for i, e in enumerate(extents) if abs(normal[i]) < 0.5) if any(
        abs(n) < 0.5 for n in normal
    ) else max(extents)
    ratio = along / across if across > 1e-12 else float("inf")
    evidence = {
        "branch_extent_along_normal_m": round(along, 9),
        "branch_extent_across_m": round(across, 9),
        "thickness_ratio": round(ratio, 6),
    }
    return ("lap" if ratio < LAP_THICKNESS_RATIO else "tee"), evidence


def groove_faces_for_edge(
    topology_faces: dict, edge: dict, branch_face_id: str, base_face_id: str
) -> dict[str, Any] | None:
    """The two faces EXPOSED at a weld-line edge.

    This is the distinction that is easy to get wrong. On a plate lapped onto a
    block, the *contact* pair is the plate's bottom face against the block's top
    face -- horizontal on horizontal, which classifies the joint. The *groove*
    faces at the weld line are the plate's SIDE face and the block's TOP face --
    vertical meeting horizontal, which is what sets the torch angle.

    The contact pair classifies. The groove faces orient. They are not the same
    faces, and the bisector rail comes from the groove.
    """
    others = [fid for fid in edge["face_ids"] if fid != branch_face_id]
    if not others:
        return None
    side_face = topology_faces[others[0]]
    base_face = topology_faces[base_face_id]
    n_a = side_face["outward_normal_xyz"]
    n_b = base_face["outward_normal_xyz"]
    bisector = _norm(_add(n_a, n_b))
    if not any(abs(c) > 1e-9 for c in bisector):
        return None  # opposed normals: no wedge, so no groove
    dihedral = math.degrees(math.acos(max(-1.0, min(1.0, _dot(n_a, n_b)))))
    return {
        "face_a": {
            "solid_id": side_face["solid_id"],
            "face_id": side_face["id"].split(":", 1)[1],
            "surface_type": side_face["surface_type"],
            "outward_normal_xyz": n_a,
        },
        "face_b": {
            "solid_id": base_face["solid_id"],
            "face_id": base_face["id"].split(":", 1)[1],
            "surface_type": base_face["surface_type"],
            "outward_normal_xyz": n_b,
        },
        "_bisector": bisector,
        "_dihedral_deg": dihedral,
    }


CHAIN_TOL_M = 1e-9


def _reverse_segment(seg: dict[str, Any]) -> dict[str, Any]:
    """Reverse a B-spline segment's traversal direction.

    Poles and weights reverse; the knot vector reflects about its own span,
    `new[i] = (first + last) - old[-1-i]`, which preserves clamping and
    multiplicities. The analytic annotation has to follow or it contradicts the
    spline.
    """
    out = dict(seg)
    out["control_points_xyz_m"] = list(reversed(seg["control_points_xyz_m"]))
    if "weights" in seg:
        out["weights"] = list(reversed(seg["weights"]))
    knots = seg["knots_u"]
    span = knots[0] + knots[-1]
    out["knots_u"] = [round(span - k, 12) for k in reversed(knots)]

    analytic = seg.get("analytic")
    if analytic and analytic["kind"] == "line":
        out["analytic"] = {
            **analytic,
            "start_xyz_m": analytic["end_xyz_m"],
            "end_xyz_m": analytic["start_xyz_m"],
        }
    elif analytic and analytic["kind"] in ("arc", "circle"):
        out["analytic"] = {
            **analytic,
            "start_angle_rad": round(
                analytic["start_angle_rad"] + analytic["sweep_angle_rad"], 12
            ),
            "sweep_angle_rad": -analytic["sweep_angle_rad"],
        }
    return out


def chain_loops(edges: list[dict]) -> list[tuple[list[tuple[dict, bool]], bool]]:
    """Order edges into every connected wire they form, reversing where needed.

    A face's edges arrive in exploration order, which is neither traversal
    order nor consistently oriented. The schema defines a curve as segments
    joined end to end, so emitting them as they come produces a bag of edges
    that merely happens to have the right total length -- on the dog_box
    dogbone every consecutive gap was 35.36 mm while the count and the
    821.239 mm total were both correct.

    **A face's boundary is not one wire.** A face with a hole has an outer loop
    and one inner loop per hole, and they are disjoint: a pipe standing on a
    plate touches it through an annulus, whose outer circle is the fillet weld
    and whose inner circle is the bore. Chaining ACROSS them would invent
    geometry, so each is returned separately and the caller decides what each
    one is. Returned in the order the edges hand them over; `_boundary_kinds`
    says which is which.

    Each loop is (ordered pairs of (edge, reversed), closed).
    """
    remaining = {e["id"]: e for e in edges}
    loops: list[tuple[list[tuple[dict, bool]], bool]] = []

    while remaining:
        first = next(iter(remaining.values()))
        del remaining[first["id"]]
        chain: list[tuple[dict, bool]] = [(first, False)]
        tail = first["end_xyz_m"]

        while remaining:
            for candidate in remaining.values():
                start, end = candidate["start_xyz_m"], candidate["end_xyz_m"]
                if math.dist(tail, start) <= CHAIN_TOL_M:
                    chain.append((candidate, False))
                    tail = end
                    break
                if math.dist(tail, end) <= CHAIN_TOL_M:
                    chain.append((candidate, True))
                    tail = start
                    break
            else:
                break  # this loop is complete; anything left starts another
            del remaining[chain[-1][0]["id"]]

        head = chain[0][0]["start_xyz_m"]
        loops.append((chain, math.dist(tail, head) <= CHAIN_TOL_M))
    return loops


def chain_edges(edges: list[dict]) -> tuple[list[tuple[dict, bool]], bool] | None:
    """The single wire these edges form, or None if they form several.

    Kept because "the weld line is one wire" is the assumption `continuity.py`,
    `span.py` and `seams.py` are written against, and their docstrings name this
    function for it. Detection itself uses `chain_loops` and handles each wire.
    """
    if not edges:
        return None
    loops = chain_loops(edges)
    return loops[0] if len(loops) == 1 else None


def _loop_bbox(chain: list[tuple[dict, bool]]) -> tuple[list[float], list[float]]:
    """A box the loop is certainly inside.

    From control points rather than sampled points: a B-spline lies within the
    convex hull of its poles, so this contains the curve exactly, with no
    sampling density to get wrong. It is conservative -- a rational arc's poles
    stand outside the arc -- and conservative is the safe direction here, since
    the box is only ever used to ask which loop encloses which.
    """
    points = [
        point for edge, _ in chain for point in edge["bspline"]["control_points_xyz_m"]
    ]
    return (
        [min(p[i] for p in points) for i in range(3)],
        [max(p[i] for p in points) for i in range(3)],
    )


def _boundary_kinds(loops: list[tuple[list[tuple[dict, bool]], bool]]) -> list[str]:
    """`single`, or `outer`/`inner` per loop: which of these wires is a hole.

    Containment of one loop's box in another's, which for the shapes this
    actually arises on -- a bore inside a pipe wall, bolt holes inside a plate
    footprint -- is unambiguous. Two contact patches side by side on one face
    contain neither the other and are both reported `outer`, which is right:
    they are two separate boundaries, not a hole.

    **This label decides nothing.** Every loop becomes a weld joint whatever it
    is called, and whether a bore is weldable is `access.py`'s answer, not a
    name's. The label exists so the operator reading a list of five joints per
    pipe can tell which one is the bore.
    """
    if len(loops) == 1:
        return ["single"]
    boxes = [_loop_bbox(chain) for chain, _ in loops]
    kinds = []
    for i, (low, high) in enumerate(boxes):
        enclosed = any(
            _bbox_contains(other_low, other_high, low, high)
            for j, (other_low, other_high) in enumerate(boxes)
            if j != i
        )
        kinds.append("inner" if enclosed else "outer")
    return kinds


def _curve_from_edges(chain: list[tuple[dict, bool]], closed: bool) -> dict[str, Any]:
    """Weld line as a curve, carrying the exact B-splines from topology.

    Nothing is resampled. A 270 degree corner relief stays a degree-2 rational
    spline, not a polyline.
    """
    segments = []
    total = 0.0
    for edge, flipped in chain:
        bs = dict(edge["bspline"])
        seg: dict[str, Any] = {
            "id": edge["id"].split(":", 1)[1],
            "degree": bs["degree"],
            "control_points_xyz_m": bs["control_points_xyz_m"],
            "knots_u": bs["knots_u"],
            "length_m": edge["length_m"],
            "source_edge_id": edge["id"],
        }
        if "weights" in bs:
            seg["weights"] = bs["weights"]
        if "analytic" in edge:
            seg["analytic"] = edge["analytic"]
        if flipped:
            seg = _reverse_segment(seg)
        # Which way the wire walked this edge. The rail's sampled groove normals
        # are stored in the EDGE's direction, so a reversed segment has to read
        # them backwards; stripped again before the segment is emitted.
        seg["_flipped"] = flipped
        segments.append(seg)
        total += edge["length_m"]
    return {"segments": segments, "closed": closed, "length_m": round(total, 12)}


def _dihedral_function(grooves: list[dict], lengths: list[float]) -> dict[str, Any]:
    """Constant if it does not vary, piecewise_linear over normalized arc length
    otherwise.

    On the dog_box top plate this comes out constant at 90 degrees even though
    four of the eight groove faces are cylinders: the relief axes are vertical,
    so their normals stay horizontal and stay 90 degrees from the block's top
    face. Constant dihedral with a rotating bisector -- which is exactly the
    case that makes the rail interesting.
    """
    values = [g["_dihedral_deg"] for g in grooves]
    if max(values) - min(values) <= DIHEDRAL_CONSTANT_TOL_DEG:
        return {"kind": "constant", "value": round(sum(values) / len(values), 6)}
    total = sum(lengths)
    knots, run = [], 0.0
    for value, length in zip(values, lengths):
        knots.append([round(run / total, 9), round(value, 6)])
        run += length
    knots.append([1.0, round(values[-1], 6)])
    return {"kind": "piecewise_linear", "knots": knots}


def _rail_segment_planar(seg: dict, bisector: list[float], offset_m: float) -> dict[str, Any]:
    """Rigid translation. Correct only when the groove normal is constant along
    the segment, i.e. when the groove face is planar."""
    shift = _scale(bisector, offset_m)
    moved = dict(seg)
    moved["control_points_xyz_m"] = [
        [round(c + d, 12) for c, d in zip(p, shift)] for p in seg["control_points_xyz_m"]
    ]
    return moved


def _sampled_normals(face: dict, edge_id: str) -> dict[str, Any] | None:
    """The groove face's normals along one edge, as `topology.py` recorded them."""
    for use in face.get("edge_uses") or []:
        if use["edge_id"] == edge_id and "face_normals" in use:
            return use["face_normals"]
    return None


def _bisector_at(sampled: dict, n_base: list[float], t: float, flipped: bool) -> list[float] | None:
    """The unit bisector at fraction `t` along a SEGMENT.

    Linear interpolation between the recorded normals, renormalized. The samples
    are in the EDGE's parameter direction, so a segment the wire walked backwards
    reads them backwards -- getting that wrong mirrors the rail about the middle
    of the segment, which looks plausible and is wrong everywhere but the centre.
    """
    fractions, normals = sampled["t"], sampled["normals_xyz"]
    if flipped:
        t = 1.0 - t
    t = min(max(t, 0.0), 1.0)
    upper = 1
    while upper < len(fractions) - 1 and fractions[upper] < t:
        upper += 1
    lo, hi = fractions[upper - 1], fractions[upper]
    w = 0.0 if hi <= lo else (t - lo) / (hi - lo)
    n_a = _norm(_add(_scale(normals[upper - 1], 1.0 - w), _scale(normals[upper], w)))
    if not any(abs(c) > 1e-9 for c in n_a):
        return None
    bisector = _norm(_add(n_a, n_base))
    return bisector if any(abs(c) > 1e-9 for c in bisector) else None


def _greville(degree: int, knots: list[float], index: int) -> float:
    """The parameter a control point is 'at': the mean of the knots spanning it.

    The right abscissa for moving one pole, because it is where that pole has
    most influence. Offsetting every pole by the bisector at its own Greville
    abscissa is the standard approximate offset of a B-spline, and it degenerates
    to the exact rigid translation when the bisector is constant -- so the planar
    case would come out identical if it were routed through here.
    """
    window = knots[index + 1 : index + degree + 1]
    return sum(window) / len(window) if window else knots[index]


def _insert_knot(seg: dict[str, Any], u: float) -> dict[str, Any]:
    """Boehm knot insertion. The curve is unchanged; it just has more poles.

    Which is the whole point: pole offsetting is only as good as how little the
    bisector turns within one pole's support, so refining the SAME curve until
    that is small is how the approximation is made to converge. Done in
    homogeneous coordinates so a rational segment -- every arc and ellipse in
    here is one -- stays exact rather than being subtly reweighted.
    """
    degree = int(seg["degree"])
    knots = list(seg["knots_u"])
    points = [list(p) for p in seg["control_points_xyz_m"]]
    weights = list(seg.get("weights") or [1.0] * len(points))
    hom = [[c * w for c in p] + [w] for p, w in zip(points, weights)]

    k = max(i for i in range(len(knots) - 1) if knots[i] <= u < knots[i + 1])
    out = hom[: k - degree + 1]
    for i in range(k - degree + 1, k + 1):
        span = knots[i + degree] - knots[i]
        a = 0.0 if span <= 0.0 else (u - knots[i]) / span
        out.append([(1.0 - a) * hom[i - 1][j] + a * hom[i][j] for j in range(4)])
    out += hom[k:]

    refined = dict(seg)
    refined["knots_u"] = knots[: k + 1] + [u] + knots[k + 1 :]
    refined["control_points_xyz_m"] = [[c / h[3] for c in h[:3]] for h in out]
    refined["weights"] = [h[3] for h in out]
    return refined


def _refine_once(seg: dict[str, Any]) -> dict[str, Any]:
    """Split every non-empty knot span at its midpoint."""
    degree = int(seg["degree"])
    knots = seg["knots_u"]
    interior = sorted(
        {
            0.5 * (knots[i] + knots[i + 1])
            for i in range(degree, len(knots) - degree - 1)
            if knots[i + 1] - knots[i] > 1e-12
        }
    )
    out = seg
    for u in interior:
        out = _insert_knot(out, u)
    return out


def _rail_segment_general(
    seg: dict, groove: dict, face: dict, offset_m: float
) -> dict[str, Any] | None:
    """Approximate offset for any groove face, with its deviation measured.

    The exact paths handle a plane (constant normal) and a cylinder whose axis is
    parallel to the base normal (closed-form concentric arc). Everything else --
    a cone, a torus, a sphere, a B-spline patch, the `extrusion` surface OCCT
    reports for an elliptical tube -- has no exact offset expressible as a
    B-spline of the same degree: the offset of an ellipse is not an ellipse. The
    joint used to be discarded whole rather than approximated, which cost the
    weld line as well as the rail.

    So: each pole moves by the bisector at its own Greville abscissa, the result
    is CHECKED rather than trusted -- sampled against the true offset at the
    places `topology.py` recorded normals -- and the segment is refined by knot
    insertion until the measured error is inside `RAIL_TOLERANCE_DEG`. The
    achieved error is reported as `max_deviation_mm` either way. A rail whose
    error is stated is usable; one whose error is unknown is what this module
    refuses to emit.

    Refinement is not optional. Offsetting the poles of a full ellipse in one
    span measures 5.97 mm out on a 20 mm offset -- a 17 degree work-angle error,
    useless. The same curve refined to tolerance lands where it should.
    """
    edge_id = seg.get("source_edge_id")
    sampled = _sampled_normals(face, edge_id) if edge_id else None
    if sampled is None:
        return None
    n_base = groove["face_b"]["outward_normal_xyz"]
    flipped = bool(seg.get("_flipped"))
    # The tolerance is an ANGLE converted to a length at this offset, not a bare
    # length: what the rail feeds is the work angle, so an error of e at radius R
    # matters as atan(e/R). The same argument as D16 -- measure in the unit the
    # quantity is actually in.
    tolerance_m = offset_m * math.tan(math.radians(RAIL_TOLERANCE_DEG))

    from seam_worker.seam.curve import Segment

    def attempt(candidate: dict[str, Any]) -> tuple[dict[str, Any], float] | None:
        degree, knots = int(candidate["degree"]), list(candidate["knots_u"])
        u0, u1 = knots[degree], knots[len(knots) - degree - 1]
        if u1 - u0 <= 0.0:
            return None
        moved_poles = []
        for i, pole in enumerate(candidate["control_points_xyz_m"]):
            u = _greville(degree, knots, i)
            bisector = _bisector_at(sampled, n_base, (u - u0) / (u1 - u0), flipped)
            if bisector is None:
                return None
            moved_poles.append(
                [round(c + d, 12) for c, d in zip(pole, _scale(bisector, offset_m))]
            )
        out = dict(candidate)
        out["control_points_xyz_m"] = moved_poles
        try:
            weld_curve, rail_curve = Segment(candidate), Segment(out)
        except Exception:  # noqa: BLE001 - an unevaluable rail is a refused rail.
            return None
        worst = 0.0
        for t in sampled["t"]:
            bisector = _bisector_at(sampled, n_base, t, flipped)
            if bisector is None:
                return None
            u = u0 + (u1 - u0) * (1.0 - t if flipped else t)
            want = _add(list(weld_curve.point(u)), _scale(bisector, offset_m))
            worst = max(worst, math.dist(want, list(rail_curve.point(u))))
        return out, worst

    working = dict(seg)
    best: tuple[dict[str, Any], float] | None = None
    for _ in range(MAX_RAIL_REFINEMENTS + 1):
        found = attempt(working)
        if found is None:
            break
        best = found
        if found[1] <= tolerance_m:
            break
        working = _refine_once(working)

    if best is None:
        return None
    moved, worst = best
    # Carried on the SEGMENT only as far as `_bisector_rail`, which lifts it onto
    # the rail and strips it: segments are the shared curve definition, and a
    # weld line must not grow fields that only mean something for a rail.
    moved["_deviation_m"] = worst
    moved["_within_tolerance"] = bool(worst <= tolerance_m)
    return moved


def _cylinder_normal_sign(face: dict) -> float:
    """+1 if the face's outward normal points AWAY from the cylinder axis
    (convex boss), -1 if it points toward it (concave relief).

    Compared at `normal_sample_xyz_m`, the point the normal was actually
    evaluated at. Two wrong ways to do this, both tried:

    - Against the radial direction at an arbitrary point on the face. The
      stored normal is sampled at the uv midpoint, and on a 270 degree patch
      the radial direction at an endpoint is 135 degrees away from it, so the
      sign comes out backwards at the ends.
    - Against the radial direction at the face centroid. A full 360 degree
      cylindrical face has its centroid exactly ON the axis, so there is no
      radial direction at all and the two full cylinders in dog_box.step drop
      out of detection entirely.

    Comparing normal and position at the same point has neither problem.
    """
    cylinder = face["cylinder"]
    axis = _norm(cylinder["axis_dir_xyz"])
    rel = _sub(face["normal_sample_xyz_m"], cylinder["axis_origin_xyz_m"])
    radial = _sub(rel, _scale(axis, _dot(rel, axis)))
    if math.sqrt(_dot(radial, radial)) < 1e-12:
        return 0.0
    return 1.0 if _dot(face["outward_normal_xyz"], _norm(radial)) >= 0.0 else -1.0


def _rail_segment_cylindrical(
    seg: dict, groove: dict, face: dict, offset_m: float
) -> dict[str, Any] | None:
    """Exact offset for an arc whose groove face is a cylinder.

    A cylinder's outward normal ROTATES along the arc, so translating the
    segment by one frozen bisector is wrong -- badly. On the dog_box corner
    reliefs (270 deg, R=25 mm) a frozen bisector puts the rail 26.1 mm off at
    the arc endpoints, on a 20 mm offset: further wrong than it is long.

    When the cylinder axis is parallel to the base face normal, the bisector
    decomposes into CONSTANT radial and axial components, because the radial
    normal stays perpendicular to the base normal all the way round. The offset
    curve is then another exact arc: concentric, radius R + s*offset*c_rad,
    shifted offset*c_ax along the axis.

    A rational B-spline circle scales exactly when its control points are
    scaled radially about the axis -- same weights, same knots -- so this is
    exact, not a fit.

    Returns None when the axis is not parallel to the base normal, in which
    case the decomposition is not constant and no exact arc exists. Better to
    emit nothing than to emit a plausible wrong curve.
    """
    cylinder = face["cylinder"]
    axis = _norm(cylinder["axis_dir_xyz"])
    origin = cylinder["axis_origin_xyz_m"]
    radius = cylinder["radius_m"]
    n_base = groove["face_b"]["outward_normal_xyz"]

    if abs(_dot(axis, n_base)) < 0.999:
        return None
    if radius <= 1e-12:
        return None

    sign = _cylinder_normal_sign(face)
    if sign == 0.0:
        return None

    # With the axis parallel to the base normal, the outward radial direction
    # u(P) is perpendicular to n_base everywhere, so |sign*u + n_base| = sqrt(2)
    # at every point and the bisector splits evenly between radial and axial:
    #     bisector(P) = (sign*u(P) + n_base) / sqrt(2)
    # The radial part is constant in MAGNITUDE while rotating in direction,
    # which is exactly why the offset stays an exact concentric arc.
    root_half = 1.0 / math.sqrt(2.0)
    bis_radial = sign * root_half
    bis_axial = _dot(n_base, axis) * root_half

    new_radius = radius + offset_m * bis_radial
    if new_radius <= 1e-9:
        return None
    k = new_radius / radius
    axial_shift = _scale(axis, offset_m * bis_axial)

    poles = []
    for p in seg["control_points_xyz_m"]:
        rel = _sub(p, origin)
        axial = _dot(rel, axis)
        radial = _sub(rel, _scale(axis, axial))
        moved = _add(_add(origin, _scale(axis, axial)), _scale(radial, k))
        poles.append([round(c + d, 12) for c, d in zip(moved, axial_shift)])

    out = dict(seg)
    out["control_points_xyz_m"] = poles
    out["length_m"] = round(seg["length_m"] * k, 12)
    return out


def _bisector_rail(
    topology_faces: dict, weld_line: dict, grooves: list[dict], offset_m: float
) -> dict[str, Any] | None:
    """Rail curve: the weld line offset along the groove bisector.

    Per segment, because the bisector genuinely flips at a corner. Segment
    counts match the weld line, so `correspondence: normalized_arc_length`
    resolves per segment -- see SEAM-MODEL.md section 5.

    Planar groove faces get a rigid translation; cylindrical ones get an exact
    concentric offset. Anything else returns None rather than a wrong curve.
    """
    segments = []
    total = 0.0
    # None until some segment had to approximate; the rail then reports the worst
    # of them. `all_within` starts true and only a real segment can falsify it.
    worst_deviation: float | None = None
    all_within = True
    for seg, groove in zip(weld_line["segments"], grooves):
        face_key = f"{groove['face_a']['solid_id']}:{groove['face_a']['face_id']}"
        face = topology_faces.get(face_key)
        surface = face["surface_type"] if face else "unknown"

        if surface == "plane":
            moved = _rail_segment_planar(seg, groove["_bisector"], offset_m)
        elif surface == "cylinder":
            moved = _rail_segment_cylindrical(seg, groove, face, offset_m)
        else:
            moved = None
        # The exact paths first, and the approximate one only where they cannot
        # answer -- including a cylinder whose axis is not parallel to the base
        # normal, where the closed form does not exist. An exact rail is never
        # traded for an approximate one.
        if moved is None and face is not None:
            moved = _rail_segment_general(seg, groove, face, offset_m)

        if moved is None:
            return None
        moved["id"] = f"rail_{seg['id']}"
        moved.pop("analytic", None)
        moved.pop("source_edge_id", None)
        moved.pop("_flipped", None)
        # Lifted off the segment and onto the rail: the worst segment sets the
        # rail's deviation, and a rail is only within tolerance if every segment
        # is. An exact segment contributes nothing, which is why the fields are
        # absent entirely on a rail that never approximated anything.
        deviation = moved.pop("_deviation_m", None)
        if deviation is not None:
            worst_deviation = max(worst_deviation or 0.0, deviation)
            all_within = all_within and bool(moved.pop("_within_tolerance", False))
        moved.pop("_within_tolerance", None)
        segments.append(moved)
        total += moved["length_m"]

    approximation: dict[str, Any] = {}
    if worst_deviation is not None:
        approximation = {
            "approximate": True,
            "max_deviation_mm": round(worst_deviation * 1000.0, 6),
            "within_tolerance": all_within,
        }

    return {
        **approximation,
        "offset_m": offset_m,
        # The rail is deliberately NOT continuous. At a corner the groove
        # normal flips, so the bisector jumps and the rail jumps with it -- by
        # exactly offset * |b1 - b2|, which is 20 mm at a 90 degree flip on a
        # 20 mm offset. That is why correspondence is per-segment
        # (SEAM-MODEL.md section 5). `closed` is therefore reported from
        # measured coincidence, never inherited from the weld line.
        "curve": {
            "segments": segments,
            "closed": math.dist(
                segments[0]["control_points_xyz_m"][0],
                segments[-1]["control_points_xyz_m"][-1],
            )
            <= CHAIN_TOL_M,
            "length_m": round(total, 12),
        },
        "continuous": all(
            math.dist(
                seg["control_points_xyz_m"][-1],
                segments[(i + 1) % len(segments)]["control_points_xyz_m"][0],
            )
            <= CHAIN_TOL_M
            for i, seg in enumerate(segments)
        ),
    }


def build_joints(
    topology: dict, pair: dict, first_index: int, rail_offset_m: float
) -> tuple[list[dict[str, Any]], list[str]]:
    """Every weld joint one contact pair yields, and why any loop was dropped.

    **One joint per boundary loop, not one per pair.** A pipe standing on a
    plate contacts it through an annulus: the outer circle is the fillet weld
    and the inner circle is the bore. Both are real boundaries of the contact
    region, so both become joints and `access.py` decides whether a torch can
    reach them -- a bore wide enough to weld inside is a weld, and a bore that
    is not comes back fully blocked and produces no candidate seam. Deciding
    here that a hole cannot be welded would be this module guessing, and it
    would be wrong on exactly the parts where it matters.

    The reasons are returned rather than swallowed. This function has four ways
    to fail per loop and they used to arrive at the caller as one hardcoded
    sentence about groove faces, which is how a chaining failure on an exhaust
    manifold spent its time being investigated as a groove-face bug.
    """
    faces = {f["id"]: f for f in topology["faces"]}
    edges = {e["id"]: e for e in topology["edges"]}
    branch = faces[pair["branch"]]

    # The weld line is the branch face's own boundary: with the branch strictly
    # inside the base, that boundary IS the free edge of the contact region.
    # Seam edges are parametrization artifacts and are never real boundaries.
    line_edges = [
        edges[u["edge_id"]] for u in branch["edge_uses"] if not u["seam"]
    ]
    seen, unique = set(), []
    for edge in line_edges:
        if edge["id"] not in seen:
            seen.add(edge["id"])
            unique.append(edge)
    if not unique:
        return [], ["the branch face has no non-seam edges to take a weld line from"]

    # Order into wires BEFORE anything downstream: groove faces and rail
    # segments are indexed positionally against the weld line, so they must be
    # built in traversal order or they describe the wrong segments.
    loops = chain_loops(unique)
    kinds = _boundary_kinds(loops)

    joints: list[dict[str, Any]] = []
    problems: list[str] = []
    for (chain, closed), kind in zip(loops, kinds):
        joint, reason = _joint_from_loop(
            topology, faces, pair, chain, closed, first_index + len(joints), rail_offset_m, kind
        )
        if joint is None:
            problems.append(f"{kind} boundary ({len(chain)} edge(s)): {reason}")
        else:
            joints.append(joint)
    return joints, problems


def _joint_from_loop(
    topology: dict,
    faces: dict,
    pair: dict,
    chain: list[tuple[dict, bool]],
    closed: bool,
    index: int,
    rail_offset_m: float,
    boundary: str,
) -> tuple[dict[str, Any] | None, str | None]:
    """One weld joint from one ordered boundary loop, or the reason there isn't."""
    branch = faces[pair["branch"]]
    base = faces[pair["base"]]

    grooves = []
    for edge, _flipped in chain:
        groove = groove_faces_for_edge(faces, edge, branch["id"], base["id"])
        if groove is None:
            return None, f"no groove faces exposed at edge {edge['id']}"
        grooves.append(groove)

    weld_line = _curve_from_edges(chain, closed)
    # Chaining matches endpoints, so the curve does not jump; it says nothing
    # about whether the curve TURNS. This writes `continuity_to_next` onto each
    # segment and keeps the measurements as evidence. See continuity.py for why
    # only C0 and G1 are ever emitted.
    junctions = annotate_curve(weld_line)
    for i, groove in enumerate(grooves):
        groove["segment_index"] = i

    joint_type, type_evidence = classify_joint_type(topology, branch)
    lengths = [edge["length_m"] for edge, _ in chain]
    rail = _bisector_rail(faces, weld_line, grooves, rail_offset_m)
    # Internal bookkeeping for the rail only; the emitted curve is the contract's
    # and the schema allows it no extra properties.
    for segment in weld_line["segments"]:
        segment.pop("_flipped", None)
    if rail is None:
        return None, "no bisector rail could be offset from the groove faces"

    # An inner boundary is a hole in the contact region -- most often a bore --
    # and it says so in the label. Nothing is filtered on this; it is so that a
    # list of two joints per pipe is readable rather than a puzzle.
    suffix = "" if boundary == "single" else f", {boundary} boundary"
    return {
        "id": f"weld_joint_{index:04d}",
        "label": f"{base['solid_id']} / {branch['solid_id']} {joint_type}{suffix}",
        "type": joint_type,
        "weld_symbol": "fillet",
        "members": [
            {"solid_id": base["solid_id"], "face_ids": [base["id"].split(":", 1)[1]], "role": "base"},
            {
                "solid_id": branch["solid_id"],
                "face_ids": [branch["id"].split(":", 1)[1]],
                "role": "branch",
            },
        ],
        "contact": {
            "kind": "coincident" if pair["verdict"] == "contact" else "gap",
            "gap_mm": {"kind": "constant", "value": round(pair["gap_m"] * 1000.0, 9)},
            "overlap_area_mm2": round(pair["contact_area_m2"] * 1e6, 6),
            "max_face_deviation_mm": round(pair["gap_m"] * 1000.0, 9),
        },
        "weld_line": weld_line,
        "groove_faces": [
            {
                "segment_index": g["segment_index"],
                "face_a": g["face_a"],
                "face_b": g["face_b"],
            }
            for g in grooves
        ],
        "dihedral_angle_deg": _dihedral_function(grooves, lengths),
        "bisector_rail": rail,
        "accessible_sides": [[round(c, 9) for c in grooves[0]["_bisector"]]],
        "confidence": 1.0 if pair["verdict"] == "contact" else 0.6,
        "evidence": {
            "method": "coincident_face_pair",
            "shared_boundary_length_mm": round(weld_line["length_m"] * 1000.0, 6),
            "normal_dot": pair["normal_dot"],
            "overlap_relation": pair["overlap_relation"],
            # Which boundary of the contact region this is. `single` means the
            # face had no holes; `inner` means this joint IS a hole's edge, so a
            # planner reading it knows to expect access.py to have opinions.
            "contact_boundary": boundary,
            "junctions": junctions,
            **type_evidence,
        },
    }, None


# --------------------------------------------------------------------------
# top level
# --------------------------------------------------------------------------


def candidate_pairs(topology: dict) -> list[tuple[dict, dict]]:
    """Cross-solid face pairs whose extents overlap. EVERY surface type.

    An extent prefilter before the O(n^2) loop. 42 faces here is 861 pairs and
    the filter is irrelevant; a 10,000-face weldment is 50 million and it is
    not. A BVH belongs here once real assemblies arrive -- the point for now is
    that the prefilter is a distinct, replaceable step.

    **Curved faces are candidates too, and that is the whole change here.** This
    used to keep only `surface_type == "plane"`, which meant a curved contact
    never reached `analyze_pair` -- and `analyze_pair`'s own "non-planar
    contact" branch, the one that exists to report exactly this, was unreachable
    from its only caller. A tube through a hole produced no joint, no near miss
    and no note: the tool said the part had nothing to weld there, silently,
    which is the failure R-UI-1 is written against.

    It still cannot AUTHOR those welds. What it can do is say so. Letting the
    pair through costs one rejected verdict and turns an invisible gap into a
    named one -- `bracket_a`'s O60 tube through a O60 hole in a 10 mm plate is
    now reported as `cylinder/cylinder`, rather than vanishing.
    """
    considered = list(topology["faces"])
    pairs = []
    for i, a in enumerate(considered):
        for b in considered[i + 1 :]:
            if a["solid_id"] == b["solid_id"]:
                continue
            if not _bbox_overlap(
                a["bbox_min_xyz_m"],
                a["bbox_max_xyz_m"],
                b["bbox_min_xyz_m"],
                b["bbox_max_xyz_m"],
                tol=NEAR_MISS_GAP_M,
            ):
                continue
            pairs.append((a, b))
    return pairs


def detect_joints(topology: dict, rail_offset_m: float = DEFAULT_RAIL_OFFSET_M) -> dict[str, Any]:
    faces = {f["id"]: f for f in topology["faces"]}
    verdicts = [analyze_pair(a, b) for a, b in candidate_pairs(topology)]

    joints, near_misses, unsupported = [], [], []
    curved: list[dict[str, Any]] = []
    for verdict in verdicts:
        if verdict["verdict"] == "contact":
            built, problems = build_joints(topology, verdict, len(joints) + 1, rail_offset_m)
            joints.extend(built)
            # Reported per loop, and only for the loops that failed: a pipe whose
            # outer circle became a weld and whose bore did not is a partial
            # success, and saying nothing about the bore would hide the half that
            # needs looking at.
            for problem in problems:
                unsupported.append({**verdict, "reason": problem})
        elif verdict["verdict"] == "near_miss":
            near_misses.append(verdict)
        elif verdict["verdict"] == "unsupported":
            # Held back rather than reported outright -- see below.
            (curved if verdict.get("curved") else unsupported).append(verdict)

    # A curved contact is worth reporting only when NOTHING was welded between
    # those two solids.
    #
    # The prefilter is an extent test, so a non-planar pair reaching here means
    # the bounding boxes overlap, not that the faces touch. Reporting all of
    # them would trade a silent gap for a noisy false one: on `pipe_bracket`
    # four cylinder/plane pairs qualify whose centroids are 104 mm apart, and on
    # `hitch` eight of them describe the very post-to-plate contact that DID
    # produce two circular joints -- claiming that could not be analysed would
    # be worse than saying nothing, because it is untrue.
    #
    # What survives is the case that matters: two solids in contact with no weld
    # between them at all. On `bracket_a` that is the O60 tube through the O60
    # hole, whose cylinders are coincident -- centroid gap 0.0 mm -- and which
    # produced nothing whatever before this.
    welded = {frozenset(m["solid_id"] for m in joint["members"]) for joint in joints}
    for verdict in curved:
        pair = frozenset(
            {faces[verdict["face_a"]]["solid_id"], faces[verdict["face_b"]]["solid_id"]}
        )
        if pair not in welded:
            unsupported.append(verdict)

    return {
        "schema": SCHEMA,
        "source": topology.get("source", {}),
        "summary": {
            "candidate_pairs": len(verdicts),
            "joint_count": len(joints),
            "near_miss_count": len(near_misses),
            "unsupported_count": len(unsupported),
            "faces_considered": len(faces),
        },
        "weld_joints": joints,
        # Surfaced deliberately: a pair 0.3 mm apart is probably a joint with a
        # root gap, and the operator should see it rather than hunt for it.
        "near_misses": near_misses,
        "unsupported": unsupported,
    }


def repo_root() -> Path:
    """The repository this module sits inside. See `paths.py`."""
    return paths.REPO_ROOT


def self_test() -> int:
    from seam_worker.cad.topology import extract_topology

    step = repo_root() / "cadquery" / "v1" / "step" / "dog_box.step"
    if not step.exists():
        print(f"detect self-test: missing fixture {step}", file=sys.stderr)
        return 1
    result = detect_joints(extract_topology(step))
    s = result["summary"]
    assert s["joint_count"] == 5, s
    print(
        f"detect self-test ok: {s['joint_count']} joints from {s['candidate_pairs']} "
        f"candidate pairs ({s['near_miss_count']} near misses, "
        f"{s['unsupported_count']} unsupported)"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", default="", help="STEP file (runs topology first)")
    parser.add_argument("--topology", default="", help="topology JSON instead of a STEP")
    parser.add_argument("--out", default="")
    parser.add_argument("--rail-offset-m", type=float, default=DEFAULT_RAIL_OFFSET_M)
    parser.add_argument("--summary", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    ns = parser.parse_args(argv)

    if ns.self_test:
        return self_test()
    if ns.topology:
        topology = json.loads(Path(ns.topology).read_text(encoding="utf-8"))
    elif ns.step:
        from seam_worker.cad.topology import extract_topology

        topology = extract_topology(Path(ns.step))
    else:
        raise ValueError("one of --step, --topology or --self-test is required")

    result = detect_joints(topology, ns.rail_offset_m)
    if ns.out:
        out = Path(ns.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
        print(f"wrote joints: {ns.out}")
    if ns.summary or not ns.out:
        print(json.dumps(result["summary"] if ns.summary else result, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"detect: {exc}", file=sys.stderr)
        raise SystemExit(1)
