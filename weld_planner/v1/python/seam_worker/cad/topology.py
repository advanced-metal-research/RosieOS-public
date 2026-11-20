#!/usr/bin/env python
"""Stage 1: STEP -> B-rep topology with surface geometry and per-solid face ownership.

Emits `amr-weld-planner-v1.topology.v1`, the input to joint detection. What
distinguishes it from the prior CAD path in this repo:

- **Surfaces are read.** `cadquery/v1/python/extract_weld_edges.py` touches OCP
  in exactly one place (`BRepAdaptor_Curve`, as a point evaluator) and never
  imports `BRepAdaptor_Surface`. Plane normals and cylinder axes are never
  fetched, which is the structural reason joint detection cannot be built on it.
  Dihedral angles need surface geometry, so this reads it.
- **Curves stay analytic.** Edges are converted to exact rational B-splines via
  `GeomConvert`, not sampled into polylines. A circle comes out as a degree-2
  rational spline with weights cos(60 deg), exact to machine precision, rather
  than as N points. See GOAL.md R4.
- **No parameter-space sampling.** The prior path sampled uniformly in curve
  parameter (`param = first + (last - first) * t`), which is not arc length.
  Nothing here samples at all.

Self-contained. No imports from `cadquery/v1/` or `tesseract/v1/`.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

from seam_worker import paths

SCHEMA = "amr-weld-planner-v1.topology.v1"

# STEP files in this repo are millimetres; the weld program contract is metres.
DEFAULT_UNIT_SCALE_TO_M = 0.001

# Quantum for geometric content hashes, in source units. Coarse enough to be
# stable across OCCT versions and rebuilds, fine enough not to collide.
GEOM_KEY_QUANTUM = 1e-6


class TopologyError(ValueError):
    """A B-rep this stage cannot read as geometry.

    Raised rather than returned so it cannot be mistaken for a topology with
    fewer entities in it, and raised rather than left to OCCT so the failure is
    a message instead of a signal -- see `bspline_of_edge`.
    """


# --------------------------------------------------------------------------
# geometry helpers
# --------------------------------------------------------------------------


def _norm(v: tuple[float, float, float]) -> tuple[float, float, float]:
    n = math.sqrt(sum(c * c for c in v))
    if n < 1e-12:
        return (0.0, 0.0, 0.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def _cross(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _round_vec(v: Any, scale: float, places: int = 9) -> list[float]:
    return [round(float(c) * scale, places) for c in v]


def geom_key(*parts: Any) -> str:
    """Stable content hash. Independent of traversal order and rebuild order.

    Public because `tessellate.py` uses it to map OCCT shapes back to topology
    identifiers by content rather than by replicating the ordering logic --
    which would be a second place for the two modules to drift apart.
    """
    blob = "|".join(
        f"{round(p / GEOM_KEY_QUANTUM):d}" if isinstance(p, float) else str(p) for p in parts
    )
    return hashlib.sha1(blob.encode("utf-8")).hexdigest()[:12]


def sorted_solids(shape: Any) -> list[Any]:
    """Solids in deterministic order: volume descending, then centroid.

    OCCT traversal order is not guaranteed stable across versions, so
    identifiers are assigned from sorted geometry. Shared with
    `tessellate.py` so both modules assign the same `solid_N` to the same
    solid -- otherwise face ids in the display mesh would silently point at
    different geometry than the topology.
    """
    return sorted(
        shape.Solids(),
        key=lambda s: (-round(s.Volume(), 6), tuple(round(c, 6) for c in s.Center().toTuple())),
    )


def face_geom_key(face: Any) -> str:
    return geom_key(
        face.geomType(), float(face.Area()), *(float(c) for c in face.Center().toTuple())
    )


def edge_geom_key(solid_id: str, edge: Any) -> str:
    return geom_key(
        "E", solid_id, float(edge.Length()), *(float(c) for c in edge.Center().toTuple())
    )


# --------------------------------------------------------------------------
# curves
# --------------------------------------------------------------------------


def expand_knots(bspline: Any) -> list[float]:
    """OCCT stores (knot, multiplicity) pairs; the schema wants a flat vector.

    The result satisfies len(knots) == n_poles + degree + 1 for a clamped
    curve, which `tests/programmer/test_topology.py` asserts on every edge.
    """
    flat: list[float] = []
    for i in range(1, bspline.NbKnots() + 1):
        flat.extend([float(bspline.Knot(i))] * int(bspline.Multiplicity(i)))
    return flat


def _is_degenerate_edge(wrapped_edge: Any) -> bool:
    """True for an edge that collapses to a point and carries no 3D curve.

    A cone's apex and a sphere's poles are these. OCCT marks them, and the mark
    is the only cheap way to know before asking for a curve that is not there.
    """
    from OCP.BRep import BRep_Tool
    from OCP.TopoDS import TopoDS

    return bool(BRep_Tool.Degenerated_s(TopoDS.Edge(wrapped_edge)))


def bspline_of_edge(edge: Any, scale: float) -> dict[str, Any]:
    """Exact rational B-spline for an edge, trimmed to the edge's parameter range.

    `BRep_Tool.Curve_s` hands back the *untrimmed* underlying curve — infinite
    for a line — so it must be wrapped in a Geom_TrimmedCurve before conversion
    or the poles describe the wrong extent.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.Geom import Geom_TrimmedCurve
    from OCP.GeomConvert import GeomConvert

    wrapped = edge.wrapped
    adaptor = BRepAdaptor_Curve(wrapped)
    u0, u1 = float(adaptor.FirstParameter()), float(adaptor.LastParameter())
    curve = BRep_Tool.Curve_s(wrapped, 0.0, 0.0)
    # A null handle here is a SEGFAULT one line later, not an exception: OCCT
    # dereferences it inside Geom_TrimmedCurve. Callers filter degenerate edges
    # out before reaching this, and this is the guard for every path that does
    # not -- a raised error loses one edge, a segfault loses the whole worker
    # and poisons the process pool behind it.
    if curve is None:
        raise TopologyError(
            f"edge has no 3D curve (degenerate?): {_round_vec(edge.Center().toTuple(), scale)}"
        )
    bs = GeomConvert.CurveToBSplineCurve_s(Geom_TrimmedCurve(curve, u0, u1))

    poles = [_round_vec(bs.Pole(i).Coord(), scale) for i in range(1, bs.NbPoles() + 1)]
    out: dict[str, Any] = {
        "degree": int(bs.Degree()),
        "control_points_xyz_m": poles,
        "knots_u": [round(k, 12) for k in expand_knots(bs)],
    }
    if bs.IsRational():
        out["weights"] = [round(float(bs.Weight(i)), 12) for i in range(1, bs.NbPoles() + 1)]
    return out


def _curve_endpoints(edge: Any) -> tuple[Any, Any]:
    """Endpoints in the underlying curve's parameter direction.

    Not `edge.startPoint()`. A TopAbs_REVERSED edge traverses its curve
    backwards, so cadquery reports the curve's last point as the start. On
    dog_box.step that is 52 of 62 line edges. Since the B-spline poles come
    from the curve, endpoints must too, or the record contradicts itself.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Curve

    adaptor = BRepAdaptor_Curve(edge.wrapped)
    return (
        adaptor.Value(adaptor.FirstParameter()).Coord(),
        adaptor.Value(adaptor.LastParameter()).Coord(),
    )


def _face_edge_uses(face: Any) -> list[tuple[Any, str, bool]]:
    """(edge, orientation, is_seam) for every edge USE on the face.

    A seam edge on a periodic surface is used twice by the same face, once
    forward and once reversed. That is correct 2-manifold topology, not an
    open shell -- but it means "distinct faces per edge" is the wrong
    invariant, and that seams must be skipped when tracing real boundaries.
    """
    from OCP.BRep import BRep_Tool
    from OCP.TopAbs import TopAbs_EDGE, TopAbs_Orientation
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopoDS import TopoDS

    uses: list[tuple[Any, str, bool]] = []
    explorer = TopExp_Explorer(face.wrapped, TopAbs_EDGE)
    while explorer.More():
        wrapped = TopoDS.Edge(explorer.Current())
        reversed_ = wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED
        uses.append(
            (
                wrapped,
                "reversed" if reversed_ else "forward",
                bool(BRep_Tool.IsClosed_s(wrapped, face.wrapped)),
            )
        )
        explorer.Next()
    return uses


def _as_cq_edge(wrapped: Any) -> Any:
    import cadquery as cq

    return cq.Edge(wrapped)


def analytic_of_edge(edge: Any, scale: float) -> dict[str, Any] | None:
    """Readability annotation. The B-spline stays the source of truth."""
    from OCP.BRepAdaptor import BRepAdaptor_Curve

    kind = edge.geomType()
    adaptor = BRepAdaptor_Curve(edge.wrapped)
    if kind == "LINE":
        p0 = adaptor.Value(adaptor.FirstParameter())
        p1 = adaptor.Value(adaptor.LastParameter())
        return {
            "kind": "line",
            "start_xyz_m": _round_vec(p0.Coord(), scale),
            "end_xyz_m": _round_vec(p1.Coord(), scale),
        }
    if kind == "CIRCLE":
        circ = adaptor.Circle()
        span = float(adaptor.LastParameter()) - float(adaptor.FirstParameter())
        return {
            "kind": "circle" if abs(span - 2.0 * math.pi) < 1e-9 else "arc",
            "center_xyz_m": _round_vec(circ.Location().Coord(), scale),
            "radius_m": round(float(circ.Radius()) * scale, 12),
            "axis_xyz": list(_norm(tuple(circ.Axis().Direction().Coord()))),
            "start_angle_rad": round(float(adaptor.FirstParameter()), 12),
            "sweep_angle_rad": round(span, 12),
        }
    return None


# --------------------------------------------------------------------------
# surfaces
# --------------------------------------------------------------------------


def surface_of_face(face: Any, scale: float) -> dict[str, Any]:
    """Surface type and its defining parameters.

    This is the data the prior extractor never fetched, and the data joint
    detection is built on: coincidence needs plane origins and normals,
    dihedral needs normals at the weld line.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Surface

    adaptor = BRepAdaptor_Surface(face.wrapped)
    kind = face.geomType()
    out: dict[str, Any] = {
        "surface_type": kind.lower(),
        "uv_bounds": [
            round(float(adaptor.FirstUParameter()), 9),
            round(float(adaptor.LastUParameter()), 9),
            round(float(adaptor.FirstVParameter()), 9),
            round(float(adaptor.LastVParameter()), 9),
        ],
    }
    if kind == "PLANE":
        plane = adaptor.Plane()
        out["plane"] = {
            "origin_xyz_m": _round_vec(plane.Location().Coord(), scale),
            "normal_xyz": list(_norm(tuple(plane.Axis().Direction().Coord()))),
        }
    elif kind == "CYLINDER":
        cyl = adaptor.Cylinder()
        out["cylinder"] = {
            "axis_origin_xyz_m": _round_vec(cyl.Location().Coord(), scale),
            "axis_dir_xyz": list(_norm(tuple(cyl.Axis().Direction().Coord()))),
            "radius_m": round(float(cyl.Radius()) * scale, 12),
        }
    elif kind == "CONE":
        cone = adaptor.Cone()
        out["cone"] = {
            "apex_xyz_m": _round_vec(cone.Apex().Coord(), scale),
            "axis_dir_xyz": list(_norm(tuple(cone.Axis().Direction().Coord()))),
            "half_angle_rad": round(float(cone.SemiAngle()), 12),
            "ref_radius_m": round(float(cone.RefRadius()) * scale, 12),
        }
    elif kind == "SPHERE":
        sph = adaptor.Sphere()
        out["sphere"] = {
            "center_xyz_m": _round_vec(sph.Location().Coord(), scale),
            "radius_m": round(float(sph.Radius()) * scale, 12),
        }
    return out


def outward_normal(face: Any, scale: float) -> tuple[list[float], list[float]]:
    """(unit normal pointing OUT of the owning solid, the point it was sampled at).

    The underlying surface normal is dU x dV. A face carrying TopAbs_REVERSED
    uses the opposite sense, so the cross product must be negated. Getting this
    wrong silently inverts every dihedral angle downstream, which is why
    `test_outward_normals_point_away_from_solid` checks it against solid
    centroids rather than trusting the flag.

    The sample POINT is returned alongside because a normal without the
    location it was taken at is ambiguous on any curved surface. Stage 2 needs
    it to decide whether a cylindrical face is convex or concave: the face
    centroid cannot answer that, since a full 360 degree cylinder has its
    centroid exactly on the axis and no radial direction at all.
    """
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.gp import gp_Pnt, gp_Vec
    from OCP.TopAbs import TopAbs_Orientation

    adaptor = BRepAdaptor_Surface(face.wrapped)
    u = 0.5 * (float(adaptor.FirstUParameter()) + float(adaptor.LastUParameter()))
    v = 0.5 * (float(adaptor.FirstVParameter()) + float(adaptor.LastVParameter()))
    point, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
    adaptor.D1(u, v, point, du, dv)
    n = _norm(_cross(tuple(du.Coord()), tuple(dv.Coord())))
    if face.wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED:
        n = (-n[0], -n[1], -n[2])
    return [round(c, 9) for c in n], _round_vec(point.Coord(), scale)


# How many normals to record along an edge of a curved face. Uniform in the
# edge's own parameter, and the count is odd so the midpoint is always one of
# them.
#
# Recorded only for NON-planar faces: on a plane the normal is constant and the
# face record already carries it, so sampling would be 17 copies of one vector on
# every edge of every plate.
FACE_NORMAL_SAMPLES = 17


def face_normals_along_edge(
    face: Any, wrapped_edge: Any, samples: int = FACE_NORMAL_SAMPLES
) -> dict[str, Any] | None:
    """This face's outward normal at points along one of its edges.

    **Why this exists.** Stage 2 builds the bisector rail -- the work-angle
    reference -- by offsetting the weld line along the bisector of the two groove
    faces, and `detect.py` is deliberately OCCT-free: it sees only what this
    module records. With one normal per face it could offset exactly against a
    plane (constant normal) or a cylinder (closed-form arc) and had to discard
    the joint entirely for anything else -- so an elliptical tube standing on a
    plate, whose side face OCCT calls an `extrusion`, produced no weld at all.
    These samples are what let the bisector VARY along a segment, which is what a
    general groove face needs.

    Normals are oriented outward the same way `_face_normal` does it, by the
    face's own TopAbs orientation, so a REVERSED face does not silently invert
    the rail. Returns None for a planar face, whose normal does not vary.
    """
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve, BRepAdaptor_Surface
    from OCP.gp import gp_Pnt, gp_Vec
    from OCP.ShapeAnalysis import ShapeAnalysis_Surface
    from OCP.TopAbs import TopAbs_Orientation
    from OCP.TopoDS import TopoDS

    adaptor = BRepAdaptor_Surface(face.wrapped)
    if adaptor.GetType().name == "GeomAbs_Plane":
        return None

    edge = TopoDS.Edge(wrapped_edge)
    if BRep_Tool.Degenerated_s(edge):
        return None
    curve = BRepAdaptor_Curve(edge)
    u0, u1 = float(curve.FirstParameter()), float(curve.LastParameter())
    flipped = face.wrapped.Orientation() == TopAbs_Orientation.TopAbs_REVERSED

    # The edge's 3D points have to be turned back into surface parameters to ask
    # for a normal there; the edge's own pcurve is not available for every
    # representation, so this projects, which works for all of them.
    projector = ShapeAnalysis_Surface(BRep_Tool.Surface_s(face.wrapped))
    fractions: list[float] = []
    normals: list[list[float]] = []
    for i in range(samples):
        t = i / (samples - 1)
        uv = projector.ValueOfUV(curve.Value(u0 + (u1 - u0) * t), 1e-6)
        point, du, dv = gp_Pnt(), gp_Vec(), gp_Vec()
        adaptor.D1(uv.X(), uv.Y(), point, du, dv)
        n = _norm(_cross(tuple(du.Coord()), tuple(dv.Coord())))
        if flipped:
            n = (-n[0], -n[1], -n[2])
        if not any(abs(c) > 1e-9 for c in n):
            # A degenerate parametrization at this sample. Reported as absent
            # rather than filled with a guess -- stage 2 then declines the rail
            # instead of building one from a zero vector.
            return None
        fractions.append(round(t, 9))
        normals.append([round(c, 9) for c in n])
    return {"t": fractions, "normals_xyz": normals}


# --------------------------------------------------------------------------
# extraction
# --------------------------------------------------------------------------


def load_step(path: Path) -> Any:
    from cadquery import importers

    imported = importers.importStep(str(path))
    return imported.val() if hasattr(imported, "val") else imported


def extract_topology(
    step_path: Path, unit_scale_to_m: float = DEFAULT_UNIT_SCALE_TO_M
) -> dict[str, Any]:
    scale = unit_scale_to_m
    shape = load_step(step_path)

    # Deterministic ordering. OCCT traversal order is not guaranteed stable
    # across versions, so identifiers are assigned from sorted geometry and
    # every entity also carries a content hash for cross-run comparison.
    solids = sorted_solids(shape)

    doc_solids: list[dict[str, Any]] = []
    doc_faces: list[dict[str, Any]] = []
    doc_edges: list[dict[str, Any]] = []
    # Edge USES that collapsed to a point, summed over faces. Reported because
    # a count of zero on one workpiece and six on the next is the difference
    # between a shape with a cone in it and one without.
    degenerate_edges = 0

    # Edges are shared between faces of the same solid; key them by content so
    # the face->edge and edge->face maps joint detection needs stay consistent.
    edge_index: dict[str, dict[str, Any]] = {}

    for si, solid in enumerate(solids):
        solid_id = f"solid_{si}"
        bb = solid.BoundingBox()
        faces = sorted(
            solid.Faces(),
            key=lambda f: (
                f.geomType(),
                -round(f.Area(), 6),
                tuple(round(c, 6) for c in f.Center().toTuple()),
            ),
        )

        face_ids: list[str] = []
        for fi, face in enumerate(faces):
            face_id = f"{solid_id}:face_{fi:04d}"
            centroid = face.Center().toTuple()
            fbb = face.BoundingBox()
            normal, normal_at = outward_normal(face, scale)
            record: dict[str, Any] = {
                "id": face_id,
                "solid_id": solid_id,
                "area_m2": round(float(face.Area()) * scale * scale, 12),
                "centroid_xyz_m": _round_vec(centroid, scale),
                "outward_normal_xyz": normal,
                "normal_sample_xyz_m": normal_at,
                "bbox_min_xyz_m": _round_vec((fbb.xmin, fbb.ymin, fbb.zmin), scale),
                "bbox_max_xyz_m": _round_vec((fbb.xmax, fbb.ymax, fbb.zmax), scale),
                "geom_key": face_geom_key(face),
            }
            record.update(surface_of_face(face, scale))

            # cadquery's Face.Edges() silently deduplicates the seam edge of a
            # periodic surface, so a cylinder reports 3 edges where OCCT has 4
            # uses. Explore raw: stage 2 needs every use, with its orientation,
            # to order edges into wires and to skip seams.
            edge_uses: list[dict[str, Any]] = []
            for wrapped_edge, orientation, is_seam in _face_edge_uses(face):
                # A DEGENERATED edge has no 3D curve at all -- it is the apex of
                # a cone or the pole of a sphere, where a whole parametric edge
                # collapses to a point. `BRep_Tool.Curve_s` hands back a null
                # handle for one, and OCCT then SEGFAULTS inside
                # `Geom_TrimmedCurve` rather than raising, which takes the pool
                # worker with it and leaves `ProcessPoolExecutor` reporting
                # `BrokenProcessPool` for every job after it.
                #
                # Skipped rather than repaired: it has zero length and no
                # curve, so it can never be a weld line, and it is a
                # parametrization artifact of exactly the kind `is_seam`
                # already marks. Counted into the summary so a workpiece that
                # has them says so instead of quietly having fewer edges.
                if _is_degenerate_edge(wrapped_edge):
                    degenerate_edges += 1
                    continue
                edge = _as_cq_edge(wrapped_edge)
                ec = edge.Center().toTuple()
                key = edge_geom_key(solid_id, edge)
                existing = edge_index.get(key)
                if existing is None:
                    # Endpoints follow the CURVE's parametrization, not the
                    # edge's face-relative orientation, so start/end, the
                    # analytic block and the B-spline poles all agree. 52 of 62
                    # line edges here are TopAbs_REVERSED, where cadquery's
                    # startPoint() is the curve's LAST point.
                    start, end = _curve_endpoints(edge)
                    entry = {
                        "id": f"{solid_id}:edge_{len(edge_index):04d}",
                        "solid_id": solid_id,
                        "curve_type": edge.geomType().lower(),
                        "length_m": round(float(edge.Length()) * scale, 12),
                        "closed": bool(edge.IsClosed()),
                        "start_xyz_m": _round_vec(start, scale),
                        "end_xyz_m": _round_vec(end, scale),
                        "centroid_xyz_m": _round_vec(ec, scale),
                        "bspline": bspline_of_edge(edge, scale),
                        "face_ids": [],
                        "use_count": 0,
                        "is_seam": False,
                        "geom_key": key,
                    }
                    analytic = analytic_of_edge(edge, scale)
                    if analytic is not None:
                        entry["analytic"] = analytic
                    edge_index[key] = entry
                    doc_edges.append(entry)
                    existing = entry
                existing["face_ids"].append(face_id)
                existing["use_count"] += 1
                existing["is_seam"] = existing["is_seam"] or is_seam
                use: dict[str, Any] = {
                    "edge_id": existing["id"],
                    "orientation": orientation,
                    "seam": is_seam,
                }
                # The normal of THIS face along THIS edge, for a curved face.
                # Stage 2's rail needs the groove normal as a function of
                # position, and it has no geometry kernel of its own.
                sampled = face_normals_along_edge(face, wrapped_edge)
                if sampled is not None:
                    use["face_normals"] = sampled
                edge_uses.append(use)

            record["edge_uses"] = edge_uses
            record["edge_ids"] = sorted({u["edge_id"] for u in edge_uses})
            doc_faces.append(record)
            face_ids.append(face_id)

        doc_solids.append(
            {
                "id": solid_id,
                "volume_m3": round(float(solid.Volume()) * scale**3, 15),
                "centroid_xyz_m": _round_vec(solid.Center().toTuple(), scale),
                "bbox_min_xyz_m": _round_vec((bb.xmin, bb.ymin, bb.zmin), scale),
                "bbox_max_xyz_m": _round_vec((bb.xmax, bb.ymax, bb.zmax), scale),
                "face_ids": face_ids,
                "geom_key": geom_key(
                    "S", float(solid.Volume()), *(float(c) for c in solid.Center().toTuple())
                ),
            }
        )

    return {
        "schema": SCHEMA,
        "source": {
            "kind": "step",
            "path": str(step_path),
            "sha256": hashlib.sha256(step_path.read_bytes()).hexdigest(),
            "unit_scale_to_m": scale,
            "importer": "cadquery.importers.importStep",
        },
        "summary": {
            "solid_count": len(doc_solids),
            "face_count": len(doc_faces),
            "edge_count": len(doc_edges),
            "surface_types": sorted({f["surface_type"] for f in doc_faces}),
            "curve_types": sorted({e["curve_type"] for e in doc_edges}),
            "degenerate_edge_uses": degenerate_edges,
        },
        "solids": doc_solids,
        "faces": doc_faces,
        "edges": doc_edges,
    }


# --------------------------------------------------------------------------
# cli
# --------------------------------------------------------------------------


def repo_root() -> Path:
    """The repository this module sits inside. See `paths.py`."""
    return paths.REPO_ROOT


def self_test() -> int:
    """Fast smoke check for the pixi task. The real suite is `pixi run test`."""
    step = repo_root() / "cadquery" / "v1" / "step" / "dog_box.step"
    if not step.exists():
        print(f"topology self-test: missing fixture {step}", file=sys.stderr)
        return 1
    doc = extract_topology(step)
    s = doc["summary"]
    assert s["solid_count"] == 6, s
    assert s["face_count"] > 0 and s["edge_count"] > 0, s
    for edge in doc["edges"]:
        bs = edge["bspline"]
        assert len(bs["knots_u"]) == len(bs["control_points_xyz_m"]) + bs["degree"] + 1, edge["id"]
    print(
        f"topology self-test ok: {s['solid_count']} solids, {s['face_count']} faces, "
        f"{s['edge_count']} edges, surfaces {s['surface_types']}, curves {s['curve_types']}"
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", default="", help="STEP file to extract")
    parser.add_argument("--out", default="", help="write topology JSON here")
    parser.add_argument("--unit-scale-to-m", type=float, default=DEFAULT_UNIT_SCALE_TO_M)
    parser.add_argument("--summary", action="store_true", help="print the summary only")
    parser.add_argument("--self-test", action="store_true", help="run the worker smoke check")
    ns = parser.parse_args(argv)

    if ns.self_test:
        return self_test()
    if not ns.step:
        raise ValueError("--step is required unless --self-test is set")

    doc = extract_topology(Path(ns.step), ns.unit_scale_to_m)
    if ns.out:
        out = Path(ns.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
        print(f"wrote topology: {ns.out}")
    if ns.summary or not ns.out:
        print(json.dumps(doc["summary"] if ns.summary else doc, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"topology: {exc}", file=sys.stderr)
        raise SystemExit(1)
