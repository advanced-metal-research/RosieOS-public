#!/usr/bin/env python
"""The M3 backend spike: which library gets to be the T2 verifier's geometry engine.

`IMPLEMENTATION.md` says the choice is made by a spike against the ACTUAL
fixture meshes rather than from the libraries' README pages, and this is that
spike. Three candidates, five questions, ordered so that failing an early one
makes the later numbers irrelevant.

1. **Does it get a known answer right?** Two boxes at an analytic separation,
   separated and overlapping. A backend that misses this is out regardless of
   how fast it is.
2. **Does it do concave-vs-concave at all?** Every robot link here is concave
   and so is the workpiece. A backend that silently convexifies is answering a
   different question, and answering it in the unsafe direction inside a
   pocket -- it reports clearance where the real geometry has a wall.
3. **How fast is a discrete exact distance query on the real cell meshes?**
   This is what the verifier's throughput is built out of if the certificate ends
   up being discrete distance plus a motion bound.
4. **Is there a working swept-volume CCD, and does it agree with dense
   sampling?** A time-of-contact is only worth having if it matches the answer
   a thousand discrete checks give. Checked against exactly that.
5. **Does it catch the tunnelling case?** A 4 mm plate crossed in one step,
   both endpoints clear by 250 mm. This is the case the verifier exists for.

Run: `pixi run -e bench motion-ccd-spike`.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

import numpy as np

MESH_ROOT = Path(__file__).resolve().parents[3] / "robot_description" / "robots" / "rosie_1400_v3" / "meshes"
GAP_M = 0.25
REPEATS = 2000
DENSE = 4000  # samples in the ground-truth sweep


def _pose(xyz, rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    """A 4x4, built without scipy so every backend sees the same numbers."""
    cr, cp, cy = np.cos(rpy)
    sr, sp, sy = np.sin(rpy)
    rot = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = xyz
    return out


def _c_shape():
    """A concave body with a slot, built by concatenation rather than by a
    boolean -- trimesh's boolean backend is an optional native dependency and
    this spike should not need one to answer a geometry question.

    Three 200x200 mm slabs forming a C opening toward +z, with a 120 mm slot
    120 mm deep. A probe parked in the mouth is 40 mm from the nearest wall
    and 60 mm inside the convex hull.
    """
    import trimesh

    parts = [
        trimesh.creation.box(extents=(0.2, 0.2, 0.08)),                    # the back
        trimesh.creation.box(extents=(0.04, 0.2, 0.12)),                   # two walls
        trimesh.creation.box(extents=(0.04, 0.2, 0.12)),
    ]
    parts[0].apply_translation([0.0, 0.0, -0.06])
    parts[1].apply_translation([-0.08, 0.0, 0.06])
    parts[2].apply_translation([+0.08, 0.0, 0.06])
    return trimesh.util.concatenate(parts)


def _plate():
    import trimesh

    return trimesh.creation.box(extents=(0.004, 0.6, 0.6))


def _cell_meshes() -> dict[str, Any]:
    import trimesh

    if not MESH_ROOT.is_dir():
        raise SystemExit(f"spike-ccd: no cell meshes under {MESH_ROOT}")
    return {n: trimesh.load(MESH_ROOT / f"{n}.dae", force="mesh")
            for n in ("link_3", "link_6", "h_frame", "floor")}


# --------------------------------------------------------------------------
# python-fcl


def fcl_backend(**_) -> dict[str, Any]:
    import fcl

    def build(mesh):
        model = fcl.BVHModel()
        model.beginModel(len(mesh.vertices), len(mesh.faces))
        model.addSubModel(np.asarray(mesh.vertices, dtype=np.float64),
                          np.asarray(mesh.faces, dtype=np.int32))
        model.endModel()
        return fcl.CollisionObject(model, fcl.Transform())

    def distance(a, b) -> float:
        result = fcl.DistanceResult()
        d = fcl.distance(a, b, fcl.DistanceRequest(enable_nearest_points=False), result)
        # FCL returns -1 for "penetrating" rather than a depth. Collapse it to
        # a single sentinel so the callers here can just test `<= 0`.
        return -1.0 if d < 0 else float(d)

    def place(obj, matrix) -> None:
        obj.setTransform(fcl.Transform(np.ascontiguousarray(matrix[:3, :3], dtype=np.float64),
                                       np.ascontiguousarray(matrix[:3, 3], dtype=np.float64)))

    def swept(a, b, a_to, b_to, *, motion: int = 1, solver: int = 1):
        request = fcl.ContinuousCollisionRequest()
        request.ccd_motion_type = motion
        request.ccd_solver_type = solver
        result = fcl.ContinuousCollisionResult()
        fcl.continuousCollide(
            a, fcl.Transform(np.ascontiguousarray(a_to[:3, :3]), np.ascontiguousarray(a_to[:3, 3])),
            b, fcl.Transform(np.ascontiguousarray(b_to[:3, :3]), np.ascontiguousarray(b_to[:3, 3])),
            request, result)
        return float(result.time_of_contact) if result.is_collide else None

    def primitive(half: float, xyz) -> Any:
        return fcl.CollisionObject(fcl.Box(2 * half, 2 * half, 2 * half),
                                   fcl.Transform(np.asarray(xyz, dtype=np.float64)))

    def sphere(radius: float, xyz) -> Any:
        return fcl.CollisionObject(fcl.Sphere(radius),
                                   fcl.Transform(np.asarray(xyz, dtype=np.float64)))

    return {"name": "python-fcl", "build": build, "distance": distance,
            "place": place, "swept": swept, "primitive": primitive, "sphere": sphere}


# --------------------------------------------------------------------------
# coal (the hpp-fcl fork, conda-forge)


def coal_backend(**_) -> dict[str, Any]:
    import coal

    def build(mesh):
        model = coal.BVHModelOBBRSS()
        verts = np.asarray(mesh.vertices, dtype=np.float64)
        faces = np.asarray(mesh.faces, dtype=np.int64)
        # eigenpy's StdVector converters, not python lists: `addSubModel` is
        # bound to `std::vector<Vec3s>` and a list of numpy rows does not
        # convert. The error names both C++ overloads, neither of which
        # mentions ndarray, which is the whole hint.
        vs, ts = coal.StdVec_Vec3s(), coal.StdVec_Triangle()
        for v in verts:
            vs.append(v)
        for f in faces:
            ts.append(coal.Triangle(int(f[0]), int(f[1]), int(f[2])))
        model.beginModel(len(faces), len(verts))
        model.addSubModel(vs, ts)
        model.endModel()
        return coal.CollisionObject(model, coal.Transform3s())

    def distance(a, b) -> float:
        result = coal.DistanceResult()
        return float(coal.distance(a, b, coal.DistanceRequest(), result))

    def place(obj, matrix) -> None:
        obj.setTransform(coal.Transform3s(np.ascontiguousarray(matrix[:3, :3], dtype=np.float64),
                                          np.ascontiguousarray(matrix[:3, 3], dtype=np.float64)))

    def primitive(half: float, xyz) -> Any:
        return coal.CollisionObject(coal.Box(2 * half, 2 * half, 2 * half),
                                    coal.Transform3s(np.eye(3), np.asarray(xyz, dtype=np.float64)))

    return {"name": "coal", "build": build, "distance": distance,
            "place": place, "swept": None, "primitive": primitive}


# --------------------------------------------------------------------------
# Bullet, through pybullet


def bullet_backend(*, concave: bool = True) -> dict[str, Any]:
    import pybullet as p

    if p.getConnectionInfo().get("isConnected"):
        p.disconnect()
    p.connect(p.DIRECT)

    def build(mesh):
        # GEOM_FORCE_CONCAVE_TRIMESH is what "exact mesh" means here. Without
        # it Bullet takes the convex hull of the vertices, which is a
        # different body -- and the spike runs BOTH so the difference is a
        # measurement rather than a claim.
        kwargs = {}
        if concave:
            kwargs["flags"] = p.GEOM_FORCE_CONCAVE_TRIMESH
        shape = p.createCollisionShape(
            p.GEOM_MESH,
            vertices=np.asarray(mesh.vertices, dtype=np.float64).tolist(),
            indices=np.asarray(mesh.faces, dtype=np.int32).reshape(-1).tolist(),
            **kwargs,
        )
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape)

    def distance(a, b) -> float:
        points = p.getClosestPoints(bodyA=a, bodyB=b, distance=10.0)
        if not points:
            return float("nan")  # not "far": Bullet declined to answer
        return float(min(pt[8] for pt in points))

    def place(obj, matrix) -> None:
        quat = _quat_from_matrix(matrix[:3, :3])
        p.resetBasePositionAndOrientation(obj, matrix[:3, 3].tolist(), quat)

    def primitive(half: float, xyz) -> Any:
        shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half] * 3)
        body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape)
        p.resetBasePositionAndOrientation(body, list(xyz), [0, 0, 0, 1])
        return body

    return {"name": "pybullet-concave" if concave else "pybullet-convex",
            "build": build, "distance": distance, "place": place,
            "swept": None, "primitive": primitive}


def _quat_from_matrix(rot: np.ndarray) -> list[float]:
    """xyzw, Shepperd's method. Written out so the spike does not drag scipy in
    for three lines and so every backend gets the same rotation."""
    m = np.asarray(rot, dtype=np.float64)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return [float(x), float(y), float(z), float(w)]


BACKENDS: dict[str, Callable[..., dict[str, Any]]] = {
    "python-fcl": fcl_backend,
    "coal": coal_backend,
    "pybullet-concave": lambda: bullet_backend(concave=True),
    "pybullet-convex": lambda: bullet_backend(concave=False),
}


# --------------------------------------------------------------------------
# The questions


def q0_pair_support(backend: dict[str, Any]) -> dict[str, Any]:
    """Which geometry pairs the backend will answer for at all.

    Asked first because a backend that declines a pair is not slow or
    inaccurate -- it returns nothing, and a caller that reads "no contact
    points" as "no contact" has a verifier that passes everything. Mesh-vs-mesh is
    the only row that matters here: every arm link, the h-frame and the
    workpiece are all meshes.
    """
    import trimesh

    box = trimesh.creation.box(extents=(0.2, 0.2, 0.2))
    near = trimesh.creation.box(extents=(0.2, 0.2, 0.2))
    near.apply_translation([0.45, 0.0, 0.0])  # 250 mm apart, walls facing

    out = {}
    for label, pair in (("mesh-mesh", (box, near)),):
        a, b = backend["build"](pair[0]), backend["build"](pair[1])
        d = backend["distance"](a, b)
        out[label] = "declined" if np.isnan(d) else round(d, 4)
    if backend.get("primitive"):
        a, b = backend["primitive"](0.1, [0.0, 0, 0]), backend["primitive"](0.1, [0.45, 0, 0])
        d = backend["distance"](a, b)
        out["primitive-primitive"] = "declined" if np.isnan(d) else round(d, 4)
        a = backend["build"](box)
        b = backend["primitive"](0.1, [0.45, 0, 0])
        d = backend["distance"](a, b)
        out["mesh-primitive"] = "declined" if np.isnan(d) else round(d, 4)
    return out


def q1_known_answer(backend: dict[str, Any]) -> dict[str, Any]:
    import trimesh

    a_mesh = trimesh.creation.box(extents=(0.2, 0.2, 0.2))
    b_mesh = trimesh.creation.box(extents=(0.2, 0.2, 0.2))
    b_mesh.apply_translation([0.2 + GAP_M, 0.0, 0.0])
    over = trimesh.creation.box(extents=(0.2, 0.2, 0.2))
    over.apply_translation([0.1, 0.0, 0.0])

    a, b, c = (backend["build"](m) for m in (a_mesh, b_mesh, over))
    apart, touching = backend["distance"](a, b), backend["distance"](a, c)
    return {
        "separated_m": None if np.isnan(apart) else round(apart, 6),
        "error_mm": None if np.isnan(apart) else round(abs(apart - GAP_M) * 1000.0, 6),
        "overlap_reports_contact": bool(touching <= 0.0),  # nan compares False
        "declined": bool(np.isnan(apart) or np.isnan(touching)),
    }


def q2_concavity(backend: dict[str, Any]) -> dict[str, Any]:
    """A probe in the mouth of a C: 40 mm from the wall, 60 mm inside the hull."""
    import trimesh

    probe = trimesh.creation.box(extents=(0.04, 0.04, 0.04))
    probe.apply_translation([0.0, 0.0, 0.06])
    a, b = backend["build"](_c_shape()), backend["build"](probe)
    d = backend["distance"](a, b)
    return {
        "distance_m": None if np.isnan(d) else round(d, 6),
        "exact_m": 0.04,
        "matches_exact": bool(not np.isnan(d) and abs(d - 0.04) < 0.002),
        "convexified": bool(not np.isnan(d) and d <= 0.0),
        "declined": bool(np.isnan(d)),
    }


def q3_throughput(backend: dict[str, Any], meshes: dict[str, Any]) -> dict[str, Any]:
    rng = np.random.default_rng(0)
    t0 = time.perf_counter()
    link = backend["build"](meshes["link_3"])
    frame = backend["build"](meshes["h_frame"])
    build_ms = (time.perf_counter() - t0) * 1000.0

    matrices = np.tile(np.eye(4), (REPEATS, 1, 1))
    matrices[:, :3, 3] = rng.uniform([-0.7, -0.9, 0.0], [0.7, 0.9, 0.7], size=(REPEATS, 3))

    backend["place"](link, matrices[0])
    backend["distance"](link, frame)

    t0 = time.perf_counter()
    hits = declined = 0
    for m in matrices:
        backend["place"](link, m)
        d = backend["distance"](link, frame)
        if np.isnan(d):
            declined += 1
        elif d <= 0.0:
            hits += 1
    elapsed = time.perf_counter() - t0
    return {
        "build_ms": round(build_ms, 2),
        "us_per_query": round(elapsed / REPEATS * 1e6, 2),
        "queries_per_s": int(REPEATS / elapsed),
        "in_contact": f"{hits}/{REPEATS}",
        "declined": declined,
    }


def _first_contact(backend, moving, static, lo, hi, samples: int = DENSE) -> float | None:
    """Ground truth by brute force: the earliest `t` at which dense sampling
    of the same interpolation finds contact. Slow on purpose."""
    for i in range(samples + 1):
        t = i / samples
        backend["place"](moving, _lerp(lo, hi, t))
        d = backend["distance"](moving, static)
        if not np.isnan(d) and d <= 0.0:
            return t
    return None


def _lerp(lo: np.ndarray, hi: np.ndarray, t: float) -> np.ndarray:
    """Translation lerp with the rotation held -- the motion model FCL's
    CCDM_TRANS assumes, so the ground truth and the CCD agree on what "the
    motion" is."""
    out = lo.copy()
    out[:3, 3] = lo[:3, 3] * (1 - t) + hi[:3, 3] * t
    return out


def q4_ccd_control(backend: dict[str, Any]) -> dict[str, Any]:
    """Two spheres on a collision course, whose time of contact is arithmetic.

    THE control for question 5. Radius 0.1 each, starting 1 m apart, closing
    at 2 m per unit time: they touch when the gap reaches 0.2 m, i.e. at
    t = 0.4. If the backend gets this right its CCD entry point is being
    driven correctly, and a wrong answer on meshes afterwards is the
    backend's and not the harness's.
    """
    if backend["swept"] is None or "sphere" not in backend:
        return {"available": False}
    a = backend["sphere"](0.1, [-1.0, 0, 0])
    b = backend["sphere"](0.1, [0.0, 0, 0])
    out: dict[str, Any] = {"available": True, "exact_toc": 0.4}
    for label, motion in (("trans", 0), ("linear", 1)):
        toc = backend["swept"](a, b, _pose([1.0, 0, 0]), np.eye(4), motion=motion)
        out[label] = {"toc": toc, "correct": bool(toc is not None and abs(toc - 0.4) < 0.01)}
    return out


def q5_agrees_with_dense(backend: dict[str, Any], meshes: dict[str, Any]) -> dict[str, Any]:
    """A real link crossing the real h_frame, CCD against 4000 samples."""
    if backend["swept"] is None:
        return {"available": False}
    link, frame = backend["build"](meshes["link_3"]), backend["build"](meshes["h_frame"])
    lo, hi = _pose([0.0, -1.4, 0.30]), _pose([0.0, 1.4, 0.30])

    t0 = time.perf_counter()
    truth = _first_contact(backend, link, frame, lo, hi)
    dense_ms = (time.perf_counter() - t0) * 1000.0

    out: dict[str, Any] = {"available": True, "dense_toc": truth,
                           "dense_ms": round(dense_ms, 1)}
    for label, motion in (("trans", 0), ("linear", 1), ("screw", 2)):
        backend["place"](link, lo)
        t0 = time.perf_counter()
        try:
            toc = backend["swept"](link, frame, hi, np.eye(4), motion=motion)
            err = None
        except Exception as exc:  # noqa: BLE001 - recording the failure IS the result
            toc, err = None, f"{type(exc).__name__}"
        out[label] = {
            "toc": None if toc is None else round(toc, 4),
            "ms": round((time.perf_counter() - t0) * 1000.0, 3),
            "agrees": bool(truth is not None and toc is not None and abs(toc - truth) < 0.01),
            "error": err,
        }
    return out


def q6_tunnelling(backend: dict[str, Any], meshes: dict[str, Any]) -> dict[str, Any]:
    """A 4 mm plate crossed in one step, both endpoints clear by 250 mm."""
    link = backend["build"](meshes["link_3"])
    plate = backend["build"](_plate())
    lo, hi = _pose([-0.5, -0.1, -0.3]), _pose([0.5, -0.1, -0.3])

    backend["place"](link, lo)
    d_lo = backend["distance"](link, plate)
    backend["place"](link, hi)
    d_hi = backend["distance"](link, plate)
    truth = _first_contact(backend, link, plate, lo, hi)

    out: dict[str, Any] = {
        "endpoints_m": [None if np.isnan(d_lo) else round(d_lo, 4),
                        None if np.isnan(d_hi) else round(d_hi, 4)],
        "endpoints_clear": bool(d_lo > 0 and d_hi > 0),
        "really_collides": truth is not None,
    }
    if backend["swept"] is not None:
        backend["place"](link, lo)
        t0 = time.perf_counter()
        try:
            toc = backend["swept"](link, plate, hi, np.eye(4), motion=0)
            err = None
        except Exception as exc:  # noqa: BLE001
            toc, err = None, f"{type(exc).__name__}"
        out["ccd_toc"] = None if toc is None else round(toc, 4)
        out["ccd_ms"] = round((time.perf_counter() - t0) * 1000.0, 3)
        out["caught_it"] = bool(toc is not None)
        out["ccd_error"] = err
    return out


def q7_cost_vs_separation(backend: dict[str, Any], meshes: dict[str, Any]) -> dict[str, Any]:
    """Query time at 5 mm, 50 mm and 500 mm of clearance.

    Conservative advancement's whole economics live here. It takes a step
    proportional to the current distance, so a segment through open space
    costs a handful of FAR queries and a segment threading a fixture costs
    many NEAR ones. If far queries were as expensive as near ones the verifier's
    cost would scale with trajectory length; if they early-out, it scales
    with how much of the trajectory is actually tight.
    """
    link = backend["build"](meshes["link_3"])
    frame = backend["build"](meshes["h_frame"])

    def at(x: float) -> float:
        backend["place"](link, _pose([x, 0.0, 0.3]))
        return backend["distance"](link, frame)

    def offset_for(gap: float) -> float:
        """Bisect for the x that puts the link exactly `gap` from the frame.

        Solved rather than computed from bounding boxes: the h-frame is a
        FRAME, hollow and open in the middle, so "the wall is at x = 0.6" is
        not a fact about the distance to it. The first version of this
        measured 377 mm while claiming 5 mm.
        """
        lo, hi = 0.0, 3.0
        for _ in range(60):
            mid = 0.5 * (lo + hi)
            d = at(mid)
            if d < 0 or d < gap:
                lo = mid
            else:
                hi = mid
        return hi

    out: dict[str, Any] = {}
    for label, gap in (("5mm", 0.005), ("50mm", 0.05), ("500mm", 0.5)):
        measured = at(offset_for(gap))
        reps = 400
        backend["distance"](link, frame)
        t0 = time.perf_counter()
        for _ in range(reps):
            backend["distance"](link, frame)
        out[label] = {"us": round((time.perf_counter() - t0) / reps * 1e6, 1),
                      "measured_m": None if np.isnan(measured) else round(measured, 4)}
    return out


def run(names: list[str]) -> int:
    meshes = _cell_meshes()
    print(f"spike-ccd: cell meshes from {MESH_ROOT}")
    print(f"  link_3 {len(meshes['link_3'].faces)} tris, "
          f"h_frame {len(meshes['h_frame'].faces)} tris\n")

    ran = 0
    for name in names:
        print(f"--- {name}")
        try:
            backend = BACKENDS[name]()
        except Exception as exc:  # noqa: BLE001
            print(f"  unavailable: {type(exc).__name__}: {exc}\n")
            continue
        for label, fn in (("pair support", lambda b: q0_pair_support(b)),
                          ("known answer", lambda b: q1_known_answer(b)),
                          ("concavity", lambda b: q2_concavity(b)),
                          ("throughput", lambda b: q3_throughput(b, meshes)),
                          ("ccd control", lambda b: q4_ccd_control(b)),
                          ("ccd vs dense", lambda b: q5_agrees_with_dense(b, meshes)),
                          ("tunnelling", lambda b: q6_tunnelling(b, meshes)),
                          ("cost vs gap", lambda b: q7_cost_vs_separation(b, meshes))):
            try:
                value: Any = fn(backend)
            except Exception as exc:  # noqa: BLE001
                value = {"error": f"{type(exc).__name__}: {exc}"}
            print(f"  {label:13s} {value}")
        ran += 1
        print()
    return 0 if ran else 1


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--only", nargs="*", choices=sorted(BACKENDS), default=None)
    args = parser.parse_args(argv)
    return run(args.only or list(BACKENDS))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
