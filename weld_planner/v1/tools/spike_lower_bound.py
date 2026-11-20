#!/usr/bin/env python
"""Is coal's `distance_lower_bound` sound, and how much does it buy?

The M3 spike (`spike_ccd.py`) established that no candidate backend has a
working swept-volume CCD for mesh pairs, so the T2 verifier has to build its own
conservative advancement on top of discrete queries. Conservative advancement
does not need the exact distance -- it needs a CERTIFIED LOWER BOUND on it, so
it can advance by `bound / speed` and know it cannot have skipped a contact.

coal's `collide()` returns one alongside the boolean, and the first
measurement said it costs 0.2 us where the exact distance costs 200. A
500-fold difference in the inner loop of the verifier is worth an hour of
checking, because the failure mode if the bound is ever LOOSE IN THE WRONG
DIRECTION -- reported larger than the truth -- is a verifier that advances past a
collision and passes a trajectory that hits the fixture.

So two questions, in order:

1. **Is `bound <= exact` always?** Checked over random poses spanning far,
   near and penetrating, against `distance()` on the same pair. One
   counterexample kills the idea.
2. **How tight is it, and what does it cost?** A sound but useless bound
   (always zero) would also pass question 1. Tightness sets the step length
   and therefore how many queries a segment needs.

Run: `pixi run -e bench python tools/spike_lower_bound.py`.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

MESH_ROOT = Path(__file__).resolve().parents[3] / "robot_description" / "robots" / "rosie_1400_v3" / "meshes"
SAMPLES = 4000


def _bvh(mesh) -> Any:
    import coal

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
    return model


def run(pair: tuple[str, str], seed: int) -> int:
    import coal
    import trimesh

    a_mesh = trimesh.load(MESH_ROOT / f"{pair[0]}.dae", force="mesh")
    b_mesh = trimesh.load(MESH_ROOT / f"{pair[1]}.dae", force="mesh")
    moving = coal.CollisionObject(_bvh(a_mesh), coal.Transform3s())
    static = coal.CollisionObject(_bvh(b_mesh), coal.Transform3s())

    print(f"lower-bound: {pair[0]} ({len(a_mesh.faces)} tris) vs "
          f"{pair[1]} ({len(b_mesh.faces)} tris), {SAMPLES} poses\n")

    # Poses drawn tight around the static mesh so a good share of them
    # penetrate: a sample that is always metres away tests nothing.
    rng = np.random.default_rng(seed)
    lo, hi = b_mesh.bounds
    span = hi - lo
    centres = rng.uniform(lo - 0.25 * span, hi + 0.25 * span, size=(SAMPLES, 3))

    request = coal.CollisionRequest()
    request.enable_distance_lower_bound = True

    exact = np.empty(SAMPLES)
    bound = np.empty(SAMPLES)
    hit = np.zeros(SAMPLES, dtype=bool)

    t_exact = t_bound = 0.0
    for i, c in enumerate(centres):
        moving.setTransform(coal.Transform3s(np.eye(3), c))

        t0 = time.perf_counter()
        result = coal.CollisionResult()
        coal.collide(moving, static, request, result)
        t_bound += time.perf_counter() - t0
        bound[i] = result.distance_lower_bound
        hit[i] = result.isCollision()

        t0 = time.perf_counter()
        dresult = coal.DistanceResult()
        exact[i] = coal.distance(moving, static, coal.DistanceRequest(), dresult)
        t_exact += time.perf_counter() - t0

    clear = ~hit
    # Question 1. Only meaningful where there IS a positive distance to bound;
    # for a penetrating pair the "distance" is negative and the contract is
    # just that `isCollision()` is true.
    violations = np.nonzero(clear & (bound > exact + 1e-9))[0]
    print(f"  in contact:      {int(hit.sum())}/{SAMPLES}")
    print(f"  bound > exact:   {len(violations)}  <- must be 0")
    if len(violations):
        for i in violations[:5]:
            print(f"    pose {i}: bound {bound[i]:.6f} > exact {exact[i]:.6f}")

    # Question 2.
    ratio = bound[clear] / np.maximum(exact[clear], 1e-9)
    print(f"  tightness:       median {np.median(ratio):.3f} of the true distance, "
          f"p10 {np.percentile(ratio, 10):.3f}, p90 {np.percentile(ratio, 90):.3f}")
    print(f"  bound cost:      {t_bound / SAMPLES * 1e6:8.2f} us")
    print(f"  exact cost:      {t_exact / SAMPLES * 1e6:8.2f} us")
    print(f"  speedup:         {t_exact / t_bound:8.1f}x")

    # The bound is only useful if it is nonzero where it matters -- a bound of
    # zero forces the advancement to a standstill.
    zeroish = int((clear & (bound < 1e-6)).sum())
    print(f"  bound ~= 0 while clear: {zeroish}/{int(clear.sum())}"
          f"  <- these force a fallback to the exact query")

    if len(violations):
        print("\nlower-bound: FAIL, the bound is not a lower bound")
        return 1
    print("\nlower-bound: sound over this sample")

    # --------------------------------------------------------------------
    # The query the verifier actually wants is not "how far" and not "a bound on
    # how far" but "IS ANYTHING CLOSER THAN m" -- because conservative
    # advancement with a step of `m / speed` needs exactly that predicate and
    # nothing more. `security_margin` inflates both bodies by m, so an
    # `isCollision()` of false is a certificate that the gap exceeds m, and
    # the traversal can stop at the first box pair that proves it.
    print(f"\n  the verifier's real primitive -- \"closer than m?\" over the same {SAMPLES} poses")
    print(f"  {'margin':>8}  {'us':>8}  {'vs exact':>9}  {'says yes':>9}   agrees with distance()")
    for margin in (0.001, 0.005, 0.02, 0.05, 0.2):
        request = coal.CollisionRequest()
        request.security_margin = margin
        yes = 0
        wrong = 0
        t0 = time.perf_counter()
        for i, c in enumerate(centres):
            moving.setTransform(coal.Transform3s(np.eye(3), c))
            result = coal.CollisionResult()
            coal.collide(moving, static, request, result)
            near = result.isCollision()
            yes += int(near)
            # `exact` is -1 for a penetrating pair, which is "closer than m"
            # for every m, so compare on that footing.
            truth = hit[i] or (exact[i] < margin)
            wrong += int(near != truth)
        us = (time.perf_counter() - t0) / SAMPLES * 1e6
        print(f"  {margin * 1000:6.0f}mm  {us:8.2f}  {t_exact / SAMPLES * 1e6 / us:8.1f}x  "
              f"{yes:6d}/{SAMPLES}   {'all' if wrong == 0 else f'{wrong} DISAGREE'}")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pair", nargs=2, default=["link_3", "h_frame"])
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    return run((args.pair[0], args.pair[1]), args.seed)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
