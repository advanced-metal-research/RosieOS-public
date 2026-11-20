#!/usr/bin/env python
"""The whole collision term at once: joints in, per-sphere clearance and
its derivative out, on random poses.

`bench_fk_curobo.py` timed the kinematics alone and `bench_esdf.py` the
field alone. This runs them as the optimizer will: a batch of random joint
vectors through the fused FK into the fused ESDF, producing

    d[b, s] = field(p[b, s]) - radius[s]        clearance per sphere
    dd/dq[b, a]                                 back through both

## Accuracy is measured against truth, not against the other implementation

The field is a 5 mm trilinear grid, so it has error, and the honest way to
size it is against the mesh itself: `mesh_query_point_sign_winding_number`
gives the exact signed distance and the exact closest point at each sphere
centre, on the same BVH the field was baked from but with no grid in the
way.

The exact spatial gradient follows from the closest point,

    grad d = sign * (p - c) / |p - c|

and the exact derivative in joint space is that pushed back through the
kinematics -- which is done with the SAME FK backward the fused path uses,
so the comparison isolates the field's error rather than mixing it with the
chain's. The chain is checked separately in `weld_motion_planner.robot_cell.robot.fk`'s self-test
and agrees with autograd to 1e-7.

Poses are rejection-sampled to be interesting: a random arm configuration
is usually nowhere near anything, and a collision term measured only where
it is inactive says nothing.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch

V1 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(V1 / "python"))

CELL_ROOT = Path(os.environ.get("WELD_CELL_ROOT") or (V1.parents[1] / "robot_description" / "robots"))
# A cuRobo checkout for the comparison; CUROBO_ROOT overrides the default.
CUROBO = Path(os.environ.get("CUROBO_ROOT") or Path.home() / "projects" / "curobo")

from weld_motion_planner.geometry import esdf  # noqa: E402
from weld_motion_planner.robot_cell.collision import pairs  # noqa: E402
from weld_motion_planner.robot_cell.collision import spheres as spheremod  # noqa: E402
from weld_motion_planner.robot_cell.robot import cell as cellmod  # noqa: E402
from weld_motion_planner.robot_cell.robot import fk as fkmod  # noqa: E402

sys.path.insert(0, str(V1 / "tools"))
from bench_collision import env_mesh, read_cell  # noqa: E402


def exact_distance(mesh, points: torch.Tensor):
    """Signed distance and its spatial gradient at `points`, from the BVH.

    No grid. This is what the field is an approximation OF, so it is the
    only thing worth calling the field's error against.
    """
    import warp as wp

    wp.init()
    device = points.device
    warp_device = wp.device_from_torch(device)
    bvh = wp.Mesh(
        points=wp.array(mesh.vertices.astype(np.float32), dtype=wp.vec3, device=warp_device),
        indices=wp.array(mesh.faces.reshape(-1).astype(np.int32), dtype=wp.int32, device=warp_device),
    )
    n = len(points)
    value = torch.empty(n, dtype=torch.float32, device=device)
    grad = torch.empty(n, 3, dtype=torch.float32, device=device)

    @wp.kernel
    def probe(
        mesh_id: wp.uint64,
        pts: wp.array(dtype=wp.vec3),
        reach: float,
        out_d: wp.array(dtype=float),
        out_g: wp.array(dtype=wp.vec3),
    ):
        i = wp.tid()
        p = pts[i]
        query = wp.mesh_query_point_sign_winding_number(mesh_id, p, reach)
        if not query.result:
            out_d[i] = reach
            out_g[i] = wp.vec3(0.0, 0.0, 1.0)
            return
        closest = wp.mesh_eval_position(mesh_id, query.face, query.u, query.v)
        delta = p - closest
        length = wp.length(delta)
        out_d[i] = query.sign * length
        if length > 1.0e-9:
            # The signed distance grows away from the surface on both sides,
            # so the gradient is the outward direction either way -- which is
            # `sign * (p - c)/|p - c|`, not `(p - c)/|p - c|`.
            out_g[i] = (query.sign / length) * delta
        else:
            out_g[i] = wp.vec3(0.0, 0.0, 0.0)

    lo, hi = mesh.bounds
    reach = float(np.linalg.norm(hi - lo)) * 4.0
    wp.launch(
        probe,
        dim=n,
        inputs=[bvh.id, wp.from_torch(points.contiguous(), dtype=wp.vec3), reach,
                wp.from_torch(value), wp.from_torch(grad, dtype=wp.vec3)],
        device=warp_device,
    )
    wp.synchronize_device(warp_device)
    return value, grad


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--plan", default=str(V1 / "data" / "motion" / "bracket_a.weldplan"))
    parser.add_argument("--voxel-mm", type=float, default=5.0)
    parser.add_argument("--poses", type=int, default=256, help="random poses for the accuracy pass")
    parser.add_argument("--batches", default="128,1024,8192")
    parser.add_argument("--reps", type=int, default=60)
    parser.add_argument("--seed", type=int, default=1)
    ns = parser.parse_args(argv)

    if not torch.cuda.is_available():
        print("no CUDA; nothing to run")
        return 0
    device = torch.device("cuda", torch.cuda.current_device())
    torch.manual_seed(ns.seed)

    cell = read_cell(Path(ns.plan))
    cell_id = str(cell.get("id"))
    model = spheremod.load(cell_id)
    if model is None:
        raise SystemExit(f"no sphere model for {cell_id}")
    arm_chain = cellmod.sphere_links(cell)
    on_arm = [n for n in arm_chain if model.links.get(n)]
    env_names = [str(link.get("name")) for link in (cell.get("links") or [])
                 if str(link.get("name")) not in arm_chain]
    covered = pairs.env_checked(cell, on_arm, env_names)

    mesh = env_mesh(cell, cellmod.zero_pose(cell))
    field = esdf.bake(mesh, voxel_m=ns.voxel_mm / 1000.0, margin_m=0.10, frame="world", device=device)
    fused, spec, radii, _sl, _slc = fkmod.make(cell, covered, model.links, device)
    n_spheres = int(radii.numel())
    print(f"{cell_id}: {spec.num_dof} axes, {n_spheres} spheres, "
          f"env {len(mesh.faces)} triangles, field {ns.voxel_mm} mm {list(field.shape)}")

    def clearance(q: torch.Tensor) -> torch.Tensor:
        return field.sample_fused(fused(q)) - radii

    # ---- poses that actually touch the geometry -------------------------
    #
    # Uniform random joints put the arm in the air, where the collision term
    # is flat and any implementation looks accurate. Sample wide, keep the
    # poses whose nearest sphere is within a hand's width of something.
    pool = (torch.rand(ns.poses * 8, spec.num_dof, device=device) - 0.5) * 2.4
    with torch.no_grad():
        near = clearance(pool).min(dim=1).values
    keep = (near < 0.10).nonzero().reshape(-1)[: ns.poses]
    if len(keep) < ns.poses // 4:
        keep = near.argsort()[: ns.poses]
    q = pool[keep].contiguous()
    with torch.no_grad():
        d_here = clearance(q)
    print(f"  accuracy on {len(q)} poses x {n_spheres} spheres = {len(q) * n_spheres:,} clearances")
    print(f"  min {float(d_here.min()) * 1000:.1f} mm, "
          f"{int((d_here < 0).sum())} in collision, "
          f"{int((d_here < 0.05).sum())} inside a 50 mm margin")

    # ---- value accuracy against the mesh --------------------------------
    with torch.no_grad():
        pts = fused(q).reshape(-1, 3).clone()
    truth_d, truth_g = exact_distance(mesh, pts)
    with torch.no_grad():
        ours_d = field.sample_fused(pts)
    inside_box = ~((pts < field.origin_pos_m) | (pts > field.upper_pos_m)).any(dim=-1)
    err = (ours_d - truth_d).abs()
    print("\n  distance value, ours (5 mm trilinear) against the exact mesh:")
    for label, mask in (("inside the grid", inside_box), ("outside it", ~inside_box)):
        if not int(mask.sum()):
            continue
        e = err[mask]
        print(f"    {label:<16} n={int(mask.sum()):>7}  mean {float(e.mean()) * 1000:.3f} mm"
              f"  p99 {float(e.float().quantile(0.99)) * 1000:.3f} mm"
              f"  max {float(e.max()) * 1000:.3f} mm")
    # Outside the grid the field is a documented LOWER bound, so the signed
    # error there should be one-sided. Check that rather than its magnitude.
    if int((~inside_box).sum()):
        signed = (ours_d - truth_d)[~inside_box]
        print(f"    outside it, signed: max overestimate {float(signed.max()) * 1000:.3f} mm "
              f"(a lower bound would keep this at or below zero)")

    # ---- gradient accuracy ----------------------------------------------
    #
    # Both sides go back through the SAME FK backward, so what is left is
    # the field's spatial gradient against the exact one. Restricted to the
    # active set, since a sphere the cost cannot see contributes nothing.
    active = (d_here < 0.05).reshape(-1)
    gate = active.to(torch.float32).unsqueeze(-1)

    qa = q.clone().requires_grad_(True)
    out = fused(qa)
    ours_spatial = torch.autograd.grad(field.sample_fused(out).sum(), out)[0].reshape(-1, 3)

    # Only the active set is pushed back. A sphere the cost cannot see
    # contributes nothing to the derivative the optimizer uses, and most of
    # them are outside the grid where the field is deliberately a lower
    # bound whose gradient points at the box rather than at the geometry --
    # including them measures the extension, not the field.
    qb = q.clone().requires_grad_(True)
    outb = fused(qb)
    outb.backward((truth_g.reshape(outb.shape[0], outb.shape[1], 3) * gate.reshape(outb.shape[0], outb.shape[1], 1)))
    truth_q = qb.grad.clone()

    qc = q.clone().requires_grad_(True)
    outc = fused(qc)
    outc.backward((ours_spatial.reshape(outc.shape[0], outc.shape[1], 3) * gate.reshape(outc.shape[0], outc.shape[1], 1)))
    ours_q = qc.grad.clone()

    cos = torch.nn.functional.cosine_similarity(ours_spatial[active], truth_g[active], dim=-1)
    scale = max(float(truth_q.abs().max()), 1e-12)
    rel = float((ours_q - truth_q).abs().max()) / scale
    print("\n  gradient, ours against the exact mesh, on the active set only:")
    print(f"    spatial direction: mean cos {float(cos.mean()):.5f}, "
          f"p1 {float(cos.float().quantile(0.01)):.5f}, worst {float(cos.min()):.5f}")
    print(f"    joint-space dd/dq: worst relative {rel:.2e}, "
          f"mean relative {float((ours_q - truth_q).abs().mean()) / scale:.2e}")

    # How much of the direction error is the grid and how much is genuine
    # ambiguity: near a medial axis the true gradient flips between two
    # equally-near surfaces, and no interpolant can pick a side.
    poor = cos < 0.99
    if int(poor.sum()):
        d_poor = truth_d[active][poor].abs()
        print(f"    {int(poor.sum())} of {int(active.sum())} active spheres below cos 0.99; "
              f"their |distance| is {float(d_poor.mean()) * 1000:.1f} mm mean, "
              f"{float(d_poor.min()) * 1000:.1f} mm min -- the gradient is worst where the "
              f"distance is near zero, which is where it is genuinely ambiguous")

    # ---- performance ----------------------------------------------------
    def timed(fn, reps: int) -> float:
        for _ in range(5):
            fn()
        torch.cuda.synchronize()
        t0 = time.perf_counter()
        for _ in range(reps):
            fn()
        torch.cuda.synchronize()
        return (time.perf_counter() - t0) / reps

    # Where the iteration's time goes. The three rows nest: kinematics
    # alone, then the field on top, then the hinge cost on top of that --
    # so each column's increment is that stage's own cost, backward
    # included.
    print("\n| batch | FK f+b | +field f+b | +hinge f+b | M clearance/s | 200 iters |")
    print("|---|---|---|---|---|---|")
    for token in ns.batches.split(","):
        batch = int(token)
        qb2 = ((torch.rand(batch, spec.num_dof, device=device) - 0.5) * 1.2).requires_grad_(True)

        def fk_step():
            out = fused(qb2)
            (out * out).sum().backward()
            qb2.grad = None

        def field_step():
            clearance(qb2).sum().backward()
            qb2.grad = None

        def hinge_step():
            torch.relu(0.05 - clearance(qb2)).pow(2).sum().backward()
            qb2.grad = None

        t_fk = timed(fk_step, ns.reps)
        t_field = timed(field_step, ns.reps)
        t_hinge = timed(hinge_step, ns.reps)
        work = batch * n_spheres
        print(f"| {batch} | {t_fk * 1000:.3f} ms | {t_field * 1000:.3f} ms | {t_hinge * 1000:.3f} ms "
              f"| {work / t_hinge / 1e6:.1f} | {200 * t_hinge * 1000:.0f} ms |")

    print("\n  The increment from the middle column to the right one is the hinge:"
          "\n  `relu(margin - d)^2` as separate torch kernels, each with its own launch"
          "\n  and its own backward node. cuRobo shapes the activation INSIDE the"
          "\n  collision kernel, which is the same trick and the obvious next one.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
