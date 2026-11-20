#!/usr/bin/env python
"""The query head-to-head that BENCHMARKS.md said was not worth standing up.

It was worth standing up. Their own tests carry the whole recipe --
`SceneCfg(mesh=[...])`, `create_scene_collision`, `CollisionBuffer
.from_shape`, `get_sphere_distance` -- and the claim that wiring it
faithfully was "a larger job than the answer is worth" survived exactly
until somebody asked why.

## What is being compared

One optimizer iteration's worth of collision evaluation, gradient included,
on the same part and the same query spheres:

- **ours**: baked exact ESDF, trilinear `sample`, penetration cost
  `relu(activation - d)`, gradient by autograd through `grid_sample`.
- **cuRoboV2 mesh path**: their PRODUCTION checker over the same triangles
  as a `Mesh` obstacle -- a fused Warp kernel that BVH-queries the mesh per
  sphere at evaluation time, no grid at all, analytic gradient written in
  the same pass and returned through their autograd Function.
- **cuRoboV2 voxel path**: the same checker over a `VoxelGrid` obstacle
  whose feature tensor is OUR exact baked field, so both table paths query
  identical data and what is measured is purely the machinery: their fused
  kernel with in-pass analytic gradient against grid_sample plus autograd.

Three contenders, two architectures: pay the mesh per query (their BVH
path), or pay it once and read a table (both grid paths). The voxel row is
the like-for-like one; the mesh row is what a CAD user gets by default.

Semantics note: `get_sphere_distance` returns an activation-shaped cost, not
a raw distance, so values are compared only for "did both see the contact",
while TIME is compared exactly -- each side does its own full production
evaluation.

Run: `pixi run -e bench python tools/bench_query_curobo.py`
"""

from __future__ import annotations

import os
import statistics
import sys
import time
from pathlib import Path

import torch

# A cuRobo checkout for the comparison; CUROBO_ROOT overrides the default.
CUROBO = Path(os.environ.get("CUROBO_ROOT") or Path.home() / "projects" / "curobo")
SHAPES = [(1, 160, 60), (64, 160, 60), (256, 160, 60), (64, 160, 500), (64, 160, 1000)]
ACTIVATION_M = 0.01
SPHERE_RADIUS_M = 0.008


def _time(fn, repeats: int = 7) -> float:
    times = []
    for _ in range(repeats):
        torch.cuda.synchronize()
        mark = time.perf_counter()
        fn()
        torch.cuda.synchronize()
        times.append(time.perf_counter() - mark)
    return statistics.median(times)


def main() -> int:
    sys.path.insert(0, str(CUROBO))
    from curobo._src.geom.collision.buffer_collision import CollisionBuffer
    from curobo._src.geom.collision.collision_scene import (
        SceneCollisionCfg,
        create_scene_collision,
    )
    from curobo._src.geom.types import Mesh as CuMesh
    from curobo._src.geom.types import SceneCfg
    from curobo._src.types.device_cfg import DeviceCfg
    from curobo._src.util.warp import init_warp
    from weld_motion_planner.geometry.esdf import _ensure_kernel, bake
    from weld_motion_planner.geometry.mesh import tessellate
    from weld_motion_planner.io.plan_request import read

    device_cfg = DeviceCfg(device=torch.device("cuda:0"))
    init_warp(quiet=True, device_cfg=device_cfg)
    _ensure_kernel()

    from curobo._src.geom.types import VoxelGrid

    voxel_mm = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    mesh = tessellate(read("data/motion/bracket_a.weldplan").step_bytes)
    field = bake(mesh, voxel_m=voxel_mm / 1000.0)

    scene = SceneCfg(
        mesh=[
            CuMesh(
                name="bracket_a",
                pose=[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                vertices=mesh.vertices.tolist(),
                faces=mesh.faces.reshape(-1).astype(int).tolist(),
            )
        ]
    )
    checker = create_scene_collision(
        SceneCollisionCfg(
            device_cfg=device_cfg,
            scene_model=scene,
            cache={"mesh": 2, "primitive": 1},
            max_distance=0.1,
            num_envs=1,
        )
    )

    # Their voxel obstacle, fed OUR field. Their layout is x-major with
    # centres at pose + (i - (n-1)/2) * voxel -- exactly ours once the pose
    # is the grid's centre -- so the tensor drops in without resampling and
    # any indexing mismatch shows up as garbage at the sanity probe.
    origin = field.origin_pos_m.detach().cpu().double()
    shape = torch.tensor(field.shape, dtype=torch.float64)
    centre = (origin + (shape - 1) / 2.0 * field.voxel_m).tolist()
    voxel_scene = SceneCfg(
        voxel=[
            VoxelGrid(
                name="bracket_a_esdf",
                pose=[*centre, 1.0, 0.0, 0.0, 0.0],
                dims=[float(n) * field.voxel_m for n in field.shape],
                voxel_size=field.voxel_m,
                feature_tensor=field.values.reshape(-1).contiguous().cuda().to(torch.float16),
                # fp16 because their voxel kernel requires it -- worth ~0.1 mm
                # of rounding on metre-scale values, noted in the writeup.
            )
        ]
    )
    voxel_checker = create_scene_collision(
        SceneCollisionCfg(
            device_cfg=device_cfg,
            scene_model=voxel_scene,
            cache={"primitive": 1},
            max_distance=0.1,
            num_envs=1,
        )
    )

    lo = field.origin_pos_m.cuda()
    hi = field.upper_pos_m.cuda()
    weight = torch.tensor([1.0], device="cuda")
    activation = torch.tensor([ACTIVATION_M], device="cuda")

    mib = field.values.numel() * field.values.element_size() / 1024**2
    print(f"# bracket_a, {len(mesh.faces)} triangles, {voxel_mm:.1f} mm field ({mib:.0f} MiB)")
    print("# curobo-mesh = BVH per query; curobo-voxel = their checker over OUR exact field\n")
    print("| seeds x waypoints x spheres | points | ours-autograd | ours-fused | curobo-voxel | curobo-mesh | voxel/fused |")
    print("|---|---|---|---|---|---|---|")

    for batch, horizon, spheres in SHAPES:
        torch.manual_seed(1)
        centres = lo + torch.rand(batch, horizon, spheres, 3, device="cuda") * (hi - lo)

        ours_pts = centres.clone().requires_grad_(True)

        def ours(q=ours_pts):
            d = field.sample(q)
            torch.nn.functional.relu(ACTIVATION_M + SPHERE_RADIUS_M - d).sum().backward()

        fused_pts = centres.clone().requires_grad_(True)

        def ours_fused(q=fused_pts):
            d = field.sample_fused(q)
            torch.nn.functional.relu(ACTIVATION_M + SPHERE_RADIUS_M - d).sum().backward()

        query = torch.cat(
            [centres, torch.full((batch, horizon, spheres, 1), SPHERE_RADIUS_M, device="cuda")],
            dim=-1,
        )
        buffer = CollisionBuffer.from_shape(query.shape, device_cfg)
        theirs_pts = query.clone().requires_grad_(True)

        def theirs(q=theirs_pts, buf=buffer):
            cost = checker.get_sphere_distance_raw(q, buf, weight, activation)
            cost.sum().backward()

        voxel_buffer = CollisionBuffer.from_shape(query.shape, device_cfg)
        voxel_pts = query.clone().requires_grad_(True)

        def theirs_voxel(q=voxel_pts, buf=voxel_buffer):
            cost = voxel_checker.get_sphere_distance_raw(q, buf, weight, activation)
            cost.sum().backward()

        ours_s = _time(ours)
        fused_s = _time(ours_fused)
        theirs_s = _time(theirs)
        voxel_s = _time(theirs_voxel)
        count = batch * horizon * spheres
        print(
            f"| {batch}x{horizon}x{spheres} | {count / 1e6:.2f}M | {ours_s * 1000:.2f} ms "
            f"| {fused_s * 1000:.2f} ms | {voxel_s * 1000:.2f} ms | {theirs_s * 1000:.2f} ms "
            f"| {voxel_s / fused_s:.2f}x |"
        )

    # Sanity: both must actually see a contact a probe on the surface makes.
    on_surface = mesh.vertices[len(mesh.vertices) // 2]
    probe = torch.tensor(on_surface, dtype=torch.float32, device="cuda").reshape(1, 1, 1, 3)
    ours_d = float(field.sample(probe))
    q = torch.cat(
        [probe, torch.full((1, 1, 1, 1), SPHERE_RADIUS_M, device="cuda")], dim=-1
    )
    buf = CollisionBuffer.from_shape(q.shape, device_cfg)
    theirs_c = float(checker.get_sphere_distance_raw(q, buf, weight, activation).sum())
    buf2 = CollisionBuffer.from_shape(q.shape, device_cfg)
    voxel_c = float(voxel_checker.get_sphere_distance_raw(q, buf2, weight, activation).sum())
    ok = abs(ours_d) < 0.002 and theirs_c > 0 and voxel_c > 0
    print(f"\n# sanity at a surface vertex: ours d={ours_d * 1000:.2f} mm (~0), "
          f"mesh cost={theirs_c:.4f}, voxel cost={voxel_c:.4f} (both >0 = contact seen): "
          f"{'ok' if ok else 'MISMATCH'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
