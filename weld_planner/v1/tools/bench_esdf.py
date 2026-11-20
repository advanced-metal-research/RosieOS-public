#!/usr/bin/env python
"""Is the exact field as fast as cuRoboV2's, and how much more accurate?

`esdf.py` departs from the plan and from cuRoboV2: instead of seeding a
narrow band and propagating a far field, every voxel gets a true
closest-point query against a GPU BVH. The claim was that on part-sized
grids this is affordable. This measures it.

## What is compared, and what is not

The comparable thing is CONSTRUCTION: given a grid and a surface, how long
until every voxel holds a distance, and how wrong is it.

- **ours** -- one `wp.mesh_query_point_sign_winding_number` per voxel.
  Exact by construction, no seeding, no propagation.
- **cuRoboV2** -- `JumpFloodingEDT`, the real class out of the real
  repository, over a seed grid built by voxelizing the same mesh. This is
  their `integrator_esdf` path with the depth fusion removed, which is the
  only way to run it on CAD rather than on a camera.

What is NOT compared: their mapper end to end. It ingests depth images,
maintains a block-sparse TSDF and updates incrementally, and none of that
has an analogue here -- our scene is known CAD, bounded, and baked once. A
number for it would be a number about a different problem.

The accuracy column is the point of the exercise as much as the timing.
JFA propagates from voxel CENTRES, so its answer is quantized to the grid
before it starts; ours measures to the triangle.

Run: `pixi run -e bench motion-bench`
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch

# A cuRobo checkout for the comparison; CUROBO_ROOT overrides the default.
CUROBO = Path(os.environ.get("CUROBO_ROOT") or Path.home() / "projects" / "curobo")


def _time(fn, repeats: int = 3) -> tuple[float, Any]:
    """Median wall time over `repeats`, with the GPU actually finished.

    Median rather than mean because the first run of anything on a GPU pays
    for kernel load and allocator growth, and a mean lets that leak into
    every number. `synchronize` because a CUDA launch returns immediately
    and timing without it measures the Python, not the work.
    """
    times = []
    out = None
    for _ in range(repeats):
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        mark = time.perf_counter()
        out = fn()
        if torch.cuda.is_available():
            torch.cuda.synchronize()
        times.append(time.perf_counter() - mark)
    return statistics.median(times), out


def _voxelize(mesh, lo, counts, voxel) -> torch.Tensor:
    """Surface voxels as JFA seed sites, the way their integrator seeds.

    A site of -1 means "no site yet"; a voxel containing surface holds its
    own coordinates PACKED as 3x10 bits -- `| 2 unused | 10 z | 10 y | 10 x |`,
    their `pack_site_coords`. Not a flat index: the first version of this
    harness seeded flat indices, JFA read them as packed coordinates, and the
    result was a field whose distances ran to 329 mm on a 166 mm part. The
    numbers looked like a benchmark and were an encoding bug.

    Building the seed grid is part of THEIR cost, so it is timed with them.
    """
    import warp as wp

    device = torch.device("cuda")
    total = int(np.prod(counts))
    sites = torch.full((total,), -1, dtype=torch.int32, device=device)

    # Seed from the triangles themselves rather than from a solid fill: the
    # EDT wants the surface, and this is the cheapest honest way to get it.
    points = torch.as_tensor(mesh.vertices, dtype=torch.float32, device=device)
    faces = torch.as_tensor(mesh.faces.astype(np.int64), device=device)
    a, b, c = points[faces[:, 0]], points[faces[:, 1]], points[faces[:, 2]]
    # Subdivide until no edge is longer than a few voxels, then sprinkle.
    # This harness has now under-seeded JFA twice, and both times the error
    # column blamed their algorithm for this file's laziness: first a fixed
    # 16 samples per triangle (4.3 mm of "JFA error" on dog_box's plates),
    # then an area-proportional count capped at 4096 (a "resolution-
    # independent 2-3 mm floor" that was really the cap binding at fine
    # voxels -- with dense seeding JFA's near-band max is ~1 voxel and DOES
    # shrink with resolution). Subdivision makes coverage a geometric
    # property instead of a sampling budget, which is the only version that
    # cannot quietly regress a third time.
    for _ in range(24):
        ab = torch.linalg.norm(a - b, dim=-1)
        bc = torch.linalg.norm(b - c, dim=-1)
        ca = torch.linalg.norm(c - a, dim=-1)
        edge = torch.maximum(ab, torch.maximum(bc, ca))
        big = edge > 4.0 * voxel
        if not bool(big.any()):
            break
        # Rotate so the LONGEST edge is a-b before splitting it. Splitting
        # a fixed edge does not shrink a triangle whose long edge is one of
        # the other two, and the loop would spin its 24 rounds and leave
        # slivers unseeded.
        sa, sb, sc = a[big], b[big], c[big]
        b_longest = (bc[big] >= ab[big]) & (bc[big] >= ca[big])
        c_longest = (ca[big] >= ab[big]) & (ca[big] > bc[big])
        ra = torch.where(c_longest.unsqueeze(-1), sc, torch.where(b_longest.unsqueeze(-1), sb, sa))
        rb = torch.where(c_longest.unsqueeze(-1), sa, torch.where(b_longest.unsqueeze(-1), sc, sb))
        rc = torch.where(c_longest.unsqueeze(-1), sb, torch.where(b_longest.unsqueeze(-1), sa, sc))
        mid = 0.5 * (ra + rb)
        a = torch.cat([a[~big], ra, mid])
        b = torch.cat([b[~big], mid, rb])
        c = torch.cat([c[~big], rc, rc])
    n = 32
    u = torch.rand(len(a), n, 1, device=device)
    v = torch.rand(len(a), n, 1, device=device) * (1 - u)
    samples = (a[:, None] + u * (b - a)[:, None] + v * (c - a)[:, None]).reshape(-1, 3)
    samples = torch.cat([samples, a, b, c])

    origin = torch.as_tensor(lo, dtype=torch.float32, device=device)
    index = torch.round((samples - origin) / voxel).long()
    inside = ((index >= 0) & (index < torch.as_tensor(counts, device=device))).all(dim=-1)
    index = index[inside]
    flat = (index[:, 0] * counts[1] + index[:, 1]) * counts[2] + index[:, 2]
    packed = (index[:, 0] | (index[:, 1] << 10) | (index[:, 2] << 20)).to(torch.int32)
    sites[flat] = packed
    wp.synchronize()
    return sites


def bench_part(step_bytes: bytes, label: str, voxel_mm: float, margin_m: float = 0.05) -> dict[str, Any]:
    from weld_motion_planner.geometry.esdf import _ensure_kernel, bake
    from weld_motion_planner.geometry.mesh import tessellate

    _ensure_kernel()
    mesh = tessellate(step_bytes)
    voxel = voxel_mm / 1000.0

    ours_s, field = _time(lambda: bake(mesh, voxel_m=voxel, margin_m=margin_m))
    counts = np.array(field.shape)
    lo = field.origin_pos_m.cpu().numpy()
    row: dict[str, Any] = {
        "part": label,
        "triangles": len(mesh.faces),
        "voxel_mm": voxel_mm,
        "margin_mm": round(margin_m * 1000.0, 1),
        "grid": [int(v) for v in counts],
        "voxels_m": round(float(np.prod(counts)) / 1e6, 2),
        "ours_bake_s": round(ours_s, 4),
        "ours_mvoxel_per_s": round(float(np.prod(counts)) / 1e6 / ours_s, 1),
    }

    # -- theirs ----------------------------------------------------------
    if not (CUROBO / "curobo").is_dir() or not torch.cuda.is_available():
        row["curobo"] = "not available"
        return row
    sys.path.insert(0, str(CUROBO))
    try:
        from curobo._src.perception.mapper.esdf.edt_jump_flooding import JumpFloodingEDT
    except Exception as exc:  # noqa: BLE001 - reported, not raised: this is a benchmark.
        row["curobo"] = f"import failed: {exc}"
        return row

    device = torch.device("cuda")
    seed_s, sites = _time(lambda: _voxelize(mesh, lo, counts, voxel))
    edt = JumpFloodingEDT(tuple(int(v) for v in counts), voxel, device)

    def run_jfa():
        buffer = sites.clone()
        edt.propagate(buffer)
        return buffer

    jfa_s, propagated = _time(run_jfa)
    row["curobo_seed_s"] = round(seed_s, 4)
    row["curobo_jfa_s"] = round(jfa_s, 4)
    row["curobo_total_s"] = round(seed_s + jfa_s, 4)
    row["curobo_mvoxel_per_s"] = round(float(np.prod(counts)) / 1e6 / (seed_s + jfa_s), 1)
    row["speedup_ours"] = round((seed_s + jfa_s) / ours_s, 2)

    # -- accuracy --------------------------------------------------------
    # JFA gives the nearest SITE, so distance is centre-to-centre. Compared
    # against our exact field on the same voxels, unsigned, since JFA has no
    # sign of its own.
    packed = propagated.view(-1).long()
    valid = packed >= 0
    ijk = torch.stack([packed & 1023, (packed >> 10) & 1023, (packed >> 20) & 1023], dim=-1)
    grid = torch.stack(
        torch.meshgrid(
            *[torch.arange(int(n), device=device) for n in counts], indexing="ij"
        ),
        dim=-1,
    ).reshape(-1, 3)
    theirs = torch.linalg.norm((grid - ijk).float(), dim=-1) * voxel
    ours = field.values.reshape(-1).abs().to(device)
    error = (theirs[valid] - ours[valid]).abs()

    def p99(t: torch.Tensor) -> float:
        # A RANDOM subsample. `t[:1M]` was the previous version, and in a
        # grid-ordered tensor that is a spatial slab of low-x voxels, not a
        # sample -- a percentile over one end of the part.
        pick = torch.randperm(len(t), device=t.device)[: min(len(t), 1_000_000)]
        return float(torch.quantile(t[pick], 0.99))

    row["curobo_vs_exact_mean_mm"] = round(float(error.mean()) * 1000.0, 3)
    row["curobo_vs_exact_p99_mm"] = round(p99(error) * 1000.0, 3)
    row["curobo_vs_exact_max_mm"] = round(float(error.max()) * 1000.0, 3)

    # The NEAR band is the only one that decides anything. A torch lives
    # within a centimetre of the part; error at 200 mm is a number about a
    # region nothing ever visits, and averaging over the whole grid is
    # dominated by exactly that region. `WEAVE-AND-VERIFICATION.md` §5 is
    # about millimetres of clearance, so this is the column to read.
    near = ours[valid] < 0.010
    if bool(near.any()):
        band = error[near]
        row["near_band_voxels"] = int(near.sum())
        row["near_mean_mm"] = round(float(band.mean()) * 1000.0, 3)
        row["near_p99_mm"] = round(p99(band) * 1000.0, 3)
        row["near_max_mm"] = round(float(band.max()) * 1000.0, 3)
    row["field_mib"] = round(field.values.numel() * 4 / 1024**2, 1)

    return row


def bench_queries(field, shapes: list[tuple[int, int, int]]) -> list[dict[str, Any]]:
    """Query throughput in the shape trajectory optimization actually uses.

    `[seeds, waypoints, spheres]`, which is cuRoboV2's `[batch, horizon,
    num_spheres]` under another name. Forward alone is not the number that
    matters: the optimizer needs dcost/dq, so every query is a forward AND a
    backward, and grid_sample's backward scatters through atomics where its
    forward only gathers. Measuring forward alone would flatter the field by
    whatever that costs.
    """
    device = field.values.device
    span = torch.as_tensor(
        (np.array(field.shape) - 1) * field.voxel_m, dtype=torch.float32, device=device
    )
    origin = field.origin_pos_m.to(device)

    rows = []
    for seeds, waypoints, spheres in shapes:
        count = seeds * waypoints * spheres
        points = torch.rand(seeds, waypoints, spheres, 3, device=device) * span + origin

        forward_s, _ = _time(lambda p=points: field.sample(p), repeats=5)

        with_grad = points.clone().requires_grad_(True)

        def round_trip(q=with_grad):
            # Bound as a default argument, not captured: a closure over the
            # loop variable would time whatever the LAST shape happened to
            # be, for every row, and the table would look flat for a reason
            # that has nothing to do with the GPU.
            out = field.sample(q)
            # `relu(-d)` is the shape the collision term actually takes: only
            # penetration pushes, and the gradient is zero in free space.
            torch.nn.functional.relu(-out).sum().backward()

        both_s, _ = _time(round_trip, repeats=5)
        rows.append(
            {
                "shape": f"{seeds}x{waypoints}x{spheres}",
                "points": count,
                "forward_ms": round(forward_s * 1000.0, 3),
                "forward_grad_ms": round(both_s * 1000.0, 3),
                "forward_gpts_per_s": round(count / forward_s / 1e9, 2),
                "with_grad_gpts_per_s": round(count / both_s / 1e9, 2),
                "iters_per_s": round(1.0 / both_s, 1),
            }
        )
    return rows


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--voxel-mm", type=float, nargs="*", default=[3.0, 1.5, 1.0])
    parser.add_argument("--parts", nargs="*", default=["bracket_a", "bracket_b", "dog_box"])
    parser.add_argument(
        "--margin-mm",
        type=float,
        default=50.0,
        help="padding past the part's bounding box; large values emulate a cell-scale grid",
    )
    parser.add_argument("--skip-queries", action="store_true")
    ns = parser.parse_args(argv)

    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.utils.device import probe

    here = Path(__file__).resolve().parents[1] / "data" / "motion"
    found = probe()
    print(f"# device: {found.get('name', 'cpu')} ({found.get('capability', '-')}), "
          f"torch {found['torch']} for CUDA {found['built_for_cuda']}")
    print(f"# cuRoboV2 from {CUROBO}\n")

    rows: list[dict[str, Any]] = []
    for part in ns.parts:
        path = here / f"{part}.weldplan"
        if not path.exists():
            print(f"skipped {part}: no plan request", file=sys.stderr)
            continue
        step = read(path).step_bytes
        for voxel_mm in ns.voxel_mm:
            row = bench_part(step, part, voxel_mm, margin_m=ns.margin_mm / 1000.0)
            rows.append(row)
            print(json.dumps(row))

    if ns.skip_queries:
        _print_table(rows)
        return 0

    # -- query throughput, once, on a representative field ----------------
    from weld_motion_planner.geometry.esdf import bake as bake_field
    from weld_motion_planner.geometry.mesh import tessellate as tess

    step = read(here / f"{ns.parts[0]}.weldplan").step_bytes
    field = bake_field(tess(step), voxel_m=0.0015)
    query_rows = bench_queries(
        field,
        [
            (1, 160, 60),
            (64, 160, 60),
            (256, 160, 60),
            # Sphere-count stress. 500 spheres is a whole-cell model -- arm,
            # torch, fixture and the second bay -- rather than the ~60 an arm
            # alone needs, and 1000 is past anything a fitted model should
            # reach. If the field holds at these counts, sphere budget is not
            # a constraint on how the robot is modelled.
            (64, 160, 500),
            (64, 160, 1000),
            (256, 160, 500),
            (64, 512, 500),
        ],
    )
    print("\n| seeds x waypoints x spheres | points | forward | +backward | Gpts/s | optimizer iters/s |")
    print("|---|---|---|---|---|---|")
    for q in query_rows:
        print(
            f"| {q['shape']} | {q['points'] / 1e6:.2f}M | {q['forward_ms']:.2f} ms "
            f"| {q['forward_grad_ms']:.2f} ms | {q['with_grad_gpts_per_s']:.2f} | {q['iters_per_s']:.0f} |"
        )

    _print_table(rows)
    return 0


def _print_table(rows: list[dict[str, Any]]) -> None:
    print(
        "\n| part | voxel | grid | Mvox | MiB | ours | cuRobo seed+JFA | ratio "
        "| all err mean/p99 | NEAR err mean/p99/max |"
    )
    print("|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        theirs = (
            f"{r['curobo_total_s'] * 1000:.1f} ms ({r['curobo_seed_s'] * 1000:.1f}+{r['curobo_jfa_s'] * 1000:.1f})"
            if "curobo_total_s" in r
            else str(r.get("curobo", "-"))
        )
        error = (
            f"{r['curobo_vs_exact_mean_mm']:.2f} / {r['curobo_vs_exact_p99_mm']:.2f} mm"
            if "curobo_vs_exact_mean_mm" in r
            else "-"
        )
        ratio = f"{r.get('speedup_ours', float('nan')):.2f}x" if "speedup_ours" in r else "-"
        near = (
            f"{r['near_mean_mm']:.2f} / {r['near_p99_mm']:.2f} / {r['near_max_mm']:.2f} mm"
            if "near_mean_mm" in r
            else "-"
        )
        print(
            f"| {r['part']} | {r['voxel_mm']} mm | {'x'.join(str(v) for v in r['grid'])} "
            f"| {r['voxels_m']} | {r.get('field_mib', 0)} | {r['ours_bake_s'] * 1000:.1f} ms "
            f"| {theirs} | {ratio} | {error} | {near} |"
        )


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
