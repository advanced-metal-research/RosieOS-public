#!/usr/bin/env python
"""Fused forward kinematics, ours against cuRoboV2's, on the same robot.

The point of a head to head is that both sides get the same problem. So
this hands cuRobo OUR cell -- the same `robot.urdf`, the same six driven
axes, and the same 300 authored spheres from
`robot_description/robots/rosie_1400_v3/spheres.json` -- rather than timing their
Franka against our arm and calling the ratio a result.

Both kernels answer the same question: given a batch of joint vectors,
where is every collision sphere in world coordinates, differentiably.
cuRobo's also produces link poses, quaternions and optionally jacobians in
the same launch, which is work ours does not do; that is noted rather than
subtracted, since it is not separable from their kernel.

    PYTHONPATH=<cuda-core prefix>:/path/to/curobo python tools/bench_fk_curobo.py
    python tools/bench_fk_curobo.py --curobo-root /path/to/curobo
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import zipfile
from pathlib import Path
from typing import Any

import torch

V1 = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(V1 / "python"))

CELL_ROOT = V1.parents[1] / "robot_description" / "robots"

from weld_motion_planner.robot_cell.collision import spheres as spheremod  # noqa: E402
from weld_motion_planner.robot_cell.robot import cell as cellmod  # noqa: E402
from weld_motion_planner.robot_cell.robot import fk as fkmod  # noqa: E402


def read_cell(plan: Path) -> dict[str, Any]:
    with zipfile.ZipFile(plan) as archive:
        return json.loads(archive.read("cell.json"))


def capture(fn, warmup: int = 3):
    """Record `fn` into a CUDA graph and hand back a replay, or None.

    Both sides get this or neither do. cuRobo pre-allocates its forward and
    backward buffers, which is exactly the property that makes a call
    capturable, and their planner replays graphs for the same reason ours
    does -- so timing our captured path against their eager one would be
    measuring the harness, not the kernels.

    The warmup runs on a side stream because capture refuses to start while
    the default stream has work that has not been through the allocator's
    graph-safe pool.
    """
    side = torch.cuda.Stream()
    side.wait_stream(torch.cuda.current_stream())
    with torch.cuda.stream(side):
        for _ in range(warmup):
            fn()
    torch.cuda.current_stream().wait_stream(side)
    torch.cuda.synchronize()
    graph = torch.cuda.CUDAGraph()
    try:
        with torch.cuda.graph(graph):
            fn()
    except Exception as exc:  # noqa: BLE001 - capture is best-effort
        print(f"  (could not capture: {type(exc).__name__}: {exc})")
        return None
    return graph.replay


def timed(fn, reps: int, warmup: int = 5) -> float:
    for _ in range(warmup):
        fn()
    torch.cuda.synchronize()
    start = time.perf_counter()
    for _ in range(reps):
        fn()
    torch.cuda.synchronize()
    return (time.perf_counter() - start) / reps


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--plan", default=str(V1 / "data" / "motion" / "bracket_a.weldplan"))
    parser.add_argument("--batches", default="32,128,512,2048")
    parser.add_argument("--reps", type=int, default=100)
    parser.add_argument(
        "--curobo-root",
        type=Path,
        default=None,
        help="optional cuRobo source root; otherwise it must be importable",
    )
    ns = parser.parse_args(argv)

    if not torch.cuda.is_available():
        print("no CUDA; nothing to compare")
        return 0
    device = torch.device("cuda", torch.cuda.current_device())

    cell = read_cell(Path(ns.plan))
    cell_id = str(cell.get("id"))
    model = spheremod.load(cell_id)
    if model is None:
        raise SystemExit(f"no sphere model for {cell_id}")
    covered = [n for n in cellmod.sphere_links(cell) if model.links.get(n)]

    # ---- ours ------------------------------------------------------------
    fused, spec, _radii, sphere_link, _local = fkmod.make(cell, covered, model.links, device)
    # The same kernels without graph capture, so the eager paths can be
    # compared to each other as well: cuRobo's `compute_kinematics` here is
    # eager, and a captured graph against an uncaptured one is a comparison
    # of two different things.
    eager, _s2, _r2, _l2, _lc2 = fkmod.make(cell, covered, model.links, device, graph=False)
    n_spheres = len(sphere_link)
    print(f"{cell_id}: {spec.num_dof} driven axes, {n_spheres} spheres over {len(covered)} links")
    print(f"  axes: {', '.join(spec.axis_names)}")

    # ---- theirs ----------------------------------------------------------
    if ns.curobo_root is not None:
        curobo_root = ns.curobo_root.expanduser().resolve()
        if not curobo_root.is_dir():
            raise SystemExit(f"cuRobo root does not exist: {curobo_root}")
        sys.path.insert(0, str(curobo_root))
    try:
        from curobo.kinematics import Kinematics, KinematicsCfg  # noqa: E402
        from curobo.types import JointState  # noqa: E402
    except ModuleNotFoundError as error:
        raise SystemExit(
            "cuRobo is not importable; set PYTHONPATH or pass --curobo-root"
        ) from error

    urdf = CELL_ROOT / cell_id / "robot.urdf"
    collision_spheres = {
        link: [{"center": list(centre), "radius": float(radius)} for centre, radius in model.links[link]]
        for link in covered
    }
    data = {
        "urdf_path": str(urdf),
        "asset_root_path": str(CELL_ROOT / cell_id),
        "base_link": str((cell.get("frames") or {}).get("base") or "world"),
        "tool_frames": [covered[-1]],
        "collision_link_names": list(covered),
        "collision_spheres": collision_spheres,
        # Their loader dereferences these without a default when a collision
        # model is built from a dict rather than from one of their YAMLs.
        "self_collision_buffer": dict.fromkeys(covered, 0.0),
        "self_collision_ignore": {link: [] for link in covered},
        # No `lock_joints`. The tool frame is the last sphere-bearing link,
        # so the chain their loader builds runs base -> link_6 and never
        # touches the positioner or the H-frame -- those axes are not in the
        # tree to be locked, and naming them is an error rather than a
        # no-op. Same six axes as ours, which is the point.
    }
    config = KinematicsCfg.from_data_dict({"robot_cfg": {"kinematics": data}})
    robot = Kinematics(config)
    print(f"  cuRobo loaded: {robot.get_dof()} dof, "
          f"{robot.kinematics_config.num_spheres if hasattr(robot, 'kinematics_config') else '?'} spheres")

    print()
    print("| batch | ours fwd | cuRobo fwd | ours fwd (eager) | cuRobo fwd (eager) "
          "| ours fwd+bwd | cuRobo fwd+bwd |")
    print("|---|---|---|---|---|---|---|")
    for token in ns.batches.split(","):
        batch = int(token)
        q = (torch.rand(batch, spec.num_dof, device=device) - 0.5) * 1.2

        ours_fwd = timed(lambda: fused(q), ns.reps)
        eager_fwd = timed(lambda: eager(q), ns.reps)

        qa = q.clone().requires_grad_(True)

        def ours_step():
            out = fused(qa)
            (out * out).sum().backward()
            qa.grad = None

        ours_both = timed(ours_step, ns.reps)

        qe = q.clone().requires_grad_(True)

        def eager_step():
            out = eager(qe)
            (out * out).sum().backward()
            qe.grad = None

        eager_both = timed(eager_step, ns.reps)

        # Their input has to live at a fixed address for a graph to replay
        # against it, same as ours.
        q_static = q.clone()
        js = JointState.from_position(q_static, joint_names=robot.joint_names)

        def their_fwd_call():
            robot.compute_kinematics(js)

        theirs_fwd = timed(their_fwd_call, ns.reps)
        replay = capture(their_fwd_call)

        # A graph replays against fixed addresses, so a real caller has to
        # write this iteration's joints into the static buffer first. Ours
        # pays that copy inside `forward`; theirs has to be charged it too,
        # or the comparison is of a kernel against a kernel plus a memcpy.
        def their_graph_step():
            q_static.copy_(q)
            replay()

        theirs_fwd_graph = timed(their_graph_step, ns.reps) if replay is not None else float("nan")

        qb = q.clone().requires_grad_(True)
        jsb = JointState.from_position(qb, joint_names=robot.joint_names)

        def their_step():
            state = robot.compute_kinematics(jsb)
            s = state.get_link_spheres()[..., :3]
            (s * s).sum().backward()
            qb.grad = None

        theirs_both = timed(their_step, ns.reps)

        print(
            f"| {batch} | {ours_fwd * 1000:.3f} ms | {theirs_fwd_graph * 1000:.3f} ms "
            f"| {eager_fwd * 1000:.3f} ms | {theirs_fwd * 1000:.3f} ms "
            f"| {ours_both * 1000:.3f} ms | {theirs_both * 1000:.3f} ms |"
        )

    # One correctness cross-check, so the timings are of two things that
    # agree. Different sphere ORDER between the two, so compare the sorted
    # set of world centres rather than element by element.
    q = torch.zeros(1, spec.num_dof, device=device)
    ours = fused(q)[0].detach()
    theirs = robot.compute_kinematics(
        JointState.from_position(q, joint_names=robot.joint_names)
    ).get_link_spheres().reshape(-1, 4)[:, :3].detach()
    if len(theirs) == len(ours):
        a = ours[ours[:, 0].argsort()]
        b = theirs[theirs[:, 0].argsort()]
        print(f"\n  zero-pose sphere centres agree to {float((a - b).abs().max()) * 1000:.4f} mm "
              f"over {len(ours)} spheres")
    else:
        print(f"\n  sphere counts differ: ours {len(ours)}, cuRobo {len(theirs)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
