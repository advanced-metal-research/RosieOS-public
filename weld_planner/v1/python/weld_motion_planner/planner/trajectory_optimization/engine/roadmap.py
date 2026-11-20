#!/usr/bin/env python
"""A probabilistic roadmap over the driven axes, to seed what descent cannot reach.

## Why this exists

`freespace.py` solves a link by descending from a seed, and a local method
cannot leave the homotopy class its seed is in -- it can only make that class
tidier. On `bracket_b_2x` the approach must swing the wrist about 200 degrees
from the cell's reset pose, and every straight, staged, bowed and standoff
seed folds `end_effector` into `link_3` somewhere in the middle of the flight.
Measured, `docs/issues/0009`: eight seeds, thirty-two combinations of penalty
weight and iteration budget, and self-clearance never rose above -49 mm from
-76. The optimizer was not failing to converge; it was converging inside a
wall.

So the seed has to come from somewhere global. This is that somewhere, and it
is the shape cuRobo settled on too -- `MotionPlanner._plan_single` runs
TrajOpt from a linear seed first and calls `PRMGraphPlanner.find_path` only
when that fails, then interpolates the waypoints onto the trajopt horizon as
`seed_traj`.

## What it is and, more importantly, what it is not

**It is a seed generator.** The roadmap screens at SAMPLES -- nodes, and a
handful of points along each edge -- which is exactly the thing this planner
refuses to accept as proof anywhere else. That is fine here and nowhere else:
what comes out is a starting point for an optimizer whose result is judged by
the T2 verifier, which certifies the continuous curve. A path this module calls
clear is a hypothesis. `verify.certify_segment` is the only thing in this
repository that calls anything proven.

**It screens against the arm as well as the world.** Both, because the
failure it exists to fix is the arm hitting itself and an environment-only
screen scores that as perfectly clear -- the bug that hid this for a day.

**It is deterministic.** A Sobol sequence, unscrambled, not an RNG: planning
the same plan request twice must give the same plan, and a roadmap drawn from
`random` would quietly break that. Same reason `freespace`'s seeds are fixed
curves rather than perturbations.

## The sampling box

Joint limits, intersected with a padded hull of the configurations the caller
actually needs joined. A roadmap spread over travel the plan never visits
spends its samples on nothing -- and on this cell the positioner alone would
take a third of them, since its limits span a full turn and one plan uses a
few degrees of it.

Run: `pixi run -e motion python -m weld_motion_planner.planner.trajectory_optimization.engine.roadmap <request.weldplan>`.
"""

from __future__ import annotations

import argparse
import heapq
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import torch

from weld_motion_planner.io.plan_request import PlanRequest
from weld_motion_planner.io.plan_request import read as read_request
from weld_motion_planner.robot_cell.robot_cell import RobotCell

# How many configurations to draw. Screening is one batched forward
# kinematics, so this is cheap; the cost that scales is edge validation.
SAMPLES = 4096
# Neighbours per node. Above about a dozen the extra edges are longer than the
# ones already there and buy no connectivity.
NEIGHBOURS = 12
# How finely an edge is screened: radians of the largest axis between interior
# samples -- a step SIZE, not a count, because edges here differ in length by
# an order of magnitude and a fixed count leaves gaps on the long ones.
# Measured on `bracket_a_2x` with the narrowed 2026-08-21 limits: a route
# whose every edge passed the old fixed-8 screen penetrated the positioner
# table by 8 mm between the samples, was handed to the optimizer as a seed,
# and the solve converged 41 mm inside the part. Still NOT a proof -- see the
# module docstring. Same value and reasoning as `freespace.SHORTCUT_STEP_RAD`.
EDGE_STEP_RAD = 0.05
# Nodes a start or a goal is allowed to attach to.
ATTACH = 24

# A node has to stand clear, an edge only has to not penetrate. The asymmetry
# is deliberate: the endpoints of every link ARE weld poses, millimetres off
# the work by definition, so an edge that had to satisfy the node margin could
# never reach one.
NODE_MARGIN_M = 0.02
# And the self margin is SMALLER than the environment one, which looks like a
# typo and is not. This arm's sphere model never reports more than 17.8 mm
# between the pairs it checks -- measured over 4096 samples of the box, and
# the poses the planner already certifies sit at 7.6 to 12.5 mm. So 20 mm is
# not a cautious self margin on this cell, it is an impossible one: the first
# roadmap built with it screened out all 4096 samples and refused. At 5 mm the
# yield is 32%.
NODE_SELF_MARGIN_M = 0.005

# Padding on the hull of the configurations to be joined, radians.
BOX_PAD_RAD = 0.6
# Poses per screening chunk. 1663 self-pairs times this is the tensor that
# sets the ceiling; at 8192 it is 54 MB, which fits anywhere.
CHUNK = 8192


class RoadmapError(RuntimeError):
    """A roadmap this refuses to build, with the reason."""


@dataclass
class Roadmap:
    """Configurations known to stand clear, and the moves between them."""

    # The axis order every array here is in. Carried rather than assumed: a
    # roadmap read back beside a cell whose axes differ would index silently.
    axis_names: list[str]
    nodes: np.ndarray                       # (N, len(axis_names))
    adjacency: list[list[tuple[int, float]]]
    scale: np.ndarray                       # rad per second, the metric
    screened: int = 0                       # how many samples were drawn
    edges: int = 0
    notes: list[str] = field(default_factory=list)

    def summary(self) -> dict[str, Any]:
        degree = [len(a) for a in self.adjacency]
        return {
            "sampled": self.screened,
            "nodes": len(self.nodes),
            "edges": self.edges,
            "mean_degree": round(float(np.mean(degree)), 2) if degree else 0.0,
            "isolated": int(sum(1 for d in degree if d == 0)),
        }


# --------------------------------------------------------------------------
# Screening


@torch.no_grad()
def clearances(scene, q: torch.Tensor, held: dict[str, float], names: list[str],
               chunk: int = CHUNK) -> tuple[torch.Tensor, torch.Tensor]:
    """`(env, self)` worst gap per configuration, in chunks.

    Kept as TWO numbers rather than the checker's merged one: the torch in
    the plate and the forearm in the wrist are different failures with
    different fixes (`docs/issues/0009`).

    Chunked because the self-collision tensor is the batch times the pair
    count -- 1663 pairs on this arm -- and the whole edge-validation batch at
    once is gigabytes for arithmetic that is a subtract and a norm.
    """
    check = scene.checker(names, held)
    env, mine = [], []
    for start in range(0, q.shape[0], chunk):
        block = check.q_full(q[start:start + chunk])
        env.append(scene.clearance(block).min(dim=1).values)
        own = scene.self_clearance(block)
        mine.append(own.min(dim=1).values if own.numel()
                    else torch.full_like(env[-1], float("inf")))
    return torch.cat(env), torch.cat(mine)


# --------------------------------------------------------------------------
# Building


def _box(request: PlanRequest, focus: np.ndarray,
         names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    """The sampling box: the cell's limits, narrowed to what is being joined."""
    limits = {str(j.get("name")): (j.get("limits_rad") or {})
              for j in request.cell.get("joints") or []}
    lower, upper = [], []
    for i, name in enumerate(names):
        limit = limits.get(name) or {}
        if "lower" not in limit or "upper" not in limit:
            raise RoadmapError(
                f"axis {name} has no position limit in the cell, so there is no "
                "box to sample -- see `ik.limits_of`, which refuses for the same "
                "reason")
        lo = max(float(limit["lower"]), float(focus[:, i].min()) - BOX_PAD_RAD)
        hi = min(float(limit["upper"]), float(focus[:, i].max()) + BOX_PAD_RAD)
        if hi <= lo:                                # the hull already fills it
            lo, hi = float(limit["lower"]), float(limit["upper"])
        lower.append(lo)
        upper.append(hi)
    return np.array(lower), np.array(upper)


def build(request: PlanRequest, scene, device, *, focus: np.ndarray,
          held: dict[str, float] | None = None,
          samples: int = SAMPLES, neighbours: int = NEIGHBOURS,
          edge_step_rad: float = EDGE_STEP_RAD,
          node_margin_m: float = NODE_MARGIN_M,
          node_self_margin_m: float = NODE_SELF_MARGIN_M,
          cell: Any = None) -> Roadmap:
    """Sample, screen, connect, validate. Once per plan request, not per link."""
    # Assembled once by the caller. Building the axis order and the profile here
    # is how two stages came to disagree about the same cell.
    cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
    names = list(cell.moved_axes)
    focus = np.atleast_2d(np.asarray(focus, dtype=np.float64))
    lower, upper = _box(request, focus, names)

    limits = getattr(cell.profile, "limits", None) or {}
    # The metric is TIME, not radians. A radian of the base is a slower move
    # than a radian of the wrist, and a roadmap that called them equal would
    # route through whichever axis happens to be geared low.
    scale = np.array([float((limits.get(n) or {}).get("velocity_rad_s") or 1.0)
                      for n in names])

    engine = torch.quasirandom.SobolEngine(dimension=len(names), scramble=False)
    raw = engine.draw(samples).double().numpy()
    drawn = lower + raw * (upper - lower)

    q = torch.as_tensor(drawn, dtype=torch.float64, device=device)
    env, mine = clearances(scene, q, held or {}, names)
    keep = (env > node_margin_m) & (mine > node_self_margin_m)
    nodes = drawn[keep.cpu().numpy()]
    if not len(nodes):
        raise RoadmapError(
            f"none of {samples} sampled configurations stood clear at "
            f"{node_margin_m * 1000:.0f} mm; the box is inside something")

    # k nearest in the time metric, then every candidate edge validated at
    # once. One big screen beats one per edge for the same reason everything
    # else here is batched.
    scaled = torch.as_tensor(nodes / scale, dtype=torch.float64, device=device)
    k = min(neighbours + 1, len(nodes))
    distance = torch.cdist(scaled, scaled)
    near = distance.topk(k, largest=False)
    left = torch.arange(len(nodes), device=device).unsqueeze(1).expand(-1, k).reshape(-1)
    right = near.indices.reshape(-1)
    once = left < right                       # drops self-edges and duplicates
    pairs = torch.stack([left[once], right[once]], dim=1)
    weight = near.values.reshape(-1)[once]

    valid = _validate(scene, torch.as_tensor(nodes, dtype=torch.float64, device=device),
                      pairs, names, held or {}, edge_step_rad)
    pairs = pairs[valid].cpu().numpy()
    weight = weight[valid].cpu().numpy()

    adjacency: list[list[tuple[int, float]]] = [[] for _ in nodes]
    for (a, b), w in zip(pairs, weight, strict=True):
        adjacency[int(a)].append((int(b), float(w)))
        adjacency[int(b)].append((int(a), float(w)))
    return Roadmap(axis_names=names, nodes=nodes, adjacency=adjacency, scale=scale,
                   screened=samples, edges=len(pairs))


@torch.no_grad()
def _validate(scene, nodes: torch.Tensor, pairs: torch.Tensor, names: list[str],
              held: dict[str, float], step_rad: float) -> torch.Tensor:
    """`(P,)` which candidate edges have no penetration at their interior.

    INTERIOR only. The endpoints of an edge are either roadmap nodes, already
    screened, or the link's own weld poses, which sit inside any positive
    margin by design -- so including them would reject every edge that
    reaches a weld.

    Sampled at a fixed angular STEP per edge, not a fixed count -- see
    `EDGE_STEP_RAD` for what the count cost. Edges have different lengths, so
    the samples are PACKED: `edge_of` says which edge each flat sample
    belongs to, and an edge fails if any of its samples does.
    """
    if not len(pairs):
        return torch.zeros(0, dtype=torch.bool, device=nodes.device)
    a = nodes[pairs[:, 0]]
    b = nodes[pairs[:, 1]]
    reach = (b - a).abs().max(dim=1).values
    counts = torch.clamp((reach / max(step_rad, 1e-6)).round().long(),
                         min=2, max=512)
    edge_of = torch.repeat_interleave(
        torch.arange(len(pairs), device=nodes.device), counts)
    first = torch.cat([torch.zeros(1, dtype=torch.long, device=nodes.device),
                       counts.cumsum(0)[:-1]])
    step = torch.arange(int(counts.sum()), device=nodes.device) - first[edge_of]
    t = ((step + 1).to(nodes.dtype) / (counts[edge_of] + 1).to(nodes.dtype)
         ).unsqueeze(1)
    samples = a[edge_of] * (1 - t) + b[edge_of] * t
    env, mine = clearances(scene, samples, held, names)
    ok = torch.ones(len(pairs), dtype=torch.bool, device=nodes.device)
    ok[edge_of[~((env > 0.0) & (mine > 0.0))]] = False
    return ok


# --------------------------------------------------------------------------
# Querying


@torch.no_grad()
def path(roadmap: Roadmap, scene, device, start: np.ndarray, end: np.ndarray, *,
         held: dict[str, float] | None = None, attach: int = ATTACH,
         edge_step_rad: float = EDGE_STEP_RAD) -> np.ndarray | None:
    """Waypoints from `start` to `end`, or `None` if the roadmap has no route.

    `None` is a real answer and is reported as one. A roadmap that cannot
    connect two poses has said something -- either they are in different
    components of the free space or it was sampled too thinly -- and
    inventing a straight line to hand back would erase the distinction
    between "here is a way" and "I did not find one".
    """
    nodes = roadmap.nodes
    if not len(nodes):
        return None
    held = held or {}
    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)

    tensor = torch.as_tensor(nodes, dtype=torch.float64, device=device)
    scale = torch.as_tensor(roadmap.scale, dtype=torch.float64, device=device)
    ends = torch.as_tensor(np.stack([start, end]), dtype=torch.float64, device=device)
    distance = torch.cdist(ends / scale, tensor / scale)               # (2, N)
    k = min(attach, len(nodes))
    near = distance.topk(k, largest=False)

    # Both terminals' attachment edges validated in ONE screen: they are the
    # same kind of query and there are only `2k` of them.
    stacked = torch.cat([tensor[near.indices[0]], tensor[near.indices[1]]])
    terminals = torch.cat([ends[0:1].expand(k, -1), ends[1:2].expand(k, -1)])
    both = torch.cat([stacked, terminals])
    pairs = torch.stack([torch.arange(2 * k, device=device),
                         torch.arange(2 * k, device=device) + 2 * k], dim=1)
    ok = _validate(scene, both, pairs, roadmap.axis_names, held,
                   edge_step_rad).cpu().numpy()

    links: dict[int, list[tuple[int, float]]] = {0: [], 1: []}
    for side in (0, 1):
        for j in range(k):
            if ok[side * k + j]:
                links[side].append((int(near.indices[side, j]),
                                    float(near.values[side, j])))
    if not links[0] or not links[1]:
        return None

    # Dijkstra, with `heapq` rather than a graph library. The planner's own
    # environment does not depend on `networkx` (it is declared for the
    # cuRobo benchmark feature and nothing else), and a shortest path over a
    # few thousand nodes is a heap and a loop.
    goal = {j: w for j, w in links[1]}
    best = {j: w for j, w in links[0]}
    came: dict[int, int] = {}
    queue = [(w, j) for j, w in links[0]]
    heapq.heapify(queue)
    seen: set[int] = set()
    reached = None
    total = float("inf")
    while queue:
        cost, node = heapq.heappop(queue)
        if node in seen:
            continue
        seen.add(node)
        if node in goal and cost + goal[node] < total:
            total, reached = cost + goal[node], node
        if cost >= total:
            continue
        for other, weight in roadmap.adjacency[node]:
            step = cost + weight
            if step < best.get(other, float("inf")):
                best[other] = step
                came[other] = node
                heapq.heappush(queue, (step, other))
    if reached is None:
        return None

    chain = [reached]
    while chain[-1] in came:
        chain.append(came[chain[-1]])
    chain.reverse()
    return np.vstack([start, nodes[chain], end])


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("request", type=Path)
    parser.add_argument("--samples", type=int, default=SAMPLES)
    parser.add_argument("--seams", type=int, default=3)
    args = parser.parse_args(argv)

    # Imported here and not at the top: `freespace` imports this module, and
    # the winding question is its, not the roadmap's.
    from weld_motion_planner.planner.dp_seam_search.options import SeamSearchOptions
    from weld_motion_planner.planner.planner_main import (
        MotionPlanningOptions,
        plan_motion,
    )
    from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
        rewound,
        winding_reach,
    )
    from weld_motion_planner.planner.trajectory_optimization.options import (
        WeldTrajoptOptions,
    )
    from weld_motion_planner.robot_cell.collision import model as collision_model

    request = read_request(args.request)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    trajectories = plan_motion(
        request,
        MotionPlanningOptions(run_weld_trajopt=True, run_connecting_trajopt=False,
                              seam_search=SeamSearchOptions(k_best=1),
                              weld_trajopt=WeldTrajoptOptions(verify=False)),
        device=device).weld_trajectories[: args.seams]
    cell = RobotCell.of(request.cell, device=device)
    scene = collision_model.for_planner(request, cell)
    held = dict(cell.profile.held)
    home = np.array([cell.profile.reset_pose[n] for n in cell.moved_axes])

    # Over every winding still on the table, which is the box `freespace`
    # asks its questions in.
    graph = build(request, scene, device, held=held, samples=args.samples,
                  focus=winding_reach(request, trajectories, home, cell=cell),
                  cell=cell)
    print("roadmap:", graph.summary())

    def walk(label, plan):
        stops = ([("reset", home)]
                 + [(r.seam_id + " start", r.knots[0]) for r in plan])
        print(f"  {label}")
        for (an, a), (bn, b) in zip(stops[:-1], stops[1:], strict=True):
            route = path(graph, scene, device, a, b, held=held, attach=256)
            print(f"    {an:17s} -> {bn:17s} "
                  + ("no route" if route is None else f"{len(route)} waypoints"))

    walk("as M4 wound them:", trajectories)
    walk("rewound:", rewound(request, trajectories, home, roadmap=graph, scene=scene,
                             device=device, held=held, cell=cell))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
