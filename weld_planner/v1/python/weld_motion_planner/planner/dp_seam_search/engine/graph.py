#!/usr/bin/env python
"""Which nodes can follow which, and the best ways through.

`nodes.py` builds a lattice: one layer per seam sample, and in each layer
every way of welding that sample. This joins the layers and searches them.

An edge joins two nodes in adjacent layers where every joint can cover the gap
in the travel time between the two samples:

    time between samples  = seam step / travel speed
    a joint may move      = that time x its velocity limit

`threshold_from_limits` returns nothing when the cell states no rates; the caller
then states a threshold and `Graph.derived` records that it was STATED rather
than derived.

The search is one Viterbi sweep, exact ON THE DISCRETIZATION: it finds the best
path through the samples, not the best trajectory; refinement (M5) turns one into
the other, seeded from all K paths. When nothing crosses, the sweep reports the
FRONTIER -- the furthest layer any path reached -- and why every node in the next
layer was unavailable. `nodes.py` supplies the reasons a node was never feasible;
this adds `NO_EDGE`: feasible in itself, unreachable from anything that survived.
"""

from __future__ import annotations

import argparse
import itertools
import sys
from dataclasses import dataclass
from typing import Any

import numpy as np
import torch

from weld_motion_planner.planner.dp_seam_search.engine.nodes import CAUSE_NAMES, Lattice
from weld_motion_planner.robot_cell.robot_cell import RobotCell

# A node that is feasible on its own and unreachable from the previous layer.
# Numbered past `nodes.py`'s cause codes so the two spaces merge below.
NO_EDGE = 5
EDGE_CAUSE_NAMES = {**CAUSE_NAMES, NO_EDGE: "no edge"}

INFINITY = float("inf")


class GraphError(ValueError):
    """A graph this refuses to build or search, with the reason."""


# --------------------------------------------------------------------------
# The edge rule


def threshold_from_limits(profile: Any, axis_names: list[str],
                          step_s: float) -> np.ndarray | None:
    """Per-joint edge thresholds in radians, or `None` if the cell cannot say.

    `rate * step_s`, one per axis: a joint may move its velocity limit times the
    time between samples. `None` when any axis is silent about its rate, and then
    the caller states a threshold and `Graph.derived` records which happened. A
    stated 25 degrees at a 0.1 s step claims 250 deg/s on every joint, where
    `rosie_1400_v3` puts the positioner at 60.

    Takes the profile OBJECT or a raw document; `weld_motion_planner.robot_cell.robot.profile`
    normalises the key to `velocity_rad_s`, older documents wrote it two other ways.
    """
    limits = (profile.limits if hasattr(profile, "limits")
              else (profile or {}).get("limits") or {})
    rates = []
    for name in axis_names:
        spec = limits.get(name) or {}
        rate = (spec.get("velocity_rad_s") or spec.get("velocity")
                or spec.get("max_velocity_rad_per_s"))
        if not rate:
            return None
        rates.append(float(rate))
    if step_s <= 0:
        raise GraphError("a positive sample step is needed to turn rates into a threshold")
    return np.asarray(rates, dtype=np.float64) * float(step_s)


def bins_for(threshold_rad: float, span_rad: float = 2.0 * np.pi) -> int:
    """How many bins a span needs before neighbours are inside the threshold.

    STRICTLY inside, not equal to: the edge test is `<=` per component, so an axis
    that spends the whole budget alone leaves nothing for the arm.

    The closed form `floor(span/threshold) + 1` is followed by a loop because where
    the threshold divides the turn exactly -- 12 degrees into 360 -- the quotient
    lands either side of 30 depending on how `deg2rad` rounded.
    """
    if threshold_rad <= 0 or span_rad <= 0:
        raise GraphError("a positive threshold and span are needed to size bins")
    n = max(2, int(np.floor(span_rad / threshold_rad)) + 1)
    while span_rad / n >= threshold_rad:
        n += 1
    return n


# --------------------------------------------------------------------------
# The output


@dataclass
class Trace:
    """The sweep's own working state, kept layer by layer instead of discarded.

    Built only when `search(trace=True)` asks: it costs `L` times the sweep's
    running state, small at the shipped 15 bins and not at 128.

    Everything here indexes SLOTS, not lattice nodes -- the sweep compacts away
    infeasible nodes and sorts what is left by positioner bin, so slot 7 of layer
    3 has no relation to slot 7 of layer 4. `alive` is the per-layer translation.
    """

    # Per layer: the lattice node index of each live slot, in sweep order.
    alive: list[np.ndarray]
    # Per layer: (live, k) least total weighted joint motion reaching this slot
    # from layer 0; `inf` where nothing reaches it.
    cost: list[np.ndarray]
    # Per layer: (live, k) into the PREVIOUS layer's slots and their k. Layer 0
    # is empty; it is where the walk stops.
    back_node: list[np.ndarray]
    back_slot: list[np.ndarray]
    # Per transition: all source-destination pairs, the pairs the positioner band
    # left to look at, and the pairs that were edges. `kept / all` is what band
    # pruning bought, `edges / kept` what the threshold would have rejected.
    pairs_all: np.ndarray
    pairs_kept: np.ndarray
    edges: np.ndarray

    def layers(self) -> int:
        """How many layers the sweep wrote; short of the seam's own count at a
        frontier."""
        return len(self.cost)


@dataclass
class Graph:
    """The searched lattice: the paths, and what stopped the rest."""

    # (K, L) node index per layer for each kept path, -1 past the frontier.
    paths: np.ndarray
    # (K,) accumulated cost of each path; inf for a path that never crossed.
    costs: np.ndarray
    # (L,) how many nodes each layer could be reached at.
    reachable: np.ndarray
    frontier: int | None
    frontier_causes: dict[str, int]
    # Turn, travel and cornerness at the frontier; `None` when nothing stopped.
    frontier_demand: dict[str, Any] | None
    threshold_rad: np.ndarray
    derived: bool
    layers: int
    nodes_per_layer: int
    # Binned axes spaced further apart than the edge threshold, so the search
    # cannot move along them; exact for the POSITIONER, advisory for SPIN -- see
    # `pinned_axes`. `docs/issues/0004`: the shipped defaults spaced the positioner
    # 40 degrees against a 25 degree threshold, so its redundancy was unavailable.
    pinned_axes: dict[str, dict[str, float]]
    implied_rate_deg_per_s: list[float] | None
    # Only when `search(trace=True)` asked; nothing in the planner reads it.
    trace: Trace | None = None

    @property
    def crossed(self) -> bool:
        return self.frontier is None and bool(np.isfinite(self.costs).any())

    def joints(self, lattice: Lattice, k: int = 0) -> np.ndarray:
        """(L, len(lattice.state_axes)) along path `k`: the arm's axes then the
        positioner, which is part of the state the search moved through and not
        decoration.

        A cell with no positioner returns the arm's columns alone -- the same
        width as `cell.moved_axes`, which is what every consumer of this labels
        the columns with.
        """
        L, S, P, B = lattice.shape
        q_flat = lattice.q.reshape(L, -1, lattice.q.shape[-1])
        path = self.paths[k]
        width = q_flat.shape[-1] + (0 if lattice.positioner_axis is None else 1)
        out = np.full((len(path), width), np.nan)
        reached = np.nonzero(path >= 0)[0]
        out[reached, :q_flat.shape[-1]] = q_flat[reached, path[reached]].cpu().numpy()
        if lattice.positioner_axis is not None:
            positioner = np.broadcast_to(
                np.asarray(lattice.positioner_rad).reshape(1, 1, P, 1),
                (L, S, P, B)).reshape(L, -1)
            out[reached, -1] = positioner[reached, path[reached]]
        return out

    def unwound(self, lattice: Lattice, k: int = 0) -> np.ndarray:
        """`joints`, with the turns put back so the arm can execute it.

        Joints are stored canonically in `(-pi, pi]` and the edge rule prices the
        smallest LEGAL motion, so a two degree step through 180 costs two while the
        stored values read as 358; a controller handed those would unwind the wrist
        twice a seam. Resolved HERE, on the chosen path only: doing it in the
        lattice would make a node's value depend on the path that reached it. Only
        axes wider than a turn wind, and only inside their limits.
        """
        out = self.joints(lattice, k).copy()
        bounds = np.asarray(lattice.limits_rad, dtype=np.float64)
        winds = (bounds[:, 1] - bounds[:, 0]) >= (2.0 * np.pi - 1e-6)
        reached = np.nonzero(~np.isnan(out).any(axis=1))[0]
        for row, next_row in itertools.pairwise(reached):
            turns = np.round((out[row] - out[next_row]) / (2.0 * np.pi))
            offered = out[next_row] + turns * 2.0 * np.pi
            take = winds & (offered >= bounds[:, 0]) & (offered <= bounds[:, 1])
            out[next_row] = np.where(take, offered, out[next_row])
        return out

    def summary(self) -> dict[str, Any]:
        return {
            "crossed": self.crossed,
            "paths": int(np.isfinite(self.costs).sum()),
            "best_cost": None if not np.isfinite(self.costs).any() else round(float(self.costs.min()), 5),
            "frontier": self.frontier,
            "frontier_causes": self.frontier_causes or None,
            "frontier_demand": self.frontier_demand,
            "threshold_deg": [round(float(np.rad2deg(v)), 2) for v in self.threshold_rad],
            "threshold_derived_from_limits": self.derived,
            "thinnest_reachable_layer": int(self.reachable.min()),
            # A seam that failed is not evidence about the robot until this is empty.
            "pinned_axes": self.pinned_axes or None,
            # `threshold / step_s`, checkable against a datasheet. `None` when the
            # lattice came from a sample COUNT, which implies nothing about time.
            "implied_joint_rate_deg_per_s": self.implied_rate_deg_per_s,
        }


# --------------------------------------------------------------------------
# The sweep


# Where the chunking trade bottoms out, measured on `dog_box` seam 1 at 96 bins,
# 9731 live nodes a layer:
#
#     budget   destinations   slices   elements/slice     time     peak
#      1 MiB              1     9731           68,117   15.749s   0.15 GiB
#     16 MiB             11      885          749,287    1.635s   0.15 GiB
#     64 MiB             45      217        3,065,265    0.497s   0.19 GiB
#    256 MiB            181       54       12,329,177    0.337s   0.32 GiB
#      1 GiB            725       14       49,384,825    0.445s   0.83 GiB
#    one slice         9731        1      662,846,527    0.609s   9.34 GiB
#
# Too fine and the card idles; too coarse and the allocation costs more than the
# parallelism buys. 256 MiB is fastest AND a twenty-ninth of one slice's memory.
DEFAULT_CHUNK_BYTES = 256 << 20


def search(lattice: Lattice, *, threshold_rad: Any, k_best: int = 4,
           derived: bool = False, weights: Any = None,
           chunk_bytes: int = DEFAULT_CHUNK_BYTES, prune: bool = True,
           trace: bool = False) -> Graph:
    """Join the layers and sweep them. One batched pass per layer.

    - `threshold_rad`: per axis or a scalar, over the arm's axes FOLLOWED BY the
      positioner, which is an axis of the state and not a label on the node.
    - `weights`: scale the distances that become edge costs. Left unweighted the
      search buys a big shoulder swing to save a small wrist one.
    - `chunk_bytes`: caps the transition tensor, exactly -- see the loop.
    - `trace`: keeps the cost table for `/dp.html`; off in every planning path.
    """
    L, S, P, B = lattice.shape
    N = S * P * B
    if L < 2:
        raise GraphError("a graph needs at least two layers")
    if chunk_bytes < 1:
        raise GraphError("the transition chunk must be at least one byte")

    device = lattice.q.device
    q, bounds, winds = _state_with_positioner(lattice)
    dof = q.shape[-1]

    threshold = torch.as_tensor(
        np.broadcast_to(np.asarray(threshold_rad, dtype=np.float64), (dof,)).copy(),
        device=device, dtype=torch.float32)
    if bool((threshold <= 0).any()):
        raise GraphError("every joint's edge threshold must be positive")
    axis_weight = (torch.ones(dof, device=device, dtype=torch.float32)
                   if weights is None
                   else torch.as_tensor(np.asarray(weights, dtype=np.float64),
                                        device=device, dtype=torch.float32))

    # THE TRANSITION IS BUILT OVER FEASIBLE NODES ONLY. About 13% of slots are
    # feasible on this corpus, so a table over all `N` slots is 98% pairs where one
    # end does not exist. On `dog_box` seam 1: 0.46 GiB of table at 15 bins against
    # 0.009 GiB of live pairs, a factor of 53, holding at 58 out to 43 bins -- the
    # difference between 43 bins fitting on a 31.3 GiB card and 128. Exact, and it
    # makes the layers RAGGED, so the running state below is a list.
    feasible = lattice.feasible.reshape(L, N)
    alive = [torch.nonzero(feasible[layer], as_tuple=False).squeeze(-1)
             for layer in range(L)]
    # The positioner's own threshold, and `None` when there is no positioner --
    # `threshold[-1]` would then be J6's, a number that means nothing to a band
    # over positioner bins even though the prune is off anyway at one bin.
    alive, bins_of, starts, reach_bins = _band_prune_order(
        alive, lattice,
        None if lattice.positioner_axis is None else float(threshold[-1]), prune)
    q_live = [q[layer][alive[layer]] for layer in range(L)]

    cost = torch.zeros((len(alive[0]), k_best), device=device, dtype=torch.float32)
    cost[:, 1:] = INFINITY
    back_node: list[torch.Tensor] = [torch.empty(0, dtype=torch.long, device=device)]
    back_slot: list[torch.Tensor] = [torch.empty(0, dtype=torch.long, device=device)]
    reachable = np.zeros(L, dtype=np.int64)
    reachable[0] = len(alive[0])

    # Counted on the DEVICE and read back once at the end: a `.item()` inside the
    # chunk loop is a synchronise per chunk, and at 96 bins that is 885 of them.
    kept_cost: list[np.ndarray] = []
    kept_back: list[tuple[np.ndarray, np.ndarray]] = []
    counts: list[tuple[int, int, torch.Tensor]] = []
    if trace:
        kept_cost.append(cost.cpu().numpy().copy())
        kept_back.append((np.zeros((0, k_best), dtype=np.int64),
                          np.zeros((0, k_best), dtype=np.int64)))

    frontier: int | None = None
    if reachable[0] == 0:
        frontier = 0
    for layer in range(L - 1) if frontier is None else ():
        sources, destinations = len(alive[layer]), len(alive[layer + 1])
        if destinations == 0:
            frontier = layer + 1
            break
        # DESTINATIONS IN CHUNKS. `topk` runs per destination over all sources, so
        # slicing the destination axis is exactly equivalent and bounds the
        # transition at `sources x chunk x dof`. Unchunked at 96 bins on `dog_box`
        # seam 1 the search peaked at 9.34 GiB against the lattice build's 5.42,
        # setting the ceiling on 3% of the wall time. Sized from the sources a
        # chunk will see, not all of them: pruning cuts that to about
        # `(2*reach+1)/P`, and budgeting for the full set makes the chunks seven
        # times finer than needed -- 2.1x where sizing for the band gives 6.
        share = min(1.0, (2.0 * reach_bins + 1.0) / P) if starts is not None else 1.0
        chunk = max(1, int(chunk_bytes
                           // max(1, int(sources * share) * (4 * dof + 3 * k_best + 4) * 4)))
        # The ring laid out twice, so a band of positioner bins that wraps past the
        # end is still ONE contiguous slice. Two copies cost a few megabytes.
        if starts is not None:
            q_ring = torch.cat([q_live[layer], q_live[layer]], dim=0)
            cost_ring = torch.cat([cost, cost], dim=0)
        pieces_cost, pieces_index = [], []
        looked = 0
        admitted = (torch.zeros((), device=device, dtype=torch.long) if trace
                    else None)
        for start in range(0, destinations, chunk):
            stop = min(start + chunk, destinations)
            if starts is None:
                offset, src, src_cost = 0, q_live[layer], cost
            else:
                lo = int(bins_of[layer + 1][start]) - reach_bins
                hi = int(bins_of[layer + 1][stop - 1]) + reach_bins
                if hi - lo + 1 >= P:
                    offset, src, src_cost = 0, q_live[layer], cost
                else:
                    # A SLICE, not a gather: index tensors per chunk cost more in
                    # launches than the pruning saved, 1.7x where slicing gives 6.
                    # `lo` may be negative and `hi` past the end; both mean the
                    # band wraps, and after normalising a wrap is exactly
                    # `hi_bin < lo_bin`, so the slice ends in the second copy.
                    lo_bin, hi_bin = lo % P, hi % P
                    offset = int(starts[layer][lo_bin])
                    end = int(starts[layer][hi_bin + 1]) + (
                        sources if hi_bin < lo_bin else 0)
                    src, src_cost = q_ring[offset:end], cost_ring[offset:end]
            if src.shape[0] == 0:
                pieces_cost.append(torch.full((stop - start, k_best), INFINITY,
                                              device=device))
                pieces_index.append(torch.zeros((stop - start, k_best),
                                                dtype=torch.long, device=device))
                continue
            is_edge, motion_cost = _edge_costs(
                src, q_live[layer + 1][start:stop],
                threshold=threshold, axis_weight=axis_weight,
                bounds=bounds, winds=winds)
            if trace:
                looked += src.shape[0] * (stop - start)
                admitted += is_edge.sum()
            # Candidates into each next-layer node: every (from-node, slot) pair.
            candidate = torch.where(is_edge.unsqueeze(-1),
                                    motion_cost.unsqueeze(-1) + src_cost.unsqueeze(1),
                                    torch.full((1,), INFINITY, device=device))
            candidates = candidate.permute(1, 0, 2).reshape(
                stop - start, src.shape[0] * k_best)
            best, index = torch.topk(candidates, k=min(k_best, candidates.shape[1]),
                                     dim=1, largest=False)
            if offset:
                # `index` counts within the slice, which may sit in the ring's
                # second copy; back-pointers must name the layer's own nodes.
                index = (((index // k_best + offset) % sources) * k_best
                         + (index % k_best))
            pieces_cost.append(best)
            pieces_index.append(index)
        best = torch.cat(pieces_cost, dim=0)
        index = torch.cat(pieces_index, dim=0)

        cost = best
        back_node.append(index // k_best)
        back_slot.append(index % k_best)
        if trace:
            kept_cost.append(cost.cpu().numpy().copy())
            kept_back.append((back_node[-1].cpu().numpy().copy(),
                              back_slot[-1].cpu().numpy().copy()))
            counts.append((sources * destinations, looked, admitted))
        live = torch.isfinite(cost).any(dim=1)
        reachable[layer + 1] = int(live.sum())
        if not bool(live.any()):
            frontier = layer + 1
            break

    causes: dict[str, int] = {}
    demand: dict[str, Any] | None = None
    if frontier is not None:
        causes = _frontier_causes(lattice, frontier)
        demand = seam_demand(lattice, frontier)

    end = (frontier - 1) if frontier is not None else (L - 1)
    paths, costs = _best_paths(cost, back_node, back_slot, alive, end, k_best)

    table: Trace | None = None
    if trace:
        # `alive` is trimmed to the layers the sweep reached, so the lists are
        # the same length whether or not there was a frontier.
        depth = len(kept_cost)
        table = Trace(
            alive=[a.cpu().numpy().copy() for a in alive[:depth]],
            cost=kept_cost,
            back_node=[b[0] for b in kept_back],
            back_slot=[b[1] for b in kept_back],
            pairs_all=np.asarray([c[0] for c in counts], dtype=np.int64),
            pairs_kept=np.asarray([c[1] for c in counts], dtype=np.int64),
            edges=np.asarray([int(c[2]) for c in counts], dtype=np.int64))

    return Graph(paths=paths, costs=costs, reachable=reachable, frontier=frontier,
                 frontier_causes=causes, frontier_demand=demand, trace=table,
                 threshold_rad=threshold.cpu().numpy(),
                 derived=derived, layers=L, nodes_per_layer=N,
                 pinned_axes=pinned_axes(lattice, threshold.cpu().numpy()),
                 implied_rate_deg_per_s=(
                     None if not lattice.step_s else
                     [round(float(np.rad2deg(v) / lattice.step_s), 1)
                      for v in threshold.cpu().numpy()]))


def _state_with_positioner(lattice: Lattice
                           ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    """The state the search moves through: `(q, bounds, winds)`.

    `q` is `(L, N, len(lattice.state_axes))` -- the arm's joints WITH the
    positioner appended. THE POSITIONER IS PART OF THE STATE: `lattice.q` holds
    the arm alone, and searching on that treats two nodes with the same posture
    and different positioner angles as one place -- it would whip the workpiece
    180 degrees between samples 1.27 mm apart for free. The symptom was six
    K-best paths of identical cost whose postures differed by 0.0 degrees.

    A cell with no positioner appends nothing, and there is no degenerate column
    to append: the grid has one bin because the work has one pose, so a constant
    column would cost every edge a zero and every allocation a seventh of its
    width to say so.

    `bounds` is `(len(state_axes), 2)` position limits in the same order, and
    `winds` marks the axes wider than a whole turn -- the ones `_edge_costs` must
    measure by the smallest LEGAL motion rather than the plain difference.
    """
    L, S, P, B = lattice.shape
    N = S * P * B
    device = lattice.q.device
    arm_q = lattice.q.reshape(L, N, -1).to(torch.float32)
    if lattice.positioner_axis is None:
        q = arm_q
    else:
        positioner_column = torch.as_tensor(lattice.positioner_rad, device=device,
                                            dtype=torch.float32)
        positioner_column = positioner_column.reshape(1, 1, P, 1).expand(L, S, P, B
                                                                         ).reshape(L, N, 1)
        q = torch.cat([arm_q, positioner_column], dim=-1)

    bounds = torch.as_tensor(np.asarray(lattice.limits_rad, dtype=np.float64),
                             device=device, dtype=torch.float32)
    if bounds.shape[0] != q.shape[-1]:
        raise GraphError(f"the lattice carries {bounds.shape[0]} limits for "
                         f"{q.shape[-1]} state axes; they are the arm's followed "
                         "by the positioner")
    winds = (bounds[:, 1] - bounds[:, 0]) >= (2.0 * np.pi - 1e-6)
    return q, bounds, winds


def _band_prune_order(alive: list[torch.Tensor], lattice: Lattice,
                      positioner_threshold: float | None, prune: bool):
    """Sort each layer's live nodes by positioner bin, and say how many bins an
    edge can reach: `(alive, bins_of, starts, reach_bins)`.

    PAIRS THE POSITIONER ALONE RULES OUT ARE NEVER BUILT. Its value is fixed by
    the node's BIN, so `|bin_i - bin_j| * gap > threshold` is a guaranteed
    non-edge decided by two integers -- 80% of the pairs at 15 bins and 86.7% at
    128, converging to `1 - 2*threshold/360`. Exact, not a heuristic: those are
    pairs the `<= threshold` test would reject anyway.

    SPIN and the branch cannot be pruned this way: spin reaches the state only
    through the joints it moves, so bins far apart in spin can be close in joint
    space, and the branch is an unordered label.

    The sort travels with `alive`, so downstream code still reads slot indices;
    `starts[layer][bin]` is where that bin begins in the sorted array, so a band
    of bins is a slice. `bins_of` and `starts` are `None` when pruning is off or
    the grid gives it nothing to work with.
    """
    L, _, P, B = lattice.shape
    bin_gap = float(np.min(np.diff(np.sort(np.asarray(lattice.positioner_rad))))
                    if P > 1 else np.inf)
    if positioner_threshold is None or not (
            prune and P > 1 and np.isfinite(bin_gap) and bin_gap > 0):
        return alive, None, None, P
    device = alive[0].device
    reach_bins = int(np.floor(positioner_threshold / bin_gap))
    bins_of = [((alive[layer] // B) % P) for layer in range(L)]
    order = [torch.argsort(b, stable=True) for b in bins_of]
    alive = [alive[layer][order[layer]] for layer in range(L)]
    bins_of = [bins_of[layer][order[layer]] for layer in range(L)]
    starts = [torch.searchsorted(b, torch.arange(P + 1, device=device))
              for b in bins_of]
    return alive, bins_of, starts, reach_bins


def _edge_costs(source_q: torch.Tensor, destination_q: torch.Tensor, *,
                threshold: torch.Tensor, axis_weight: torch.Tensor,
                bounds: torch.Tensor, winds: torch.Tensor
                ) -> tuple[torch.Tensor, torch.Tensor]:
    """Which source-destination pairs are edges, and what each one costs.

    `(sources, destinations)` bool and float32: an edge where EVERY axis moves
    inside its threshold, costing the weighted sum of the per-axis motion.

    The motion measured is the smallest LEGAL one. A joint wider than a turn has
    several legal values for one posture -- `J6` spans 1146 degrees here, so
    -179 is equally +181 -- and values are stored canonically in `(-pi, pi]`, so
    a plain difference reads a drift through 180 as a 358 degree step when the
    arm moves two. `turns` offers the destination the number of whole turns that
    brings it nearest the source, and the offer counts only if it is still
    inside its limits afterwards.

    The path keeps its canonical values; `Graph.unwound` converts them. Storing
    wound values would make a node depend on the path that reached it, the
    ordering dependence `docs/issues/0002` refuses.
    """
    raw = source_q.unsqueeze(1) - destination_q.unsqueeze(0)
    if bool(winds.any()):
        turns = torch.round(raw / (2.0 * np.pi))
        offered = destination_q.unsqueeze(0) + turns * (2.0 * np.pi)
        legal = winds & (offered >= bounds[:, 0]) & (offered <= bounds[:, 1])
        motion = torch.where(legal, (raw - turns * (2.0 * np.pi)).abs(), raw.abs())
    else:
        motion = raw.abs()
    return (motion <= threshold).all(dim=-1), (motion * axis_weight).sum(dim=-1)


def _best_paths(cost: torch.Tensor, back_node: list, back_slot: list,
                alive: list, end: int, k_best: int
                ) -> tuple[np.ndarray, np.ndarray]:
    """The K cheapest ways to the last reached layer, walked back to layer 0.

    Nodes are visited cheapest-best-slot first, and every finite slot of a node
    is taken before the next node -- so two of the K may end on the same node by
    different histories, which is what K-best means here.
    """
    paths = np.full((k_best, len(alive)), -1, dtype=np.int64)
    costs = np.full(k_best, INFINITY)
    kept = 0
    if len(cost):
        order = torch.argsort(cost[:, 0])
        for node in order.tolist():
            for slot in range(k_best):
                if kept >= k_best or not np.isfinite(float(cost[node, slot])):
                    continue
                costs[kept] = float(cost[node, slot])
                paths[kept] = _backtrack(back_node, back_slot, alive, end, node, slot)
                kept += 1
            if kept >= k_best:
                break
    return paths, costs


def _backtrack(back_node, back_slot, alive, end: int, node: int, slot: int) -> np.ndarray:
    """Walk the back-pointers, translating COMPACT indices back to slots.

    `node` counts within `alive[layer]`; the returned index is into the layer's full
    `spin x positioner x branch` grid, which is what everything downstream uses.
    """
    L = len(alive)
    out = np.full(L, -1, dtype=np.int64)
    out[end] = int(alive[end][node])
    for layer in range(end, 0, -1):
        previous = int(back_node[layer][node, slot])
        if previous < 0:
            break
        slot = int(back_slot[layer][node, slot])
        node = previous
        out[layer - 1] = int(alive[layer - 1][node])
    return out


# --------------------------------------------------------------------------
# The diagnosis, when nothing crosses


def pinned_axes(lattice: Lattice, threshold_rad: np.ndarray) -> dict[str, dict[str, float]]:
    """Which binned axes the search cannot move along at this threshold.

    An edge needs EVERY component of the state to move less than its threshold, so
    an axis whose adjacent bins sit further apart than that can never be crossed: a
    search over its bins is a search over the one chosen at layer 0.

    **Exact for the positioner, ADVISORY for spin.** The positioner is a state
    component with its own threshold, so wider-spaced bins genuinely cannot be
    joined: on `dog_box` seam 1 the chosen path uses ONE positioner bin at 60 degree
    spacing and SIX at 24.

    Spin reaches the state only through inverse kinematics, so `x` degrees of spin
    is not `x` degrees of any joint -- the electrode sits exactly 45 degrees off
    J6's axis, so a roll about it spreads across J4, J5 and J6. On `dog_box` seam 1
    at 45 degree spin bins, adjacent bins sit a median 51.4 degrees apart in joint
    space but a MINIMUM of 22.97, and 1.3% of pairs are inside a 25 degree
    threshold; the chosen path crosses a spin bin at layer 4 with the branch
    unchanged and 21.3 degrees on J6 as its largest move. So spin is reported as
    DEGRADED, not absent. `docs/issues/0004`.
    """
    out: dict[str, dict[str, float]] = {}
    arm_axes = len(lattice.axis_names)
    checks = [("spin", np.asarray(lattice.spin_rad, dtype=float),
               float(np.min(threshold_rad[:arm_axes])))]
    # Spin alone on a cell with no positioner. The `len(bins) < 2` skip below
    # would cover the single bin, but there is no `threshold_rad[arm_axes]` to
    # read on a six-axis state and the index would be the failure.
    if lattice.positioner_axis is not None:
        checks.append(
            (lattice.positioner_axis, np.asarray(lattice.positioner_rad, dtype=float),
             float(threshold_rad[arm_axes])))
    for name, bins, limit in checks:
        if len(bins) < 2:
            continue
        gap = float(np.min(np.diff(np.sort(bins))))
        if gap < limit:
            continue
        # `gap * len(bins)` and not `max - min`: both axes use `endpoint=False` over
        # a range that closes, so the span is the count times the gap. A positioner
        # that does NOT close asks for one bin more than needed, the safe direction.
        out[name] = {
            "bin_gap_deg": round(float(np.rad2deg(gap)), 3),
            "threshold_deg": round(float(np.rad2deg(limit)), 3),
            # The size the caller should build, not advice.
            "bins_needed": int(bins_for(limit, gap * len(bins))),
            "bins_given": len(bins),
        }
    return out


def seam_demand(lattice: Lattice, frontier: int) -> dict[str, Any] | None:
    """What the SEAM asks for at the frontier, with no arm in the question.

    Node causes point at the robot -- move the part, widen the positioner. This
    points at the PROGRAM: where the weld line turns a corner the torch swings
    further than the tip travels, and no arm reaches that at the authored speed.
    Split the seam at the corner or slow through it.
    """
    if lattice.electrode_turn_rad is None or frontier < 1:
        return None
    turn = lattice.electrode_turn_rad
    here = float(turn[frontier - 1])
    step_m = 0.0
    if lattice.weld_pos_m is not None and frontier < lattice.weld_pos_m.shape[0]:
        a = lattice.weld_pos_m[frontier - 1, 0]
        b = lattice.weld_pos_m[frontier, 0]
        step_m = float((b - a).norm())
    typical = float(turn.median())
    return {
        "electrode_turn_deg": round(float(np.rad2deg(here)), 2),
        "typical_turn_deg": round(float(np.rad2deg(typical)), 2),
        "tip_travel_mm": round(step_m * 1000.0, 3),
        # Relative, not absolute: a genuinely curved seam turns a little at every
        # sample and that is not a corner.
        "is_a_corner": bool(here > max(np.deg2rad(15.0), 8.0 * typical)),
    }


def _frontier_causes(lattice: Lattice, frontier: int) -> dict[str, int]:
    """Why every node of the first unreachable layer was unavailable.

    A node that never passed screening carries its own reason from `nodes.py`; one
    that did and still could not be entered gets `NO_EDGE`.
    """
    layer = lattice.cause[frontier].reshape(-1)
    counts = {name: int((layer == code).sum()) for code, name in CAUSE_NAMES.items()}
    counts.pop("feasible", None)
    counts[EDGE_CAUSE_NAMES[NO_EDGE]] = int((layer == 0).sum())
    return {k: v for k, v in counts.items() if v}


# --------------------------------------------------------------------------
# Fitting on the card


def transition_bytes(live_nodes: int, dof: int, k_best: int) -> int:
    """What one layer's transition will allocate, before it is allocated.

    `live_nodes` is the number of FEASIBLE nodes in the fatter of the two layers,
    not the number of slots. Predicted rather than caught as an out-of-memory, so a
    rung that cannot run is REPORTED as not run.

    The multipliers count INTERMEDIATES, not just the named tensors -- the first
    version was wrong by 3.75x. `delta` costs two copies, the threshold compare and
    the weighting one each, the wrap two more `(F, F, dof)` for the turn count and
    the legality mask, `step` about three `(N, N, k_best)`. Fitted against the
    allocator and pinned by a test, to OVERESTIMATE: wrong low attempts a rung that
    cannot run, wrong high only declines one that would have squeezed in.
    """
    return live_nodes * live_nodes * (7 * dof + 3 * k_best + 4) * 4


def escalate(build: Any, threshold_rad: Any, *, k_best: int = 2, rungs: int = 3,
             growth: int = 2, budget_bytes: int | None = None) -> dict[str, Any]:
    """Search the coarsest lattice that could work, and go finer only if it fails.

    `build(bins) -> Lattice` is called once per rung. The first rung is `bins_for`
    of the tightest threshold -- the coarsest grid at which no axis is pinned --
    and each later rung multiplies it by `growth`. Climbing stops early when:

    - **The wall did not move.** A rung whose frontier is the previous rung's layer
      means the obstacle is not the grid. `bracket_a_placed` seams 5 and 7 do this:
      15 and 30 bins both stop at layer 1, both reporting 60.0 degrees of electrode
      turn across 0.32 mm of tip travel, which is a corner.
    - **The next rung does not fit.** Reported as a rung that was not run, with the
      size it would have taken.

    Returns the ladder as well as the answer, so a seam that only crossed at the
    last rung is visible even though it passed.
    """
    if rungs < 1 or growth < 2:
        raise GraphError("a ladder needs at least one rung and a growth of at least 2")
    base = bins_for(float(np.min(np.asarray(threshold_rad, dtype=np.float64))))
    ladder: list[dict[str, Any]] = []
    best: Any = None
    lattice: Lattice | None = None
    for rung in range(rungs):
        bins = base * (growth ** rung)
        if lattice is not None and budget_bytes is not None:
            # Projected from the LAST rung's measured feasible fraction, which
            # barely moves with bin count -- 13.7% at 15 bins and 13.1% at 43 on
            # `dog_box` seam 1. The slot count is a sixty-fold overestimate.
            rate = float(lattice.feasible.sum()) / float(np.prod(lattice.shape))
            projected = int(bins * bins * lattice.branches * rate)
            want = transition_bytes(projected, len(lattice.axis_names) + 1, k_best)
            if want > budget_bytes:
                ladder.append({"bins": bins, "ran": False,
                               "projected_live_nodes": projected,
                               "would_need_gib": round(want / 2**30, 2),
                               "budget_gib": round(budget_bytes / 2**30, 2)})
                break
        lattice = build(bins)
        result = search(lattice, threshold_rad=threshold_rad, k_best=k_best)
        ladder.append({"bins": bins, "ran": True, "gap_deg": round(360.0 / bins, 2),
                       "crossed": result.crossed, "frontier": result.frontier,
                       "feasible_rate": lattice.summary()["rate"]})
        best = result
        if result.crossed:
            break
        if len(ladder) >= 2 and ladder[-2].get("frontier") == result.frontier:
            ladder[-1]["stopped"] = "the frontier did not move when the grid halved"
            break
    return {"result": best, "lattice": lattice, "ladder": ladder,
            "bins": ladder[-1]["bins"] if ladder else base,
            "crossed": bool(best is not None and best.crossed)}


# --------------------------------------------------------------------------
# CLI


def self_test() -> int:
    if not torch.cuda.is_available():
        print("graph: no CUDA; the lattice is a batched torch path")
        return 0
    import time
    from pathlib import Path

    from weld_motion_planner.geometry.seam_frame import Seam
    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.planner.dp_seam_search.engine import nodes
    from weld_motion_planner.robot_cell.collision.scene import placement_matrix

    root = Path(__file__).resolve().parents[5] / "data" / "motion"
    plan = read(root / "bracket_b_2x.weldplan")
    joints = plan.joints_by_id
    spec = plan.seams[0]
    seam = Seam(spec, joints.get(str(spec.get("weld_joint_ref"))), device=torch.device("cuda"))
    lattice = nodes.build(RobotCell.of(plan.cell), seam,
                          work_to_piece=placement_matrix(plan.placement))

    from weld_motion_planner.robot_cell.robot.profile import for_cell as profile_for_cell

    axis_names = lattice.state_axes
    step_s = lattice.step_s or 0.1
    derived = threshold_from_limits(profile_for_cell(plan.cell),
                                    axis_names, step_s)
    print(f"graph: {lattice.shape[0]} layers x "
          f"{lattice.shape[1] * lattice.shape[2] * lattice.shape[3]} nodes, "
          f"{step_s * 1000:.0f} ms a sample")
    if derived is None:
        print("  the cell states no joint velocity limits, so no threshold can be "
              "derived from physics; stating one instead and recording that")
    else:
        print("  edge rule from the cell profile: "
              + ", ".join(f"{a} {np.rad2deg(v):.1f}deg"
                          for a, v in zip(axis_names, derived)))
    threshold = derived if derived is not None else np.deg2rad(25.0)

    t0 = time.perf_counter()
    graph = search(lattice, threshold_rad=threshold, k_best=4,
                   derived=derived is not None)
    torch.cuda.synchronize()
    print(f"  swept in {(time.perf_counter() - t0) * 1000:.0f} ms")
    print(f"  {graph.summary()}")

    if not graph.crossed:
        print("graph: FAIL, this seam has a feasible node at every sample, so a "
              "path should exist at a 25 degree threshold")
        return 1

    # Re-checked from the lattice, and from `unwound` NOT `joints`: canonical values
    # may cross +-180 on a joint wider than a turn, seen on real data as a 359.85
    # degree raw step that is 0.15 degrees of actual wrist motion.
    joints_out = graph.unwound(lattice, 0)
    step_max = np.abs(np.diff(joints_out, axis=0)).max()
    print(f"  worst joint step along the best path: {np.rad2deg(step_max):.2f} deg")
    if step_max > float(np.max(graph.threshold_rad)) + 1e-5:
        print("graph: FAIL, the best path takes a step past its own threshold")
        return 1
    if np.isnan(joints_out).any():
        print("graph: FAIL, the best path has a gap in it")
        return 1

    # A threshold too tight to cross must report a frontier rather than a path.
    tight = search(lattice, threshold_rad=np.deg2rad(0.05), k_best=2)
    print(f"  at 0.05 deg: {tight.summary()['frontier']=}, "
          f"causes {tight.summary()['frontier_causes']}")
    if tight.crossed:
        print("graph: FAIL, a 0.05 degree threshold should not cross this seam")
        return 1
    if tight.frontier is None or not tight.frontier_causes:
        print("graph: FAIL, a failed sweep must report a frontier and its causes")
        return 1

    print("graph: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except GraphError as exc:
        print(f"graph: {exc}", file=sys.stderr)
        raise SystemExit(1)
