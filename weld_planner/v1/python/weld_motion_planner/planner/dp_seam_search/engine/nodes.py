#!/usr/bin/env python
"""The lattice the redundancy search runs on: every way to weld each point.

At a point along a seam the weld program pins five numbers -- three putting the
torch tip on `P(s)`, two aiming the electrode. On `rosie_1400_v3` welding moves
SEVEN axes: `J1..J6` and the positioner `J7`. So the feasible set there is
two-dimensional, coordinatized by **spin** (the torch about its own electrode,
which changes nothing about the weld and everything about the posture) and
**`J7`**.

Seven and not eight: `J9`, the swing carrying both tables, is a station index
this cell does not weld through, so `build` HOLDS it -- named and valued on
`Lattice.held`, because holding it with a `torch.zeros` instead produced output
indistinguishable from a screen over the whole positioner (`docs/issues/0003`).

**A cell need not have a positioner at all.** `rosie_1420_v1` is an arm on a
pedestal and the part is bolted to the floor, so the chain to its work frame is
empty and nothing turns the weld under the torch. Then the feasible set is
ONE-dimensional -- spin alone -- and this builds a lattice with a single
positioner bin: `positioner_axis` is `None`, `Lattice.state_axes` is the arm's
six, and the state the graph searches has no positioner column. The grid keeps
its four axes so that every shape, index and reshape downstream is the same
arithmetic on both cells; what changes is that `P` is 1 because there is one way
the work can sit, not because the sweep was capped.

Fix spin and `J7` and the arm is six equations in six unknowns: a FINITE set of
roots. This arm's wrist is spherical -- `J4`, `J5`, `J6` meet at one point to
within the float, measured -- which caps that at **eight** rather than a general
6R's sixteen, and its joint limits leave four to eight standing at a real weld
pose. So the feasible set at one weld point is up to eight disconnected sheets.

That is the shape of the output. The grid is `samples x spin bins x J7 bins x
branch`, and the fourth axis is a LABEL saying which sheet, not a coordinate on
one: no continuous motion crosses sheets, so a trajectory has to stay on one.
Holding the branch index fixed along the seam does not yet guarantee that
(`docs/issues/0002`).

**Every rejection carries a CAUSE, not a boolean.** The right fix depends on
which: collision means the part is placed badly, joint limit means the
positioner range is wrong, no-IK means the point is out of reach. To keep the
last two separable the solve runs WITHOUT clamping to the limits -- clamped, a
pose needing travel the arm lacks looks identical to one no posture reaches.

**`tool0`'s origin is the electrode tip.** The gun is baked into `link_6`'s mesh,
which ends there, so the tool origin goes on the weld point and no mount offset
is applied on top -- `MOUNT_NOTE`. How far off the work the planner then holds
it is the SEAM's own `process.standoff_mm`, a profile evaluated per sample the
same way the work and travel angles are -- `authored_standoff_m`.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import torch

from weld_motion_planner.geometry.seam_frame import Seam, apply_angles
from weld_motion_planner.robot_cell.robot import ik, ik_closed
from weld_motion_planner.robot_cell.robot.fk import torch_link_transform
from weld_motion_planner.robot_cell.robot.tool import ToolError
from weld_motion_planner.robot_cell.robot_cell import RobotCell
from weld_motion_planner.utils.timing import Stopwatch, cuda_sync

# Cause codes. Ordered by when the screen catches them, so a histogram reads
# left to right as "how far into the funnel did this node get".
FEASIBLE = 0
NO_IK = 1
JOINT_LIMIT = 2
COLLISION = 3
# Converged, legal and clear -- but the same posture a lower-numbered branch
# already found. Kept as its own code rather than dropped so the histogram
# stays honest about how many of the eight branches are genuinely distinct,
# which is a property of the ARM and worth watching.
DUPLICATE = 4
CAUSE_NAMES = {FEASIBLE: "feasible", NO_IK: "no ik", JOINT_LIMIT: "joint limit",
               COLLISION: "collision", DUPLICATE: "duplicate"}

# Two postures closer than this in every joint are the same node's worth of
# choice. The same measure the graph's edges use, so a "distinct" branch here
# is one the graph would also refuse to merge.
BRANCH_TOL_RAD = 0.1

# How much the collision screen may hold at once. It places every sphere of the
# arm for every node, which is 315 million points on a 64-bin lattice, so
# unsliced it is the largest allocation in the whole planner by an order of
# magnitude. See `_screen_collision`.
COLLISION_CHUNK_BYTES = 256 << 20

# Seconds of travel between samples when a caller does not say. A TIME and not a
# count: the edge rule is a rate limit, and a fixed count per seam makes one
# threshold mean thirty different things across this corpus.
#
# A FLOOR paired with the bin defaults. The cell's positioner does 60 deg/s, so a
# 250 ms step allows it 15 degrees between samples; bins spaced wider than that
# pin the axis (`docs/issues/0004`) and `graph.pinned_axes` says so. 100 ms is
# coherent at 61 bins but makes the longest seam 5.6 million nodes and 637 ms --
# affordable for a plan, not for a debug bundle, which is why `make_graph_demo`
# takes the step as an argument.
DEFAULT_STEP_S = 0.25

MOUNT_NOTE = (
    "the electrode axis comes from the cell's own tool asset "
    "(the torch section of robot_description/<model>/config.json), and the tool frame's origin IS the "
    "electrode tip -- the gun is baked into link_6's mesh, which ends at "
    "tool0 -- so the tool origin goes on the weld point itself. "
    "tooling.json's body_length_mm describes that baked gun; it is not an "
    "offset to apply on top of it"
)


class NodeError(ValueError):
    """A lattice this refuses to build, with the reason."""


# --------------------------------------------------------------------------
# The output


@dataclass
class Lattice:
    """The grid, and what screening made of it."""

    fractions: np.ndarray        # (L,) along the seam, 0..1
    spin_rad: np.ndarray         # (S,)
    # (P,) where the positioner was sampled. A cell with no positioner has ONE
    # bin whose angle is the zero of a chain with no joints -- `build` evaluates
    # it rather than special-casing it -- and `positioner_axis` is `None`, which
    # is what every reader must test rather than the length of this.
    positioner_rad: np.ndarray   # (P,)
    positioner_axis: str | None
    axis_names: list[str]
    # Every other axis on the chain to the work, and the value it was pinned at
    # while this lattice was screened. Empty is a claim, not a default: it says
    # nothing on that chain was held back -- either the positioner had exactly
    # one axis, or (`positioner_axis is None`) the chain has none at all.
    held: dict[str, float]
    # (len(state_axes), 2) position limits in the order the search's state uses
    # them -- the arm's axes, then the positioner's when the cell has one.
    # Carried because a joint
    # whose range is wider than a turn has several legal values for one posture,
    # and the edge rule has to measure the smallest legal motion rather than the
    # difference of two canonical numbers -- `graph.search`, and the wrap.
    limits_rad: np.ndarray
    # Seconds of travel between samples, or `None` when the caller gave a
    # count instead. The number the edge threshold has to be chosen WITH: a
    # threshold is a joint rate times this, so the two together imply a rate and
    # `graph` reports it. See `samples_for`.
    step_s: float | None
    # (L,) the work and travel angles this lattice was actually built at, in
    # degrees. Reported for the same reason `held` is: these were 0.0 for a
    # milestone while the seams asked for a ramp from 34.5 to 10, and nothing in
    # any output said which weld had been planned. `docs/issues/0005`.
    work_deg: np.ndarray
    travel_deg: np.ndarray
    # (L,) how far off the work the tool origin was held, in millimetres --
    # the seam's own `process.standoff_mm` profile, evaluated per sample like
    # the angles above and carried for the same reason they are.
    standoff_mm: np.ndarray
    branches: int                # IK branches kept per (sample, spin, positioner)
    q: torch.Tensor              # (L, S, P, B, dof) arm joints
    cause: torch.Tensor          # (L, S, P, B) int8, see the codes above
    position_error_m: torch.Tensor   # (L, S, P, B)
    clearance_m: torch.Tensor | None  # (L, S, P, B) when a scene screened it
    # (L-1,) how far the ELECTRODE DIRECTION turns between samples, in radians.
    # The seam and the authored angles alone, no arm -- so it separates "this
    # cell cannot reach" from "no cell could follow this at this speed".
    # `bracket_a`'s first seam turns 60 degrees while the tip travels 2.58 mm,
    # rounding a corner 2.1 mm in; the remedy is to split or slow the seam, and
    # only this number says so.
    electrode_turn_rad: torch.Tensor | None = None
    # (L, P, 3) where the weld is, in WORLD, per sample and positioner angle.
    # Spin and branch do not move it -- that is what makes them free -- so it
    # is stored without those axes. A viewer marks it, and a torch that does
    # not reach it is a kinematics bug no green cell can show.
    weld_pos_m: torch.Tensor | None = None
    # What building this cost, by stage, in milliseconds -- `setup`, `targets`,
    # `ik`, `screen`, `grid`. Carried on the lattice rather than returned beside
    # it so every caller gets it without asking; `dp_seam_search_main.py` folds
    # it into the per-seam breakdown the drawer draws. Empty on a lattice built
    # by something that did not time it.
    timings_ms: dict[str, float] = field(default_factory=dict)

    @property
    def shape(self) -> tuple[int, int, int, int]:
        return (len(self.fractions), len(self.spin_rad), len(self.positioner_rad),
                self.branches)

    @property
    def state_axes(self) -> list[str]:
        """The axes the search moves through, in the order the state uses them:
        the arm's, then the positioner when the cell has one.

        ONE place spells "arm then positioner", because four call sites spelled
        it as `[*axis_names, positioner_axis]` and every one of them would put a
        `None` in the list on a cell with no positioner -- a name that reaches
        the scene's axis map, the edge threshold and the exported waypoint
        labels. It is also exactly `cell.moved_axes`, which the graph's
        threshold is built over, so a lattice and its edge rule cannot disagree
        about how wide the state is.
        """
        return [*self.axis_names, *([] if self.positioner_axis is None
                                    else [self.positioner_axis])]

    @property
    def feasible(self) -> torch.Tensor:
        return self.cause == FEASIBLE

    def histogram(self) -> dict[str, int]:
        return {name: int((self.cause == code).sum()) for code, name in CAUSE_NAMES.items()}

    def per_layer(self) -> np.ndarray:
        """(L,) how many nodes survived at each sample. Zero is the frontier."""
        return self.feasible.sum(dim=(1, 2, 3)).cpu().numpy()

    def frontier(self) -> int | None:
        """The first sample with no feasible node, or `None` if all survive.

        The layer index and not a fraction, because what the operator is told
        is "the weld stops being possible HERE" and the sample is where the
        evidence is.
        """
        counts = self.per_layer()
        dead = np.nonzero(counts == 0)[0]
        return int(dead[0]) if len(dead) else None

    def summary(self) -> dict[str, Any]:
        total = int(np.prod(self.shape))
        counts = self.per_layer()
        return {
            "shape": {"samples": self.shape[0], "spin": self.shape[1],
                      "positioner": self.shape[2], "branches": self.shape[3]},
            "positioner_axis": self.positioner_axis,
            # Reported even when empty. A screen that swept one axis of a
            # two-axis positioner and did not say so read exactly like a screen
            # over the whole thing, which is `docs/issues/0003`.
            "held_deg": {name: round(float(np.rad2deg(v)), 3)
                         for name, v in sorted(self.held.items())},
            # Range and not the whole series: the series is `L` numbers and the
            # question a reader has is "which weld was this", which two numbers
            # answer. `work_deg` and `travel_deg` carry the series.
            "seconds_between_samples": self.step_s,
            "angles_deg": {
                "work": [round(float(self.work_deg.min()), 3),
                         round(float(self.work_deg.max()), 3)],
                "travel": [round(float(self.travel_deg.min()), 3),
                           round(float(self.travel_deg.max()), 3)],
            },
            "standoff_mm": [round(float(self.standoff_mm.min()), 3),
                            round(float(self.standoff_mm.max()), 3)],
            "nodes": total,
            "feasible": int(self.feasible.sum()),
            "rate": round(float(self.feasible.sum()) / total, 4),
            "causes": self.histogram(),
            "layers_with_nothing": int((counts == 0).sum()),
            "frontier_sample": self.frontier(),
            "thinnest_layer": int(counts.min()),
            "worst_electrode_turn_deg": (
                None if self.electrode_turn_rad is None
                else round(float(torch.rad2deg(self.electrode_turn_rad.max())), 2)),
        }


# --------------------------------------------------------------------------
# The build: sample, bin, place, pose, solve, screen


def build(cell: RobotCell, seam: Seam, *,
          samples: int | None = None, step_s: float = DEFAULT_STEP_S,
          spin_bins: int = 25, positioner_bins: int = 90,
          positioner_axis: str | None = None,
          positioner_span_rad: float = np.pi,
          held: dict[str, float] | None = None,
          tool_axis: Any = None,
          work_to_piece: np.ndarray | None = None,
          scene: Any = None, clearance_floor_m: float = 0.0,
          analytic: bool = True,
          spin_offset_bins: float = 0.0,
          positioner_offset_bins: float = 0.0,
          device=None, dtype=torch.float32) -> Lattice:
    """Enumerate and screen. One batched solve for the whole grid.

    ONE inverse-kinematics call, not a loop over samples: a solve at batch 256
    and at batch 2048 cost the same wall time here (`BENCHMARKS.md` §9), so a
    64x16x16 lattice is 16384 poses and one batch.

    `dtype` is **float32 for the screen**. This card runs float64 at a
    sixty-fourth of its float32 rate -- 2.25 ms against 90.54 for a 4096-square
    multiply -- and the screen is 91% of the build, so moving it buys 5.4x
    (3.12 s to 0.58 s on a 64-bin lattice) and costs 0.05% of nodes changing
    class at the tolerance boundary. float32 resolves 0.24 um at this cell's
    two-metre scale against a 100 um tolerance; float16 resolves 1953 um, twenty
    times coarser than the tolerance it would test. The verifier is untouched --
    `verify.py` re-derives everything in independent float64.

    `spin_bins` and `positioner_bins` at 25 put both at 14.4 degrees, under the
    15 this cell's 60 deg/s positioner allows over `DEFAULT_STEP_S`. At the 12
    and 9 they once were -- 30 and 40 degrees against a 25 degree threshold --
    NO EDGE can join two positioner bins, so the search picks one at layer 0 and
    welds the whole seam without it (`docs/issues/0004`).

    `held` pins the other axes on the way to the work frame -- `J9` on
    `rosie_1400_v3`; `held_work_axes` says why they are pinned rather than left.

    The two `*_offset_bins` are a fraction of ONE bin, `[0, 1)`. They ask whether
    a result depends on where the bins landed; they do not search finer, and
    `docs/issues/0004` has the numbers.
    """
    device = device or (torch.device("cuda", torch.cuda.current_device())
                        if torch.cuda.is_available() else torch.device("cpu"))
    # A mark CLOSES the stretch since the last one and names it -- a lap
    # button, so every `watch.mark("x")` below sits AFTER the work it labels.
    # It synchronises the device first, without which none of these numbers
    # mean anything on CUDA; `utils/timing.py`'s header is the argument.
    watch = Stopwatch(cuda_sync(device))
    cell = cell.to(device, dtype)
    descriptor = cell.descriptor
    tool_frame = str((descriptor.get("frames") or {}).get("tool") or "")
    if not tool_frame:
        raise NodeError("this cell names no tool frame; there is nothing to place")
    work_frame = str((descriptor.get("frames") or {}).get("work") or "")
    if not work_frame:
        raise NodeError("this cell names no work frame")
    # `None` asks the CELL, and the cell may answer `None` too -- a station with
    # no positioner. So this stays `None` from here on and every use of it below
    # is guarded; there is no sentinel, because "the caller did not say" and
    # "there is nothing to say" want the same behaviour.
    if positioner_axis is None:
        positioner_axis = cell.positioner_axis

    tool_link = cell.arm.chain.link_names.index(tool_frame)
    arm_lower, arm_upper = cell.arm.joint_limits

    L = samples_for(seam, step_s) if samples is None else int(samples)
    # ONE bin when nothing turns the work, whatever the caller asked for. Not a
    # cap: there is a single way the part can sit, and `positioner_bins` on such
    # a cell would enumerate the same pose that many times -- P copies of every
    # node, P times the solve, and a lattice whose `feasible` rate is unchanged
    # while its cost is not.
    S = spin_bins
    P = positioner_bins if positioner_axis is not None else 1
    fractions = np.linspace(0.0, 1.0, L)
    # ONE offset for the whole lattice, not one per layer. Jittering layers
    # independently would mean holding one bin along the seam no longer holds one
    # angle, so every edge would inherit a bin's width of joint motion it does
    # not need.
    spin_rad = spin_bin_angles(S, spin_offset_bins)
    if positioner_axis is None:
        if positioner_offset_bins:
            raise NodeError(
                f"positioner_offset_bins is {positioner_offset_bins} and this cell "
                "has no positioner; there is no grid for it to shift")
        # The one bin's angle, and the limits row that would describe it, are
        # both absent rather than zero: a zero here is a claim about an axis,
        # and `limits_rad` with a spurious row would make the graph's state one
        # column wider than the arm.
        positioner_rad = np.zeros(1, dtype=float)
        positioner_bounds = np.zeros((0, 2), dtype=float)
    else:
        positioner_lower, positioner_upper = positioner_plan_limits(
            *positioner_hardware_limits(descriptor, positioner_axis),
            plan_span_rad=positioner_span_rad)
        positioner_rad = positioner_bin_angles(
            positioner_lower, positioner_upper,
            bin_count=P, offset=positioner_offset_bins)
        positioner_bounds = np.array([[positioner_lower, positioner_upper]], dtype=float)

    # Sampled in the PART frame -- see "From the part to the world" below. At the
    # SEAM's precision, the higher of the two, and cast here so no caller has to
    # keep two dtypes in step.
    watch.mark("setup")
    sampled = seam.frames(torch.as_tensor(fractions, dtype=seam.dtype, device=device))
    sampled = {key: (value.to(dtype) if torch.is_tensor(value) else value)
               for key, value in sampled.items()}
    # THE PROGRAM'S OWN ANGLES, and there is no argument for asking for others:
    # a lattice planned at angles nobody authored answers about a different weld
    # than the one being welded. This took `work_deg=0.0, travel_deg=0.0` and
    # every caller in the repo let it, which is `docs/issues/0005` -- across the
    # corpus the authored travel angle averages 10.5 degrees and swings 24.5
    # within a single seam, so the flat weld it planned was nobody's.
    #
    # A FUNCTION OF POSITION, not a constant, which is why these are series.
    authored_work, authored_travel = authored_angles(seam, fractions)
    work_series = torch.as_tensor(authored_work, dtype=dtype, device=device)
    travel_series = torch.as_tensor(authored_travel, dtype=dtype, device=device)
    part_electrode = apply_angles(sampled["t"], sampled["r"],
                                  work_series, travel_series)

    # PART -> WORK: the operator's placement.
    work_pos_m, work_electrode = placed_on_the_table(
        sampled["pos_m"], part_electrode, work_to_piece,
        device=device, dtype=dtype)

    # WORK -> WORLD: the positioner, one pose per bin, carrying the seam with it.
    held_axes = held_work_axes(cell.work_chain, positioner_axis, held)
    workpiece_rotation, workpiece_origin_pos_m = workpiece_pose_per_bin(
        cell.work_chain, work_frame, positioner_axis=positioner_axis,
        positioner_angles_rad=positioner_rad, held_axes_rad=held_axes,
        device=device, dtype=dtype)
    world_pos_m, world_electrode = seam_turned_under_the_torch(
        work_pos_m, work_electrode, workpiece_rotation, workpiece_origin_pos_m,
        spin_bins=S)
    # Spin does not move the seam, so it is flattened separately -- same
    # `(sample, spin, bin)` order, one value per node.
    spin_per_node = torch.as_tensor(spin_rad, dtype=dtype, device=device
                                    ).reshape(1, S, 1).expand(L, S, P).reshape(-1)

    tool_asset = cell.arm.tool
    if tool_axis is None:
        tool_axis = tool_asset.electrode_axis
    # THE PROGRAM'S OWN STANDOFF, per sample, for the same reason the angles
    # above are the program's own. A distance that trades clearance against
    # shielding with the local geometry is the seam's to author, not one
    # number a cell asset states for every weld it will ever run.
    standoff_series_m = torch.as_tensor(authored_standoff_m(seam, fractions),
                                        dtype=dtype, device=device)
    standoff_per_node = standoff_series_m.reshape(L, 1, 1).expand(L, S, P).reshape(-1)

    target_rot, target_pos = torch_pose(world_pos_m, world_electrode,
                                        spin_per_node,
                                        standoff_m=standoff_per_node,
                                        tool_axis=tool_axis)

    watch.mark("targets")
    q, reaches, error, B = _solve_ik_all_branches(
        cell, tool_link, target_rot, target_pos,
        analytic=analytic, device=device, dtype=dtype)

    # The cause, decided per node -- an if/elif/else written as masked
    # overwrites, because there are millions of nodes: NO_IK unless the branch
    # reaches its target, JOINT_LIMIT when it reaches but a joint sits outside
    # its range, FEASIBLE only when both hold. Telling those first two apart is
    # why the solve ran unclamped.
    inside_limits = ((q >= arm_lower) & (q <= arm_upper)).all(dim=-1)
    cause = torch.full(reaches.shape, NO_IK, dtype=torch.int8, device=device)
    cause = torch.where(reaches & inside_limits,
                        torch.full_like(cause, FEASIBLE), cause)
    cause = torch.where(reaches & ~inside_limits,
                        torch.full_like(cause, JOINT_LIMIT), cause)
    _mark_duplicate_branches(q, cause)

    watch.mark("ik")
    clearance = None
    if scene is not None:
        # ONLY the nodes that are still feasible. The verdict this screen
        # feeds -- `colliding` below -- is masked to `cause == FEASIBLE`, yet
        # the screen used to run on every branch of every node, NO_IK and
        # JOINT_LIMIT included. At this corpus's feasible rates (0.6-6%) that
        # is 16-160x the clearance queries the verdict can use, and on the
        # deployment target the screen is compute-bound: measured on Thor,
        # 462 ms of a 1,193 ms seam. A node the screen never saw keeps +inf,
        # which reads as "not measured, not needed" -- nothing outside this
        # function consumes the grid.
        flat_q = q.reshape(B * len(target_pos), -1)
        angles = torch.as_tensor(positioner_rad, dtype=dtype, device=device)
        positioner_flat = (angles.reshape(1, 1, P).expand(L, S, P)
                           .reshape(-1).repeat(B))
        keep = (cause == FEASIBLE).reshape(-1)
        clearance_flat = torch.full((len(flat_q),), float("inf"),
                                    dtype=dtype, device=device)
        if bool(keep.any()):
            # The positioner column goes in only when there is one. On a cell
            # without, the workpiece is part of the scene's fixed geometry and
            # the arm alone is what moves, which is what `_screen_collision`
            # receives.
            clearance_flat[keep] = _screen_collision(
                scene, cell.arm.chain, flat_q[keep],
                None if positioner_axis is None else positioner_flat[keep],
                positioner_axis, held_axes, device, dtype)
        clearance = clearance_flat.reshape(B, -1)
        colliding = (clearance < clearance_floor_m) & (cause == FEASIBLE)
        cause = torch.where(colliding, torch.full_like(cause, COLLISION), cause)
        clearance = _to_grid(clearance, B, L, S, P)

    watch.mark("screen")
    lattice = Lattice(
        fractions=fractions, spin_rad=spin_rad, positioner_rad=positioner_rad,
        positioner_axis=positioner_axis, axis_names=list(cell.arm.chain.axis_names),
        held=held_axes, step_s=(None if samples is not None else float(step_s)),
        limits_rad=np.concatenate([
            np.stack([arm_lower.double().cpu().numpy(),
                      arm_upper.double().cpu().numpy()], axis=-1),
            positioner_bounds]),
        work_deg=work_series.cpu().numpy(),
        travel_deg=travel_series.cpu().numpy(),
        standoff_mm=standoff_series_m.double().cpu().numpy() * 1000.0,
        branches=B,
        q=_to_grid(q, B, L, S, P, tail=cell.arm.chain.num_dof),
        cause=_to_grid(cause, B, L, S, P),
        position_error_m=_to_grid(error, B, L, S, P),
        clearance_m=clearance,
        weld_pos_m=world_pos_m.reshape(L, S, P, 3)[:, 0, :, :].contiguous(),
        # The WORK-frame electrode, before the positioner: the turn measured is
        # the one along the seam, and a bin adds the same rotation to every
        # sample in it.
        electrode_turn_rad=_turn(work_electrode),
    )
    watch.mark("grid")
    # No `total`: `plan_seam` measures the whole call as its `lattice` stage,
    # and a second total inside would be counted twice by anything summing
    # these into a breakdown.
    lattice.timings_ms = watch.ms(total=False)
    return lattice


# --------------------------------------------------------------------------
# How many samples


def travel_speed_m_s(seam: Any) -> float:
    """How fast the torch walks this seam, in metres a second.

    From the seam's own `process.travel_speed_mm_s`. Constant only for now: a
    varying speed makes the seam's duration an integral rather than a division,
    and no program in the corpus writes one, so inventing the integral would be
    inventing a test for it too.
    """
    process = getattr(seam, "process", None)
    if process is None:
        process = (seam or {}).get("process") or {}
    spec = process.get("travel_speed_mm_s") or {}
    kind = str(spec.get("kind") or "")
    if kind != "constant":
        raise NodeError(
            f"travel speed is a {kind!r} profile and this only knows constants; "
            f"the seam's duration would be an integral and nothing authors one yet")
    value = float(spec.get("value") or 0.0)
    if value <= 0.0:
        raise NodeError("travel speed must be positive to time a seam")
    return value / 1000.0


def samples_for(seam: Any, step_s: float) -> int:
    """How many samples put `step_s` seconds of travel between neighbours.

    **A fixed count per seam is the wrong unit and this is why.** The edge rule
    is a rate limit -- a joint may move its velocity limit times the time
    between samples -- so a fixed count makes one threshold mean a different
    thing on every seam. Across this corpus 32 samples spaces neighbours 0.32 mm
    apart on a 10 mm seam and 10.13 mm apart on a 314 mm one, a factor of
    thirty, against a single 25 degree threshold that can only be right for one
    of them.

    Timed instead, the threshold is one number everywhere: `rate x step_s`.

    At least two, because a seam is a path and a path needs somewhere to come
    from.
    """
    if step_s <= 0.0:
        raise NodeError("the sample step must be a positive number of seconds")
    seconds = float(seam.length_m) / travel_speed_m_s(seam)
    return max(2, int(np.ceil(seconds / step_s)) + 1)


# --------------------------------------------------------------------------
# Where the bins are


def _bin_offset(name: str, given: Any) -> float:
    """A grid offset: a fraction of ONE bin, `[0, 1)`.

    Refused rather than taken modulo one. A caller who wrote 1.5 meant something
    an offset cannot do, and `docs/issues/0004` measured that what they probably
    wanted is a different bin COUNT.
    """
    offset = float(given)
    if not 0.0 <= offset < 1.0:
        raise NodeError(
            f"{name} is {offset}; an offset is a fraction of ONE bin, so "
            "0 <= it < 1. Past a bin it is not a shifted grid any more: a closing "
            "axis lands back on itself, so the run repeats the one before it.")
    return offset


def spin_bin_angles(bin_count: int, offset: float = 0.0) -> np.ndarray:
    """Where the torch's roll about its own electrode is sampled: one whole turn.

    `endpoint=False`, because -180 and +180 are one posture. Binning both gives
    that posture twice the weight of every other in every count the graph makes.

    An offset under one bin leaves the top sample below `+pi`, so nothing wraps
    and the array stays sorted -- `positioner_bin_angles` has the cost of losing
    that.
    """
    bin_width = 2.0 * np.pi / bin_count
    return (np.linspace(-np.pi, np.pi, bin_count, endpoint=False)
            + _bin_offset("spin_offset_bins", offset) * bin_width)


def positioner_hardware_limits(descriptor: dict[str, Any],
                               positioner_axis: str) -> tuple[float, float]:
    """The joint's own position limits, read off the cell.

    Refused when absent rather than defaulted, because bins outside what the
    hardware states would be invented angles.
    """
    joint = next((j for j in descriptor.get("joints") or []
                  if str(j.get("name")) == positioner_axis), None)
    if joint is None:
        raise NodeError(f"no joint named {positioner_axis!r} in this cell")
    limits = joint.get("limits_rad") or {}
    if "lower" not in limits or "upper" not in limits:
        raise NodeError(f"{positioner_axis} has no position limit; "
                        "its bins would be invented")
    return float(limits["lower"]), float(limits["upper"])


def positioner_plan_limits(hardware_lower: float, hardware_upper: float, *,
                           plan_span_rad: float) -> tuple[float, float]:
    """The bound the plan holds the positioner to: the joint's own limits,
    intersected with `plan_span_rad` centred on zero.

    +-90 degrees by default -- the owner's call. It has to reach BOTH the bins
    and `Lattice.limits_rad`, or the sweep winds past what was enumerated, and
    the plan commands a posture nothing screened. That is why this is a function
    two call sites share rather than an expression one of them owns.
    """
    half_span = float(plan_span_rad) / 2.0
    return (max(float(hardware_lower), -half_span),
            min(float(hardware_upper), half_span))


def positioner_bin_angles(lower: float, upper: float, *, bin_count: int,
                          offset: float = 0.0) -> np.ndarray:
    """Where the positioner is sampled, inside the bound it is given.

    A WHOLE TURN and a CAPPED ARC are different shapes, and the offset is what
    makes the difference visible:

    - A whole turn has the same orientation at both ends, so the top bin is left
      out and there is a bin of slack. The offset is a true shift -- same width,
      every bin moved by the same amount.
    - A capped arc has a bin ON each end and nowhere to shift into. The start
      moves and the top stays, so the width tightens by `offset / (bins - 1)`,
      1.1% at the default 90. Tighter is the direction the edge rule absorbs;
      a gap too WIDE is what pins an axis (`docs/issues/0004`).

    Clipping a shifted grid back into the arc is what this did before, and it
    collapsed the MINIMUM gap instead of moving the bins: at 0.9 of a bin on the
    real 90-bin grid, 2.0225 degrees became 0.2022. `graph.search` sizes its band
    prune off that minimum, so the prune fell from dropping 72.2% of the pairs to
    dropping none -- at every nonzero offset, not just past a bin.
    """
    offset = _bin_offset("positioner_offset_bins", offset)
    lower, upper = float(lower), float(upper)
    if (upper - lower) >= 2.0 * np.pi - 1e-6:
        bin_width = (upper - lower) / bin_count
        return lower + offset * bin_width + np.arange(bin_count) * bin_width
    if bin_count < 2:
        if offset:
            raise NodeError(
                f"positioner_offset_bins is {offset} on a one-bin capped "
                "positioner; a single bin has no width to take a fraction of.")
        return np.asarray([lower], dtype=float)
    bin_width = (upper - lower) / (bin_count - 1)
    return np.linspace(lower + offset * bin_width, upper, bin_count)


# --------------------------------------------------------------------------
# The angles and the standoff the program authored


def _authored_profile(key: str, spec: dict[str, Any], fr: np.ndarray) -> np.ndarray:
    """One authored profile, evaluated per fraction.

    `constant` and `piecewise_linear` are the only shapes in the corpus;
    anything else is REFUSED rather than flattened to zero, because a weld
    planned at a value nobody authored is the failure this exists to end
    (`docs/issues/0005`).
    """
    kind = str(spec.get("kind") or "")
    if kind == "constant":
        return np.full(fr.shape, float(spec.get("value", 0.0)))
    if kind == "piecewise_linear":
        knots = np.asarray(spec.get("knots") or [], dtype=np.float64)
        if knots.ndim != 2 or knots.shape[0] < 1:
            raise NodeError(f"{key} is piecewise_linear with no usable knots")
        return np.interp(fr, knots[:, 0], knots[:, 1])
    raise NodeError(f"{key} is a {kind!r} profile and this does not know "
                    f"how to evaluate one; welding it at zero would be a "
                    f"different weld than the one authored")


def authored_angles(seam: Any, fractions: Any) -> tuple[Any, Any]:
    """`(work_deg, travel_deg)` the weld PROGRAM asked for, per sample.

    Takes a `Seam` or a raw seam document. An absent spec is zeros; a spec
    with a shape this cannot evaluate is refused -- `_authored_profile`.

    `fractions` are the SEAM's own 0..1, not the weld line's -- the axis the
    authoring side evaluates these on too.

    A second implementation on purpose: this package may not import
    `seam_worker` (`tests/motion_planner/test_boundary.py`), so the two are
    checked against the committed plan requests instead.
    """
    orientation = getattr(seam, "orientation", None)
    if orientation is None:
        orientation = (seam or {}).get("orientation") or {}
    fr = np.atleast_1d(np.asarray(fractions, dtype=np.float64))
    out = []
    for key in ("work_angle_deg", "travel_angle_deg"):
        spec = orientation.get(key) or {}
        out.append(_authored_profile(key, spec, fr) if spec else np.zeros(fr.shape))
    return out[0], out[1]


def authored_standoff_m(seam: Any, fractions: Any) -> np.ndarray:
    """(N,) how far off the work the PROGRAM holds the tool origin, in metres.

    From the seam's own `process.standoff_mm`, the same profile shapes as the
    angles; a seam that authors none falls back to its wire's `stickout_mm`,
    the one number the procedure already states for how far the torch is held
    off the work (`seam_worker.reach.tooling` makes the same call).

    UNLIKE the angles, an absent spec is not zeros: zero puts the tip in the
    part, so a seam that authors neither profile nor stickout is refused.
    A cell's tool asset once stated one standoff for every weld the cell would
    ever run; a distance that trades clearance against shielding with the
    local geometry is the program's to author, per sample.
    """
    process = getattr(seam, "process", None)
    if process is None:
        process = (seam or {}).get("process") or {}
    fr = np.atleast_1d(np.asarray(fractions, dtype=np.float64))
    spec = process.get("standoff_mm")
    if spec:
        return _authored_profile("standoff_mm", spec, fr) / 1000.0
    stickout = (process.get("wire") or {}).get("stickout_mm")
    if isinstance(stickout, int | float) and not isinstance(stickout, bool) \
            and float(stickout) > 0.0:
        return np.full(fr.shape, float(stickout) / 1000.0)
    raise NodeError(
        "this seam authors no process.standoff_mm and its wire states no "
        "stickout_mm; planning at zero would put the tip in the part")


# --------------------------------------------------------------------------
# From the part to the world, in two moves
#
#   PART   where the seam is authored, and the frame the STEP is in.
#   WORK   the positioner's table. `work_to_piece`, the operator's placement, is
#          the move.
#   WORLD  the cell floor, where the arm's inverse kinematics is posed. Driving
#          the cell's chain to its work frame is the move, and the positioner
#          angle is what it depends on.
#
# So a positioner bin does not change the target the arm is given -- it changes
# WHERE THE WELD IS, because the part is bolted to the thing that turned.


def placed_on_the_table(part_pos_m: torch.Tensor, part_electrode: torch.Tensor,
                        work_to_piece: Any, *, device, dtype
                        ) -> tuple[torch.Tensor, torch.Tensor]:
    """PART frame to WORK frame: the seam, put where the operator put the part.

    A rigid placement, so the point is rotated and translated and the electrode
    direction only rotated. `None` means the caller has already done it, or is
    planning a part whose own frame IS the table's.
    """
    if work_to_piece is None:
        return part_pos_m, part_electrode
    placement = torch.as_tensor(work_to_piece, dtype=dtype, device=device)
    rotation, part_origin_pos_m = placement[:3, :3], placement[:3, 3]
    return (part_pos_m @ rotation.T + part_origin_pos_m,
            part_electrode @ rotation.T)


def held_work_axes(work_chain: Any, positioner_axis: str | None,
                   asked: dict[str, float] | None) -> dict[str, float]:
    """Every axis on the way to the work frame EXCEPT the one being binned, and
    the angle it is pinned at.

    Pinned EXPLICITLY, rather than left on the zero a fresh tensor happens to
    hold. `rosie_1400_v3` has two axes on that chain and bins one, so a screen
    "over the positioner" is really a screen over one of its axes -- and these
    values travel out on the lattice so a reader of any output can see which.

    `positioner_axis=None` bins NOTHING, so every axis on the chain is held --
    which on a cell whose work frame is its base is an empty chain and an empty
    dict. The distinction survives in `Lattice.positioner_axis`, not here: an
    empty `held` means "nothing was held back" on both cells.
    """
    on_chain = list(work_chain.axis_names)
    if positioner_axis is not None and positioner_axis not in on_chain:
        raise NodeError(f"{positioner_axis} is not on the chain to the work frame")
    off_chain = sorted(set(asked or {}) - set(on_chain))
    if off_chain:
        raise NodeError(f"{off_chain} are not on the chain to the work frame; "
                        f"it has {on_chain}")
    return {name: float((asked or {}).get(name, 0.0))
            for name in on_chain if name != positioner_axis}


def workpiece_pose_per_bin(work_chain: Any, work_frame: str, *,
                           positioner_axis: str | None,
                           positioner_angles_rad: np.ndarray,
                           held_axes_rad: dict[str, float],
                           device, dtype) -> tuple[torch.Tensor, torch.Tensor]:
    """WORK frame to WORLD, once per positioner bin.

    `(rotation, origin_pos_m)`, shaped `(bins, 3, 3)` and `(bins, 3)`.

    FORWARD KINEMATICS on a second chain, not an `Rz` written out here. The
    positioner is a table on a mount with its own offsets, so which axis it
    actually turns about is a fact in the cell's URDF; spelling a rotation about
    Z would be right on this cell only until it was not, and silently.

    That is also why a cell with NO positioner runs the same code rather than
    returning an identity: its work frame may still sit somewhere other than the
    base -- a table bolted to the floor -- and the fixed offsets on the way to it
    are exactly what this composes. `rosie_1420_v1` happens to name the base
    itself, and then the walk is zero joints long and the answer IS the identity,
    arrived at rather than assumed.
    """
    column_of = {name: i for i, name in enumerate(work_chain.axis_names)}
    joint_angles = torch.zeros(len(positioner_angles_rad), work_chain.num_dof,
                               dtype=dtype, device=device)
    for name, angle_rad in held_axes_rad.items():
        joint_angles[:, column_of[name]] = angle_rad
    if positioner_axis is not None:
        joint_angles[:, column_of[positioner_axis]] = torch.as_tensor(
            positioner_angles_rad, dtype=dtype, device=device)
    return torch_link_transform(
        work_chain, work_chain.link_names.index(work_frame), joint_angles)


def seam_turned_under_the_torch(work_pos_m: torch.Tensor,
                                work_electrode: torch.Tensor,
                                workpiece_rotation: torch.Tensor,
                                workpiece_origin_pos_m: torch.Tensor,
                                *, spin_bins: int
                                ) -> tuple[torch.Tensor, torch.Tensor]:
    """The seam in WORLD, one copy per positioner bin, flattened to one batch.

    Composed onto the GEOMETRY rather than inverted into the target, which leaves
    the arm's inverse kinematics a plain world-frame question. Folding the
    positioner into the target instead would put it inside the solve, where it is
    not one of the six unknowns.

    Returns `(pos_m, electrode)`, each `(samples * spin_bins * bins, 3)` in
    `(sample, spin, bin)` order -- the lattice's own, so a flat index and a grid
    index address the same node. Spin does not move the seam, so its copies are
    an `expand` and share storage until the reshape.
    """
    samples, bins = work_pos_m.shape[0], workpiece_rotation.shape[0]
    rotation = workpiece_rotation.reshape(1, 1, bins, 3, 3)
    turned_pos_m = (rotation @ work_pos_m.reshape(samples, 1, 1, 3).unsqueeze(-1)
                      ).squeeze(-1) + workpiece_origin_pos_m.reshape(1, 1, bins, 3)
    turned_electrode = (rotation @ work_electrode.reshape(samples, 1, 1, 3).unsqueeze(-1)
                        ).squeeze(-1)
    return (turned_pos_m.expand(samples, spin_bins, bins, 3).reshape(-1, 3),
            turned_electrode.expand(samples, spin_bins, bins, 3).reshape(-1, 3))


# --------------------------------------------------------------------------
# The pose the arm is asked for


def torch_pose(point: torch.Tensor, electrode: torch.Tensor, spin: torch.Tensor,
               *, standoff_m: Any, tool_axis: Any) -> tuple[torch.Tensor, torch.Tensor]:
    """The tool pose that puts the electrode on the weld, per spin value.

    `electrode` is `u`, pointing AWAY out of the work; `tool_axis` runs from the
    tool origin toward the tip in the TOOL's frame. So the pose carries
    `tool_axis` onto `-u`, and the origin sits `standoff` back along `+u`.
    `spin` rotates about `u`, which is why it is free.

    `standoff_m` is a scalar or `(N,)` -- the authored profile varies along
    the seam, and each pose here is one node of it.

    `tool_axis` is REQUIRED, never defaulted: a wrong one turns every target
    orientation by the angle between guess and truth, and nothing notices --
    inverse kinematics converges onto the rotated pose and the lattice fills
    with green.

    `(N, 3)` in, `(N, 3, 3)` and `(N, 3)` out.
    """
    u = electrode / electrode.norm(dim=-1, keepdim=True).clamp_min(1e-30)

    # A frame with `-u` as its third column, and a zero of spin that does not
    # collapse: cross with whichever world axis `u` is least parallel to.
    axis = -u
    ref = torch.zeros_like(axis)
    least = axis.abs().argmin(dim=-1)
    ref[torch.arange(len(axis), device=axis.device), least] = 1.0
    e0 = torch.linalg.cross(ref, axis, dim=-1)
    e0 = e0 / e0.norm(dim=-1, keepdim=True).clamp_min(1e-30)
    e1 = torch.linalg.cross(axis, e0, dim=-1)

    c, s = torch.cos(spin).unsqueeze(-1), torch.sin(spin).unsqueeze(-1)
    a = c * e0 + s * e1
    b = torch.linalg.cross(axis, a, dim=-1)
    # `world` carries the tool's own (a1, a2, tool_axis)-aligned triad into
    # world, so composing with the tool triad's inverse gives the tool pose.
    world = torch.stack([a, b, axis], dim=-1)

    tool = _tool_triad(tool_axis, point.device, point.dtype)
    rot = world @ tool.transpose(-1, -2)
    off = torch.as_tensor(standoff_m, dtype=point.dtype, device=point.device)
    return rot, point + u * (off.unsqueeze(-1) if off.ndim else off)


def _tool_triad(tool_axis: Any, device, dtype) -> torch.Tensor:
    """A right-handed 3x3 whose THIRD column is `tool_axis`, in the tool frame.

    Built the same way the world triad above is, so that `world @ tool^T` is a
    pure rotation and the spin parameter still means a turn about the
    electrode rather than about whatever the construction happened to pick.
    """
    a = torch.as_tensor(np.asarray(tool_axis, dtype=np.float64), device=device, dtype=dtype)
    a = a / a.norm().clamp_min(1e-30)
    ref = torch.zeros(3, device=device, dtype=dtype)
    ref[int(a.abs().argmin())] = 1.0
    e0 = torch.linalg.cross(ref, a)
    e0 = e0 / e0.norm().clamp_min(1e-30)
    e1 = torch.linalg.cross(a, e0)
    return torch.stack([e0, e1, a], dim=-1)


# --------------------------------------------------------------------------
# The solve: up to eight roots per pose


def _solve_ik_all_branches(
        cell: RobotCell, tool_link: int,
        target_rot: torch.Tensor, target_pos: torch.Tensor,
        *, analytic: bool, device, dtype,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, int]:
    """The arm's joints at every target pose, every branch kept.

    `(q, reaches, position_error_m, branch_count)`, shaped `(B, N, dof)`,
    `(B, N)`, `(B, N)`. `reaches` is "this branch holds a posture that
    reaches the target" -- on the closed form that is existence plus an
    audit, on the iterative path it is convergence; the mask means the
    same thing to the caller either way.

    UNCLAMPED to the joint limits on purpose: clamping would make "the arm has
    no travel for this" and "no posture reaches this" the same event, and the
    frontier histogram exists to tell them apart.

    Every branch is KEPT, not collapsed to the first that reaches: branches
    are disconnected islands, and a plan may need a different one partway along
    the seam. Collapsing was measured -- 0.19% of adjacent feasible samples
    jumped by exactly pi or 2pi against a 99th percentile of 0.013 rad, which
    are branch flips the edge rule would have cut the lattice at, in places
    decided by seed order rather than geometry.
    """
    arm_chain = cell.arm.chain
    seed = torch.zeros(len(target_pos), arm_chain.num_dof, dtype=dtype, device=device)
    seed[:, 1] = -0.8
    seed[:, 2] = 0.8

    # The arm's closed-form description, or `None` for either of two reasons:
    # the caller said `analytic=False`, or `ik_closed.form_of` measured the arm
    # and it does not HAVE a closed form -- wrong axis count, a turned joint
    # frame, axes off the canonical pattern. `None` is not a failure; it means
    # this cell takes the iterative solver below, slower and never wrong.
    closed_form = None
    if analytic:
        zero = torch.zeros(1, arm_chain.num_dof, device=device, dtype=dtype)
        rot0, pos0 = torch_link_transform(arm_chain, tool_link, zero)
        closed_form = ik_closed.form_of(cell.descriptor,
                                        list(arm_chain.axis_names),
                                        rot0[0].double().cpu().numpy(),
                                        pos0[0].double().cpu().numpy())
    if closed_form is not None:
        # CLOSED FORM. All eight roots in one elementwise pass -- no iteration,
        # no convergence tolerance, and the branch index names a posture family
        # instead of a seed. Between 261x and 863x the iterative path, and exact
        # against double-precision kinematics where the iterative one stops at
        # whatever tolerance it was given.
        # `exists` is NOT convergence -- nothing here iterates. It is the
        # algebra's own domain conditions: the shoulder's `arcsin` in range,
        # the elbow triangle closing, the wrist away from its singularity.
        q, exists = ik_closed.solve(closed_form, target_rot, target_pos)
        branch_count = ik_closed.BRANCHES
        # And this is not a tolerance check but an AUDIT: FK the root back
        # and measure that it lands on the target, so a pose where the
        # closed form's premises leak is caught rather than trusted.
        _rot_back, pos_back = torch_link_transform(
            arm_chain, tool_link, q.reshape(-1, arm_chain.num_dof))
        error = torch.linalg.norm(
            pos_back - target_pos.repeat(branch_count, 1),
            dim=-1).reshape(branch_count, -1)
        reaches = exists & (error < ik.DEFAULT_POS_TOL_M)
        # A branch that does not exist still holds NUMBERS: `ik_closed` clamps
        # its arcsin/arccos inputs so the elementwise pass never dies, and a
        # dead branch's six values are that clamp's residue -- garbage that
        # LOOKS like a posture. Overwritten with the seed, the same value the
        # iterative path leaves in poses it skipped, so a "no ik" node holds
        # the same thing whichever solver built the lattice.
        q = torch.where(reaches.unsqueeze(-1), q, seed.unsqueeze(0))
        error = torch.where(reaches, error, torch.full_like(error, float("inf")))
        return q, reaches, error, branch_count

    # ITERATIVE, over every pose, for an arm the closed form cannot describe.
    # Slower is this path's whole contract; there is deliberately no reach
    # prefilter in front of it any more. The one that was here skipped 28.5%
    # of the solves on `dog_box` seam 1 and never changed an answer -- but it
    # was 170 lines of wrist-centre geometry optimizing a fallback that no
    # committed cell takes, and alongside the closed form it was measured as
    # the same test twice (93.2 ms with against 89.9 without at 64 bins).
    solutions = ik.solve_multi(arm_chain, tool_link, target_rot, target_pos, seed)
    return (torch.stack([sol.q for sol in solutions], dim=0),
            torch.stack([sol.converged for sol in solutions], dim=0),
            torch.stack([sol.position_error_m for sol in solutions], dim=0),
            len(solutions))


def _mark_duplicate_branches(q: torch.Tensor, cause: torch.Tensor) -> None:
    """Relabel FEASIBLE branches that landed on a posture an earlier branch
    already holds -- see `DUPLICATE`. In place, sequential over the branches
    (eight of them) and batched over everything else."""
    for later in range(1, len(cause)):
        seen = torch.zeros_like(cause[later], dtype=torch.bool)
        for earlier in range(later):
            seen |= ((q[later] - q[earlier]).abs().max(dim=-1).values < BRANCH_TOL_RAD) \
                & (cause[earlier] != NO_IK)
        cause[later] = torch.where(seen & (cause[later] == FEASIBLE),
                                   torch.full_like(cause[later], DUPLICATE),
                                   cause[later])


# --------------------------------------------------------------------------
# The collision screen


def _screen_collision(scene, arm_chain, q_arm, positioner_per_pose,
                      positioner_axis, held_axes, device, dtype, *,
                      chunk_bytes: int = COLLISION_CHUNK_BYTES) -> torch.Tensor:
    """Least clearance per pose, through the scene the search already has.

    The check itself is `scene.checker` -- one home for the axis mapping, the
    held axes and the arm-against-itself merge, shared with M5 and M6. What is
    THIS function's is the batch's size. The caller now hands poses it has
    already SELECTED (the feasible subset) with each pose's positioner angle
    beside it, so the lattice's grid shape no longer lives here.

    IN CHUNKS, for the same reason the graph's transition is: one pose's
    clearance does not depend on another's, so slicing the batch is an identity,
    and unsliced this is by far the largest thing in the process. The arm
    carries 300 spheres, so a 64-bin lattice asks for the world position of
    1048576 x 300 = 315 million points -- 3.5 GiB of coordinates before the
    distance field is consulted, and `fk.py` pairs every point buffer with a
    gradient buffer it allocates whether or not anything is differentiating.

    Measured before it was chunked: 1.01 GiB of peak became 24.23 at 64 bins,
    and 96 bins ran out of memory asking for 7.91 GiB in one go. The screen
    costs about 10% in TIME -- it was never the clock -- but it capped the
    resolution at half of what the rest of the build manages.
    """
    # `positioner_axis` and `positioner_per_pose` are `None` together on a cell
    # with no positioner, and then the pose IS the arm's joints. `PoseCheck`
    # would drop an axis it does not know, so a `None` in the name list is not
    # what catches this -- the column would be built and silently ignored, and
    # the two lengths would still have to agree.
    if positioner_axis is None:
        check = scene.checker(list(arm_chain.axis_names), held_axes)
        q_moved = q_arm
    else:
        check = scene.checker([*arm_chain.axis_names, positioner_axis], held_axes)
        q_moved = torch.cat([q_arm, positioner_per_pose.unsqueeze(-1)], dim=-1)
    # PADDED to one fixed chunk size, and this is load-bearing: the fused FK
    # underneath caches buffers AND records a CUDA graph per distinct batch
    # size. A subset whose length is "however many nodes were feasible" is a
    # new batch size on nearly every seam of every request, and each one
    # keeps its graph for the life of the process -- measured, the test
    # suite leaked its way to 24.7 GiB and OOM'd. Padding with copies of the
    # first pose makes every chunk THE SAME size; the copies' clearances are
    # computed and discarded.
    n = len(q_moved)
    spheres = int(scene.radii.shape[0])
    # 40 bytes a sphere: points, their gradient twins, the per-sphere clearance
    # and its reduction. Fitted against the allocator like
    # `graph.transition_bytes`, and high rather than low for the same reason.
    bytes_per_pose = spheres * 40
    poses_per_chunk = max(1, int(chunk_bytes // bytes_per_pose))
    padded = -(-n // poses_per_chunk) * poses_per_chunk
    if padded != n:
        q_moved = torch.cat(
            [q_moved, q_moved[:1].expand(padded - n, -1)], dim=0)
    least_clearance = torch.empty(padded, dtype=dtype, device=device)
    # The screen only READS; nothing in it may ever join an autograd graph.
    with torch.no_grad():
        for start in range(0, padded, poses_per_chunk):
            gap = check.clearance(q_moved[start:start + poses_per_chunk])
            least_clearance[start:start + poses_per_chunk] = gap.to(dtype)
    return least_clearance[:n]


# --------------------------------------------------------------------------
# Back onto the grid


def _to_grid(flat: torch.Tensor, B: int, L: int, S: int, P: int, tail: int = 0):
    """`(B, L*S*P[, tail]) -> (L, S, P, B[, tail])`.

    The branch axis is moved LAST so the first three stay the ones every
    caller indexes by -- a sample is still `lattice.cause[i]`, and a view that
    only cares about samples does not have to know how many branches there
    were.
    """
    shape = (B, L, S, P) + ((tail,) if tail else ())
    moved = flat.reshape(shape)
    order = (1, 2, 3, 0) + ((4,) if tail else ())
    return moved.permute(*order).contiguous()


def _turn(direction: torch.Tensor) -> torch.Tensor:
    """(L-1,) the angle between consecutive electrode directions."""
    a, b = direction[:-1], direction[1:]
    a = a / a.norm(dim=-1, keepdim=True).clamp_min(1e-30)
    b = b / b.norm(dim=-1, keepdim=True).clamp_min(1e-30)
    return torch.arccos((a * b).sum(dim=-1).clamp(-1.0, 1.0))


# --------------------------------------------------------------------------
# CLI


def self_test() -> int:
    if not torch.cuda.is_available():
        print("nodes: no CUDA; the lattice is a batched torch path")
        return 0
    import time
    from pathlib import Path

    from weld_motion_planner.io.plan_request import read
    from weld_motion_planner.robot_cell.collision.scene import placement_matrix

    root = Path(__file__).resolve().parents[5] / "data" / "motion"
    request = read(root / "bracket_b_2x.weldplan")
    cell = RobotCell.of(request.cell, device=torch.device("cuda"))
    seam_spec = request.seams[0]
    joints = request.joints_by_id
    seam = Seam(seam_spec, joints.get(str(seam_spec.get("weld_joint_ref"))),
                device=torch.device("cuda"))

    print(f"nodes: seam {seam.id!r}, {seam.length_m * 1000:.1f} mm")
    print(f"  {MOUNT_NOTE}")
    standoff_mm = authored_standoff_m(seam, np.linspace(0.0, 1.0, 64)) * 1000.0
    print(f"  the program holds the tool origin "
          f"{standoff_mm.min():.0f}-{standoff_mm.max():.0f} mm off the weld")

    t0 = time.perf_counter()
    lattice = build(cell, seam,
                    work_to_piece=placement_matrix(request.placement))
    torch.cuda.synchronize()
    ms = (time.perf_counter() - t0) * 1000.0
    summary = lattice.summary()
    print(f"  {summary['nodes']} nodes in {ms:.0f} ms "
          f"({ms / summary['nodes'] * 1000:.1f} us each)")
    print(f"  {summary}")

    if summary["feasible"] == 0:
        print("nodes: FAIL, nothing is weldable -- a lattice with no feasible node "
              "cannot exercise anything downstream")
        return 1
    if summary["causes"]["no ik"] == 0 and summary["causes"]["joint limit"] == 0:
        print("nodes: FAIL, every node passed; the screen is not screening")
        return 1

    counts = lattice.per_layer()
    print(f"  feasible per sample: min {counts.min()}, median "
          f"{int(np.median(counts))}, max {counts.max()}")
    frontier = lattice.frontier()
    print(f"  frontier: {'none, every sample has a node' if frontier is None else frontier}")

    # A node that says feasible must actually put the torch where it belongs.
    ok = lattice.feasible.reshape(-1)
    if bool(ok.any()):
        worst = float(lattice.position_error_m.reshape(-1)[ok].max())
        print(f"  worst position error among feasible nodes: {worst * 1000:.4f} mm")
        if worst > ik.DEFAULT_POS_TOL_M:
            print("nodes: FAIL, a node called feasible does not reach its target")
            return 1

    print("nodes: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except (NodeError, ToolError, ik.IKError) as exc:
        print(f"nodes: {exc}", file=sys.stderr)
        raise SystemExit(1)
