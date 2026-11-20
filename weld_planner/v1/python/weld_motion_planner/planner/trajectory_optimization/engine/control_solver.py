#!/usr/bin/env python
"""The free-space solve, on B-spline CONTROL POINTS.

`docs/issues/0015` is why this exists beside `freespace.py` rather than inside
it. The old solver optimizes knot POSITIONS and derives each tangent as a
centred difference, which is a Catmull-Rom spline: `C1`, so acceleration jumps
at every knot -- measured at 1.73 rad/s^2 on one transit, 0.83 of this cell's
own limit -- and `verify.check_limits` differences the waypoints it is handed,
so it samples the knots and steps over the jump.

Here the variable is the control polygon of a clamped cubic B-spline. Three
things follow, and they are the whole reason for the change:

- **`C2` by construction.** Acceleration is continuous; there is no jump to
  price, penalise or miss.
- **Derivatives are analytic.** `q, v, a, j` come from four constant matrices
  (`bspline.Basis`), so a limit check means the same thing at any sample
  density. A finite difference on a coarse grid under-reads, and on a fine one
  reads a `C1` curve's jump as unbounded.
- **The step BUDGET disappears.** The old solver capped each knot-to-knot step
  and had to keep the horizon long enough to pay for the path (`_repaced`,
  `_carry_floor`, `_move_caps`). A control point is not a step, so there is
  nothing to run out of -- and with it goes the failure those guards existed
  for: a re-paced seed stopping short of its own endpoint and `_clamp_ends`
  turning the shortfall into one lunge at the weld.

## What is kept

The collision model is this repository's, unchanged: spheres for the body,
fields for everything it must not hit, the augmented Lagrangian and its buffer
(`collision/cost.py`), and the structural adjacent-pair rule
(`collision/pairs.py`). So is the monotone Adam descent and its CUDA graph
capture, which is a property of the loop rather than of the representation.

What is replaced is the representation and the rate cost (`rates.py`).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
import torch

from weld_motion_planner.planner.trajectory_optimization.engine.bspline import (
    DEGREE,
    Basis,
    clamped_knots,
)
from weld_motion_planner.planner.trajectory_optimization.engine.rates import (
    W_COLLIDE_ENV,
    W_COLLIDE_SELF,
    Limits,
    RateCost,
    collision_hinge,
)

#: Control points per motion. cuRobo's `n_knots: 16` for the same job. More
#: buys shape the collision term can use; fewer is smoother by force rather
#: than by choice.
CONTROL_POINTS = 16

#: Curve samples per control-point span for the COST grid -- cuRobo's
#: `interpolation_steps`, and sized the way it sizes its: a ratio to the
#: control count rather than a flat number. The cost, the collision term
#: and every ratio `retime` accepts on are computed at these samples, so
#: this is the resolution the OPTIMIZER judges at; the wire the verifier
#: certifies is sampled separately by time (`WIRE_DT_S`). cuRobo runs 4
#: (61 samples on 16 knots) under a swept collision term; this planner's
#: collision hinge is point-sampled, so the ratio was swept rather than
#: copied: 4, 6, 8 and 10 (61-151 samples) all held every contract on the
#: corpus -- the T2 certificate, the rate limits on the curve, the pinned
#: stops -- and the detour sat at 1.20-1.21 regardless. Nothing the grid
#: was paying for above 4 was buying anything measurable, and the cost
#: grid is the per-step price of every descent.
INTERPOLATION_STEPS = 4

#: The cost grid: `(CONTROL_POINTS - 1) * INTERPOLATION_STEPS + 1`.
SAMPLES = (CONTROL_POINTS - 1) * INTERPOLATION_STEPS + 1

#: Sample spacing of the WIRE -- the `(q, qdot, t)` Hermite the controller
#: replays and the T2 verifier certifies. By TIME, not by count: a fixed
#: count made a 2.2 s transit 73 samples/s and an 8 s approach 20, so the
#: certified density fell with the length of the motion. 0.01 s, the
#: controller's own clock (cuRobo's `interpolation_dt` is 0.02). A motion
#: whose duration is not a multiple of it takes `round(T / 0.01)` intervals
#: spread evenly, so the spacing is the target and not exactly it. The
#: curve is exact at any time (the basis is evaluated, not interpolated),
#: so the wire's density is a choice about the controller, not about the
#: solve. The verifier no longer pays for it: it certifies the curve on its
#: cubic pieces (`freespace.certified_samples`) and reads its clearance report
#: on a bounded sample (`freespace.REPORT_SAMPLES`); BENCHMARKS.md 17.
WIRE_DT_S = 0.01

#: A wire never has fewer intervals than this, however short the motion.
WIRE_MIN_INTERVALS = 8

#: Cap on reads per HALF CHORD in the swept collision term -- cuRobo's
#: `SWEEP_STEPS`, and its walk (`scene._make_sweep_kernel`): from each
#: sample's sphere centre toward each neighbour's, advancing by the
#: clearance just read, so a chord far from anything costs ONE read and only
#: a chord near the part walks. Measured on the 22-seam corpus at 61 cost
#: samples: the point-sampled hinge let a transit penetrate BETWEEN its
#: samples; a torch chord read (K fixed points on every chord) closed that
#: and cost the coarser grid's whole saving back (5.4 s against 3.8 s); the
#: kernel's adaptive walk closes it at 3.8 s. The cap was swept 3 / 6 / 10
#: with identical answers and timings -- it only binds inside a wall's
#: reach, where the walk's reads are exactly the ones wanted -- so 10 is
#: kept over cuRobo's 3 for the coverage it buys where it binds.
SWEEP_STEPS = 10

#: Control points held at each end of a motion that stops there. For a clamped
#: cubic, `P0 = P1` makes velocity zero at the end and `P0 = P1 = P2` makes
#: acceleration zero too, so three is exactly "arrive stopped, and stop
#: smoothly". The same count as `freespace.PIN_KNOTS`, now with a structural
#: meaning rather than a band of repeated waypoints.
PIN_CONTROL = 3

INNER_STEPS = 80
OUTER_STEPS = 3
ADAM_LR = 5e-3

#: How close to a limit `retime` aims. Short of 1.0 because the accept test
#: measures the curve it just solved, and a horizon chosen to land exactly on
#: the limit lands over it as often as under.
RETIME_TARGET = 0.9
#: 5, raised from 3: the batched shortcut changes which corridors the
#: re-solve lands in, and one corpus approach needed a fourth grow to get
#: back under its acceleration limit (left at 1.047 with three).
RETIME_ATTEMPTS = 5

#: Descent budget for RE-solves -- retime's shrunk-horizon passes and the
#: shortcut cycle's re-runs. Half the initial budget, because a re-solve is
#: seeded with the control polygon a converged solve just produced: it is
#: refining an answer, not finding one. Measured on the 22-seam corpus with
#: the full suite as the judge: 40 holds every contract the 80-step re-solve
#: held, and the connecting stage drops by the difference.
RESOLVE_STEPS = 40
#: A shrink is kept only if clearance did not fall by more than this.
RETIME_GIVE_M = 0.001

#: Sampling step for the seed-path shortcut screen, radians of the widest
#: joint. `freespace.SHORTCUT_STEP_RAD`, unchanged.
SHORTCUT_STEP_RAD = 0.05

#: Straighten-then-descend alternations after the first solve. The seed-level
#: prune cannot open the big spans -- at seed time the direct line still
#: skims the part -- so, as the old solver did (`SHORTCUT_CYCLES`), the pass
#: re-runs on the SOLVED path, where descent has already pushed the curve off
#: the part and the straight line through the bow screens clear.
SHORTCUT_CYCLES = 3

#: Points the solved curve is downsampled to before pruning. Enough that a
#: kept interior point can still describe a lift-off; few enough that the
#: screen stays cheap.
SHORTCUT_KNOTS = 41

#: Below this many inner steps a capture cannot pay for its own warm-up and
#: recording. `freespace.CAPTURE_MIN_STEPS`, unchanged.
CAPTURE_MIN_STEPS = 16

#: The solver's working precision. float32, deliberately: the FK and both
#: collision fields already run float32 internally (`q_full` casts to the
#: scene's dtype on the way in), so float64 up here bought no accuracy the
#: downstream could see -- it bought `cutlass d884gemm` kernels, 13% of the
#: step's GPU time on the profile that motivated this, and on Jetson-class
#: hardware (the deployment target) FP64 runs at 1/64 of FP32 with no tensor
#: cores at all. Endpoint positions are DATA and are written back from the
#: float64 spec in `_collect`, so the contract "the ends are exact" is about
#: the problem statement, not the working precision.
SOLVER_DTYPE = torch.float64

#: Seconds a cell is assumed to need to swing an axis across its whole
#: acceleration range. No cell descriptor states a jerk limit, and a hinge
#: against infinity holds nothing, so this is what the jerk bound is built
#: from. Loose on purpose: it is a smoothness backstop, not a drive rating.
JERK_SPAN_S = 0.2


@dataclass
class MoveProblem:
    """One motion to solve: where it starts, where it ends, and what it may
    pass through on the way."""

    kind: str
    start: np.ndarray
    end: np.ndarray
    via: list[np.ndarray] = field(default_factory=list)
    route: np.ndarray | None = None
    pin_start: bool = False
    pin_end: bool = False

    def pinned(self) -> tuple[int, int]:
        return (PIN_CONTROL if self.pin_start else 1,
                PIN_CONTROL if self.pin_end else 1)


@dataclass
class Solved:
    """One answer, sampled on the curve rather than on the control polygon."""

    control: np.ndarray            # (K, dof)
    knots: np.ndarray              # (S, dof)  the trajectory itself
    velocity: np.ndarray           # (S, dof)  rad/s, analytic
    acceleration: np.ndarray       # (S, dof)  rad/s^2, analytic
    jerk: np.ndarray               # (S, dof)  rad/s^3, analytic
    times_s: np.ndarray
    duration_s: float
    clearance_m: float
    env_clearance_m: float
    self_clearance_m: float
    cost_before: float
    cost_after: float
    v_ratio: float
    a_ratio: float
    j_ratio: float
    pin_start: bool = False
    pin_end: bool = False


def _seed_control(problem: MoveProblem, basis: Basis, dof: int) -> np.ndarray:
    """Control points whose curve follows the seed path, by least squares.

    The seed is a polyline -- straight, through a standoff, or a roadmap route
    -- and a B-spline does not interpolate its control polygon, so handing the
    polyline over as control points would pull the curve off it near the bends.
    Fitting instead costs one `lstsq` on a matrix that is already built.

    The end rows are then OVERWRITTEN with the exact endpoints. A clamped
    B-spline begins and ends on its end control points, so writing them is what
    makes the endpoints exact rather than fitted.
    """
    path = [np.asarray(problem.start, dtype=np.float64)]
    if problem.route is not None and len(problem.route) >= 2:
        path = [np.asarray(p, dtype=np.float64) for p in problem.route]
    elif problem.via:
        path = [path[0]] + [np.asarray(v, dtype=np.float64)
                            for v in problem.via]
        path.append(np.asarray(problem.end, dtype=np.float64))
    else:
        path.append(np.asarray(problem.end, dtype=np.float64))
    path = np.stack(path)

    # The polyline, resampled onto the curve's own sample grid by arc length,
    # so the fit is against a path with a comparable parametrization.
    leg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    walked = np.concatenate([[0.0], np.cumsum(leg)])
    if walked[-1] <= 1e-12:
        target = np.repeat(path[:1], basis.n_samples, axis=0)
    else:
        want = np.linspace(0.0, walked[-1], basis.n_samples)
        target = np.stack([np.interp(want, walked, path[:, i])
                           for i in range(dof)], axis=1)

    matrix = basis.position.detach().cpu().numpy()
    control, *_ = np.linalg.lstsq(matrix, target, rcond=None)
    control[0] = problem.start
    control[-1] = problem.end
    return control


class ControlSolver:
    """A batch of motions, solved as one tensor problem.

    Every motion in the batch shares the basis -- same control count, same
    sample count -- so the whole rollout is four `matmul`s and the shapes never
    change, which is what lets the descent be captured as a CUDA graph.
    """

    def __init__(self, problems: list[MoveProblem], limits: Limits, *,
                 device: torch.device, durations: list[float],
                 planner_collision_model: Any = None,
                 pose_check: Any = None,
                 seeds: list[np.ndarray] | None = None,
                 dtype: torch.dtype = SOLVER_DTYPE):
        if not problems:
            raise ValueError("nothing to solve")
        self.problems = problems
        self.device = device
        self.dtype = dtype
        self.dof = len(limits.velocity)
        self.B = len(problems)
        self.basis = Basis(CONTROL_POINTS, SAMPLES, device=device, dtype=dtype)
        self.rates = RateCost(limits)
        self.planner_collision_model = planner_collision_model
        self.pose_check = pose_check

        self.duration = torch.as_tensor(
            [[[d]] for d in durations], dtype=dtype, device=device)

        control = np.stack(seeds if seeds is not None else
                           [_seed_control(p, self.basis, self.dof)
                            for p in problems])
        self.p = torch.tensor(control, dtype=dtype, device=device,
                              requires_grad=True)

        # The pinned rows, as data. A clamped curve already meets its end
        # control points, so pinning ONE end row is the plain clamp every
        # motion has; pinning three is the stop.
        index = torch.arange(CONTROL_POINTS, device=device).view(1, -1)
        head = torch.as_tensor([[p.pinned()[0]] for p in problems],
                               device=device)
        tail = torch.as_tensor([[p.pinned()[1]] for p in problems],
                               device=device)
        self._held = ((index < head) | (index >= CONTROL_POINTS - tail)
                      ).unsqueeze(-1)
        self._head_value = torch.as_tensor(
            np.stack([p.start for p in problems]), dtype=dtype, device=device)
        self._tail_value = torch.as_tensor(
            np.stack([p.end for p in problems]), dtype=dtype, device=device)
        self._is_head = (index < head).unsqueeze(-1)
        self._write_held()

    @torch.no_grad()
    def _write_held(self) -> None:
        """Put the pinned control points back.

        Adam moves every element it is given, so the held rows are restored
        after each step rather than trusted to stay. This is the whole of the
        endpoint contract: there is no separate clamp, and no clip.
        """
        head = self._head_value.unsqueeze(1).expand(-1, CONTROL_POINTS, -1)
        tail = self._tail_value.unsqueeze(1).expand(-1, CONTROL_POINTS, -1)
        self.p.data.copy_(torch.where(
            self._held, torch.where(self._is_head, head, tail), self.p.data))

    def collision_cost(self, *, rho: float | None = None,
                       buffer_weight: float | None = None) -> Any:
        """A cost object sized to THIS solver's batch.

        Built here rather than passed in because the augmented Lagrangian keeps
        one multiplier per constraint per pose, so its shape is the batch's.
        `retime` re-solves a SUBSET of the motions and therefore needs its own;
        handing the original down mismatched by exactly the motions that had
        already finished.

        The multipliers start at zero, as `freespace` does for the same reason:
        a retime that inherited accumulated pressure cut into the part.
        """
        if self.planner_collision_model is None:
            return None
        from weld_motion_planner.robot_cell.collision.cost import (
            DEFAULT_BUFFER_WEIGHT,
            CollisionCost,
        )
        from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
            BUFFER_M,
            BUFFER_SELF_M,
            COLLIDE_RHO,
        )
        # The SAME pressure the old path uses. `rho` at 2000 rather than
        # `COLLIDE_RHO` was measured here to turn two clear transits into
        # penetrating ones (+1.3 mm -> -1.0 mm, +1.8 mm -> -6.6 mm): the
        # augmented Lagrangian is what finishes, and at a thirtieth of the
        # weight it does not.
        rows = self.B * self.basis.n_samples
        env_buffer, self_buffer = self._buffers()
        return CollisionCost(
            scene=self.planner_collision_model, batch=rows,
            margin_m=0.0, self_margin_m=0.0,
            rho=COLLIDE_RHO if rho is None else rho,
            buffer_m=env_buffer, buffer_self_m=self_buffer,
            buffer_weight=(DEFAULT_BUFFER_WEIGHT if buffer_weight is None
                           else buffer_weight))

    @torch.no_grad()
    def _buffers(self) -> tuple[torch.Tensor, torch.Tensor]:
        """Per-sample standoff preference: the full buffer mid-flight, each
        end's OWN clearance at the ends, ramped between.

        `freespace.MoveSolver.buffers` states why, and a flat buffer here was
        measured doing exactly what it warns of: "a uniform 50 mm asks for
        something unachievable at exactly the two places the optimizer cannot
        move, and then pushes on every knot near them to compensate, bending
        the plunge away from the weld it is trying to reach". A motion BEGINS
        at a weld pose millimetres off the work, so 50 mm is not on offer
        there; demanding it bows the approach outward.

        The ends are MEASURED per row, not assumed: a different fixture, gun or
        placement gives a different number.
        """
        from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
            BUFFER_M,
            BUFFER_RAMP_KNOTS,
            BUFFER_SELF_M,
        )
        samples = self.basis.n_samples
        flat = torch.zeros(self.B * samples, dtype=self.dtype, device=self.device)
        if self.planner_collision_model is None or self.pose_check is None:
            return flat + BUFFER_M, flat + BUFFER_SELF_M

        ends = torch.cat([self._head_value, self._tail_value])          # (2B, dof)
        full = self.pose_check.q_full(ends)
        env_end = self.planner_collision_model.clearance(full).min(dim=1).values.to(self.dtype)
        own = self.planner_collision_model.self_clearance(full)
        self_end = (own.min(dim=1).values.to(self.dtype) if own.numel()
                    else torch.full((2 * self.B,), BUFFER_SELF_M,
                                    dtype=self.dtype, device=self.device))

        # The ramp is measured in SAMPLES here rather than knots, so the band
        # covers the same fraction of the motion it did before: `BUFFER_RAMP_KNOTS`
        # of the old solver's knots is that share of its horizon.
        reach_samples = max(BUFFER_RAMP_KNOTS * samples / 40.0, 1.0)
        position = torch.arange(samples, dtype=self.dtype, device=self.device)
        from_start = position.unsqueeze(0).expand(self.B, -1)
        from_end = (samples - 1) - from_start
        nearer_is_start = from_start <= from_end
        reach = torch.clamp(torch.minimum(from_start, from_end) / reach_samples,
                            min=0.0, max=1.0)

        def ramp(at_ends: torch.Tensor, wide: float) -> torch.Tensor:
            near = torch.where(nearer_is_start,
                               at_ends[:self.B].view(-1, 1),
                               at_ends[self.B:].view(-1, 1)).clamp(0.0, wide)
            return (near + (wide - near) * reach).reshape(-1)

        return ramp(env_end, BUFFER_M), ramp(self_end, BUFFER_SELF_M)

    def states(self) -> tuple[torch.Tensor, ...]:
        """`(q, v, a, j)` for the batch, all analytic."""
        return self.basis.states(self.p, self.duration)

    def cost(self, collision: Any = None) -> torch.Tensor:
        """`(B,)`: the rate cost, plus collision on the sampled curve.

        Collision is ONE weighted hinge per field, cuRobo's shape -- no
        multipliers, no outer loop, nothing iterative. `collision` is accepted
        and ignored; it is the augmented Lagrangian this replaced, kept in the
        signature only so callers do not have to change while both solvers
        exist.

        Charged at the SAMPLES, which are the curve, so there is no midpoint
        construction to get wrong.
        """
        q, v, a, j = self.states()
        total = self.rates(q, v, a, j, self.duration)
        if self.planner_collision_model is not None and self.pose_check is not None:
            env, own, chord = self._collision_reads(q)
            total = total + collision_hinge(env.to(total.dtype), W_COLLIDE_ENV)
            if chord is not None:
                total = total + collision_hinge(chord.to(total.dtype),
                                                W_COLLIDE_ENV)
            if own.numel():
                total = total + collision_hinge(own.to(total.dtype),
                                                W_COLLIDE_SELF)
        return total

    def _collision_reads(self, q: torch.Tensor):
        """`(env, self, chord)` per motion, each `(B, N)` least clearances.

        `env` and `self` are the fields at the SAMPLES. `chord` is the
        environment field read along the straight line between each pair of
        consecutive samples' sphere centres, at `SWEEP_STEPS` interior
        points -- the swept term. It is built from the centres the one FK
        produced, so the whole read shares that FK's single autograd node
        and its gradient lands on both samples' control points.

        The chord is the Cartesian path of a sphere between two samples to
        second order (the true path bows off it by `O(dt^2)`); what it buys
        is that a wall between two samples is SEEN by the optimizer instead
        of found by the verifier.
        """
        model = self.planner_collision_model
        samples = q.shape[1]
        full = self.pose_check.q_full(q.reshape(-1, self.dof))
        env_s, own_s, world = model.clearances_and_world(full)
        env = env_s.min(dim=-1).values.view(self.B, samples)
        own = (own_s.min(dim=-1).values.view(self.B, samples)
               if own_s.numel() else own_s)
        if SWEEP_STEPS <= 0 or samples < 2:
            return env, own, None
        chord = model.chord_clearance(world, full, samples, steps=SWEEP_STEPS)
        return env, own, chord.min(dim=-1).values.view(self.B, samples)

    def run(self, collision: Any = None, *, inner_steps: int = INNER_STEPS,
            outer_steps: int = OUTER_STEPS) -> list[Solved]:
        """Monotone Adam descent, the shape `freespace.MoveSolver` uses.

        Kept deliberately: the loop is not what `docs/issues/0015` is about,
        and changing the representation and the loop in one step would leave
        no way to say which one moved a number.
        """
        # `capturable` puts Adam's step count and hyper-parameters on the
        # device, which is what lets one recorded step serve the whole
        # descent. The step itself is launch-bound, not compute-bound --
        # the pattern, the reasons and the measured 5.5 ms -> 1.0 ms are
        # `freespace.MoveSolver._capture`'s, ported.
        capturable = self.device.type == "cuda"
        opt = torch.optim.Adam([self.p], lr=ADAM_LR, capturable=capturable)
        with torch.no_grad():
            best = self.cost(collision).clone()
            before = best.clone()
        # A persistent buffer, not a fresh clone per step: the recorded
        # kernels read THIS tensor, so the revert baseline must live at one
        # address for the life of the graph.
        self._snapshot = self.p.detach().clone()
        steps = max(int(inner_steps), 1)
        graph, warmed = self._capture(opt, collision, best, steps)
        outers = max(int(outer_steps), 1)
        for outer in range(outers):
            # The capture's warm-up runs were real steps, taken out of the
            # FIRST outer's budget and nobody else's.
            for _ in range(steps - (warmed if outer == 0 else 0)):
                if graph is not None:
                    graph.replay()
                else:
                    self._step(opt, collision, best)
            # No multiplier step and NO moment reset: the collision term is
            # a fixed-weight hinge, so an outer step has nothing to update,
            # and the eager loop never reset Adam's moments either -- the
            # outers are simply more steps. (A reset was tried while porting
            # the capture and moved the corpus transit from 1.19 to 1.41 of
            # its geodesic: fresh moments take uncalibrated steps the
            # monotone gate then rejects.) One recorded step therefore
            # serves every outer unchanged.
        return self._collect(before, best, collision)

    def _capture(self, opt: torch.optim.Optimizer, collision: Any,
                 best: torch.Tensor, steps: int) -> tuple[Any, int]:
        """`(graph, warm steps taken)`: one `_step` recorded as a CUDA graph.

        `freespace.MoveSolver._capture`, ported with its contract: the
        warm-up runs on a side stream because capture refuses to start while
        ordinary work is pending, its two runs are REAL steps the caller
        counts, and whatever refuses to record falls back to the eager loop
        with the iterate the warm-up left.

        Everything downstream was already built for this -- the fused FK is
        one graph node, `_SelfGap` allocates no warp grad buffers, and every
        warp launch lands on torch's current stream (`utils.device.
        warp_stream`), without which the kernels would be silently absent
        from the replay.
        """
        if self.device.type != "cuda" or steps < CAPTURE_MIN_STEPS:
            return None, 0
        try:
            stream = torch.cuda.Stream()
            stream.wait_stream(torch.cuda.current_stream())
            with torch.cuda.stream(stream):
                for _ in range(2):
                    self._step(opt, collision, best)
            torch.cuda.current_stream().wait_stream(stream)
            graph = torch.cuda.CUDAGraph()
            with torch.cuda.graph(graph):
                self._step(opt, collision, best)
        except Exception:  # noqa: BLE001 -- whatever refused, eager answers
            return None, 2
        return graph, 2

    def _step(self, opt: torch.optim.Optimizer, collision: Any,
              best: torch.Tensor) -> None:
        """One monotone step, in place on `(p, moments, best, _snapshot)`.

        Every quantity it touches is a pre-existing tensor updated in place
        and every decision a mask -- no device-to-host read anywhere, which
        is what makes this the unit a CUDA graph can replay.
        """
        with torch.no_grad():
            self._snapshot.copy_(self.p.detach())
        opt.zero_grad(set_to_none=False)
        self.cost(collision).sum().backward()
        opt.step()
        self._write_held()
        with torch.no_grad():
            now = self.cost(collision)
            better = now < best
            # Monotone: a row whose cost got worse is reverted, so the
            # stored iterate is never worse than the seed.
            self.p.data.copy_(torch.where(
                better.view(-1, 1, 1), self.p.data, self._snapshot))
            best.copy_(torch.where(better, now, best))

    @torch.no_grad()
    def _collect(self, before: torch.Tensor, after: torch.Tensor,
                 collision: Any) -> list[Solved]:
        q, v, a, j = self.states()
        v_ratio, a_ratio, j_ratio = self.rates.worst_ratios(v, a, j)
        infinite = [float("inf")] * self.B
        env_worst, self_worst = list(infinite), list(infinite)
        if self.planner_collision_model is not None and self.pose_check is not None:
            env, own, chord = self._collision_reads(q)

            def worst(values):
                return values.min(dim=1).values.double().cpu().numpy().tolist()

            # KEPT APART, as `freespace._clearance` keeps them: "the torch is
            # in the plate" and "the forearm is in the wrist" are different
            # failures with different fixes, and a ranking that reads only the
            # environment field scores a folded arm as perfectly clear. The
            # environment's worst includes the chord reads: a motion whose
            # samples are clear and whose chords are not is not clear.
            env_worst = worst(env if chord is None
                              else torch.cat([env, chord], dim=1))
            if own.numel():
                self_worst = worst(own)
        out = []
        for m in range(self.B):
            duration = float(self.duration[m, 0, 0])
            knots = q[m].double().cpu().numpy()
            velocity = v[m].double().cpu().numpy()
            acceleration = a[m].double().cpu().numpy()
            # The ends are DATA -- a weld pose, the reset pose -- and the
            # clamped basis holds the curve on its end control points, so
            # writing the spec's own float64 values here restores exactly
            # what float32 storage rounded, and nothing else.
            knots[0] = self.problems[m].start
            knots[-1] = self.problems[m].end
            # A pinned end's zero velocity and acceleration are CONSTRUCTION
            # -- `PIN_CONTROL` bitwise-identical control points -- but the
            # float32 GEMM does not deliver the cancellation: its FMA keeps
            # the second product exact against the first one's rounding, so
            # `-39*c + 39*c` leaves the ROUNDING RESIDUAL of one product,
            # measured 7.5e-7 rad/s on a real transit whose pinned rows were
            # equal to the bit. What construction guarantees, the report
            # states.
            pin_start, pin_end = self.problems[m].pinned()
            if pin_start >= 2:
                velocity[0] = 0.0
                acceleration[0] = 0.0
            if pin_end >= 2:
                velocity[-1] = 0.0
                acceleration[-1] = 0.0
            out.append(Solved(
                control=self.p[m].double().cpu().numpy(),
                knots=knots,
                velocity=velocity,
                acceleration=acceleration,
                jerk=j[m].double().cpu().numpy(),
                pin_start=pin_start >= 2,
                pin_end=pin_end >= 2,
                times_s=np.linspace(0.0, duration, SAMPLES),
                duration_s=duration,
                clearance_m=float(min(env_worst[m], self_worst[m])),
                env_clearance_m=float(env_worst[m]),
                self_clearance_m=float(self_worst[m]),
                cost_before=float(before[m]),
                cost_after=float(after[m]),
                v_ratio=float(v_ratio[m]),
                a_ratio=float(a_ratio[m]),
                j_ratio=float(j_ratio[m])))
        return out


def retime(solver: ControlSolver, solved: list[Solved],
           *, attempts: int = RETIME_ATTEMPTS, target: float = RETIME_TARGET,
           inner_steps: int = INNER_STEPS) -> tuple[list[Solved], list[list[str]]]:
    """Shrink each horizon until a rate is near its limit, and re-solve on it.

    `_horizon_for`'s closed form prices the STRAIGHT line and pads it, so a
    solved motion sits well inside its limits -- measured on this cell, an
    acceleration ratio of 0.62 where 1.0 was available. That surplus is time
    nobody asked for.

    **No carry floor.** The old solver had to bound the shrink from below
    (`_carry_floor`): its cap was a per-step budget, shrinking the horizon
    shrank the budget, and past a point the re-paced seed could not reach its
    own endpoint -- the shortfall landing as one lunge at the weld. A control
    point is not a step. The endpoints are the end control points and stay
    exact at any horizon, so the only thing a shrink can break is a rate, and
    the accept test asks about rates.

    A re-solve rather than a rescale, for the reason `freespace.retime` gives:
    the horizon enters the derivatives, so a shorter one changes what the cost
    wants, and the optimizer is asked again -- seeded with the control polygon
    it just produced, which is already the right shape.
    """
    notes: list[list[str]] = [[] for _ in solved]

    def wanted(answer: Solved) -> float:
        """The multiplier that brings the binding rate to `target`.

        On a horizon `k` times shorter velocity divides by `k`, acceleration by
        `k^2` and jerk by `k^3`, so each rate gives a demand and the largest
        wins.

        **Not capped at 1.0.** `duration_for` prices the STRAIGHT line, and a
        motion seeded through a standoff or a roadmap route does not fly one:
        measured on this corpus, an approach seeded through its standoff came
        back at 3.37 of the acceleration limit on a horizon that was legal for
        the straight line. With the multiplier capped, such a motion asked for
        no change and stayed illegal -- there was no way to LENGTHEN a clock
        that was too short, which is the job `_horizon_for`'s growth loop did
        for the old representation and nothing here had inherited.
        """
        demands = [answer.v_ratio / max(target, 1e-9),
                   (answer.a_ratio / max(target, 1e-9)) ** 0.5,
                   (answer.j_ratio / max(target, 1e-9)) ** (1.0 / 3.0)]
        return float(max(demands))

    scales = [wanted(s) for s in solved]
    for _ in range(max(int(attempts), 0)):
        # Both directions: below 0.98 there is time to win back, above 1.02 the
        # horizon is too short to be legal at all.
        todo = [m for m, scale in enumerate(scales)
                if scale <= 0.98 or scale > 1.02]
        if not todo:
            break
        faster = ControlSolver(
            [solver.problems[m] for m in todo], solver.rates.limits,
            device=solver.device,
            durations=[solved[m].duration_s * scales[m] for m in todo],
            planner_collision_model=solver.planner_collision_model,
            pose_check=solver.pose_check,
            seeds=[solved[m].control for m in todo], dtype=solver.dtype)
        answers = faster.run(faster.collision_cost(),
                             inner_steps=min(inner_steps, RESOLVE_STEPS),
                             outer_steps=1)

        for slot, m in enumerate(todo):
            fast = answers[slot]
            was, now = solved[m].clearance_m, fast.clearance_m
            refusal = None
            worst_now = max(fast.v_ratio, fast.a_ratio, fast.j_ratio)
            worst_was = max(solved[m].v_ratio, solved[m].a_ratio,
                            solved[m].j_ratio)
            # A GROW is judged on whether it improved the ratio, not on whether
            # it reached legality in one step: a motion at 3.37 may need two.
            if worst_now > 1.0 and worst_now >= worst_was:
                refusal = f"it would reach {worst_now:.2f} of a rate limit"
            elif np.isfinite(was) and now < was - RETIME_GIVE_M:
                # NO WORSE, rather than non-negative. Requiring `now >= 0`
                # refuses every shrink on a motion that starts out penetrating,
                # however much the shrink improved it -- and "already in
                # collision" is a fact about the endpoints the caller gave, not
                # something a horizon can fix. The verdict is where that is
                # judged; here the only question is whether shrinking made it
                # worse.
                refusal = (f"it would leave {now * 1000:.1f} mm where "
                           f"{was * 1000:.1f} mm was clear")
            if refusal is not None:
                # Bisect toward the horizon it arrived with rather than give
                # up: the answer to "too short" is a longer one, not the
                # original.
                notes[m].append(f"refused {fast.duration_s:.2f} s: {refusal}")
                # Bisect toward "leave it alone" from whichever side it came.
                scales[m] = 0.5 * (scales[m] + 1.0)
                continue
            notes[m].append(
                f"retimed {solved[m].duration_s:.2f} s -> {fast.duration_s:.2f} s")
            solved[m] = fast
            scales[m] = wanted(fast)
    return solved, notes


class _AsSolved:
    """A `control_solver.Solved` wearing `freespace._Solved`'s field names.

    `plan_moves` reads `knots`, `qdot`, `times_s`, the clearances and the cost
    pair, and assembles a `Move` from them. Adapting here keeps that assembly,
    the verifier call and every downstream reader untouched, so switching
    representation does not become a rewrite of the reporting path.

    `knots` are now the CURVE at `SAMPLES` points rather than 16 waypoints an
    interpolant is threaded through, and `qdot` is the curve's own analytic
    velocity rather than a centred difference. That is the point: the verifier
    differences what it is handed, so handing it the curve is what makes
    `check_limits` see what the arm actually does.
    """

    __slots__ = ("knots", "qdot", "times_s", "clearance_m", "env_clearance_m",
                 "self_clearance_m", "cost_before", "cost_after", "seed",
                 "cap_excess_rad", "span_times_s")

    def __init__(self, answer: Solved):
        # The WIRE: the curve re-evaluated at `WIRE_DT_S`, in float64 on the
        # CPU. Exact -- the basis is evaluated at the wire's own times, not
        # interpolated from the cost grid -- and independent of the solve's
        # sample count, so the certificate's density follows the motion's
        # duration rather than the optimizer's grid.
        duration = float(answer.duration_s)
        count = max(int(round(duration / WIRE_DT_S)), WIRE_MIN_INTERVALS) + 1
        basis = Basis(CONTROL_POINTS, count, device="cpu", dtype=torch.float64)
        control = torch.as_tensor(answer.control, dtype=torch.float64)[None]
        q, v, _a, _j = basis.states(
            control, torch.tensor([[[duration]]], dtype=torch.float64))
        knots = q[0].numpy().copy()
        qdot = v[0].numpy().copy()
        # The same two contracts `_collect` keeps on the cost grid: the ends
        # are the spec's own data, and a pinned end's zero velocity is
        # construction, stated rather than left to float64's last bit.
        knots[0], knots[-1] = answer.knots[0], answer.knots[-1]
        if answer.pin_start:
            qdot[0] = 0.0
        if answer.pin_end:
            qdot[-1] = 0.0
        self.knots = knots
        self.qdot = qdot
        self.times_s = np.linspace(0.0, duration, count)
        # Where the curve's cubic pieces meet, in seconds: between two wire
        # samples inside one span the wire's Hermite IS the curve, which is
        # what lets the verifier certify spans instead of wire intervals
        # (`freespace.certified_samples`).
        self.span_times_s = np.unique(
            clamped_knots(DEGREE, CONTROL_POINTS)[DEGREE: CONTROL_POINTS + 1]) * duration
        self.clearance_m = answer.clearance_m
        self.env_clearance_m = answer.env_clearance_m
        self.self_clearance_m = answer.self_clearance_m
        self.cost_before = answer.cost_before
        self.cost_after = answer.cost_after
        self.seed = "control"
        self.cap_excess_rad = 0.0


def _screen_segments(segments: list[tuple[np.ndarray, np.ndarray]],
                     model: Any, checker: Any, device: Any) -> list[bool]:
    """Which straight joint-space segments screen clear, in ONE call.

    The legacy screen's shape (`MoveSolver._clear_spans`) with ONE changed
    number: the gap must clear `COLLIDE_ACTIVATION_M`, not zero. Zero was
    right for the augmented Lagrangian, which charged penetration only; this
    solver's hinge charges everything inside the activation skin, so a
    shortcut accepted at `gap > 0` hands the optimizer a corridor the cost
    itself fights -- measured, a corpus approach pruned into a ~2 mm corridor
    sat at collision cost 26,000, could not descend out of it, and came back
    at 1.047 of its acceleration limit with every retime grow refused. The
    screen's acceptance and the cost's free region must be the same region.

    Batched: the first version screened one segment at a time, each a kernel
    launch plus a device-to-host read -- 8.3 s of a 10.5 s connecting stage
    on the 22-seam corpus. One clearance call and one read per LEVEL now.
    """
    points: list[np.ndarray] = []
    counts: list[int] = []
    for a, b in segments:
        reach = float(np.abs(b - a).max())
        n = int(np.clip(round(reach / SHORTCUT_STEP_RAD), 2, 512))
        t = np.linspace(0.0, 1.0, n + 2)[1:-1, None]
        points.append(a[None] * (1.0 - t) + b[None] * t)
        counts.append(n)
    flat = torch.as_tensor(np.concatenate(points), dtype=SOLVER_DTYPE,
                           device=device)
    full = checker.q_full(flat)
    env = model.clearance(full).min(dim=1).values
    own = model.self_clearance(full)
    mine = (own.min(dim=1).values if own.numel()
            else torch.full_like(env, float("inf")))
    gap = torch.minimum(env, mine).cpu().numpy()
    from weld_motion_planner.planner.trajectory_optimization.engine.rates import (
        COLLIDE_ACTIVATION_M,
    )
    out, at = [], 0
    for n in counts:
        out.append(bool(gap[at:at + n].min() > COLLIDE_ACTIVATION_M))
        at += n
    return out


def _prune_seed_paths(paths: list[list[np.ndarray]], model: Any, checker: Any,
                      device: Any) -> list[list[np.ndarray]]:
    """Every path's shortcut, coarse-to-fine, one screen per LEVEL.

    `MoveSolver.shortcut`'s span schedule on polylines: at each level the
    candidate spans are one width, overlapping by half, over every path at
    once; an accepted span DELETES its interior points (a polyline prune,
    where the knot version straightened in place); accepts within a level
    are taken left to right, never two sharing an interior. Halving the
    width visits `O(L log L)` candidates against all-pairs' `O(L^2)`, and
    the whole level -- every span of every path -- is one `_screen_segments`
    call.

    An empty scene collapses every path to its straight line, as the legacy
    pass did.
    """
    if model is None or checker is None:
        return [[p[0], p[-1]] for p in paths]
    pruned = [list(p) for p in paths]
    width = max(len(p) for p in pruned) - 1
    while width >= 2:
        spans: list[tuple[int, int, int]] = []
        for k, path in enumerate(pruned):
            L = len(path)
            step = max(1, width // 2)
            starts = list(range(0, L - width, step))
            last = L - 1 - width
            if last >= 0 and (not starts or starts[-1] != last):
                starts.append(last)
            spans += [(k, i, i + width) for i in starts if width >= 2]
        if spans:
            keep = _screen_segments(
                [(pruned[k][i], pruned[k][j]) for k, i, j in spans],
                model, checker, device)
            # Left to right per path, never two spans sharing an interior:
            # a span was screened against the path as it stood.
            cut: dict[int, list[tuple[int, int]]] = {}
            claimed: dict[int, int] = {}
            for (k, i, j), ok in zip(spans, keep):
                if ok and i >= claimed.get(k, 0):
                    cut.setdefault(k, []).append((i, j))
                    claimed[k] = j
            for k, ranges in cut.items():
                path = pruned[k]
                out, at = [], 0
                for i, j in ranges:
                    out += path[at:i + 1]
                    at = j
                pruned[k] = out + path[at:]
        width = max(2, width // 2) if width > 2 else 1
    return pruned


def _prune_seed_path(points: list[np.ndarray], model: Any, checker: Any,
                     device: Any) -> list[np.ndarray]:
    """One path's shortcut -- `_prune_seed_paths` of a batch of one, kept
    because the docstring below is the pass's contract.

    This is the straightening pass the old solver ran (`MoveSolver.shortcut`,
    itself cuRobo's `PathPruner` on knots) and this solver had not inherited
    -- and its absence is exactly what the side-by-side showed. The transit
    between `bracket_b_2x`'s two welds is a wrist reorientation: cuRobo flies
    it in 72 mm of tool travel, this solver flew 785 mm, because the seed was
    forced through both standoff vias and `W_TRAVEL` cannot pull a path out
    of the collision hinge's basin. cuRobo has no task-space term either --
    its short tool path is what a short JOINT path looks like on a wrist move
    -- so the fix is the same pruning it does, not a new cost.

    The screen keeps this honest where it matters: an approach whose direct
    line dives through the part (`docs/issues/0009`) keeps its standoff,
    because that line does not come back clear.
    """
    return _prune_seed_paths([points], model, checker, device)[0]


def _shortcut_cycle(problems: list[MoveProblem], solved: list[Solved],
                    limits: Limits, device: Any, planner_collision_model: Any,
                    pose_check: Any, vmax: np.ndarray, amax: np.ndarray,
                    *, inner_steps: int, outer_steps: int,
                    retime_attempts: int) -> tuple[list[Solved], list[list[str]]]:
    """One straighten-then-descend alternation on already-solved motions.

    The seed-level prune runs before anything has lifted the path off the
    part, so the spans that matter -- the bow a standoff via forces -- never
    screen clear there. Here the SOLVED curve is downsampled, pruned with the
    same screen, and any motion whose pruned polyline is meaningfully shorter
    is re-solved along it. The re-solve is kept only if it is still legal and
    actually shorter; a motion that was already straight keeps its answer and
    pays nothing.

    Measured on the two-weld `bracket_b_2x` program this exists for: the
    transit between the welds is a wrist reorientation cuRobo flies at 1.84
    rad of joint travel; seeded through both standoffs this solver flew 3.64
    rad, and one alternation is what closes that gap.
    """
    notes: list[list[str]] = [[] for _ in solved]
    candidates: list[int] = []
    replacements: list[MoveProblem] = []
    all_paths = []
    for answer in solved:
        take = np.linspace(0, len(answer.knots) - 1, SHORTCUT_KNOTS)
        all_paths.append([answer.knots[int(round(k))] for k in take])
    all_pruned = _prune_seed_paths(all_paths, planner_collision_model,
                                   pose_check, device)
    for index, (problem, answer) in enumerate(zip(problems, solved)):
        path, pruned = all_paths[index], all_pruned[index]
        before = float(np.abs(np.diff(np.stack(path), axis=0)).sum())
        after = float(np.abs(np.diff(np.stack(pruned), axis=0)).sum())
        if after < before * 0.95:
            candidates.append(index)
            replacements.append(MoveProblem(
                kind=problem.kind, start=problem.start, end=problem.end,
                via=[np.asarray(q, dtype=np.float64) for q in pruned[1:-1]],
                route=None, pin_start=problem.pin_start,
                pin_end=problem.pin_end))
            notes[index].append(
                f"shortcut: seed path {before:.2f} -> {after:.2f} rad")
    if not candidates:
        return solved, notes

    from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
        duration_for,
    )

    def leg_time(problem: MoveProblem) -> float:
        path = np.stack([problem.start, *problem.via, problem.end])
        return sum(duration_for(leg, vmax, amax)
                   for leg in np.abs(np.diff(path, axis=0)))

    solver = ControlSolver(replacements, limits, device=device,
                           durations=[leg_time(p) for p in replacements],
                           planner_collision_model=planner_collision_model,
                           pose_check=pose_check)
    redone = solver.run(solver.collision_cost(),
                        inner_steps=min(inner_steps, RESOLVE_STEPS),
                        outer_steps=1)
    redone, renotes = retime(solver, redone, attempts=retime_attempts,
                             inner_steps=min(inner_steps, RESOLVE_STEPS))
    for slot, index in enumerate(candidates):
        new, old = redone[slot], solved[index]
        legal = max(new.v_ratio, new.a_ratio, new.j_ratio) <= 1.0
        # And NO WORSE in clearance -- `retime`'s rule, which this cycle
        # did not have. Without it a shortcut re-solve that shaved the
        # corner into the part was accepted for being shorter and legal in
        # its rates: measured on the 22-seam corpus, the transit
        # seam_0011 -> seam_0016 came back with its own env clearance at
        # -7 mm and the T2 verifier failing it, while the cycle's note
        # said "shortcut kept".
        clear = (not np.isfinite(old.clearance_m)
                 or new.clearance_m >= old.clearance_m - RETIME_GIVE_M)
        old_l1 = float(np.abs(np.diff(old.knots, axis=0)).sum())
        new_l1 = float(np.abs(np.diff(new.knots, axis=0)).sum())
        if legal and clear and new_l1 < old_l1:
            solved[index] = new
            notes[index] += renotes[slot]
            notes[index].append(
                f"shortcut kept: joint path {old_l1:.2f} -> {new_l1:.2f} rad")
        else:
            notes[index].append(
                "shortcut dropped: re-solve was not better"
                + ("" if clear else f" (clearance {new.clearance_m * 1000:.1f} mm"
                                    f" < {old.clearance_m * 1000:.1f} mm)"))
    return solved, notes


def solve_batch(request: Any, planner_collision_model: Any, device: Any, *,
                specs: list[Any], held: dict[str, float], cell: Any = None,
                inner_steps: int = INNER_STEPS, outer_steps: int = OUTER_STEPS,
                retime_attempts: int = RETIME_ATTEMPTS
                ) -> tuple[list[Solved], list[list[str]]]:
    """`freespace.plan_moves`'s solve, on control points.

    Takes the same `MoveSpec` list the old path takes and returns solved
    curves, so the caller's verification, reporting and `Move` assembly are
    untouched. The horizon still comes from `_horizon_for`'s closed form: a
    hinge cannot rescue an infeasible clock -- measured, a horizon picked from
    velocity alone left the acceleration ratio at 2.92 and no amount of
    descending fixed it, because no curve over that horizon is legal.
    """
    from weld_motion_planner.planner.trajectory_optimization.engine.freespace import (
        duration_for,
        real_position_limits,
    )
    from weld_motion_planner.robot_cell.robot_cell import RobotCell

    cell = (cell or RobotCell.of(request.cell, device=device)).to(device)
    names = list(cell.moved_axes)
    limit_doc = getattr(cell.profile, "limits", None) or {}
    vmax = np.array([float((limit_doc.get(n) or {}).get("velocity_rad_s") or 1e9)
                     for n in names])
    amax = np.array([float((limit_doc.get(n) or {}).get("acceleration_rad_s2") or 1e9)
                     for n in names])
    lower, upper = real_position_limits(request, names)

    def tensor(values, dtype=SOLVER_DTYPE):
        return torch.as_tensor(values, dtype=dtype, device=device)

    limits = Limits(
        position_lower=tensor(lower), position_upper=tensor(upper),
        velocity=tensor(vmax), acceleration=tensor(amax),
        # No cell states a jerk limit, so it is derived: the jerk that changes
        # acceleration across its whole range within `JERK_SPAN_S`. Stated
        # rather than left at infinity so the jerk hinge has something to hold.
        jerk=tensor(amax * 2.0 / JERK_SPAN_S))

    pose_check = (None if planner_collision_model is None else
                  planner_collision_model.checker(names, held))

    problems = [MoveProblem(kind=s.kind, start=np.asarray(s.start, float),
                            end=np.asarray(s.end, float), via=list(s.via or []),
                            route=s.route, pin_start=s.pin_start,
                            pin_end=s.pin_end) for s in specs]
    seed_paths = []
    for problem in problems:
        if problem.route is not None and len(problem.route) >= 2:
            seed_paths.append([np.asarray(q, dtype=np.float64)
                               for q in problem.route])
        else:
            seed_paths.append([problem.start,
                               *(np.asarray(v, dtype=np.float64)
                                 for v in problem.via),
                               problem.end])
    pruned_paths = _prune_seed_paths(seed_paths, planner_collision_model,
                                     pose_check, device)

    # Judged on the CURVE, not the polyline. The screen certifies every
    # straight segment of the pruned polyline, but the seed the solver flies
    # is a 16-control-point least-squares fit, and with a standoff via gone
    # the fit ROUNDS the corner it protected -- measured on the corpus
    # approach, a pruned polyline clear to 7.5 mm fitted to a seed 73 mm
    # inside the part, and no descent recovers from that basin. So each
    # move's pruning is accepted only if the fitted seed's own cost says the
    # optimizer would rather start there; the fit and one cost evaluation
    # per variant is what that costs.
    def seed_cost(paths: list[list[np.ndarray]]) -> np.ndarray:
        variant = []
        for problem, path in zip(problems, paths):
            variant.append(MoveProblem(
                kind=problem.kind, start=problem.start, end=problem.end,
                via=[np.asarray(q) for q in path[1:-1]], route=None,
                pin_start=problem.pin_start, pin_end=problem.pin_end))
        legs = [sum(duration_for(leg, vmax, amax)
                    for leg in np.abs(np.diff(np.stack(path), axis=0)))
                for path in paths]
        probe = ControlSolver(variant, limits, device=device, durations=legs,
                              planner_collision_model=planner_collision_model,
                              pose_check=pose_check)
        with torch.no_grad():
            return probe.cost(None).double().cpu().numpy()

    kept = seed_paths
    if any(len(a) != len(b) for a, b in zip(seed_paths, pruned_paths)):
        was, now = seed_cost(seed_paths), seed_cost(pruned_paths)
        kept = [pruned if float(c1) <= float(c0) else original
                for original, pruned, c0, c1
                in zip(seed_paths, pruned_paths, was, now)]
    for problem, pruned in zip(problems, kept):
        if problem.route is not None and len(problem.route) >= 2:
            problem.route = pruned if len(pruned) >= 2 else None
        else:
            problem.via = pruned[1:-1]
    def horizon(problem: MoveProblem) -> float:
        """Time for the SEED PATH, not the straight line between the ends.

        `duration_for` prices one leg. A motion seeded through a standoff, or
        round a roadmap route, or across a winding `rewound` chose, flies a
        longer path than its endpoints suggest -- and a horizon short enough to
        make that path illegal cannot be rescued by the rate hinge, only
        penalised. Measured on `bracket_b_2x` rewound, the straight-line
        horizon left the approach at 3.69 of the acceleration limit; summing
        the seed's own legs leaves it at 0.89.

        `retime` still earns the surplus back afterwards, in both directions.
        """
        path = [np.asarray(problem.start, dtype=np.float64)]
        if problem.route is not None and len(problem.route) >= 2:
            path = [np.asarray(p, dtype=np.float64) for p in problem.route]
        elif problem.via:
            path += [np.asarray(v, dtype=np.float64) for v in problem.via]
            path.append(np.asarray(problem.end, dtype=np.float64))
        else:
            path.append(np.asarray(problem.end, dtype=np.float64))
        legs = np.abs(np.diff(np.stack(path), axis=0))
        return sum(duration_for(leg, vmax, amax) for leg in legs)

    durations = [horizon(p) for p in problems]

    solver = ControlSolver(problems, limits, device=device, durations=durations,
                           planner_collision_model=planner_collision_model,
                           pose_check=pose_check)
    solved = solver.run(solver.collision_cost(), inner_steps=inner_steps,
                        outer_steps=outer_steps)
    solved, notes = retime(solver, solved, attempts=retime_attempts,
                           inner_steps=inner_steps)
    for _cycle in range(SHORTCUT_CYCLES):
        solved, cycle_notes = _shortcut_cycle(
            problems, solved, limits, device, planner_collision_model,
            pose_check, vmax, amax, inner_steps=inner_steps,
            outer_steps=outer_steps, retime_attempts=retime_attempts)
        for note, extra in zip(notes, cycle_notes):
            note += extra
        if not any("shortcut kept" in line for n in cycle_notes for line in n):
            break
    return solved, notes


def solve_batch_as_freespace(request: Any, planner_collision_model: Any,
                             device: Any, **kwargs: Any):
    """`solve_batch`, adapted to what `plan_moves` expects."""
    solved, notes = solve_batch(request, planner_collision_model, device,
                                **kwargs)
    return [_AsSolved(s) for s in solved], notes, solved
