#!/usr/bin/env python
"""The collision term the optimizer descends on, and the multipliers that
make it actually reach zero.

## Clearance is a measurement; this is a cost

`scene.py` answers how far every sphere is from everything. That number
cannot be minimised directly -- doing so tells the arm to run away from the
workpiece it is supposed to be welding. What the optimizer needs is a term
that is exactly zero when the pose is safe and speaks up only when it is
not:

    violation = relu(margin - clearance)
    cost      = violation^2

Three decisions in that.

**The margin exists so the term has a gradient BEFORE contact.** Without it
the cost activates on touch, and everywhere else its derivative is exactly
zero -- the optimizer gets no signal until it is already inside the floor,
and is then climbing out of a hole rather than steering around one.

**The hinge keeps it local.** Measured on real poses, 27 of 300 spheres sit
inside a 50 mm margin. The other 273 contribute nothing to the value and
nothing to the gradient, which is also why the self-collision backward can
skip most of its atomics.

**The square makes it C1.** `relu(u)` alone has a kink at zero -- the
derivative jumps from 0 to -1 and a gradient method chatters across it.
`d/dx relu(u)^2 = 2 relu(u) u'`, and the `relu(u)` factor vanishes exactly
where the kink was.

## Why a fixed penalty weight does not converge

At the optimum a penalty gradient has to BALANCE the rest of the objective,
so it settles wherever `2w * violation` matches the pull of everything else
-- a violation of roughly `1/w`, never zero. Measured on 32 colliding poses
of `rosie_1400_v3`: at `w = 50` it plateaus at 11.2 mm and stays there, 240
further iterations buying 0.4 mm. Raising `w` to 500 reaches 1.7 mm and
moves the arm 2.03 rad from where it started instead of 1.33 -- feasibility
bought by wrecking every other term.

The augmented Lagrangian gives each constraint its own multiplier that
ACCUMULATES on the ones still violated and stays put on the ones satisfied,
so the pressure builds where it is needed instead of uniformly:

    L    = 0.5 * rho * sum relu(lambda/rho - c)^2        c = clearance - margin
    lambda <- max(0, lambda - rho * c)                   between inner solves

Same feasibility as `w = 500` at `w = 50`'s distortion. With `rho` growing
across outer steps it reaches exactly zero violation, and that is not the
grid's doing -- the floor is identical at 10 mm and 5 mm voxels, so the
field's kinks are not what limits it.

## The constraint and the preference are not the same thing

An AL exists to drive a violation to exactly zero, so what it is pointed at
had better be **satisfiable**. `clearance >= 0` is: a path can be
collision-free, and reaching exactly zero violation is precisely the answer
wanted. `clearance >= 50 mm` is not, and pointing an AL at it is a
mistake this module made and `freespace.py` inherited:

    outer       rho   lam max    lam>0  in margin  in collision  worst viol mm
        0     128.0      10.1      520        520            55          77.63
        5   14073.7     1257.4     128        118             0          46.91

Zero in collision, and still 118 entries under pressure with `rho` grown 281x
and multipliers to 1257. The entries are poses merely INSIDE the buffer, and
some of them are the clamped ends of the link -- weld poses, ~4 mm off the
work, which cannot move by construction and so can never satisfy anything.
An AL chasing them accumulates forever and inflates `rho` for every other
constraint with it.

So the two are separate terms over the SAME clearance query:

- **the constraint** -- `clearance >= margin_m`, under the AL, and free space
  sets `margin_m = 0` because that is its real constraint;
- **the preference** -- `clearance >= margin_m + buffer_m`, a hinge at a FIXED
  `buffer_weight`, quadratic near the buffer and LINEAR past `buffer_knee_m`.
  It is allowed to plateau at "small", because a buffer that ends up 30 mm
  instead of 50 mm has cost nothing; what may not plateau is penetration, and
  that is the other term's job. The linear tail is argued at `buffer`: a
  preference whose pull grows without bound in the deficit will trade the
  constraint away to serve itself, and did.

**And only the AL can finish that job.** Swept on the one link in the corpus
that has to press against something -- `bracket_a_2x`'s first transit -- with
the hard term as a fixed weight and nothing else changed:

    fixed weight      env clearance
            30 000        -2.50 mm
           100 000        -0.90
           300 000        -0.41
         1 000 000        -0.12
         3 000 000        -0.06

(The fixed-weight variant needs no separate code: `value` with
`step_multipliers` never called IS the squared hinge at weight `rho / 2`.)

A hundredfold in weight, three orders of magnitude in violation, and it never
crosses zero -- which is the plateau argued above, seen from the other side. A
fixed penalty settles where its gradient BALANCES the objective, and at that
point the constraint is still active by construction. Reaching zero needs a
multiplier to carry the balance instead, which is what the AL is.

The division of labour that comes out of this is worth stating, because the
weights look mismatched until it is: the buffer does the heavy lifting, since
anything in collision is also deep inside the buffer, and the AL's job is the
last fraction of a millimetre -- the part no weight can buy. Which is also why
free space raises `rho` well above this module's default: at 50 the AL's opening
weight is 25 against the buffer's 20 000, so a penetration would be priced
below a standoff. `freespace.COLLIDE_RHO` has that measurement.

cuRobo takes the other branch: `gradient_trajopt.yml` gives collision a fixed
weight of 1e5 at an `activation_distance` of 2.5 mm and no multipliers at all.
Its narrow activation is what makes that work; nothing here contradicts it.

## What is not here

The seam constraint, which is handled by projection and not by cost, and
the bands, which are clipped. This is the soft half.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from dataclasses import field as dataclass_field
from typing import Any

import torch

from weld_motion_planner.robot_cell.collision.scene import Scene

DEFAULT_MARGIN_M = 0.05
DEFAULT_SELF_MARGIN_M = 0.02
DEFAULT_RHO = 50.0
DEFAULT_RHO_GROWTH = 1.6

# The buffer's fixed weight, and where its quadratic gives way to a linear
# tail. `buffer` argues the tail; the knee is 10 mm because that is the scale at
# which standoff stops being a preference and starts being the constraint's
# business -- inside 10 mm the two terms are pulling the same way anyway.
#
# The weight is what it is because two links in the corpus pull opposite ways
# and only one setting serves both. `bracket_b_2x`'s approach needs standoff
# preferred STRONGLY, since at margin zero grazing the work is legal and the
# travel term will shave it; `bracket_a_2x`'s tight transit has no 50 mm
# available anywhere and a strong SQUARED buffer drove it into the wall. Swept
# with the linear tail, worst clearance of each:
#
#     buffer weight   knee    b_2x approach   a_2x transit
#             1 000   10 mm         3.92 mm       -0.15 mm
#             5 000   10 mm         3.92           0.10
#            20 000   10 mm         3.92           2.36
#             5 000   50 mm         3.92          -0.26
#
# The tail is what makes a single answer possible: raising the weight buys force
# near the buffer without adding any where the deficit is hopeless. At 50 mm of
# knee -- effectively no tail -- the tight link fails again at the same weight
# that clears it at 10.
DEFAULT_BUFFER_WEIGHT = 20000.0
DEFAULT_BUFFER_KNEE_M = 0.01


class CostError(ValueError):
    """A cost this refuses to build, with the reason."""


@dataclass
class CollisionCost:
    """Augmented Lagrangian on the scene's clearances.

    Multipliers are per (pose, constraint) and live here rather than in the
    optimizer, because they are state of the CONSTRAINT and not of the
    search: an outer step updates them, an inner step reads them.
    """

    scene: Scene
    batch: int
    margin_m: float = DEFAULT_MARGIN_M
    self_margin_m: float = DEFAULT_SELF_MARGIN_M
    rho: float = DEFAULT_RHO
    rho_growth: float = DEFAULT_RHO_GROWTH
    # The preference, for `hinge` only. Scalars, or `(batch,)` tensors when the
    # buffer is not the same everywhere -- which it is not near a weld.
    buffer_m: Any = 0.0
    buffer_self_m: Any = 0.0
    buffer_knee_m: float = DEFAULT_BUFFER_KNEE_M
    # ZERO by default, so the constraint is all a caller gets unless it asks
    # for the preference. `DEFAULT_BUFFER_WEIGHT` is what free space passes;
    # making it this field's default silently added a second hinge to M5 and to
    # this module's own self-test, both of which want the AL alone.
    buffer_weight: float = 0.0
    # The SWEPT read. When `sweep_samples` is set, the poses handed in are
    # laid out motion-major in runs of that many consecutive samples, and a
    # sphere's environment clearance at a sample is the least of the field
    # there and along the chords to its neighbouring samples, at most
    # `sweep_steps` reads per half chord (`scene.chord_clearance`). The
    # multipliers then act on the least value -- a constraint on the
    # SEGMENT and not the point, which is what keeps a wall between two
    # samples from being the verifier's find. Self clearance stays per pose.
    sweep_samples: int = 0
    sweep_steps: int = 0
    lam_env: torch.Tensor = dataclass_field(default=None)  # type: ignore[assignment]
    lam_self: torch.Tensor = dataclass_field(default=None)  # type: ignore[assignment]

    def __post_init__(self) -> None:
        device = self.scene.radii.device
        if self.lam_env is None:
            self.lam_env = torch.zeros(self.batch, self.scene.num_spheres, device=device)
        if self.lam_self is None:
            pairs = 0 if self.scene.self_pairs is None else int(self.scene.self_pairs.shape[0])
            self.lam_self = torch.zeros(self.batch, pairs, device=device)
        # A device scalar, not a Python float, and updated IN PLACE like the
        # multipliers: a float would be baked into a CUDA graph as a constant,
        # and the recorded loop would keep the outer step's old rho forever.
        if not torch.is_tensor(self.rho):
            self.rho = torch.tensor(float(self.rho), device=device)

    def constraints(self, q: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """`c >= 0` form: clearance beyond the margin, env and self.

        Through `scene.clearances`, one fk pass for the pair: separate calls
        pay the kinematics twice and, recorded into a CUDA graph, lose one
        branch's gradient to the fused FK's shared scratch -- that method's
        docstring has the measurement. Swept when `sweep_samples` says so:
        the chord read shares that one fk node through the world centres.
        """
        if self.sweep_samples > 1 and self.sweep_steps > 0:
            env, own, world = self.scene.clearances_and_world(q)
            chord = self.scene.chord_clearance(world, q, self.sweep_samples,
                                               steps=self.sweep_steps)
            env = torch.minimum(env, chord.to(env.dtype))
        else:
            env, own = self.scene.clearances(q)
        return env - self.margin_m, own - self.self_margin_m

    def value(self, q: torch.Tensor) -> torch.Tensor:
        """(B,) the augmented Lagrangian, plus the buffer if one is asked for.

        Per pose and not summed, so a batch of independent trajectories does
        not have its gradients mixed by the reduction.

        The AL is on `margin_m` and the buffer is a fixed hinge `buffer_m`
        beyond it -- the module docstring argues why they are not the same
        machine. `buffer_weight` defaults to zero, so a caller that wants only
        the constraint gets only the constraint and nothing about M5 changed.
        """
        env, own = self.constraints(q)
        total = 0.5 * self.rho * torch.relu(self.lam_env / self.rho - env).pow(2).sum(dim=-1)
        if own.numel():
            total = total + 0.5 * self.rho * torch.relu(
                self.lam_self / self.rho - own).pow(2).sum(dim=-1)
        return total + self.buffer(env, own)

    def buffer(self, env: torch.Tensor, own: torch.Tensor) -> torch.Tensor:
        """(B,) the standoff preference alone, at fixed weight and BOUNDED pull.

        `buffer_m` may be a `(B,)` tensor, which is how a weld pose gets a
        narrower buffer than mid-flight: the arm is millimetres off the work
        there because it is welding, and a buffer wider than the gun's own
        standoff asks the optimizer to leave a place it is required to be.

        **Quadratic near the buffer, LINEAR beyond `buffer_knee_m`.** A plain
        squared hinge has a gradient proportional to the deficit, so a pose 50
        mm short of the buffer pulls fifty times harder than one 1 mm short --
        which is backwards for a preference. A large deficit means there is no
        room here, not that the optimizer should push harder; and pushing
        harder is exactly what it did. Measured on `bracket_a_2x`'s first
        transit, a link with no 50 mm of clearance available anywhere along it:

            hard AL at rho 60 000, buffer weight 1000     -0.53 mm
            hard AL at rho 60 000, buffer OFF             +0.02 mm

        At -0.5 mm of penetration the squared buffer is worth 2.5 against the
        AL's 0.0075, so the preference outvoted the constraint by ~300x and held
        the arm inside the work in order to be further from it somewhere else.
        The linear tail caps the buffer's gradient at `2 w * knee` and leaves the
        constraint the last word, which is the whole point of having two terms.

        The knee is where a Huber loss puts it and for the same reason. Below it
        the term is `C1` and behaves like the squared hinge `cost.py`'s
        docstring argues for; above it, it is a constant pressure.
        """
        if not self.buffer_weight:
            return torch.zeros(env.shape[0], dtype=env.dtype, device=env.device)
        total = self._huber(self._as_column(self.buffer_m, env) - env)
        if own.numel():
            total = total + self._huber(
                self._as_column(self.buffer_self_m, own) - own)
        return self.buffer_weight * total

    def _huber(self, deficit: torch.Tensor) -> torch.Tensor:
        """`sum` over constraints of the quadratic-then-linear hinge."""
        short = torch.relu(deficit)
        knee = max(float(self.buffer_knee_m), 1e-9)
        near = torch.clamp(short, max=knee)
        return (near.pow(2) + 2.0 * knee * (short - near)).sum(dim=-1)

    @staticmethod
    def _as_column(value: Any, like: torch.Tensor) -> torch.Tensor:
        """A scalar or a `(B,)` buffer, shaped to broadcast over constraints."""
        if isinstance(value, torch.Tensor):
            return value.to(like.dtype).to(like.device).view(-1, 1)
        return torch.as_tensor(float(value), dtype=like.dtype, device=like.device)

    def penalty(self, q: torch.Tensor) -> torch.Tensor:
        """(B,) the plain squared hinge, for comparison and for a first pass
        before any multiplier has accumulated."""
        env, own = self.constraints(q)
        total = torch.relu(-env).pow(2).sum(dim=-1)
        if own.numel():
            total = total + torch.relu(-own).pow(2).sum(dim=-1)
        return total

    @torch.no_grad()
    def step_multipliers(self, q: torch.Tensor, *, grow: bool = True) -> None:
        """One outer step: raise the multipliers on constraints still violated.

        `max(0, ...)` is what keeps this an INEQUALITY: a constraint with
        slack pushes its multiplier down to zero and then stops, rather than
        going negative and pulling the pose back toward the obstacle.
        """
        env, own = self.constraints(q)
        # In place, so a CUDA graph that recorded reads of these tensors sees
        # the outer step's update rather than the storage it was born with.
        self.lam_env.copy_(torch.clamp(self.lam_env - self.rho * env, min=0.0))
        if own.numel():
            self.lam_self.copy_(torch.clamp(self.lam_self - self.rho * own, min=0.0))
        if grow:
            self.rho *= self.rho_growth

    @torch.no_grad()
    def report(self, q: torch.Tensor) -> dict[str, Any]:
        env, own = self.constraints(q)
        out: dict[str, Any] = {
            "rho": round(float(self.rho), 3),
            "env_worst_violation_mm": round(float(torch.relu(-env).max()) * 1000.0, 3),
            "env_active": int((env < 0).sum()),
            "env_in_collision": int((env + self.margin_m < 0).sum()),
        }
        if own.numel():
            out.update({
                "self_worst_violation_mm": round(float(torch.relu(-own).max()) * 1000.0, 3),
                "self_active": int((own < 0).sum()),
                "self_in_collision": int((own + self.self_margin_m < 0).sum()),
            })
        return out


def self_test() -> int:
    if not torch.cuda.is_available():
        print("cost: no CUDA; the scene is a GPU path, nothing to check")
        return 0
    import numpy as np

    from weld_motion_planner.robot_cell.collision import scene as scenemod

    device = torch.device("cuda", torch.cuda.current_device())
    cell = scenemod._demo_cell()
    model = {
        "link_1": [((0.15, 0.0, 0.0), 0.06)],
        "link_2": [((0.10, 0.0, 0.0), 0.05), ((0.25, 0.0, 0.0), 0.05)],
        "link_3": [((0.10, 0.0, 0.0), 0.05)],
    }
    scene = scenemod.build(
        cell, model, device=device,
        placement={"anchor_xyz_m": [0, 0, 0], "offset_xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
        part=scenemod.box_mesh([-0.08, -0.08, 0.0], [0.08, 0.08, 0.16]),
        fixtures=scenemod.box_mesh([-0.25, -0.25, -0.04], [0.25, 0.25, 0.0]),
        env=scenemod.box_mesh([-1.5, -1.5, -0.05], [1.5, 1.5, 0.0]),
        part_voxel_m=0.006, coarse_voxel_m=0.012, margin_m=0.12,
    )
    index = {name: i for i, name in enumerate(scene.spec.axis_names)}

    # Start from poses that are genuinely in trouble: elbows folded so the
    # arm hits itself, which is the constraint with the least slack.
    torch.manual_seed(0)
    batch = 16
    q0 = torch.zeros(batch, scene.spec.num_dof, device=device)
    q0[:, index["J2"]] = float(np.pi) + torch.linspace(-0.25, 0.25, batch, device=device)
    q0[:, index["J3"]] = float(np.pi) + torch.linspace(-0.25, 0.25, batch, device=device)

    def run(kind: str, iters: int = 400, weight: float = 50.0) -> tuple[float, float]:
        cost = CollisionCost(scene, batch=batch, margin_m=0.02, self_margin_m=0.02)
        q = q0.clone().requires_grad_(True)
        opt = torch.optim.Adam([q], lr=0.02)
        for it in range(iters):
            opt.zero_grad()
            stay = ((q - q0) ** 2).sum()
            loss = (weight * cost.penalty(q).sum() if kind == "fixed" else cost.value(q).sum()) + stay
            loss.backward()
            opt.step()
            if kind == "al" and it % 50 == 49:
                cost.step_multipliers(q.detach())
        with torch.no_grad():
            env, own = cost.constraints(q)
            worst = float(torch.relu(-env).max())
            if own.numel():
                worst = max(worst, float(torch.relu(-own).max()))
        return worst * 1000.0, float((q.detach() - q0).norm())

    start = CollisionCost(scene, batch=batch, margin_m=0.02, self_margin_m=0.02)
    print(f"cost: {batch} poses, {scene.num_spheres} spheres, "
          f"{0 if scene.self_pairs is None else int(scene.self_pairs.shape[0])} self pairs")
    print(f"  at the start: {start.report(q0)}")

    fixed_v, fixed_d = run("fixed")
    al_v, al_d = run("al")
    print(f"  fixed penalty w=50:    worst violation {fixed_v:7.3f} mm, moved {fixed_d:.3f} rad")
    print(f"  augmented Lagrangian:  worst violation {al_v:7.3f} mm, moved {al_d:.3f} rad")

    if al_v > 0.05:
        print("cost: FAIL, the augmented Lagrangian did not drive the violation to zero")
        return 1
    if al_v >= fixed_v:
        print("cost: FAIL, AL is no better than a fixed penalty -- the multipliers are "
              "not accumulating, or the outer step never runs")
        return 1

    # The multipliers are what makes the difference, so check they behave:
    # up on the violated, down to zero on the satisfied.
    cost = CollisionCost(scene, batch=batch, margin_m=0.02, self_margin_m=0.02)
    before = cost.lam_env.clone()
    cost.step_multipliers(q0, grow=False)
    env, _ = cost.constraints(q0)
    violated = (env < 0)
    if not bool((cost.lam_env[violated] > before[violated]).all()):
        print("cost: FAIL, a violated constraint's multiplier did not rise")
        return 1
    if not bool((cost.lam_env[~violated] == 0).all()):
        print("cost: FAIL, a satisfied constraint's multiplier went above zero")
        return 1

    print("cost: ok")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    return self_test()


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"cost: {exc}", file=sys.stderr)
        raise SystemExit(1)
