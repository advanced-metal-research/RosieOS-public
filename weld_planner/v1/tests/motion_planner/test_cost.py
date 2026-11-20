"""The collision cost, and the multipliers that make it reach zero.

The claim under test is not "the cost is computed correctly" -- that is
`test_scene.py`'s job -- but that the augmented Lagrangian does the thing it
was chosen for. A fixed penalty converges to a violation of roughly `1/w`
and stops; AL keeps pushing on the constraints still violated. If the
multiplier update ever stopped running, or ran without accumulating, every
other test here would still pass and the planner would quietly ship poses
that clip the floor by a millimetre.
"""

from __future__ import annotations

import numpy as np
import pytest
import torch
from weld_motion_planner.robot_cell.collision import scene as scenemod
from weld_motion_planner.robot_cell.collision.cost import CollisionCost

pytestmark = pytest.mark.skipif(
    not torch.cuda.is_available(), reason="the scene is a GPU path"
)

MODEL = {
    "link_1": [((0.15, 0.0, 0.0), 0.06)],
    "link_2": [((0.10, 0.0, 0.0), 0.05), ((0.25, 0.0, 0.0), 0.05)],
    "link_3": [((0.10, 0.0, 0.0), 0.05)],
}
BATCH = 12


@pytest.fixture(scope="module")
def scene():
    return scenemod.build(
        scenemod._demo_cell(), MODEL,
        device=torch.device("cuda", torch.cuda.current_device()),
        placement={"anchor_xyz_m": [0, 0, 0], "offset_xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]},
        part=scenemod.box_mesh([-0.08, -0.08, 0.0], [0.08, 0.08, 0.16]),
        fixtures=scenemod.box_mesh([-0.25, -0.25, -0.04], [0.25, 0.25, 0.0]),
        env=scenemod.box_mesh([-1.5, -1.5, -0.05], [1.5, 1.5, 0.0]),
        part_voxel_m=0.008, coarse_voxel_m=0.015, margin_m=0.12,
    )


@pytest.fixture(scope="module")
def folded(scene):
    """Poses that self-collide: both elbows near pi."""
    device = scene.radii.device
    index = {n: i for i, n in enumerate(scene.spec.axis_names)}
    q = torch.zeros(BATCH, scene.spec.num_dof, device=device)
    spread = torch.linspace(-0.25, 0.25, BATCH, device=device)
    q[:, index["J2"]] = float(np.pi) + spread
    q[:, index["J3"]] = float(np.pi) + spread
    return q


def descend(scene, q0, kind: str, iters: int = 400, weight: float = 50.0) -> float:
    cost = CollisionCost(scene, batch=q0.shape[0], margin_m=0.02, self_margin_m=0.02)
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
    return worst


def test_the_start_is_actually_infeasible(scene, folded):
    """Otherwise both methods 'converge' by having nothing to do."""
    cost = CollisionCost(scene, batch=BATCH, margin_m=0.02, self_margin_m=0.02)
    report = cost.report(folded)
    assert report["self_in_collision"] > 0
    assert report["self_worst_violation_mm"] > 10


def test_a_fixed_penalty_plateaus_short_of_feasible(scene, folded):
    """The reason AL is here. At the optimum the penalty gradient balances
    the rest of the objective, so it settles at a violation of roughly 1/w
    and more iterations do not help."""
    assert descend(scene, folded, "fixed") > 1e-3


def test_the_augmented_lagrangian_reaches_feasible(scene, folded):
    assert descend(scene, folded, "al") < 1e-4


def test_multipliers_rise_on_the_violated_and_stay_at_zero_on_the_satisfied(scene, folded):
    """`max(0, ...)` is what keeps this an inequality. Without the clamp a
    constraint with slack would go negative and pull the pose back TOWARD
    the obstacle."""
    cost = CollisionCost(scene, batch=BATCH, margin_m=0.02, self_margin_m=0.02)
    before = cost.lam_env.clone()
    cost.step_multipliers(folded, grow=False)
    env, _ = cost.constraints(folded)
    violated = env < 0
    assert bool((cost.lam_env[violated] > before[violated]).all())
    assert bool((cost.lam_env[~violated] == 0).all())


def test_rho_grows_only_when_asked(scene, folded):
    cost = CollisionCost(scene, batch=BATCH)
    # `float()`, because `rho` is a device tensor updated IN PLACE -- so a
    # CUDA graph that recorded it sees the outer step's growth -- and a bare
    # reference to it is therefore a live view, not a snapshot.
    start = float(cost.rho)
    cost.step_multipliers(folded, grow=False)
    assert float(cost.rho) == pytest.approx(start)
    cost.step_multipliers(folded, grow=True)
    assert float(cost.rho) == pytest.approx(start * cost.rho_growth)


def test_the_cost_is_zero_where_there_is_slack(scene):
    """A hinge that leaked a little everywhere would push the arm away from
    the workpiece it is supposed to be welding.

    Not the zero pose: this demo's table sits inside the arm's reach on
    purpose, so at q = 0 link_3's sphere is INSIDE the part. J2 = -1.4 lifts
    the arm clear, and the slack is asserted before the cost is, so this
    cannot go vacuous if the geometry moves again.
    """
    device = scene.radii.device
    index = {n: i for i, n in enumerate(scene.spec.axis_names)}
    cost = CollisionCost(scene, batch=1, margin_m=0.0005, self_margin_m=0.0005)
    q = torch.zeros(1, scene.spec.num_dof, device=device)
    q[0, index["J2"]] = -1.4
    with torch.no_grad():
        env, own = cost.constraints(q)
        assert float(env.min()) > 0, "this pose is meant to have slack and does not"
        assert float(own.min()) > 0
        assert float(cost.penalty(q)) == 0.0
        assert float(cost.value(q)) == 0.0


def test_the_cost_is_per_pose_not_summed(scene, folded):
    """A batch of independent trajectories must not have its gradients mixed
    by the reduction."""
    cost = CollisionCost(scene, batch=BATCH, margin_m=0.02, self_margin_m=0.02)
    with torch.no_grad():
        assert cost.value(folded).shape == (BATCH,)
        assert cost.penalty(folded).shape == (BATCH,)


def test_the_gradient_reaches_the_positioner(scene, folded):
    """The whole point of the workpiece-frame fields. A detached work
    transform would zero this and nothing else would notice."""
    index = {n: i for i, n in enumerate(scene.spec.axis_names)}
    cost = CollisionCost(scene, batch=BATCH, margin_m=0.30, self_margin_m=0.02)
    q = folded.clone().requires_grad_(True)
    cost.penalty(q).sum().backward()
    assert abs(float(q.grad[:, index["J7"]].abs().max())) > 1e-9
