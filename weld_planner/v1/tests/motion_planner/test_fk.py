"""Forward kinematics, fused, against a reference that is not itself.

A kernel checked only against its own output is checked against nothing, so
everything here compares the Warp path to plain torch on the CPU in float64
-- a different precision, a different device, and a different set of
instructions, which is what makes the agreement mean something.

The gradient is checked against autograd rather than against finite
differences. The Jacobian in the kernel is analytic (`a x (p - o)` per
revolute joint), so the right target is exact differentiation of the
reference, not a difference quotient that would introduce its own error and
blur the comparison.
"""

from __future__ import annotations

import pytest
import torch
from weld_motion_planner.robot_cell.robot import fk

pytestmark = pytest.mark.skipif(
    not torch.cuda.is_available(), reason="the fused kinematics path is a GPU path"
)


def chain_cell() -> dict:
    """Two revolute axes that are not parallel, then a fixed joint.

    Perpendicular axes and a non-trivial rpy on the second origin, so that a
    transposed rotation or a compose order swapped for its inverse cannot
    produce the same answer by symmetry.
    """
    return {
        "frames": {"base": "base", "tool": "c"},
        "links": [{"name": n} for n in ("base", "a", "b", "c")],
        "joints": [
            {"name": "J1", "parent": "base", "child": "a", "type": "revolute",
             "xyz_m": [0.0, 0.0, 0.20], "rpy_rad": [0.0, 0.0, 0.0], "axis_xyz": [0, 0, 1]},
            {"name": "J2", "parent": "a", "child": "b", "type": "revolute",
             "xyz_m": [0.10, 0.0, 0.05], "rpy_rad": [0.0, 0.3, 0.0], "axis_xyz": [0, 1, 0]},
            {"name": "J3", "parent": "b", "child": "c", "type": "fixed",
             "xyz_m": [0.0, 0.04, 0.30], "rpy_rad": [0.1, 0.0, 0.0]},
        ],
    }


LINKS = ["a", "b", "c"]
MODEL = {
    "a": [((0.01, 0.0, 0.02), 0.05), ((0.0, 0.03, 0.0), 0.04)],
    "b": [((0.0, 0.0, 0.10), 0.06)],
    "c": [((0.02, 0.0, 0.0), 0.03), ((0.0, 0.0, 0.05), 0.02)],
}


@pytest.fixture(scope="module")
def built():
    device = torch.device("cuda")
    return fk.make(chain_cell(), LINKS, MODEL, device), device


def test_chain_structure():
    spec = fk.build(chain_cell(), LINKS)
    # The fixed joint is on the chain and is not a degree of freedom. Both
    # halves matter: dropping it would move link `c`, and counting it would
    # give the optimizer an axis the robot does not have.
    assert spec.axis_names == ["J1", "J2"]
    assert spec.num_dof == 2
    assert list(spec.count) == [1, 2, 3]
    assert list(spec.dof) == [0, 0, 1, 0, 1, -1]
    assert spec.max_depth == 3


def test_values_match_the_reference(built):
    (fused, spec, _radii, sphere_link, sphere_local), device = built
    torch.manual_seed(3)
    q = (torch.rand(7, spec.num_dof, device=device) - 0.5) * 3.0
    got = fused(q).detach().double().cpu()
    want = fk.reference(spec, q.detach().double().cpu(), sphere_link, sphere_local)
    assert torch.allclose(got, want, atol=1e-6), float((got - want).abs().max())


def test_gradient_matches_autograd(built):
    (fused, spec, _radii, sphere_link, sphere_local), device = built
    torch.manual_seed(11)
    q = (torch.rand(5, spec.num_dof, device=device) - 0.5) * 3.0
    weight = torch.randn(5, len(sphere_link), 3, device=device)

    qa = q.clone().requires_grad_(True)
    (fused(qa) * weight).sum().backward()

    qb = q.detach().double().cpu().clone().requires_grad_(True)
    (fk.reference(spec, qb, sphere_link, sphere_local) * weight.double().cpu()).sum().backward()

    scale = max(float(qb.grad.abs().max()), 1e-9)
    assert float((qa.grad.double().cpu() - qb.grad).abs().max()) / scale < 1e-4


def test_zero_pose_places_spheres_by_hand(built):
    """One pose worked out on paper, so the reference is not the only witness.

    At q = 0 link `a` sits 200 mm up the z axis with no rotation, so its
    first sphere -- local (10, 0, 20) mm -- lands at (10, 0, 220) mm. If the
    chain were composed parent-after-child, or the origin applied after the
    joint rotation, this is the assertion that would notice.
    """
    (fused, spec, _radii, _link, _local), device = built
    q = torch.zeros(1, spec.num_dof, device=device)
    got = fused(q)[0, 0].detach().cpu()
    assert torch.allclose(got, torch.tensor([0.01, 0.0, 0.22]), atol=1e-6), got


def test_batch_size_can_change(built):
    """The buffers are cached per batch size and must not leak between them."""
    (fused, spec, _radii, sphere_link, sphere_local), device = built
    torch.manual_seed(5)
    for batch in (1, 4, 1, 16):
        q = (torch.rand(batch, spec.num_dof, device=device) - 0.5) * 2.0
        got = fused(q).detach().double().cpu()
        want = fk.reference(spec, q.detach().double().cpu(), sphere_link, sphere_local)
        assert got.shape == (batch, len(sphere_link), 3)
        assert torch.allclose(got, want, atol=1e-6)


def test_a_chain_deeper_than_the_backward_ladder_is_refused():
    """The guard for a trap that would otherwise be silent.

    `fk_backward` folds Jacobian columns into six named registers, chosen by
    the joint's position in the chain, because a kernel cannot index a local
    array. A seventh revolute joint would fall into the final `else`, add
    itself to the sixth accumulator, and return a wrong gradient with
    nothing to say so.
    """
    joints = [
        {"name": f"J{i}", "parent": "base" if i == 1 else f"L{i - 1}", "child": f"L{i}",
         "type": "revolute", "xyz_m": [0.0, 0.0, 0.1], "rpy_rad": [0.0, 0.0, 0.0],
         "axis_xyz": [0, 0, 1]}
        for i in range(1, 8)
    ]
    cell = {
        "frames": {"base": "base", "tool": "L7"},
        "links": [{"name": "base"}] + [{"name": f"L{i}"} for i in range(1, 8)],
        "joints": joints,
    }
    # Six is fine, seven is not.
    fk.build(cell, ["L6"])
    with pytest.raises(fk.FKError, match="backward kernel folds at most"):
        fk.build(cell, ["L7"])
