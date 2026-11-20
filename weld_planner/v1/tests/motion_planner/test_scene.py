"""The assembled collision model: frames, groups, and who checks what.

The test that matters here is the frame asymmetry. Everything else in this
file guards a rule that would fail loudly; that one guards a failure with no
symptom at all -- a detached work transform zeroes `d(cost)/d(J7)`, the
planner keeps running, and the positioner quietly stops being a degree of
freedom it can use.
"""

from __future__ import annotations

import numpy as np
import pytest
import torch
from weld_motion_planner.robot_cell.collision import scene as scenemod
from weld_motion_planner.robot_cell.collision.scene import (
    Scene,
    SceneError,
    box_mesh,
    build,
    placement_matrix,
)

pytestmark = pytest.mark.skipif(
    not torch.cuda.is_available(), reason="the distance fields are a GPU path"
)

CELL = scenemod._demo_cell()
MODEL = {
    "link_1": [((0.15, 0.0, 0.0), 0.06)],
    "link_2": [((0.10, 0.0, 0.0), 0.05), ((0.25, 0.0, 0.0), 0.05)],
    "link_3": [((0.10, 0.0, 0.0), 0.05)],
}
FLAT = {"anchor_xyz_m": [0, 0, 0], "offset_xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]}


@pytest.fixture(scope="module")
def scene() -> Scene:
    return build(
        CELL, MODEL, device=torch.device("cuda", torch.cuda.current_device()),
        placement=FLAT,
        part=box_mesh([-0.08, -0.08, 0.0], [0.08, 0.08, 0.16]),
        fixtures=box_mesh([-0.25, -0.25, -0.04], [0.25, 0.25, 0.0]),
        env=box_mesh([-1.5, -1.5, -0.05], [1.5, 1.5, 0.0]),
        part_voxel_m=0.006, coarse_voxel_m=0.012, margin_m=0.12,
    )


def axis(scene: Scene, name: str) -> int:
    return scene.spec.axis_names.index(name)


def test_the_positioner_axis_reaches_the_fk_spec(scene):
    """The work link joins the chain even though it carries no spheres.

    Without it the positioner is not in `q` at all, and no amount of
    correct gradient plumbing downstream can recover an axis the kinematics
    never had.
    """
    assert "J7" in scene.spec.axis_names
    assert scene.work_index is not None
    assert scene.spec.link_names[scene.work_index] == "table"


def test_world_field_does_not_move_with_the_positioner(scene):
    q = torch.zeros(1, scene.spec.num_dof, device=scene.radii.device, requires_grad=True)
    only_world = Scene(
        fk=scene.fk, spec=scene.spec, radii=scene.radii,
        groups=[g for g in scene.groups if g.frame == "world"],
        work_index=scene.work_index, work_to_piece=scene.work_to_piece,
        sphere_links=scene.sphere_links,
    )
    only_world.clearance(q).sum().backward()
    assert abs(float(q.grad[0, axis(scene, "J7")])) < 1e-9


def test_workpiece_fields_do_move_with_the_positioner(scene):
    """The one with no other symptom. A detached `T_work` reads as zero here
    and as a working planner everywhere else."""
    device = scene.radii.device
    q = torch.zeros(1, scene.spec.num_dof, device=device)
    q[0, axis(scene, "J2")] = -0.6           # reach toward the table
    q = q.requires_grad_(True)
    only_piece = Scene(
        fk=scene.fk, spec=scene.spec, radii=scene.radii,
        groups=[g for g in scene.groups if g.frame == "workpiece"],
        work_index=scene.work_index, work_to_piece=scene.work_to_piece,
        sphere_links=scene.sphere_links,
    )
    only_piece.clearance(q).sum().backward()
    assert abs(float(q.grad[0, axis(scene, "J7")])) > 1e-6


def test_the_test_pose_reaches_inside_a_grid(scene):
    """Otherwise every assertion here is about the conservative extension.

    Outside a grid the field returns a distance to the BOX, not to the
    geometry, and it does so whatever the values inside are -- so a pose
    that never enters a grid would pass every other test in this file with
    the field baked from nothing at all. The first version of the demo cell
    put the table beyond the arm's reach and did exactly that.
    """
    device = scene.radii.device
    q = torch.zeros(1, scene.spec.num_dof, device=device)
    q[0, axis(scene, "J2")] = -0.6
    reached = 0
    with torch.no_grad():
        world = scene.fk(q)
        for group in scene.groups:
            pts = world.index_select(1, group.spheres)
            if group.frame == "workpiece":
                pts = scene.world_to_workpiece(pts, q)
            lo, hi = group.field.origin_pos_m, group.field.upper_pos_m
            reached += int((~((pts < lo) | (pts > hi)).any(dim=-1)).sum())
    assert reached > 0


def test_clearance_is_the_min_over_groups(scene):
    """A union of bodies is the min of the samples, not the sum or the first."""
    device = scene.radii.device
    q = torch.zeros(1, scene.spec.num_dof, device=device)
    with torch.no_grad():
        whole = scene.clearance(q)
        parts = []
        for group in scene.groups:
            sub = Scene(fk=scene.fk, spec=scene.spec, radii=scene.radii, groups=[group],
                        work_index=scene.work_index, work_to_piece=scene.work_to_piece,
                        sphere_links=scene.sphere_links)
            parts.append(sub.clearance(q))
    assert torch.allclose(whole, torch.stack(parts).min(dim=0).values, atol=1e-6)


def test_every_sphere_is_checked_by_something(scene):
    """`UNCHECKED_M` surviving into the answer means a sphere no group looks
    at -- invisible to collision, and it would read as enormous clearance."""
    q = torch.zeros(1, scene.spec.num_dof, device=scene.radii.device)
    with torch.no_grad():
        d = scene.clearance(q)
    assert torch.isfinite(d).all()
    assert float(d.max()) < scenemod.UNCHECKED_M / 2


def test_link_adjacent_to_the_environment_leaves_the_env_group(scene):
    """`link_1` hangs off `floor`, so it is not checked against the world
    field -- but it IS still checked against the part."""
    env = next(g for g in scene.groups if g.label == "env")
    piece = next(g for g in scene.groups if g.frame == "workpiece")
    every = set(scene.sphere_links)
    env_links = {scene.sphere_links[int(i)] for i in env.spheres}
    piece_links = {scene.sphere_links[int(i)] for i in piece.spheres}
    # Stated as the rule rather than as this cell's link list, so adding a
    # link to the demo does not read as a regression.
    assert env_links == every - {"link_1"}
    assert piece_links == every


def test_placement_puts_the_anchor_inside_the_rotation():
    """`ui/src/state/placement.ts`'s convention, which this must match or the
    planner plans against a part the authoring side drew elsewhere.

    A 90 degree yaw about an anchor offset in x must SPIN the part about
    that anchor, not swing it around the work origin.
    """
    m = placement_matrix({"anchor_xyz_m": [0.1, 0, 0], "offset_xyz_m": [0.1, 0, 0],
                          "rpy_rad": [0, 0, np.pi / 2]})
    anchor_lands_on = m @ np.array([0.1, 0.0, 0.0, 1.0])
    assert np.allclose(anchor_lands_on[:3], [0.1, 0.0, 0.0], atol=1e-9)


def test_a_cell_with_no_work_frame_refuses_a_workpiece_field():
    cell = {k: v for k, v in CELL.items()}
    cell["frames"] = {"base": "world", "tool": "link_2"}
    with pytest.raises(SceneError, match="no work frame"):
        build(cell, MODEL, device=torch.device("cuda", torch.cuda.current_device()),
              placement=FLAT, part=box_mesh([-0.05] * 3, [0.05] * 3), coarse_voxel_m=0.02)


def test_a_cell_with_no_spheres_is_refused():
    with pytest.raises(SceneError, match="no arm link carries spheres"):
        build(CELL, {}, device=torch.device("cuda", torch.cuda.current_device()),
              env=box_mesh([-1, -1, -0.05], [1, 1, 0.0]), coarse_voxel_m=0.05)


def test_box_mesh_is_wound_outward():
    """A box wound inward reads as a hole with the field's sign flipped --
    the exact failure `esdf.py` documents and cannot rescue."""
    from weld_motion_planner.geometry import esdf

    field = esdf.bake(box_mesh([-0.1, -0.1, -0.1], [0.1, 0.1, 0.1]), voxel_m=0.01,
                      device=torch.device("cuda", torch.cuda.current_device()))
    centre = torch.zeros(1, 3, device=field.values.device)
    assert float(field.sample(centre)) < -0.09


def test_self_pairs_skip_the_same_link_and_its_neighbours(scene):
    """Two links across a bearing overlap at the mating face by construction,
    and a link overlaps itself wherever its cover needed it to. Checking
    either reports a collision at every pose the robot can reach."""
    assert scene.self_pairs is not None and scene.self_pairs.numel()
    for a, b in scene.self_pairs.tolist():
        la, lb = scene.sphere_links[a], scene.sphere_links[b]
        assert la != lb
        assert {la, lb} not in ({"link_1", "link_2"}, {"link_2", "link_3"})
    # Only link_1 against link_3 survives on this arm.
    assert int(scene.self_pairs.shape[0]) == 1


def test_a_folded_pose_self_collides_by_the_amount_the_geometry_says(scene):
    """Both elbows at pi put link_3's sphere at x = 0.10 and link_1's at
    x = 0.15, 50 mm apart against radii summing to 110 mm."""
    device = scene.radii.device
    q = torch.zeros(1, scene.spec.num_dof, device=device)
    q[0, axis(scene, "J2")] = float(np.pi)
    q[0, axis(scene, "J3")] = float(np.pi)
    with torch.no_grad():
        gap = scene.self_clearance(q)
    assert float(gap.min()) == pytest.approx(-0.060, abs=0.002)


def test_self_collision_backward_matches_plain_torch(scene):
    """A hand-written kernel backward that scatters through atomics, checked
    against the gather-and-norm it replaced."""
    device = scene.radii.device
    q = torch.zeros(1, scene.spec.num_dof, device=device)
    q[0, axis(scene, "J2")] = float(np.pi)
    q[0, axis(scene, "J3")] = float(np.pi)
    weight = torch.randn(1, int(scene.self_pairs.shape[0]), device=device,
                         generator=torch.Generator(device=device).manual_seed(11))

    qa = q.clone().requires_grad_(True)
    (scene.self_clearance(qa) * weight).sum().backward()

    qb = q.clone().requires_grad_(True)
    world = scene.fk(qb)
    a, b = scene.self_pairs[:, 0], scene.self_pairs[:, 1]
    delta = world.index_select(1, a) - world.index_select(1, b)
    ref = torch.linalg.vector_norm(delta, dim=-1) - (
        scene.radii.index_select(0, a) + scene.radii.index_select(0, b))
    (ref * weight).sum().backward()

    scale = max(float(qb.grad.abs().max()), 1e-12)
    assert float((qa.grad - qb.grad).abs().max()) / scale < 1e-4


def test_the_zero_gradient_shortcut_does_not_change_the_answer(scene):
    """Backward returns early when a pair's incoming gradient is zero, which
    is most of them under a hinge. It must be an optimisation and not a
    change of answer."""
    device = scene.radii.device
    q = torch.zeros(1, scene.spec.num_dof, device=device)
    q[0, axis(scene, "J2")] = float(np.pi)
    q[0, axis(scene, "J3")] = float(np.pi)

    qa = q.clone().requires_grad_(True)
    torch.relu(0.02 - scene.self_clearance(qa)).pow(2).sum().backward()

    qb = q.clone().requires_grad_(True)
    world = scene.fk(qb)
    a, b = scene.self_pairs[:, 0], scene.self_pairs[:, 1]
    delta = world.index_select(1, a) - world.index_select(1, b)
    ref = torch.linalg.vector_norm(delta, dim=-1) - (
        scene.radii.index_select(0, a) + scene.radii.index_select(0, b))
    torch.relu(0.02 - ref).pow(2).sum().backward()

    scale = max(float(qb.grad.abs().max()), 1e-12)
    assert float((qa.grad - qb.grad).abs().max()) / scale < 1e-4


# --------------------------------------------------------------------------
# The checker: somebody else's axes, this scene's chain


def test_the_checker_scatters_by_name_and_fills_the_held(scene):
    """`q_full` must agree with scattering by hand -- the four call sites it
    replaced (the lattice screen, M5, M6, the roadmap) each did exactly this,
    and the lattice screen had already drifted: it left held axes at zero."""
    q = torch.tensor([[0.3, -0.2, 0.5], [0.0, 0.1, -0.4]], device=scene.radii.device,
                     dtype=torch.float64)
    check = scene.checker(["J1", "J2", "J7"], {"J3": 0.25})
    full = check.q_full(q)
    assert full.shape == (2, scene.spec.num_dof)
    assert full[:, axis(scene, "J1")].tolist() == pytest.approx([0.3, 0.0])
    assert full[:, axis(scene, "J2")].tolist() == pytest.approx([-0.2, 0.1])
    assert full[:, axis(scene, "J7")].tolist() == pytest.approx([0.5, -0.4])
    # Held: filled with the profile's value, not the zero a fresh tensor holds.
    assert full[:, axis(scene, "J3")].tolist() == pytest.approx([0.25, 0.25])


def test_the_checker_takes_per_pose_held_values(scene):
    """M5's rows may sit on different stations, so `held` accepts one value
    per pose, not one for the batch."""
    q = torch.zeros(3, 2, device=scene.radii.device)
    per_pose = torch.tensor([0.1, 0.2, 0.3], device=scene.radii.device)
    full = scene.checker(["J1", "J2"], {"J3": per_pose}).q_full(q)
    assert full[:, axis(scene, "J3")].tolist() == pytest.approx([0.1, 0.2, 0.3])


def test_the_checker_merges_the_arm_against_itself(scene):
    """`clearance` is the worst gap ANYWHERE. The folded pose collides with
    the arm's own forearm and clears every field, so a checker that read only
    the fields would call it clear -- the exact hole the lattice screen once
    had (60% of screened-feasible nodes on `bracket_a_placed` seam 1)."""
    check = scene.checker(list(scene.spec.axis_names))
    folded = torch.zeros(1, scene.spec.num_dof, device=scene.radii.device)
    # Both elbows at pi: the fold the self-collision tests above measure at
    # about -60 mm of gap.
    folded[0, axis(scene, "J2")] = float(np.pi)
    folded[0, axis(scene, "J3")] = float(np.pi)
    full = check.q_full(folded)
    env_part = scene.clearance(full).min(dim=-1).values
    self_part = scene.self_clearance(full).min(dim=-1).values
    merged = check.clearance(folded)
    assert float(self_part[0]) < 0.0, "the folded pose should self-collide"
    assert float(merged[0]) == pytest.approx(
        float(torch.minimum(env_part, self_part)[0])), \
        "the merge is not the min of the fields and the arm against itself"


def test_the_checker_carries_gradients(scene):
    """The same object serves the screen and an optimizer, which is the point
    of having one: `clearance` must backpropagate to the CALLER's q, held
    axes and unknown axes contributing nothing."""
    q = torch.tensor([[0.4, 0.9, 0.3]], device=scene.radii.device,
                     dtype=torch.float64, requires_grad=True)
    check = scene.checker(["J1", "J2", "J7"], {"J3": 0.1})
    gap = check.clearance(q)
    gap.sum().backward()
    assert q.grad is not None
    assert bool(torch.isfinite(q.grad).all())
    assert float(q.grad.abs().max()) > 0.0, \
        "a clearance that ignores the pose has no gradient to offer"


def test_an_arm_whose_links_are_all_adjacent_has_nothing_to_check():
    """A two-link arm. Zero pairs is the right answer, and it must come back
    as an empty tensor rather than as a failure."""
    cell = {**CELL, "frames": {"base": "world", "tool": "link_2", "work": "table"}}
    built = build(cell, {k: v for k, v in MODEL.items() if k != "link_3"},
                  device=torch.device("cuda", torch.cuda.current_device()),
                  placement=FLAT, env=box_mesh([-1.5, -1.5, -0.05], [1.5, 1.5, 0.0]),
                  coarse_voxel_m=0.05)
    assert int(built.self_pairs.shape[0]) == 0
    q = torch.zeros(1, built.spec.num_dof, device=built.radii.device)
    assert built.self_clearance(q).shape == (1, 0)
