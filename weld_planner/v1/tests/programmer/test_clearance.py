"""Does this torch, at these angles, fit?

The check is a ray fan, and a ray fan is the kind of thing that produces
confident numbers long after it has stopped being right. Three defences, in
order of how much they are worth:

1. **Closed form.** A cylinder on the bisector of a wedge of dihedral theta
   clears it iff `theta >= 2*atan(R/d)`. That is `tooling.fits_dihedral_deg`,
   derived independently of any ray, and the fan has to reproduce it on a
   synthetic groove whose angle is known by construction rather than measured
   off a fixture someone might re-export.
2. **Conservation.** Clear plus blocked equals the seam length, exactly. Three
   unit conventions meet in this module -- frames in metres, OCCT in
   millimetres, the torch in millimetres -- and a 1000x slip cannot survive
   this assertion.
3. **Monotonicity.** A fatter torch never clears more; a longer standoff never
   clears less. Both are physics, neither depends on a fixture, and a sign
   error in the frame or the fan breaks at least one.

The pinned pipe_bracket numbers come last on purpose. They are the least valuable
assertions here -- they pin behaviour rather than prove correctness -- but they
are what catches a silent change in the frames or the sampling.
"""

import copy
import inspect
import itertools
import math

import numpy as np
import pytest
from seam_worker.reach.access import annotate
from seam_worker.reach.clearance import (
    COARSE_STRIDE,
    FINE_RAY_COUNT,
    MM_PER_M,
    ClearanceError,
    SeamGeometry,
    barrel_axes,
    build_lattice,
    cast_fan,
    clearance,
    collides_at,
    fan_collides,
    free_standoffs,
    intersector,
    search_inputs,
    synthetic_corner,
)
from seam_worker.seam.detect import detect_joints
from seam_worker.seam.seams import default_seams
from seam_worker.reach.tooling import DEFAULT_TORCH, fits_dihedral_deg, weld_standoff_mm
from seam_worker.cad.topology import extract_topology, load_step


@pytest.fixture(scope="module")
def pipe_bracket(pipe_bracket_step):
    topology = extract_topology(pipe_bracket_step)
    joints = annotate(pipe_bracket_step, topology, detect_joints(topology))
    joints["seams"] = default_seams(joints)["seams"]
    return {
        "shape": load_step(pipe_bracket_step),
        "joints": {j["id"]: j for j in joints["weld_joints"]},
        "seams": {s["id"]: s for s in joints["seams"]},
    }


# What the shipped torch used to carry as `default_standoff_mm`, and what every
# call below was measuring at without saying so. `clearance()` now requires the
# gap, because a fit angle is a claim about a gun AND a distance -- so the tests
# state it too, through one helper rather than at thirty call sites.
NOMINAL_STANDOFF_MM = 10.0
# The stickout every detected seam carried when the geometry below was recorded.
# The default is 17 mm since 2026-09-11 (`seams.DEFAULT_STANDOFF_MM`); these
# measure the part at a stated distance, so they state it rather than follow
# the default -- at 17 mm the +/-12 degree cliff is 79% clear, not blocked.
RECORDED_STICKOUT_MM = 12.0


def _measure(shape, joint, seam, torch=None, **kwargs):
    """`clearance()` at the gap the seam asks for, or the nominal one."""
    kwargs.setdefault("standoff_mm", weld_standoff_mm(seam, NOMINAL_STANDOFF_MM))
    return clearance(shape, joint, seam, torch, **kwargs)


def _seam(pipe_bracket, seam_id, **orientation):
    seam = copy.deepcopy(pipe_bracket["seams"][seam_id])
    for key, value in orientation.items():
        seam["orientation"][key] = {"kind": "constant", "value": float(value)}
    return pipe_bracket["joints"][seam["weld_joint_ref"]], seam


# ---------------------------------------------------------------- closed form


@pytest.mark.parametrize("dihedral", [120.0, 100.0, 90.0, 80.0, 78.0])
def test_a_groove_wider_than_the_fit_angle_is_clear(dihedral):
    """77.32 degrees for the shipped torch at 10 mm. Above it, everything fits."""
    assert dihedral > fits_dihedral_deg(DEFAULT_TORCH, 10.0)
    shape, joint, seam = synthetic_corner(dihedral)
    result = _measure(shape, joint, seam)
    assert result["clear_fraction"] == 1.0, dihedral


@pytest.mark.parametrize("dihedral", [77.0, 70.0, 60.0, 45.0])
def test_a_groove_tighter_than_the_fit_angle_is_blocked(dihedral):
    assert dihedral < fits_dihedral_deg(DEFAULT_TORCH, 10.0)
    shape, joint, seam = synthetic_corner(dihedral)
    result = _measure(shape, joint, seam)
    assert result["clear_fraction"] == 0.0, dihedral


def test_the_fan_reproduces_the_closed_form_to_within_half_a_degree():
    """The single most valuable assertion in this file.

    Bisect the dihedral at which the ray fan flips, and require it to land on
    `2*atan(R/d)`. Two entirely separate derivations -- one trigonometric, one
    by casting 65 rays at a B-rep -- have to agree.
    """
    shape_cache: dict[float, tuple] = {}

    def clear_at(dihedral: float) -> bool:
        if dihedral not in shape_cache:
            shape_cache[dihedral] = synthetic_corner(dihedral)
        shape, joint, seam = shape_cache[dihedral]
        return _measure(shape, joint, seam)["clear_fraction"] > 0.5

    lo, hi = 60.0, 100.0  # blocked, clear
    for _ in range(7):
        mid = 0.5 * (lo + hi)
        if clear_at(mid):
            hi = mid
        else:
            lo = mid
    measured = 0.5 * (lo + hi)
    assert measured == pytest.approx(fits_dihedral_deg(DEFAULT_TORCH, 10.0), abs=0.5)


def test_the_closed_form_agreement_holds_at_another_standoff():
    """Pulling the torch back must move the flip point, by the right amount."""
    shape, joint, seam = synthetic_corner(60.0)
    # 60 degrees needs R/d <= tan(30) = 0.577, so d >= 13.86 mm for R = 8.
    assert _measure(shape, joint, seam, standoff_mm=12.0)["clear_fraction"] == 0.0
    assert _measure(shape, joint, seam, standoff_mm=16.0)["clear_fraction"] == 1.0
    assert fits_dihedral_deg(DEFAULT_TORCH, 16.0) < 60.0 < fits_dihedral_deg(DEFAULT_TORCH, 12.0)


# --------------------------------------------------------------- conservation


def test_clear_plus_blocked_is_the_whole_seam(pipe_bracket):
    """A 1000x unit error cannot survive this.

    Frames arrive in metres, OCCT works in millimetres, the torch is in
    millimetres. One conversion, one place, and this is the tripwire.
    """
    for seam_id in ("seam_0001", "seam_0003", "seam_0007", "seam_0011"):
        joint, seam = _seam(pipe_bracket, seam_id)
        result = _measure(pipe_bracket["shape"], joint, seam)
        total = result["clear_length_m"] + result["blocked_length_m"]
        assert total == pytest.approx(result["seam_length_m"], abs=1e-9), seam_id
        assert result["clear_fraction"] + sum(
            hi - lo for lo, hi in result["blocked_spans"]
        ) == pytest.approx(1.0, abs=1e-9)


def test_the_seam_length_is_the_seam_and_not_the_joint(pipe_bracket):
    """`seam_0002` is a 10 mm piece of a 110 mm joint. Reporting the joint's
    length here would make every coverage percentage wrong by the ratio."""
    joint, seam = _seam(pipe_bracket, "seam_0002")
    result = _measure(pipe_bracket["shape"], joint, seam)
    assert result["seam_length_m"] == pytest.approx(0.010, abs=1e-4)
    assert joint["weld_line"]["length_m"] > 0.05


def test_spans_are_in_seam_fraction_not_joint_s(pipe_bracket):
    """Everything the operator sees is drawn on the seam's own axis, because
    that is the axis the angle profiles are drawn on."""
    joint, seam = _seam(pipe_bracket, "seam_0003")
    result = _measure(pipe_bracket["shape"], joint, seam)
    for lo, hi in result["blocked_spans"] + result["clear_runs"]:
        assert 0.0 <= lo <= hi <= 1.0, (lo, hi)


# -------------------------------------------------------------- monotonicity


def test_a_fatter_torch_never_clears_more(pipe_bracket):
    joint, seam = _seam(pipe_bracket, "seam_0007", travel_angle_deg=10.0)
    got = [
        _measure(
            pipe_bracket["shape"], joint, seam, {**DEFAULT_TORCH, "body_radius_mm": radius}
        )["clear_fraction"]
        for radius in (2.0, 5.0, 8.0, 10.0, 12.5)
    ]
    assert got == sorted(got, reverse=True), got
    assert got[0] > got[-1], "the radius has to matter at all"


def test_a_longer_standoff_never_clears_less(pipe_bracket):
    """The lever the optimizer searches over, and the one that is
    metallurgically expensive -- which is why the band defaults to zero width."""
    joint, seam = _seam(pipe_bracket, "seam_0007")
    got = [
        _measure(pipe_bracket["shape"], joint, seam, standoff_mm=d)["clear_fraction"]
        for d in (6.0, 10.0, 12.0, 16.0, 20.0)
    ]
    assert got == sorted(got), got
    assert got[0] == 0.0, "R > d means nothing fits, by the closed form"


def test_the_admissible_work_angle_window_is_a_cliff(pipe_bracket):
    """Why the optimizer bins work angle at 2 degrees and not 10.

    On this fillet the torch is fine at +/-6 degrees and gone at +/-12. A
    10 degree bin would step straight over the usable band.
    """
    joint, _ = _seam(pipe_bracket, "seam_0007")
    got = {}
    for work in (-12.0, -6.0, 0.0, 6.0, 12.0):
        _, seam = _seam(pipe_bracket, "seam_0007", work_angle_deg=work, travel_angle_deg=10.0)
        got[work] = _measure(
            pipe_bracket["shape"], joint, seam, standoff_mm=RECORDED_STICKOUT_MM
        )["clear_fraction"]
    assert got[-12.0] == 0.0 and got[12.0] == 0.0, got
    assert got[-6.0] > 0.5 and got[0.0] > 0.5 and got[6.0] > 0.5, got


# ------------------------------------------------------- how a ray reports it


def _solid_block(half_mm=200.0):
    """A block with the weld line buried at its centre. Nothing about it is a
    weld; it is the degenerate case the parity rule exists for."""
    import cadquery as cq

    shape = cq.Workplane("XY").box(2 * half_mm, 2 * half_mm, 2 * half_mm).val()
    joint = {
        "id": "weld_joint_buried",
        "weld_line": {
            "closed": False,
            "length_m": 0.08,
            "segments": [
                {
                    "degree": 1,
                    "knots_u": [0.0, 0.0, 1.0, 1.0],
                    "control_points_xyz_m": [[-0.04, 0.0, 0.0], [0.04, 0.0, 0.0]],
                    "length_m": 0.08,
                }
            ],
        },
    }
    seam = {
        "id": "seam_buried",
        "weld_joint_ref": joint["id"],
        "span": {"start_s": 0.0, "end_s": 1.0},
        "direction": "forward",
        "reference": {"mode": "fixed_vector", "fixed_direction_xyz": [0.0, 0.0, 1.0]},
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 0.0},
        },
        "process": {"wire": {"stickout_mm": 10.0}},
    }
    return shape, joint, seam


def test_a_buried_torch_is_caught_by_parity_not_by_a_hit():
    """The case a bounded ray misses entirely.

    The tip sits at the centre of a 400 mm block, so a 150 mm barrel starting
    10 mm out is buried along its whole length and never reaches a surface: the
    nearest face is 200 mm away, past its far end. Nothing lies *inside* the
    body, and only the odd number of crossings BEYOND it says the far end is
    still in the metal.
    """
    shape, joint, seam = _solid_block()
    geometry = SeamGeometry(joint, seam)
    point, t, r = geometry.frame(0.5)
    fan = cast_fan(intersector(shape), point, t, r, 0.0, 0.0, 8.0, FINE_RAY_COUNT)

    standoff, length = 10.0, 150.0
    inside_body = [w for hits in fan for w in hits if standoff < w <= standoff + length]
    assert not inside_body, f"the test premise is wrong: {inside_body[:3]}"
    assert [w for hits in fan for w in hits if w > standoff + length], "no far crossing"
    assert fan_collides(fan, standoff, length), "parity did not fire"
    assert _measure(shape, joint, seam)["clear_fraction"] == 0.0


def test_parity_does_not_fire_on_a_torch_that_is_merely_pointing_at_something():
    """The other half of the rule: one crossing beyond the barrel means buried,
    two mean it is looking through a wall at open air on the far side."""
    # Two crossings beyond the barrel: the far face of a plate the torch has
    # already cleared. Even parity, no collision.
    assert not fan_collides([[400.0, 420.0]], 10.0, 150.0)
    assert fan_collides([[400.0]], 10.0, 150.0)
    assert fan_collides([[100.0]], 10.0, 150.0), "a hit inside the body"
    assert not fan_collides([[5.0]], 10.0, 150.0), "inside the standoff gap"
    assert not fan_collides([[]], 10.0, 150.0)


def test_material_inside_the_standoff_gap_is_not_a_collision():
    """A fillet's own two faces are always within the gap. Counting them would
    report every weld in the world as blocked."""
    # A 90 degree groove: the faces cross the rim rays at 8 mm, inside the
    # 10 mm gap.
    shape, joint, seam = synthetic_corner(90.0)
    geometry = SeamGeometry(joint, seam)
    point, t, r = geometry.frame(0.5)
    fan = cast_fan(intersector(shape), point, t, r, 0.0, 0.0, 8.0, FINE_RAY_COUNT)
    near = [w for hits in fan for w in hits if w < 10.0]
    assert near, "the test premise is wrong: nothing in the gap"
    assert not fan_collides(fan, 10.0, 150.0)


def test_one_set_of_casts_answers_for_every_standoff():
    """The reason `hits_along` keeps the whole list, and the reason standoff is
    an outer loop in the optimizer rather than a third lattice axis."""
    shape, joint, seam = synthetic_corner(70.0)
    geometry = SeamGeometry(joint, seam)
    point, t, r = geometry.frame(0.5)
    fan = cast_fan(intersector(shape), point, t, r, 0.0, 0.0, 8.0, FINE_RAY_COUNT)

    candidates = [6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 20.0]
    free = free_standoffs(fan, 150.0, candidates)
    # 70 degrees needs R/d <= tan(35) = 0.700, so d >= 11.4 mm.
    assert all(d >= 11.0 for d in free), free
    assert 12.0 in free and 8.0 not in free
    # ...and re-casting from scratch at one of them must agree.
    assert (_measure(shape, joint, seam, standoff_mm=12.0)["clear_fraction"] > 0.5) is True
    assert _measure(shape, joint, seam, standoff_mm=8.0)["clear_fraction"] == 0.0


def test_the_coarse_fan_only_ever_under_reports_collisions(pipe_bracket):
    """The polarity that makes the optimizer's lattice safe to build cheaply
    and unsafe to trust without a fine re-verification."""
    for seam_id in ("seam_0003", "seam_0007", "seam_0011"):
        joint, seam = _seam(pipe_bracket, seam_id)
        fine = _measure(pipe_bracket["shape"], joint, seam, ray_count=FINE_RAY_COUNT)
        coarse = _measure(
            pipe_bracket["shape"], joint, seam,
            ray_count=FINE_RAY_COUNT // COARSE_STRIDE + 1,
        )
        assert coarse["clear_fraction"] >= fine["clear_fraction"] - 1e-9, seam_id


# ------------------------------------------------------------------ the frame


def test_the_disc_basis_is_orthonormal_and_tied_to_the_seam():
    """A fan tied to a world axis would rotate relative to the groove as the
    seam turns, so the collision it found would depend on how the part was
    modelled."""
    t = np.array([1.0, 0.0, 0.0])
    for angle in (0.0, 0.3, 1.0, 1.5):
        u = np.array([math.sin(angle), 0.0, math.cos(angle)])
        e1, e2 = barrel_axes(t, u)
        assert float(np.linalg.norm(e1)) == pytest.approx(1.0)
        assert float(np.linalg.norm(e2)) == pytest.approx(1.0)
        assert float(np.dot(e1, u)) == pytest.approx(0.0, abs=1e-12)
        assert float(np.dot(e2, u)) == pytest.approx(0.0, abs=1e-12)
        assert float(np.dot(e1, e2)) == pytest.approx(0.0, abs=1e-12)
        # e1 is the tangent's perpendicular part, so it stays on the t-u plane.
        assert abs(float(np.dot(np.cross(t, u), e1))) == pytest.approx(0.0, abs=1e-12)


def test_the_disc_basis_survives_the_degenerate_case():
    """Travel angle 90 degrees points the torch straight down the seam."""
    t = np.array([1.0, 0.0, 0.0])
    e1, e2 = barrel_axes(t, t)
    assert float(np.linalg.norm(e1)) == pytest.approx(1.0)
    assert float(np.dot(e1, t)) == pytest.approx(0.0, abs=1e-12)
    assert float(np.dot(e2, t)) == pytest.approx(0.0, abs=1e-12)


def test_frames_come_back_in_millimetres(pipe_bracket):
    """The one conversion, checked against the curve it came from."""
    joint, seam = _seam(pipe_bracket, "seam_0007")
    geometry = SeamGeometry(joint, seam)
    pos_mm, _, _ = geometry.frame(0.5)
    from seam_worker.seam.span import s_at

    expected_m = geometry.curve.point(s_at(geometry.span, 0.5))
    assert pos_mm == pytest.approx(np.asarray(expected_m) * MM_PER_M, abs=1e-9)
    assert float(np.linalg.norm(pos_mm)) > 1.0, "millimetres, not metres"


# ------------------------------------------------------------------- refusals


def test_a_reversed_seam_negates_the_tangent_and_nothing_else():
    """The schema's own sentence: "sets the sign of the tangent t". No second
    convention, so every consequence has to fall out of `apply_angles`.

    Checked on a synthetic corner, where the right answers are known by
    construction. The two that matter:

    - `r` does NOT move. It is the rail direction projected perpendicular to
      `t`, and (d.-t)(-t) == (d.t)t, so the datum stays put when you turn
      around -- which is what a datum is for.
    - the work roll REVERSES. +6 degrees leans toward the other plate, because
      "lean left" depends on which way you are facing.
    """
    shape, joint, seam = synthetic_corner(90.0)
    forward = SeamGeometry(joint, seam)
    back = SeamGeometry(joint, {**seam, "direction": "reverse"})

    pf, tf, rf = forward.frame(0.4)
    pb, tb, rb = back.frame(0.4)
    assert np.allclose(pf, pb), "the point on the seam does not move"
    assert np.allclose(tf, -tb), "the travel tangent does"
    assert np.allclose(rf, rb), "the 0-degree datum does not"

    # A symmetric groove at work 0 answers the same either way: with nothing
    # leaning, there is nothing for the direction to reverse.
    at_zero = {"kind": "constant", "value": 0.0}
    seam["orientation"]["work_angle_deg"] = at_zero
    seam["orientation"]["travel_angle_deg"] = at_zero
    assert _measure(shape, joint, seam)["clear_fraction"] == pytest.approx(
        _measure(shape, joint, {**seam, "direction": "reverse"})["clear_fraction"]
    )

    # Tip it over, and forward at +work must equal reverse at -work -- the same
    # torch in the same place, described from either end.
    def tilted(work, direction):
        s = copy.deepcopy(seam)
        s["direction"] = direction
        s["orientation"]["work_angle_deg"] = {"kind": "constant", "value": work}
        return _measure(shape, joint, s)["clear_fraction"]

    for work in (20.0, 35.0, 50.0):
        assert tilted(work, "forward") == pytest.approx(tilted(-work, "reverse")), work


def test_reversing_moves_where_the_torch_fouls_to_the_other_end(pipe_bracket):
    """Drag leans the torch back over completed weld, so turning around moves
    the fouling to the other end of the seam.

    On this fixture the arm is symmetric, so the spans come back MIRRORED and
    the percentage does not move at all -- which is worth pinning precisely
    because it is the case where a coverage figure alone would suggest the
    button did nothing. What the operator sees change is the red line, and the
    red line is drawn from these spans.
    """
    joint, seam = _seam(pipe_bracket, "seam_0007", travel_angle_deg=25.0)
    forward = _measure(pipe_bracket["shape"], joint, seam)
    back = _measure(pipe_bracket["shape"], joint, {**seam, "direction": "reverse"})

    assert forward["blocked_spans"] != back["blocked_spans"], "reversing has to do something"
    mirrored = sorted([1.0 - hi, 1.0 - lo] for lo, hi in forward["blocked_spans"])
    flat = [v for span in mirrored for v in span]
    assert [v for span in back["blocked_spans"] for v in span] == pytest.approx(flat, abs=2e-3)
    assert back["clear_fraction"] == pytest.approx(forward["clear_fraction"], abs=2e-3)


def test_an_unknown_direction_is_refused_rather_than_read_as_forward(pipe_bracket):
    """`forward` is the default, but a typo is not a default."""
    joint, seam = _seam(pipe_bracket, "seam_0001")
    seam["direction"] = "backwards"
    with pytest.raises(ClearanceError, match="expected 'forward' or 'reverse'"):
        _measure(pipe_bracket["shape"], joint, seam)


# ------------------------------------------------------------- what it reports


def test_the_result_carries_the_torch_that_produced_it(pipe_bracket):
    """R-UI-2. A coverage number without its torch and standoff is not a
    measurement, it is a rumour."""
    joint, seam = _seam(pipe_bracket, "seam_0001")
    process = seam.get("process") or {}
    seam["process"] = {**process, "wire": {**(process.get("wire") or {}), "stickout_mm": RECORDED_STICKOUT_MM}}
    result = _measure(pipe_bracket["shape"], joint, seam)
    assert result["torch"]["body_radius_mm"] == 8.0
    assert result["standoff_mm"] == 12.0, "the seam's own stickout, not the torch default"
    assert result["fits_dihedral_deg"] == pytest.approx(67.38, abs=0.01)
    assert result["ray_count"] == FINE_RAY_COUNT
    assert result["samples"] > 2


def test_sealed_stretches_are_reported_separately(pipe_bracket):
    """No angle and no optimizer recovers a sealed pocket, so it is a different
    colour on screen and a different decision for the operator."""
    joint, seam = _seam(pipe_bracket, "seam_0007")
    result = _measure(pipe_bracket["shape"], joint, seam)
    assert result["sealed_spans"], "seam_0007 abuts the gusset"
    for lo, hi in result["sealed_spans"]:
        assert 0.0 <= lo < hi <= 1.0


def test_a_weld_with_no_sealed_stretch_says_so(pipe_bracket):
    joint, seam = _seam(pipe_bracket, "seam_0002")
    assert _measure(pipe_bracket["shape"], joint, seam)["sealed_spans"] == []


def test_short_clear_stubs_at_the_ends_are_absorbed():
    """A 2 mm weldable stub before the first obstruction is not somewhere
    anyone strikes an arc, and absorbing it must not lose length."""
    shape, joint, seam = synthetic_corner(90.0, seam_length_mm=40.0)
    result = _measure(shape, joint, seam, min_clear_run_m=1.0)
    # An impossible floor: everything must end up on one side or the other.
    assert result["clear_length_m"] + result["blocked_length_m"] == pytest.approx(
        result["seam_length_m"], abs=1e-9
    )


# ------------------------------------------------------------------- pinned


def test_pipe_bracket_coverage_is_pinned(pipe_bracket):
    """The least valuable assertions here, and the ones that catch a silent
    change in the frames or the sampling. Recorded at work 0, travel as
    authored, the shipped Ø16 torch, and the 12 mm stickout the seams carried
    then (`RECORDED_STICKOUT_MM`), stated rather than inherited."""
    expected = {
        "seam_0001": 0.882,
        "seam_0002": 1.000,
        "seam_0003": 0.459,
        "seam_0007": 0.790,
        # 0.775 since 2026-08-12, down 0.006. `access._snap_to_corners` moved
        # this run's boundary onto the corner it landed a bisection's width
        # from, so a sub-0.1 mm sliver moved from weldable to blocked and
        # the covered fraction is measured over a slightly shorter seam.
        # See docs/issues/0001.
        "seam_0011": 0.775,
    }
    for seam_id, want in expected.items():
        joint, seam = _seam(pipe_bracket, seam_id)
        got = _measure(
            pipe_bracket["shape"], joint, seam, standoff_mm=RECORDED_STICKOUT_MM
        )["clear_fraction"]
        assert got == pytest.approx(want, abs=0.005), f"{seam_id}: {got:.3f} != {want}"


def test_the_result_is_reproducible(pipe_bracket):
    """An optimizer built on a non-deterministic oracle cannot be debugged."""
    joint, seam = _seam(pipe_bracket, "seam_0003")
    first = _measure(pipe_bracket["shape"], joint, seam)
    second = _measure(pipe_bracket["shape"], joint, seam)
    assert first == second


# --------------------------------------------------------- the search's oracle


def test_the_lattice_conserves_the_seam(pipe_bracket):
    """Every oracle sample belongs to exactly one decision knot, and the arc
    they speak for adds up to the seam.

    The ends carry HALF a step because they sit on the boundary rather than
    inside it. Without that the total comes out one step long and every
    coverage percentage is quietly inflated -- by 2.4% on an 81 mm seam, which
    is small enough to look like measurement noise and never be questioned.
    """
    from seam_worker.reach.clearance import build_lattice
    from seam_worker.reach.orient import bin_values

    joint, seam = _seam(pipe_bracket, "seam_0007")
    lattice = build_lattice(
        pipe_bracket["shape"],
        joint,
        seam,
        work_bins=bin_values(-2.0, 2.0, 2.0),
        travel_bins=[10.0],
        standoffs_mm=[12.0],
    )
    assert sum(lattice["arc_mm"]) == pytest.approx(lattice["seam_length_mm"], abs=1e-6)
    assert len(lattice["knot_fractions"]) == len(lattice["arc_mm"])
    assert len(lattice["spacing_mm"]) == len(lattice["arc_mm"]) - 1
    # No cell may claim more arc than its knot owns.
    for knot, cells in enumerate(lattice["clear_mm"][0]):
        for row in cells:
            for value in row:
                assert value <= lattice["arc_mm"][knot] + 1e-9


def test_the_optimum_is_never_worse_than_the_authored_profile(pipe_bracket):
    """The one guarantee the operator needs before pressing the button.

    Measured in the SAME lattice, so this compares like with like: the authored
    cell is a cell of the table the search chose from, not a separate run.
    """
    from seam_worker.reach.clearance import build_lattice
    from seam_worker.reach.orient import bin_values, optimise

    joint, seam = _seam(pipe_bracket, "seam_0007")
    work = bin_values(-6.0, 6.0, 2.0)
    travel = bin_values(0.0, 20.0, 5.0)
    lattice = build_lattice(
        pipe_bracket["shape"], joint, seam, work_bins=work, travel_bins=travel,
        standoffs_mm=[12.0],
    )
    n = len(lattice["arc_mm"])
    authored = sum(
        lattice["clear_mm"][0][k][work.index(0.0)][travel.index(10.0)] for k in range(n)
    )
    best = optimise(
        clear_mm=lattice["clear_mm"][0],
        work_bins=work,
        travel_bins=travel,
        arc_mm=lattice["arc_mm"],
        spacing_mm=lattice["spacing_mm"],
        authored_work=[0.0] * n,
        authored_travel=[10.0] * n,
        work_limits={"max_rate_deg_per_mm": 0.5},
        travel_limits={"max_rate_deg_per_mm": 0.5},
    )
    assert best["clear_mm"] >= authored - 1e-9
    # On this fillet there is real room: the travel angle sweeps and buys arc
    # that a constant profile cannot reach.
    assert best["clear_mm"] > authored, "pipe_bracket's column fillet has room to improve"
    assert best["clear_mm"] <= lattice["seam_length_mm"] + 1e-9


def test_one_set_of_casts_serves_every_standoff(pipe_bracket):
    """Why standoff is an outer loop and not a third lattice axis.

    Four standoffs come out of the same rays, and pulling the torch back never
    clears less -- the same monotonicity the direct check has, now visible
    through the table the optimiser searches.
    """
    from seam_worker.reach.clearance import build_lattice
    from seam_worker.reach.orient import bin_values

    joint, seam = _seam(pipe_bracket, "seam_0007")
    work = bin_values(-2.0, 2.0, 2.0)
    standoffs = [8.0, 12.0, 16.0, 20.0]
    lattice = build_lattice(
        pipe_bracket["shape"], joint, seam, work_bins=work, travel_bins=[10.0],
        standoffs_mm=standoffs,
    )
    # The cost is one set of casts, not one per standoff.
    assert lattice["rays_cast"] == lattice["samples"] * len(work) * 1 * lattice["ray_count"]

    totals = [sum(sum(sum(r) for r in knot) for knot in slab) for slab in lattice["clear_mm"]]
    assert totals == sorted(totals), f"pulling back should never clear less: {totals}"


def test_the_lattice_is_reproducible(pipe_bracket):
    from seam_worker.reach.clearance import build_lattice
    from seam_worker.reach.orient import bin_values

    joint, seam = _seam(pipe_bracket, "seam_0007")
    args = {
        "work_bins": bin_values(-2.0, 2.0, 2.0),
        "travel_bins": [10.0],
        "standoffs_mm": [12.0],
    }
    first = build_lattice(pipe_bracket["shape"], joint, seam, **args)
    second = build_lattice(pipe_bracket["shape"], joint, seam, **args)
    assert first == second


# ------------------------------------------------- building the lattice in bits


def test_chunk_ranges_tile_the_axis_exactly():
    """No gap and no overlap, at every size including the awkward ones."""
    from seam_worker.reach.clearance import chunk_ranges

    for samples in (1, 2, 3, 7, 41, 92, 100):
        for chunks in (1, 2, 3, 5, 8, 16, 200):
            ranges = chunk_ranges(samples, chunks)
            assert ranges[0][0] == 0
            assert ranges[-1][1] == samples
            for (_, hi), (lo, _) in itertools.pairwise(ranges):
                assert hi == lo
            assert all(hi > lo for lo, hi in ranges)
            # Never more pieces than asked for, and never a wasted empty one.
            assert len(ranges) <= min(chunks, samples)
    assert chunk_ranges(0, 4) == []


def test_a_lattice_built_in_pieces_equals_one_built_whole(pipe_bracket):
    """The claim the whole job rests on: splitting the sample axis across
    workers is addition, not approximation.

    Checked against the SAME arguments run in one piece, so a difference is a
    difference in the merge and nowhere else. This is the test that would catch
    the ends' half-share being counted by two chunks -- the one error in this
    file that would look like a plausible coverage number.
    """
    from seam_worker.reach.clearance import build_lattice, chunk_ranges, merge_lattices
    from seam_worker.reach.orient import bin_values

    joint, seam = _seam(pipe_bracket, "seam_0007")
    args = {
        "work_bins": bin_values(-2.0, 2.0, 2.0),
        "travel_bins": [10.0],
        "standoffs_mm": [10.0, 14.0],
    }
    whole = build_lattice(pipe_bracket["shape"], joint, seam, **args)
    parts = [
        build_lattice(pipe_bracket["shape"], joint, seam, index_range=r, **args)
        for r in chunk_ranges(whole["samples"], 5)
    ]
    assert len(parts) == 5
    merged = merge_lattices(parts)
    assert merged["clear_mm"] == whole["clear_mm"]
    assert merged["arc_mm"] == whole["arc_mm"]
    assert merged["rays_cast"] == whole["rays_cast"]
    assert merged["index_range"] == [0, whole["samples"]]


def test_merging_refuses_a_partition_with_a_hole():
    """A gap would read as a stretch of seam nothing clears, and an overlap
    would push coverage past 100%. Both are refused rather than summed."""
    from seam_worker.reach.clearance import ClearanceError, merge_lattices

    def chunk(lo, hi):
        return {
            "knot_fractions": [0.0, 1.0], "spacing_mm": [10.0], "seam_length_mm": 10.0,
            "work_bins": [0.0], "travel_bins": [0.0], "standoffs_mm": [10.0],
            "ray_count": 5, "sample_step_mm": 2.0, "samples": 6,
            "index_range": [lo, hi], "rays_cast": 1, "warnings": [],
            "clear_mm": [[[[1.0]], [[1.0]]]], "arc_mm": [1.0, 1.0],
        }

    assert merge_lattices([chunk(0, 3), chunk(3, 6)])["arc_mm"] == [2.0, 2.0]
    with pytest.raises(ClearanceError, match="do not tile"):
        merge_lattices([chunk(0, 2), chunk(3, 6)])
    with pytest.raises(ClearanceError, match="do not tile"):
        merge_lattices([chunk(0, 4), chunk(3, 6)])
    with pytest.raises(ClearanceError, match="cover 4 of 6"):
        merge_lattices([chunk(0, 4)])
    with pytest.raises(ClearanceError, match="disagree on 'work_bins'"):
        merge_lattices([chunk(0, 3), {**chunk(3, 6), "work_bins": [2.0]}])
    with pytest.raises(ClearanceError, match="no lattice chunks"):
        merge_lattices([])


# -------------------------------------------- a seam reduced to search inputs


def _knots():
    return [0.0, 0.25, 0.5, 0.75, 1.0]


def test_the_authored_profile_is_read_at_the_decision_knots():
    """Not at the oracle samples: the authored profile is what the search ties
    toward, and it only ever decides at a knot."""
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "piecewise_linear", "knots": [[0.0, -4.0], [1.0, 4.0]]},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
        }
    }
    got = search_inputs(seam, _knots())
    assert got["authored_work"] == [-4.0, -2.0, 0.0, 2.0, 4.0]
    assert got["authored_travel"] == [10.0] * 5

    # And the search window FOLLOWS the ramp rather than standing still: with
    # the default 15 degrees of deviation, the admissible set at each knot is
    # centred on what was authored there. That is the whole difference between
    # a deviation and an absolute band.
    assert got["work_limits"]["min"] == [-19.0, -17.0, -15.0, -13.0, -11.0]
    assert got["work_limits"]["max"] == [11.0, 13.0, 15.0, 17.0, 19.0]


def test_declared_limits_become_a_per_knot_mask():
    """Varying limits are the reason they are scalar functions -- the window
    narrows where a flange rises beside the seam -- so the mask has to vary too."""
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
            "limits": {
                "work_angle_deg": {
                    "min": {"kind": "constant", "value": -6.0},
                    "max": {"kind": "piecewise_linear", "knots": [[0.0, 6.0], [1.0, 0.0]]},
                    "max_rate_deg_per_mm": 0.5,
                }
            },
        }
    }
    got = search_inputs(seam, _knots())
    assert got["work_limits"]["min"] == [-6.0] * 5
    assert got["work_limits"]["max"] == [6.0, 4.5, 3.0, 1.5, 0.0]
    assert got["work_limits"]["max_rate_deg_per_mm"] == 0.5
    # The bins span the union of the window, and the mask narrows it per knot.
    assert got["work_bins"] == [-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0]


def test_a_profile_outside_its_own_declared_limit_is_refused():
    """A declared limit is a hard mask, so an authored profile outside it is not
    something to widen around -- it is a weld that does not satisfy its own
    procedure. Refused, because silently moving it would be the optimiser
    choosing which of the two the operator meant."""
    from seam_worker.reach.clearance import ClearanceError, search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 9.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
            "limits": {
                "work_angle_deg": {
                    "min": {"kind": "constant", "value": -6.0},
                    "max": {"kind": "constant", "value": 6.0},
                }
            },
        }
    }
    with pytest.raises(ClearanceError, match="outside its declared limit"):
        search_inputs(seam, _knots())


def test_the_window_is_centred_on_what_was_authored():
    """A deviation is measured FROM the profile, so an unusual authored angle
    is the middle of its own window rather than the edge of somebody's band."""
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 34.0},
        }
    }
    got = search_inputs(seam, _knots(), travel_deviation_deg=10.0)
    assert got["travel_limits"]["min"] == [24.0] * 5
    assert got["travel_limits"]["max"] == [44.0] * 5
    # Bracketed by whole bins, so the authored value is within half a bin of a
    # real cell rather than outside the top one.
    assert max(got["travel_bins"]) >= 44.0 and min(got["travel_bins"]) <= 24.0


def test_zero_deviation_still_leaves_the_authored_profile_reachable():
    """"Do not move it" must not mean "no legal cell exists".

    A grid anchored at zero always has a multiple within half a step of any
    value, so the window never closes tighter than that -- otherwise a weld
    whose authored angle falls between two bins would have an empty lattice and
    the search would refuse a weld with nothing wrong with it.
    """
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 3.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
        }
    }
    got = search_inputs(seam, _knots(), work_deviation_deg=0.0)
    admissible = [
        b for b in got["work_bins"] if got["work_limits"]["min"][0] <= b <= got["work_limits"]["max"][0]
    ]
    assert admissible, "an empty lattice would refuse a weld with nothing wrong with it"
    assert all(abs(b - 3.0) <= 1.0 + 1e-9 for b in admissible)


def test_a_weld_can_pin_its_own_deviation_tighter_than_the_default():
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
            "limits": {"work_angle_deg": {"max_deviation_deg": 4.0}},
        }
    }
    got = search_inputs(seam, _knots())
    assert got["work_limits"]["min"] == [-4.0] * 5
    assert got["work_limits"]["max"] == [4.0] * 5
    assert got["travel_limits"]["max"] == [50.0] * 5, "the other axis keeps the default"


def test_a_split_deviation_bounds_each_direction_on_its_own():
    """`max_deviation_plus_deg` / `max_deviation_minus_deg`: the two directions
    are rarely worth the same -- dragging further is often free where pushing
    at all fouls the nozzle -- so the document may price them separately. Both
    are magnitudes, and when either is present the pair WINS over the
    symmetric shorthand rather than intersecting with it: one knob per claim.
    """
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 10.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
            "limits": {
                "work_angle_deg": {
                    "max_deviation_plus_deg": 6.0,
                    "max_deviation_minus_deg": 2.0,
                    # Ignored: the split fields are present.
                    "max_deviation_deg": 30.0,
                }
            },
        }
    }
    got = search_inputs(seam, _knots())
    assert got["work_limits"]["min"] == [8.0] * 5
    assert got["work_limits"]["max"] == [16.0] * 5


def test_a_split_deviation_missing_one_side_defaults_that_side_only():
    """Half a split falls back to the settings default for the OTHER half --
    stating "+2" is a claim about up, not a claim that down is forbidden."""
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
            "limits": {"work_angle_deg": {"max_deviation_plus_deg": 2.0}},
        }
    }
    got = search_inputs(seam, _knots(), work_deviation_deg=15.0)
    assert got["work_limits"]["max"] == [2.0] * 5
    assert got["work_limits"]["min"] == [-15.0] * 5


def test_a_declared_band_and_a_deviation_intersect():
    """Both apply. They are different claims -- where the angle may BE, and how
    far it may MOVE -- and the admissible set is the intersection."""
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 10.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
            "limits": {
                "work_angle_deg": {
                    "min": {"kind": "constant", "value": 0.0},
                    "max": {"kind": "constant", "value": 12.0},
                    "max_deviation_deg": 6.0,
                }
            },
        }
    }
    got = search_inputs(seam, _knots())
    # deviation gives [4, 16]; the band gives [0, 12]; the answer is [4, 12].
    assert got["work_limits"]["min"] == [4.0] * 5
    assert got["work_limits"]["max"] == [12.0] * 5


def test_an_undeclared_rate_limit_is_reported_rather_than_assumed():
    """A free rate limit is taken literally by the search, and it should be:
    nothing was declared, so nothing is violated. But on pipe_bracket's column
    fillet that produced a travel profile stepping 20 -> 10 in one 10 mm knot
    and down to 0 in another, because both steps bought arc and J3 only breaks
    ties. Right answer, wrong profile to weld -- so the operator is told which
    of the two they are looking at.

    ONCE, however many axes are loose. It was emitted per axis first, and four
    lines of amber under a two-line result made the caveat louder than the
    answer."""
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 10.0},
        }
    }
    warnings = search_inputs(seam, _knots())["warnings"]
    assert warnings == [
        (
            "no rate limit on the work or travel angle, so the search may return "
            "a profile that changes faster than a torch can follow"
        )
    ], warnings

    seam["orientation"]["limits"] = {"work_angle_deg": {"max_rate_deg_per_mm": 0.5}}
    only_travel = search_inputs(seam, _knots())["warnings"]
    assert only_travel == [
        (
            "no rate limit on the travel angle, so the search may return a "
            "profile that changes faster than a torch can follow"
        )
    ], only_travel

    seam["orientation"]["limits"]["travel_angle_deg"] = {"max_rate_deg_per_mm": 0.5}
    assert not search_inputs(seam, _knots())["warnings"]


def test_a_single_bin_axis_says_so_rather_than_looking_searched():
    from seam_worker.reach.clearance import search_inputs

    seam = {
        "orientation": {
            "work_angle_deg": {"kind": "constant", "value": 0.0},
            "travel_angle_deg": {"kind": "constant", "value": 0.0},
            "limits": {
                "work_angle_deg": {
                    "min": {"kind": "constant", "value": -0.5},
                    "max": {"kind": "constant", "value": 0.5},
                }
            },
        }
    }
    got = search_inputs(seam, _knots())
    admissible = [
        b for b in got["work_bins"] if -0.5 - 1e-9 <= b <= 0.5 + 1e-9
    ]
    assert admissible == [0.0]
    assert any("single" in w for w in got["warnings"]), got["warnings"]


def test_an_index_range_outside_the_seam_is_refused(pipe_bracket):
    from seam_worker.reach.clearance import ClearanceError, build_lattice

    joint, seam = _seam(pipe_bracket, "seam_0007")
    with pytest.raises(ClearanceError, match="outside the"):
        build_lattice(
            pipe_bracket["shape"], joint, seam, work_bins=[0.0], travel_bins=[0.0],
            standoffs_mm=[10.0], index_range=(0, 100000),
        )


# ------------------------------------------------- boundaries an arc is struck at


def test_a_clear_run_contains_no_colliding_arc(pipe_bracket):
    """The boundary of a clear run is not a statistic, it is a weld start.

    `refine` bisects between a proven-clear sample and a proven-blocked one.
    Returning the midpoint of the final bracket is the better ESTIMATOR of the
    true transition and the wrong answer here: it is unbiased, so half the time
    it lands inside the blocked region. Three halvings of a 2 mm step leave a
    0.25 mm bracket, so the midpoint can sit 0.125 mm the wrong side.

    Measured before the bias was added: 9 of these 36 boundaries left up to
    0.10 mm of colliding arc inside a run the operator had just split off as
    weldable. It is invisible on screen and it is a collision.

    Probed 0.02 mm INSIDE each run -- inside, because the boundary itself is
    the transition and either answer there is defensible.
    """
    rays = intersector(pipe_bracket["shape"])
    checked = 0
    for seam_id in pipe_bracket["seams"]:
        for work in (0.0, 3.0, -3.0):
            joint, seam = _seam(pipe_bracket, seam_id, work_angle_deg=work)
            result = _measure(pipe_bracket["shape"], joint, seam)
            if not result["clear_runs"] or result["clear_fraction"] in (0.0, 1.0):
                continue
            geometry = SeamGeometry(joint, seam)
            length_mm = result["seam_length_m"] * MM_PER_M
            for run in result["clear_runs"]:
                for edge, inward in ((run[0], +1), (run[1], -1)):
                    if edge <= 1e-9 or edge >= 1.0 - 1e-9:
                        continue
                    checked += 1
                    inside = edge + inward * 0.02 / length_mm
                    assert not collides_at(
                        rays, geometry, DEFAULT_TORCH, result["standoff_mm"],
                        inside, FINE_RAY_COUNT,
                    ), (
                        f"{seam_id} at work={work}: the clear run boundary {edge:.6f} has "
                        "colliding arc inside it, so a split here leaves a stub the torch "
                        "cannot weld"
                    )
    assert checked >= 20, f"only {checked} boundaries had a transition to check"


def test_the_boundary_is_the_last_proven_clear_sample(pipe_bracket):
    """Not the midpoint of the bracket.

    Pins the estimator itself rather than only its consequence, so a future
    change back to a midpoint fails here with the reason rather than as a
    mysterious collision on one fixture.
    """
    from seam_worker.reach import clearance as module

    source = inspect.getsource(module.clearance)
    assert "return lo" in source, (
        "the clear/blocked bisection no longer returns the proven-clear side; "
        "see test_a_clear_run_contains_no_colliding_arc for why that matters"
    )


# --------------------------------------------- running the seam the other way


def test_a_mirrored_lattice_equals_a_reversed_one(pipe_bracket):
    """The identity the reverse search rests on, end to end.

    `apply_angles` gives `u(-t, w, tr) == u(t, -w, -tr)`, so a reversed seam's
    clearance is the forward table read at the negated angles. That is checked
    against `frames.apply_angles` in `tests/programmer/test_frames.py`; what this checks is
    that it survives everything between -- the seam frames, the barrel axes, the
    ray fan and the arc accumulation.

    If it ever stops holding, reversal has to start casting its own rays, and
    the optimiser's direction loop stops being free.
    """
    from seam_worker.reach.orient import mirror_clear

    work = [-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0]
    travel = [-10.0, -5.0, 0.0, 5.0, 10.0]
    joint, seam = _seam(pipe_bracket, "seam_0007")
    common = {"work_bins": work, "travel_bins": travel, "standoffs_mm": [12.0]}

    forward = build_lattice(pipe_bracket["shape"], joint, {**seam, "direction": "forward"}, **common)
    reverse = build_lattice(pipe_bracket["shape"], joint, {**seam, "direction": "reverse"}, **common)
    mirrored = [mirror_clear(part, work, travel) for part in forward["clear_mm"]]

    assert mirrored == reverse["clear_mm"], "the mirror is not the reversed lattice"
    # ...and the two directions genuinely differ, or the assertion above holds
    # for the boring reason.
    assert forward["clear_mm"] != reverse["clear_mm"]


def test_symmetric_bins_are_only_paid_for_when_asked(pipe_bracket):
    """Mirroring needs the negated angle to be in the table, which means bins
    symmetric about zero. That is wider than the window around a drag angle, so
    it costs rays -- and is therefore opt-in rather than always on."""
    _, seam = _seam(pipe_bracket, "seam_0007")
    knots = [0.0, 0.5, 1.0]
    narrow = search_inputs(seam, knots)
    wide = search_inputs(seam, knots, both_directions=True)

    assert len(wide["travel_bins"]) >= len(narrow["travel_bins"])
    for axis in ("work_bins", "travel_bins"):
        assert wide[axis] == [-v for v in reversed(wide[axis])], f"{axis} is not symmetric"
    # The authored profile and its window are untouched: 10 degrees of drag is
    # 10 degrees of drag whichever way round the seam runs.
    assert wide["authored_travel"] == narrow["authored_travel"]
