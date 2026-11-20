"""Seam frames, and the angle conventions that ride on them.

`SEAM-MODEL.md` section 5 executed against real detected geometry rather than a
synthetic curve. The frame is what everything visual and everything the planner
does hangs off, and its failure modes are quiet: a sign flipped here shows up
as a torch pointing into the workpiece three stages later.

The travel-angle sign in particular is a coin flip that `DECISIONS.md` D4 marks
as still open, so it is verified numerically here and never assumed.
"""

import itertools
import math

import numpy as np
import pytest
from seam_worker.seam.detect import detect_joints
from seam_worker.seam.frames import apply_angles, seam_frames

MM = 1000.0
COLUMN_JOINT = "weld_joint_0003"


@pytest.fixture(scope="module")
def column_joint(pipe_bracket_topology):
    return next(
        j for j in detect_joints(pipe_bracket_topology)["weld_joints"] if j["id"] == COLUMN_JOINT
    )


@pytest.fixture(scope="module")
def frames(column_joint):
    return seam_frames(
        column_joint["weld_line"],
        {"mode": "rail_curve", "rail": column_joint["bisector_rail"]["curve"]},
        samples=128,
    )


# --------------------------------------------------------------------------
# the frame itself
# --------------------------------------------------------------------------


def test_frame_is_orthonormal_everywhere(frames):
    assert not frames["warnings"]
    worst = 0.0
    for row in frames["frames"]:
        t = np.array(row["tangent_xyz"])
        r = np.array(row["reference_xyz"])
        b = np.array(row["binormal_xyz"])
        worst = max(
            worst,
            abs(np.dot(t, r)),
            abs(np.dot(t, b)),
            abs(np.dot(r, b)),
            abs(np.linalg.norm(t) - 1.0),
            abs(np.linalg.norm(r) - 1.0),
            abs(np.linalg.norm(b) - 1.0),
        )
    assert worst < 1e-8, f"worst orthonormality error {worst:.2e}"


def test_binormal_is_t_cross_r(frames):
    """b = t x r, the handedness SEAM-MODEL section 5 fixes."""
    for row in frames["frames"]:
        t = np.array(row["tangent_xyz"])
        r = np.array(row["reference_xyz"])
        # components are stored rounded to 9 dp, so the cross of the rounded
        # vectors differs from the rounded cross at that scale
        assert np.cross(t, r) == pytest.approx(np.array(row["binormal_xyz"]), abs=1e-8)


def test_reference_is_the_45_degree_bisector_on_a_90_degree_fillet(frames, column_joint):
    """Dihedral is 90 degrees all the way round this loop, so the rail
    direction sits at 45 degrees out of the seam plane at every sample --
    including through the corner arcs, where its horizontal part rotates but
    its vertical part must not."""
    assert column_joint["dihedral_angle_deg"]["value"] == pytest.approx(90.0)
    for row in frames["frames"]:
        assert row["reference_xyz"][2] == pytest.approx(math.sqrt(0.5), abs=1e-6), row["s"]


def test_points_lie_on_the_weld_line(frames, column_joint):
    """Frames are sampled from the curve, so they must reproduce its length."""
    points = np.array([row["point_xyz_m"] for row in frames["frames"]])
    walked = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    # chord sum under-reports arc length slightly on the corner arcs
    assert walked == pytest.approx(column_joint["weld_line"]["length_m"], rel=2e-3)
    assert frames["length_m"] == pytest.approx(column_joint["weld_line"]["length_m"], rel=1e-9)


def test_curvature_is_zero_on_the_straights_and_finite_on_the_corners(frames):
    curvatures = [row["curvature_1_per_m"] for row in frames["frames"]]
    assert min(curvatures) == pytest.approx(0.0, abs=1e-6), "straight runs"
    assert max(curvatures) > 50.0, "corner arcs should be strongly curved"


# --------------------------------------------------------------------------
# angle conventions -- verified, never assumed
# --------------------------------------------------------------------------


def test_positive_travel_angle_is_drag():
    """DECISIONS.md D4, still marked open, so it is pinned by measurement.

    t=+x, r=+z, work 0, travel +15 deg. u must lean toward +t, putting the
    torch body AHEAD of the weld point and the electrode pointing back over
    completed weld.
    """
    t = np.array([1.0, 0.0, 0.0])
    r = np.array([0.0, 0.0, 1.0])
    u = apply_angles(t, r, 0.0, 15.0)
    assert u == pytest.approx([math.sin(math.radians(15)), 0.0, math.cos(math.radians(15))], abs=1e-12)
    assert np.dot(u, t) > 0, "positive travel must be drag/backhand"

    pushed = apply_angles(t, r, 0.0, -15.0)
    assert np.dot(pushed, t) < 0, "negative travel must be push/forehand"


def test_zero_angles_leave_the_torch_on_the_reference():
    t = np.array([1.0, 0.0, 0.0])
    r = np.array([0.0, 0.0, 1.0])
    assert apply_angles(t, r, 0.0, 0.0) == pytest.approx(r, abs=1e-12)


def test_work_angle_rotates_within_the_cross_section_plane():
    """Work angle must not tilt the torch along travel -- that is travel's job.
    So the component along t stays zero however far it rolls."""
    t = np.array([1.0, 0.0, 0.0])
    r = np.array([0.0, 0.0, 1.0])
    for work in (-60.0, -20.0, 20.0, 45.0, 60.0):
        u = apply_angles(t, r, work, 0.0)
        assert np.dot(u, t) == pytest.approx(0.0, abs=1e-12), work
        assert np.linalg.norm(u) == pytest.approx(1.0, abs=1e-12)


def test_spin_stays_free(frames):
    """R2. The frame determines position and electrode DIRECTION -- five
    constraints. Rotation about the electrode axis is never produced here, and
    that missing sixth number is the freedom the planner needs."""
    for row in frames["frames"]:
        assert set(row) == {
            "s",
            "point_xyz_m",
            "tangent_xyz",
            "reference_xyz",
            "binormal_xyz",
            "curvature_1_per_m",
        }


def test_weave_axis_is_perpendicular_to_travel_and_electrode(frames):
    """b' carries weave displacement, so it must be orthogonal to both the
    travel direction and the electrode axis at every real sample and every
    angle -- otherwise weave would modulate standoff or travel speed."""
    worst = 0.0
    for row in frames["frames"][::8]:
        t = np.array(row["tangent_xyz"])
        r = np.array(row["reference_xyz"])
        for work in (-45.0, 0.0, 45.0):
            for travel in (-20.0, 0.0, 20.0):
                from seam_worker.seam.frames import rotation

                r_prime = rotation(t, math.radians(work)) @ r
                b_prime = np.cross(t, r_prime)
                u = apply_angles(t, r, work, travel)
                worst = max(worst, abs(np.dot(b_prime, t)), abs(np.dot(b_prime, u)))
    assert worst < 1e-12, f"b' not orthogonal (worst {worst:.2e})"


# --------------------------------------------------------------------------
# rail correspondence
# --------------------------------------------------------------------------


def test_rail_correspondence_is_per_segment(column_joint):
    """Segment counts must match, or per-segment correspondence cannot resolve
    and the rail would have to be matched over the whole curve -- which smears
    a corner discontinuity across the corner instead of respecting it."""
    rail = column_joint["bisector_rail"]
    assert len(rail["curve"]["segments"]) == len(column_joint["weld_line"]["segments"])

    result = seam_frames(
        column_joint["weld_line"], {"mode": "rail_curve", "rail": rail["curve"]}, samples=256
    )
    assert not result["warnings"]
    for row in result["frames"]:
        assert np.linalg.norm(row["reference_xyz"]) == pytest.approx(1.0, abs=1e-9)


def test_rail_continuity_tracks_tangent_continuity(pipe_bracket_topology, dog_box_topology):
    """`continuous` is a real geometric signal, not bookkeeping.

    The rail steps where the groove normal jumps, and only there:

      - dog_box dogbone loops: the 270 degree reliefs are undercut lobes, NOT
        tangent to the adjacent lines, so the normal jumps -> 20 mm step.
      - pipe_bracket column: a rounded square whose corner radii ARE tangent to
        the flat sides, so the normal is continuous -> no step, even though it
        has exactly the same 4-arc-plus-4-line structure.
      - gusset rectangles: sharp 90 degree corners -> 20 mm step.
      - single closed circles: no junctions at all -> continuous.

    Segment counts and curve types cannot tell these apart; only the normals
    can. A first version of this test assumed 4 arcs + 4 lines meant
    discontinuous and was wrong about the column for exactly that reason.
    """
    observed = {}
    for name, topology in (("dog_box", dog_box_topology), ("pipe_bracket", pipe_bracket_topology)):
        for joint in detect_joints(topology)["weld_joints"]:
            rail = joint["bisector_rail"]
            segments = rail["curve"]["segments"]
            gaps = [
                float(
                    np.linalg.norm(
                        np.subtract(
                            seg["control_points_xyz_m"][-1],
                            segments[(i + 1) % len(segments)]["control_points_xyz_m"][0],
                        )
                    )
                )
                for i, seg in enumerate(segments)
            ]
            observed[f"{name}:{joint['id']}"] = (rail["continuous"], max(gaps))

    for key, (continuous, gap) in observed.items():
        if continuous:
            assert gap == pytest.approx(0.0, abs=1e-9), f"{key} claims continuous but steps"
        else:
            assert gap > 1e-6, f"{key} claims discontinuous but does not step"

    # the distinction that matters: same structure, opposite continuity
    assert observed["dog_box:weld_joint_0002"][0] is False, "dogbone lobes are undercut"
    assert observed["pipe_bracket:weld_joint_0003"][0] is True, "rounded square is tangent"
    assert observed["dog_box:weld_joint_0004"][0] is True, "single circle has no junctions"
    assert observed["pipe_bracket:weld_joint_0001"][0] is False, "sharp rectangle corners"


def test_fixed_vector_mode_orthogonalises_against_the_tangent(column_joint):
    """A constant reference always yields a VALID frame -- it is orthogonalised
    at every point. What it does not yield is a constant angle to the surface,
    which is the documented cost of `fixed_vector`."""
    result = seam_frames(
        column_joint["weld_line"],
        {"mode": "fixed_vector", "fixed_direction_xyz": [0.0, 0.0, 1.0]},
        samples=64,
    )
    assert not result["warnings"]
    for row in result["frames"]:
        t = np.array(row["tangent_xyz"])
        r = np.array(row["reference_xyz"])
        assert np.dot(t, r) == pytest.approx(0.0, abs=1e-9)
        assert np.linalg.norm(r) == pytest.approx(1.0, abs=1e-9)


def test_a_degenerate_reference_is_reported_not_hidden():
    """`Q - P` parallel to the tangent is the one case the construction cannot
    handle. It must say so rather than emit a plausible wrong frame."""
    straight = {
        "segments": [
            {
                "degree": 1,
                "control_points_xyz_m": [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
                "knots_u": [0.0, 0.0, 1.0, 1.0],
            }
        ]
    }
    result = seam_frames(
        straight, {"mode": "fixed_vector", "fixed_direction_xyz": [1.0, 0.0, 0.0]}, samples=8
    )
    assert result["warnings"], "a reference parallel to the tangent must warn"
    assert "parallel" in result["warnings"][0]


def test_adaptive_sampling_puts_frames_where_the_curve_bends(column_joint):
    result = seam_frames(
        column_joint["weld_line"],
        {"mode": "rail_curve", "rail": column_joint["bisector_rail"]["curve"]},
        max_chord_m=50e-6,
        max_angle_rad=0.05,
    )
    curved = [r for r in result["frames"] if r["curvature_1_per_m"] > 1.0]
    straight = [r for r in result["frames"] if r["curvature_1_per_m"] <= 1.0]
    assert curved and straight
    # the corners are a small fraction of the length but get a large share of frames
    arc_fraction = sum(
        s["length_m"] for s in column_joint["weld_line"]["segments"]
        if s.get("analytic", {}).get("kind") == "arc"
    ) / column_joint["weld_line"]["length_m"]
    assert len(curved) / len(result["frames"]) > arc_fraction


def test_refinement_never_returns_fewer_frames_than_the_uniform_grid(column_joint):
    """The tolerances ADD to the grid. They used to replace it, and replacing it
    decimates: `sample_adaptive` caps its step at an eighth of the curve, so a
    straight came back with nine frames and took the comb and the playhead with
    it."""
    reference = {"mode": "rail_curve", "rail": column_joint["bisector_rail"]["curve"]}
    plain = seam_frames(column_joint["weld_line"], reference, samples=160)
    refined = seam_frames(
        column_joint["weld_line"], reference, samples=160, max_chord_m=50e-6, max_angle_rad=0.05
    )
    assert len(plain["frames"]) == 160
    assert len(refined["frames"]) > len(plain["frames"])
    kept = {row["s"] for row in refined["frames"]}
    assert all(row["s"] in kept for row in plain["frames"]), "the uniform grid is a floor"


def test_refinement_is_driven_by_the_rail_not_the_seam(column_joint):
    """The rail is the curve that gets drawn from this table, and it does not
    advance at the seam's rate: on this joint every corner's rail segment is
    2.4x its seam segment, so a uniform step in seam arc length walks the rail
    2.4x as far. The seam's own chords are inside tolerance exactly where the
    rail's are worst, so a refinement that measured only the seam would do
    nothing at all here."""
    from seam_worker.seam.curve import Curve
    from seam_worker.seam.frames import _rail_point

    weld_line = column_joint["weld_line"]
    rail_curve = column_joint["bisector_rail"]["curve"]
    seam, rail = Curve(weld_line), Curve(rail_curve)
    ratios = [r / s for r, s in zip(rail.lengths, seam.lengths)]
    assert max(ratios) > 2.0, "fixture no longer has a rail that outruns its seam"

    def worst_rail_sag(rows):
        worst = 0.0
        for a, b in itertools.pairwise(rows):
            if seam.locate(a["s"])[0] != seam.locate(b["s"])[0]:
                continue  # a corner step (D6), not a facet
            lo, hi = _rail_point(rail, seam, a["s"]), _rail_point(rail, seam, b["s"])
            mid = _rail_point(rail, seam, (a["s"] + b["s"]) / 2)
            worst = max(worst, float(np.linalg.norm(mid - (lo + hi) / 2)))
        return worst

    reference = {"mode": "rail_curve", "rail": rail_curve}
    plain = seam_frames(weld_line, reference, samples=160)
    refined = seam_frames(
        weld_line, reference, samples=160, max_chord_m=50e-6, max_angle_rad=0.05
    )
    # The seam alone would have reported itself fine: 50 um is the mesher's own
    # linear tolerance and the uniform grid already meets it on the seam.
    assert worst_rail_sag(plain["frames"]) > 30e-6
    assert worst_rail_sag(refined["frames"]) < worst_rail_sag(plain["frames"]) / 3


@pytest.fixture(scope="module")
def stepping_joint():
    """A joint whose rail is genuinely discontinuous.

    `pipe_bracket`'s corners are fillet radii, so its bisector turns through
    them and every rail there comes back `continuous`. The step this is about
    needs a sharp corner, which `bracket_a` has.
    """
    from seam_worker import paths
    from seam_worker.cad.topology import extract_topology

    step = paths.FIXTURES_DIR / "bracket_a.step"
    if not step.exists():
        pytest.skip(f"fixture STEP not present: {step}")
    joints = detect_joints(extract_topology(step))["weld_joints"]
    return next(
        j for j in joints if (j.get("bisector_rail") or {}).get("continuous") is False
    )


def test_refinement_leaves_the_corner_steps_alone(stepping_joint):
    """The bisector genuinely flips at a corner (D6) and the rail steps with it.
    Refining across that step would chase a discontinuity down to the step floor
    and quietly change what the viewport asserts at a corner, so intervals that
    span a segment boundary are passed through untouched."""
    reference = {"mode": "rail_curve", "rail": stepping_joint["bisector_rail"]["curve"]}
    plain = seam_frames(stepping_joint["weld_line"], reference, samples=160)
    refined = seam_frames(
        stepping_joint["weld_line"], reference, samples=160, max_chord_m=50e-6, max_angle_rad=0.05
    )
    assert not refined["warnings"], "nothing should have hit the step floor"
    # A step chased by bisection would show as frames crowded around the corner.
    lengths = [s["length_m"] for s in stepping_joint["weld_line"]["segments"]]
    boundaries = np.cumsum(lengths)[:-1] / sum(lengths)
    for label, rows in (("plain", plain), ("refined", refined)):
        for edge in boundaries:
            near = [r for r in rows["frames"] if abs(r["s"] - edge) < 0.002]
            assert len(near) <= 2, f"{label}: frames crowding the corner at s={edge}"


def test_self_test_entry_point_passes():
    from seam_worker.seam.frames import self_test

    assert self_test() == 0
