"""Default seam generation: split at obstructions and at corners, grouped.

A joint is the whole boundary where two parts meet. A seam is what you weld, and
it has to respect both what a torch can reach and where the weld line turns.
Three rules this file pins:

- an unreachable stretch SPLITS a joint into separate runs, rather than
  producing one seam with holes in it. A planner reading a seam is entitled to
  assume the arc stays lit from start_s to end_s.
- a corner sharper than the threshold splits a run into pieces. At a C0
  junction the tangent flips and the torch cannot drive through.
- a reachable run that straddles s=0 on a closed loop stays ONE run. It is
  continuous weld; that it crosses the arbitrary point `chain_edges` started
  the wire at is an artifact of parametrisation, not a break in the metal.

The two splits are not interchangeable, which is what `origin` records: pieces
of a group touch end to end and can be re-merged, separate groups have
unweldable material between them.

Assertions here are structural first and pinned second. A count alone would
pass for the wrong reasons the moment a threshold moves.
"""

import json
from itertools import pairwise

import pytest
from jsonschema import Draft202012Validator
from seam_worker.reach.access import annotate
from seam_worker.seam.continuity import junction_positions_s
from seam_worker.seam.detect import detect_joints
from seam_worker.seam.seams import (
    DEFAULT_MAX_CORNER_ANGLE_DEG,
    DEFAULT_MIN_SEAM_LENGTH_M,
    default_seams,
    run_length_m,
    seams_for_joint,
)

MM = 1000.0


@pytest.fixture(scope="module")
def annotated(pipe_bracket_step, pipe_bracket_topology):
    return annotate(pipe_bracket_step, pipe_bracket_topology, detect_joints(pipe_bracket_topology))


@pytest.fixture(scope="module")
def generated(annotated):
    return default_seams(annotated)


def _joint(doc, seam):
    return next(j for j in doc["weld_joints"] if j["id"] == seam["weld_joint_ref"])


def _length_mm(doc, seam):
    joint = _joint(doc, seam)
    span = [seam["span"]["start_s"], seam["span"]["end_s"]]
    return run_length_m(span, joint["weld_line"]["length_m"]) * MM


# --------------------------------------------------------------------------
# the reachability split
# --------------------------------------------------------------------------


def test_obstruction_splits_a_joint_into_separate_groups(annotated, generated):
    """The column loop is interrupted twice by gussets, so it is two GROUPS of
    81.48 mm -- not one 182.83 mm seam with two gaps in it."""
    assert generated["summary"]["joint_count"] == 5
    assert generated["summary"]["group_count"] == 6

    column = [s for s in generated["seams"] if s["weld_joint_ref"] == "weld_joint_0003"]
    assert {s["origin"]["group_id"] for s in column} == {
        "weld_joint_0003.run00",
        "weld_joint_0003.run01",
    }
    for seam in column:
        assert _length_mm(annotated, seam) == pytest.approx(81.48, abs=0.1)
        assert seam["origin"]["split_reason"] == "reachability"


def test_a_run_straddling_the_start_stays_one_seam(annotated, generated):
    """Combined where continuous. Expressed as start_s > end_s.

    This is also the case the corner split must not break: the column is a
    rounded square whose arcs are tangent to its flats, so there is no corner
    to cut at and the wrapping run stays a single piece.
    """
    wrapping = [s for s in generated["seams"] if s["span"]["end_s"] < s["span"]["start_s"]]
    assert len(wrapping) == 1, "exactly one run wraps on this weldment"
    seam = wrapping[0]
    assert _joint(annotated, seam)["weld_line"]["closed"] is True, "only a closed curve may wrap"
    assert _length_mm(annotated, seam) == pytest.approx(81.48, abs=0.1)
    assert seam["origin"]["group_size"] == 1, "a tangent loop has nothing to cut at"
    assert "wraps" in seam["label"]


def test_no_seam_covers_a_blocked_stretch(annotated, generated):
    """The point of the whole exercise."""
    joints = {j["id"]: j for j in annotated["weld_joints"]}
    for seam in generated["seams"]:
        joint = joints[seam["weld_joint_ref"]]
        lo, hi = seam["span"]["start_s"], seam["span"]["end_s"]
        covered = [(lo, hi)] if hi >= lo else [(lo, 1.0), (0.0, hi)]
        for blo, bhi in joint["accessibility"]["blocked_spans"]:
            for clo, chi in covered:
                overlap = min(chi, bhi) - max(clo, blo)
                assert overlap <= 1e-6, f"{seam['id']} covers blocked [{blo:.4f},{bhi:.4f}]"


# --------------------------------------------------------------------------
# the corner split
# --------------------------------------------------------------------------


def test_every_interior_boundary_is_a_real_corner(annotated, generated):
    """Structural, not counted: any piece boundary that is not a run boundary
    must coincide with a junction whose measured angle exceeds the threshold.
    A cut anywhere else would be inventing a break in the weld."""
    joints = {j["id"]: j for j in annotated["weld_joints"]}
    checked = 0
    for seam in generated["seams"]:
        origin = seam["origin"]
        if origin["index_in_group"] == 0:
            continue  # the group's first boundary is the run's own start
        joint = joints[seam["weld_joint_ref"]]
        corners = [
            j["s"]
            for j in junction_positions_s(joint["weld_line"])
            if j["tangent_angle_deg"] > DEFAULT_MAX_CORNER_ANGLE_DEG
        ]
        start = seam["span"]["start_s"]
        assert any(abs(c - start) < 1e-6 for c in corners), (
            f"{seam['id']} starts at {start}, which is not a corner"
        )
        assert origin["boundary_continuity"][0] == "C0"
        checked += 1
    assert checked > 0, "this fixture must actually produce interior boundaries"


def test_pieces_of_a_group_are_contiguous_and_ordered(generated):
    """A group can be re-merged into one seam without inventing geometry, which
    is only true if piece i ends exactly where piece i+1 begins."""
    groups: dict[str, list] = {}
    for seam in generated["seams"]:
        groups.setdefault(seam["origin"]["group_id"], []).append(seam)

    for group_id, seams in groups.items():
        seams.sort(key=lambda s: s["origin"]["index_in_group"])
        assert [s["origin"]["index_in_group"] for s in seams] == list(range(len(seams)))
        assert all(s["origin"]["group_size"] == len(seams) for s in seams)
        for before, after in pairwise(seams):
            assert before["span"]["end_s"] == pytest.approx(
                after["span"]["start_s"], abs=1e-9
            ), f"{group_id} has a gap between pieces"


def test_the_corner_split_is_length_neutral(annotated, generated):
    """Cutting must not lose arc.

    On pipe_bracket four runs used to end a bisection's width (0.031 mm) short
    of the corner they were welded into, leaving stubs that were ABSORBED into
    their neighbour. That absorption is what this test used to count, and it
    was the wrong cure: absorbing forward carried the seam ACROSS the corner,
    so its first micrometre sat on the far face of a 90 degree turn and no arm
    could follow it. Five of the motion corpus's fifteen seams were unplannable
    for that reason -- see docs/issues/0001.

    `access._snap_to_corners` now puts a boundary that lands within the
    bisection's own error onto the corner itself, so the stubs are never
    created. Zero absorbed is the fix working, not the counter breaking.
    """
    uncut = default_seams(annotated, max_corner_angle_deg=180.0)
    assert generated["summary"]["weldable_length_m"] == pytest.approx(
        uncut["summary"]["weldable_length_m"], abs=1e-12
    )
    assert generated["summary"]["pieces_absorbed_below_min_length"] == 0
    shortest = min(_length_mm(annotated, s) for s in generated["seams"])
    assert shortest >= DEFAULT_MIN_SEAM_LENGTH_M * MM


def test_a_threshold_above_every_corner_reproduces_the_unsplit_output(annotated, generated):
    """The split is purely additive: at 180 degrees nothing is cut, and the
    result is what this module produced before corners existed -- one seam per
    reachable run, with the same spans."""
    uncut = default_seams(annotated, max_corner_angle_deg=180.0)
    assert uncut["summary"]["seam_count"] == 6
    assert uncut["summary"]["group_count"] == 6
    assert all(s["origin"]["group_size"] == 1 for s in uncut["seams"])

    spans = {s["origin"]["group_id"]: s["span"] for s in uncut["seams"]}
    groups: dict[str, list] = {}
    for seam in generated["seams"]:
        groups.setdefault(seam["origin"]["group_id"], []).append(seam)
    for group_id, pieces in groups.items():
        pieces.sort(key=lambda s: s["origin"]["index_in_group"])
        assert pieces[0]["span"]["start_s"] == spans[group_id]["start_s"]
        assert pieces[-1]["span"]["end_s"] == spans[group_id]["end_s"]


def test_the_threshold_is_a_dial_over_a_bimodal_distribution(annotated):
    """Every junction on this fixture is either 0 or 90 degrees, so any
    threshold between them gives the same answer. If that ever stops being
    true, this test is where it shows up."""
    counts = {
        t: default_seams(annotated, max_corner_angle_deg=t)["summary"]["seam_count"]
        for t in (1.0, 5.0, 45.0, 89.0)
    }
    assert len(set(counts.values())) == 1, counts
    assert default_seams(annotated, max_corner_angle_deg=91.0)["summary"]["seam_count"] == 6


def test_a_tangent_loop_is_never_cut(annotated):
    """pipe_bracket's column: eight segments, eight G1 junctions, zero cuts.

    The pair with the gusset rectangles is the whole argument for measuring
    tangents rather than counting segments -- both are 4-or-8-segment closed
    loops and only one of them has corners.
    """
    column = next(j for j in annotated["weld_joints"] if j["id"] == "weld_joint_0003")
    for seam in seams_for_joint(column):
        assert seam["origin"]["group_size"] == 1
        assert seam["origin"]["boundary_continuity"] == [None, None]


# --------------------------------------------------------------------------
# origin: the two splits are distinguishable
# --------------------------------------------------------------------------


def test_split_reason_prefers_the_obstruction_where_both_are_true(annotated, generated):
    """A gusset's blocked stretch begins exactly at the corner it is welded
    into, so the last piece of those runs ends at a corner AND at unweldable
    material. "No torch can reach beyond here" is the stronger claim, and the
    corner is still recorded in boundary_continuity, so preferring it hides
    nothing."""
    last_pieces = [
        s
        for s in generated["seams"]
        if s["origin"]["index_in_group"] == s["origin"]["group_size"] - 1
        and s["weld_joint_ref"] != "weld_joint_0003"
    ]
    assert last_pieces
    for seam in last_pieces:
        assert seam["origin"]["split_reason"] == "reachability"
        assert seam["origin"]["boundary_continuity"][1] in {"C0", None}


def test_a_joint_with_neither_split_says_so():
    """split_reason "none" is a real state: one boundary, no obstruction, no
    corner. Reporting "reachability" there would imply an obstruction that does
    not exist."""
    joint = {
        "id": "weld_joint_9999",
        "weld_line": {"length_m": 0.1, "closed": False, "segments": []},
    }
    seams = seams_for_joint(joint)
    assert len(seams) == 1
    assert seams[0]["span"] == {"start_s": 0.0, "end_s": 1.0}
    assert seams[0]["origin"]["split_reason"] == "none"
    assert seams[0]["origin"]["group_size"] == 1


def test_group_ids_are_unique_per_run_and_shared_within_it(generated):
    by_group: dict[str, set] = {}
    for seam in generated["seams"]:
        by_group.setdefault(seam["origin"]["group_id"], set()).add(seam["weld_joint_ref"])
    for group_id, joints in by_group.items():
        assert len(joints) == 1, f"{group_id} spans more than one joint"
        assert group_id.startswith(next(iter(joints)))


# --------------------------------------------------------------------------
# conservation and thresholds
# --------------------------------------------------------------------------


def test_weldable_plus_blocked_equals_the_whole_boundary(annotated, generated):
    """Conservation. Nothing is silently dropped or double counted."""
    boundary = sum(j["weld_line"]["length_m"] for j in annotated["weld_joints"])
    blocked = annotated["summary"]["blocked_length_m"]
    # 1 um, not 1 nm. `seams` stores every span as `round(s, 6)` of a
    # NORMALIZED parameter, which on a 190 mm weld line is 0.095 um per
    # boundary, and this corpus has 28 of them. The two sides therefore cannot
    # agree below about a micrometre and never could -- 1e-9 passed by luck,
    # because the roundings happened to cancel until a boundary moved. A
    # micrometre still catches a real conservation break by three orders of
    # magnitude; the failures this guards against are millimetres.
    assert generated["summary"]["weldable_length_m"] == pytest.approx(
        boundary - blocked, abs=1e-6
    )
    assert generated["summary"]["runs_below_min_length"] == 0


def test_a_short_run_is_dropped_and_counted(annotated):
    """Dropping is allowed, dropping SILENTLY is not -- the summary says how
    many runs fell below the threshold."""
    huge = default_seams(annotated, min_length_m=0.09)
    assert huge["summary"]["seam_count"] < 6
    assert huge["summary"]["runs_below_min_length"] > 0
    assert huge["summary"]["min_seam_length_m"] == 0.09


def test_process_minimum_is_not_baked_into_the_geometry(annotated):
    """`min_seam_length_m` is a judgement about a procedure, so it is a
    parameter of intent generation. The geometric analysis in access.py must
    not have filtered anything on those grounds."""
    for joint in annotated["weld_joints"]:
        runs = joint["accessibility"]["reachable_runs"]
        lengths = joint["accessibility"]["reachable_lengths_m"]
        assert len(runs) == len(lengths)
        assert all(length > 0 for length in lengths)
    assert DEFAULT_MIN_SEAM_LENGTH_M > 0
    assert DEFAULT_MAX_CORNER_ANGLE_DEG > 0


def test_a_joint_without_accessibility_still_yields_a_seam():
    """Absence of analysis is not evidence of obstruction."""
    joint = {
        "id": "weld_joint_9999",
        "weld_line": {"length_m": 0.1, "closed": False, "segments": []},
    }
    seams = seams_for_joint(joint)
    assert len(seams) == 1
    assert seams[0]["span"] == {"start_s": 0.0, "end_s": 1.0}


# --------------------------------------------------------------------------
# the contract
# --------------------------------------------------------------------------


def test_generated_seams_validate_against_the_contract(annotated, generated, schema_dir):
    schema = json.loads(
        (schema_dir / "amr-weld-planner-v1.weld-program.v1.schema.json").read_text()
    )
    program = {
        "schema": "amr-weld-planner-v1.weld-program.v1",
        "program_id": "pipe_bracket_default_seams",
        "source": annotated["source"],
        "workpiece": {"frame_id": "pipe_bracket_origin"},
        "weld_joints": annotated["weld_joints"],
        "seams": generated["seams"],
    }
    errors = sorted(Draft202012Validator(schema).iter_errors(program), key=lambda e: list(e.path))
    assert not errors, "\n".join(f"{list(e.path)}: {e.message}" for e in errors[:5])


def test_spin_is_free_on_every_generated_seam(generated):
    """R2 survives seam generation."""
    for seam in generated["seams"]:
        assert seam["orientation"]["spin"]["mode"] == "free", seam["id"]


def test_self_test_entry_point_passes():
    from seam_worker.seam.seams import self_test

    assert self_test() == 0
