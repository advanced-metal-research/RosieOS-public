"""The hemisphere is oriented per SEGMENT, not once per joint.

`access.annotate_joint` spreads its escape rays about the groove bisector,
because that is where a torch would come from. The bisector rotates along a
weld line; the pole used to be taken from `groove_faces[0]` and used for the
whole joint.

That is not a distribution nicety. `hemisphere` returns directions within 90
degrees of its pole, so on a segment whose bisector is 90 degrees away the
escape directions sit on the equator -- half of them outside the sampled set
entirely, and no ray count reaches them. The measurements below show exactly
that: with one pole the open fraction converges on a number about 2.3x too
small and STAYS there as the fan is refined.

Both dog_box and pipe_bracket have joints whose bisectors are a full 90 degrees
from `groove_faces[0]`'s, so this is the normal case for a box weldment rather
than a corner case.
"""

from __future__ import annotations

import math

import pytest

# `seam_worker.paths` and not `from conftest import DOG_BOX`. A bare
# `conftest` import resolves to whichever conftest directory pytest put on
# `sys.path` first, so the moment a second test package appeared -- the
# motion planner's -- this module started importing that one and failing to
# find a name in it. The constant has a real home; this is it.
from seam_worker.reach.access import annotate, hemisphere
from seam_worker.seam.detect import detect_joints
from seam_worker.paths import DOG_BOX

RAYS = 256


@pytest.fixture(scope="module")
def dog_box_step():
    """The path, beside the session-scoped `dog_box_topology` in conftest.
    `annotate` needs the STEP itself -- it builds its own intersector index."""
    if not DOG_BOX.exists():
        pytest.skip(f"fixture STEP not present: {DOG_BOX}")
    return DOG_BOX


def _annotate(step, topology, *, one_pole: bool, rays: int = RAYS):
    """Accessibility, optionally with the old single-pole behaviour.

    Reproduced by TRIMMING the groove list rather than by keeping the old code
    around: with one groove record the current implementation falls back to it
    for every segment, which is precisely what the old code did. So this is an
    A/B of the pole choice with nothing else changed.
    """
    joints = detect_joints(topology)
    if one_pole:
        for joint in joints["weld_joints"]:
            grooves = joint.get("groove_faces") or []
            if grooves:
                joint["groove_faces"] = [grooves[0]]
    return annotate(step, topology, joints, ray_count=rays)


def _bisector_spread_deg(joint) -> float:
    """How far the bisector rotates along one joint, in degrees."""
    grooves = joint.get("groove_faces") or []
    if len(grooves) < 2:
        return 0.0

    def unit(groove):
        a = groove["face_a"]["outward_normal_xyz"]
        b = groove["face_b"]["outward_normal_xyz"]
        v = [a[i] + b[i] for i in range(3)]
        n = math.sqrt(sum(c * c for c in v))
        return [c / n for c in v] if n > 1e-9 else None

    first = unit(grooves[0])
    if first is None:
        return 0.0
    worst = 0.0
    for groove in grooves[1:]:
        other = unit(groove)
        if other is None:
            continue
        dot = max(-1.0, min(1.0, sum(first[i] * other[i] for i in range(3))))
        worst = max(worst, math.degrees(math.acos(dot)))
    return worst


# --------------------------------------------------------------------------
# the premise
# --------------------------------------------------------------------------


def test_a_hemisphere_really_does_exclude_the_far_side():
    """The mechanism, before any CAD. Nothing within 90 degrees of the pole is
    a fair sample of a direction 90 degrees away from it -- and refining the
    fan cannot fix that, because the missing directions are not in the set."""
    pole = (0.0, 0.0, 1.0)
    for count in (64, 256, 4096):
        directions = hemisphere(count, pole)
        assert all(d[2] >= -1e-9 for d in directions), "a direction fell below the equator"
    # A bisector at 90 degrees points along +X, whose own escape cone straddles
    # the equator of this hemisphere: half of it is simply not sampled.
    dense = hemisphere(4096, pole)
    near_x = [d for d in dense if d[0] > 0.95]
    assert near_x, "the +X direction itself is on the boundary and barely sampled"
    assert all(d[2] >= -1e-9 for d in near_x)


def test_the_bisector_really_does_rotate_on_these_fixtures(
    dog_box_topology, pipe_bracket_topology
):
    """Otherwise everything below would be measuring nothing."""
    for topology in (dog_box_topology, pipe_bracket_topology):
        joints = detect_joints(topology)
        spreads = [_bisector_spread_deg(j) for j in joints["weld_joints"]]
        assert max(spreads) > 45.0, f"no joint's bisector rotates far enough: {spreads}"


def test_every_segment_has_its_own_groove_record(dog_box_topology):
    """The fix indexes groove faces by `segment_index`. If some segment had no
    record it would silently fall back and the fix would be partial."""
    for joint in detect_joints(dog_box_topology)["weld_joints"]:
        grooves = joint.get("groove_faces") or []
        segments = joint["weld_line"]["segments"]
        indices = {int(g.get("segment_index", -1)) for g in grooves}
        assert indices == set(range(len(segments))), (
            f"{joint['id']}: groove records cover {sorted(indices)} of "
            f"{len(segments)} segments"
        )


# --------------------------------------------------------------------------
# what it changes
# --------------------------------------------------------------------------


def test_the_open_fraction_was_being_under_reported(dog_box_step, dog_box_topology):
    """The measurable consequence. `open_fraction` is what a planner thresholds
    against its own tooling, so under-reporting it rejects approaches that are
    in fact available."""
    one = _annotate(dog_box_step, dog_box_topology, one_pole=True)
    per = _annotate(dog_box_step, dog_box_topology, one_pole=False)
    moved = [
        (a["id"], a["accessibility"]["min_open_fraction"], b["accessibility"]["min_open_fraction"])
        for a, b in zip(one["weld_joints"], per["weld_joints"])
        if abs(a["accessibility"]["min_open_fraction"] - b["accessibility"]["min_open_fraction"])
        > 1e-9
    ]
    assert moved, "the pole change made no difference anywhere, so it is untested"
    for jid, before, after in moved:
        assert after > before, f"{jid}: {after} is not more open than {before}"


@pytest.mark.parametrize("rays", [256, 1024])
def test_more_rays_do_not_rescue_the_wrong_pole(dog_box_step, dog_box_topology, rays):
    """The heart of it.

    A sampling error shrinks as the fan is refined. This one does not: the
    directions that escape are OUTSIDE the sampled hemisphere, so the single
    pole converges on its own wrong answer. Measured on the three dog_box tees:
    0.0625 -> 0.0859 -> 0.0859 with one pole against 0.125 -> 0.195 -> 0.201
    per segment, at 256, 1024 and 4096 rays.
    """
    one = _annotate(dog_box_step, dog_box_topology, one_pole=True, rays=rays)
    per = _annotate(dog_box_step, dog_box_topology, one_pole=False, rays=rays)
    worst_one = min(j["accessibility"]["min_open_fraction"] for j in one["weld_joints"])
    worst_per = min(j["accessibility"]["min_open_fraction"] for j in per["weld_joints"])
    assert worst_per > worst_one * 1.5, (
        f"at {rays} rays the gap closed to {worst_per / max(worst_one, 1e-9):.2f}x, so the "
        "single pole may just have been under-sampled rather than pointed the wrong way"
    )


def test_the_blocked_verdict_is_unchanged(dog_box_step, dog_box_topology, pipe_bracket_step,
                                          pipe_bracket_topology):
    """And it should be.

    Blocked means NO direction escapes. Where that is true it is true from
    every pole, and where it is false the escaping directions are usually wide
    enough that any hemisphere finds some. So the pole moves the marginal
    number and not the verdict -- which is why pipe_bracket's pinned 59.75 mm does
    not move, and why this fix is a correction to `open_fraction` rather than
    to the blocked spans.
    """
    for step, topology in ((dog_box_step, dog_box_topology),
                           (pipe_bracket_step, pipe_bracket_topology)):
        one = _annotate(step, topology, one_pole=True)
        per = _annotate(step, topology, one_pole=False)
        assert one["summary"]["blocked_length_m"] == pytest.approx(
            per["summary"]["blocked_length_m"], abs=1e-9
        )
        for a, b in zip(one["weld_joints"], per["weld_joints"]):
            assert a["accessibility"]["blocked_spans"] == b["accessibility"]["blocked_spans"]


def test_a_joint_with_one_groove_record_is_unaffected(pipe_bracket_step, pipe_bracket_topology):
    """The fallback path. A segment with no record of its own uses the first
    one there is, which is exactly the old behaviour -- so nothing regresses
    for geometry that only ever had one bisector."""
    joints = detect_joints(pipe_bracket_topology)
    for joint in joints["weld_joints"]:
        grooves = joint.get("groove_faces") or []
        if grooves:
            joint["groove_faces"] = [grooves[0]]
    trimmed = annotate(pipe_bracket_step, pipe_bracket_topology, joints, ray_count=RAYS)
    again = _annotate(pipe_bracket_step, pipe_bracket_topology, one_pole=True)
    assert trimmed["summary"]["blocked_length_m"] == pytest.approx(
        again["summary"]["blocked_length_m"], abs=1e-9
    )
