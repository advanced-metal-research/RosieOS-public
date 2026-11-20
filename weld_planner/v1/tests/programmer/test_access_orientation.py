"""Accessibility samples the weld line the way the weld line runs.

`chain_edges` orders a joint's edges head to tail and REVERSES the ones that
need it, so an emitted segment runs whichever way the loop does.
`_sample_weld_line` re-reads the ORIGINAL OCCT edge through `BRepAdaptor_Curve`,
which knows nothing about that and walks the edge in its own parameter
direction. Without the orientation check, every flag on a reversed segment is
recorded at a MIRRORED `s`.

It hid for two reasons, and both are why these tests exist.

The boundary bisection is RIGHT: `_blocked_at` evaluates the emitted curve, so a
transition bracketed between two mirrored samples is then refined at true
positions. The boundary settles between the mirrored guess and the truth, which
reads as a plausible blocked span slightly in the wrong place rather than as
nonsense.

And `pipe_bracket`, the fixture nearly every other test is written against, is
IMMUNE: its gussets sit symmetrically on their runs, so mirroring a blocked span
maps it onto itself. Its seam count, group count and weldable length are
identical with the bug and without it. `bracket_b` is not symmetric, and there
the 10 mm a gusset covers came back as 7.9 mm shifted 2.5 mm along the weld --
so a seam was offered running 4.6 mm underneath the gusset standing on it.
"""

from __future__ import annotations

import math

import pytest
from seam_worker import paths
from seam_worker.reach.access import (
    _occt_edges_by_key,
    _sample_weld_line,
    _weld_curve,
    annotate,
    edge_key_map,
)
from seam_worker.seam.detect import detect_joints
from seam_worker.cad.topology import extract_topology, load_step

BRACKET_B = paths.FIXTURES_DIR / "bracket_b.step"


@pytest.fixture(scope="module")
def bracket_b():
    if not BRACKET_B.is_file():
        pytest.skip(f"fixture STEP not present: {BRACKET_B}")
    topology = extract_topology(BRACKET_B)
    return BRACKET_B, topology, detect_joints(topology)


def test_every_sample_lands_where_the_curve_says_it_does(bracket_b):
    """The invariant, and the whole bug in one line.

    `_sample_weld_line` reports `(point, s)`. Evaluating the emitted weld curve
    at that same `s` must give back that same point -- they are two routes to
    one position on one line. Before the orientation check they disagreed by up
    to 110 mm, and over half of every fixture's samples were misplaced.
    """
    step, topology, detected = bracket_b
    shape = load_step(step)
    edges_by_key = _occt_edges_by_key(shape)
    key_by_edge_id = edge_key_map(topology)

    worst = 0.0
    checked = 0
    for joint in detected["weld_joints"]:
        curve = _weld_curve(joint)
        samples = _sample_weld_line(edges_by_key, key_by_edge_id, joint, 2.0)
        assert samples, f"{joint['id']} produced no samples"
        for coord, s, _segment in samples:
            here = [float(v) * 1000.0 for v in curve.point(s)]
            worst = max(worst, math.dist(coord, here))
            checked += 1
    assert checked > 100, "not enough samples to be measuring anything"
    # A millimetre of slack for the B-spline's own approximation of an arc,
    # against the 110 mm the mirrored walk produced.
    assert worst < 2.0, f"sampler and curve disagree by {worst:.2f} mm"


def test_a_gusset_blocks_where_the_gusset_actually_is(bracket_b):
    """The user-visible consequence, pinned in millimetres.

    `solid_2` stands on the base-to-upright weld line over x -65..-55. The
    blocked span has to be that stretch. With the samples mirrored it came back
    as x -67.45..-59.58: 2.5 mm of perfectly weldable arc discarded at one end,
    and 4.6 mm of arc offered at the other that has a gusset sitting on it.
    """
    step, topology, _detected = bracket_b
    joints = annotate(step, topology, detect_joints(topology))
    gusset = next(s for s in topology["solids"] if s["id"] == "solid_2")
    lo_x = gusset["bbox_min_xyz_m"][0] * 1000.0
    hi_x = gusset["bbox_max_xyz_m"][0] * 1000.0

    joint = next(
        j
        for j in joints["weld_joints"]
        if {m["solid_id"] for m in j["members"]} == {"solid_0", "solid_1"}
        and j["weld_line"]["segments"][0]["analytic"]["start_xyz_m"][0] < 0
    )
    curve = _weld_curve(joint)
    spans = joint["accessibility"]["blocked_spans"]
    assert len(spans) == 1, f"expected one blocked stretch, got {spans}"
    start, end = spans[0]
    xs = sorted(float(curve.point(s)[0]) * 1000.0 for s in (start, end))

    # Both ends within a millimetre of the gusset's real footprint. The residual
    # is the escape test at a corner, where a ray can still slip out diagonally
    # for a fraction of a millimetre past the obstruction -- the same effect
    # `MIN_RESOLVED_RUN_M` exists for, and orders of magnitude below the 4.6 mm
    # the orientation bug produced.
    assert xs[0] == pytest.approx(lo_x, abs=1.0), f"low end {xs[0]:.2f} vs gusset {lo_x}"
    assert xs[1] == pytest.approx(hi_x, abs=1.0), f"high end {xs[1]:.2f} vs gusset {hi_x}"
