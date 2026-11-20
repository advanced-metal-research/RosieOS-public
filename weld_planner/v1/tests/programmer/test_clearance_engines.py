"""The barrel test, against the exact surfaces it replaced.

`clearance.py` used to cast every ray through OCCT's `BRepIntCurveSurface_Inter`,
which intersects the exact NURBS of every face with no spatial index. Measured
here: 150.8 us per ray on `pipe_bracket`, 6.3 ms per ray on the exhaust manifold.
One weld's angle lattice was 400,554 rays and 56 seconds; the optimiser is that
loop over every seam.

It now casts through `occlusion.py`'s triangle BVH, the same index `access.py`
uses, and the same lattice is 0.050 seconds. That is a 1131x change in the
number a person waits on, which is exactly the kind of change that is worth
distrusting -- so these are the tests that make the speed claim answerable.

**The oracle is kept runnable, not quoted.** `--engine occt` still works, and
every test here runs both and compares. What is compared is never a timing; it
is the verdict that decides whether a stretch of weld gets made:

- `clear_fraction` and the blocked spans of a real weld;
- every cell of an angle lattice, which is what the optimiser ranks;
- the closed form, which neither engine gets to define.

**Where they are allowed to differ.** The BVH sees facets, OCCT sees surfaces,
so a boundary can land a facet's width apart. That shows up as whole lattice
cells disagreeing when a sample sits within one deflection of a collision edge:
8 of 28,611 on `pipe_bracket`, and the totals 0.033% apart. The tolerances below are
the measured ones, tight enough that a real regression cannot hide under them.

The one asymmetry worth stating: a truncated hit list has the WRONG crossing
parity, which turns a buried torch into a clear one. That is the single
direction this module must never err in, so `MAX_HITS` overflow raises rather
than returning a plausible number.
"""

import numpy as np
import pytest
from seam_worker.reach.access import annotate
from seam_worker.reach.clearance import (
    FINE_RAY_COUNT,
    SeamGeometry,
    build_lattice,
    clearance,
    decision_knots,
    ray_engine,
    search_inputs,
    synthetic_corner,
)
from seam_worker.seam.detect import detect_joints
from seam_worker.seam.seams import default_seams
from seam_worker.seam.span import span_fraction
from seam_worker.reach.tooling import (
    DEFAULT_TORCH,
    fits_dihedral_deg,
    standoff_candidates,
    weld_standoff_mm,
)
from seam_worker.cad.topology import extract_topology, load_step

# What the torch record used to carry as its fallback. Stated here because
# `clearance()` now requires the gap rather than reaching for one.
NOMINAL_STANDOFF_MM = 10.0

ENGINES = ("occt", "bvh")


@pytest.fixture(scope="module")
def longest_seam(pipe_bracket_step):
    """The longest weld on `pipe_bracket`, with its joint and a fresh shape.

    The longest one because it is the one that crosses the most geometry, so a
    disagreement between the intersectors has the most chances to appear.
    """
    topology = extract_topology(pipe_bracket_step)
    joints = annotate(pipe_bracket_step, topology, detect_joints(topology))
    joints["seams"] = default_seams(joints)["seams"]
    by_id = {j["id"]: j for j in joints["weld_joints"]}
    seam = max(
        joints["seams"],
        key=lambda s: span_fraction(s["span"])
        * by_id[s["weld_joint_ref"]]["weld_line"]["length_m"],
    )
    return {
        "shape": load_step(pipe_bracket_step),
        "joint": by_id[seam["weld_joint_ref"]],
        "seam": seam,
    }


# --------------------------------------------------------------------------
# the verdict a person acts on
# --------------------------------------------------------------------------


def test_both_engines_agree_on_how_much_of_a_real_weld_is_reachable(
    longest_seam, pipe_bracket_step
):
    """`clear_fraction` is the number on screen and the number the trim and
    split decisions are made from. If the engines disagree anywhere, disagreeing
    here is what would matter.

    A shape of its own per engine: `occlusion.triangles` cleans the
    triangulation off a shape before meshing it, so sharing one would have the
    mesh move under OCCT's feet.
    """
    results = {
        engine: clearance(
            load_step(pipe_bracket_step), longest_seam["joint"], longest_seam["seam"],
            standoff_mm=weld_standoff_mm(longest_seam["seam"], NOMINAL_STANDOFF_MM),
            engine=engine, key=f"agree-{engine}",
        )
        for engine in ENGINES
    }
    occt, bvh = results["occt"], results["bvh"]
    assert bvh["clear_fraction"] == pytest.approx(occt["clear_fraction"], abs=1e-4), (
        occt["clear_fraction"],
        bvh["clear_fraction"],
    )
    assert len(bvh["blocked_spans"]) == len(occt["blocked_spans"]), (
        occt["blocked_spans"],
        bvh["blocked_spans"],
    )
    for (a0, a1), (b0, b1) in zip(occt["blocked_spans"], bvh["blocked_spans"]):
        # A boundary is bisected three times over a seam of ~80 mm, so a span
        # edge is only resolved to about 1% of the seam anyway.
        assert b0 == pytest.approx(a0, abs=0.01), (occt["blocked_spans"], bvh["blocked_spans"])
        assert b1 == pytest.approx(a1, abs=0.01), (occt["blocked_spans"], bvh["blocked_spans"])


def test_the_indexed_engine_reproduces_the_closed_form(longest_seam):
    """`tooling.fits_dihedral_deg` is derived from geometry, not from a ray, so
    it is the one assertion here that no intersector gets a vote on.

    A cylinder of radius R on the bisector of a wedge of dihedral theta clears
    it iff `theta >= 2*atan(R/d)`. Checked either side of that angle.
    """
    torch = {**DEFAULT_TORCH, "body_radius_mm": 8.0, "body_length_mm": 150.0}
    standoff = 10.0
    threshold = fits_dihedral_deg(torch, standoff)
    for dihedral, expect_clear in ((threshold + 12.0, True), (threshold - 12.0, False)):
        shape, joint, seam = synthetic_corner(dihedral)
        result = clearance(
            shape, joint, seam, torch,
            standoff_mm=standoff, engine="bvh", key=f"closed-form-{dihedral}",
        )
        clear = result["clear_fraction"] > 0.5
        assert clear is expect_clear, (
            f"dihedral {dihedral:.1f} vs threshold {threshold:.1f}: "
            f"clear_fraction {result['clear_fraction']}"
        )


# --------------------------------------------------------------------------
# the lattice the optimiser ranks
# --------------------------------------------------------------------------


def _banded(seam, band_mm):
    """The seam with `band_mm` of declared stickout freedom either side."""
    import copy

    from seam_worker.reach.tooling import weld_standoff_mm

    out = copy.deepcopy(seam)
    authored = weld_standoff_mm(out, NOMINAL_STANDOFF_MM)
    wire = out.setdefault("process", {}).setdefault("wire", {})
    wire["stickout_mm"] = authored
    wire["stickout_limits_mm"] = {"min": authored - band_mm, "max": authored + band_mm}
    return out


@pytest.fixture(scope="module")
def lattices(longest_seam, pipe_bracket_step):
    """One angle lattice per engine, over identical bins and standoffs.

    Every fourth bin rather than all 17x17. The full lattice is 400,554 rays and
    the OCCT side of it takes 57 seconds, which is a minute per run to compare
    cells that differ from their neighbours by one bin of work angle. The
    subsampled lattice still spans the whole band, the whole seam and every
    standoff -- it is the same comparison at a quarter of the resolution, and it
    is 5 seconds.
    """
    geometry = SeamGeometry(longest_seam["joint"], longest_seam["seam"])
    knots, _ = decision_knots(geometry, 10.0)
    inputs = search_inputs(longest_seam["seam"], knots)
    out = {}
    for engine in ENGINES:
        out[engine] = build_lattice(
            load_step(pipe_bracket_step),
            longest_seam["joint"],
            longest_seam["seam"],
            DEFAULT_TORCH,
            work_bins=inputs["work_bins"][::4],
            travel_bins=inputs["travel_bins"][::4],
            # A ladder of rungs is what makes this a cell-for-cell comparison
            # rather than a single-slice one, and the only way to get one is to
            # declare the freedom in the procedure -- which is the point: a
            # settings-wide band no longer exists to widen it from outside.
            standoffs_mm=standoff_candidates(_banded(longest_seam["seam"], 5.0), 10.0),
            engine=engine,
            key=f"lattice-{engine}",
        )
    return out


def test_every_lattice_cell_agrees_with_the_exact_intersector(lattices):
    """Cell for cell, because the optimiser compares cells.

    A total that matched while individual cells did not would be a lattice with
    the right area and the wrong shape, and the path through it -- which is the
    actual output -- would be different.
    """
    occt = np.asarray(lattices["occt"]["clear_mm"], dtype=float)
    bvh = np.asarray(lattices["bvh"]["clear_mm"], dtype=float)
    assert bvh.shape == occt.shape
    diff = np.abs(bvh - occt)
    exact = float((diff < 1e-9).mean())
    # Measured: 28,603 of 28,611 cells identical. Pinned a little loose so a
    # fixture re-export cannot fail it, tight enough that a broken kernel does.
    assert exact > 0.99, f"only {exact:.4%} of cells agree; worst {diff.max():.3f} mm"
    assert bvh.sum() == pytest.approx(occt.sum(), rel=1e-3), (occt.sum(), bvh.sum())


def test_the_two_engines_cast_the_same_number_of_rays(lattices):
    """The speedup must come from the traversal, not from quietly doing less."""
    assert lattices["bvh"]["rays_cast"] == lattices["occt"]["rays_cast"]
    assert lattices["bvh"]["samples"] == lattices["occt"]["samples"]
    assert lattices["bvh"]["knot_fractions"] == lattices["occt"]["knot_fractions"]


def test_a_chunked_lattice_sums_to_the_whole_one(longest_seam, pipe_bracket_step):
    """The kernel runs the samples in parallel and folds them onto knots
    afterwards. Partitioning the sample axis must therefore still partition the
    sums -- the property `merge_lattices` relies on."""
    from seam_worker.reach.clearance import chunk_ranges, merge_lattices

    geometry = SeamGeometry(longest_seam["joint"], longest_seam["seam"])
    knots, _ = decision_knots(geometry, 10.0)
    inputs = search_inputs(longest_seam["seam"], knots)
    args = {
        "work_bins": inputs["work_bins"],
        "travel_bins": inputs["travel_bins"],
        "standoffs_mm": [10.0, 14.0],
        "engine": "bvh",
        "key": "chunked",
    }
    shape = load_step(pipe_bracket_step)
    whole = build_lattice(shape, longest_seam["joint"], longest_seam["seam"], DEFAULT_TORCH, **args)
    parts = [
        build_lattice(
            shape, longest_seam["joint"], longest_seam["seam"], DEFAULT_TORCH,
            index_range=r, **args,
        )
        for r in chunk_ranges(whole["samples"], 4)
    ]
    merged = merge_lattices(parts)
    assert np.allclose(
        np.asarray(merged["clear_mm"], float), np.asarray(whole["clear_mm"], float)
    )


# --------------------------------------------------------------------------
# the hit list itself
# --------------------------------------------------------------------------


def test_the_hit_lists_themselves_agree(longest_seam, pipe_bracket_step):
    """One rung below the verdict: the distances, before anything reads a
    standoff off them.

    Counts are allowed to differ where a ray grazes a facet edge -- 5% of rays
    on this fixture -- but where both engines see the same number of surfaces,
    they must agree on WHERE they are to well under a facet.
    """
    geometry = SeamGeometry(longest_seam["joint"], longest_seam["seam"])
    engines = {e: ray_engine(load_step(pipe_bracket_step), e, f"hits-{e}") for e in ENGINES}
    from seam_worker.reach.clearance import barrel_axes
    from seam_worker.seam.frames import apply_angles
    from seam_worker.reach.tooling import barrel_offsets

    compared = worst = 0
    worst_gap = 0.0
    for fraction in (0.1, 0.35, 0.6, 0.9):
        point, t, r = geometry.frame(fraction)
        u = apply_angles(t, r, 0.0, 0.0)
        e1, e2 = barrel_axes(t, u)
        for a, b in barrel_offsets(8.0, FINE_RAY_COUNT)[::4]:
            origin = tuple(point + a * e1 + b * e2)
            direction = (float(u[0]), float(u[1]), float(u[2]))
            lists = {e: sorted(eng.hits_along(origin, direction)) for e, eng in engines.items()}
            compared += 1
            if len(lists["occt"]) != len(lists["bvh"]):
                worst += 1
                continue
            for x, y in zip(lists["occt"], lists["bvh"]):
                worst_gap = max(worst_gap, abs(x - y))
    assert compared > 50
    assert worst / compared < 0.15, f"{worst}/{compared} rays disagreed on hit COUNT"
    # A 0.01 mm deflection facet cannot move a crossing further than its own
    # sagitta; measured worst is 0.0105 mm.
    assert worst_gap < 0.05, worst_gap


def test_a_truncated_hit_list_raises_rather_than_flipping_a_parity(pipe_bracket_step):
    """The one direction this must never err in.

    `fan_collides` calls the far end buried when the crossings beyond it are
    odd. Drop one hit and that parity inverts -- a torch driven through solid
    metal reports clear, with no symptom anywhere. So overflow is an exception,
    not a truncation.
    """
    from seam_worker.cad import occlusion

    engine = ray_engine(load_step(pipe_bracket_step), "bvh", "overflow")
    original = occlusion.MAX_HITS
    try:
        occlusion.MAX_HITS = 1  # any ray crossing a plate twice now overflows
        with pytest.raises(ValueError, match="MAX_HITS"):
            for z in np.linspace(-40.0, 40.0, 80):
                engine.hits_along((0.0, 0.0, float(z)), (0.0, 0.0, 1.0))
                engine.hits_along((0.0, 0.0, float(z)), (1.0, 0.0, 0.0))
    finally:
        occlusion.MAX_HITS = original


def test_the_result_records_which_intersector_measured_it(longest_seam, pipe_bracket_step):
    """A coverage number carrying no engine is one that will be read back as
    exact once the tessellation is forgotten. Same rule `access.py` keeps."""
    for engine in ENGINES:
        result = clearance(
            load_step(pipe_bracket_step), longest_seam["joint"], longest_seam["seam"],
            standoff_mm=weld_standoff_mm(longest_seam["seam"], NOMINAL_STANDOFF_MM),
            engine=engine, key=f"prov-{engine}",
        )
        assert result["intersector"]["engine"] == engine
    indexed = clearance(
        load_step(pipe_bracket_step), longest_seam["joint"], longest_seam["seam"],
        standoff_mm=weld_standoff_mm(longest_seam["seam"], NOMINAL_STANDOFF_MM),
        engine="bvh", key="prov-detail",
    )
    assert indexed["intersector"]["deflection_mm"] > 0
    assert indexed["intersector"]["triangles"] > 0


def test_an_unknown_engine_is_refused(pipe_bracket_step):
    from seam_worker.reach.clearance import ClearanceError

    with pytest.raises(ClearanceError, match="unknown engine"):
        ray_engine(load_step(pipe_bracket_step), "raymarch")
