"""The angle search: a dynamic program, checked against brute force.

No CAD in this file, so it runs in milliseconds. That is the point of keeping
`orient.py` free of OCCT -- the search can be checked exhaustively on lattices
small enough to enumerate, which is the only way to know a dynamic program is
actually returning the optimum rather than something plausible.

`test_the_dp_matches_brute_force` is the one that matters. Everything else
describes a property; that one proves the answer.
"""

import itertools

import pytest
from seam_worker.reach.orient import (
    DEFAULT_KNOT_SPACING_MM,
    OrientError,
    bin_values,
    optimise,
    search,
    simplify,
    to_profile,
)

# --------------------------------------------------------------------------- bins


def test_zero_is_always_a_choice():
    """Work angle 0 is the overwhelmingly common answer. A grid anchored on the
    band's lower edge would offer -14.7, -12.7, ... and never 0."""
    for lo, hi in [(-15.0, 15.0), (-14.7, 9.3), (-6.0, 6.0), (0.0, 20.0)]:
        assert 0.0 in bin_values(lo, hi, 2.0), (lo, hi)


def test_a_symmetric_band_gives_a_symmetric_odd_set():
    values = bin_values(-6.0, 6.0, 2.0)
    assert values == [-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0]
    assert len(values) % 2 == 1
    assert values == [-v for v in reversed(values)]


def test_bins_stay_inside_the_band():
    for lo, hi, step in [(-15.0, 15.0, 2.0), (-7.0, 3.0, 2.0), (1.0, 19.0, 5.0)]:
        for v in bin_values(lo, hi, step):
            assert lo - 1e-9 <= v <= hi + 1e-9


def test_a_band_too_narrow_to_contain_a_bin_is_refused():
    """Snapping to the nearest bin would move outside a declared limit, and a
    limit quietly widened is the failure `orientation.limits` exists against."""
    with pytest.raises(OrientError, match="no multiple"):
        bin_values(1.1, 1.9, 2.0)
    with pytest.raises(OrientError, match="empty band"):
        bin_values(5.0, -5.0, 2.0)


# ------------------------------------------------------------------- the search


def _problem(clear, work, travel, **kw):
    n = len(clear)
    base = {
        "clear_mm": clear,
        "work_bins": work,
        "travel_bins": travel,
        "arc_mm": [10.0] * n,
        "spacing_mm": [10.0] * (n - 1),
        "authored_work": [0.0] * n,
        "authored_travel": [0.0] * n,
    }
    base.update(kw)
    return base


def _brute_force(problem):
    """Every path, scored the same way, so the DP has something to be wrong
    against. Exponential -- only usable on a toy lattice, which is the point."""
    work, travel = problem["work_bins"], problem["travel_bins"]
    clear, arc = problem["clear_mm"], problem["arc_mm"]
    n = len(clear)
    rate_w = (problem.get("work_limits") or {}).get("max_rate_deg_per_mm")
    rate_t = (problem.get("travel_limits") or {}).get("max_rate_deg_per_mm")

    best = None
    states = list(itertools.product(range(len(work)), range(len(travel))))
    for path in itertools.product(states, repeat=n):
        legal = True
        for i in range(n - 1):
            gap = problem["spacing_mm"][i]
            if rate_w is not None and abs(work[path[i + 1][0]] - work[path[i][0]]) > rate_w * gap + 1e-9:
                legal = False
                break
            if rate_t is not None and abs(travel[path[i + 1][1]] - travel[path[i][1]]) > rate_t * gap + 1e-9:
                legal = False
                break
        if not legal:
            continue
        j1 = sum(clear[i][a][b] for i, (a, b) in enumerate(path))
        j2 = -sum(
            arc[i]
            * (
                abs(work[a] - problem["authored_work"][i])
                + abs(travel[b] - problem["authored_travel"][i])
            )
            for i, (a, b) in enumerate(path)
        )
        j3 = -sum(
            abs(work[path[i + 1][0]] - work[path[i][0]])
            + abs(travel[path[i + 1][1]] - travel[path[i][1]])
            for i in range(n - 1)
        )
        score = (j1, j2, j3)
        if best is None or score > best[0]:
            best = (score, path)
    return best


def test_the_dp_matches_brute_force():
    """The assertion this file exists for.

    A lattice small enough to enumerate every path -- 6 states over 4 segments
    is 1296 -- with an irregular clear table so the answer is not something a
    greedy walk would stumble onto anyway.
    """
    work = [-4.0, 0.0, 4.0]
    travel = [0.0, 10.0]
    clear = [
        [[6.0, 1.0], [2.0, 9.0], [0.0, 3.0]],
        [[0.0, 4.0], [7.0, 2.0], [5.0, 5.0]],
        [[3.0, 3.0], [1.0, 0.0], [8.0, 6.0]],
        [[9.0, 2.0], [4.0, 4.0], [2.0, 7.0]],
    ]
    problem = _problem(clear, work, travel, work_limits={"max_rate_deg_per_mm": 0.4})
    got = optimise(**problem)
    score, path = _brute_force(problem)

    assert got["clear_mm"] == pytest.approx(score[0])
    assert -got["deviation_deg_mm"] == pytest.approx(score[1])
    assert -got["variation_deg"] == pytest.approx(score[2])
    assert list(zip(got["work_bin_index"], got["travel_bin_index"], strict=True)) == [
        list(p) and tuple(p) for p in path
    ]


def test_the_dp_matches_brute_force_unconstrained():
    """The same, with no rate limit, so every path is legal and the search space
    is the full 1296."""
    work = [-2.0, 2.0]
    travel = [0.0, 5.0, 10.0]
    clear = [
        [[1.0, 8.0, 2.0], [4.0, 0.0, 6.0]],
        [[7.0, 1.0, 1.0], [0.0, 9.0, 3.0]],
        [[2.0, 2.0, 5.0], [6.0, 3.0, 0.0]],
        [[0.0, 4.0, 7.0], [8.0, 1.0, 2.0]],
    ]
    problem = _problem(clear, work, travel)
    got = optimise(**problem)
    score, _ = _brute_force(problem)
    assert got["clear_mm"] == pytest.approx(score[0])
    assert -got["deviation_deg_mm"] == pytest.approx(score[1])


def test_coverage_outranks_everything():
    """Lexicographic, not weighted: no amount of smoothness or fidelity buys a
    millimetre of coverage."""
    work = [0.0, 10.0]
    travel = [0.0]
    # Staying at the authored 0 covers nothing; the far bin covers everything.
    clear = [[[0.0], [10.0]] for _ in range(4)]
    got = optimise(**_problem(clear, work, travel))
    assert got["work_deg"] == [10.0] * 4
    assert got["clear_mm"] == 40.0


def test_a_tie_goes_to_the_authored_profile():
    """J2 is the tie-break, and it must pull toward what a person wrote."""
    work = [-4.0, 0.0, 4.0]
    travel = [0.0]
    clear = [[[10.0], [10.0], [10.0]] for _ in range(4)]
    got = optimise(**_problem(clear, work, travel, authored_work=[4.0] * 4))
    assert got["work_deg"] == [4.0] * 4, "should sit on the authored angle, not on zero"
    assert got["deviation_deg_mm"] == 0.0


def test_a_tie_in_coverage_and_fidelity_goes_to_the_smoothest():
    """J3, the last resort: same coverage, same deviation, fewer wiggles."""
    work = [-2.0, 2.0]
    travel = [0.0]
    clear = [[[10.0], [10.0]] for _ in range(4)]
    # Authored halfway between the two bins, so both are equally far.
    got = optimise(**_problem(clear, work, travel, authored_work=[0.0] * 4))
    assert got["variation_deg"] == 0.0, "a flat path beats a zig-zag at equal cost"
    assert len(set(got["work_deg"])) == 1


def test_the_band_is_a_hard_mask():
    """A profile outside its limits is not a worse weld, it is a different
    weld -- so coverage cannot buy its way past a declared limit."""
    work = [0.0, 10.0]
    travel = [0.0]
    clear = [[[0.0], [10.0]] for _ in range(4)]
    got = optimise(**_problem(clear, work, travel, work_limits={"max": 0.0}))
    assert got["work_deg"] == [0.0] * 4
    assert got["clear_mm"] == 0.0, "the mask cost everything, and that is correct"


def test_a_band_that_admits_no_bin_is_refused():
    work = [-4.0, 0.0, 4.0]
    with pytest.raises(OrientError, match="admits none"):
        optimise(**_problem([[[1.0]] * 3] * 2, work, [0.0], work_limits={"min": 20.0}))


def test_crossed_limits_are_refused():
    work = [0.0]
    with pytest.raises(OrientError, match="cross"):
        optimise(**_problem([[[1.0]]] * 2, work, [0.0], work_limits={"min": 5.0, "max": -5.0}))


def test_a_rate_limit_can_make_coverage_unreachable():
    """A limit on the RATE bounds change, not value, so it only bites when the
    coverage needs the angle to MOVE. Two opposite corridors do that; a single
    corridor the path can simply park inside never tests anything.
    """
    work = bin_values(-6.0, 6.0, 2.0)
    travel = [0.0]

    def cell(i, v):
        if i == 1:
            return 10.0 if v <= -4.0 else 0.0
        if i == 3:
            return 10.0 if v >= 4.0 else 0.0
        return 10.0

    clear = [[[cell(i, v)] for v in work] for i in range(5)]
    loose = optimise(**_problem(clear, work, travel, work_limits={"max_rate_deg_per_mm": 0.5}))
    tight = optimise(**_problem(clear, work, travel, work_limits={"max_rate_deg_per_mm": 0.1}))
    assert loose["clear_mm"] == 50.0
    assert tight["clear_mm"] == 40.0, "one corridor has to be given up"


def test_the_rate_limit_is_enforced_on_every_step():
    work = bin_values(-10.0, 10.0, 2.0)
    clear = [[[10.0] if v >= 8.0 else [0.0] for v in work] for _ in range(4)]
    got = optimise(**_problem(clear, work, [0.0], work_limits={"max_rate_deg_per_mm": 0.2}))
    for a, b in zip(got["work_deg"], got["work_deg"][1:], strict=False):
        assert abs(b - a) <= 0.2 * 10.0 + 1e-9, got["work_deg"]


def test_the_search_is_reproducible():
    """An optimiser that is not deterministic cannot be debugged, and a project
    that reopens to a different answer cannot be reviewed."""
    work = [-4.0, 0.0, 4.0]
    travel = [0.0, 10.0]
    clear = [[[5.0, 5.0], [5.0, 5.0], [5.0, 5.0]] for _ in range(6)]
    runs = [optimise(**_problem(clear, work, travel)) for _ in range(20)]
    assert all(r == runs[0] for r in runs)


def test_a_malformed_lattice_is_refused_rather_than_guessed_at():
    with pytest.raises(OrientError, match="no segments"):
        optimise(**_problem([], [0.0], [0.0]))
    with pytest.raises(OrientError, match="arc_mm"):
        optimise(
            clear_mm=[[[1.0]], [[1.0]]],
            work_bins=[0.0],
            travel_bins=[0.0],
            arc_mm=[10.0],
            spacing_mm=[10.0],
            authored_work=[0.0, 0.0],
            authored_travel=[0.0, 0.0],
        )


# ------------------------------------------------------------------- profiles


def test_a_flat_path_becomes_a_constant():
    """So the optimiser and the spline editor write the SAME document for the
    same shape -- otherwise a profile nobody touched reads as edited the moment
    it is optimised."""
    got = to_profile([0.0, 0.25, 0.5, 0.75, 1.0], [7.0] * 5, 1.0)
    assert got == {"kind": "constant", "value": 7.0}


def test_a_ramp_needs_no_interior_knots():
    got = to_profile([0.0, 0.25, 0.5, 0.75, 1.0], [0.0, 2.5, 5.0, 7.5, 10.0], 0.5)
    assert got["kind"] == "piecewise_linear"
    assert got["knots"] == [[0.0, 0.0], [1.0, 10.0]]


def test_the_ends_are_pinned_to_zero_and_one():
    """The schema requires s to run 0..1, and a profile starting at 0.06 would
    be silently extended by every consumer's clamp."""
    got = to_profile([0.06, 0.5, 0.94], [0.0, 9.0, 0.0], 0.5)
    assert got["knots"][0][0] == 0.0
    assert got["knots"][-1][0] == 1.0


def test_a_corner_survives_simplification():
    kept = simplify([0.0, 0.25, 0.5, 0.75, 1.0], [0.0, 0.0, 10.0, 0.0, 0.0], 1.0)
    assert 2 in kept, "the peak is the whole shape"
    assert kept[0] == 0 and kept[-1] == 4


def _at_fraction(profile, f):
    if profile["kind"] == "constant":
        return profile["value"]
    knots = profile["knots"]
    for (x0, y0), (x1, y1) in itertools.pairwise(knots):
        if x0 <= f <= x1:
            return y0 if x1 == x0 else y0 + (y1 - y0) * (f - x0) / (x1 - x0)
    return knots[-1][1]


def test_simplification_measures_the_deviation_in_degrees():
    """The textbook perpendicular distance is meaningless here: x is a
    dimensionless fraction and y is degrees, so the "distance" has no unit and
    dividing by the chord's own length makes the tolerance depend on its slope.

    This is the exact path that exposed it -- the first real seam the optimiser
    was run against. Under the perpendicular measure the whole thing collapsed
    to a straight 20 -> 10 ramp, throwing away four percentage points of the
    coverage the search had just found.
    """
    fractions = [i / 8 for i in range(9)]
    values = [20.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0, 10.0]
    kept = simplify(fractions, values, 2.5)
    assert 1 in kept, "an 8.75 degree departure from the chord is not a straight line"
    assert 7 in kept, "nor is a 10 degree notch"


def test_every_point_survives_within_the_tolerance():
    """The actual contract of the simplification, and what the unit bug broke:
    the emitted profile is what gets welded, so no sample of the searched path
    may sit further than the tolerance from it."""
    cases = [
        [20.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.0, 10.0],
        [0.0, 2.0, 4.0, 6.0, 4.0, 2.0, 0.0, -2.0, -4.0],
        [-6.0, -6.0, 6.0, 6.0, -6.0, -6.0, 6.0, 6.0, 0.0],
    ]
    fractions = [i / 8 for i in range(9)]
    for values in cases:
        profile = to_profile(fractions, values, 1.0)
        for f, want in zip(fractions, values, strict=True):
            assert abs(_at_fraction(profile, f) - want) <= 1.0 + 1e-9, (values, f)


def test_simplification_cannot_break_the_rate_limit():
    """Douglas-Peucker draws a chord with the AVERAGE slope of the steps it
    replaced, and an average cannot exceed a bound every step satisfied. The
    check inside `to_profile` is therefore an assertion about the code, and this
    test is what says it never fires on a legal path.
    """
    n = 21
    fractions = [i / (n - 1) for i in range(n)]
    # A path that rides exactly at the limit the whole way: 0.5 deg/mm over a
    # 100 mm seam in 20 steps is 2.5 degrees a step.
    values = [2.5 * i for i in range(n)]
    got = to_profile(fractions, values, 1.0, length_mm=100.0, max_rate_deg_per_mm=0.5)
    assert got["kind"] == "piecewise_linear"

    # And a path that breaks it is reported rather than quietly emitted.
    with pytest.raises(OrientError, match="already broke it"):
        to_profile([0.0, 1.0], [0.0, 100.0], 0.5, length_mm=100.0, max_rate_deg_per_mm=0.5)


def test_the_default_knot_spacing_leaves_room_to_move():
    """The whole reason the decision grid is coarser than the oracle's.

    At the oracle's 2 mm a 0.5 deg/mm limit permits 1 degree -- half of one
    2 degree work bin -- so no transition is ever legal and the search returns
    the authored profile while reporting success.
    """
    rate, work_bin = 0.5, 2.0
    assert rate * 2.0 < work_bin, "the oracle's own spacing would freeze the search"
    assert rate * DEFAULT_KNOT_SPACING_MM >= 2 * work_bin, "and the decision grid must not"


# --------------------------------------------------------- the standoff loop


def _lattice(slabs, standoffs, n=5, work=(-2.0, 0.0, 2.0), travel=(10.0,)):
    """A hand-built table: `slabs[d][knot][work][travel]` millimetres of arc."""
    return {
        "knot_fractions": [i / (n - 1) for i in range(n)],
        "spacing_mm": [10.0] * (n - 1),
        "arc_mm": [10.0] * n,
        "seam_length_mm": 50.0,
        "work_bins": list(work),
        "travel_bins": list(travel),
        "standoffs_mm": list(standoffs),
        "clear_mm": slabs,
    }


def _uniform(value, n=5, work=3, travel=1):
    return [[[value] * travel for _ in range(work)] for _ in range(n)]


def test_a_weld_with_no_stickout_band_gets_one_dp():
    """The default, and the whole defence against a confidently wrong weld:
    absent limits mean zero freedom, so the standoff never moves."""
    got = search(
        _lattice([_uniform(10.0)], [12.0]),
        authored_work=[0.0] * 5, authored_travel=[10.0] * 5,
        authored_standoff_mm=12.0,
    )
    assert got["standoff_mm"] == 12.0
    assert got["standoff_departure_mm"] == 0.0
    assert len(got["standoffs_tried"]) == 1


def test_coverage_outranks_the_standoff_price():
    """J1 is coverage and the price lives in J2, so no cost -- however large --
    can trade weld away to keep a procedure number. Worth pinning, because the
    tempting implementation is a weighted sum where a big enough constant
    silently does exactly that."""
    near = _uniform(10.0)
    near[2] = [[0.0], [0.0], [0.0]]
    lattice = _lattice([near, _uniform(10.0)], [10.0, 14.0])
    for cost in (0.0, 3.0, 1e6):
        got = search(
            lattice, authored_work=[0.0] * 5, authored_travel=[10.0] * 5,
            authored_standoff_mm=10.0, standoff_cost_deg_per_mm=cost,
        )
        assert got["standoff_mm"] == 14.0, cost
        assert got["clear_mm"] == 50.0


def test_the_price_breaks_a_coverage_tie():
    """Which is the only thing it can do -- and therefore the only claim the
    declared constant has to be right about."""
    lattice = _lattice([_uniform(10.0), _uniform(10.0), _uniform(10.0)], [10.0, 12.0, 14.0])
    got = search(
        lattice, authored_work=[0.0] * 5, authored_travel=[10.0] * 5,
        authored_standoff_mm=12.0, standoff_cost_deg_per_mm=3.0,
    )
    assert got["standoff_mm"] == 12.0
    assert [row["chosen"] for row in got["standoffs_tried"]] == [False, True, False]


def test_every_standoff_tried_is_reported_with_its_departure():
    """R-UI-2. A coverage number without the standoff that produced it is the
    provenance failure the UI rules exist against, and the operator has to be
    able to see what the band bought before accepting it."""
    lattice = _lattice([_uniform(6.0), _uniform(8.0), _uniform(10.0)], [10.0, 12.0, 14.0])
    got = search(
        lattice, authored_work=[0.0] * 5, authored_travel=[10.0] * 5,
        authored_standoff_mm=10.0,
    )
    rows = got["standoffs_tried"]
    assert [r["standoff_mm"] for r in rows] == [10.0, 12.0, 14.0]
    assert [r["departure_mm"] for r in rows] == [0.0, 2.0, 4.0]
    assert [r["clear_mm"] for r in rows] == [30.0, 40.0, 50.0]
    assert got["standoff_departure_mm"] == 4.0
    assert got["authored_clear_mm"] == 30.0, "the baseline is the authored standoff"


def test_an_authored_standoff_that_is_not_a_candidate_is_refused():
    """Otherwise "never worse than what was authored" is measured against a
    weld nobody authored."""
    with pytest.raises(OrientError, match="not among the lattice"):
        search(
            _lattice([_uniform(10.0)], [12.0]),
            authored_work=[0.0] * 5, authored_travel=[10.0] * 5,
            authored_standoff_mm=10.0,
        )


def test_the_search_emits_scalar_functions_the_schema_can_carry():
    """A flat answer must come out `constant`, or a profile the operator never
    touched would read as edited the moment it was optimised."""
    got = search(
        _lattice([_uniform(10.0)], [12.0]),
        authored_work=[0.0] * 5, authored_travel=[10.0] * 5,
        authored_standoff_mm=12.0,
    )
    assert got["work_angle_deg"] == {"kind": "constant", "value": 0.0}
    assert got["travel_angle_deg"] == {"kind": "constant", "value": 10.0}
    assert got["clear_fraction"] == 1.0


def test_a_ramped_answer_comes_out_as_knots_inside_the_rate_limit():
    corridor = [
        [[0.0], [0.0], [10.0]] if i >= 3 else [[10.0], [10.0], [0.0]]
        for i in range(5)
    ]
    got = search(
        _lattice([corridor], [12.0]),
        authored_work=[0.0] * 5, authored_travel=[10.0] * 5,
        authored_standoff_mm=12.0,
        work_limits={"max_rate_deg_per_mm": 0.5},
    )
    profile = got["work_angle_deg"]
    assert profile["kind"] == "piecewise_linear"
    assert profile["knots"][0][0] == 0.0 and profile["knots"][-1][0] == 1.0
    assert got["work_path_deg"][0] == 0.0 and got["work_path_deg"][-1] == 2.0
    assert got["clear_mm"] == 50.0


def test_a_negative_standoff_price_is_refused():
    with pytest.raises(OrientError, match="must not be negative"):
        search(
            _lattice([_uniform(10.0)], [12.0]),
            authored_work=[0.0] * 5, authored_travel=[10.0] * 5,
            authored_standoff_mm=12.0, standoff_cost_deg_per_mm=-1.0,
        )


def test_the_standoff_search_is_reproducible():
    lattice = _lattice([_uniform(10.0), _uniform(10.0)], [10.0, 12.0])
    args = {
        "authored_work": [0.0] * 5, "authored_travel": [10.0] * 5,
        "authored_standoff_mm": 10.0,
    }
    runs = [search(lattice, **args) for _ in range(20)]
    assert all(run == runs[0] for run in runs)


def test_self_test_entry_point_passes():
    from seam_worker.reach.orient import self_test

    assert self_test() == 0


def test_orient_imports_nothing_from_the_geometry_stack():
    """`orient.py` is pure on purpose: it is why this file runs in milliseconds
    and can be checked against brute force at all.

    Checked over the IMPORT statements, not the source text -- the docstring
    names `clearance.py` when explaining where the lattice comes from, and a
    substring search calls that a dependency. The rule is about what the module
    reaches for, not about what it mentions.
    """
    import ast
    import inspect

    from seam_worker.reach import orient

    tree = ast.parse(inspect.getsource(orient))
    imported: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])

    banned = {"OCP", "cadquery", "topology", "clearance", "access", "curve", "frames"}
    assert not (imported & banned), f"orient.py imports {imported & banned}"
    assert "math" in imported, "the module should still be doing arithmetic"


# --------------------------------------------------------------------------
# running the seam the other way round
# --------------------------------------------------------------------------


def _directional_lattice():
    """A lattice where one direction is plainly better than the other.

    Symmetric bins, because a mirrored table has to be able to hold the negated
    angles. Clear arc is put at a POSITIVE work angle only, so reading the table
    mirrored -- which is what welding it backwards does -- finds it at the
    negative one instead.
    """
    work = [-4.0, -2.0, 0.0, 2.0, 4.0]
    travel = [-10.0, 0.0, 10.0]
    knots = 3
    clear = [
        [[[10.0 if w > 0 else 0.0 for _ in travel] for w in work] for _ in range(knots)]
    ]
    return {
        "clear_mm": clear,
        "work_bins": work,
        "travel_bins": travel,
        "standoffs_mm": [12.0],
        "arc_mm": [10.0] * knots,
        "spacing_mm": [10.0] * (knots - 1),
        "knot_fractions": [0.0, 0.5, 1.0],
        "seam_length_mm": 30.0,
    }


def test_mirroring_is_a_negation_of_both_angles():
    """Reversing travel negates the tangent, and `apply_angles` turns that into
    `u(-t, w, tr) == u(t, -w, -tr)` exactly. So the reversed table is this one
    read at the negated angles -- no rays, just indices."""
    from seam_worker.reach.orient import mirror_clear

    work = [-4.0, -2.0, 0.0, 2.0, 4.0]
    travel = [-10.0, 0.0, 10.0]
    original = [[[100 * a + b for b in range(len(travel))] for a in range(len(work))]]
    mirrored = mirror_clear(original, work, travel)
    for a, w in enumerate(work):
        for b, t in enumerate(travel):
            back_a = work.index(-w)
            back_b = travel.index(-t)
            assert mirrored[0][a][b] == original[0][back_a][back_b]


def test_mirroring_asymmetric_bins_is_refused():
    """Rather than silently truncated. The negated angle simply is not in the
    table, and a mirror that quietly dropped it would be wrong rather than
    missing."""
    from seam_worker.reach.orient import mirror_clear

    with pytest.raises(OrientError, match="symmetric"):
        mirror_clear([[[1.0]]], [5.0], [0.0])


def test_reversing_is_offered_and_wins_when_it_buys_arc():
    lattice = _directional_lattice()
    # The window keeps the search in the NEGATIVE half, where the forward table
    # has no clear arc at all. Mirrored, that same half is the clear one -- so
    # the only way to weld this seam is to run it the other way round, which is
    # exactly the situation the feature is for.
    window = {"min": [-4.0] * 3, "max": [-2.0] * 3}
    forward = search(
        lattice, authored_work=[-3.0] * 3, authored_travel=[0.0] * 3,
        authored_standoff_mm=12.0, work_limits=window,
    )
    both = search(
        lattice, authored_work=[-3.0] * 3, authored_travel=[0.0] * 3,
        authored_standoff_mm=12.0, work_limits=window,
        directions=("forward", "reverse"), authored_direction="forward",
    )
    assert forward["clear_mm"] == 0.0, "the fixture does not trap the forward search"
    assert both["clear_mm"] >= forward["clear_mm"]
    assert both["direction"] == "reverse"
    assert both["direction_changed"] is True
    assert both["direction_clear_mm"]["reverse"] > both["direction_clear_mm"]["forward"]


def test_a_direction_is_reported_even_when_it_does_not_change():
    """"We looked and forward won" and "we never looked" are different facts,
    and only one of them makes the direction a choice."""
    plan = search(
        _directional_lattice(), authored_work=[3.0] * 3, authored_travel=[0.0] * 3,
        authored_standoff_mm=12.0, directions=("forward", "reverse"),
    )
    assert plan["direction"] == "forward"
    assert plan["direction_changed"] is False
    assert set(plan["direction_clear_mm"]) == {"forward", "reverse"}


def test_direction_is_decided_on_coverage_alone():
    """Not on the whole lexicographic tuple.

    Measured on pipe_bracket: comparing on the full tuple turned four seams round
    and two of them gained 0.0 percentage points -- reverse merely won the
    deviation tie-break. Turning a weld round moves which end the arc strikes;
    it is not a tie-break-sized change, and a list quietly reordering itself to
    save a fraction of a degree is churn nobody spots.
    """
    # Same clear arc either way, so J1 ties. Authored angles sit at +2, which
    # is exactly reachable forward and needs the mirror to reach reversed --
    # so reverse would win on deviation if deviation were allowed to decide.
    work = [-2.0, 0.0, 2.0]
    travel = [0.0]
    lattice = {
        "clear_mm": [[[[10.0] for _ in work] for _ in range(2)]],
        "work_bins": work, "travel_bins": travel, "standoffs_mm": [12.0],
        "arc_mm": [10.0, 10.0], "spacing_mm": [10.0],
        "knot_fractions": [0.0, 1.0], "seam_length_mm": 20.0,
    }
    plan = search(
        lattice, authored_work=[2.0, 2.0], authored_travel=[0.0, 0.0],
        authored_standoff_mm=12.0, directions=("forward", "reverse"),
        authored_direction="forward",
    )
    assert plan["direction_clear_mm"]["forward"] == plan["direction_clear_mm"]["reverse"]
    assert plan["direction"] == "forward", "a tie in arc turned the weld round anyway"


def test_the_authored_direction_wins_a_tie_whichever_it_is():
    """Including when the weld is already reversed -- the rule is "do not churn",
    not "prefer forward"."""
    work = [-2.0, 0.0, 2.0]
    lattice = {
        "clear_mm": [[[[10.0] for _ in work] for _ in range(2)]],
        "work_bins": work, "travel_bins": [0.0], "standoffs_mm": [12.0],
        "arc_mm": [10.0, 10.0], "spacing_mm": [10.0],
        "knot_fractions": [0.0, 1.0], "seam_length_mm": 20.0,
    }
    plan = search(
        lattice, authored_work=[0.0, 0.0], authored_travel=[0.0, 0.0],
        authored_standoff_mm=12.0, directions=("forward", "reverse"),
        authored_direction="reverse",
    )
    assert plan["direction"] == "reverse"
    assert plan["direction_changed"] is False


def test_every_direction_comes_back_as_a_full_plan():
    """So the caller can verify both and decide on what it measures.

    The lattice decides at 17 rays and 10 mm knots. Measured on dog_box, two
    seams tie there (98.8% either way) and separate at the fine fan (97.6%
    against 97.9%) -- and the coarse tie went to whichever was authored, so the
    better direction was being discarded before anything measured it properly.
    A provisional pick is still returned for callers that do not verify.
    """
    plan = search(
        _directional_lattice(), authored_work=[0.0] * 3, authored_travel=[0.0] * 3,
        authored_standoff_mm=12.0, directions=("forward", "reverse"),
    )
    assert set(plan["by_direction"]) == {"forward", "reverse"}
    for name, candidate in plan["by_direction"].items():
        assert candidate["direction"] == name
        # A FULL plan, not a score: the caller has to be able to pose the seam
        # at these angles and measure it.
        for field in ("work_angle_deg", "travel_angle_deg", "standoff_mm", "clear_fraction"):
            assert field in candidate, f"{name} is missing {field}"
    # The provisional pick is one of them, spread over the top level as before.
    assert plan["work_angle_deg"] == plan["by_direction"][plan["direction"]]["work_angle_deg"]


def test_each_direction_gets_its_own_standoff_ladder():
    """`standoffs_tried` reports the chosen standoff for THAT direction, so a
    caller that overrules the provisional pick still shows a consistent ladder
    beside the answer it took."""
    plan = search(
        _directional_lattice(), authored_work=[0.0] * 3, authored_travel=[0.0] * 3,
        authored_standoff_mm=12.0, directions=("forward", "reverse"),
    )
    for candidate in plan["by_direction"].values():
        chosen = [row for row in candidate["standoffs_tried"] if row["chosen"]]
        assert len(chosen) == 1
        assert chosen[0]["standoff_mm"] == candidate["standoff_mm"]
