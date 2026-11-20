"""The indexed intersector, against the exact one it replaced.

`access.py` used to intersect every ray with every exact surface of the shape,
with no spatial index: 130 us per ray, and six minutes for one exhaust manifold.
`occlusion.py` answers the same question against a BVH over triangles at about
1.5 us per ray. That is only allowed to be the default if it gives the same
answers, so this file is the comparison rather than a claim in a docstring.

The load-bearing test is `test_both_engines_agree_on_pipe_bracket`. It runs the
whole stage twice, once through each intersector, and demands the blocked spans
match -- because the blocked span is what becomes a candidate seam, and a
reachability answer that drifts silently is exactly the class of failure R-UI-1
exists against.

`test_the_default_deflection_stays_under_the_surface_skip` is a regression for a
wrong answer that measured beautifully: at 0.25 mm deflection against a 0.05 mm
skip, a sample point on the exact bore circle of a pipe sits BEHIND the facet
chords -- inside mesh material -- so every ray registers an immediate hit and the
bore read 50 of 53 samples blocked where the truth is 22.
"""

import json

import numpy as np
import pytest
from seam_worker.reach.access import (
    DEFAULT_ENGINE,
    SURFACE_SKIP,
    annotate,
    hemisphere,
    make_engine,
)
from seam_worker.seam.detect import detect_joints
from seam_worker.cad.occlusion import (
    DEFAULT_DEFLECTION_MM,
    DEFAULT_DILATION_MM,
    Engine,
    as_array,
    build,
    self_test,
    triangles,
)
from seam_worker.cad.topology import load_step

MM = 1000.0


@pytest.fixture(scope="module")
def pipe_bracket_joints(pipe_bracket_step, pipe_bracket_topology):
    return detect_joints(pipe_bracket_topology)


@pytest.fixture(scope="module")
def annotated(pipe_bracket_step, pipe_bracket_topology, pipe_bracket_joints):
    """The whole stage, once per engine. Module-scoped: the occt run is seconds."""
    out = {}
    for engine in ("bvh", "occt"):
        fresh = json.loads(json.dumps(pipe_bracket_joints))
        out[engine] = annotate(pipe_bracket_step, pipe_bracket_topology, fresh, engine=engine)
    return out


# --------------------------------------------------------------------------
# parity: the reason this is allowed to be the default
# --------------------------------------------------------------------------


def test_both_engines_agree_on_pipe_bracket(annotated):
    """Same blocked spans, same lengths, same worst-case open fraction.

    Measured identical to four decimals of a millimetre on every joint, so this
    asserts tightly rather than with a generous tolerance: a loose bound here
    would let a real divergence through, which is the only thing the test is for.
    """
    bvh, occt = annotated["bvh"], annotated["occt"]
    assert bvh["summary"]["blocked_length_m"] == pytest.approx(
        occt["summary"]["blocked_length_m"], abs=1e-6
    )

    for a, b in zip(bvh["weld_joints"], occt["weld_joints"], strict=True):
        one, two = a["accessibility"], b["accessibility"]
        assert a["id"] == b["id"]
        assert len(one["blocked_spans"]) == len(two["blocked_spans"]), a["id"]
        assert one["blocked_length_m"] == pytest.approx(two["blocked_length_m"], abs=1e-7), a["id"]
        assert one["min_open_fraction"] == pytest.approx(two["min_open_fraction"], abs=1e-6)
        # Same samples escalated means the coarse pass agreed too, not just the
        # verdict it led to.
        assert one["escalated_samples"] == two["escalated_samples"], a["id"]
        for span_a, span_b in zip(one["blocked_spans"], two["blocked_spans"], strict=True):
            assert span_a == pytest.approx(span_b, abs=1e-4), a["id"]


def test_the_probe_points_in_the_module_docstring_hold_for_both_engines(pipe_bracket_step):
    """The four points `access.py` documents: the gusset centre is sealed and the
    three around it are progressively more open. Both intersectors, one call each,
    so the docstring stays true of whichever one is in force."""
    shape = load_step(pipe_bracket_step)
    probes = [(25.0, 0.0, 10.0), (25.0, 5.0, 10.0), (25.0, 8.0, 10.0), (25.0, 20.0, 10.0)]
    dirs = hemisphere(256, (0.0, 1.0, 1.0))

    for engine_name in ("bvh", "occt"):
        engine = make_engine(shape, engine_name, key=f"probe-{engine_name}")
        arr = as_array(dirs) if engine_name == "bvh" else dirs
        counts = [engine.escaping(p, arr) for p in probes]
        assert counts[0] == 0, f"{engine_name}: the gusset centre should be sealed"
        assert counts[1] > 0, f"{engine_name}: the gusset edge should not be sealed"
        # Monotone as the point moves clear of the gusset.
        assert counts[1] <= counts[2] < counts[3], f"{engine_name}: {counts}"


# --------------------------------------------------------------------------
# the tolerance that produced a wrong answer
# --------------------------------------------------------------------------


def test_the_default_deflection_stays_under_the_surface_skip():
    """The invariant behind the 50-versus-22 bore. If a facet may sit further
    from the true surface than the distance the ray test ignores, a point on the
    exact curve can be inside mesh material and everything reads blocked."""
    assert DEFAULT_DEFLECTION_MM < SURFACE_SKIP, (DEFAULT_DEFLECTION_MM, SURFACE_SKIP)


def test_asking_for_a_deflection_actually_changes_the_mesh(pipe_bracket_step):
    """The knob has to move something.

    `BRepMesh_IncrementalMesh` KEEPS a triangulation already hanging off the
    shape, so without `BRepTools.Clean_s` first, the deflection argument is
    accepted and ignored. Measured on the exhaust manifold before the fix: the
    same 357,810 triangles at every deflection from 0.01 mm to 1.0 mm.

    Two callers on one shape is the case that makes this more than a lost
    speedup. `access.py` needs a fine mesh because it decides whether a ray
    escapes through a narrow aperture; anything wanting a coarse one for speed
    would, on a shape `access` had already meshed, silently get the fine one --
    or worse, win the race and leave `access` reporting reachable where nothing
    is.
    """
    shape = load_step(pipe_bracket_step)
    fine = build(*triangles(shape, 0.01, 0.0), 0.01, 0.0)
    coarse = build(*triangles(shape, 1.0, 0.0), 1.0, 0.0)
    assert coarse.triangle_count < fine.triangle_count, (
        f"both meshed to {fine.triangle_count} triangles, so the deflection "
        "was ignored"
    )


def test_the_answer_is_stable_under_refinement(
    pipe_bracket_step, pipe_bracket_joints, pipe_bracket_topology
):
    """Refining the mesh must not move the verdict.

    This is the property the deflection default is really chosen for, and it is
    testable where the mechanism behind the manifold's coarse-mesh anomaly is
    not: if halving the deflection changed the answer, the default would be
    sitting somewhere the answer still depends on it.
    """
    blocked = []
    for deflection in (0.02, 0.01, 0.005):
        doc = annotate(
            pipe_bracket_step,
            pipe_bracket_topology,
            json.loads(json.dumps(pipe_bracket_joints)),
            engine="bvh",
            deflection_mm=deflection,
        )
        blocked.append(doc["summary"]["blocked_length_m"])
    assert blocked[0] == pytest.approx(blocked[1], abs=1e-6), blocked
    assert blocked[1] == pytest.approx(blocked[2], abs=1e-6), blocked


# --------------------------------------------------------------------------
# dilation: conservative on purpose
# --------------------------------------------------------------------------


def test_dilation_seals_a_slit_no_torch_could_enter():
    """A ray through a 0.05 mm gap is called blocked, because nothing weldable
    fits through it. Over-reporting occlusion is the safe direction for a module
    whose answer is "can this be reached at all"."""
    import cadquery as cq

    def open_through(gap_mm: float, dilation_mm: float) -> bool:
        left = cq.Workplane("XY").box(10.0, 40.0, 40.0).translate((-5.0 - gap_mm / 2, 0, 0)).val()
        right = cq.Workplane("XY").box(10.0, 40.0, 40.0).translate((5.0 + gap_mm / 2, 0, 0)).val()
        shape = cq.Compound.makeCompound([left, right])
        engine = Engine(
            build(*triangles(shape, 0.01, dilation_mm), 0.01, dilation_mm), SURFACE_SKIP
        )
        return engine.any_escape((0.0, 0.0, -60.0), as_array([(0.0, 0.0, 1.0)]))

    assert open_through(2.0, DEFAULT_DILATION_MM), "a 2 mm gap must stay open"
    assert not open_through(0.05, 0.2), "a 0.05 mm slit should be sealed by 0.2 mm of margin"


def test_the_margin_never_reports_less_than_the_exact_answer(
    annotated, pipe_bracket_step, pipe_bracket_topology, pipe_bracket_joints
):
    """The dilation is conservative: it can seal a gap, never open one.

    This test used to assert something stronger and, it turns out, wrong. It
    read `undilated < dilated` and explained the difference -- 57.4062 mm
    against 59.7500 mm -- as facet edges at span boundaries sitting fractionally
    inside the true surface and letting rays past that the real geometry stops.

    That was not the cause. The whole 2.34 mm was the accessibility pole: rays
    were spread about `groove_faces[0]`'s bisector for a whole joint, and on
    pipe_bracket's later segments that bisector is 90 degrees away. With the pole
    taken per segment the undilated mesh lands on 59.7500 too, and the gap this
    test was built around is gone. A mesh artefact and a sampling artefact
    looked identical from here, and the margin was credited with fixing one
    while the other was doing the damage.

    So what is asserted now is the property that is actually load-bearing --
    growing the triangles cannot make the assembly look MORE open -- plus the
    parity that the margin does not overshoot. That the margin does real work
    is shown directly, on geometry built for it, by
    `test_dilation_seals_a_slit_no_torch_could_enter`.
    """
    undilated = annotate(
        pipe_bracket_step, pipe_bracket_topology, json.loads(json.dumps(pipe_bracket_joints)),
        engine="bvh", dilation_mm=0.0,
    )["summary"]["blocked_length_m"]
    dilated = annotated["bvh"]["summary"]["blocked_length_m"]
    exact = annotated["occt"]["summary"]["blocked_length_m"]

    assert undilated <= dilated + 1e-9, "the margin opened a gap; it is supposed to seal them"
    assert dilated == pytest.approx(exact, abs=1e-6), "the margin overshot the exact answer"
    assert DEFAULT_DILATION_MM > 0.0


# --------------------------------------------------------------------------
# provenance and plumbing
# --------------------------------------------------------------------------


def test_the_engine_and_its_tolerances_are_recorded(annotated):
    """R-UI-2 for a derived answer: a verdict computed on a 0.01 mm tessellation
    must not be readable later as if it were exact."""
    summary = annotated["bvh"]["summary"]["accessibility_intersector"]
    assert summary == {
        "engine": "bvh",
        "triangles": summary["triangles"],
        "deflection_mm": DEFAULT_DEFLECTION_MM,
        "dilation_mm": DEFAULT_DILATION_MM,
    }
    assert summary["triangles"] > 0

    # `method` is the ALGORITHM and does not change with the intersector: same
    # hemisphere escape, same statistics, same meaning for every field. Only
    # `intersector` says which geometry answered, which is why the schema's
    # method enum stayed a single value.
    for engine_name in ("bvh", "occt"):
        block = annotated[engine_name]["weld_joints"][0]["accessibility"]
        assert block["method"] == "hemisphere_escape"
        assert block["intersector"]["engine"] == engine_name
    assert annotated["bvh"]["weld_joints"][0]["accessibility"]["intersector"][
        "deflection_mm"
    ] == DEFAULT_DEFLECTION_MM
    # The exact engine has no tessellation, so it reports none.
    assert annotated["occt"]["weld_joints"][0]["accessibility"]["intersector"] == {"engine": "occt"}


def test_the_accessibility_block_still_validates_against_the_contract(annotated, schema_dir):
    """The provenance is a schema change, so it is checked against the schema
    rather than assumed compatible -- `accessibility` is `additionalProperties:
    false`, which is how the first attempt at this was caught."""
    from jsonschema import Draft202012Validator

    schema = json.loads((schema_dir / "amr-weld-planner-v1.weld-program.v1.schema.json").read_text())
    block_schema = {
        **schema["$defs"]["weld_joint"]["properties"]["accessibility"],
        "$defs": schema["$defs"],
    }
    for engine_name in ("bvh", "occt"):
        for joint in annotated[engine_name]["weld_joints"]:
            errors = list(Draft202012Validator(block_schema).iter_errors(joint["accessibility"]))
            assert not errors, f"{engine_name} {joint['id']}: {[e.message for e in errors]}"


def test_the_default_engine_is_the_indexed_one():
    assert DEFAULT_ENGINE == "bvh"


def test_an_unknown_engine_is_refused(pipe_bracket_step):
    with pytest.raises(ValueError, match="unknown engine"):
        make_engine(load_step(pipe_bracket_step), "rtx")


def test_the_index_is_built_once_per_shape(pipe_bracket_step):
    """The cache that keeps the speedup: `annotate_joint` is the unit of
    parallelism, so without it every joint would re-tessellate."""
    from seam_worker.cad import occlusion

    shape = load_step(pipe_bracket_step)
    first = occlusion.index_for(shape, "cache-test")
    assert occlusion.index_for(shape, "cache-test") is first
    assert occlusion.index_for(shape, "cache-test", deflection_mm=0.02) is not first


def test_a_ray_from_inside_a_sealed_box_never_escapes():
    """The semantics in one assertion, independent of any fixture."""
    import cadquery as cq

    box = cq.Workplane("XY").box(20.0, 20.0, 20.0).val()
    engine = Engine(build(*triangles(box, 0.01, 0.0), 0.01, 0.0), SURFACE_SKIP)
    dirs = as_array(np.array(hemisphere(64, (0.0, 0.0, 1.0))))
    assert engine.escaping((0.0, 0.0, 0.0), dirs) == 0
    assert engine.escaping((0.0, 0.0, 200.0), dirs) > 0


def test_self_test_entry_point_passes():
    assert self_test() == 0
