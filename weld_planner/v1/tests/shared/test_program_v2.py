"""`robot.v4.program.v2`: the shared pre-compile weld program, both halves.

The structural half is the JSON Schema in `robot/v4/contracts/programs`; the
semantic half is `weldplan.program_v2.problems`. This file runs both on the
same fixtures and proves they agree, which matters because the Go and
TypeScript twins have only the semantic half and restate the schema by hand.
It also holds the vendored seam vocabulary equal to this module's own schema:
the copy exists because those twins have no schema library, and the copy is
only safe while something notices it drifting.

Lives in `tests/shared` because `weldplan` is co-owned and this must run
under both `-e default` and `-e motion`. No CAD, milliseconds.
"""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator
from weldplan.program_v2 import (
    FORBIDDEN_KEYS,
    FREESPACE_FORBIDDEN_KEYS,
    SCHEMA,
    problems,
    self_test,
)

VENDORED = (
    "identifier",
    "vec3",
    "unit_vec3",
    "scalar_function",
    "analytic",
    "curve_segment",
    "curve",
    "orientation_limit_axis",
    "seam",
    "weld_joint",
    "groove_face",
    "weave",
    "source",
    "workpiece",
)

# Node indexes in the complete fixture:
# 0 io, 1 home, 2 approach, 3 weld(seam), 4 transit, 5 io,
# 6 weld(posed_polyline), 7 dwell, 8 transit, 9 weld(manual_pose_wp),
# 10 retract, 11 home, 12 io.


@pytest.fixture(scope="module")
def schema(program_contracts_dir: Path) -> dict:
    return json.loads((program_contracts_dir / "robot.v4.program.v2.schema.json").read_text())


@pytest.fixture(scope="module")
def validator(schema: dict) -> Draft202012Validator:
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema)


@pytest.fixture
def complete(program_contracts_dir: Path) -> dict:
    return json.loads((program_contracts_dir / "fixtures/complete-program-v2.json").read_text())


def _refs(node, acc: set[str]) -> set[str]:
    if isinstance(node, dict):
        for key, value in node.items():
            if key == "$ref":
                acc.add(value)
            else:
                _refs(value, acc)
    elif isinstance(node, list):
        for value in node:
            _refs(value, acc)
    return acc


# --------------------------------------------------------------------------
# the schema
# --------------------------------------------------------------------------


def test_the_schema_is_self_contained(schema):
    """Read by path from four languages; a `$ref` to another document is a
    URL nobody can fetch."""
    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    for ref in _refs(schema, set()):
        assert ref.startswith("#/$defs/"), ref
        assert ref.split("/")[-1] in schema["$defs"], ref


def test_every_object_is_closed(schema):
    """The structural half of the no-solution rule. A field the schema does
    not name is refused, so a trajectory cannot ride in under a name the deny
    list did not think of. `extensions` is the one open door and is named."""
    open_objects = []

    def walk(node, path):
        if isinstance(node, dict):
            is_object = node.get("type") == "object" and "properties" in node
            if is_object and node.get("additionalProperties", True) is not False:
                open_objects.append(path)
            for key, value in node.items():
                walk(value, f"{path}.{key}")
        elif isinstance(node, list):
            for i, value in enumerate(node):
                walk(value, f"{path}[{i}]")

    walk(schema, "$")
    # `weld_joint.evidence` is open on purpose, in v1 and here: it holds the
    # detector's reasons, and a `coincident_face_pair` joint states different
    # ones (`normal_dot`, `overlap_relation`, `thickness_ratio`) than a
    # boundary-length match does. Closing it would mean naming every method's
    # vocabulary in advance. What closing it was for -- keeping solved
    # quantities out -- `_forbidden_problems` already gives: it walks the whole
    # document from `$` and does not stop at open objects.
    assert open_objects == ["$.$defs.weld_joint.properties.evidence"], open_objects


def test_the_vendored_seam_vocabulary_matches_this_modules_schema(schema, schema_dir):
    """Copied, not `$ref`'d, because Go and TypeScript have no schema library.
    Equal after parsing, or the copy is edited and the twins diverge."""
    source = json.loads((schema_dir / "amr-weld-planner-v1.weld-program.v1.schema.json").read_text())
    seam = copy.deepcopy(source["$defs"]["seam"])
    del seam["properties"]["process"]
    seam["required"] = [k for k in seam["required"] if k != "process"]
    # The one deliberate difference, D24: v1 carries `weld_joints` at top
    # level, program.v2 carries it INSIDE the workpiece, because a joint is an
    # objective fact about the part and that is what `workpiece` already holds.
    # Everything else about the object still has to match, and the joint list
    # itself is still vendored rather than rewritten.
    workpiece = copy.deepcopy(source["properties"]["workpiece"])
    workpiece["properties"]["weld_joints"] = source["properties"]["weld_joints"]
    expected = {
        **{name: source["$defs"][name] for name in VENDORED if name in source["$defs"]},
        "seam": seam,
        "weave": source["$defs"]["seam"]["properties"]["process"]["properties"]["weave"],
        "source": source["properties"]["source"],
        "workpiece": workpiece,
    }
    assert set(expected) == set(VENDORED)
    for name in VENDORED:
        assert schema["$defs"][name] == expected[name], (
            f"$defs/{name} has drifted from the weld planner schema; re-vendor it, do not edit the copy"
        )
    # closed under reference: a new internal $ref upstream widens the copy
    # or this fails, never a dangling ref in the contracts directory
    for ref in _refs({name: schema["$defs"][name] for name in VENDORED}, set()):
        assert ref.split("/")[-1] in VENDORED, f"{ref} is referenced but not vendored"


def test_geometry_seam_is_the_seam_plus_a_kind_an_inline_curve_and_its_provenance(schema):
    """Two additions and no subtractions. `kind` is the discriminator, and
    `search_provenance` says what chose the orientation beside it -- the seam's
    counterpart to `torch_angles_deg` on the classic geometry, and provenance
    in the same sense: the profiles are the authority and nothing re-derives
    them from it."""
    seam = schema["$defs"]["seam"]
    geometry = schema["$defs"]["geometry_seam"]
    for key, value in seam["properties"].items():
        if key != "curve":
            assert geometry["properties"][key] == value, key
    assert set(geometry["properties"]) - set(seam["properties"]) == {
        "kind", "search_provenance",
    }
    assert geometry["properties"]["curve"]["$ref"] == "#/$defs/curve"
    # Provenance is never required: a seam nobody searched has none.
    assert set(geometry["required"]) == set(seam["required"]) | {"kind", "curve"}


def test_search_provenance_keeps_values_rather_than_a_preset_to_resolve(schema):
    """The reason a preset can be edited safely. A weld that stored an id and
    resolved it later would change meaning when the library did, which is the
    property `resolve_weld_list` had and D24 gave up on purpose."""
    block = schema["$defs"]["search_provenance"]
    assert "preset_id" not in block["required"], "a label cannot be load-bearing"
    for key in ("work_deviation_deg", "travel_deviation_deg", "work_bin_deg",
                "travel_bin_deg", "knot_spacing_mm", "sample_step_mm",
                "standoff_mm", "standoff_cost_deg_per_mm", "intersector"):
        assert key in block["required"], f"{key} decides the answer and must be kept"
    assert block["additionalProperties"] is False


# --------------------------------------------------------------------------
# the fixtures, both layers
# --------------------------------------------------------------------------


def test_the_complete_fixture_passes_both_layers_and_covers_every_op(validator, complete):
    assert list(validator.iter_errors(complete)) == []
    assert problems(complete) == []
    ops = [node["op"] for node in complete["nodes"]]
    assert set(ops) == {"weld", "approach", "transit", "retract", "home", "dwell", "io"}
    kinds = {node["geometry"]["kind"] for node in complete["nodes"] if node["op"] == "weld"}
    assert kinds == {"seam", "posed_polyline", "manual_pose_wp"}
    assert complete["schema"] == SCHEMA


@pytest.mark.parametrize(
    ("name", "phrase", "structural"),
    [
        ("invalid-program-v2-freespace-solution.json", "'trajectory'", True),
        ("invalid-program-v2-dense-joints.json", "'q_rad'", True),
        ("invalid-program-v2-weld-parameters-filled.json", "reserved", True),
        # Structurally valid on purpose: two welds side by side is a shape
        # JSON Schema cannot forbid. The semantic layer is what catches it,
        # in all three languages.
        ("invalid-program-v2-adjacent-welds.json", "transit is missing", False),
        # Also structurally valid: a cross-reference is not something JSON
        # Schema can follow. The seam names a joint, the joint list is empty,
        # and only the semantic layer can say so.
        ("invalid-program-v2-dangling-joint-ref.json", "is not in workpiece.weld_joints", False),
        # Also semantic: every number in the block is fine on its own, and
        # only the pair -- a bin wider than the window it walked -- is wrong.
        ("invalid-program-v2-search-bin-wider-than-window.json", "which is one bin", False),
    ],
)
def test_each_invalid_fixture_is_refused_naming_the_defect(
    validator, program_contracts_dir, name, phrase, structural
):
    doc = json.loads((program_contracts_dir / "fixtures" / name).read_text())
    found = problems(doc)
    assert any(phrase in line for line in found), found
    errors = list(validator.iter_errors(doc))
    assert bool(errors) is structural, [e.message for e in errors]


def test_the_rails_weld_is_the_dog_box_example_seam_with_its_curve_inlined(
    complete, schema_dir
):
    """Derived, not authored. Regenerating the example forces the fixture to
    follow, so the fixture cannot quietly describe a seam that no longer
    exists."""
    example = json.loads((schema_dir / "examples/dog_box_lap_perimeter.json").read_text())
    seam = copy.deepcopy(example["seams"][0])
    process = seam.pop("process")
    seam["curve"] = example["weld_joints"][0]["weld_line"]
    weld = complete["nodes"][3]
    assert weld["op"] == "weld"
    # The provenance is not in the example -- the example is a weld program and
    # a weld program does not record how its angles were searched. It is the
    # one part of this node authored here rather than derived.
    provenance = weld["geometry"]["search_provenance"]
    assert weld["geometry"] == {"kind": "seam", "search_provenance": provenance, **seam}
    assert weld["travel_speed_mm_s"] == process["travel_speed_mm_s"]
    assert weld["weave"] == process["weave"]
    assert weld["weld_parameters"] == {}
    assert complete["source"] == example["source"]
    # workpiece carries the joint list in program.v2, where v1 has it at top
    # level (D24). Derived from the same example, so a re-detected joint moves
    # the fixture rather than leaving it describing a joint nobody found.
    workpiece = copy.deepcopy(example["workpiece"])
    workpiece["weld_joints"] = example["weld_joints"]
    assert complete["workpiece"] == workpiece


def test_the_classic_weld_carries_its_baked_rpy_verbatim(complete):
    """The classic flow's orientation is authority. Every point along the
    straight edge carries the same RPY, which is what `torchOrientationRpy`
    produces for a constant travel direction; the angles ride along only as
    provenance."""
    weld = complete["nodes"][6]
    geometry = weld["geometry"]
    assert geometry["kind"] == "posed_polyline"
    rpys = {tuple(point["rpy_rad"]) for point in geometry["points"]}
    assert len(rpys) == 1
    assert geometry["torch_angles_deg"] == {"work": 45.0, "travel": 10.0, "spin": 0.0}


def test_the_manual_weld_is_hand_placed_poses_in_the_workpiece_frame(complete):
    """The Waypoint weld tool's form, kept editable: the points and the
    continuity rule are the weld, and no resolved curve rides along."""
    weld = complete["nodes"][9]
    geometry = weld["geometry"]
    assert geometry["kind"] == "manual_pose_wp"
    assert geometry["frame_id"] == complete["workpiece"]["frame_id"]
    assert set(geometry) == {"kind", "frame_id", "points"}
    assert [p["continuity"] for p in geometry["points"]] == ["C0", "C0", "C0"]
    # The corner of the 60 mm + 40 mm L is at s = 0.6; the standoff knot is there.
    assert weld["standoff_mm"]["knots"][1] == [0.6, 15.0]


def test_manual_poses_are_refused_when_they_coincide_or_leave_the_workpiece(complete):
    def copy_onto_previous(d):
        points = d["nodes"][9]["geometry"]["points"]
        points[1] = copy.deepcopy(points[0])

    _expect(complete, copy_onto_previous, "points 0 and 1 coincide")
    _expect(
        complete,
        lambda d: d["nodes"][9]["geometry"].__setitem__("frame_id", "world"),
        "is not the workpiece frame",
    )
    _expect(
        complete,
        lambda d: d["nodes"][9]["geometry"]["points"][1].pop("continuity"),
        "continuity must be C0, C1 or C2",
    )


def test_manual_poses_may_be_an_arc_of_three_off_a_line(complete):
    ok = copy.deepcopy(complete)
    ok["nodes"][9]["geometry"]["shape"] = "arc"
    assert problems(ok) == []

    def collinear(d):
        d["nodes"][9]["geometry"]["shape"] = "arc"
        d["nodes"][9]["geometry"]["points"][1]["xyz_m"] = [-0.07, 0.0425, 0.153]

    def four(d):
        d["nodes"][9]["geometry"]["shape"] = "arc"
        points = d["nodes"][9]["geometry"]["points"]
        points.append({**copy.deepcopy(points[2]), "xyz_m": [-0.02, 0.0225, 0.153]})

    _expect(complete, collinear, "lie on a line")
    _expect(complete, four, "exactly three")


# --------------------------------------------------------------------------
# the semantic layer
# --------------------------------------------------------------------------


def test_the_module_self_test_passes():
    assert self_test() == 0


def _expect(doc: dict, mutate, phrase: str) -> None:
    bad = copy.deepcopy(doc)
    mutate(bad)
    found = " ".join(problems(bad))
    assert phrase in found, found


def test_the_line_must_alternate_and_events_are_transparent(complete):
    nodes = complete["nodes"]
    _expect(complete, lambda d: d["nodes"].pop(4), "transit is missing")
    _expect(complete, lambda d: d["nodes"].__delitem__(slice(1, 3)), "approach is missing")
    _expect(complete, lambda d: d["nodes"].insert(5, {**nodes[1], "id": "home_mid"}), "middle")
    _expect(
        complete,
        lambda d: d["nodes"].insert(11, {**nodes[10], "id": "retract_again"}),
        "only a home or nothing may follow",
    )
    ok = copy.deepcopy(complete)
    ok["nodes"][3:3] = [{**nodes[0], "id": "io_early"}, {**nodes[7], "id": "dwell_early"}]
    assert problems(ok) == []


def test_solutions_are_refused_wherever_they_hide(complete):
    _expect(complete, lambda d: d["nodes"][4].__setitem__("trajectory", []), "'trajectory'")
    _expect(complete, lambda d: d["nodes"][4].__setitem__("points", []), "'points'")
    _expect(complete, lambda d: d["nodes"][3].__setitem__("q_rad", []), "'q_rad'")
    _expect(
        complete, lambda d: d["defaults"].__setitem__("knots", []), "'knots'"
    )
    # the scoped list is scoped: knots on a weld profile is seam vocabulary
    ok = copy.deepcopy(complete)
    ok["nodes"][3]["travel_speed_mm_s"] = {
        "kind": "piecewise_linear",
        "knots": [[0.0, 8.0], [1.0, 6.0]],
    }
    assert problems(ok) == []
    assert FREESPACE_FORBIDDEN_KEYS.isdisjoint(FORBIDDEN_KEYS)


def test_the_deny_lists_agree_with_the_twins(program_contracts_dir, repo_root):
    """Three hand-written lists in three languages. Read the other two out
    of their sources so a key added to one and not the others is a red test
    here rather than a document one validator accepts and another refuses."""
    go = (program_contracts_dir / "program_v2_test.go").read_text()
    ts = (
        repo_root / "offline-programming/v1/ui/src/contracts/robot-v4-program-v2.ts"
    )
    if not ts.exists():
        pytest.skip(f"TypeScript twin not present: {ts}")
    ts_text = ts.read_text()
    for key in FORBIDDEN_KEYS | FREESPACE_FORBIDDEN_KEYS:
        assert f'"{key}"' in go, f"Go twin is missing {key!r}"
        assert f'"{key}"' in ts_text, f"TypeScript twin is missing {key!r}"


def test_taught_moves_are_authored_targets_not_solved_trajectories(complete, validator):
    pose = {"frame_id": "world", "xyz_m": [0.,0.,0.], "rpy_rad": [0.,0.,0.], "joint_names": ["J1"], "joint_values_rad": [0.]}
    move = {"id": "p1", "op": "move", "enabled": True, "label": "P1", "comment": "", "row_hash": "pending:p1",
            "motion": "joint", "target": pose, "speed_scale": .1, "speed_mm_s": 25.,
            "capture": {"source": "preview", "observed_at": "2026-09-25T00:00:00Z", "model_id": "rosie_1420_v1", "robot_description_sha256": "sha256:" + "1"*64, "pose": copy.deepcopy(pose)}}
    complete["nodes"] = [move]
    assert list(validator.iter_errors(complete)) == []
    assert problems(complete) == []
    move["motion"] = "circular"
    assert problems(complete) == []  # Incomplete authoring; planner requires the via pose.
    move["target_space"] = "cartesian"
    assert list(validator.iter_errors(complete)) == []
    move["via"] = copy.deepcopy(pose)
    move["capture"]["pose"]["joint_values_rad"] = []
    assert any("capture.pose" in e for e in problems(complete))


def test_a_loop_return_home_may_follow_the_programs_own_last_home(complete, validator):
    # OLP loop mode appends a Home marked rosie.loop_return after the
    # program's last node; a second plain Home is still a Home in the middle.
    looped = copy.deepcopy(complete)
    looped["nodes"].append({**complete["nodes"][11], "id": "rosie-loop-return", "label": "Return to program start",
                            "extensions": {"rosie.loop_return": True}})
    assert problems(looped) == []
    validator.validate(looped)
    _expect(complete, lambda d: d["nodes"].append({**complete["nodes"][11], "id": "home_again"}), "middle")
