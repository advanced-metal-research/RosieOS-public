"""The plan request: what leaves this tool, and what it refuses to let leave.

No CAD here -- `write_request` is a serializer and a set of cross-checks, so
these are milliseconds. The end-to-end route is exercised in `test_server.py`.
"""

from __future__ import annotations

import copy
import json
import zipfile
from io import BytesIO

import pytest
from weldplan import program_v2
from weldplan.plan_request import (
    EXTENSION,
    MIMETYPE,
    SCHEMA,
    PlanRequestError,
    problems,
    read_request,
    write_request,
)

# A program.v2 with one seam weld, so `_request` exercises the container and
# not the document's own rules -- program_v2's tests own those. The orientation
# band is kept because it is the reason this handoff exists.
PROGRAM = program_v2._minimal()
PROGRAM["placement"] = {
    "anchor_xyz_m": [0.0, 0.0, 0.0],
    "offset_xyz_m": [0.1, 0.0, 0.1],
    "rpy_rad": [0.0, 0.0, 0.4],
    "cell_id": "c",
}
_WELD = next(node for node in PROGRAM["nodes"] if node["op"] == "weld")
_WELD["geometry"] = {
    "kind": "seam",
    "id": "seam_0001",
    "curve": {
        "segments": [
            {
                "degree": 1,
                "control_points_xyz_m": [[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]],
                "knots_u": [0.0, 0.0, 1.0, 1.0],
            }
        ]
    },
    "reference": {"mode": "rail"},
    "orientation": {
        "work_angle_deg": {"kind": "constant", "value": 0.0},
        "travel_angle_deg": {"kind": "constant", "value": 10.0},
        # The reason this handoff exists. A planner that cannot reach work 0
        # can trade inside this instead of failing or silently deviating.
        "limits": {
            "work_angle_deg": {
                "min": {"kind": "constant", "value": -6.0},
                "max": {"kind": "constant", "value": 6.0},
                "max_rate_deg_per_mm": 0.5,
            }
        },
    },
}

PLACEMENT = PROGRAM["placement"]

CELL = {
    "id": "c",
    "frames": {"base": "world", "work": "table"},
    "links": [{"name": "table", "visuals": [{"mesh": "meshes/table.dae"}]}],
    "joints": [],
    "axes": ["J7"],
}


def _request(**kwargs):
    body = {
        "request_id": "r",
        "program": PROGRAM,
        "cell": CELL,
        "step_bytes": b"ISO-10303-21;",
        "step_filename": "part.step",
        "created_utc": "2026-01-01T00:00:00Z",
    }
    body.update(kwargs)
    return write_request(**body)


# --------------------------------------------------------------------------
# what it carries
# --------------------------------------------------------------------------


def test_the_planning_context_travels_verbatim_and_verified():
    """The robot the plan is for, as the OLP compiled it, member for member
    (`docs/PLANNING-CONTEXT.md`). Packed, never read: a byte changed after
    packing is refused like any other member."""
    context = {"robot.urdf": b"<robot name='c'/>", "planner.json": b'{"schema":"x"}\n',
               "identity.json": b"{}\n", "meshes/table.dae": b"<COLLADA/>"}
    data = _request(context=context)
    opened = read_request(data)
    assert opened["context"] == context
    assert opened["manifest"]["planning_context"] == {
        "root": "context/",
        "files": ["identity.json", "meshes/table.dae", "planner.json", "robot.urdf"],
    }
    assert read_request(_request())["context"] is None

    packed = zipfile.ZipFile(BytesIO(data))
    tampered = BytesIO()
    with zipfile.ZipFile(tampered, "w") as out:
        for item in packed.infolist():
            payload = packed.read(item.filename)
            if item.filename == "context/robot.urdf":
                payload = b"<robot name='other'/>"
            out.writestr(item, payload)
    with pytest.raises(PlanRequestError, match="digest"):
        read_request(tampered.getvalue())

    for bad in ("../x", "/x", "a/../b"):
        with pytest.raises(PlanRequestError, match="relative"):
            _request(context={bad: b""})


def test_the_program_passes_through_verbatim():
    """Byte-identical to what `/api/programs` emits.

    The plan request must not edit the contract on its way out. If a plan request
    could rewrite a program even slightly, then "the program" would mean two
    different documents depending on which door it left by -- and the one the
    planner acted on would be the one nobody validated.
    """
    opened = read_request(_request())["program"]
    # Everything but the cell pin, which the packer resolves from the
    # descriptor it writes -- the producer had no bytes to hash.
    # Everything, the robot pin included: a descriptor with no identity has
    # nothing to resolve the placeholder to.
    assert opened == PROGRAM


def test_the_admissible_bands_survive_the_handoff():
    """What E is FOR. Without the band a planner has two choices when it cannot
    hit the authored angle: fail, or deviate and not say so."""
    seam = read_request(_request())["program"]["nodes"][3]["geometry"]
    limits = seam["orientation"]["limits"]["work_angle_deg"]
    assert limits["min"]["value"] == -6.0
    assert limits["max"]["value"] == 6.0
    assert limits["max_rate_deg_per_mm"] == 0.5


def test_the_placement_is_in_the_program_and_the_container_does_not_repeat_it():
    """It used to travel beside the program, because a weld program with a
    placement bolted in would be a statement about one table. program.v2 is a
    different document -- it already names the cell and every move the arm
    makes -- and D23 put the placement inside it, "because every planner needs
    it and it is small". The container's job is then to not carry a second
    copy, which could only ever disagree with the first."""
    data = _request()
    opened = read_request(data)
    assert opened["program"]["placement"] == PLACEMENT
    assert opened["placement"] == PLACEMENT, "read out of the program, under its old name"
    assert "placement.json" not in zipfile.ZipFile(BytesIO(data)).namelist()


def test_the_cell_is_inlined_rather_than_named():
    """A request naming a cell by id alone would resolve differently on another
    machine -- and the placement is measured from that cell's work frame, so a
    different one silently relocates the part."""
    cell = read_request(_request())["cell"]
    assert cell["id"] == "c"
    assert cell["frames"]["work"] == "table"
    assert cell["axes"] == ["J7"]


def test_the_cells_meshes_are_dropped_and_it_says_so():
    """The paths would dangle on the planner's machine, and it has its own
    copy. Dropping them silently would invite someone to resolve them, find
    nothing, and conclude the cell is empty."""
    cell = read_request(_request())["cell"]
    assert cell["links"][0]["visuals"] == []
    assert cell["meshes_omitted"] is True


def test_the_torch_is_in_the_request_and_never_in_the_weld_program():
    """Both halves of the rule at once. Which gun is fitted is a fact about
    this cell today; a torch inside the program would widen a
    tooling-independent contract by the act of handing it over."""
    torch = {"id": "mig_gas_nozzle_16", "body_radius_mm": 8.0}
    opened = read_request(_request(tooling=torch))
    assert opened["tooling"] == torch
    # Not a substring guard any more: program.v2 legitimately carries a
    # `torch_policy` in its freespace defaults, which is what the planner may
    # do with the gun and not which gun is fitted. The rule is about the
    # descriptor, so it is the descriptor's own fields that must be absent.
    blob = json.dumps(opened["program"])
    for token in ("mig_gas_nozzle_16", "body_radius_mm", "body_length_mm", "default_standoff_mm"):
        assert token not in blob, f"the program carries {token!r}, which is the gun"


def test_the_step_travels_with_it():
    """A planner has to collision-check the actual part, and a request naming a
    STEP by hash is unusable on a machine that has never seen it."""
    assert read_request(_request())["step_bytes"] == b"ISO-10303-21;"


# --------------------------------------------------------------------------
# what it refuses
# --------------------------------------------------------------------------


def test_a_request_without_a_placement_is_refused():
    with pytest.raises(PlanRequestError, match="no placement"):
        _request(program={**PROGRAM, "placement": None})


def test_a_placement_naming_another_cell_is_refused():
    """The dangerous one. Both halves are individually valid, and the result is
    a part somewhere nobody put it -- which comes back as a plan that does not
    fit, and reads as a planner bug."""
    with pytest.raises(PlanRequestError, match="mean something else"):
        _request(program={**PROGRAM, "placement": {**PLACEMENT, "cell_id": "a_different_cell"}})


def test_a_program_in_another_frame_is_refused():
    """A placement measures where the workpiece frame is. If the program's
    geometry is in some other frame, the two describe different things and
    every pose is off by however far apart they are."""
    with pytest.raises(PlanRequestError, match="different things"):
        _request(program={**PROGRAM, "workpiece": {"frame_id": "world"}})


def test_a_hand_placed_weld_that_was_not_lowered_is_refused():
    """The seam search reads seams only. A manual_pose_wp packed as-is would
    be skipped like a posed_polyline, and the plan would come back without
    that weld in it."""
    nodes = copy.deepcopy(PROGRAM["nodes"])
    weld = next(node for node in nodes if node["op"] == "weld")
    weld["geometry"] = {
        "kind": "manual_pose_wp",
        "frame_id": "workpiece",
        "points": [
            {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [3.14159, 0.0, 0.0], "continuity": "C1"},
            {"xyz_m": [0.1, 0.0, 0.0], "rpy_rad": [3.14159, 0.0, 0.0], "continuity": "C1"},
        ],
    }
    with pytest.raises(PlanRequestError, match="still manual_pose_wp"):
        _request(program={**PROGRAM, "nodes": nodes})


def test_an_empty_program_is_refused():
    with pytest.raises(PlanRequestError, match="nothing to plan"):
        _request(program={**PROGRAM, "nodes": []})


def test_a_cell_with_no_work_frame_is_refused():
    with pytest.raises(PlanRequestError, match="no work frame"):
        _request(cell={**CELL, "frames": {"base": "world"}})


def test_every_refusal_is_collected_not_just_the_first():
    """So a request with three things wrong takes one round trip to fix rather
    than three."""
    found = problems({"nodes": []}, None)
    assert len(found) >= 3


# --------------------------------------------------------------------------
# the container
# --------------------------------------------------------------------------


def test_it_is_identifiable_without_parsing_any_json():
    """The ODF trick, as in the project bundle: `mimetype` stored uncompressed
    and first sits at a fixed offset, so `file` and `unzip -p` can tell this
    from any other zip with a custom extension."""
    archive = zipfile.ZipFile(BytesIO(_request()))
    assert archive.namelist()[0] == "mimetype"
    assert archive.getinfo("mimetype").compress_type == zipfile.ZIP_STORED
    assert archive.read("mimetype").decode() == MIMETYPE
    assert EXTENSION == ".weldplan"
    assert json.loads(archive.read("manifest.json"))["schema"] == SCHEMA


def test_it_is_byte_stable():
    """Same inputs, same bytes. "Did the request change?" is the first question
    anyone asks when a plan comes back different."""
    assert _request() == _request()


def test_a_swapped_member_is_caught_by_its_digest():
    """The manifest declares a digest per entry, and they are checked on read.
    Otherwise a request could be edited after it was built and the planner
    would act on something this tool never validated."""
    original = zipfile.ZipFile(BytesIO(_request()))
    tampered = BytesIO()
    with zipfile.ZipFile(tampered, "w") as out:
        for item in original.infolist():
            payload = original.read(item.filename)
            if item.filename == "program.json":
                payload = json.dumps({**PROGRAM, "program_id": "swapped"}).encode()
            out.writestr(item.filename, payload)
    with pytest.raises(PlanRequestError, match="digest"):
        read_request(tampered.getvalue())


def test_another_format_is_refused_rather_than_half_read():
    with pytest.raises(PlanRequestError, match="mimetype"):
        read_request(_zip_without_mimetype())


def _zip_without_mimetype() -> bytes:
    buffer = BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        archive.writestr("manifest.json", json.dumps({"schema": SCHEMA}))
    return buffer.getvalue()


def test_self_test_entry_point_passes():
    from weldplan.plan_request import self_test

    assert self_test() == 0


# --------------------------------------------------------------------------
# the boundary
# --------------------------------------------------------------------------


def test_the_contract_package_imports_nothing_but_the_standard_library():
    """`weldplan` is what a motion planner opens a `.weldplan` with, and it has
    to be able to do that without this module's environment.

    A reader that needed `seam_worker` would be taking on OCCT, numba and
    cadquery to parse a zip of JSON -- and would be coupled to the producer's
    internals besides. The arrow points one way: `seam_worker` depends on
    `weldplan`, never the reverse. Asserted here rather than written down in
    `docs/motion/MOTION.md` alone, because a rule that only lives in prose is
    one import away from being untrue and nobody finds out until the planner
    will not install.

    Checked by reading the source rather than by inspecting `sys.modules`: an
    import inside a function would not have run yet at test time, and that is
    exactly the kind that gets added without anyone noticing.
    """
    import ast

    from seam_worker import paths

    package = paths.V1 / "python" / "weldplan"
    offenders = []
    for source in sorted(package.rglob("*.py")):
        for node in ast.walk(ast.parse(source.read_text(encoding="utf-8"))):
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ""]
            else:
                continue
            for name in names:
                if name.split(".")[0] in {"seam_worker", "numpy", "cadquery", "OCP", "numba"}:
                    offenders.append(f"{source.name}:{node.lineno} imports {name}")
    assert not offenders, "weldplan must stay dependency-free: " + "; ".join(offenders)


def test_the_authoring_side_re_exports_the_shared_check_rather_than_copying_it():
    """One definition of a well-formed placement, not two that drift.

    `seam_worker.robot.placement.problems` is this module's `placement_problems`.
    If someone re-implements it on the authoring side, a record the producer
    accepts becomes one the reader rejects -- and that failure surfaces as a
    plan request refused at the very end of the operator's work.
    """
    from seam_worker.robot.placement import problems as authoring_side
    from weldplan.plan_request import placement_problems

    assert authoring_side is placement_problems


def _repacked_with_program(program: dict) -> bytes:
    """The same container, with `program.json` swapped and the manifest's
    digest for it corrected -- so the refusal under test is about the
    program's SCHEMA and not about a digest that no longer matches."""
    import hashlib

    source = zipfile.ZipFile(BytesIO(_request()))
    payload = json.dumps(program, sort_keys=True, separators=(",", ":")).encode()
    manifest = json.loads(source.read("manifest.json"))
    for entry in manifest.get("contents") or []:
        if entry["path"] == "program.json":
            entry["bytes"] = len(payload)
            entry["sha256"] = hashlib.sha256(payload).hexdigest()
    out = BytesIO()
    with zipfile.ZipFile(out, "w") as archive:
        for name in source.namelist():
            if name == "program.json":
                archive.writestr(name, payload)
            elif name == "manifest.json":
                archive.writestr(name, json.dumps(manifest, sort_keys=True, separators=(",", ":")))
            else:
                archive.writestr(name, source.read(name))
    return out.getvalue()


def test_a_pre_v2_program_is_refused_rather_than_opened_empty():
    """The container has carried more than one program format, and shrugging at
    the old one was worse than refusing it.

    `PlanRequest.seams` reads weld NODES. A `weld-program.v1` document has
    none, so it opened cleanly, planned nothing, and came back as a 200 with an
    empty seam list -- which is indistinguishable from a part with nothing to
    weld. The whole committed corpus sat at v1 for that reason, and every test
    parametrized over it passed while proving nothing about a weld."""
    v1 = {"schema": "amr-weld-planner-v1.weld-program.v1", "seams": [], "program_id": "old"}
    with pytest.raises(PlanRequestError) as caught:
        read_request(_repacked_with_program(v1))
    assert "weld-program.v1" in str(caught.value)
    assert program_v2.SCHEMA in str(caught.value), "the refusal says what it does read"


def test_a_program_with_no_schema_at_all_is_refused():
    with pytest.raises(PlanRequestError, match="robot.v4.program.v2"):
        read_request(_repacked_with_program({"program_id": "nameless"}))


def test_the_operators_fixtures_travel_with_it_and_absence_is_not_an_empty_list():
    """The OLP's stand rides in `fixtures.json`, hashed like every member; a
    request that says nothing about fixtures reads back None, not [], because
    the seam worker keeps the cell's placeholder only for the former."""
    stand = {
        "id": "stand", "name": "Stand",
        "shape": {"kind": "box", "size_m": [0.5, 0.5, 1.0]},
        "xyz_m": [1.0, 1.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0],
    }
    data = _request(fixtures=[stand])
    opened = read_request(data)
    assert opened["fixtures"] == [stand]
    manifest = opened["manifest"]
    assert manifest["fixtures"] == {"path": "fixtures.json", "count": 1}
    assert any(entry["path"] == "fixtures.json" for entry in manifest["contents"])
    assert _request(fixtures=[stand]) == data, "same fixtures, same bytes"
    assert read_request(_request(fixtures=[]))["fixtures"] == []
    assert read_request(_request())["fixtures"] is None
    with pytest.raises(PlanRequestError):
        _request(fixtures={"id": "stand"})


def test_a_box_fixture_stands_on_its_footprint_where_it_was_placed():
    from weld_motion_planner.robot_cell.collision.fixtures import (
        FixtureError,
        fixture_mesh,
        fixture_meshes,
    )

    mesh = fixture_mesh({
        "id": "stand", "shape": {"kind": "box", "size_m": [0.5, 0.4, 1.0]},
        "xyz_m": [1.0, 2.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0],
    })
    low, high = mesh.bounds
    assert low.tolist() == pytest.approx([0.75, 1.8, 0.0])
    assert high.tolist() == pytest.approx([1.25, 2.2, 1.0])
    assert mesh.faces.shape == (12, 3)
    # A quarter turn about z swaps the footprint's sides.
    turned = fixture_mesh({
        "id": "stand", "shape": {"kind": "box", "size_m": [0.5, 0.4, 1.0]},
        "xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 1.5707963267948966],
    })
    low, high = turned.bounds
    assert low.tolist() == pytest.approx([-0.2, -0.25, 0.0], abs=1e-9)
    assert high.tolist() == pytest.approx([0.2, 0.25, 1.0], abs=1e-9)
    # A mesh is placed by its own origin.
    placed = fixture_mesh({
        "id": "clamp", "shape": {"kind": "mesh", "vertices": [0, 0, 0, 1, 0, 0, 0, 1, 0], "faces": [0, 1, 2]},
        "xyz_m": [0.0, 0.0, 0.3], "rpy_rad": [0.0, 0.0, 0.0],
    })
    assert placed.vertices[:, 2].tolist() == pytest.approx([0.3, 0.3, 0.3])
    assert fixture_meshes(None) == [] and fixture_meshes([]) == []
    with pytest.raises(FixtureError):
        fixture_mesh({"id": "x", "shape": {"kind": "cylinder"}})
    with pytest.raises(FixtureError):
        fixture_mesh({"id": "x", "shape": {"kind": "box", "size_m": [0.0, 1.0, 1.0]}})


def test_explicit_no_workpiece_roundtrip_keeps_cell_and_refuses_welds():
    pose = {"frame_id": "world", "xyz_m": [0.,0.,0.], "rpy_rad": [0.,0.,0.], "joint_names": ["J1"], "joint_values_rad": [0.]}
    program = copy.deepcopy(PROGRAM)
    program["nodes"] = [{"id": "p1", "op": "move", "enabled": True, "label": "P1", "comment": "", "row_hash": "pending:p1",
        "motion": "joint", "target": pose, "speed_scale": .1, "speed_mm_s": 25.,
        "capture": {"source": "preview", "observed_at": "2026-09-25T00:00:00Z", "model_id": "c", "robot_description_sha256": "sha256:"+"1"*64, "pose": copy.deepcopy(pose)}}]
    binding = {"model_id": "c", "robot_description_sha256": "sha256:"+"1"*64, "machine_planning_calibration_sha256": "sha256:"+"2"*64}
    cell = {**CELL, "axes": ["J1"], "robot_cell": binding}
    start = {"schema": "rosie.measured-planning-start.v1", "joint_names": ["J1"], "joint_values_rad": [.2], "robot_cell": binding, "coordinate_identity": "config:1:32", "observation_identity": "adapter:daemon", "observed_time_ns": "100"}
    packed = _request(program=program, cell=cell, step_bytes=None, start_state=start)
    assert read_request(packed)["start_state"] == start
    archive = zipfile.ZipFile(BytesIO(packed))
    assert any(e["path"] == "start_state.json" for e in json.loads(archive.read("manifest.json"))["contents"])

    opened = read_request(packed)
    assert opened["step_bytes"] is None
    assert opened["manifest"]["source"] == {"kind": "none"}
    assert opened["cell"]["id"] == CELL["id"]
    with pytest.raises(PlanRequestError, match="requires workpiece geometry"):
        _request(step_bytes=None)


def test_the_cells_outputs_travel_bound_and_unknown_channels_are_refused_at_pack():
    """`io_outputs.json` is the connected cell's outputs (weldplan.io_schedule),
    bound by the manifest like every member; an `io` row the cell cannot
    switch is refused before anything is packed."""
    import copy
    outputs = {"schema": "rosie.cell-io-outputs.v1", "torch_qualified": False,
               "outputs": [{"name": "gas", "class": "process", "expiry_ns": 200_000_000}]}
    program = copy.deepcopy(PROGRAM)
    program["nodes"] = [n for n in program["nodes"] if n["op"] != "io"]
    program["nodes"].append({"id": "gas_on", "op": "io", "enabled": True, "label": "Gas on", "comment": "",
                             "row_hash": "pending:gas_on", "channel": "gas", "state": True})
    opened = read_request(_request(program=program, io_outputs=outputs))
    assert opened["io_outputs"] == outputs
    assert read_request(_request())["io_outputs"] is None
    program["nodes"][-1]["channel"] = "revpi:1"
    with pytest.raises(PlanRequestError, match=r"Gas on \(io\) switches 'revpi:1', which is not an output of this cell"):
        _request(program=program, io_outputs=outputs)
    # Without the cell's outputs an offline export still packs its rows;
    # planning it refuses them (M6, weldplan.measured_start.refuse_io).
    assert read_request(_request(program=program))["io_outputs"] is None


def test_measured_start_is_bound_to_request_and_checked_on_read():
    from weldplan.measured_start import validate
    cell = {"axes": ["J1", "J2"], "robot_cell": {"model_id": "rosie_1420_v1", "robot_description_sha256": "sha256:model", "machine_planning_calibration_sha256": "sha256:calibration"}}
    program = {"nodes": [{"op": "move"}]}
    start = {"schema": "rosie.measured-planning-start.v1", "joint_names": cell["axes"], "joint_values_rad": [.2, -.1], "robot_cell": cell["robot_cell"], "coordinate_identity": "config:1:32", "observation_identity": "adapter:daemon", "observed_time_ns": "100"}
    assert validate(start, cell, program) is start
    for wrong in ({**start, "joint_names": ["J1"]}, {**start, "joint_values_rad": [0., float("nan")]}, {**start, "coordinate_identity": ""}, {**start, "robot_cell": {**cell["robot_cell"], "machine_planning_calibration_sha256": "changed"}}):
        with pytest.raises(ValueError): validate(wrong, cell, program)
    # Every motion node plans from the measured pose (2026-10-11); only an
    # output is refused, by name, because nothing would switch it.
    for nodes in ([{"op": "home", "label": "HOME"}, {"op": "move", "label": "P1"}],
                  [{"op": "move"}, {"op": "approach"}, {"op": "weld", "label": "W1"}, {"op": "retract"}, {"op": "home"}],
                  [{"op": "home"}, {"op": "approach"}, {"op": "weld"}, {"op": "retract"}],
                  [{"op": "home"}]):
        assert validate(start, cell, {"nodes": nodes}) is start
    with pytest.raises(ValueError, match=r"^IO switches an output: plan with the cell connected"):
        validate(start, cell, {"nodes": [{"op": "move"}, {"op": "io", "label": "IO"}]})
    with pytest.raises(ValueError, match=r"^Gas \(io\), IO switch outputs: plan with the cell connected"):
        validate(start, cell, {"nodes": [{"op": "io", "label": "Gas"}, {"op": "home"}, {"op": "io"}]})
    assert validate(start, cell, {"nodes": [{"op": "io", "enabled": False}, {"op": "move"}, {"op": "dwell"}]}) is start
    with pytest.raises(ValueError, match="at least one motion node"):
        validate(start, cell, {"nodes": [{"op": "dwell"}]})
