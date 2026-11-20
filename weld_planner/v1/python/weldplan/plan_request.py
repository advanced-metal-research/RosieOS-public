#!/usr/bin/env python
"""A plan request: everything a motion planner needs, and nothing it decides.

## Why this carries separate documents and not one bigger program

GOAL.md non-negotiable 5 says geometry is expressed in the workpiece frame and
nothing upstream may commit to a world pose. Non-negotiable 1 says the program
describes the weld and never the motion. Both are still true here -- so a
planner cannot be handed a weld program alone, because it does not say where
the part is, and it must not be handed a weld program with a placement bolted
into it, because that would make every program a statement about one table.

So the two travel TOGETHER and stay SEPARATE. `program.json` is byte-identical
to what `/api/programs` emits and stays subject to its own grep guard -- no
nozzle, no reach, no torch. `placement.json` sits beside it and says where the
part was fixtured. A planner reads both; a program consumer reads one and is
unaffected.

## What is in the plan request, and why each thing is there

- `program.json` -- `robot.v4.program.v2`: what to weld and every move the arm
  makes, as nodes. Carries the orientation profiles AND their admissible bands,
  which is what this whole handoff is for: a planner that cannot reach work 0
  degrees can trade inside the band instead of failing or silently deviating.
  It also carries the PLACEMENT, which used to travel beside it as
  `placement.json`. D23 put it inside the document because every planner needs
  it and it is small; two copies of it in one container could only disagree.
- `cell.json` -- the cell descriptor, INLINED, and since the planning
  context PARSED FROM IT: the packer reads it off `context/robot.urdf`
  (`weldplan.urdf.descriptor_of`), so the joints every stage walks are the
  corrected joints the OLP compiled. The same argument that inlines presets
  into a project bundle: a request naming a cell by id alone would resolve
  differently on another machine, and the placement is measured from that
  cell's work frame, so getting a different one silently relocates the part.
  program.v2 names the cell by id and sha256 instead, for the opposite and
  equally good reason -- a descriptor is a large opaque spreadsheet cell. The
  container is where the two meet: `write_request` stamps the digest of the
  bytes it writes, and `read_request` refuses a request whose program names a
  different one.
- `tooling.json` -- the torch. Legitimately here and legitimately NOT in the
  program: which gun is fitted is a fact about this cell today, and the
  clearance the planner must respect depends on it.
- `source/<name>.step` -- the geometry. A planner has to collision-check
  against the actual part, and a request that named a STEP by hash would be
  unusable on a machine that has never seen it.
- `context/` -- the PLANNING CONTEXT (`docs/PLANNING-CONTEXT.md`): the robot
  this plan is for, compiled once by the OLP from the description and the
  cell's `cell.json` -- the corrected URDF, the merged numbers
  (`planner.json`), the sphere model and the meshes. A planner reads these and
  never a description directory of its own: the second copy it used to find
  by name on its own disk is the one that drifted. Absent on a request from
  before the context; the planner then falls back to its local files.

## What is deliberately NOT in it

The cell's MESHES. The descriptor pins the kinematics -- joints, axes, limits,
frames -- which is what the placement depends on and what would silently
relocate the part if it differed. Collision geometry for the CELL is the
planner's own: it has the URDF, it has its own convention for visual versus
collision meshes (GOAL.md notes `tesseract` substitutes one for the other), and
shipping megabytes of DAE would not make its answer more correct.

No joint values either. The positioner is prescribed by the planner, not by
this tool -- which is why the plan tab has no axis sliders.

## Determinism

Same inputs, same bytes, same digests, like the project bundle. A handoff that
differs run to run cannot be diffed, and "did the request change?" is the first
question anyone asks when a plan comes back different.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import math
import sys
import zipfile
from pathlib import Path
from typing import Any

from weldplan import program_v2


def placement_problems(placement: dict[str, Any]) -> list[str]:
    """What is wrong with a placement record, as sentences.

    Lives here rather than in `seam_worker.robot.placement`, which re-exports it,
    because a placement record is one of the four documents inside the
    plan request: rejecting a malformed one is the reader's job, and the reader
    must not have to import the authoring side to do it. `math.isfinite` and
    not `np.isfinite` for the same reason -- the guard in front of it has
    already established that the value is a real int or float.
    """
    out: list[str] = []
    cell_id = placement.get("cell_id")
    if cell_id is not None and not (isinstance(cell_id, str) and cell_id):
        out.append(f"cell_id must be a cell name or null, got {cell_id!r}")
    for key, count in (("anchor_xyz_m", 3), ("offset_xyz_m", 3), ("rpy_rad", 3)):
        value = placement.get(key)
        if not isinstance(value, list | tuple) or len(value) != count:
            out.append(f"{key} must be {count} numbers")
            continue
        if any(
            not isinstance(v, int | float) or isinstance(v, bool) or not math.isfinite(v)
            for v in value
        ):
            out.append(f"{key} must be finite numbers, got {list(value)!r}")
    return out


SCHEMA = "amr-weld-planner-v1.plan-request.v1"
MIMETYPE = "application/vnd.gradient.amr-weld-planner-v1.plan-request"
EXTENSION = ".weldplan"

# Where the planning context's members sit inside the container.
CONTEXT_ROOT = "context/"

# The digest a producer writes when it has no descriptor bytes to hash (the
# tool), and the identity it writes when it had no description to pin the
# robot to.
_UNPINNED = "0" * 64
_UNPINNED_ROBOT = "sha256:" + "0" * 64

# The same fixed stamp the project bundle uses. Zip records a mtime per entry,
# so without this the bytes differ every run and the digests are unverifiable.
_FIXED_DATE = (1980, 1, 1, 0, 0, 0)


class PlanRequestError(ValueError):
    """A request that cannot be built or read, with a reason."""


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _canonical(value: Any) -> bytes:
    return json.dumps(value, indent=2, sort_keys=True).encode() + b"\n"


def problems(
    program: dict[str, Any],
    cell: dict[str, Any] | None,
) -> list[str]:
    """What is wrong with a request, as sentences the operator can act on.

    Checked here rather than trusted from the caller because a plan request
    leaves this tool. Everything after it is somebody else's process, and a
    request that is wrong in a way only the planner notices comes back as a
    plan that does not fit -- which reads as a planner bug.

    The program's own rules are program.v2's and are not restated here; this
    asks the questions only the container can, because only the container has
    the cell beside the program that names it.
    """
    out: list[str] = [f"program: {line}" for line in program_v2.problems(program)]

    # The one cross-check between the two halves. The program's geometry is in
    # the workpiece frame and the placement says where that frame is; if the
    # program declared some other frame the placement would be measuring the
    # wrong thing, and every pose would be off by however far apart they are.
    frame = (program.get("workpiece") or {}).get("frame_id")
    if frame is not None and frame != "workpiece" and any(n.get("op") == "weld" for n in program.get("nodes", [])):
        out.append(
            f"the program's geometry is in frame {frame!r}, but a placement is measured "
            "against the workpiece frame; the two describe different things"
        )

    # A hand-placed weld is the authoring tool's form. The seam search reads
    # seams, and a manual_pose_wp it could not see would be skipped the way a
    # posed_polyline is -- a plan that answers 200 without that weld in it. The
    # producer lowers it to a seam before packing (the OLP does); one that got
    # here was not lowered.
    for node in program.get("nodes") or []:
        geometry = node.get("geometry") if isinstance(node, dict) else None
        if isinstance(geometry, dict) and geometry.get("kind") == "manual_pose_wp":
            out.append(
                f"weld {node.get('id')!r} is still manual_pose_wp; the producer lowers "
                "hand-placed poses to a seam before a plan request is packed"
            )

    placement = program.get("placement")
    if not isinstance(placement, dict):
        out.append("no placement: the part has not been put anywhere, so no pose can be planned")
    else:
        out.extend(placement_problems(placement))
        named = placement.get("cell_id")
        if not named:
            out.append(
                "the placement does not say which cell it was authored against, so its "
                "offsets cannot be resolved to a work frame"
            )
        elif cell is not None and cell.get("id") != named:
            out.append(
                f"the placement was authored against cell {named!r} but cell "
                f"{cell.get('id')!r} is attached; its offsets mean something else there"
            )

    ref = program.get("robot")
    if isinstance(ref, dict) and cell is not None and ref.get("model_id") != cell.get("id"):
        out.append(
            f"the program is written for robot {ref.get('model_id')!r} but cell "
            f"{cell.get('id')!r} is attached"
        )

    if cell is None:
        out.append("no cell: a placement's offsets are measured from a work frame and there is none")
    elif not (cell.get("frames") or {}).get("work"):
        out.append(
            f"cell {cell.get('id')!r} has no work frame, so there is nowhere to fixture a part"
        )

    return out


def _pinned_robot(program: dict[str, Any], cell: dict[str, Any]) -> dict[str, Any]:
    """The program's robot pin against the description this request is packed
    with (`docs/PLANNING-CONTEXT.md`).

    The descriptor carries the identity the planning context was compiled
    from (`robot_cell.robot_description_sha256`). A program that pinned no description
    (the all-zero identity) takes it. A program that pinned THIS one is
    consistent. A program that pinned another is stale -- the description
    moved under it -- and the container is not entitled to decide that the
    placement survived: that is the typed adoption in the authoring tool.

    A descriptor with no identity (a request from before the contract) can
    check nothing, and the pin passes through as written.
    """
    ref = program.get("robot")
    if not isinstance(ref, dict):
        return program
    packed = str(((cell.get("robot_cell") or {}).get("robot_description_sha256")) or "")
    if not packed:
        return program
    declared = ref.get("robot_description_sha256")
    if declared == _UNPINNED_ROBOT:
        return {**program, "robot": {**ref, "robot_description_sha256": packed}}
    if declared != packed:
        raise PlanRequestError(
            f"the program is written for {ref.get('model_id')!r} at description "
            f"{str(declared)[:19]}... and this request packs {packed[:19]}...; the description "
            "changed since this program was created -- review the changes in Workspace, "
            "accept the current robot settings, and plan again"
        )
    return program


def _pinned(program: dict[str, Any], key: str, data: bytes | None) -> dict[str, Any]:
    """Stamp an asset ref with the digest of the bytes travelling beside it.

    program.v2 names the cell and the tool by id and sha256 and says the hash
    "is what lets a planner refuse a mismatch instead of silently planning
    against something else". A producer authoring against a live workstation
    has no descriptor bytes to hash and writes a placeholder, so the packer --
    the first place in the chain that HOLDS the descriptor -- resolves it.

    A producer that named a real, different digest is not corrected: that is a
    disagreement about which asset this program is for, and the container is
    not entitled to settle it.
    """
    ref = program.get(key)
    if not isinstance(ref, dict):
        return program
    if data is None:
        raise PlanRequestError(
            f"the program names a {key} but no {key} descriptor was given to pack beside it"
        )
    digest = _sha256(data)
    declared = ref.get("sha256")
    if isinstance(declared, str) and declared != _UNPINNED and declared != digest:
        raise PlanRequestError(
            f"the program pins {key} {ref.get('id')!r} at {declared[:12]}... and the "
            f"descriptor packed with it hashes to {digest[:12]}..."
        )
    return {**program, key: {**ref, "sha256": digest}}


def write_request(
    *,
    request_id: str,
    program: dict[str, Any],
    cell: dict[str, Any],
    step_bytes: bytes | None,
    step_filename: str,
    tooling: dict[str, Any] | None = None,
    label: str | None = None,
    created_utc: str | None = None,
    ui_version: str = "0.1.0",
    fixtures: list[dict[str, Any]] | None = None,
    context: dict[str, bytes] | None = None,
    start_state: dict[str, Any] | None = None,
    io_outputs: dict[str, Any] | None = None,
) -> bytes:
    """Serialize a plan request. Deterministic: same input, same bytes.

    `fixtures` are the workcell bodies the operator defined in the OLP (a
    stand under the part, a clamp), in the cell's world frame; see
    `weld_motion_planner.robot_cell.collision.fixtures` for the entry shape.
    `None` means the request says nothing about them and the cell's own
    placeholder, if it has one, stands; an empty list means there are none.

    `context` is the planning context's members by context-relative path,
    packed verbatim under `context/`: this is a packer, and the numbers in
    there were settled by whoever compiled them.
    """
    bad = problems(program, cell)
    if fixtures is not None and not isinstance(fixtures, list):
        bad.append("fixtures must be a list of fixture entries")
    if bad:
        raise PlanRequestError("; ".join(bad))
    if step_bytes is None and any(n.get("op") == "weld" for n in program.get("nodes", [])):
        raise PlanRequestError("A weld program requires workpiece geometry")
    from .measured_start import refuse_io, validate as validate_start
    try:
        validate_start(start_state, cell, program, io_outputs)
        # With the cell's outputs packed, every `io` row must be one it can
        # switch (`io_schedule.py`). Without them the request still packs (an
        # offline export carries its rows); planning refuses them (M6).
        if io_outputs is not None:
            refuse_io(program.get("nodes", []), io_outputs)
    except ValueError as exc:
        raise PlanRequestError(str(exc)) from exc
    context_members = _context_members(context)

    cell_bytes = _canonical(_without_meshes(cell))
    tooling_bytes = _canonical(tooling) if tooling is not None else None
    program = _pinned_robot(program, cell)
    program = _pinned(program, "tool", tooling_bytes)

    source_path = f"source/{Path(step_filename).name}"
    members: list[tuple[str, bytes]] = [
        *(([(source_path, step_bytes)]) if step_bytes is not None else []),
        ("program.json", _canonical(program)),
        ("cell.json", cell_bytes),
    ]
    if tooling_bytes is not None:
        members.append(("tooling.json", tooling_bytes))
    fixtures_bytes = _canonical(fixtures) if fixtures is not None else None
    if fixtures_bytes is not None:
        members.append(("fixtures.json", fixtures_bytes))
    if start_state is not None:
        members.append(("start_state.json", _canonical(start_state)))
    if io_outputs is not None:
        members.append(("io_outputs.json", _canonical(io_outputs)))
    members.extend(context_members)

    manifest = {
        "schema": SCHEMA,
        "request_id": request_id,
        "label": label,
        "created_utc": created_utc,
        "app": {"module": "amr-weld-planner/v1", "ui_version": ui_version},
        "program": {
            "path": "program.json",
            "program_id": program.get("program_id"),
            "schema": program.get("schema"),
            "node_count": len(program.get("nodes") or []),
            "weld_count": sum(
                1 for node in program.get("nodes") or []
                if isinstance(node, dict) and node.get("op") == "weld"
            ),
        },
        "cell": {"path": "cell.json", "id": cell.get("id")},
        **({"planning_context": {
            "root": CONTEXT_ROOT,
            "files": [path[len(CONTEXT_ROOT):] for path, _ in context_members],
        }} if context_members else {}),
        "source": {"kind": "none"} if step_bytes is None else {
            "kind": "step",
            "path": source_path,
            "filename": Path(step_filename).name,
            "sha256": _sha256(step_bytes),
            "bytes": len(step_bytes),
        },
        # Said out loud, because the absences are decisions and a reader
        # otherwise has to infer them from what is missing.
        "planner_owns": [
            "positioner joint values -- prescribed by the planner, not by this tool",
            "collision meshes for the CELL -- the descriptor here pins the kinematics only",
            "torch orientation within the admissible bands each seam declares",
        ],
        # The operator's own bodies, when the request carries them: what
        # the arm must clear that is neither the cell nor the part.
        "fixtures": (
            {"path": "fixtures.json", "count": len(fixtures)} if fixtures is not None else None
        ),
        "contents": [
            {"path": path, "sha256": _sha256(data), "bytes": len(data)} for path, data in members
        ],
    }

    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w") as archive:
        # The ODF trick, as in the project bundle: `mimetype` stored
        # uncompressed and first sits at a fixed offset, so the format is
        # identifiable without parsing any JSON.
        info = zipfile.ZipInfo("mimetype", date_time=_FIXED_DATE)
        info.compress_type = zipfile.ZIP_STORED
        archive.writestr(info, MIMETYPE.encode())
        for path, data in [("manifest.json", _canonical(manifest)), *members]:
            entry = zipfile.ZipInfo(path, date_time=_FIXED_DATE)
            entry.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(entry, data)
    return buffer.getvalue()


def _context_members(context: dict[str, bytes] | None) -> list[tuple[str, bytes]]:
    """The context's members as zip entries, in path order, each path checked:
    relative, `/`-separated, nowhere near `..`. A context is packed, never
    interpreted, so the only thing to refuse is a path that would land outside
    `context/` when unpacked."""
    if not context:
        return []
    out: list[tuple[str, bytes]] = []
    for path in sorted(context):
        parts = path.split("/")
        if not path or path.startswith("/") or "\\" in path or any(
                part in ("", ".", "..") for part in parts):
            raise PlanRequestError(f"planning context member {path!r} is not a relative path")
        data = context[path]
        if not isinstance(data, bytes | bytearray):
            raise PlanRequestError(f"planning context member {path!r} is not bytes")
        out.append((CONTEXT_ROOT + path, bytes(data)))
    return out


def _without_meshes(cell: dict[str, Any]) -> dict[str, Any]:
    """The cell's kinematics, with the visual mesh references dropped.

    The paths would be dangling on the planner's machine and it has its own
    copy anyway. Keeping them would invite someone to resolve them relative to
    the request, find nothing, and conclude the cell is empty -- which is the
    same failure mode as drawing a link at the origin.
    """
    return {
        **cell,
        "links": [{**link, "visuals": []} for link in (cell.get("links") or [])],
        "meshes_omitted": True,
    }


def read_request(data: bytes) -> dict[str, Any]:
    """Parse and VERIFY a request. Every declared digest is checked."""
    try:
        archive = zipfile.ZipFile(io.BytesIO(data))
    except zipfile.BadZipFile as exc:
        raise PlanRequestError(f"not a zip archive: {exc}") from exc

    names = set(archive.namelist())
    if "mimetype" not in names or archive.read("mimetype").decode() != MIMETYPE:
        raise PlanRequestError(
            f"not a {EXTENSION}: the mimetype entry is missing or is for another format"
        )
    if "manifest.json" not in names:
        raise PlanRequestError("no manifest.json")

    manifest = json.loads(archive.read("manifest.json"))
    if manifest.get("schema") != SCHEMA:
        raise PlanRequestError(f"unknown schema {manifest.get('schema')!r}")

    for entry in manifest.get("contents") or []:
        path = entry["path"]
        if path not in names:
            raise PlanRequestError(f"{path} is declared in the manifest and missing from the zip")
        payload = archive.read(path)
        if _sha256(payload) != entry["sha256"]:
            raise PlanRequestError(f"{path} does not match its declared digest")

    program = json.loads(archive.read("program.json"))
    # The container has carried more than one program format in its life, and
    # a reader that shrugged at the old one was worse than one that refused.
    # `PlanRequest.seams` reads weld NODES; a `weld-program.v1` document has
    # none, so it opened cleanly, planned nothing, and answered 200 with an
    # empty seam list. The whole committed corpus sat like that while the
    # tests over it stayed green.
    declared_program = program.get("schema")
    if declared_program != program_v2.SCHEMA:
        raise PlanRequestError(
            f"program.json is {declared_program!r}; this reads {program_v2.SCHEMA!r} only. "
            "A pre-v2 program has no weld nodes, so it would open and plan nothing — "
            "rebuild the request from a program.v2 document."
        )
    cell = json.loads(archive.read("cell.json"))
    tooling = json.loads(archive.read("tooling.json")) if "tooling.json" in names else None
    # The context's members are the ones the manifest declares -- and so the
    # ones the loop above verified -- never whatever else sits under context/.
    declared = manifest.get("planning_context")
    context: dict[str, bytes] | None = None
    if isinstance(declared, dict):
        root = str(declared.get("root") or CONTEXT_ROOT)
        context = {}
        for path in declared.get("files") or []:
            member = root + str(path)
            if member not in names:
                raise PlanRequestError(f"the planning context declares {member} and it is missing")
            context[str(path)] = archive.read(member)

    # The refusals the pins exist for. The digests above prove each member
    # is the one the manifest packed; these prove they are the ones the
    # PROGRAM was written against. Getting a different robot silently
    # relocates the part, because the placement is measured from that
    # description's work frame.
    robot = program.get("robot")
    packed = str(((cell.get("robot_cell") or {}).get("robot_description_sha256")) or "")
    if isinstance(robot, dict) and packed and robot.get("robot_description_sha256") != packed:
        raise PlanRequestError(
            f"the program is written for {robot.get('model_id')!r} at description "
            f"{str(robot.get('robot_description_sha256'))[:19]}... and the request packs {packed[:19]}..."
        )
    ref = program.get("tool")
    if isinstance(ref, dict):
        if "tooling.json" not in names:
            raise PlanRequestError("the program names a tool and no tooling.json is packed with it")
        declared = ref.get("sha256")
        actual = _sha256(archive.read("tooling.json"))
        if declared != actual:
            raise PlanRequestError(
                f"the program pins tool {ref.get('id')!r} at "
                f"{str(declared)[:12]}... and tooling.json hashes to {actual[:12]}..."
            )

    io_outputs = None
    if "io_outputs.json" in names:
        if not any(e.get("path") == "io_outputs.json" for e in manifest.get("contents", [])):
            raise PlanRequestError("the cell's outputs are not bound by the request manifest")
        io_outputs = json.loads(archive.read("io_outputs.json"))
    start_state = None
    if "start_state.json" in names:
        if not any(e.get("path") == "start_state.json" for e in manifest.get("contents", [])):
            raise PlanRequestError("measured start is not bound by the request manifest")
        from .measured_start import validate as validate_start
        try:
            start_state = validate_start(json.loads(archive.read("start_state.json")), cell, program, io_outputs)
        except ValueError as exc:
            raise PlanRequestError(str(exc)) from exc
    return {
        "manifest": manifest,
        "program": program,
        # Read out of the program rather than a member of its own; kept as a
        # key because every consumer reaches for it by name.
        "placement": program.get("placement"),
        "cell": cell,
        "tooling": tooling,
        # None when the request predates fixtures or says nothing about them.
        "fixtures": json.loads(archive.read("fixtures.json")) if "fixtures.json" in names else None,
        "context": context,
        "start_state": start_state,
        # The connected cell's outputs (`io_schedule.SCHEMA`), when the
        # program switches any; None otherwise.
        "io_outputs": io_outputs,
        "step_bytes": None if manifest["source"].get("kind") == "none" else archive.read(manifest["source"]["path"]),
    }


def self_test() -> int:
    program = program_v2._minimal()
    placed = program["placement"]
    cell = {
        "id": "c",
        "frames": {"base": "world", "work": "table"},
        "links": [{"name": "table", "visuals": [{"mesh": "meshes/t.dae"}]}],
        "joints": [],
    }

    assert problems(program, cell) == []
    assert "no cell" in " ".join(problems(program, None))
    assert "mean something else" in " ".join(
        problems({**program, "placement": {**placed, "cell_id": "other"}}, cell)
    )
    assert "different things" in " ".join(
        problems({**program, "workpiece": {"frame_id": "world"}}, cell)
    )
    assert "is attached" in " ".join(
        problems({**program, "robot": {"model_id": "other", "robot_description_sha256": _UNPINNED_ROBOT}}, cell)
    )
    # The program's own rules travel with it, prefixed so a reader knows which
    # half refused: a document the schema would take can still be unplannable.
    assert "program: nodes is empty" in " ".join(problems({**program, "nodes": []}, cell))

    pack = dict(
        request_id="r",
        program=program,
        cell=cell,
        step_bytes=b"ISO-10303-21;",
        step_filename="part.step",
    )
    data = write_request(**pack)
    assert data == write_request(**pack), "the bytes are not deterministic"

    back = read_request(data)
    # A descriptor with no identity cannot resolve the pin; it passes through.
    assert back["program"]["robot"]["robot_description_sha256"] == _UNPINNED_ROBOT
    assert back["program"] == program
    assert back["placement"] == placed

    # With an identity on the descriptor, the placeholder is resolved to it,
    # a program pinned to it is consistent, and one pinned elsewhere is stale.
    identity = "sha256:" + "a" * 64
    stamped = {**cell, "robot_cell": {"model_id": "c", "robot_description_sha256": identity,
                                      "machine_planning_calibration_sha256": "sha256:" + "b" * 64, "machine_planning_calibration_source": "empty"}}
    resolved = read_request(write_request(**{**pack, "cell": stamped}))
    assert resolved["program"]["robot"]["robot_description_sha256"] == identity
    pinned = {**program, "robot": {"model_id": "c", "robot_description_sha256": identity}}
    read_request(write_request(**{**pack, "cell": stamped, "program": pinned}))
    try:
        write_request(**{**pack, "cell": stamped,
                         "program": {**program, "robot": {"model_id": "c", "robot_description_sha256": "sha256:" + "e" * 64}}})
    except PlanRequestError as exc:
        assert "review the changes in Workspace" in str(exc), exc
    else:
        raise AssertionError("a program pinned to another description was packed")

    # A descriptor swapped for another robot's after packing, digests
    # rewritten to match: the program's pin still says the part would be
    # placed against a work frame it was not measured from.
    other = io.BytesIO()
    signed = write_request(**{**pack, "cell": stamped, "program": pinned})
    packed_zip = zipfile.ZipFile(io.BytesIO(signed))
    manifest = json.loads(packed_zip.read("manifest.json"))
    swapped = _canonical(_without_meshes({**stamped, "robot_cell": {
        **stamped["robot_cell"], "robot_description_sha256": "sha256:" + "f" * 64}}))
    for entry in manifest["contents"]:
        if entry["path"] == "cell.json":
            entry["sha256"], entry["bytes"] = _sha256(swapped), len(swapped)
    with zipfile.ZipFile(other, "w") as out:
        for item in packed_zip.infolist():
            payload = packed_zip.read(item.filename)
            if item.filename == "cell.json":
                payload = swapped
            if item.filename == "manifest.json":
                payload = _canonical(manifest)
            entry = zipfile.ZipInfo(item.filename, date_time=_FIXED_DATE)
            entry.compress_type = item.compress_type
            out.writestr(entry, payload)
    try:
        read_request(other.getvalue())
    except PlanRequestError as exc:
        assert "the request packs" in str(exc), exc
    else:
        raise AssertionError("a swapped robot was accepted")
    assert back["cell"]["links"][0]["visuals"] == [], "the meshes should have been dropped"
    assert back["step_bytes"] == b"ISO-10303-21;"
    assert back["context"] is None, "a request packed without a context has none"

    # The planning context travels verbatim, member by member, and is verified
    # like everything else: a member edited after packing is refused.
    context = {"robot.urdf": b"<robot name='c'/>", "planner.json": b"{}\n",
               "meshes/t.dae": b"<COLLADA/>"}
    with_context = write_request(**pack, context=context)
    assert with_context == write_request(**pack, context=context)
    assert read_request(with_context)["context"] == context
    for bad in ("../x", "/x", "a//b", "a/./b"):
        try:
            write_request(**pack, context={bad: b""})
        except PlanRequestError:
            pass
        else:
            raise AssertionError(f"context member {bad!r} was packed")

    # Tamper with a MEMBER and keep the manifest, which is what the digests
    # are for. Flipping a byte at random tests zlib's CRC instead -- it fails
    # for a reason this module had no part in, and passes when the flip lands
    # somewhere the zip does not read.
    swapped = io.BytesIO()
    original = zipfile.ZipFile(io.BytesIO(data))
    with zipfile.ZipFile(swapped, "w") as out:
        for item in original.infolist():
            payload = original.read(item.filename)
            if item.filename == "program.json":
                payload = _canonical({**program, "program_id": "not-what-was-signed"})
            entry = zipfile.ZipInfo(item.filename, date_time=_FIXED_DATE)
            entry.compress_type = item.compress_type
            out.writestr(entry, payload)
    try:
        read_request(swapped.getvalue())
    except PlanRequestError as exc:
        assert "digest" in str(exc), exc
    else:
        raise AssertionError("a request whose program had been swapped was accepted")

    print("plan request self-test ok: refuses what it cannot resolve, round trips, deterministic")
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--read", default="", help=f"a {EXTENSION} to describe")
    ns = parser.parse_args(argv)
    if ns.self_test:
        return self_test()
    if not ns.read:
        raise PlanRequestError("--read or --self-test")
    opened = read_request(Path(ns.read).read_bytes())
    print(json.dumps(opened["manifest"], indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main(sys.argv[1:]))
    except Exception as exc:  # noqa: BLE001 - CLI worker prints concise failures.
        print(f"planrequest: {exc}", file=sys.stderr)
        raise SystemExit(1)
