"""The planning context, as the planner reads it.

`docs/PLANNING-CONTEXT.md`: the OLP compiles the robot once and the planner
reads what arrived -- profile, spheres, torch, URDF and meshes -- rather than
resolving a description on its own disk by model name. These pin that a
request carrying a context is served from it, that the profile it yields is
the one the local derivation yielded for the same numbers, and that a request
from before the context still reads the local description.

The context here is built the way the OLP builds it (`internal/robotconfig`
`Compile`): the real `rosie_1400_v3` files for the members that travel
verbatim, and a `planner.json` in the shape the Go compiler writes. The
compiler's own merge rules are tested on its side; this side tests the reader.
"""

from __future__ import annotations

import copy
import hashlib
import json
import zipfile
from pathlib import Path

import pytest
from weld_motion_planner.robot_cell import context as contextmod
from weld_motion_planner.robot_cell.robot.cell import CellError
from weld_motion_planner.robot_cell.robot.profile import for_cell, load
from weld_motion_planner.robot_cell.robot_cell import RobotCell

from .committed import committed_plan

ROOT = Path(__file__).resolve().parents[2]
FIXTURE = ROOT / "data" / "motion" / "hitch_2x.weldplan"
DESCRIPTION = ROOT.parents[1] / "robot_description" / "robots" / "rosie_1400_v3"
MODEL = "rosie_1400_v3"

IDENTITY = {"model_id": MODEL, "robot_description_sha256": "sha256:" + "1" * 64,
            "machine_planning_calibration_sha256": "sha256:" + "2" * 64, "machine_planning_calibration_source": "target"}


@pytest.fixture()
def descriptor() -> dict:
    with zipfile.ZipFile(committed_plan(FIXTURE)) as archive:
        cell = json.loads(archive.read("cell.json"))
    cell["robot_cell"] = {**IDENTITY, "settings": {}}
    return cell


def planner_document(**overrides) -> dict:
    """`planner.json` as the Go compiler writes it for the model alone: the
    description's own numbers, the enforced rate the planning rate."""
    config = json.loads((DESCRIPTION / "config.json").read_text())
    limits = {
        axis: {"velocity_rad_s": spec["velocity_rad_s"],
               "enforced_velocity_rad_s": spec["velocity_rad_s"],
               "acceleration_rad_s2": spec["acceleration_rad_s2"]}
        for axis, spec in config["planning"]["joints"].items()
    }
    document = {
        "schema": contextmod.SCHEMA, "model_id": MODEL, "identity": IDENTITY,
        "bounds": {}, "limits": limits, "driven": config["axes"]["driven"],
        "held": config["axes"]["held"], "reset_pose": config["reset_pose"],
        "frames": config["frames"], "torch": config["torch"],
    }
    document.update(overrides)
    return document


def members(planner: dict | None = None) -> dict[str, bytes]:
    files = {
        "robot.urdf": (DESCRIPTION / "robot.urdf").read_bytes(),
        "spheres.json": (DESCRIPTION / "spheres.json").read_bytes(),
        "planner.json": json.dumps(planner or planner_document()).encode(),
        "identity.json": json.dumps(IDENTITY).encode(),
    }
    for mesh in sorted((DESCRIPTION / "meshes").glob("*.dae")):
        files[f"meshes/{mesh.name}"] = mesh.read_bytes()
    return files


@pytest.fixture(autouse=True)
def clean_registry():
    contextmod._REGISTRY.clear()
    yield
    for found in list(contextmod._REGISTRY.values()):
        found.close()
    contextmod._REGISTRY.clear()


def test_a_context_yields_the_profile_the_local_derivation_did(descriptor):
    """Same numbers in, same profile out: the reader maps `planner.json` onto
    the profile every stage already takes, so a plan made from a context and
    one made from the local description agree on rates, holds and the reset
    pose."""
    contextmod.from_members(members())
    from_context = for_cell(descriptor)
    local = load(MODEL)
    assert from_context.driven == local.driven
    assert from_context.held == local.held
    assert from_context.limits == local.limits
    assert from_context.reset_pose == local.reset_pose


def test_a_request_from_before_the_context_is_refused(descriptor):
    """There is no description on this machine to fall back to by name --
    that lookup was the second derivation the context ended -- so every
    reader says so, in words, naming the fix."""
    import torch
    from weld_motion_planner.robot_cell.robot_cell import RobotCellError

    assert contextmod.of(descriptor) is None
    with pytest.raises(CellError, match="planning context"):
        for_cell(descriptor)
    with pytest.raises(RobotCellError, match="planning context"):
        RobotCell.of(descriptor, device=torch.device("cpu"))
    with pytest.raises(contextmod.ContextError, match="planning context"):
        contextmod.mesh_root_for(descriptor)


def test_the_context_is_found_by_identity_not_by_reference(descriptor):
    contextmod.from_members(members())
    other = copy.deepcopy(descriptor)
    assert contextmod.of(other) is contextmod.of(descriptor)
    stranger = copy.deepcopy(descriptor)
    stranger["robot_cell"]["machine_planning_calibration_sha256"] = "sha256:" + "3" * 64
    assert contextmod.of(stranger) is None


def test_the_cell_takes_its_spheres_torch_and_urdf_from_the_context(descriptor):
    """What `RobotCell.of` used to look up by model name on disk comes from
    what arrived; the URDF cuRobo reads is the context's copy, beside the
    meshes."""
    import torch

    found = contextmod.from_members(members())
    cell = RobotCell.of(descriptor, device=torch.device("cpu"))
    assert cell.context is found
    assert cell.arm.spheres is not None and cell.arm.spheres.path is None
    assert cell.arm.spheres.count > 50
    assert cell.arm.tool.electrode_axis == (1.0, 0.0, 0.0) and cell.arm.tool.path is None

    urdf = found.urdf_path()
    assert urdf.is_file()
    assert hashlib.sha256(urdf.read_bytes()).hexdigest() == \
        hashlib.sha256((DESCRIPTION / "robot.urdf").read_bytes()).hexdigest()
    assert (found.mesh_root() / "link_1.dae").is_file()
    assert contextmod.mesh_root_for(descriptor) == found.mesh_root()
    root = found.materialize()
    found.close()
    assert not root.exists(), "a closed context left its files behind"


def test_a_held_arm_joint_is_refused_against_the_descriptor(descriptor):
    contextmod.from_members(members(planner_document(held={"J2": 0.0}, driven=["J1"])))
    with pytest.raises(CellError, match="only the positioner may be held"):
        for_cell(descriptor)


def test_a_held_positioner_leaves_the_driven_set(descriptor):
    contextmod.from_members(members(planner_document(
        held={"J8": 0.0, "J9": 0.0, "J7": 0.25}, driven=["J1", "J2", "J3", "J4", "J5", "J6"])))
    profile = for_cell(descriptor)
    assert profile.held["J7"] == 0.25 and "J7" not in profile.driven


@pytest.mark.parametrize(("broken", "says"), [
    ({"planner.json": b"{}"}, "schema"),
    ({"identity.json": b'{"model_id":"x"}'}, "identity.json"),
    ({"identity.json": json.dumps({**IDENTITY, "machine_planning_calibration_sha256": "sha256:" + "9" * 64}).encode()},
     "different pairs"),
])
def test_a_context_that_does_not_name_its_pair_is_refused(broken, says):
    files = {**members(), **broken}
    with pytest.raises(contextmod.ContextError, match=says):
        contextmod.PlanningContext(files)


def test_the_registry_is_bounded():
    for i in range(contextmod._KEEP + 2):
        identity = {**IDENTITY, "machine_planning_calibration_sha256": f"sha256:{i:064d}"}
        files = members(planner_document(identity=identity))
        files["identity.json"] = json.dumps(identity).encode()
        contextmod.from_members(files)
    assert len(contextmod._REGISTRY) == contextmod._KEEP


def test_native_envelopes_do_not_alias_and_planner_reads_narrowed_j6():
    import torch
    import xml.etree.ElementTree as ET
    from weldplan.urdf import descriptor_of

    contexts, descriptors = [], []
    for index, lower in enumerate((-2.094395102, -1.5)):
        digest = "sha256:" + str(index + 3) * 64
        planner = planner_document(execution_limits_sha256=digest)
        planner["limits"]["J6"] = {"velocity_rad_s": 0.12,
                                      "enforced_velocity_rad_s": 0.2,
                                      "acceleration_rad_s2": 0.5}
        files = members(planner)
        files["identity.json"] = json.dumps({**IDENTITY, "execution_limits_sha256": digest}).encode()
        urdf = ET.fromstring(files["robot.urdf"])
        limits = urdf.find("./joint[@name='J6']/limit")
        assert limits is not None
        limits.set("lower", str(lower))
        limits.set("upper", "2.094395102")
        files["robot.urdf"] = ET.tostring(urdf)
        contexts.append(contextmod.from_members(files))
        descriptors.append(descriptor_of(files))

    assert len(contextmod._REGISTRY) == 2
    for index, descriptor in enumerate(descriptors):
        assert contextmod.of(descriptor) is contexts[index]
        assert descriptor["robot_cell"]["execution_limits_sha256"] == contexts[index].identity["execution_limits_sha256"]
        cell = RobotCell.of(descriptor, device=torch.device("cpu"))
        joint = cell.arm.chain.axis_names.index("J6")
        lower, upper = cell.arm.joint_limits
        assert float(lower[joint]) == pytest.approx((-2.094395102, -1.5)[index])
        assert float(upper[joint]) == pytest.approx(2.094395102)
        assert float(lower[joint]) > -2.094446759543233
        profile = for_cell(descriptor)
        assert profile.limits["J6"]["acceleration_rad_s2"] == 0.5
        assert profile.limits["J6"]["velocity_rad_s"] == 0.12


def test_native_envelope_key_preserves_offline_compatibility():
    expected = "|".join(IDENTITY[key] for key in contextmod._IDENTITY_KEYS)
    assert contextmod.key_of(IDENTITY) == expected
    assert contextmod.key_of({**IDENTITY, "execution_limits_sha256": None}) == expected
    assert contextmod.key_of({**IDENTITY, "machine_configuration_sha256": "unrelated"}) == expected


@pytest.mark.parametrize("digest", ["bad", "sha256:" + "x" * 64, None])
def test_native_envelope_identity_must_be_valid_and_match_planner(digest):
    files = members(planner_document(execution_limits_sha256="sha256:" + "3" * 64))
    files["identity.json"] = json.dumps({**IDENTITY, "execution_limits_sha256": digest}).encode()
    with pytest.raises(contextmod.ContextError, match="execution.limit"):
        contextmod.PlanningContext(files)


def test_the_fixture_requests_carry_their_context():
    """The committed corpus is packed with the context the OLP compiles
    (`tools/refresh_corpus_cells.py`), and its descriptor was parsed from that
    context's URDF: the two name the same pair, and the joints agree."""
    from weld_motion_planner.io.plan_request import read
    from weldplan.urdf import descriptor_of

    request = read(committed_plan(FIXTURE))
    assert request.context is not None
    assert request.cell["robot_cell"] == request.context.identity
    parsed = descriptor_of(request.context.files)
    assert [j["xyz_m"] for j in parsed["joints"]] == [j["xyz_m"] for j in request.cell["joints"]]
    assert [j["limits_rad"] for j in parsed["joints"]] == [j["limits_rad"] for j in request.cell["joints"]]
