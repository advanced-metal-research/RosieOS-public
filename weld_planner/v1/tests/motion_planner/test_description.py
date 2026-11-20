"""The robot description: `config.json`, `spheres.json` and the merged profile.

Step 2 of `docs/ROBOT-DESCRIPTION-CONTRACT.md` moved what the planner used to
keep beside itself -- `<model>.profile.json`, `<model>.tool.json`,
`<model>.spheres.json` -- into the model's own directory, and made the profile
a COMPUTED merge instead of a fourth file. So the first test here is a golden:
the numbers that come out are the numbers those files had, to the digit. A
refactor that quietly changed a rate limit would be a planner parametrizing
against something nobody measured, which is the failure the deleted files'
notes spend six paragraphs warning about.

The provenance test is the proposal's own failure row: "spheres refit merged
without their URDF" and "mesh replaced, URDF text same" both have to fail here
rather than in a weld.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import sys

import pytest
from weld_motion_planner.robot_cell.collision import spheres as spheremod
from weld_motion_planner.robot_cell.robot import description
from weld_motion_planner.robot_cell.robot.profile import load
from weld_motion_planner.robot_cell.robot.tool import require as require_tool
from weldplan.robot_description import (
    CellDocumentError,
    DescriptionError,
    description_files,
    robot_description_sha256,
    mesh_sha256,
    parse_config,
)

MODELS = ("rosie_1400_v3", "rosie_1420_v1")


def tools_registry():
    """`robot_description/tools/manifest.py`, the writer of the file under
    test. Loaded by path: it is a tool beside the descriptions, not a module of
    this package, and importing it by name would need a second package root."""
    # The tool sits beside the robots/ directory, not inside it.
    source = description.DESCRIPTION_ROOT.parent / "tools" / "manifest.py"
    spec = importlib.util.spec_from_file_location("robot_description_registry", source)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


# What data/motion/cells/<model>.profile.json carried, per axis: the planning
# rate, the acceleration, and the enforced rate a cell with no machine_planning_calibration.json falls
# back to. 0.09 rad/s on J1-J3 is the drives' max profile velocity; 0.2 is
# RTCore's runtime speed limit. See the models' config.json notes.
SLOW = {"velocity_rad_s": 0.09, "acceleration_rad_s2": 2.094395, "enforced_velocity_rad_s": 0.09}
WRIST = {"velocity_rad_s": 0.2, "acceleration_rad_s2": 4.18879, "enforced_velocity_rad_s": 0.2}
POSITIONER = {"velocity_rad_s": 0.2, "acceleration_rad_s2": 1.047198, "enforced_velocity_rad_s": 0.2}
GOLDEN = {
    "rosie_1400_v3": {
        "driven": ["J1", "J2", "J3", "J4", "J5", "J6", "J7"],
        "held": {"J8": 0.0, "J9": 0.0},
        "reset_pose": {f"J{i}": 0.0 for i in range(1, 8)},
        "limits": {"J1": SLOW, "J2": SLOW, "J3": SLOW, "J4": WRIST, "J5": WRIST, "J6": WRIST,
                   "J7": POSITIONER, "J8": POSITIONER, "J9": POSITIONER},
    },
    "rosie_1420_v1": {
        "driven": ["J1", "J2", "J3", "J4", "J5", "J6"],
        "held": {},
        "reset_pose": {f"J{i}": 0.0 for i in range(1, 7)},
        "limits": {"J1": SLOW, "J2": SLOW, "J3": SLOW, "J4": WRIST, "J5": WRIST, "J6": WRIST},
    },
}


def config_document(model: str) -> dict:
    return json.loads(description.config_path(model).read_text(encoding="utf-8"))


@pytest.mark.parametrize("model", MODELS)
def test_the_merged_profile_is_the_deleted_files_numbers(model):
    profile = load(model)
    want = GOLDEN[model]
    assert profile is not None
    assert profile.cell_id == model
    assert profile.driven == want["driven"]
    assert profile.held == want["held"]
    assert profile.reset_pose == want["reset_pose"]
    assert profile.limits == want["limits"]
    assert profile.path == description.config_path(model)
    # Why those numbers are what they are is in the model's README, not in a
    # note the identity would hash.
    assert profile.notes == []


@pytest.mark.parametrize("model", MODELS)
def test_the_torch_comes_from_the_description(model):
    torch = require_tool(model)
    assert torch.cell_id == model
    assert torch.tool_frame == "tool0"
    assert torch.electrode_axis == (1.0, 0.0, 0.0)
    assert torch.path == description.config_path(model)


@pytest.mark.parametrize("model", MODELS)
def test_the_sphere_model_is_beside_the_meshes_it_was_fitted_to(model):
    found = spheremod.load(model)
    assert found is not None and found.cell_id == model
    assert found.path == description.spheres_path(model)
    assert found.count > 50, "the arm lost spheres in the move"
    urdf = description.urdf_path(model).read_bytes()
    assert found.provenance["robot_urdf_sha256"] == hashlib.sha256(urdf).hexdigest()
    assert found.provenance["meshes_sha256"] == mesh_sha256(description.path(model))


@pytest.mark.parametrize("model", MODELS)
def test_a_refit_that_never_happened_fails_here(model, tmp_path):
    """The proposal's failure row: a mesh replaced with the URDF text the same
    is not detectable anywhere else. The sphere model carries the hash of the
    meshes it was fitted to, so editing one without a refit fails."""
    copy = tmp_path / model
    shutil.copytree(description.path(model), copy)
    mesh = sorted((copy / "meshes").glob("*.dae"))[0]
    mesh.write_bytes(mesh.read_bytes() + b"<!-- a vertex moved -->\n")
    found = spheremod.load(model, root=tmp_path)
    assert found is not None
    assert found.provenance["meshes_sha256"] != mesh_sha256(copy)


@pytest.mark.parametrize("model", MODELS)
def test_the_registry_names_every_file_the_description_is_made_of(model):
    """`robot_description_manifest.json` is the identity's whole input, so a file the robot
    needs that nobody registered is a plan fenced on less than the robot."""
    manifest = tools_registry()
    found = manifest.problems(description.path(model))
    assert found == [], "\n".join(found)
    listed = description_files(description.path(model))
    assert listed == manifest.registered(description.path(model))
    for name in ("robot.urdf", "robot.srdf", "config.json", "spheres.json"):
        assert name in listed
    assert any(name.startswith("meshes/") for name in listed)


def test_only_a_registered_file_is_part_of_the_robot(tmp_path):
    """The reason this is a manifest and not a hash of the directory: a CAD
    export or a README beside the URDF is provenance, and a plan made before
    somebody added one is still a plan for this robot."""
    copy = tmp_path / "rosie_1420_v1"
    shutil.copytree(description.path("rosie_1420_v1"), copy)
    before = robot_description_sha256(copy)
    (copy / "NOTES.md").write_text("not part of the robot\n", encoding="utf-8")
    (copy / "fusion_part_export").mkdir(exist_ok=True)
    (copy / "fusion_part_export" / "junk.json").write_text("{}", encoding="utf-8")
    assert robot_description_sha256(copy) == before

    urdf = copy / "robot.urdf"
    urdf.write_bytes(urdf.read_bytes().replace(b'<robot name=', b'<robot  name='))
    assert robot_description_sha256(copy) != before


def test_dropping_a_file_from_the_registry_is_a_different_robot(tmp_path):
    """The manifest is hashed as an entry of its own, so it cannot quietly
    stop covering a file."""
    copy = tmp_path / "rosie_1420_v1"
    shutil.copytree(description.path("rosie_1420_v1"), copy)
    before = robot_description_sha256(copy)
    manifest = json.loads((copy / "robot_description_manifest.json").read_text(encoding="utf-8"))
    manifest["files"] = [f for f in manifest["files"] if f != "robot.srdf"]
    (copy / "robot_description_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    assert robot_description_sha256(copy) != before

    # And a registered file that is not there at all is not an identity.
    (copy / "robot.srdf").unlink()
    manifest["files"].append("robot.srdf")
    (copy / "robot_description_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    with pytest.raises(DescriptionError, match="robot.srdf"):
        robot_description_sha256(copy)


def test_a_description_may_not_register_the_cells_own_file(tmp_path):
    """The cell serves `machine_planning_calibration.json` beside the
    description's files. A manifest that registers a file of that name would
    collide with it in the served set, and the adapter refuses the whole set
    at start; both readers (this one and `robotdesc.DescriptionFiles`) refuse
    the name instead, where the manifest is read."""
    copy = tmp_path / "rosie_1420_v1"
    shutil.copytree(description.path("rosie_1420_v1"), copy)
    (copy / "machine_planning_calibration.json").write_text(
        '{"schema":"rosie.machine-planning-calibration.v1","model_id":"rosie_1420_v1"}\n', encoding="utf-8")
    manifest = json.loads((copy / "robot_description_manifest.json").read_text(encoding="utf-8"))
    manifest["files"].append("machine_planning_calibration.json")
    (copy / "robot_description_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    with pytest.raises(DescriptionError, match="the cell's own file"):
        description_files(copy)


@pytest.mark.parametrize("model", MODELS)
def test_a_planning_rate_may_not_exceed_the_urdfs_rating(model):
    velocities = description.urdf_velocities(model)
    assert velocities and set(velocities) >= set(GOLDEN[model]["limits"])
    document = config_document(model)
    fast = json.loads(json.dumps(document))
    fast["planning"]["joints"]["J1"]["velocity_rad_s"] = velocities["J1"] * 1.5
    with pytest.raises(CellDocumentError, match="above the URDF's rating"):
        parse_config(fast, model_id=model, velocities=velocities)


def test_the_ceiling_caps_every_joint():
    model = "rosie_1400_v3"
    velocities = description.urdf_velocities(model)
    document = config_document(model)
    lowered = json.loads(json.dumps(document))
    lowered["planning"]["speed_ceiling_rad_s"] = 0.05
    config = parse_config(lowered, model_id=model, velocities=velocities)
    assert {spec["velocity_rad_s"] for spec in config.planning.values()} == {0.05}
    raised = json.loads(json.dumps(document))
    # Above whatever the fastest joint is rated for, not a number: the rating
    # is the drive cap through the gear ratio (P8-44) and moves with either.
    raised["planning"]["speed_ceiling_rad_s"] = max(velocities.values()) * 1.5
    with pytest.raises(CellDocumentError, match="above the URDF's fastest joint"):
        parse_config(raised, model_id=model, velocities=velocities)


@pytest.mark.parametrize(("mangle", "says"), [
    (lambda d: d.update(extra=1), "unknown key 'extra'"),
    (lambda d: d.update(schema="rosie.robot-config.v0"), "schema"),
    (lambda d: d.update(model_id="another"), "for model 'another'"),
    (lambda d: d.pop("planning"), "no planning section"),
    (lambda d: d.pop("axes"), "no axes section"),
    (lambda d: d.pop("torch"), "no torch section"),
    (lambda d: d.pop("kinematic_correction_caps"), "no kinematic_correction_caps"),
    (lambda d: d["planning"]["joints"].update(J8={"velocity_rad_s": 0.1, "acceleration_rad_s2": 1.0}),
     "no revolute joint 'J8'"),
    (lambda d: d["planning"]["joints"]["J1"].update(velocity_deg_s=5.157), "unknown key"),
    (lambda d: d["planning"]["joints"]["J1"].update(acceleration_rad_s2=0), "must be positive"),
    (lambda d: d["axes"]["driven"].append("J1"), None),  # a repeat is harmless
    (lambda d: d["axes"]["held"].update(J1=0.0), "both driven and held"),
    (lambda d: d["axes"]["driven"].append("J9"), "no revolute joint 'J9'"),
    (lambda d: d["reset_pose"].update(J9=0.0), "no revolute joint 'J9'"),
    (lambda d: d["torch"].update(electrode_axis=[0.0, 0.0, 0.0]), "no direction"),
    (lambda d: d["torch"].update(tool_frame=""), "must name a link"),
    (lambda d: d["kinematic_correction_caps"].update(xyz_m=-1), "must not be negative"),
])
def test_config_refusals(mangle, says):
    model = "rosie_1420_v1"
    document = config_document(model)
    mangle(document)
    velocities = description.urdf_velocities(model)
    if says is None:
        parse_config(document, model_id=model, velocities=velocities)
        return
    with pytest.raises(CellDocumentError, match=says):
        parse_config(document, model_id=model, velocities=velocities)


def test_a_model_without_a_description_is_a_state_not_an_error(tmp_path):
    assert description.config("no_such_model") is None
    assert load("no_such_model") is None
    assert spheremod.load("no_such_model") is None
    assert description.urdf_velocities("no_such_model") == {}
    # And an invalid one is an error, named with the file it is in.
    (tmp_path / "m").mkdir()
    (tmp_path / "m" / "config.json").write_text('{"schema": "wrong"}')
    with pytest.raises(CellDocumentError, match="config.json"):
        description.config("m", tmp_path)
