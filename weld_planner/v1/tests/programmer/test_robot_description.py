"""The packer's half of the planning context.

The robot arrives COMPILED: the OLP read the description and the target's
`machine_planning_calibration.json`, judged the deviation and composed it onto the URDF
(docs/PLANNING-CONTEXT.md). What is pinned here is what the packer does with
that -- parses the descriptor off the context's own URDF, stamps the identity
the dense header carries, refuses a context that does not name its pair --
and what it no longer does: apply anything.

The URDF is the REAL `rosie_1420_v1`, so the descriptor read is the one a
plan would be made against.
"""

from __future__ import annotations

import hashlib
import json
import math

import pytest
from seam_worker import paths
from seam_worker.workers import WorkerError, run
from weldplan import robot_description as rd
from weldplan.urdf import CellError, descriptor_of, read_urdf

DESCRIPTION = paths.REPO_ROOT / "robot_description" / "robots" / "rosie_1420_v1"
URDF = DESCRIPTION / "robot.urdf"
# Any description identity: what it names is the daemon's business, not this
# module's, which only checks the SHAPE before stamping it.
IDENTITY = "sha256:" + "01234567" * 8
CELL = "sha256:" + "89abcdef" * 8


@pytest.fixture(scope="module")
def model() -> dict:
    return read_urdf(URDF)


def identity(**over) -> dict:
    return {"model_id": "rosie_1420_v1", "robot_description_sha256": IDENTITY,
            "machine_planning_calibration_sha256": CELL, "machine_planning_calibration_source": "target", **over}


def context(**over) -> dict[str, bytes]:
    """A context's three required members for the 1420, as the OLP writes
    them: the URDF, the numbers with the frames, the identity."""
    config = json.loads((DESCRIPTION / "config.json").read_text())
    members = {
        "robot.urdf": URDF.read_bytes(),
        "planner.json": json.dumps({"schema": "rosie.planning-context.v1",
                                    "frames": config["frames"]}).encode(),
        "identity.json": json.dumps(identity()).encode(),
    }
    members.update(over)
    return members


def joint(descriptor: dict, name: str) -> dict:
    return next(j for j in descriptor["joints"] if j["name"] == name)


def test_the_descriptor_knows_the_design_rate(model):
    # The rating is the drive cap through the gear ratio (P8-44): 3000 motor
    # rpm is 100 pi rad/s at the motor, over 100:1 on J1 and the 1420's J5,
    # over 18:1 on J4, over 10:1 on J6. What the planner plans at is
    # config.json's, far below, and a context's URDF carries that rate
    # instead -- the descriptor reads whichever file it is given and restates
    # nothing.
    assert joint(model, "J1")["velocity_rad_s"] == pytest.approx(math.pi)
    assert joint(model, "J4")["velocity_rad_s"] == pytest.approx(100 * math.pi / 18)
    assert joint(model, "J5")["velocity_rad_s"] == pytest.approx(math.pi)
    assert joint(model, "J6")["velocity_rad_s"] == pytest.approx(10 * math.pi)
    assert "velocity_rad_s" not in joint(model, "pedestal_to_base")


def test_the_empty_document_is_the_daemons_bytes():
    """A planner that could not reach the target plans against these bytes, so
    they must be the ones a cell with no file serves and hashes."""
    assert rd.empty_cell_bytes("rosie_1420_v1") == (
        b'{"schema":"rosie.machine-planning-calibration.v1","model_id":"rosie_1420_v1"}\n')
    assert rd.machine_planning_calibration_sha256(b"x") == "sha256:" + hashlib.sha256(b"x").hexdigest()


def test_the_descriptor_is_read_off_the_context_and_stamped(model):
    """The context's URDF, the context's frames, the context's identity: the
    packer combines nothing of its own."""
    cell = descriptor_of(context())
    assert cell["robot_cell"] == identity()
    assert rd.header_identity(cell) == identity()
    assert rd.header_identity(model) is None
    assert cell["id"] == "rosie_1420_v1"
    assert cell["source"] == {"filename": "robot.urdf"}
    assert cell["frames"]["work"], "the frames come from planner.json"
    assert [j["name"] for j in cell["joints"] if j["type"] == "revolute"] == \
        [j["name"] for j in model["joints"] if j["type"] == "revolute"]
    assert "settings" not in cell["robot_cell"], "the numbers live in the context, not the stamp"
    json.dumps(cell)


def test_the_machine_configuration_is_recorded_when_the_context_names_one():
    """A plan made for a machine records that machine's whole configuration
    beside the pair it is fenced on; a plan made offline records none, and the
    header block is the same four keys it always was."""
    machine = "sha256:" + "9" * 64
    recorded = identity(machine_configuration_sha256=machine)
    cell = descriptor_of(context(**{"identity.json": json.dumps(recorded).encode()}))
    assert cell["robot_cell"] == recorded
    assert rd.header_identity(cell)["machine_configuration_sha256"] == machine
    offline = descriptor_of(context())
    assert "machine_configuration_sha256" not in offline["robot_cell"]
    assert "machine_configuration_sha256" not in rd.header_identity(offline)


def test_the_corrected_urdf_is_what_the_descriptor_reads():
    """A cell composed onto the URDF by the OLP arrives on the descriptor's
    joints without this side applying anything: what the file says, the
    descriptor says."""
    text = URDF.read_text()
    before = joint(read_urdf(URDF), "J4")["xyz_m"]
    at = text.index('name="J4"')
    start = text.index('<origin xyz="', at) + len('<origin xyz="')
    end = text.index('"', start)
    shifted = text[:start] + f"{before[0] + 0.001!r} {before[1]!r} {before[2]!r}" + text[end:]
    cell = descriptor_of(context(**{"robot.urdf": shifted.encode()}))
    assert joint(cell, "J4")["xyz_m"] == pytest.approx([before[0] + 0.001, before[1], before[2]])
    assert joint(cell, "J3")["xyz_m"] == joint(read_urdf(URDF), "J3")["xyz_m"]


@pytest.mark.parametrize(("members", "says"), [
    ({"identity.json": b"{}"}, "identity.json.model_id"),
    ({"identity.json": json.dumps(identity(model_id="rosie_1400_v3")).encode()}, "identity says"),
    ({"planner.json": b"nope"}, "not JSON"),
])
def test_a_context_that_does_not_name_its_pair_is_refused(members, says):
    with pytest.raises(CellError, match=says):
        descriptor_of(context(**members))


def test_a_context_missing_a_member_is_refused():
    members = context()
    del members["planner.json"]
    with pytest.raises(CellError, match="planner.json"):
        descriptor_of(members)


def test_the_packer_takes_no_robot_but_the_context():
    """No URDF, no settings, no frames of its own: a request that still sends
    them is from before the context, and is refused rather than combined."""
    with pytest.raises(WorkerError, match="planning_context"):
        run({"operation": "build_plan_request", "step_base64": "SVNPLTEwMzAzLTIxOw==",
             "program": {}, "cell_urdf_base64": "", "cell_settings": {}})
