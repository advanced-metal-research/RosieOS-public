"""Reading a URDF as data.

Two kinds of test, and the second is the one that found the real bug.

Synthetic fixtures pin the rules -- which joint types are modelled, what a
package:// path reduces to, that an unsupported joint is reported rather than
dropped. They are fast and they are written by the same person as the parser,
which is exactly their weakness.

The bundle test reads the ACTUAL `rosie_1400_v3` URDF from
`offline-programming/v1`, as data, the way `test_pipe_bracket.py` reads a real
STEP. It is skipped when that module is not checked out, because a sibling
module's absence is not this module's failure.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from seam_worker import paths
from seam_worker.robot.cell import (
    SCHEMA,
    CellError,
    load_cells,
    problems,
    read_urdf,
    summarise,
)

BUNDLE = paths.REPO_ROOT / "offline-programming" / "v1" / "ui" / ".staged-public" / "robot"

MINIMAL = """<?xml version="1.0"?>
<robot name="tiny">
  <material name="grey"><color rgba="0.2 0.3 0.4 1"/></material>
  <link name="world"/>
  <link name="arm">
    <visual>
      <origin xyz="1 2 3" rpy="0 0 1.5"/>
      <geometry><mesh filename="package://tiny/meshes/arm.dae"/></geometry>
      <material name="grey"/>
    </visual>
  </link>
  <joint name="J1" type="revolute">
    <parent link="world"/><child link="arm"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0.5 0"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3"/>
  </joint>
</robot>
"""


def _write(tmp_path: Path, text: str, name: str = "robot.urdf") -> Path:
    path = tmp_path / name
    path.write_text(text, encoding="utf-8")
    return path


# --------------------------------------------------------------- the parser


def test_a_urdf_reduces_to_links_joints_and_frames(tmp_path):
    cell = read_urdf(_write(tmp_path, MINIMAL))
    assert cell["schema"] == SCHEMA
    assert cell["id"] == "tiny"
    assert [link["name"] for link in cell["links"]] == ["world", "arm"]
    assert cell["axes"] == ["J1"]

    joint = cell["joints"][0]
    assert joint["parent"] == "world" and joint["child"] == "arm"
    assert joint["xyz_m"] == [0.1, 0.2, 0.3]
    assert joint["rpy_rad"] == [0.0, 0.5, 0.0]
    assert joint["axis_xyz"] == [0.0, 1.0, 0.0]
    assert joint["limits_rad"] == {"lower": -3.0, "upper": 3.0}


def test_a_parent_element_with_no_children_is_still_a_parent(tmp_path):
    """The bug this file exists for.

    An ElementTree element with no children is FALSY, so
    `node.find("parent") or fallback` silently discards `<parent link="x"/>`.
    Every joint in the real bundle vanished and the parser reported twelve
    links "not connected to the base" while looking like it had read the file.
    """
    cell = read_urdf(_write(tmp_path, MINIMAL))
    assert len(cell["joints"]) == 1, "the joint was dropped"
    assert not bool(ET.fromstring('<parent link="world"/>')), (
        "if this ever becomes truthy the guard above is no longer needed"
    )


def test_the_package_prefix_is_stripped_to_a_bundle_relative_path(tmp_path):
    """`package://` is a ROS resource locator and means nothing to a browser.
    What survives is the path relative to the bundle, which is what the asset
    mount serves -- so a descriptor stays valid however it is hosted."""
    cell = read_urdf(_write(tmp_path, MINIMAL))
    visual = cell["links"][1]["visuals"][0]
    assert visual["mesh"] == "meshes/arm.dae"
    assert visual["xyz_m"] == [1.0, 2.0, 3.0]
    assert visual["rgba"] == [0.2, 0.3, 0.4, 1.0]


def test_an_unsupported_joint_is_reported_and_its_limb_named(tmp_path):
    """A link quietly missing from a cell is a collision nobody sees coming,
    so it is named twice: once for the joint, once for what it disconnects."""
    text = MINIMAL.replace("</robot>", """
      <link name="slider"/>
      <joint name="J9" type="prismatic">
        <parent link="world"/><child link="slider"/>
        <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="1 0 0"/>
      </joint>
    </robot>""")
    cell = read_urdf(_write(tmp_path, text))
    found = problems(cell)
    assert any("prismatic" in p for p in found), found
    assert any("'slider' is not connected" in p for p in found), found
    assert cell["axes"] == ["J1"], "an unmodelled joint is not an axis"


def test_a_non_mesh_visual_is_reported_rather_than_drawn_as_nothing(tmp_path):
    text = MINIMAL.replace(
        '<geometry><mesh filename="package://tiny/meshes/arm.dae"/></geometry>',
        '<geometry><box size="1 1 1"/></geometry>',
    )
    cell = read_urdf(_write(tmp_path, text))
    assert any("non-mesh" in p for p in cell["problems"]), cell["problems"]
    assert cell["links"][1]["visuals"] == []


def test_a_cell_with_no_role_mapping_says_so(tmp_path):
    """`positioner_table_a` is a fact about one Fusion export; "the frame a
    workpiece is fixtured to" is a fact about welding. An unknown cell gets no
    work frame rather than a guess from a link name."""
    cell = read_urdf(_write(tmp_path, MINIMAL))
    assert cell["frames"] == {}
    assert any("no work frame" in p for p in cell["problems"])


def test_a_file_that_is_not_a_urdf_is_refused(tmp_path):
    with pytest.raises(CellError, match="not parseable XML"):
        read_urdf(_write(tmp_path, "<robot"))
    with pytest.raises(CellError, match="expected <robot>"):
        read_urdf(_write(tmp_path, "<?xml version='1.0'?><scene/>"))


def test_a_disconnected_link_is_reported(tmp_path):
    text = MINIMAL.replace("</robot>", '<link name="orphan"/></robot>')
    cell = read_urdf(_write(tmp_path, text))
    assert any("'orphan' is not connected" in p for p in problems(cell))


def test_an_absent_bundle_directory_is_not_an_error(tmp_path):
    """A sibling module that is not checked out means placement has nothing to
    offer, which the UI says -- rather than inventing a robot."""
    assert load_cells(tmp_path / "nope") == []


def test_one_unreadable_bundle_does_not_hide_the_others(tmp_path):
    (tmp_path / "good").mkdir()
    _write(tmp_path / "good", MINIMAL)
    (tmp_path / "bad").mkdir()
    _write(tmp_path / "bad", "<robot")
    cells = load_cells(tmp_path)
    # The id comes from `<robot name>`, and only falls back to the directory
    # when the file cannot be parsed at all -- which is the "bad" one here.
    assert [c["id"] for c in cells] == ["bad", "tiny"]
    assert cells[0]["problems"] and not cells[0]["links"]
    assert cells[1]["links"]


# ------------------------------------------------------------- the real cell


@pytest.mark.skipif(not BUNDLE.is_dir(), reason="offline-programming/v1 is not checked out")
def test_the_shipped_cell_reads_without_problems():
    """The real `rosie_1400_v3`, read as DATA -- no import from that module,
    the same way a real STEP is read as a fixture.

    Pinned by shape rather than by number where possible, but the axis list is
    pinned exactly: those nine names are what a placement record and a plan
    request will refer to, and one of them changing is a contract change.
    """
    cells = {c["id"]: c for c in load_cells(BUNDLE)}
    cell = cells["rosie_1400_v3"]
    assert cell["problems"] == [], cell["problems"]
    assert cell["axes"] == ["J1", "J2", "J3", "J4", "J5", "J6", "J9", "J7", "J8"]
    assert cell["frames"] == {
        "base": "world",
        "tool": "tool0",
        "work": "positioner_table_a_top",
    }
    # The work frame is the middle of the table's top face, where the OLP
    # measures a placement from -- not the table link's origin on J7's axis.
    top = next(j for j in cell["joints"] if j["child"] == "positioner_table_a_top")
    assert (top["type"], top["parent"]) == ("fixed", "positioner_table_a")
    assert top["xyz_m"] == [0.0, -0.622, 0.1]
    # Every link is reachable and every mesh is a real file in the bundle. The
    # URDF's thirteen plus the top-face marker, which has no mesh.
    assert len(cell["links"]) == 14
    for link in cell["links"]:
        for visual in link["visuals"]:
            assert (BUNDLE / cell["id"] / visual["mesh"]).exists(), visual["mesh"]

    # The arm's base offset, straight out of the URDF. It is also the planner
    # origin the sibling module uses, so a wrong sign here would put every
    # authored pose in the wrong place.
    j1 = next(j for j in cell["joints"] if j["name"] == "J1")
    assert j1["xyz_m"] == [0.7874, 0.6858, 0.4808375]


@pytest.mark.skipif(not BUNDLE.is_dir(), reason="offline-programming/v1 is not checked out")
def test_a_cell_without_a_positioner_fixtures_to_the_floor():
    """`rosie_1420_v1` is an arm on a pedestal with no table in the model. That
    is not a missing work frame -- it is a cell where the workpiece does not
    move, so the work frame is the base."""
    cells = {c["id"]: c for c in load_cells(BUNDLE)}
    cell = cells["rosie_1420_v1"]
    assert cell["problems"] == [], cell["problems"]
    assert cell["frames"]["work"] == "world"
    assert not any(link["name"].startswith("positioner") for link in cell["links"])


def test_a_floor_cell_carries_the_fixture_its_urdf_does_not():
    """The part stands on something, and on a cell with no positioner that
    something is not in the robot's own model. Without it the planner believes
    there is clear air under the workpiece and will route the forearm through
    whatever is really holding it -- measured on the placeholder box, the arm
    enters it in 19 of 300 random postures.

    It is a descriptor body and NOT a link in `robot.urdf`: that file is
    embedded byte for byte in the RTCore nats bridge and its sha256 is pinned
    in the pair config, so a link added there stops the cell arming.
    """
    cells = {c["id"]: c for c in load_cells(BUNDLE)}
    cell = cells["rosie_1420_v1"]
    assert cell["problems"] == [], cell["problems"]
    fixtures = [link for link in cell["links"] if link["name"] == "fixture_placeholder"]
    assert len(fixtures) == 1
    joint = next(j for j in cell["joints"] if j["child"] == "fixture_placeholder")
    assert joint["type"] == "fixed" and joint["parent"] == "world"
    assert joint["xyz_m"] == [1.0, 1.0, 0.0]
    # It is furniture, not an axis: nothing may drive it.
    assert "fixture_placeholder" not in cell["axes"]

    # And the cell that has a real table gets none of this.
    assert not any(link["name"].startswith("fixture")
                   for link in cells["rosie_1400_v3"]["links"])


@pytest.mark.skipif(not BUNDLE.is_dir(), reason="offline-programming/v1 is not checked out")
def test_the_summary_is_one_line_an_operator_can_act_on():
    cell = {c["id"]: c for c in load_cells(BUNDLE)}["rosie_1400_v3"]
    line = summarise(cell)
    assert "14 links" in line and "9 axes" in line and "positioner_table_a_top" in line
    assert "\n" not in line


def test_self_test_entry_point_passes():
    from seam_worker.robot.cell import self_test

    assert self_test() == 0
