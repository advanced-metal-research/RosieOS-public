"""`detect_joints`, as the OLP calls it: joints, accessibility, then seams.

The authoring server's job ran accessibility between detection and the default
seams. When the server was deleted (c4cd1f1) the worker operation that replaced
it went straight from detection to seams, so every seam was a whole
corner-to-corner loop with nothing split where no torch reaches -- and the
motion planner could not cross the operator's seams on bracket_b_2x, which it
crossed when they were authored on the annotated pieces.
"""

from __future__ import annotations

import base64

import pytest

from seam_worker import paths
from seam_worker.workers import run

BRACKET_B = paths.FIXTURES_DIR / "bracket_b_2x.step"


@pytest.fixture(scope="module")
def detected():
    if not BRACKET_B.exists():
        pytest.skip(f"missing fixture {BRACKET_B}")
    request = {
        "operation": "detect_joints",
        "step_base64": base64.b64encode(BRACKET_B.read_bytes()).decode(),
    }
    return run(request)["joints"]


def test_every_joint_carries_accessibility(detected):
    missing = [j["id"] for j in detected["weld_joints"] if "accessibility" not in j]
    assert missing == [], missing


def test_a_loop_with_unreachable_arc_is_split_where_the_torch_stops(detected):
    """weld_joint_0002 is a closed tee loop. Annotated, it is five pieces and the
    last ends because the torch cannot go further; unannotated it was four
    corner-to-corner pieces, all "continuity"."""
    pieces = [s for s in detected["seams"] if s["weld_joint_ref"] == "weld_joint_0002"]
    assert len(pieces) == 5, [s["label"] for s in pieces]
    assert pieces[-1]["origin"]["split_reason"] == "reachability"
