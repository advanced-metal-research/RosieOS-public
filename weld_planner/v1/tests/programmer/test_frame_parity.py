"""The client's angle math must equal the server's.

`WEBUI.md` section 8 accepts one deliberate duplication: the two angle
rotations are implemented twice, in `python/seam_worker/frames.py` and in
`ui/src/frame-math.ts`, so that dragging a work- or travel-angle slider does
not round-trip to the server. Everything subtle about the frame -- the
orthogonalisation, the rail correspondence, the per-segment matching at
corners -- stays server-side precisely because duplicating *that* would be
reckless.

But an undefended duplication is how a sign convention drifts apart between two
languages, and the symptom is a torch pointing into the workpiece three stages
downstream. This is the test that makes the duplication safe.

It runs the real TypeScript, via node's type stripping, rather than a Python
transcription of it -- a transcription would just be a third implementation to
disagree with.
"""

import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest
from seam_worker import paths
from seam_worker.seam.frames import apply_angles

UI = paths.V1 / "ui"
SCRIPT = UI / "scripts" / "frame-parity.mjs"


def _node() -> str | None:
    for candidate in (
        shutil.which("node"),
        str(Path.home() / ".rosie" / "node-22.13.1" / "bin" / "node"),
    ):
        if candidate and Path(candidate).exists():
            return candidate
    return None


@pytest.fixture(scope="module")
def client_cases():
    node = _node()
    if node is None:
        pytest.skip("node not available")
    if not SCRIPT.exists():
        pytest.skip(f"missing {SCRIPT}")
    result = subprocess.run(
        [node, "--experimental-strip-types", str(SCRIPT)],
        capture_output=True,
        text=True,
        cwd=UI,
        timeout=120,
        check=False,
        env={**os.environ, "NODE_NO_WARNINGS": "1"},
    )
    if result.returncode != 0:
        pytest.fail(f"frame-parity.mjs failed:\n{result.stderr[-2000:]}")
    return json.loads(result.stdout)


def test_client_and_server_agree_on_every_case(client_cases):
    assert len(client_cases) >= 100, "expected a real grid, not a token case"
    worst = 0.0
    for case in client_cases:
        theirs = np.array(case["u"], dtype=float)
        tangent = np.array(case["t"], dtype=float)
        # The server negates in `SeamGeometry.frame`; the client in
        # `travelTangent`. One line each, in two languages, so it is exactly the
        # kind of thing that drifts -- and a client drawing the comb one way
        # while the server measured clearance the other would put a torch on
        # screen that is not the torch that was checked.
        if case.get("reverse"):
            tangent = -tangent
        mine = apply_angles(
            tangent,
            np.array(case["r"], dtype=float),
            case["work"],
            case["travel"],
        )
        worst = max(worst, float(np.linalg.norm(mine - theirs)))
    assert worst < 1e-12, f"TypeScript and Python disagree by {worst:.3e}"


def test_the_grid_covers_both_travel_directions(client_cases):
    assert any(case.get("reverse") for case in client_cases)
    assert any(not case.get("reverse") for case in client_cases)
    # And reversing has to actually change the answer, or the parity above
    # would be comparing a flag nobody honoured against a flag nobody honoured.
    by_key = {
        (tuple(c["t"]), tuple(c["r"]), c["work"], c["travel"], c["reverse"]): c["u"]
        for c in client_cases
    }
    moved = sum(
        1
        for (t, r, w, tv, rev), u in by_key.items()
        if not rev
        and (w, tv) != (0.0, 0.0)
        and float(np.linalg.norm(np.array(u) - np.array(by_key[(t, r, w, tv, True)]))) > 1e-9
    )
    assert moved > 0, "reversing changed nothing anywhere"


def test_the_grid_actually_exercises_both_signs(client_cases):
    """A grid of zeros would pass the parity test and prove nothing."""
    works = {case["work"] for case in client_cases}
    travels = {case["travel"] for case in client_cases}
    assert any(w < 0 for w in works) and any(w > 0 for w in works)
    assert any(t < 0 for t in travels) and any(t > 0 for t in travels)
    # and non-axis-aligned frames, so nothing passes on a convenient basis
    assert any(
        sum(1 for c in case["t"] if abs(c) > 1e-9) > 1 for case in client_cases
    ), "expected oblique tangents in the grid"


def test_drag_sign_survives_the_round_trip(client_cases):
    """DECISIONS.md D4 is still open, so the convention is pinned on BOTH sides
    rather than only in the Python.

    Against the TRAVEL tangent, which is the point: drag means leaning back
    over completed weld, and which way that is depends on which way you are
    walking. On a reversed seam `u . t` is negative and the weld is still being
    dragged -- so this assertion is also what says a reversed seam does not
    quietly become a push.
    """
    for case in client_cases:
        if case["travel"] <= 0:
            continue
        t = np.array(case["t"], dtype=float)
        if case.get("reverse"):
            t = -t
        assert float(np.dot(np.array(case["u"], dtype=float), t)) > 0, (
            "positive travel must be drag in the client too"
        )
