"""Fixtures for the motion planner's tests.

Separate from `tests/conftest.py` on purpose: that one imports
`seam_worker.paths`, and a planner test that reached for it would be
importing the authoring side by the back door. Everything here resolves from
this file's own position, and the only package it touches is `weld_motion_planner`.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from .committed import FULL_TREE, SKIP_REASON

V1 = Path(__file__).resolve().parents[2]
MOTION_FIXTURES = V1 / "data" / "motion"


def _plan_requests() -> list[Path]:
    return sorted(MOTION_FIXTURES.glob("*.weldplan"))


def pytest_generate_tests(metafunc):
    """Parametrize over every committed plan request.

    Generated rather than named, so adding a fixture is dropping a file in --
    a list of names here would be a second place to remember, and the one
    that gets forgotten.
    """
    if "motion_fixture_path" not in metafunc.fixturenames:
        return
    found = _plan_requests()
    if not found and not FULL_TREE:
        metafunc.parametrize("motion_fixture_path", [pytest.param(None, marks=pytest.mark.skip(reason=SKIP_REASON))])
        return
    if not found:
        raise pytest.UsageError(
            f"no fixture plan requests in {MOTION_FIXTURES}; run tools/make_motion_fixtures.py"
        )
    metafunc.parametrize("motion_fixture_path", found, ids=lambda p: p.stem)


@pytest.fixture
def plan_request(motion_fixture_path):
    from weld_motion_planner.io.plan_request import read

    request = read(motion_fixture_path)
    # A corpus that carries no welds is worse than no corpus: the tests
    # parametrized over it run, pass, and prove nothing about a weld. That is
    # how the whole committed corpus sat at `weld-program.v1` -- zero seams,
    # zero freespace nodes -- while the planner refused every real weld for
    # authoring no standoff, and the suite stayed green through all of it.
    if not request.seams:
        raise AssertionError(
            f"{motion_fixture_path.name} carries no seams "
            f"(program schema {(request.program or {}).get('schema')!r}). "
            "Rebuild it: tools/make_motion_fixtures.py"
        )
    return request
