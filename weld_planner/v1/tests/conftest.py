from pathlib import Path

import pytest

# No `sys.path` surgery. `python/` is on PYTHONPATH for the whole environment
# -- pixi sets it in `[activation.env]` -- so `seam_worker` and `weldplan`
# import here exactly as they do under `pixi run serve` and under any of the
# module CLIs. A conftest that patched the path would have made the test suite
# the one context where imports resolve differently from production, which is
# the opposite of what a test environment is for.
from seam_worker import paths

V1 = paths.V1
REPO = paths.REPO_ROOT

DOG_BOX = paths.DOG_BOX
PIPE_WELD = paths.PIPE_BRACKET


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return REPO


@pytest.fixture(scope="session")
def schema_dir() -> Path:
    return paths.SCHEMA_DIR


@pytest.fixture(scope="session")
def program_contracts_dir() -> Path:
    """`robot/v4/contracts/programs`, where `robot.v4.program.v2` lives. The
    one place the path is counted; skipped, not failed, on a checkout without
    the robot module."""
    if not paths.PROGRAM_V2_SCHEMA.exists():
        pytest.skip(f"program contracts not present: {paths.PROGRAM_CONTRACTS_DIR}")
    return paths.PROGRAM_CONTRACTS_DIR


@pytest.fixture(scope="session")
def pipe_bracket_step() -> Path:
    if not PIPE_WELD.exists():
        pytest.skip(f"fixture STEP not present: {PIPE_WELD}")
    return PIPE_WELD


@pytest.fixture(scope="session")
def pipe_bracket_topology(pipe_bracket_step):
    """Column on a base plate with two gussets.

    Adversarial in a way dog_box is not: every joint is a tee rather than a
    lap, and every joint has a stretch of weld line pressed against a third
    part, so a torch cannot reach it. dog_box has no occlusion anywhere.
    """
    from seam_worker.cad.topology import extract_topology

    return extract_topology(pipe_bracket_step)


@pytest.fixture(scope="session")
def dog_box_topology():
    """Extracted once per session. STEP import plus B-spline conversion is the
    slow part; every topology test reads the same document."""
    if not DOG_BOX.exists():
        pytest.skip(f"fixture STEP not present: {DOG_BOX}")
    from seam_worker.cad.topology import extract_topology

    return extract_topology(DOG_BOX)
