"""Every filesystem anchor this module reads, in one place.

**Why one module rather than a `parents[N]` in each caller.** These constants
used to be computed where they were used -- twenty-odd copies of
`Path(__file__).resolve().parents[1] / "fixtures"`, plus six identical
`repo_root()` helpers counting three levels up. That is fine until the tree
moves, and then it is the worst possible failure: `parents[3]` from a file one
level deeper resolves to a real directory that simply does not hold what was
asked for, so nothing raises. `sample_steps` filters on `exists()` and the
Examples list silently loses an entry; `CELL_ROOT` misses and Placement offers
no cells and calls that normal. A wrong path here does not crash, it edits what
the operator is shown. Counting depth in one file means a move breaks one file,
and breaks it loudly.

**The externals carry env overrides, the internals do not.** `CELL_ROOT` and
`DOG_BOX` reach out of this module into sibling modules of the repository, and
`REPO_ROOT` is how they find it. Those are the paths that a different checkout
layout, a container, or a test fixture legitimately needs to redirect, so each
reads an environment variable first. Everything under `V1` is this module's own
furniture and moves only when this file moves, so an override would be a knob
with nothing on the other end of it.

Both external paths are read as DATA. That is not the code dependency on
`cadquery/v1` or `robot_description` that GOAL.md forbids -- no module here imports from
either -- and absence is not an error in either case: the sample list is short
by one and placement has no cell to offer, both of which say so.
"""

from __future__ import annotations

import os
from pathlib import Path


def _env_path(name: str, default: Path) -> Path:
    """An override if the environment names one, the default otherwise.

    Empty is treated as unset. `FOO=` in a shell script or a compose file is
    how a variable gets cleared, and reading that as `Path("")` -- which is
    `Path(".")`, the current working directory -- would point the resolver at
    wherever the process happened to start.
    """
    value = os.environ.get(name, "").strip()
    return Path(value).expanduser().resolve() if value else default


# This file is `<v1>/python/seam_worker/paths.py`, so `parents[2]` is `<v1>`.
# The one depth count left in the module, and the reason the rest are gone.
V1 = Path(__file__).resolve().parents[2]

# `<repo>`: the repository root this component sits in. Only the
# two external constants below use it.
REPO_ROOT = _env_path("WELD_REPO_ROOT", V1.parents[1])

# ------------------------------------------------------------------ internal

SCHEMA_DIR = V1 / "schema"

DATA_DIR = V1 / "data"
FIXTURES_DIR = DATA_DIR / "fixtures"



# Runtime state: uploaded steps, artifacts, programs, projects. Gitignored, and
# overridable because a second server on the same checkout must not share it.

# The fixture every module's `--self-test` reaches for. A column on a base
# plate with two gussets: every joint is a tee, and every joint has a stretch
# pressed against a third part that no torch can reach.
PIPE_BRACKET = FIXTURES_DIR / "pipe_bracket.step"

# ------------------------------------------------------------------ external

# Work cells, read from the repository's robot descriptions.
#
# `robot_description/` and NOT `offline-programming/v1/ui/.staged-public`: both
# hold the same bundles, but the two under `offline-programming` are gitignored
# BUILD OUTPUT and would simply be absent on a fresh checkout.
CELL_ROOT = _env_path("WELD_CELL_ROOT", REPO_ROOT / "robot_description" / "robots")

# The one bundled sample that does not live in `data/fixtures/`, so the one
# that still has to be named.
DOG_BOX = _env_path("WELD_DOG_BOX", REPO_ROOT / "cadquery" / "v1" / "step" / "dog_box.step")

# The shared Robot V4 program contracts, read from the repository's deploy
# module. `robot.v4.program.v2` vendors this module's seam vocabulary, and
# `tests/shared/test_program_v2.py` holds the copy equal to the source. A data
# read, like CELL_ROOT: `weldplan` must not import this module to find it,
# and absence means the parity test skips and says so.
PROGRAM_CONTRACTS_DIR = _env_path(
    "WELD_PROGRAM_CONTRACTS_DIR", REPO_ROOT / "steamdeck" / "real" / "v4" / "contracts" / "programs"
)
PROGRAM_V2_SCHEMA = PROGRAM_CONTRACTS_DIR / "robot.v4.program.v2.schema.json"


def repo_root() -> Path:
    """Kept as a function because the CLIs call it that way."""
    return REPO_ROOT
