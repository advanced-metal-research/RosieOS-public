"""The committed plan requests, which only the full tree carries.

The `.weldplan` corpus (`data/motion/*.weldplan`) and the taught 1430
fixture are ZIP archives holding a real cell's controller definition and
meshes and the sample STEP parts, so the public mirror leaves them out
(`tools/public-mirror/private-paths.txt`). That list exists only in the full
tree: there a missing request fails the test that needs it, so a lost fixture
is never mistaken for the mirror; elsewhere the test is skipped.
"""

from __future__ import annotations

from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[4]
FULL_TREE = (REPO / "tools" / "public-mirror" / "private-paths.txt").is_file()
SKIP_REASON = "the public mirror leaves the committed plan requests out (tools/public-mirror/private-paths.txt)"


def committed_plan(path: Path) -> Path:
    """`path` when it exists; fails in the full tree and skips in the mirror when it does not."""
    if path.is_file():
        return path
    if FULL_TREE:
        pytest.fail(f"missing committed plan request {path}")
    pytest.skip(SKIP_REASON)


def committed_plans(directory: Path) -> list[Path]:
    """Every `*.weldplan` in `directory`; none fails in the full tree and skips in the mirror."""
    found = sorted(directory.glob("*.weldplan"))
    if not found and not FULL_TREE:
        pytest.skip(SKIP_REASON)
    return found
