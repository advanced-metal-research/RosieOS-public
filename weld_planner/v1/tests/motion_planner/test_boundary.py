"""The arrow points one way, and here is where that stops being prose.

`tests/shared/test_plan_request.py` holds the other half: `weldplan` imports the
standard library and nothing else. This half is about the consumer --
`weld_motion_planner` may take on torch, warp, OCCT and whatever else it needs, but
it may never import `seam_worker`.

The reason is not tidiness. The authoring side needs OCCT, numba and
cadquery; the planner needs CUDA. Letting either reach into the other would
mean a machine that plans has to install a CAD kernel, and a machine that
authors has to install a GPU stack -- and the first person to hit that
installs both, at which point the seam the whole design rests on is gone.

Checked by reading the source rather than `sys.modules`: an import inside a
function has not run at test time, and that is exactly the kind that gets
added in a hurry because it is "only for one thing".
"""

from __future__ import annotations

import ast
from pathlib import Path

V1 = Path(__file__).resolve().parents[2]
PACKAGE = V1 / "python" / "weld_motion_planner"
TESTS = Path(__file__).resolve().parent

FORBIDDEN = {"seam_worker"}


def _imports(source: Path) -> list[tuple[int, str]]:
    found: list[tuple[int, str]] = []
    for node in ast.walk(ast.parse(source.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            names = [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            # `level > 0` is a relative import: within this package by
            # definition, and never the authoring side.
            names = [node.module or ""] if not node.level else []
        else:
            continue
        found.extend((node.lineno, name) for name in names)
    return found


def test_the_planner_never_imports_the_authoring_side():
    offenders = [
        f"{source.relative_to(V1)}:{line} imports {name}"
        for source in sorted(PACKAGE.rglob("*.py"))
        for line, name in _imports(source)
        if name.split(".")[0] in FORBIDDEN
    ]
    assert not offenders, (
        "weld_motion_planner must stand alone; where it needs what the authoring side computes, "
        "it implements it and checks itself against an exported oracle: " + "; ".join(offenders)
    )


def test_the_planners_tests_do_not_import_it_either():
    """The loophole that would make the rule true and useless.

    A test suite that imports `seam_worker` to build an expected value still
    forces OCCT onto the planner's machine to run its own tests -- and worse,
    it is the place where "just import it to compare" is most tempting. The
    oracle pattern exists precisely so this stays unnecessary.
    """
    offenders = [
        f"{source.relative_to(V1)}:{line} imports {name}"
        for source in sorted(TESTS.rglob("*.py"))
        for line, name in _imports(source)
        if name.split(".")[0] in FORBIDDEN
    ]
    assert not offenders, "; ".join(offenders)


def test_the_package_is_reachable_and_names_its_own_rule():
    """The docstring is load-bearing documentation: it is the first thing
    anyone opening this package reads, and it states the boundary."""
    import weld_motion_planner

    assert "seam_worker" in (weld_motion_planner.__doc__ or ""), "the package should state its own rule"
