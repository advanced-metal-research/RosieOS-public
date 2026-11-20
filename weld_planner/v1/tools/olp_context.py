"""The planning context, compiled by the compiler that compiles it.

`docs/PLANNING-CONTEXT.md`: the robot a plan is for is derived from the
description and the cell's `machine_planning_calibration.json` exactly once, by the OLP
(`offline-programming/v1/internal/robotconfig.Compile`). A fixture built here
must carry the same context the OLP would pack, so this runs that compiler --
the `compile-context` command of the OLP binary -- rather than restating the
merge in Python, where it would drift from the first implementation on the
day somebody changed one of them.

The binary is `offline-programming/v1/bin/offline-programming-linux` when the
OLP has been built (`start-offline-programming.sh` does); otherwise `go run`
against the source, with the project's Go under `~/.rosie` on the path.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
OLP = REPO_ROOT / "offline-programming" / "v1"
BINARY = OLP / "bin" / "offline-programming-linux"


def _go() -> str | None:
    found = shutil.which("go")
    if found:
        return found
    for candidate in sorted((Path.home() / ".rosie").glob("go-*/bin/go"), reverse=True):
        if os.access(candidate, os.X_OK):
            return str(candidate)
    return None


def compiled_context(model_id: str, machine_planning_calibration: Path | None = None, *,
                     root: Path | None = None) -> dict[str, bytes]:
    """The context's members by path, as the OLP compiles them for `model_id`
    with `machine_planning_calibration` (or the model's empty document when None)."""
    with tempfile.TemporaryDirectory(prefix="olp-context-") as scratch:
        out = Path(scratch) / "context"
        args = ["compile-context", "--model", model_id, "--out", str(out)]
        if machine_planning_calibration is not None:
            args += ["--machine-planning-calibration", str(machine_planning_calibration)]
        if root is not None:
            args += ["--root", str(root)]
        if BINARY.is_file() and os.access(BINARY, os.X_OK):
            command = [str(BINARY), *args]
        else:
            go = _go()
            if go is None:
                raise SystemExit(
                    f"no OLP binary at {BINARY} and no Go toolchain to build one; run "
                    "offline-programming/v1/start-offline-programming.sh once, or put "
                    "~/.rosie/go-*/bin on PATH")
            command = [go, "run", ".", *args]
        completed = subprocess.run(command, cwd=OLP, capture_output=True, text=True, check=False)
        if completed.returncode != 0:
            raise SystemExit(
                f"compile-context failed ({completed.returncode}):\n{completed.stderr.strip()}")
        members: dict[str, bytes] = {}
        for path in sorted(p for p in out.rglob("*") if p.is_file()):
            members[path.relative_to(out).as_posix()] = path.read_bytes()
        return members
