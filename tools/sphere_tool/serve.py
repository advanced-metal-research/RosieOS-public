#!/usr/bin/env python3
"""The routes the sphere tool reads a cell through.

`spheres.html` was served by the weld planner's authoring server, and D24
deleted that server (`c4cd1f1`). What the page actually needed from it was
small and had nothing to do with authoring: a list of cells, one cell
descriptor, and the `.dae` files that descriptor names. All three come out of
`robot_description/<cell>/robot.urdf`, so they are answered here from the bundles
directly rather than by standing a FastAPI application back up.

    GET /api/cells                   {"cells": [{"id": ...}, ...]}
    GET /api/cells/<id>              the amr-weld-planner-v1.cell.v1 descriptor
    GET /cell-assets/<id>/<path>     a file from robot_description/<id>/ (meshes, spheres.json)
    GET /cell-urdf/<id>              robot_description/<id>/robot.urdf, verbatim

Read-only, and deliberately: the asset this tool authors leaves through the
browser's own download, into a diff somebody reads. A server that could write
`robot_description/<id>/spheres.json` would make a collision model change with no commit
behind it.

The committed sphere model is part of the robot's description now, so it comes
back from `/cell-assets/<id>/spheres.json` like the meshes beside it; there is
no separate `/cell-data` route and Vite needs no middleware for one.

    python3 serve.py [port] [cell-to-open-first]

Then http://127.0.0.1:8795/. With `npm run dev` instead, this still
has to be running: Vite proxies `/api` and `/cell-assets` here.
"""

from __future__ import annotations

import json
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
# The robot descriptions this tool authors, under robot_description/.
URDF = ROOT / "robot_description" / "robots"
DIST = HERE / "dist"


def _read_urdf():
    """`read_urdf`, loaded from weld_planner by FILE rather than by import.

    The parser lives over there and there is no reason for a second one, but
    its package has two names depending on which branch you are on: D24 renames
    `weld_programmer` to `seam_worker`, so an `import` of either is wrong half
    the time and fails at startup with a ModuleNotFoundError that says nothing
    about branches. Loading the module file directly sidesteps the name, and it
    is safe to do because `cell.py` imports nothing but the standard library --
    check that before widening this.

    It is also the smaller reach. GOAL.md forbids importing another component's
    CODE; this takes one function out of one file the way `cell.py` itself
    reads a URDF, and the tool degrades to a clear message if it moves.
    """
    import importlib.util

    python = ROOT / "weld_planner" / "v1" / "python"
    for package in ("seam_worker", "weld_programmer"):
        source = python / package / "robot" / "cell.py"
        if not source.is_file():
            continue
        spec = importlib.util.spec_from_file_location(f"_cell_{package}", source)
        assert spec and spec.loader
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.read_urdf
    raise SystemExit(
        "cannot find the URDF parser: looked for robot/cell.py under "
        f"{python}/seam_worker and {python}/weld_programmer")


read_urdf = _read_urdf()

# A directory under robot_description/robots with a `robot.urdf` in it is a cell. The page
# opens the FIRST entry, so the one named on the command line leads.
def cells(first: str | None) -> list[str]:
    found = sorted(path.parent.name for path in URDF.glob("*/robot.urdf"))
    return sorted(found, key=lambda name: (name != first, name))


class Handler(SimpleHTTPRequestHandler):
    known: list[str] = []

    def _send(self, body: bytes, kind: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", kind)
        self.send_header("Content-Length", str(len(body)))
        # The descriptor is parsed from a file on disk that a person is
        # editing. A cached one is a URDF fix that does not show up until the
        # browser is convinced to forget it, which costs an hour every time.
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _under(self, root: Path, tail: str) -> Path | None:
        """`tail` resolved inside `root`, or None if it climbs out of it."""
        target = (root / tail).resolve()
        return target if target.is_file() and target.is_relative_to(root.resolve()) else None

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/api/cells":
            return self._send(
                json.dumps({"cells": [{"id": name} for name in self.known]}).encode(),
                "application/json")
        if path.startswith("/api/cells/"):
            urdf = URDF / path[len("/api/cells/"):] / "robot.urdf"
            if not urdf.is_file():
                return self.send_error(404, f"no cell {urdf.parent.name!r} under robot_description/robots")
            return self._send(json.dumps(read_urdf(urdf)).encode(), "application/json")
        if path.startswith("/cell-assets/"):
            cell, _, tail = path[len("/cell-assets/"):].partition("/")
            target = self._under(URDF / cell, tail)
            if target is None:
                return self.send_error(404, path)
            return self._send(
                target.read_bytes(),
                "model/vnd.collada+xml" if target.suffix == ".dae"
                else "application/json" if target.suffix == ".json"
                else "application/octet-stream")
        if path.startswith("/cell-urdf/"):
            # The file itself, not the descriptor. Editing joint limits means
            # rewriting two attributes inside the real URDF and leaving every
            # mesh, material and comment in it untouched, which cannot be done
            # from the parsed form.
            urdf = URDF / path[len("/cell-urdf/"):] / "robot.urdf"
            if not urdf.is_file():
                return self.send_error(404, path)
            return self._send(urdf.read_bytes(), "application/xml")
        super().do_GET()

    def log_message(self, fmt: str, *args: object) -> None:
        sys.stderr.write(f"{fmt % args}\n")


def main(argv: list[str]) -> int:
    port = int(argv[0]) if argv else 8795
    Handler.known = cells(argv[1] if len(argv) > 1 else None)
    if not DIST.is_dir():
        print("no dist/ -- run `npm install && npm run build` first, or use "
              "`npm run dev` and keep this running for /api and /cell-assets",
              file=sys.stderr)
    server = ThreadingHTTPServer(("127.0.0.1", port),
                                 partial(Handler, directory=str(DIST)))
    print(f"cells: {', '.join(Handler.known)}")
    print(f"http://127.0.0.1:{port}/")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
