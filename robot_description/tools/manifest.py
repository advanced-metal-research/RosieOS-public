#!/usr/bin/env python3
"""Write or check a robot description's manifest, and print its identity.

`robot_description_manifest.json` lists the files the robot IS: the URDF and SRDF, the
meshes, `config.json`, `spheres.json`, and `rtcore_definition.json` (what
rt-core needs of the robot: gearing, encoders, drive profiles). A bench
"robot" -- motors on a table, no geometry -- has only the last of these.
Everything else in the directory --
CAD exports, the scripts that made them, measurement reports, a README -- is
provenance, and deliberately outside the identity a plan is fenced on
(`docs/ROBOT-DESCRIPTION-CONTRACT.md`).

    python3 robot_description/tools/manifest.py --check          # every model
    python3 robot_description/tools/manifest.py rosie_1420_v1    # rewrite one

`--check` is what CI and the tests run: it fails when a registered file is
gone, and when a mesh the URDF references is not registered. It does NOT fail
on an unregistered extra file, because that is the point of the manifest.

Registering a file changes the description's identity, so every plan made
against the old one is refused. That is correct -- the robot changed -- and it
is why this is a deliberate command rather than a glob evaluated at load.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "robots"
SCHEMA = "rosie.robot-manifest.v1"
# The files a description is made of, beside the meshes its URDF names.
ALWAYS = ("robot.urdf", "robot.srdf", "config.json", "spheres.json", "rtcore_definition.json")
MESH_REFERENCE = re.compile(r'filename="package://[^"]*?/meshes/([^"/]+)"')


def is_model(d: Path) -> bool:
    """A description directory: a robot with geometry, or a bench robot that
    is only its rt-core definition."""
    return d.is_dir() and ((d / "robot.urdf").is_file() or (d / "rtcore_definition.json").is_file())


def models(root: Path) -> list[Path]:
    return sorted(d for d in root.iterdir() if is_model(d))


def registered(model: Path) -> list[str]:
    """What the model's files should be: the fixed set plus every mesh in
    `meshes/`. The whole folder, not only what the URDF references: the
    planner's own fixture bodies (`fixture_placeholder.dae`) are meshes the
    cell has and the URDF does not name."""
    files = [name for name in ALWAYS if (model / name).is_file()]
    meshes = model / "meshes"
    if meshes.is_dir():
        files += [f"meshes/{p.name}" for p in meshes.iterdir() if p.is_file()]
    return sorted(files)


def document(model: Path) -> dict:
    return {"schema": SCHEMA, "model_id": model.name, "files": registered(model)}


def problems(model: Path) -> list[str]:
    """What is wrong with this model's manifest, as sentences."""
    path = model / "robot_description_manifest.json"
    out: list[str] = []
    if not path.is_file():
        return [f"{path} is missing; run this tool without --check to write it"]
    try:
        current = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        return [f"{path} is not JSON: {error}"]
    want = document(model)
    if current.get("schema") != SCHEMA:
        out.append(f"{path}: schema {current.get('schema')!r} is not {SCHEMA!r}")
    if current.get("model_id") != model.name:
        out.append(f"{path}: model_id {current.get('model_id')!r} is not {model.name!r}")
    listed = list(current.get("files") or [])
    for name in listed:
        if not (model / name).is_file():
            out.append(f"{path} registers {name!r}, which is not a file here")
    for name in want["files"]:
        if name not in listed:
            out.append(f"{path} does not register {name!r}, which this description needs")
    # A mesh the URDF names must be in the folder at all.
    if (model / "robot.urdf").is_file():
        for mesh in sorted(set(MESH_REFERENCE.findall((model / "robot.urdf").read_text(encoding="utf-8")))):
            if not (model / "meshes" / mesh).is_file():
                out.append(f"{model / 'robot.urdf'} references meshes/{mesh}, which is not here")
    return out


def write(model: Path) -> None:
    (model / "robot_description_manifest.json").write_text(
        json.dumps(document(model), indent=2) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("model", nargs="*", help="model directory name; default every model")
    parser.add_argument("--root", default=str(ROOT), help="where the descriptions live")
    parser.add_argument("--check", action="store_true", help="report instead of writing")
    args = parser.parse_args(argv)
    root = Path(args.root)
    chosen = [root / name for name in args.model] if args.model else models(root)
    failures = 0
    for model in chosen:
        if not is_model(model):
            print(f"manifest: {model} is not a robot description", file=sys.stderr)
            failures += 1
            continue
        found = problems(model)
        if args.check:
            for line in found:
                print(f"manifest: {line}", file=sys.stderr)
            failures += len(found)
            if not found:
                print(f"manifest: {model.name} ok, {len(registered(model))} files")
            continue
        write(model)
        sys.path.insert(0, str(root.parents[1] / "weld_planner" / "v1" / "python"))
        from weldplan.robot_description import robot_description_sha256  # noqa: PLC0415 -- a CLI, not a library
        print(f"manifest: {model.name} {len(registered(model))} files {robot_description_sha256(model)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
