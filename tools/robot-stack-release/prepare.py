#!/usr/bin/env python3
"""Prepare an offline, resumable install packet; never contact or restart a device."""
import argparse
import copy
import json
from pathlib import Path, PurePosixPath

import bundle


class RetiredBackendError(ValueError):
    """Refuse a retired owner before opening packages or creating install state."""


def validate_selection(selection):
    """Reject retired owners before unrelated inputs or install state are opened."""
    if any(name in selection for name in ("robot-v4-rtcore", "robot-v4-rtcore-nats-bridge", "robot-v4-pair-owner")):
        raise RetiredBackendError("reason=backend_retired; use the published rollback tarball through its historical procedure")


def prepare(spec, stack, manifest_path, fixture_path, selection, references, output):
    validate_selection(selection)
    bundle.verify(spec, stack)  # No output exists before every bundle byte is checked.
    manifest_raw = manifest_path.read_bytes()
    manifest = json.loads(manifest_raw)
    original = copy.deepcopy(manifest)
    fixture = json.loads(fixture_path.read_bytes())
    selected = {c["name"]: c for c in spec["components"] if c["name"] in selection}
    if not selected or set(selected) != set(selection):
        raise ValueError("select explicit known release components")
    if set(references) != {"cell", "home"}:
        raise ValueError("cell and home reference files are required")
    reference_hashes = {name: {"path": str(path.resolve()), "sha256": bundle.digest(path.read_bytes())}
                        for name, path in references.items()}
    intents, artifacts, slots = [], {}, []
    for name, component in selected.items():
        choice = selection[name]
        target = PurePosixPath(choice["path"])
        if not target.is_absolute() or ".." in target.parts or target.name != name or str(target) != choice["path"]:
            raise ValueError("target slot must be an absolute normalized component path")
        nodes = [n for n in manifest["nodes"] if n["node"] == choice["target_role"]]
        templates = [i for i in fixture["intents"] if i.get("target_role") == choice["target_role"] and i["process"]["name"] == name]
        if len(nodes) != 1 or len(templates) != 1:
            raise ValueError("component must resolve to one manifest node and fixture intent")
        node = nodes[0]
        old = [p for p in node["runtime_paths"] if PurePosixPath(p).name == name]
        if len(old) != 1 or old[0] == str(target):
            raise ValueError("select a new versioned slot; preserve the installed path")
        node["runtime_paths"] = [str(target) if p == old[0] else p for p in node["runtime_paths"]]
        intent = copy.deepcopy(templates[0])
        intent["process"]["path"] = str(target)
        intent["process"]["sha256"] = "sha256:" + component["sha256"]
        # Keep existing restart semantics; this packet never executes them.
        intents.append(intent)
        artifacts[f"artifacts/{name}"] = bundle.read(stack, f"artifacts/{name}")
        slots.append({"component": name, "target_role": choice["target_role"], "peer": node["peer"],
                      "previous_path": old[0], "path": str(target), "sha256": component["sha256"]})
    fixture["intents"] = intents
    output = output.absolute()
    argv = ["go", "-C", str(Path(__file__).resolve().parents[2] / "steamdeck/real/v4"), "run", ".", "process-intents", "publish", "--manifest", str(output / "manifest.json"),
            "--fixture", str(output / "intents.json")]
    plan = {"schema": "rosie.robot-stack.install-preparation.v1", "status": "prepared_only",
            "stack_manifest_sha256": bundle.digest(bundle.read(stack, "manifest.json")),
            "external_references": reference_hashes, "slots": slots,
            "dry_run_argv": argv + ["--dry-run"], "publish_argv": argv,
            "remaining": ["owner stages artifacts to named peers/slots and verifies target loader and hashes",
                          "owner reviews canonical CLI resolved intents, including policy/config transformations",
                          "owner publishes using existing daemon write authentication",
                          "owner verifies loaded executable hashes, disarmed state and reference readback"],
            "rollback": "Saved manifest and original fixture preserve prior selection; refresh ownership, restore saved intents through canonical CLI, and verify loaded hashes/references. No daemon slot rollback is presumed."}
    files = {**artifacts, "manifest.json": bundle.encode(manifest), "intents.json": bundle.encode(fixture),
             "previous-manifest.json": manifest_raw, "previous-intents.json": fixture_path.read_bytes(),
             "plan.json": bundle.encode(plan)}
    # Assert configuration preservation independently of the mutation loop.
    restored = copy.deepcopy(manifest)
    for before, after in zip(original["nodes"], restored["nodes"], strict=True):
        after["runtime_paths"] = before.get("runtime_paths", [])
        if "runtime_paths" not in before:
            after.pop("runtime_paths")
    if restored != original:
        raise ValueError("non-runtime manifest configuration changed")
    files["SHA256SUMS"] = "".join(f"{bundle.digest(raw)}  {name}\n" for name, raw in sorted(files.items())).encode()
    # Validate all existing state before resuming any missing file. Corruption is retained.
    if output.exists():
        if output.is_symlink() or not output.is_dir():
            raise ValueError("invalid preparation directory")
        for path in output.rglob("*"):
            name = path.relative_to(output).as_posix()
            if path.is_symlink() or (path.is_dir() and name in files) or (not path.is_dir() and (name not in files or bundle.read(output, name) != files[name])):
                raise ValueError("existing preparation differs; retain it and select a new output")
    output.mkdir(mode=0o700, parents=False, exist_ok=True)
    for name, raw in files.items():
        path = output / name
        if not path.exists():
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("xb") as stream:
                stream.write(raw)
        path.chmod(0o755 if name.startswith("artifacts/") else 0o600)
    for name, raw in files.items():
        if bundle.read(output, name) != raw:
            raise ValueError("preparation readback mismatch")
    return plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["spec", "bundle", "manifest", "fixture", "selection", "cell", "home", "output"]:
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    try:
        selection = json.loads(args.selection.read_bytes())
        validate_selection(selection)
        prepare(json.loads(args.spec.read_bytes()), args.bundle, args.manifest, args.fixture,
                selection, {"cell": args.cell, "home": args.home}, args.output)
    except (ValueError, OSError, KeyError, TypeError) as error:
        parser.exit(1, f"rejected: {error}\n")
    print("passed: local preparation only; remote staging, publication and loaded-state verification not_evaluated")


if __name__ == "__main__":
    main()
