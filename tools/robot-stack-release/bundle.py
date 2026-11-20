#!/usr/bin/env python3
"""Offline bundle packaging and verification. No deployment operation."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import subprocess
import io
import tarfile
import gzip


def component_table(spec):
    lines = ["# Component pins (generated from manifest release selection)", "",
             "| Component | Version | Proposed tag | Source | Artifact SHA-256 |",
             "|---|---|---|---|---|"]
    for c in spec["components"]:
        lines.append("| " + " | ".join(c.get(k, "unspecified") for k in
                     ["name", "version", "release_tag", "source", "sha256"]) + " |")
    return ("\n".join(lines) + "\n").encode()


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def relative(value):
    p = PurePosixPath(value)
    if not value or value == "." or p.is_absolute() or ".." in p.parts or str(p) != value or any(c in value for c in "\\:\n\r"):
        raise ValueError(f"unsafe relative path: {value}")
    return p


def read(root, name):
    p = root.resolve() / relative(name)
    if any(part.is_symlink() for part in [p, *p.parents]) or not p.is_file():
        raise ValueError(f"missing/nonregular file: {name}")
    return p.read_bytes()


def validate_spec(spec):
    if spec.get("schema") != "rosie.robot-stack.release.v1" or not spec.get("components"):
        raise ValueError("invalid release schema/components")
    names = set()
    for c in spec["components"]:
        if not re.fullmatch(r"[a-z0-9-]+", c["name"]) or c["name"] in names:
            raise ValueError("invalid/duplicate component")
        names.add(c["name"])
        if not re.fullmatch(r"[a-f0-9]{64}", c["sha256"]) or not re.fullmatch(r"[a-f0-9]{40}", c["source"]):
            raise ValueError("invalid artifact/source hash")
        for path in [c["artifact"], *c["source_files"], *c["receipts"]]:
            relative(path)
    for path in spec["qualification_files"]:
        relative(path)
    for path in spec.get("release_files", []):
        relative(path)
    receipt_hashes = spec.get("receipt_sha256", {})
    if not isinstance(receipt_hashes, dict) or not set(receipt_hashes) <= names:
        raise ValueError("invalid receipt hash components")
    for c in spec["components"]:
        pins = receipt_hashes.get(c["name"], {})
        if not isinstance(pins, dict) or not set(pins) <= set(c["receipts"]):
            raise ValueError("receipt hashes must select listed receipts")
        if any(not isinstance(value, str) or not re.fullmatch(r"[a-f0-9]{64}", value) for value in pins.values()):
            raise ValueError("invalid receipt hash")


def encode(value):
    return (json.dumps(value, sort_keys=True, indent=2) + "\n").encode()


def package(spec, inputs, repo, output):
    validate_spec(spec)
    files = {}
    for c in spec["components"]:
        raw = read(inputs, c["artifact"])
        if digest(raw) != c["sha256"]:
            raise ValueError(f"artifact hash mismatch: {c['name']}")
        files[f"artifacts/{c['name']}"] = raw
        for path in c["source_files"]:
            try:
                files[f"provenance/{c['name']}/source/{path}"] = subprocess.check_output(
                    ["git", "-C", str(repo), "show", f"{c['source']}:{path}"], stderr=subprocess.PIPE)
            except subprocess.CalledProcessError as error:
                raise ValueError(f"source unavailable: {c['name']} {c['source']}:{path}; "
                                 "fetch the manifest-selected component tags into --repo "
                                 "(see tools/robot-stack-release/README.md)") from error
        for path in c["receipts"]:
            raw = read(inputs, path)
            expected_hash = spec.get("receipt_sha256", {}).get(c["name"], {}).get(path)
            if expected_hash is not None and digest(raw) != expected_hash:
                raise ValueError(f"receipt hash mismatch: {c['name']} {path}")
            files[f"provenance/{c['name']}/receipts/{path}"] = raw
    for path in spec["qualification_files"]:
        files[f"qualification/{path}"] = read(inputs, path)
    for path in spec.get("release_files", []):
        files[f"release-inputs/{path}"] = read(repo, path)
    files["COMPONENTS.md"] = component_table(spec)
    manifest = {"release": spec, "files": {p: digest(raw) for p, raw in files.items()}}
    files["manifest.json"] = encode(manifest)
    sums = "".join(f"{digest(raw)}  {p}\n" for p, raw in sorted(files.items())).encode()
    # Validate everything before creating output; keep interrupted attempts,
    # and never overwrite a previous bundle.
    output.mkdir(mode=0o700)
    for name, raw in {**files, "SHA256SUMS": sums}.items():
        path = output / relative(name)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
        path.chmod(0o755 if name.startswith("artifacts/") else 0o644)
    verify(spec, output)


def verify(spec, bundle):
    validate_spec(spec)
    manifest = json.loads(read(bundle, "manifest.json"))
    if set(manifest) != {"release", "files"} or manifest["release"] != spec:
        raise ValueError("manifest does not match trusted release specification")
    inventory = manifest["files"]
    if not isinstance(inventory, dict):
        raise ValueError("invalid file inventory")
    expected = {"COMPONENTS.md"}
    if read(bundle, "COMPONENTS.md") != component_table(spec):
        raise ValueError("generated component table mismatch")
    for c in spec["components"]:
        artifact = f"artifacts/{c['name']}"
        expected.add(artifact)
        if inventory.get(artifact) != c["sha256"]:
            raise ValueError("component missing or manifest artifact hash mismatch")
        expected.update(f"provenance/{c['name']}/source/{p}" for p in c["source_files"])
        expected.update(f"provenance/{c['name']}/receipts/{p}" for p in c["receipts"])
        for path, expected_hash in spec.get("receipt_sha256", {}).get(c["name"], {}).items():
            if inventory.get(f"provenance/{c['name']}/receipts/{path}") != expected_hash:
                raise ValueError(f"receipt hash mismatch: {c['name']} {path}")
    expected.update(f"qualification/{p}" for p in spec["qualification_files"])
    expected.update(f"release-inputs/{p}" for p in spec.get("release_files", []))
    for name in inventory:
        relative(name)
    if set(inventory) != expected:
        raise ValueError("incomplete/unexpected manifest inventory")
    files = {}
    for name, expected_hash in inventory.items():
        raw = read(bundle, name)
        if digest(raw) != expected_hash:
            raise ValueError(f"file hash mismatch: {name}")
        files[name] = raw
    files["manifest.json"] = read(bundle, "manifest.json")
    expected_sums = "".join(f"{digest(raw)}  {p}\n" for p, raw in sorted(files.items())).encode()
    if read(bundle, "SHA256SUMS") != expected_sums:
        raise ValueError("checksum inventory mismatch")
    actual = set()
    for p in bundle.rglob("*"):
        if p.is_symlink() or (not p.is_dir() and not p.is_file()):
            raise ValueError("nonregular bundle entry")
        if p.is_file():
            actual.add(p.relative_to(bundle).as_posix())
    if actual != expected | {"manifest.json", "SHA256SUMS"}:
        raise ValueError("extra/missing bundle files")


def archives(spec, bundle, output):
    """Create independently consumable component assets plus the complete stack."""
    verify(spec, bundle)
    groups = {spec["release"]: sorted(p.relative_to(bundle).as_posix()
              for p in bundle.rglob("*") if p.is_file())}
    for c in spec["components"]:
        relative(c["release_tag"])
        groups[c["release_tag"]] = [f"artifacts/{c['name']}", "manifest.json", "COMPONENTS.md"] + sorted(
            p.relative_to(bundle).as_posix() for p in (bundle / "provenance" / c["name"]).rglob("*") if p.is_file())
    relative(spec["release"])
    output.mkdir(mode=0o700)
    sums = []
    for name, paths in groups.items():
        asset = output / f"{name}-{spec['platform']}.tar.gz"
        # Stable metadata; component assets carry only their own binary/provenance.
        with asset.open("xb") as target, gzip.GzipFile(fileobj=target, mode="wb", filename="", mtime=0) as compressed:
            with tarfile.open(fileobj=compressed, mode="w") as archive:
                payloads = {p: read(bundle, p) for p in paths}
                if name != spec["release"]:
                    payloads["SHA256SUMS"] = "".join(f"{digest(raw)}  {p}\n" for p, raw in sorted(payloads.items())).encode()
                for path, raw in sorted(payloads.items()):
                    info = tarfile.TarInfo(path)
                    info.size = len(raw)
                    info.mode = 0o755 if path.startswith("artifacts/") else 0o644
                    archive.addfile(info, io.BytesIO(raw))
        sums.append(f"{digest(asset.read_bytes())}  {asset.name}\n")
    (output / "SHA256SUMS").write_text("".join(sorted(sums)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["package", "verify", "archives"])
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--bundle", type=Path, required=True)
    parser.add_argument("--inputs", type=Path)
    parser.add_argument("--repo", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        spec = json.loads(args.spec.read_bytes())
        if args.command == "package":
            if args.inputs is None or args.repo is None:
                parser.error("package requires --inputs and --repo")
            package(spec, args.inputs, args.repo, args.bundle)
        elif args.command == "archives":
            if args.output is None:
                parser.error("archives requires --output")
            archives(spec, args.bundle, args.output)
        else:
            verify(spec, args.bundle)
    except (ValueError, OSError, KeyError, TypeError, subprocess.CalledProcessError) as error:
        parser.exit(1, f"rejected: {error}\n")
    print("passed: bundle integrity/completeness only; no deployment or physical claim")


if __name__ == "__main__":
    main()
