#!/usr/bin/env python3
"""Install the exact locked Weld Preflight wheel as an isolated uv tool."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

V1 = Path(__file__).parents[1]
sys.path.insert(0, str(V1 / "python"))

from weld_motion_planner.io.preflight import (
    DEFAULT_PACKAGE_LOCK,
    inspect_capabilities,
    load_package_lock,
    verify_wheel,
)


def _download(release_url: str, filename: str, destination: Path, gh: str) -> None:
    # load_package_lock has already validated the complete release URL.
    parts = release_url.split("/")
    repository, tag = "/".join(parts[3:5]), parts[7]
    completed = subprocess.run(
        [
            gh,
            "release",
            "download",
            tag,
            "--repo",
            repository,
            "--pattern",
            filename,
            "--dir",
            str(destination.parent),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "download private Weld Preflight release: "
            f"{completed.stderr or completed.stdout}"
        )
    if not destination.is_file() or destination.stat().st_size > 100_000_000:
        raise RuntimeError("downloaded Weld Preflight wheel is absent or oversized")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lock", type=Path, default=DEFAULT_PACKAGE_LOCK)
    parser.add_argument("--wheel", type=Path)
    parser.add_argument("--target", type=Path, default=V1 / ".run/weld-preflight")
    parser.add_argument("--uv", default="uv")
    parser.add_argument("--gh", default="gh")
    arguments = parser.parse_args()

    package_lock = load_package_lock(arguments.lock)
    arguments.target.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix="weld-preflight-install-", dir=arguments.target
    ) as temporary_directory:
        wheel = Path(temporary_directory) / package_lock.wheel_filename
        if arguments.wheel is None:
            _download(
                package_lock.wheel_release_url,
                package_lock.wheel_filename,
                wheel,
                arguments.gh,
            )
        else:
            wheel.write_bytes(arguments.wheel.read_bytes())
        verify_wheel(wheel, package_lock)

        tool_directory = arguments.target / "tools"
        binary_directory = arguments.target / "bin"
        environment = {
            **os.environ,
            "UV_TOOL_DIR": str(tool_directory),
            "UV_TOOL_BIN_DIR": str(binary_directory),
        }
        completed = subprocess.run(
            [
                arguments.uv,
                "tool",
                "install",
                "--force",
                "--python",
                package_lock.python,
                str(wheel),
            ],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )
        if completed.returncode != 0:
            print(completed.stderr or completed.stdout, file=sys.stderr)
            return 1

    executable = binary_directory / "weld-preflight"
    capabilities = inspect_capabilities(executable, package_lock)
    receipt = {
        "schema": "rosieos.weld-preflight-install-receipt.v1",
        "package_lock": str(arguments.lock.resolve()),
        "distribution": package_lock.distribution,
        "version": package_lock.version,
        "python": package_lock.python,
        "wheel_sha256": package_lock.wheel_sha256,
        "executable": str(executable.resolve()),
        "capabilities": capabilities,
        "execution_authority": "none",
    }
    temporary_receipt = arguments.target / ".install-receipt.json.tmp"
    temporary_receipt.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary_receipt, arguments.target / "install-receipt.json")
    print(json.dumps(receipt, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
