"""Installer transport remains authenticated and separate from the lock."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

INSTALLER = Path(__file__).parents[2] / "tools" / "install_weld_preflight.py"


def _installer() -> ModuleType:
    spec = importlib.util.spec_from_file_location("install_weld_preflight", INSTALLER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("release_url", "repository", "tag"),
    [
        (
            "https://github.com/gradient-industrial-robotics/weld-preflight/"
            "releases/download/v0.2.0/amr_motion_bench-0.2.0-py3-none-any.whl",
            "gradient-industrial-robotics/weld-preflight",
            "v0.2.0",
        ),
        (
            "https://github.com/gradient-industrial-robotics/RosieOS/releases/"
            "download/preflight-v0.3.1/amr_motion_bench-0.3.1-py3-none-any.whl",
            "gradient-industrial-robotics/RosieOS",
            "preflight-v0.3.1",
        ),
    ],
)
def test_private_release_download_uses_authenticated_github_cli(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    release_url: str,
    repository: str,
    tag: str,
) -> None:
    installer = _installer()
    destination = tmp_path / release_url.rsplit("/", maxsplit=1)[-1]
    calls: list[list[str]] = []

    def run(argv: list[str], **kwargs: object) -> SimpleNamespace:
        calls.append(argv)
        assert kwargs == {
            "check": False,
            "capture_output": True,
            "text": True,
        }
        destination.write_bytes(b"wheel")
        return SimpleNamespace(returncode=0, stdout="", stderr="")

    monkeypatch.setattr(installer.subprocess, "run", run)
    installer._download(release_url, destination.name, destination, "gh")
    assert calls == [
        [
            "gh",
            "release",
            "download",
            tag,
            "--repo",
            repository,
            "--pattern",
            destination.name,
            "--dir",
            str(tmp_path),
        ]
    ]
