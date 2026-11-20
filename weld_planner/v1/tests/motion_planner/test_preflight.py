"""The optional Preflight handoff is a process boundary over native bytes."""

from __future__ import annotations

import hashlib
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from weld_motion_planner.io import preflight

REQUEST_SHA256 = "1" * 64
CELL_SHA256 = "2" * 64


@pytest.mark.parametrize("repository,tag_prefix", [("weld-preflight", "v"), ("RosieOS", "preflight-v")])
def test_package_release_locations(tmp_path: Path, repository: str, tag_prefix: str):
    document = json.loads(preflight.DEFAULT_PACKAGE_LOCK.read_text())
    wheel = document["wheel"]
    wheel["release_url"] = (
        f"https://github.com/gradient-industrial-robotics/{repository}/releases/download/"
        f"{tag_prefix}{document['version']}/{wheel['filename']}"
    )
    path = tmp_path / "lock.json"
    path.write_text(json.dumps(document))
    assert preflight.load_package_lock(path).wheel_release_url == wheel["release_url"]
    wheel["release_url"] = wheel["release_url"].replace("github.com/", "github.com.evil/")
    path.write_text(json.dumps(document))
    with pytest.raises(preflight.PreflightUnavailable, match="release URL"):
        preflight.load_package_lock(path)

SOURCE_REVISION = "3" * 40
ROSIEOS_REPOSITORY = (
    "https://github.com/gradient-industrial-robotics/RosieOS.git"
)


def _capabilities(*, version: str = "0.3.1") -> dict[str, object]:
    return {
        "schema": "amr.weld-preflight.capabilities.v1",
        "distribution": "amr-motion-bench",
        "package_version": version,
        "requires_python": ">=3.12,<3.14",
        "cli": "weld-preflight",
        "commands": ["capabilities", "ingest-candidate"],
        "accepted_candidate_schemas": [
            "amr.weld-preflight.candidate-ingest.v0"
        ],
        "candidate_authority": "proposal_only",
        "execution_authority": "none",
    }


def _document() -> dict[str, object]:
    return {
        "schema": "amr-weld-planner-v1.motion-plan-result.v1",
        "request": {
            "request_sha256": REQUEST_SHA256,
            "request_id": "fixture-a",
            "cell_id": "rosie_1400_v3",
            "cell": {"id": "rosie_1400_v3", "path": "cell.json"},
            "contents": [{"path": "cell.json", "sha256": CELL_SHA256}],
        },
        "producer": {
            "source_revision": SOURCE_REVISION,
            "source_revision_state": "bound",
        },
        "coverage": {"execution_authority": "none"},
        "seams": [],
    }


def test_manifest_points_at_unchanged_native_bytes_and_adds_no_verdict(
    tmp_path: Path,
):
    artifact = tmp_path / "native-result.json"
    artifact.write_bytes(json.dumps(_document(), sort_keys=True).encode())

    manifest = preflight.candidate_manifest(
        artifact,
        _document(),
        source_revision=SOURCE_REVISION,
        source_state="clean",
        source_diff_sha256=None,
    )

    assert manifest["schema"] == "amr.weld-preflight.candidate-ingest.v0"
    assert manifest["case_identity"] == {
        "id": "fixture-a",
        "revision": REQUEST_SHA256,
        "sha256": REQUEST_SHA256,
    }
    assert manifest["model_identity"] == {
        "id": "rosie_1400_v3",
        "revision": CELL_SHA256,
        "sha256": CELL_SHA256,
    }
    assert manifest["source_identity"] == {
        "repository": ROSIEOS_REPOSITORY,
        "revision": SOURCE_REVISION,
        "dirty": False,
    }
    assert preflight.REPOSITORY == ROSIEOS_REPOSITORY
    assert manifest["artifact"]["path"] == artifact.name
    assert manifest["artifact"]["sha256"] == hashlib.sha256(
        artifact.read_bytes()
    ).hexdigest()
    assert manifest["artifact"]["representation"] == "planner_native"
    assert "verdict" not in manifest
    assert "execution_authority" not in manifest


def test_preflight_refuses_missing_or_ambiguous_source_identity(tmp_path: Path):
    artifact = tmp_path / "native-result.json"
    artifact.write_bytes(json.dumps(_document()).encode())

    with pytest.raises(preflight.PreflightUnavailable, match="revision is unavailable"):
        preflight.candidate_manifest(
            artifact,
            _document(),
            source_revision=None,
            source_state="clean",
            source_diff_sha256=None,
        )
    with pytest.raises(preflight.PreflightUnavailable, match="diff SHA-256"):
        preflight.candidate_manifest(
            artifact,
            _document(),
            source_revision=SOURCE_REVISION,
            source_state="dirty",
            source_diff_sha256=None,
        )


def test_ingest_invokes_only_the_public_weld_preflight_command(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    manifest = tmp_path / "native-result.preflight.json"
    store = tmp_path / "receipts"
    calls: list[list[str]] = []

    def run(argv: list[str], **kwargs: object) -> SimpleNamespace:
        calls.append(argv)
        if argv[-1] == "capabilities":
            assert kwargs == {
                "check": False,
                "capture_output": True,
                "text": True,
                "timeout": 10,
            }
            return SimpleNamespace(
                returncode=0,
                stdout=json.dumps(_capabilities()),
                stderr="",
            )
        assert kwargs == {"check": False}
        return SimpleNamespace(returncode=7)

    monkeypatch.setattr(preflight.subprocess, "run", run)
    status = preflight.ingest(Path("weld-preflight"), manifest, store)

    assert status == 7
    assert calls == [
        ["weld-preflight", "capabilities"],
        [
            "weld-preflight",
            "ingest-candidate",
            str(manifest),
            "--store",
            str(store),
        ],
    ]


def test_missing_weld_preflight_runtime_is_capability_unavailable(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
):
    def missing(*_args, **_kwargs):
        raise FileNotFoundError("not installed")

    monkeypatch.setattr(preflight.subprocess, "run", missing)
    with pytest.raises(preflight.PreflightUnavailable, match="executable not found"):
        preflight.ingest(
            Path("weld-preflight"),
            tmp_path / "candidate.preflight.json",
            tmp_path / "store",
        )


def test_package_lock_and_capabilities_fail_closed_before_ingest(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    package_lock = preflight.load_package_lock()
    assert package_lock.version == "0.3.1"
    assert package_lock.python == "3.12"
    assert package_lock.execution_authority == "none"

    calls: list[list[str]] = []

    def wrong_version(argv: list[str], **_kwargs: object) -> SimpleNamespace:
        calls.append(argv)
        return SimpleNamespace(
            returncode=0,
            stdout=json.dumps(_capabilities(version="0.3.2")),
            stderr="",
        )

    monkeypatch.setattr(preflight.subprocess, "run", wrong_version)
    with pytest.raises(preflight.PreflightUnavailable, match="do not match"):
        preflight.ingest(
            Path("weld-preflight"),
            Path("candidate.preflight.json"),
            Path("store"),
        )
    assert calls == [["weld-preflight", "capabilities"]]


def test_wheel_hash_is_an_installation_gate(tmp_path: Path) -> None:
    wheel = tmp_path / "candidate.whl"
    wheel.write_bytes(b"exact-wheel")
    package_lock = replace(
        preflight.load_package_lock(),
        wheel_filename=wheel.name,
        wheel_sha256=hashlib.sha256(wheel.read_bytes()).hexdigest(),
    )
    preflight.verify_wheel(wheel, package_lock)
    wheel.write_bytes(b"changed-wheel")
    with pytest.raises(preflight.PreflightUnavailable, match="digest"):
        preflight.verify_wheel(wheel, package_lock)
