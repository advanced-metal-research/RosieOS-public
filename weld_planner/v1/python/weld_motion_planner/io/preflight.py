"""Optional process boundary from native result bytes to Weld Preflight."""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any

SCHEMA = "amr.weld-preflight.candidate-ingest.v0"
REPOSITORY = "https://github.com/gradient-industrial-robotics/RosieOS.git"
PLANNER_ID = "amr-weld-planner-v1/dp-seam-search"
PLANNER_VERSION = "0.1.0"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_COMMIT = re.compile(r"^[0-9a-f]{40}$")
_VERSION = re.compile(r"^[0-9]+\.[0-9]+\.[0-9]+$")
PACKAGE_LOCK_SCHEMA = "rosieos.weld-preflight-package-lock.v1"
CAPABILITIES_SCHEMA = "amr.weld-preflight.capabilities.v1"
DEFAULT_PACKAGE_LOCK = Path(__file__).parents[3] / "weld-preflight.lock.json"


class PreflightUnavailable(RuntimeError):
    """The native result exists but lacks identity required for Preflight."""


@dataclass(frozen=True)
class WeldPreflightPackageLock:
    distribution: str
    version: str
    python: str
    wheel_filename: str
    wheel_sha256: str
    wheel_release_url: str
    capabilities_schema: str
    required_candidate_schema: str
    execution_authority: str


def _required_string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value or "\x00" in value:
        raise PreflightUnavailable(f"Weld Preflight lock {label} is invalid")
    return value


def load_package_lock(path: Path = DEFAULT_PACKAGE_LOCK) -> WeldPreflightPackageLock:
    """Load the exact package identity accepted by this RosieOS adapter."""
    if path.is_symlink() or not path.is_file():
        raise PreflightUnavailable(
            f"Weld Preflight package lock is unavailable: {path}"
        )
    try:
        document = json.loads(path.read_bytes())
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise PreflightUnavailable(
            f"Weld Preflight package lock is invalid: {error}"
        ) from error
    if not isinstance(document, dict):
        raise PreflightUnavailable(
            "Weld Preflight package lock must be one JSON object"
        )
    wheel = document.get("wheel")
    if not isinstance(wheel, dict):
        raise PreflightUnavailable("Weld Preflight package lock has no wheel identity")

    schema = _required_string(document.get("schema"), "schema")
    distribution = _required_string(document.get("distribution"), "distribution")
    version = _required_string(document.get("version"), "version")
    python = _required_string(document.get("python"), "python")
    capabilities_schema = _required_string(
        document.get("capabilities_schema"), "capabilities schema"
    )
    candidate_schema = _required_string(
        document.get("required_candidate_schema"), "candidate schema"
    )
    authority = _required_string(
        document.get("execution_authority"), "execution authority"
    )
    filename = _required_string(wheel.get("filename"), "wheel filename")
    digest = _required_string(wheel.get("sha256"), "wheel SHA-256")
    release_url = _required_string(wheel.get("release_url"), "wheel release URL")
    if schema != PACKAGE_LOCK_SCHEMA:
        raise PreflightUnavailable(f"unsupported Weld Preflight package lock: {schema}")
    if distribution != "amr-motion-bench" or not _VERSION.fullmatch(version):
        raise PreflightUnavailable("Weld Preflight package identity is invalid")
    if python != "3.12":
        raise PreflightUnavailable("Weld Preflight must use isolated Python 3.12")
    if capabilities_schema != CAPABILITIES_SCHEMA or candidate_schema != SCHEMA:
        raise PreflightUnavailable(
            "Weld Preflight package capabilities are incompatible"
        )
    if authority != "none":
        raise PreflightUnavailable(
            "Weld Preflight package lock grants execution authority"
        )
    if not _SHA256.fullmatch(digest):
        raise PreflightUnavailable("Weld Preflight wheel SHA-256 is invalid")
    if filename != f"amr_motion_bench-{version}-py3-none-any.whl":
        raise PreflightUnavailable(
            "Weld Preflight wheel filename does not match version"
        )
    release_root = "https://github.com/gradient-industrial-robotics"
    if release_url not in {
        f"{release_root}/weld-preflight/releases/download/v{version}/{filename}",
        f"{release_root}/RosieOS/releases/download/preflight-v{version}/{filename}",
    }:
        raise PreflightUnavailable("Weld Preflight wheel release URL is invalid")
    return WeldPreflightPackageLock(
        distribution=distribution,
        version=version,
        python=python,
        wheel_filename=filename,
        wheel_sha256=digest,
        wheel_release_url=release_url,
        capabilities_schema=capabilities_schema,
        required_candidate_schema=candidate_schema,
        execution_authority=authority,
    )


def verify_wheel(path: Path, package_lock: WeldPreflightPackageLock) -> None:
    """Verify a local release wheel before it is installed as a tool."""
    if path.is_symlink() or not path.is_file():
        raise PreflightUnavailable(f"Weld Preflight wheel is unavailable: {path}")
    if path.name != package_lock.wheel_filename:
        raise PreflightUnavailable(
            "Weld Preflight wheel filename differs from the lock"
        )
    if _sha256(path.read_bytes()) != package_lock.wheel_sha256:
        raise PreflightUnavailable("Weld Preflight wheel digest differs from the lock")


def inspect_capabilities(
    executable: Path,
    package_lock: WeldPreflightPackageLock,
) -> dict[str, Any]:
    """Require an exact installed package and schema handshake before ingest."""
    try:
        completed = subprocess.run(
            [str(executable), "capabilities"],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
    except FileNotFoundError as error:
        raise PreflightUnavailable(
            f"Weld Preflight executable not found: {executable}"
        ) from error
    except subprocess.TimeoutExpired as error:
        raise PreflightUnavailable(
            "Weld Preflight capability probe timed out"
        ) from error
    if completed.returncode != 0:
        raise PreflightUnavailable(
            f"Weld Preflight capability probe failed with status {completed.returncode}"
        )
    try:
        capabilities = json.loads(completed.stdout)
    except (TypeError, json.JSONDecodeError) as error:
        raise PreflightUnavailable(
            "Weld Preflight capabilities are unreadable"
        ) from error
    if not isinstance(capabilities, dict):
        raise PreflightUnavailable(
            "Weld Preflight capabilities must be one JSON object"
        )
    commands = capabilities.get("commands")
    schemas = capabilities.get("accepted_candidate_schemas")
    if (
        capabilities.get("schema") != package_lock.capabilities_schema
        or capabilities.get("distribution") != package_lock.distribution
        or capabilities.get("package_version") != package_lock.version
        or capabilities.get("requires_python") != ">=3.12,<3.14"
        or capabilities.get("candidate_authority") != "proposal_only"
        or capabilities.get("execution_authority") != "none"
        or not isinstance(commands, list)
        or "ingest-candidate" not in commands
        or not isinstance(schemas, list)
        or package_lock.required_candidate_schema not in schemas
    ):
        raise PreflightUnavailable(
            "Weld Preflight capabilities do not match the package lock"
        )
    return capabilities


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _machine_planning_calibration_sha256(document: dict[str, Any]) -> str:
    request = document.get("request")
    if not isinstance(request, dict):
        raise PreflightUnavailable("native result has no request identity")
    contents = request.get("contents")
    if not isinstance(contents, list):
        raise PreflightUnavailable("plan request has no content hash list")
    cell = request.get("cell")
    cell_path = cell.get("path") if isinstance(cell, dict) else "cell.json"
    matches = [
        item.get("sha256")
        for item in contents
        if isinstance(item, dict) and item.get("path") == cell_path
    ]
    if (
        len(matches) != 1
        or not isinstance(matches[0], str)
        or not _SHA256.fullmatch(matches[0])
    ):
        raise PreflightUnavailable(
            f"plan request does not bind one {cell_path!r} SHA-256"
        )
    return matches[0]


def candidate_manifest(
    artifact_path: Path,
    document: dict[str, Any],
    *,
    source_revision: str | None,
    source_state: str | None,
    source_diff_sha256: str | None,
) -> dict[str, Any]:
    """Describe one unchanged native result for ``ingest-candidate``."""
    if source_revision is None:
        raise PreflightUnavailable("planner source revision is unavailable")
    if not _COMMIT.fullmatch(source_revision):
        raise PreflightUnavailable(
            "planner source revision is not a full Git commit"
        )
    if source_state not in {"clean", "dirty"}:
        raise PreflightUnavailable(
            "planner source state must be declared clean or dirty"
        )
    if source_state == "dirty":
        if (
            source_diff_sha256 is None
            or not _SHA256.fullmatch(source_diff_sha256)
        ):
            raise PreflightUnavailable(
                "dirty planner source requires a working-tree diff SHA-256"
            )
    elif source_diff_sha256 is not None:
        raise PreflightUnavailable(
            "clean planner source cannot carry a diff SHA-256"
        )

    request = document.get("request")
    producer = document.get("producer")
    if not isinstance(request, dict) or not isinstance(producer, dict):
        raise PreflightUnavailable(
            "native result has no reusable identity envelope"
        )
    request_sha256 = request.get("request_sha256")
    request_id = request.get("request_id")
    cell_id = request.get("cell_id")
    if (
        not isinstance(request_sha256, str)
        or not _SHA256.fullmatch(request_sha256)
    ):
        raise PreflightUnavailable(
            "native result has no exact plan-request SHA-256"
        )
    if not isinstance(request_id, str) or not request_id:
        raise PreflightUnavailable("native result has no request id")
    if not isinstance(cell_id, str) or not cell_id:
        raise PreflightUnavailable("native result has no cell id")
    if producer.get("source_revision") != source_revision:
        raise PreflightUnavailable(
            "manifest source revision differs from the native result"
        )

    artifact_bytes = artifact_path.read_bytes()
    try:
        artifact_document = json.loads(artifact_bytes)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise PreflightUnavailable(
            "native result is not valid UTF-8 JSON"
        ) from error
    if artifact_document != document:
        raise PreflightUnavailable(
            "native result bytes differ from the manifest document"
        )
    machine_planning_calibration_sha256 = _machine_planning_calibration_sha256(document)
    manifest: dict[str, Any] = {
        "schema": SCHEMA,
        "candidate_id": f"{request_id}-native-{request_sha256[:12]}",
        "case_identity": {
            "id": request_id,
            "revision": request_sha256,
            "sha256": request_sha256,
        },
        "model_identity": {
            "id": cell_id,
            "revision": machine_planning_calibration_sha256,
            "sha256": machine_planning_calibration_sha256,
        },
        "source_identity": {
            "repository": REPOSITORY,
            "revision": source_revision,
            "dirty": source_state == "dirty",
        },
        "planner_identity": {
            "id": PLANNER_ID,
            "version": PLANNER_VERSION,
        },
        "artifact": {
            "path": artifact_path.name,
            "media_type": "application/json",
            "sha256": _sha256(artifact_bytes),
            "representation": "planner_native",
            "format_schema": document.get("schema"),
        },
    }
    if source_diff_sha256 is not None:
        manifest["source_identity"]["working_tree_diff_sha256"] = (
            source_diff_sha256
        )
    return manifest


def write_candidate_manifest(
    manifest_path: Path,
    artifact_path: Path,
    document: dict[str, Any],
    *,
    source_revision: str | None,
    source_state: str | None,
    source_diff_sha256: str | None,
) -> Path:
    """Write a manifest beside its artifact so the relative path stays local."""
    if manifest_path.parent.resolve() != artifact_path.parent.resolve():
        raise PreflightUnavailable(
            "Preflight manifest must be beside the native result"
        )
    manifest = candidate_manifest(
        artifact_path,
        document,
        source_revision=source_revision,
        source_state=source_state,
        source_diff_sha256=source_diff_sha256,
    )
    manifest_path.write_text(
        json.dumps(manifest, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def ingest(
    executable: Path,
    manifest_path: Path,
    store: Path,
    package_lock_path: Path = DEFAULT_PACKAGE_LOCK,
) -> int:
    """Invoke Weld Preflight's public CLI and return its exit status."""
    package_lock = load_package_lock(package_lock_path)
    inspect_capabilities(executable, package_lock)
    try:
        completed = subprocess.run(
            [
                str(executable),
                "ingest-candidate",
                str(manifest_path),
                "--store",
                str(store),
            ],
            check=False,
        )
    except FileNotFoundError as error:
        raise PreflightUnavailable(
            f"Weld Preflight executable not found: {executable}"
        ) from error
    return completed.returncode
