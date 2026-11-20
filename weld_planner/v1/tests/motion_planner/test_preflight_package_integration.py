"""Optional exact-wheel integration; enabled after provisioning the locked tool."""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest
from weld_motion_planner.io import preflight

EXECUTABLE = os.environ.get("WELD_PREFLIGHT_EXECUTABLE")


@pytest.mark.skipif(
    EXECUTABLE is None,
    reason="set WELD_PREFLIGHT_EXECUTABLE to run the exact-wheel integration",
)
def test_exact_locked_wheel_ingests_unchanged_native_bytes(tmp_path: Path) -> None:
    request_sha256 = "1" * 64
    machine_planning_calibration_sha256 = "2" * 64
    source_revision = "3" * 40
    document = {
        "schema": "amr-weld-planner-v1.motion-plan-result.v1",
        "request": {
            "request_sha256": request_sha256,
            "request_id": "package-integration",
            "cell_id": "rosie_1400_v3",
            "cell": {"id": "rosie_1400_v3", "path": "cell.json"},
            "contents": [{"path": "cell.json", "sha256": machine_planning_calibration_sha256}],
        },
        "producer": {
            "source_revision": source_revision,
            "source_revision_state": "bound",
        },
        "coverage": {"execution_authority": "none"},
        "seams": [],
    }
    artifact = tmp_path / "native-result.json"
    artifact.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    manifest = tmp_path / "native-result.preflight.json"
    preflight.write_candidate_manifest(
        manifest,
        artifact,
        document,
        source_revision=source_revision,
        source_state="clean",
        source_diff_sha256=None,
    )
    store = tmp_path / "candidate-store"
    assert preflight.ingest(Path(EXECUTABLE or ""), manifest, store) == 0

    receipts = list(store.glob("*/receipt.json"))
    assert len(receipts) == 1
    receipt = json.loads(receipts[0].read_text(encoding="utf-8"))
    assert receipt["candidate_evaluation_state"] == "not_evaluated"
    assert receipt["candidate_authority"] == "proposal_only"
    assert receipt["execution_authority"] == "none"
