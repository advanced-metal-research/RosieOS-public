"""The worker as `internal/seam` runs it: a process, a request on stdin.

`--preload` is how a spare is started, and the process leaves with os._exit
once the answer is written -- so these check what a caller actually sees at
the pipe: the answer, and an exit code that still says whether it failed.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

from seam_worker import paths

EXAMPLE = paths.REPO_ROOT / "weld_planner" / "v1" / "schema" / "examples" / "dog_box_lap_perimeter.json"


def _worker(payload: bytes, *flags: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-m", "seam_worker.workers", "--stdin", *flags],
        input=payload, capture_output=True, env=dict(os.environ), timeout=120,
    )


def test_a_preloaded_worker_answers_its_request():
    joint = json.loads(EXAMPLE.read_text())["weld_joints"][0]
    request = {
        "operation": "sample_seam_frames",
        "curve": joint["weld_line"],
        "reference": {"mode": "rail_curve", "rail": joint["bisector_rail"]["curve"]},
        "samples": 20,
    }
    done = _worker(json.dumps(request).encode(), "--preload")
    assert done.returncode == 0, done.stderr.decode()
    answer = json.loads(done.stdout)
    assert answer["frames"]["frames"], answer


def test_a_refusal_still_exits_nonzero_after_the_fast_exit():
    done = _worker(b"not json", "--preload")
    assert done.returncode == 2
    assert json.loads(done.stdout)["error"]["code"] == "invalid_request"
