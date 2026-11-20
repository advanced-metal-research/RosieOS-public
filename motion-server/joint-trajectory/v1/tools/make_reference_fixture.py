#!/usr/bin/env python3
"""Regenerate the checked-in reference fixture the tri-language tests assert on.

Deterministic by construction (no RNG, no timestamps -- created_at is a
constant), so a re-run only changes the fixture when the format itself
changes. When it does, the expected digest in
`testdata/dense_joint_trajectory_reference.expected.json` moves with it and every
language's test fails until its codec agrees -- that failure is the feature.

Run with any Python 3: the encoder is stdlib-only.
"""

import json
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(REPO / "weld_planner" / "v1" / "python"))

from weldplan import dense_joint_trajectory as dtj  # noqa: E402

AXES = dtj.AXIS_COUNT
BASE_Q = [0.1, -0.25, 0.5, -0.75, 1.0, -1.25, 0.0, 0.7853981633974483, -0.5]


def smoothstep_segment(kind, source_id, base_q, delta_q, samples, dt, torch=None):
    """A minimum-jerk-ish hop: q via 3u^2-2u^3, qd its exact derivative, so
    boundary qd is exactly 0.0 and the mid-segment velocities are non-trivial
    floats."""
    n = samples
    duration = dt * (n - 1)
    t = [round(i * dt, 12) for i in range(n)]
    q = []
    qd = []
    for i in range(n):
        u = (i * dt) / duration
        s = 3.0 * u * u - 2.0 * u * u * u
        sd = (6.0 * u - 6.0 * u * u) / duration
        q.append([base_q[a] + delta_q[a] * s for a in range(AXES)])
        qd.append([delta_q[a] * sd for a in range(AXES)])
    # The derivative is exactly zero at u=0 and u=1; force the bit pattern
    # anyway so the fixture cannot drift on a compiler's fused-multiply whim.
    qd[0] = [0.0] * AXES
    qd[-1] = [0.0] * AXES
    return dtj.DenseSegment(
        kind=kind,
        source_id=source_id,
        t_s=t,
        q_rad=q,
        qd_rad_s=qd,
        torch_on=torch if torch is not None else [False] * n,
    )


def main():
    delta_free = [0.02, -0.01, 0.005, 0.0, 0.015, -0.02, 0.0, 0.01, 0.0]
    free = smoothstep_segment("freespace", "move-1", BASE_Q, delta_free, 5, 0.01)
    weld_base = free.q_rad[-1]
    delta_weld = [0.001, 0.0005, 0.0, -0.001, 0.0, 0.0002, 0.0, 0.0, 0.0]
    weld = smoothstep_segment(
        "weld",
        "seam-4b1df2",
        weld_base,
        delta_weld,
        6,
        0.01,
        torch=[False, True, True, True, True, False],
    )

    plan = dtj.DensePlan(
        identity=dtj.DensePlanIdentity(
            plan_id="reference-plan-0001",
            program_id="reference-program",
            program_digest="sha256:" + "ab" * 32,
            manifest_revision=7,
            plan_revision=3,
            created_at="2026-09-01T00:00:00Z",
        ),
        segments=[free, weld],
    )
    blob = dtj.encode(plan)

    testdata = pathlib.Path(__file__).resolve().parents[1] / "testdata"
    (testdata / "dense_joint_trajectory_reference.rdt").write_bytes(blob)

    blocks = [dtj.encode_block(s) for s in plan.segments]
    expected = {
        "trajectory_digest": dtj.trajectory_digest(plan, blocks),
        "total_sample_count": sum(len(s.t_s) for s in plan.segments),
        "segment_sample_counts": [len(s.t_s) for s in plan.segments],
        "blob_bytes": len(blob),
        "block_sha256": [
            json.loads(
                json.dumps(e)  # keep plain strings
            )
            for e in [
                __import__("hashlib").sha256(b).hexdigest() for b in blocks
            ]
        ],
    }
    (testdata / "dense_joint_trajectory_reference.expected.json").write_text(
        json.dumps(expected, indent=2) + "\n"
    )
    print(f"wrote {len(blob)} bytes; digest {expected['trajectory_digest']}")


if __name__ == "__main__":
    main()
