# Async planning integration slice

Outcome: collision/planning work must not block feedback polling, lease
renewals, Stop or Disarm. Preserve the existing Motion Server API, limits,
runtime ownership and configured cell/model bindings. No physical deployment
is performed by this slice.

Origin: JSLOP `1d65235` / `native/async-plan/motion.patch`, adapted onto RosieOS
`01e2a26`. The client-independent feedback pump is also required. RosieOS's
existing joint-limit policy is retained; this port does not import JSLOP's
separate 30-degree/second policy.

One job captures immutable rows, kinematics, URDF digest, pair/runtime identity,
and lease identity. A retry for that exact job does not recompute it; a different
request receives busy while it runs. Stop, Disarm and owner loss invalidate
its installation generation. Completed work is installed only after fresh
authority, ownership and URDF checks. No new execution authority is created.

Cancellation means discarding a result and preserving responsive control; it
does not interrupt the underlying solver midway. Bulk accepted-artifact
finalization and large-report serialization still need their separate measured
port. This slice must not be described as eliminating every control-path stall.

Judges: the nine actual-production-loop fixtures plus existing arm/feedback
binding and status-publication regressions. The parent fails feedback continuity;
the first incomplete port also failed until polling no longer depended on a
prior jog peer. Both failed attempts are retained under the September 12 runtime
comparison output. Hardware replay follows only after source-level integration.

The existing `TestRobotV4MotionServerAcceptedPlanContractDryRun` fails with
`accepted-plan-contract program is not physically playable` on both the
unchanged parent and this candidate in the same environment. It is retained
as an unresolved baseline failure, not removed or reported as a passing gate.

Verdict: **passed** for the nine offline production-loop cases and existing
Arm-binding, feedback-binding and complete-status publication regressions.
Not deployed; large-report finalization, the baseline accepted-plan failure,
and physical replay remain outside this verdict.
