# OLP native verdict admission

Area: export integration. Base: `92f0c8a763ecb9b76571e864d1fd605108a7811a`.

The decision: refined OLP export and native-result-to-dense conversion may
proceed only when every supplied seam and connecting motion has affirmative,
internally consistent native verification. Seams also require passing tracking
certificates. Rejection occurs before row generation, catalog mutation, or dense
resampling, and identifies the source seam/move and failed check.

The existing rails candidate-row authoring path remains explicit: it can import
M4 candidates into a `saved_unplanned`, simulation program, but cannot accept any
continuous trajectory or connecting motion. Strict admission is the default;
only the unrefined rows route requests candidate-only compilation. Neither mode
grants execution authority. Raw result download/replay remains available.

Tests: native FAIL, REFUSED, missing/malformed/contradictory certificates,
tracking failure, omitted trajectory, join errors, and a late invalid move must
reject; a fully passing synthetic document must preserve conversion; the HTTP
export/replan routes must perform zero catalog writes on rejection. Candidate
authoring must remain usable and cannot bypass a failed continuous result.

No optimizer, timing, geometry, runtime or hardware change. The gate consumes
the planner's reports; it neither authenticates them nor re-certifies the rows
or dense samples after conversion. Existing downstream checks still apply.

Rollback: revert this focused export patch. The merged #59 judge is independent.
Stop after deterministic tests and reviewable pushed PR; no deployment or merge.

Verdict: `passed` for offline export admission.

On the base implementation, the two FAIL-seam regressions (row and dense
conversion) failed because neither converter raised. With the shared gate,
the three focused export suites pass: **85 passed** in the locked Pixi default
environment. They cover catalog-write refusal, all three HTTP export routes,
candidate-only mode, successful conversion, OLP's own TypeScript validator,
and the unchanged dense binary reference fixture. The export CLI self-test and
Ruff on the new gate/tests also pass. Existing lint findings in surrounding
modules were left unchanged.

The Export to OLP button now follows the continuous-motion admission condition,
with a reason when unavailable. All 17 UI verdict/eligibility checks and the
production UI build pass. Raw result export remains available for rejected plans.

The remaining ceiling is deliberate: this consumes source reports and does
not establish connection topology, request completeness, or post-conversion
motion validity. Existing downstream validators retain those responsibilities.
