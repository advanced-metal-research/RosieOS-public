# 0014 — The corpus cells predate the real joint bounds, and 14 tests pin the old arm

**Status:** open. Found 2026-08-22 while gating the OLP export
(`docs/OLP-EXPORT.md`); the refresh tool exists, the re-bundle is blocked on
the tests below.

## What happens

The three corpus requests in `data/motion/*.weldplan` carry a cell captured
before `26e61b0` gave the arm its real URDF bounds:

| joint | bundled | URDF today |
|---|---|---|
| J3 | −4.2 .. 1.53 | −1.57 .. 1.53 |
| J4 | −6.3 .. 6.3 | −π .. π |
| J5 | −6.3 .. 6.3 | −3.37 .. 1.3 |

A plan made from the committed bundles therefore emits joints today's cell
refuses — the OLP export gate met this as 2,356 `joint_limit` rejections from
the Tesseract checker, plus a J5 transit knot at −3.421 rad that has NO
representative inside the real bounds at all (−3.421 + 2π = 2.86 > 1.3). The
planner was planning an arm that no longer exists.

`tools/refresh_corpus_cells.py` rewrites the bundles against the current URDF,
preserving program, placement, tooling, STEP bytes, request id and timestamp.
Run against the refreshed bundles, the full pipeline is CLEAN end to end —
all three corpus programs plan, export, and pass OLP's simulator with
collision, limits and singularity clear.

## Why the re-bundle is not committed yet

With the refreshed cells, `tests/motion_planner` fails 14 of 550:

    test_ik_closed:  pose 16: closed form has 8 postures, sixty-four restarts find fewer
    test_nodes:      no weld point has two postures; the branches collapsed
    test_nodes:      only 25% of weld points have a posture at the authored standoff
    test_nodes:      the fallback found a posture the exact enumeration says does not exist
    … (14 total)

These are not planner bugs surfacing — they are assertions authored against
the ±6.3 rad arm. Halving J4's range and cutting J5 to −3.37..1.3 removes
whole IK branches: weld points that had a wrist-flipped second posture now
genuinely have one, and postures at the authored standoff genuinely thin out.
The tests pin capabilities of an arm the URDF no longer describes.

## What closing this takes

Re-derive the expectations against the real bounds, test by test, deciding in
each case whether the assertion's NUMBER changes (fewer postures is now
correct) or its PREMISE does (a branch-diversity property that can no longer
hold on this cell should say so, not be weakened silently). That is planner-
domain judgment, not mechanical renumbering — which is why this issue exists
instead of a commit. Until then the committed corpus keeps the old cells, the
suite stays green, and anything that needs current-bounds corpus behaviour
(the OLP export gate does) re-bundles at run time with the tool.
