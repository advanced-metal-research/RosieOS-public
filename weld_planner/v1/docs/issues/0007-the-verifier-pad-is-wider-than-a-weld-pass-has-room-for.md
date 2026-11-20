# 0007 · The verifier's safety pad is wider than a weld pass has room for, so every correct seam reports a collision it does not have

**Status:** open, **deferred on purpose.** The owner's call (2026-08-13): the
pad should probably apply, and there is a stopgap that needs no new mechanism —
grow the torch geometry and the pad is already in the model. Revisit when a
real buffer is wanted.
**Found:** 2026-08-13, wiring the T2 verifier onto M5's output for the first time.
**Blast radius today:** 5 of the 7 plannable seams on `bracket_a_placed` — all
of the ones that are correct. The two that genuinely fail fail for a different
reason (acceleration), so at the default margin the verdict carries no
information at all: everything says FAIL.

## What is wrong

`verify.DEFAULT_VERIFY_MARGIN_M` is 2 mm. The verifier reports a collision whenever
two checked bodies come within it. A weld pass holds the torch at the cell's
measured 20 mm standoff, and the gun is baked into `link_6` as its last 150 mm
of Ø30 tube — so the gun BODY clears the part by 1.3–2.4 mm while the electrode
does its job. Measured over the emitted curve, at margin 0 and margin 2 mm:

    seam        worst clearance     margin 0      margin 2 mm
    seam_0001        2.398 mm       PASS          collision
    seam_0003        1.631 mm       PASS          collision
    seam_0004        2.015 mm       PASS          collision
    seam_0006        5.955 mm       PASS          PASS
    seam_0010        6.107 mm       PASS          PASS

Nothing penetrates anything at any point of any of these trajectories; the
certificate proves it over the whole continuum. The three that flip are flipped
by the pad alone.

The failure mode is not a wrong verdict so much as a useless one. A verifier that
says FAIL for the correct answer and FAIL for the wrong answer has stopped
being a verifier, and the operator who reads it twice learns to stop reading it.

## How it was found

By running the verifier on a planned trajectory, which nothing had ever done —
until M5 shipped, the pipeline called `verify`'s helpers and never `verify`
itself. The first run came back FAIL on a seam whose tracking certificate had
just passed at 0.16 mm, which is not a combination that can both be true about
a real fault.

The screen agreed with the verifier about the geometry (1.3 mm against 1.63 mm),
which ruled out a mesh or placement bug and left the number itself. That is the
useful part of the route: two independent implementations agreeing on the
distance is what turned "the plan is wrong" into "the threshold is wrong".

## Why it happens

`verify.py`, `DEFAULT_VERIFY_MARGIN_M`, and its own docstring states the reason
honestly:

> The verifier's own safety margin, on top of the certificate. Zero would still be
> sound; this is here because the meshes are the VISUAL meshes and the fixtures
> are hand-measured.

Both clauses are true of the CELL. Neither is true of the pair that trips:

- the workpiece is exact CAD, tessellated from the STEP the program carries;
- the gun mesh is the hardware, by the owner's explicit decision on 2026-08-12
  — the mesh was chosen OVER `tooling.json`'s Ø16 precisely because it is what
  is really on the robot.

So the pad is being charged against the one pair whose geometry it was not
written for, and charged at a scale the process does not have. The same margin
against `link_4` and the weld table is exactly right and nothing here argues
otherwise.

This is the third instance of one pattern in this codebase, which is why it is
written down rather than merely fixed. `collision/cost.py`'s AL margin defaults
to 50 mm and read every weld as a violation until M5 set it to zero; the screen
carries its own margins; now the verifier. A margin tuned for *approaching* a
workpiece is wrong for *working on* one, and a weld is the case that is always
in contact-adjacent territory by definition.

## What was done

Nothing to the number, deliberately. Loosening a safety threshold to make a
verdict come out is the move this repository exists to not make, and the pad
belongs to whoever owns the cell.

What was done instead is to make the situation legible and the choice
available:

- **The verdict distinguishes contact from proximity.** `certify_segment`'s
  finding now carries `penetrating` and the margin it was judged against, and
  `Verdict.summary` reports `penetrating` beside `collisions`. A run at the
  default now reads `{'collisions': 1, 'penetrating': 0}`, which says in the
  verdict itself that nothing touched.
- **The margin is a parameter with the verifier's own default.**
  `generate.search_then_plan_weld_trajectories(verify_margin_m=...)` and `--verify-margin-mm`, defaulting
  to `DEFAULT_VERIFY_MARGIN_M` so nothing is quietly loosened. `VERIFY_MARGIN_M` in
  `weld_trajopt.py` carries the reasoning at the point of use.
- **The tests judge at margin 0**, and assert non-penetration rather than the
  verdict — the part of the answer that does not depend on anybody's policy.
  `tests/motion_planner/test_weld_trajopt.py::test_the_emitted_curve_passes_the_verifier_with_nothing_unverifiable`.

**The stopgap, and why it is a reasonable one.** A per-pair margin and an
inflated torch mesh are the same statement about the world, and only one of
them needs new mechanism: growing the gun geometry puts the buffer in the model
where every consumer — screen, AL cost, verifier, viewer — sees the same shape
without being told about it separately. That is the owner's stated plan for
now, and it is also the reason this issue is not urgent.

## What is still owed

- The decision: does the 2 mm pad apply to torch-vs-workpiece. If yes, a
  per-pair margin in `certify_segment` is the mechanism; if the answer is the
  inflated mesh instead, `spheres.json` and the `.dae` have to grow together or
  the screen and the verifier will disagree by the buffer.
- Whatever number is chosen has to be one number. Today the AL cost runs at 0,
  the verifier at 2 mm and the screen at its own, and no document states how the
  three are meant to relate.
- `report_clearance` samples the WAYPOINTS. That was sound when a trajectory
  meant a polyline; now that the emitted path is a cubic Hermite, the tightest
  instant can fall between two of them, so the reported clearance is a "no
  worse than" and the numbers in the table above are upper bounds on the true
  gap. The certificate is unaffected — it covers every instant — but if the pad
  question is ever settled by a millimetre, that report has to be tightened to
  the same standard first.
