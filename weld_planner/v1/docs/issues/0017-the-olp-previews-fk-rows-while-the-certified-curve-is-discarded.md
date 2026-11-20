# 0017 — The OLP previews FK'd rows while the planner's certified curve is discarded

**Status:** open, opened 2026-09-10 and **promoted the same day**. The route
named below has since been deleted with the authoring server, so the claim now
lands on whatever `internal/seam` puts in its place: the trajectories have to
reach the OLP, and today nothing carries them. It was filed
as a quality item — the preview reads the most derived of three documents — and
D24 then decided to delete `export/robot_v4.py` outright, which leaves a rails
plan with no rows to read at all. This is now part of the `seam_worker` cutover
rather than a follow-up to it.

## What is wrong

A rails plan is previewed in Offline Programming by interpolating
`robot.v4.program.v1` rows:

    offline-programming/v1/ui/src/domain/playback.ts:26
    export function previewTrajectory(rows: AuthoredProgramRow[]): PlanTrajectorySample[]

Those rows are a derivative. The motion server's native result
(`amr-weld-planner-v1.motion-plan-result.v1`) carries the planner's own joint
trajectory — `trajectories[seam_id]` as a cubic Hermite wire `(q, qdot, t)` at
`WIRE_DT_S`, plus `connecting_trajectories` — and `trajectory_json` states what
that buys a reader:

> a reader that interpolates it is playing the SAME curve the optimizer shaped
> and the verifier certified: the samples are exact evaluations of that Hermite,
> not a resampling somebody chose the resolution of, and a Hermite through them
> reproduces it to float64

The OLP never sees it. `/api/export/olp/rows` consumes the result, runs FK over
it to build rows, and returns everything except the trajectories:

    weld_planner/v1/python/weld_programmer/server/export_olp.py:378  (deleted in D24)
    {"ok", "program", "rows", "motion_rows", "cell_id", "refined",
     "dense" (summary only), "dense_error", "seams" (verdicts only)}

So the chain the preview actually plays is: planner joint curve -> FK to TCP
poses (`export/robot_v4.py:218`, `_tcp_pose`) -> sparse rows -> componentwise
lerp in the browser. Three lossy steps away from a curve that was already
correct, already certified, and already sitting in the response's sibling
document.

## How it was found

Reading the rails path end to end while deciding whether `export/robot_v4.py`
can be deleted. The first answer was "not yet" — the catalog and the SteamDeck
pendant read program.v1 — but the preview's dependency on it turned out to be
incidental rather than structural. D24 then took the harder line and dropped
rails rows entirely, accepting the loss of catalog and pendant reach for rails
until the executors move.

## Why it happens

The rails compile was built as one server-side round trip: pack the plan
request, plan it, compile rails to rows. Rows were the only output because rows
were the only thing OLP could hold at the time. `project.rows` is still where a
plan lands (`main.ts:1064`), and `previewTrajectory` was written against it.

Two facts have changed since:

- `project.denseTrajectory` exists (`main.ts:1069`), so the project already
  holds a second, joint-space description of the same motion.
- program.v2 nodes are the authored program, and they carry no joint values by
  design — "holding intent only: no joint values, no sampled trajectories, no
  solved freespace paths". They are not, and cannot become, the preview source.
  The preview source has to be a planner output.

The candidate outputs, in the order they are derived:

| representation | content | consumer today |
|---|---|---|
| native result `trajectories` | Hermite knots `(q, qdot, t)`, degrees | nothing in OLP |
| dense blob | 100 Hz samples, `f64 t_s, q_rad[9], qd_rad_s[9], u8 flags` | joint trajectory daemon |
| program.v1 rows | FK'd TCP pose + joints, sparse | catalog, pendant, OLP preview |

The first is the original. The other two are sampled or FK'd from it, and the
preview reads the most derived of the three.

## What was done

Nothing yet. What is owed:

1. Pass `trajectories` and `connecting_trajectories` through
   `/api/export/olp/rows` untouched, the way `dense` already is.
2. Interpolate the Hermite in the browser.
   `weld_planner/v1/ui/src/panes/plan/motion-panel.ts` already does this;
   `trajectory_json` calls it "nine lines of arithmetic on the other side".
3. Take TCP poses from the viewer's own FK rather than from the row.
   `viewer/urdf.ts` already poses the robot from joints, so the TCP marker and
   the planned torch path (`viewer/workstation.ts:241`, `:457`) can read the
   end-effector transform instead of `sample.tcpPose`.
4. Leave classic alone. It has no native result — its rows ARE the Tesseract
   planner's output — so `rows-from-nodes.ts` and `previewTrajectory` stay for
   the posed-polyline flow until Motion Server v1 emits a joint trajectory of
   its own.

**Do not glue the clocks.** Both documents warn about the same mistake, in
nearly the same words:

> Free space is NOT here: these are per-seam trajectories with no approach,
> retract or transit between them (M6), so a player must treat each as its own
> clock and not run one into the next. — `trajectory_json`

> Each block keeps the segment's own clock (`t_s` starts at 0.0) ... gluing them
> onto one timeline is exactly the mistake a player must not make.
> — `weldplan/dense_joint_trajectory.py`

Weld and connecting trajectories are played in order, each from its own zero.

With D24's deletion this is not an improvement to the preview but its
replacement: a rails plan will have no `robot.v4.program.v1` form, and
`previewTrajectory(rows)` will have nothing to read. Classic is untouched —
`rows-from-nodes.ts` still lowers its nodes and Tesseract still reads them.
Catalog and pendant reach for rails returns with the executor migration to
`robot.v4.program.v2`, tracked separately.
