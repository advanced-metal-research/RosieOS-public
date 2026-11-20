# 0013 — The weld pass itself should start and end at rest

**Status:** open, by design decision 2026-08-21. The M6 side is done (the
creep handover); the M5/authoring side is the part still owed, and it is a
process question before it is a planner question.

## What is wrong

A weld trajectory's first and last knots carry the seam's own travel speed:
M5 declares `times_s` from the travel speed — a process parameter — and the
tip-velocity projection gives every knot, the ends included, the rate that
tracks the seam at that clock. So the arm is asked to be ALREADY MOVING at
travel speed at the instant the arc starts, and still moving at it when the
arc stops.

The connecting motions used to honour that by arriving at exactly that
velocity (the old C1 handover contract). Since the creep change they refuse
to: every weld-side end of an approach, transit and retract is clamped to
REST with a crawl run-up (`freespace.CREEP_PLATEAU`, measured: the last
20-39 mm of tip travel at 86-131 mm/s). That is the right physics — an arc
starts and stops on a still torch — but it buys it by BREAKING C1 at every
weld boundary: the joined plan now has a velocity step from 0 to travel
speed at each arc start, and from travel speed to 0 at each arc end, that
nothing plans, verifies, or plays.

## How it was found

Making the creep change (2026-08-21). Clamping the motion ends to rest
immediately raised the question of what they are resting AGAINST, and the
answer today is a weld that takes off at full travel speed from knot 0.

## Why it happens

The weld's clock is pinned on purpose — `weld_trajopt` may not redistribute
time, because travel speed is process, not preference. Starting and ending
at rest therefore cannot be a velocity clamp inside the existing knots: a
rest end under the same `times_s` is a trajectory that no longer delivers
the declared travel speed near the ends, and T1 (`certify_tracking` /
travel mobility) judges exactly that delivery.

So the ramp has to be AUTHORED, the way standoff is: the weld line itself
begins with the arm at rest, accelerates to travel speed over some run-in,
and decelerates to rest over some run-out — with the arc-start dwell and
crater-fill time that real controllers already spend at those ends. How long
the run-in and run-out are, and whether the arc burns during them, are weld
process parameters (like `process.travel_speed`, `process.standoff_mm`) that
no committed plan request authors today.

## What was done, and what is owed

Done: the M6 half. Every weld-side motion end is at rest with a legal
braking envelope into a crawl, retime refuses horizons too short for it, and
`test_every_handover_is_a_stop_reached_at_a_crawl` pins the contract.

Owed, in order:

1. A process decision: run-in/run-out (or dwell) as authored parameters on
   the seam's process block — not invented by the planner.
2. M5 consumes them: the weld trajectory's own first/last knot velocities
   become 0, its `times_s` gaining the authored ramp, and T1's travel-speed
   judgement excluding (or separately judging) the ramp region.
3. M6 then meets a weld that is itself at rest, and the rest-to-rest
   handover becomes C1 again with no contract broken — the creep run-up
   stays, because closing the last centimetres slowly is about the fixture,
   not about the handover.

## Interim (2026-09-03): the dense export ramps the weld ends itself

The joint-space daemon (motion-server/joint-trajectory/v1) refuses a segment that is not at
rest at both ends (`boundary_qd_nonzero`), so the dense export could not
ship a real weld at all until this is fixed upstream. Until step 1-2 above
land, `export/dense_joint_trajectory.py` plays each weld pass through a
smoothstep time map (`_time_map`): positions stay on the certified path,
the first and last `weld_ramp_s` (0.25 s, `data/dense_joint_trajectory.params.json`)
accelerate from rest and decelerate to rest, and the pass lasts that much
longer. The arc is on throughout, so it strikes and stops on a still torch.
It is a stand-in, not the fix: the ramp is not authored, T1 does not judge
it, and the run-in is spent on the seam rather than ahead of it. Remove the
map (set `weld_ramp_s` to 0 and delete `_time_map`) when M5 consumes the
authored run-in/run-out.
