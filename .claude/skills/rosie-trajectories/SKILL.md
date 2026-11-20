---
name: rosie-trajectories
description: Dense trajectories (.rdt), .weldplan files, the weld planner and kinematics with the `rosie` CLI — inspect/validate/fetch an .rdt and its six-field identity, plan a .weldplan, planner status (CUDA vs CPU), FK and "would this straight Cartesian line be accepted". Use for any task involving an .rdt, a trajectory digest, a .weldplan, planner timing or an offline pose/line check.
---

# Trajectories, planner, kinematics

For a program in the workspace, `rosie program plan` / `preview` / `check`
are the normal path (`rosie-programs`, `rosie-verify-and-view`). These
commands are for artifacts outside it and offline checks. Nothing moves.

| Need | Command |
| --- | --- |
| Is this .rdt valid; its identity, segments, joint ranges, peak speeds | `rosie traj inspect <file.rdt>` (every dense-daemon ingest rule, offline) |
| The daemon's own verdict (needs the daemon, :8797) | `rosie traj validate <file.rdt>` |
| A filed plan's bytes from the planner | `rosie traj fetch sha256:<64 hex> [-o file.rdt]` |
| Plan a .weldplan elsewhere | `rosie program export x.weldplan` then `rosie traj plan-weldplan x.weldplan` |
| Planner CUDA or CPU, revision, plan in flight | `rosie planner status` |
| TCP pose of six joint angles | `rosie kin fk --model rosie_1420_v1 --joints 0,-30,45,0,60,0` (degrees in; needs cartesiand) |
| Would a Cartesian step be accepted | `rosie kin line --model rosie_1420_v1 --joints … --axis z --distance -50 [--frame tool]` |

- Planned trajectories of workspace programs are kept in `~/.rosie/mcp/dense/`
  (the `files:` line of `rosie program plan`).
- `rosie robot load --trajectory <file.rdt> --sim` / `rosie run --trajectory
  <file.rdt> --sim` run a file's identity instead of the workspace plan.
- A CPU-only planner (`motion-cpu`) plans MoveJ, dwell and Home only; welds
  and MoveL need the CUDA planner.
- Judge trajectories in absolute units (mm, mm/s, °, rad/s), never relative
  to the trajectory's own `dt`; the TCP must stay on its commanded path.
