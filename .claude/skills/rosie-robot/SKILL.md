---
name: rosie-robot
description: Move, run, jog and stop a RosieOS robot with the `rosie` CLI — status first, select, arm, load/play, the two-step `rosie run` confirmation, bounded joint/Cartesian jogs, pause, finish loop, stop, disarm, and the heartbeat keepalive. Use whenever a task would make a (simulated) robot move or end motion.
---

# Robot actions with rosie

Every action is an OLP request (OLP holds the grant, the 5 s deadman and the
maintenance/control gates); the rules are the library's, shared with the MCP
server. Reference: `rosie docs show cli "Running it"`.

## Before any motion, every time

```bash
rosie status          # control free/yours, no maintenance, cell SIMULATION or REAL, armed, session
rosie target show     # robot_actions: off | simulation | cell
```

- `robot_actions off` (default): only stop/disarm/pause/finish work. Say what
  would happen and stop. Turning it on is the user's call (`rosie target add
  local --robot-actions simulation`); `cell` (real robots) is the owner's only.
- Someone else controls the robot, or a maintenance notice is up → hands off.
- Name the cell and SIMULATION/REAL to the user before the first motion.
- Always pass `--sim` to motion commands unless the owner explicitly asked
  for a real robot: it refuses a real cell whatever the target allows.

## Running a planned program (the normal path)

```bash
rosie program check && rosie view scene      # rosie-verify-and-view: look first
rosie run --sim                              # PREVIEW: OLP's action + each joint's from/to; nothing moves; prints a token
# show the user the action and joint table; only if they agree:
rosie run --sim --confirm <token>            # runs, waits until it ends (--no-wait to return)
```

- The confirm carries OLP's joints list unchanged; `confirmation_stale` = a
  joint moved > 0.1° since: preview again.
- `--repeat count --count 3` / `--repeat loop` need a plan made with
  `rosie program plan --repeat …` (else `loop_not_closed`). Loops from rosie
  are attended: they end with the keepalive window.
- Manual path: `rosie robot load --sim` then `rosie robot play --sim` (Play
  needs every joint at the program's first sample; otherwise use `run`).
  `--trajectory <file.rdt>` on load/run uses that file's identity instead of
  the workspace plan.

## Jogging (bounded per call; needs an armed session: `rosie robot arm --sim`)

| Command | What | Bounds |
| --- | --- | --- |
| `rosie robot move J1=10,J3=-5 --sim` | relative joint travel, one synchronised move | ≤ 30° per joint per call, described range, fraction ≤ 0.5 (default 0.1) |
| `rosie robot move J2=-20 --to --sim` | absolute targets | travel measured from the live position |
| `rosie robot step z -20 --sim` | straight TCP step, base (or `--frame tool`) | ≤ 100 mm / 15° per call |
| `rosie robot hold J3 + --seconds 1 --sim` | timed hold-to-run | ≤ 3 s at ≤ 0.25; End always sent |

Bigger moves are several calls, each re-checked — never a raised cap.
`jog_mode_conflict` = a program plays or a loop runs: that is the answer
(don't Stop someone's loop to jog). A hold reports OLP's jog end:
`singularity_gate` (reverse out, or joint-jog), `joint_limit` /
`workspace_limit` (go the other way), `halted` / `inhibited` (re-arm).

## Ending motion (always available)

```bash
rosie robot stop      # Stop: session and grant released, loop ended, keepalive ended
rosie robot pause     # hold on the path (not Stop)
rosie robot finish    # a loop ends at program start after this cycle
rosie robot disarm
```

"STOP … NOT confirmed" → tell the operator to use the hardware E-stop now.

## The keepalive

A command is one process; OLP stops the robot when its grant holder sends no
heartbeat for 5 s. After each successful motion command rosie starts (or
extends) a background keepalive that heartbeats the session for the target's
window (default 120 s) from the last motion command. `rosie status` /
`rosie robot keepalive` show it; `robot stop|disarm` end it; when it lapses,
OLP's deadman stops the robot unless an operator's browser/Deck keeps the
session alive — expect to re-arm after a long pause.

## Never

Home (HM35), fault reset, recover-to-zero, raising jog caps or switching
`robot_actions` to get past a refusal, motion on a REAL robot the user didn't
name, or acting while another client controls the robot.
