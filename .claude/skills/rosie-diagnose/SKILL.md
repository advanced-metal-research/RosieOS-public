---
name: rosie-diagnose
description: Read-only diagnosis of a RosieOS cell with the `rosie` CLI — `rosie cell diagnose` (every layer, drive alarms looked up, state / cause / next step), status, rt-core events and Describe, alarm lookup, docs search, and comparing a cell's running configuration pin with its manifest. Use for "why won't it arm", "what does Er41.1 mean", "why did it stop", "is the cell OK". Acts on nothing.
---

# Diagnosing a cell (read-only)

```bash
rosie cell diagnose ["what the operator saw"]
```

One command reads OLP's control view and maintenance notice, the machine
status (session, stop reason, last error, loop, last jog end, axes, drive
alarms), rt-core's status and recent events when rt-control is configured,
and looks up every drive alarm against the drive profile. The verdict is the
state; the table gives each finding with its next step; exit 1 when something
blocks (a non-resettable alarm). Answer the user with: **state**, **cause**,
**what the operator does next**, citing the finding and the document.

## Going deeper

| Need | Command |
| --- | --- |
| One drive code | `rosie cell alarm Er41.1` (also `ALF1.3`, `0x3230`, or words) — says whether Arm resets it |
| rt-core's event log | `rosie cell events [--after <seq>] [--limit 50]`; continue with the printed `--after` |
| What config the controller runs | `rosie cell describe` (backend, configuration pin, axis limits) |
| What the repo approves | `rosie cell show <cell>` / `rosie cell show <cell> cell_deploy` |
| The deploy tool's own read | `rosie deploy inspect <cell> verify` (or `precheck`, `config_dry_run`) |
| A refusal code you don't know | `rosie docs search <code>`; `rosie docs show offline-programming "Jog end reasons"` |
| A page for a person | `rosie report cell --open` |

## Concluding (owner rules)

| You see | It means / next |
| --- | --- |
| maintenance notice | a deploy/build/test holds the cell; wait (it never stops a loop) |
| control `other` / `in_use_by_other_controller` | another client holds the robot; it releases or presses Stop — never take over |
| alarm the profile classes resettable | the operator Arms after checking the cause |
| alarm NOT resettable | the owner's call: fix at the drive; never re-home, never widen a tolerance |
| Home not valid after real reference loss (encoder/battery alarm, `coordinate_reference`) | the operator runs HM35 at the mechanical home pose — never suggest Home to clear a fault |
| `target_lead` warning | a warning, measurement input for issue #158; not a fault |
| `ui_heartbeat_lost` | the grant holder's heartbeats stopped > 5 s (closed tab, lapsed keepalive) |
| `planning_start_moving` | a plan was asked while the arm moved |
| running pin ≠ committed pin | configuration drift: bring the live values into the repo; never apply an uncommitted config |

Never decode telemetry or Stop dumps on the servo host; restarting
`rosie-rt-core` (which deletes them) is the owner's `realtime-core restart`,
after `copy-dumps`.
