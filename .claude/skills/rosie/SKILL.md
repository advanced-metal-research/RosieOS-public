---
name: rosie
description: Start here for any work that touches a RosieOS cell, OLP programs, the robot, rt-core, the weld planner or deploy review — the `rosie` CLI (cli/v1): install, the output contract, targets, workspaces, safety rules, and which task skill to load next. Use whenever a task involves authoring/planning/checking/viewing a weld program, robot status or motion, drive alarms, trajectories, or a release review; also when `rosie` is missing or a `rosie_*` MCP tool is mentioned.
---

# rosie: RosieOS from the shell

`rosie` is the command line for driving RosieOS. Prefer it over hand-made
curl, ad-hoc scripts or the MCP tools: one short command per job, a verdict
first, `--json` when you need fields, exit codes you can branch on. Reference:
`rosie docs show cli`; every command has `--help` with examples. These skills
work from any directory (`rosie skills install` puts them in
`~/.claude/skills`); paths like `cli/v1/...` are relative to the RosieOS
checkout, and `rosie docs show <id>` reads its documents from anywhere.

| Task | Skill |
| --- | --- |
| Services down, "unreachable", rails/planning fails while health says ok | `rosie-local-stack` |
| Write, edit, plan, save a program (waypoints, STEP parts, welds) | `rosie-programs` |
| **Check a program and LOOK at its 3D scene before anything runs** | `rosie-verify-and-view` |
| Arm, run, jog, stop a (simulated) robot | `rosie-robot` |
| "Why won't it arm / what is this alarm / why did it stop" | `rosie-diagnose` |
| `.rdt`, `.weldplan`, planner, FK / straight-line checks | `rosie-trajectories` |
| What a release would do to a cell (read-only) | `rosie-deploy-review` |

## 1. Is it installed and is everything up?

```bash
export PATH="$HOME/.rosie/bin:$PATH"
command -v rosie || echo "run cli/v1/install.sh in a RosieOS checkout"   # builds it, installs ~/.rosie/bin/rosie and these skills
rosie doctor                              # every service + the known traps, each failure with its fix
```

After a `git pull` touching `cli/v1`, `offline-programming/v1/ui/src` or
`steamdeck/real/v5/olp-core`, re-run `cli/v1/install.sh` in that checkout (doctor warns when the engine
bundle is older than OLP's sources, and when these skills are not installed
for every session).

## 2. The contract (every command)

- First line: `OK: …`, `REFUSED: …`, `UNREACHABLE: …` or `USAGE: …`. Read it;
  usually that is all you need. Then a short body, `files:` written, `next:`
  commands worth running.
- `--json` → `{ok, verdict, exit, data, artifacts, next}`; progress is on
  stderr (`-q` silences it). Use `--json` only when you need a field.
- Exit 0 ok · 1 refused / a check failed · 2 usage (see `--help`) · 3 a service
  is unreachable (run `rosie doctor`).
- Mutations print only the rows they changed. Don't follow every edit with
  `rosie program show`.

## 3. Orient before acting

```bash
rosie status      # control (free/yours/other), maintenance notice, cell SIMULATION/REAL, armed, session, keepalive
rosie target show # which OLP/planner/rt-control, robot-action mode
```

Targets (`rosie target list`): `local` is the local stack; a cell from the
Mac is its `olp ui start` tunnel (`rosie target add <cell> --olp-url
http://127.0.0.1:18894`). `--target <name>` per command.

## 4. State lives on disk (no hidden memory)

The working program is the workspace `~/.rosie/workspace/<name>/` (`--ws`,
default `current`): the program, its cell revision, searched seams,
torch/search settings, undo history. Any command can be re-run in a fresh
shell. `rosie program list` shows the cell's programs and your workspaces.
Use a separate `--ws` for a second program instead of `--force`-replacing
one with unsaved edits.

## 5. Rules no flag changes

- Report refusals in OLP's / rt-core's words. Never work around one: no
  widened bounds, no other route, no switching `robot_actions`, no `--force`
  to discard someone's work.
- Robot actions are `off` by default; `simulation` acts only on simulated
  cells; `cell` (real robots) is the owner's setting, never yours to set.
  Use `--sim` on motion commands so a real robot is refused even if the
  target allows it.
- Never Home (HM35), fault reset or recover-to-zero; Home is an operator action
  at the mechanical home pose, never a recovery.
- `rosie robot stop` is always available. "STOP … NOT confirmed" → tell the
  operator to use the hardware E-stop.
- Deploys/installs/restarts/locks/reboots are the owner's repo commands
  (`rosie docs show repo-rules`); `rosie deploy` only reads.
- Save to the cell only when asked; a 409 conflict is the user's decision.
- Before any `rosie run`: `rosie program check` and `rosie view scene`, and
  open the PNGs (`rosie-verify-and-view`).

## MCP

The `rosieos` MCP server (`mcp/v1`) is a second front end over the same
library (`cli/v1/src/lib`): same rules, same refusals. Use it only where the
host prefers MCP tools; its state is per process (lost on restart), the
CLI's is on disk. Its prompts and `rosieos://docs/*` resources still work.
