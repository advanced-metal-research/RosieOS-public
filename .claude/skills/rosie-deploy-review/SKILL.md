---
name: rosie-deploy-review
description: Review what a RosieOS release or config change would do to a robot cell, read-only, with `rosie deploy review|inspect` (release plan, precheck, OLP status, verify, config dry-run, maintenance) and the committed manifests — then hand the owner the exact repo command. Use for "what would deploying <commit> do", "is <cell> ready for a release", "what does the cell run". Never deploys.
---

# Reviewing a deploy (rosie only reads)

Deploying, installing, restarting, locking, rolling back and rebooting are
the owner's repo commands from `steamdeck/real/v4`
(`rosie docs show deploy-and-test-workflow`; `rosie docs show repo-rules
"Deploying and testing on a robot cell"`). rosie has none of them by design.

```bash
rosie deploy review <cell> --commit <sha>     # release plan + precheck + OLP status, approvals, the exact command
rosie deploy inspect <cell> <command>         # one of: release_plan (--commit), precheck, verify, olp_status, config_dry_run, maintenance_show
rosie cell show <cell> cell_deploy            # what the manifest deploys
rosie status --target <cell>                  # is the robot in use, a loop running, a notice up?
```

These run `go run . … --manifest rt-core/config/cells/<cell>.json` over this
Mac's SSH/Tailscale (Go on PATH: `~/.rosie/go-1.26.2/bin`); allow minutes.
`<cell>` is a manifest name in `rt-core/config/cells` or a part of it that
names one manifest (a short robot name resolves).

## Tell the owner, explicitly

- The tier (config / olp / full), what changes on the robot, the duration
  (`rosie docs show deploy-and-test-workflow` for measured timings), and the
  command — never run it.
- A servo host that fails a hostcheck refuses a full release:
  `--accept-hostcheck <check>` passes it only when the owner approves that
  check for that run. No other hostcheck is ever accepted.
- Uncommitted changes under `rt-core/config`, `robot_description` or the
  manifest are refused; a running pin no commit holds needs
  `--accept-config-drift <pin>` (owner's call) — better, commit the live values.
- Building/testing on a cell and using its robot are mutually exclusive: the
  lock waits for idle and never stops a loop (Finish cycle or Stop it first).
- Servo-host reboots, `--clear-drive-alarms` and pendant installs are separate
  owner approvals.
