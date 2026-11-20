---
name: rosie-local-stack
description: Bring up RosieOS offline programming locally (OLP API :8794, UI :5189, weld planner :8796, rt-core simulator via dev-stack.sh) and point `rosie` at it or at a real cell's tunnel. Use when `rosie doctor` fails, a command exits 3 (unreachable), rails or planning fails while health says ok, or before authoring/simulating without a robot.
---

# The local stack, and pointing rosie at things

```bash
rosie doctor        # what is up, what is missing, each failure with its fix
```

The commands below run from the RosieOS checkout (`rosie target show` prints
its `repo_root`).

## Pick the setup

| You need | Run | rosie |
| --- | --- | --- |
| Author + plan, no robot | the OLP launcher + the planner (below) | default `local` target |
| Also arm/run/jog a SIMULATED robot, rt-core reads | `./dev-stack.sh start` (the whole stack) | `local` finds dev-stack's rt-control socket itself; `rosie target add local --robot-actions simulation` (the user's call) |
| A real cell's OLP from the Mac | `go run . olp ui start --manifest rt-core/config/cells/<cell>.json` in `steamdeck/real/v4` (never a hand-made tunnel) | `rosie target add <cell> --olp-url http://127.0.0.1:18894`, then `--target <cell>`; robot actions stay `off` unless the owner says otherwise |

```bash
./dev-stack.sh start|status|logs|stop                                   # catalog, NATS, rt-sim, rt-control, dense daemon, planner, OLP, UI
cd weld_planner/v1 && pixi run -e motion motion-serve                   # :8796 (CUDA for welds)
export OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN=http://127.0.0.1:8796
bash offline-programming/v1/start-offline-programming.sh                 # :8794 API + Tesseract + seam worker
cd offline-programming/v1/ui && npm run dev                              # :5189, the browser app
```

## Traps `rosie doctor` checks (each has cost real debugging time)

- **Seam worker env**: rails needs `pixi install -e default` in
  `weld_planner/v1`; without it OLP health is "ok" and every rails action fails.
- **Planning origin**: OLP needs `OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN`;
  the launcher does not set it (doctor probes it: `motion_origin_unavailable`).
- **CPU-only planner**: plans MoveJ, dwell and Home only; welds and MoveL need CUDA.
- **No program store** (local OLP without `--program-store`): save to a file
  (`rosie program save --file …`).
- **Stale engine bundle** after a pull: `cli/v1/install.sh`.
- **No Chrome / snapshot page**: `rosie view` cannot render.

Others: `:8794` in a browser redirects to `:5189` on purpose (never `npm run
build` in the UI); browser programs live per origin in IndexedDB (never
suggest clearing site data — share through the cell store or a file); WSL
needs `TMPDIR` on a Linux filesystem; stop local jog before acquiring dense
execution; Python goes through pixi (`pixi run -e motion …`), never pip.
