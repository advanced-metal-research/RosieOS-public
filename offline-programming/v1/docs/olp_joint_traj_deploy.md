# Deploying an OLP joint trajectory to the nine-motor bench

A rails program planned in Offline Programming, played on the real robot through
`joint_trajectory_daemon` on the bench relay. This is the operator's runbook; the design
lives in `offline-programming/v1/README.md` ("The joint daemon as a deployment
target") and the file format in `motion-server/joint-trajectory/v1/docs/DENSE-JOINT-TRAJECTORY-V1.md`.

> **Cell facts drift.** Addresses, limits and the controller config were last
> confirmed on 2026-09-07; RTCore mode and homes on 2026-09-12 (live, all nine
> axes homed). Every step below that depends on the cell's state says how to
> check it. Check; do not trust this page.

## How the pieces fit

```text
browser :5189 ── OLP server :8794 ── weld planner motion :8796
                      │                 Plan writes the dense trajectory to
                      │                 ~/.cache/rosieos-olp/weld-planner-dense
                      │
                      │  Load to robot: fetch by digest, upload + preload
                      ├──── TCP 100.64.0.13:8797 ── joint_trajectory_daemon (the bench relay)
                      │                                   │ 100 Hz setpoints
                      │                                   └─ velocity_controller 127.0.0.1:49014
                      │                                        │ UDP, link-local IPv6
                      │                                        └─ RTCore :49004 (<servo-host>) ── EtherCAT drives
                      │
                      │  play / pause / stop / go_home, status at 20 Hz
                      ├──── NATS nats://100.64.0.13:4222  robot/v4/robot.<cell>-dense.*
                      │
                      │  ARM: under the v1 controller lease that cartesiand grants
                      └──── NATS  robot/v4/robot.<cell>.command  (RTCore bridge)
```

Two robot cells share one arm, on purpose:

| Name | What it is |
|---|---|
| `<cell>` | the v1 cell: cartesiand (Motion Server v1) and the RTCore bridge. It grants the controller lease ARM needs. It plans nothing here. |
| `<cell>-dense` | the joint daemon's cell, on its own command subject so cartesiand never sees its commands. |

| Thing | Value |
|---|---|
| SSH | `ssh user@<relay>.example.ts.net` (Tailscale SSH, shared account `user`) |
| Jetson tailnet IP | `100.64.0.13` |
| Cell NATS | `nats://100.64.0.13:4222` |
| Daemon TCP ingest | `100.64.0.13:8797` |
| RTCore host | `<servo-host>`, status on `robot/v4/rtcore.<servo-host>.status` |
| Speed limits the daemon enforces | J1-J3 0.09 rad/s, J4-J9 0.2 rad/s (min of the pair and RTCore runtime limits) |

Nothing converts the trajectory on the way. The weld planner writes the daemon's
format at Plan time, **Simulate** plays those bytes, and **Load to robot** sends the
same bytes.

## 0. Before you start

- **The cell is shared.** Others work on it too. Agree who is driving it before
  anything is armed.
- **The e-stop is within reach.** The OLP's red STOP is a software stop (see step 7),
  not the hardware e-stop.
- **Know whether a welder is connected.** The trajectory raises torch-on during every
  weld.
- **The part is where the OLP put it.** Placement is measured from the middle of
  `positioner_table_a`'s top face (D27). An offset or turn in the OLP that is not on
  the real table is a weld in the wrong place.

## 1. The program is ready (workstation)

1. A **rails** program, authored with the **Fitted torch** set to
   `Rosie 1400, link_6 gun`. Welds searched with the Ø16 nozzle bend into corners the
   arm cannot follow; delete and re-click them.
2. **Plan** succeeds. A weld that has no path is refused by name, for example
   `seam_0006: no path the arm can follow (1.65% of poses feasible, joint limit ...)`.
   There is nothing to load until every weld plans.
3. **Simulate** (▶) and watch it. The plan panel says which program the trajectory
   came from; it must not be marked stale.
4. Plans drive **J1-J7** and hold J8 and J9 at 0
   (`robot_description/robots/rosie_1400_v3/config.json`). J7 is the
   positioner: it has to be homed and armable on the cell. On `rosie_1420_v1`
   they drive J1-J6 and there is no positioner at all -- see 1b.

## 1b. If the program is for `rosie_1420_v1`

Everything above and below is written for `rosie_1400_v3`, the cell this runbook
was first walked on. A 1420 program differs in five places, and four of them are
silent if you skip them.

| | `rosie_1400_v3` | `rosie_1420_v1` |
|---|---|---|
| Pair that must be active | `rosie_1400_v3` | `rosie_1420_v1` |
| Axes the plan drives | J1-J7, J8/J9 held | **J1-J6**; there is no positioner |
| `mapped_axis_mask` | `0x1ff` | `0x3f` |
| Placement is measured from | the middle of `positioner_table_a`'s top face | **the cell origin** (this cell has no table) |
| Fitted torch | `Rosie 1400, link_6 gun` | the same gun; both models bolt `mig_torch_v1` to the same place on `link_6` |

The placement row is the one that has already cost time. The OLP measures the
workpiece offset from the cell's OWN work frame, and on a cell with no positioner
that frame is `world` -- so the numbers in the placement panel are world
coordinates, not table-relative ones. The panel says which it is underneath the
fields; read it rather than assuming.

**The part still stands on something the model only guesses at.**
`fixture_placeholder` is a 0.5 m square box 1.0 m tall at world [1, 1, 0], asked
for on 2026-09-13 because nobody has measured the real fixture. The planner
avoids it; it is not the thing in the cell. Before anything is armed, confirm
that the real fixture is inside that box, or the plan is screened against
furniture that is not there.

It is also NOT drawn in the OLP viewer: it lives in the descriptor rather than
in `robot.urdf`, because that URDF must stay byte-identical to the pair-owner's
factory copy (`local_sim_stack_contract_test.go` asserts it). So an operator
placing a part cannot see what holds it.

## 2. Check the cell (read-only)

This subscribes to RTCore's status on the cell's NATS and prints the last complete
message. It sends nothing.

```bash
ssh user@<relay>.example.ts.net python3 - <<'EOF'
import json, socket, time
subject = "robot/v4/rtcore.<servo-host>.status"
s = socket.create_connection(("127.0.0.1", 4222), timeout=3)
s.sendall(f"SUB {subject} 1\r\n".encode())
s.settimeout(1.5)
buf, start = b"", time.time()
while time.time() - start < 2.5:
    try:
        buf += s.recv(1 << 20)
    except socket.timeout:
        break
last = None
while (i := buf.find(b"MSG " + subject.encode())) >= 0:
    e = buf.find(b"\r\n", i)
    n = int(buf[i:e].split()[-1])
    if len(buf) < e + 2 + n:
        break  # a truncated tail: keep the last complete message
    last = json.loads(buf[e + 2:e + 2 + n])
    buf = buf[e + 2 + n:]
if last is None:
    raise SystemExit("no RTCore status message within 2.5 s")
st = last["status"]
for key in ("rtcore_mode", "ethercat_output_enabled", "motion_output_enabled",
            "pair_config_robot_model_id", "runtime_mode_sender_id"):
    print(f"{key:28} {st.get(key)}")
print(f"{'home_valid_axis_mask':28} {hex(st['home_valid_axis_mask'])}")
EOF
```

What you want:

| Field | Want | If not |
|---|---|---|
| `pair_config_robot_model_id` | the model your program was planned for | stop: the wrong pair is active |
| `rtcore_mode` | `live` for real motors | step 3 |
| `ethercat_output_enabled`, `motion_output_enabled` | `True` | step 3 |
| `home_valid_axis_mask` | every mapped axis: `0x1ff` (J1-J9) on the 1400, `0x3f` (J1-J6) on the 1420 | step 4 |

`motion_output_enabled: True` is not armed. Nothing is energized until ARM.

## 3. RTCore sim or live

RTCore always runs able to drive the motors; a runtime **mode** decides whether it
does. Switch it with the deploy tool, from the repo root:

```bash
rosie robot v4 deploy --manifest rt-core/config/cells/<cell>.json --system-state live --dry-run
rosie robot v4 deploy --manifest rt-core/config/cells/<cell>.json --system-state live
```

- It **disarms first**, then sets the mode. No build, no file write, no restart, and
  the cell stays **unarmed**.
- The NATS URL and cell come from the manifest. `--dry-run` prints the subject, URL
  and both payloads: check they are the bench's before running it for real.
- It sends as `rosie:robot-v4-system-state`, which is on the cell's live-mode
  allowlist. An ad-hoc controller (for example an old per-host mode script) is
  refused: `live mode command rejected: sender "..." is not in the manifest live-mode
  allowlist`.
- Use this repo's `rosie` CLI. A checkout of the predecessor monorepo has an older
  CLI than the cell's binaries.

Back to simulation: the same command with `--system-state sim`.

## 4. Homes

`home_valid_axis_mask` must cover every mapped axis -- `0x1ff` on the 1400, `0x3f`
on the 1420. The daemon refuses GO HOME while any mapped axis is unhomed, and the
executor refuses a plan whose axes are not all homed; a plan that moves J7 needs J7.

Going live does not necessarily bring homes back: on 2026-09-13 the bench went
`sim -> live` with the 1420 pair and `home_valid_axis_mask` stayed `0x0`.

If it is not: **stop here and home through the cell's own procedure** (the cell owner,
the pendant). Homing is not part of this runbook. HM35 makes an axis's *current*
encoder position its home without moving anything, so it is only correct with the
axis physically at its zero. For J7 especially: a positioner homed away from its true
zero offsets every planned table angle.

Going live keeps J1-J6's homes (the drives restore them). Homes set while in sim do
not survive the switch.

## 5. Start the joint daemon (Jetson)

```bash
ssh user@<relay>.example.ts.net '~/.rosie/motion-server/joint-trajectory/v1/jetson-dense.sh start'
ssh user@<relay>.example.ts.net '~/.rosie/motion-server/joint-trajectory/v1/jetson-dense.sh status'
# also: logs, stop
```

It runs `velocity_controller` (127.0.0.1:49014, to RTCore over link-local IPv6) and
`joint_trajectory_daemon` (ingest 100.64.0.13:8797, cell `<cell>-dense`, status
streamed at 20 Hz on `robot/v4/robot.<cell>-dense.status`).

- **The launcher lives only on the Jetson**, not in this repo.
- It is built from sources rsynced to `~/rosie-src/motion-server/joint-trajectory/v1`, into
  `~/.rosie/motion-server/joint-trajectory/v1/bin`. Rebuild the daemon and the controller **together**
  whenever `motion-server/joint-trajectory/v1` has changed since the last sync
  (`git log --since=<last sync> -- motion-server/joint-trajectory/v1`); their setpoint wire must match.
- The controller runs with the bench horizon config set on 2026-09-06
  (`/home/user/.rosie/repairs/bench-horizon-20260907/horizon-0.05.json`). Keep it.

From the workstation, both must answer:

```bash
nc -vz 100.64.0.13 8797
nc -vz 100.64.0.13 4222
```

## 6. Point the workstation OLP at the cell

```bash
cd ~/projects/RosieOS
DEV_STACK_DENSE_INGEST=100.64.0.13:8797 \
DEV_STACK_DENSE_NATS_URL=nats://100.64.0.13:4222 \
DEV_STACK_DENSE_ROBOT_CELL=<cell>-dense \
DEV_STACK_DENSE_RTCORE_HOST=<servo-host> \
DEV_STACK_DENSE_BRIDGE_CELL=<cell> \
./dev-stack.sh restart olp
```

- This restarts **only** the OLP server. The local sim cell keeps running; the OLP just
  stops talking to it.
- `DEV_STACK_DENSE_BRIDGE_CELL=<cell>` is what makes ARM take the real cell's
  controller lease. Without it ARM goes down the simulator's path.
- **The variables only apply to that command.** Any later `restart olp` or `start`
  without them points the OLP back at the local simulator, silently.
- `./dev-stack.sh status` prints its "olp target" line from *its own* environment:
  without the five variables it shows `dev-cell` even when the OLP is on the bench.

Reload the browser. **Robot > Deployment target** lists `Joint daemon ·
<cell>-dense`.

## 7. Run

In the OLP, with the rails program open:

1. **Robot > Deployment target** → `Joint daemon · <cell>-dense`. The toolbar's
   transport becomes the robot's, and the motor table fills from servo feedback. Set
   the pose toggle to **REAL** to watch the real arm in the viewer.
2. **Load to robot.** The server fetches the planned trajectory by digest from the
   motion server, then uploads and preloads it on the daemon. The program is locked
   for editing while a trajectory is staged, playing or paused.
3. **ARM.** Takes the `<cell>` controller lease, waits ~1.5 s for the bridge to see
   it, arms, and renews every 250 ms while armed. **In live mode this energizes the
   drives.**
4. **GO HOME.** Interpolates from the pose the servos report to the reset pose, under
   the cell's speed limit. Refused while a program is playing or paused, or while a
   mapped axis is unhomed.
5. **▶** to play, **❚❚** to pause, **■** to stop.

**STOP** (red, always enabled once a target is selected) is a software stop: the
daemon's stop is resent until acknowledged, the torch goes off, and the lease is
released. It is not the e-stop.

The browser sends a heartbeat every second. After 5 s of silence (a closed tab, a
sleeping laptop) the server stops the daemon (`stop_reason: ui_heartbeat_lost`). A
lapsed controller lease (NATS or the OLP gone) makes the bridge disarm the cell by
itself.

## 8. Finish

1. **■** or **STOP**, then **DISARM**.
2. If the cell should go back to simulation: step 3 with `--system-state sim`.
3. If nobody else needs the daemon:
   `ssh user@<relay>.example.ts.net '~/.rosie/motion-server/joint-trajectory/v1/jetson-dense.sh stop'`.
4. Workstation back to the local simulator: `./dev-stack.sh restart olp`, with no
   variables.

## Troubleshooting

| You see | It means | Do |
|---|---|---|
| `N of M weld(s) could not be planned: seam_...: no path the arm can follow (...)` | those welds have no reachable path | delete or re-search them with the gun preset; step 1 |
| `the welds planned but nothing connects them: ...` | every weld planned, the moves between them did not | re-plan; look at the connecting-move reason it gives |
| `Connection refused` on 8797 | the joint daemon is not running | step 5 |
| no `Joint daemon · <cell>-dense` target | the OLP was not restarted with the variables, or the page was not reloaded | step 6 |
| ARM refused, or the cell disarms by itself | `DEV_STACK_DENSE_BRIDGE_CELL` unset, or the controller lease lapsed | step 6; check the OLP can reach the cell's NATS |
| `live mode command rejected: sender ... not in the manifest live-mode allowlist` | a mode switch sent by anything but the deploy tool | step 3 |
| the daemon refuses `unmapped_axes` | the plan moves an axis the active pair does not map | check the pair on the cell; re-plan |
| the daemon refuses `speed_limit_exceeded` | a planned joint speed is above RTCore's authority limit | the cell's limits changed: put them in the cell's `machine_planning_calibration.json` (docs/ROBOT-DESCRIPTION-CONTRACT.md) and re-plan |
| GO HOME refused | a mapped axis is unhomed, or a program is playing or paused | step 4, or stop the program |
| the viewer's real pose updates slowly (~3 Hz) | the daemon's status stream is off | restart the daemon with the launcher (it publishes the stream) |
| the controller cannot reach RTCore after a network change | the RevPi firewall admits UDP 49004 only from the Jetson's link-local IPv6 address; IPv4 is dropped without a reply | keep the controller's RTCore target on `[fe80::bc87:c3e8:8027:8539%<interface>]:49004` |
