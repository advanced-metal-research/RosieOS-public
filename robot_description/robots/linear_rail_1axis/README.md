# linear_rail_1axis: a single-axis ball-screw rail on RosieOS

This folder defines a one-axis linear rail (a carriage on a ball screw, driven
by one A6-EC servo drive) for RosieOS's realtime controller, `rt-core`. It needs
no code changes: the rail is described as configuration only.

Read this file top to bottom once. Sections 1-3 explain the idea, section 4
lists the files, sections 5-9 are the steps to adapt, deploy and commission it.

Status (2026-10-12): the robot definition and machine config compile with
rt-core's configuration compiler (`rtctl compile`, no warnings), and the cell
manifest skeleton passes `manifest validate` and the repository's cell tests. Nothing has run on
hardware yet. Every number marked `_unverified` in the JSON is a design value
that must be measured on the rail.

---------------------------------------------------------------------

## 1. The idea in plain terms

- The motor's encoder counts **131,072 ticks per motor turn**.
- The A6-EC drive has its own gear-ratio setting: "**N motor turns = 1 turn
  of the output shaft**". The drive then reports position as an angle of that
  output shaft, 0 to 2π radians (0-360°).
- On a ball screw, one motor turn moves the carriage by the screw **lead**
  (mm per turn). Set **N = the number of motor turns across the whole rail**
  and the whole rail becomes **one** output turn. Every point on the rail then
  has its own unique angle: a number line from one end to the other.
- RosieOS writes the ratio as **motor turns : output turns**.
  Example: 10 mm lead, 1000 mm rail → 100 motor turns → ratio **100 : 1**.
  This is the same ratio a commissioned Rosie 1430 arm uses on five of its
  six joints, so the drive setting is a proven one.
- rt-core works in **radians of that virtual turn**. Positions, limits,
  speeds and tolerances in the config are all radians. Convert with

  ```
  k  = rail_length_mm / (2 * pi)      # mm per radian
  mm = rad * k
  rad = mm / k
  ```

  Example (1000 mm rail): 1 rad = 159.155 mm, 1 mm = 0.0062832 rad.

Two things that are easy to get wrong:

1. **The gear ratio only changes the scale of the number, not what the
   encoder knows.** What lets the rail know where it is after a power cut is
   the **encoder battery** (a battery box on the encoder cable that keeps
   counting turns while the power is off). It is required: without it the
   compiler refuses any ratio other than 1:1, and the drive would forget which
   turn it was on.
2. **You never count encoder turns yourself.** rt-core reads the drive's
   battery-backed turn count and produces one continuous position.

## 2. What this setup does and does not do

Does, with no code changes:
- arm/disarm, jog, Home (drive homing, HM35), joint moves, travel limits,
  following-error and collision checks, Home restored after a power cycle.

Does not:
- **Show millimetres natively.** rt-core has a linear axis type, but its
  robot-definition compiler only builds rotary axes and its machine compiler
  refuses linear axes on the A6-EC (the drive's feedback wraps every output
  turn). Convert radians to mm with `k` above.
- **Plan programs in the offline-programming app (OLP) or the weld planner.**
  Those need a URDF (robot geometry). This rail has none.

## 3. What you need

Hardware:
- One **A6-EC** EtherCAT servo drive and motor with the standard **17-bit
  absolute encoder** (131,072 counts per motor turn).
- An **encoder battery box** on the encoder cable. Without it this setup
  cannot work.
- A **servo host**: a Linux machine with a PREEMPT_RT kernel and the IgH
  EtherCAT master, wired to the drive on its own Ethernet port, for example a
  Revolution Pi.
- A **relay host** that the deploy commands use to build, hold releases and
  reach the servo host, for example a Jetson.
- The rail's numbers: **screw lead** (mm per motor turn) and **usable rail
  length** (mm).

On your Mac (where you run every command), from the repository root:

```bash
export PATH="$HOME/.rosie/bin:$HOME/.rosie/go-1.26.2/bin:$HOME/.rosie/node-22.13.1/bin:$PATH"
```

Read these once: `CLAUDE.md` (repository rules, especially Home and
deploying), `docs/DEPLOY_AND_TEST_WORKFLOW.md` (the deploy guide) and
`steamdeck/real/v4/README.md`, section "Adding a new robot cell".

## 4. The files in this setup

| File | What it is |
|---|---|
| `robot_description/robots/linear_rail_1axis/robot_description_manifest.json` | Lists the files that make up this robot description. |
| `robot_description/robots/linear_rail_1axis/rtcore_definition.json` | **The robot definition**: one joint `J1`, kind `external`, no URDF, gear ratio, encoder, battery, direction and mechanical travel. |
| `robot_description/robots/linear_rail_1axis/README.md` | This file. Not part of the description's hash. |
| `rt-core/config/machines/linear-rail-1axis.json` | **The machine config**: which robot description (pinned by hash), the drive's bus position, limits, safety checks, speed cap and realtime settings. |
| `rt-core/config/cells/linear-rail-1axis.json` | **The cell manifest skeleton**: the hosts and the compiled configuration's hash. Placeholder hosts; section 7 makes it deployable. |

All of them are filled in for an **example rail: 10 mm lead, 1000 mm long,
ratio 100:1, top speed 250 mm/s**. Replace the numbers with yours (section 5).

Rules the configuration compiler enforces:
- Every number about the robot needs a sibling note, `<field>_source`
  (where it came from) or `<field>_unverified` (not yet measured). A number
  with neither is refused. Keep these notes honest: update them when you
  measure something.
- The robot description is pinned by hash in the machine config, and the
  compiled configuration is pinned by hash in the cell manifest. **Any** edit,
  including a note, changes the hash. Section 6 shows how to update both.

## 5. Put in your rail's numbers

Inputs:

| Symbol | Meaning |
|---|---|
| `lead` | screw lead, mm per motor turn |
| `R` | usable rail length in mm: the length one virtual turn covers |
| `v` | top speed, mm/s |
| `a` | jog acceleration, mm/s² |

Derived:

```
k            = R / (2 * pi)                    mm per radian
ratio        = R / lead                        motor turns per virtual turn
counts_per_mm = 131072 * numerator / denominator / R
```

Gear ratio rules (the compiler refuses anything else):
- `numerator` and `denominator` are whole numbers, 1..65535 each.
- `131072 * numerator / denominator` must be a whole number. If the rail is
  not a whole number of turns, use a power-of-two denominator
  (37.5 turns → `75 : 2`).
- The ratio itself must be between **10 and 16383**. Below 10 the virtual
  turn is too short for how often rt-core samples the drive's absolute
  position; above 16383 the counts no longer fit a 32-bit integer.

What to change, and where (keep 5 mm clearance from each end stop):

| Field | File | Formula | Example value |
|---|---|---|---|
| `gear_ratio.numerator` / `denominator` | definition | `R / lead`, reduced | 100 / 1 |
| `mechanical_travel.min` | definition | `5 / k` | 0.031416 |
| `mechanical_travel.max` | definition | `(R - 5) / k` (span must stay below 2π) | 6.251769 |
| `sign` | definition | `+1` or `-1`, measured (section 8) | 1 |
| `limits.min` / `limits.max` | machine | same as mechanical travel (may only be narrower) | 0.031416 / 6.251769 |
| `limits.max_velocity` | machine | `v / k` (rad/s) | 1.5707963 |
| `drive_speed_limit_motor_rpm` | machine | `60 * v / lead` (warning above 3000) | 1500 |
| `limits.jog_acceleration` | machine | `a / k` (rad/s²) | 3.1415927 |
| `limits.following_error` | machine | `1 / k` (1 mm) | 0.0062832 |
| `drive_supervision.disarm_motion_tolerance` | machine | same as following_error | 0.0062832 |
| `limits.max_target_lead` | machine | `0.5 / k` (0.5 mm, warning only) | 0.0031416 |
| `limits.completion_tolerance` | machine | `0.05 / k` (0.05 mm) | 0.00031416 |
| `collision_watchdog.following_error_counts_max` | machine | `counts_per_mm * mm allowed` | 12000 (0.92 mm) |
| `collision_watchdog.torque_abs_max_raw` | machine | measured torque plus margin (0.1 % units) | 500 (placeholder) |
| `axes[0].slave_position` | machine | the drive's position on the EtherCAT bus, from 0 | 0 (placeholder) |
| `runtime.cpu` / `runtime.priority` | machine | the servo host's realtime CPU and priority | 0 / 80 |

Also update the `_source` / `_unverified` notes and the mm figures written in
them, so the next reader sees your rail, not the example.

**Watch the units.** One radian is a lot of rail (159 mm in the example).
Never copy a radian value from the arm configs; always convert from mm.

## 6. Update the two hashes and check the files

Run from the repository root, in this order, after **every** edit.

```bash
# a) Robot description hash -> machine config robot.robot_description.sha256
(cd robot_description/go && go run ./cmd/identity ../robots/linear_rail_1axis)
#    prints: sha256:<64 hex> ../robots/linear_rail_1axis
#    paste "sha256:<64 hex>" into rt-core/config/machines/linear-rail-1axis.json

# b) Compile the machine config -> cell manifest configuration_sha256
rm -rf /tmp/rail-compile
(cd rt-core && go run ./tools/rtctl/cmd/rtctl compile \
    --config config/machines/linear-rail-1axis.json --out /tmp/rail-compile)
#    prints /tmp/rail-compile/<64 hex>; must exit 0 with no refusal
#    paste <64 hex> into rt-core/config/cells/linear-rail-1axis.json
#    -> nodes[realtime-core].rt_core.configuration_sha256
#    (example config as committed: 8a4e85f4b2e610d60102b8822551061e5fc8c1ea8071f3a4e37e3c7392a7a6c2)

# c) Check the compiled result is what you meant
grep -E '^(limits|tracking|startup_sdos|collision_watchdog|drive_supervision)=' \
    /tmp/rail-compile/*/axes.conf
#    startup_sdos must show tracking mode ...|8192|8|4 and your ratio as
#    ...|8208|25|<numerator> and ...|8208|26|<denominator>

# d) Check the cell manifest
(cd steamdeck/real/v4 && go run . manifest validate \
    --manifest rt-core/config/cells/linear-rail-1axis.json)
```

If the compiler says `robot_description_mismatch ... update the pin`, you
edited the description without redoing step a.

## 7. Make the cell deployable, then deploy

The cell manifest, `rt-core/config/cells/linear-rail-1axis.json`, tells the
deploy commands which hosts run the rail and which compiled configuration
they must run. As committed it is a skeleton: one placeholder host
(`192.0.2.10`, a reserved documentation address) and `changeme` for every
value you must replace. Its `configuration_sha256` is the example config's
hash; redo it with section 6 after any config edit.

The repository's cell test
(`(cd rt-core && go test -count=1 ./config/cells/)`) compiles every cell
manifest and requires a new one to compile with no compiler findings; keep it
passing.

The skeleton is not deployable yet. The deploy commands also need the
relay/servo connections and a `cell_deploy` block. Use a deployed arm cell's
manifest (`rt-core/config/cells/<cell>.json`) as the template.

1. `connections`: the **relay** (`"transport": "ssh"`, `host`, `ssh_user`,
   `working_directory`) and the **servo host** (`"transport": "ssh-via"`,
   `"via_peer": "<relay name>"`, its address on the cell link, `ssh_user`).
2. `nodes`: the `realtime-core` node's `peer`/`host`/`ssh_host`/
   `remote_user` point at the servo host; `rt_core.remote_listen` is the
   address the controller listens on; `rt_core.ethercat_device_group` is the
   servo host's EtherCAT device group.
3. `cell_deploy`: copy that cell's block and change the paths and users for your
   relay (`deployments_dir`, `deployment_prefix`, `scratch_dir`, `dump_dir`,
   `home`, `path`, `igh_deps_dir`, `build_jobs`,
   `dump_copy_rate_bytes_per_s`, `olp`). Every field is explained in
   `steamdeck/real/v4/README.md`, "Adding a new robot cell".
4. Run `manifest validate` again (section 6 step d).

Deploy with the repository commands only (never by hand on a host). They
refuse uncommitted configuration, so commit first.

```bash
git add robot_description/robots/linear_rail_1axis \
        rt-core/config/machines/linear-rail-1axis.json \
        rt-core/config/cells/linear-rail-1axis.json
git commit -m "linear rail: <your rail>"

cd steamdeck/real/v4
M=rt-core/config/cells/linear-rail-1axis.json
go run . realtime-core --help                            # every command
go run . realtime-core release --commit <sha> --plan --manifest $M   # what it would do
go run . realtime-core release --commit <sha> --manifest $M          # build, install, start, verify
go run . realtime-core verify --manifest $M                          # read-only check later
```

Every command takes `--dry-run`. For a configuration-only change later
(new limits, same software), `realtime-core config --manifest $M` deploys just
the configuration. Full guide, timings and troubleshooting:
`docs/DEPLOY_AND_TEST_WORKFLOW.md`.

## 8. Commission on the rail

Do these in order, at low speed, with the hardware E-stop within reach.
Operating the controller (arm, jog, Home) is described in
`rt-core/docs/APPLICATION_HANDOFF.md` and `rt-core/docs/APPLICATION_API.md`.

1. **Bus position.** On the servo host, `ethercat slaves` lists the drives;
   set `slave_position` to the rail drive's index.
2. **Direction.** Jog slowly in +. The carriage must move away from the home
   end. If it moves toward it, set `"sign": -1` in the definition, then redo
   section 6 and redeploy.
3. **Scale.** Jog a known distance and measure it with a tape or dial gauge.
   It must equal `rad * k`. If not, the lead or ratio is wrong.
4. **Home.** Drive the carriage to its mechanical reference and have an
   operator run drive homing (HM35) there. HM35 makes the current position
   Home. **Never re-home to clear a fault.**
5. **Power cycle.** Switch the drive off and on. Home must be restored
   without re-homing. If rt-core cannot verify the position, it refuses
   rather than guessing.
6. **Full length and the wrap point.** The drive's raw count wraps once per
   rail length, at about the middle of the rail. rt-core tracks across it.
   Do one full-length move, then a power-cycle restore with the carriage past
   mid-rail.
7. **Measure, then set the safety numbers.** Record torque and following
   error at your working speeds and set `torque_abs_max_raw` and
   `following_error_counts_max` from them (section 9). Change each
   `_unverified` note to a `_source` note saying what you measured and when.

## 9. Things that will bite

- **The collision watchdog trips early at speed.** Its following-error limit
  ignores speed, and a stock A6-EC trails its command by about 27 ms (speed
  feedforward is off by default). The example's 12,000 counts (0.92 mm) is
  reached at about **34 mm/s**. Before running faster, measure the lag on
  this rail or raise the limit toward the speed-scaled bound (13.5 mm =
  176,947 counts at 250 mm/s with a 1000 mm rail). The main following-error
  fault already allows for lag (`tracking_lag_allowance`, 50 ms).
- **Speed.** Above 3000 motor rpm the compiler warns. Also check the ball
  screw's critical speed for its length.
- **Brake.** The `a6ec` drive profile assumes a motor brake. Confirm what the
  motor has. A vertical rail must have a real, tested brake.
- **No battery, no rail.** Without the encoder battery the compiler refuses
  this ratio, and the turn count is lost on power-off.
- **Configuration lives in the repository.** Never edit a config on a host.
  Change the files here, redo section 6, commit, deploy.
- **The servo host runs nothing heavy.** Builds and tests on the cell's hosts
  only through the deploy commands or `realtime-core lock`.

## 10. Where things come from

- Counts-to-position maths: `rt-core/include/coordinate_model.hpp`
  (`from_encoder`).
- Robot definition rules (drive-side ratio, battery, rotary-only):
  `rt-core/tools/rtctl/robot_definition.go`.
- Machine compiler rules (evidence modes, linear-axis refusals):
  `rt-core/tools/rtctl/compiler.go`.
- A6-EC drive profile (ratio objects 0x2010:19/1A, tracking mode 0x2000:08):
  `rt-core/config/drives/a6ec.json`.
- Single-motor template this was copied from:
  `robot_description/robots/bench_one_motor_1to1/` and
  `rt-core/config/machines/bench/one-motor-1to1.json`.
- Arm reference (100:1 joints, battery, multi_turn): a commissioned Rosie 1430
  cell's robot description under `robot_description/robots/` and its machine
  config under `rt-core/config/machines/`.
