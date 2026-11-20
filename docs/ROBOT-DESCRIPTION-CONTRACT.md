# Robot Description Contract

Status: step 1 (identity in status and header, refusal on mismatch) and step 2
(`robot_description/robots/<model>/`, one directory per robot, with `config.json`,
spheres and torch folded in and the planner's profile computed from them) are
implemented, on the local dev cell only. Step 3 -- the runtime loading the
description as data, dropping the embedded and generated URDFs and the receipt
chain -- is `robot/v4`'s and is not done here. The proposal is dated
2026-09-13.

A plan is made against a robot. Until this contract, nothing on the cell could
say which one: the joint daemon checked that a plan's axes were mapped and
homed, and played a trajectory computed against a different URDF, other
limits, or another tool0 without noticing. On a bench cell a URDF revision
met an old pair record and the cell crash-looped instead.

The contract is the ROBOT CELL IDENTITY: two hashes, compared exactly.

| | What | Identity |
|---|---|---|
| **description** | the files `robot_description/robots/<model>/robot_description_manifest.json` registers: `robot.urdf`, `robot.srdf`, `config.json`, `spheres.json`, `rtcore_definition.json`, meshes | sha256 over that manifest and those files |
| **cell** | `machine_planning_calibration.json`: how this cell deviates from the model, in the planner's terms | sha256 of the file's bytes |

Home calibration, EtherCAT wiring and drive settings are outside both. A
re-homing or a rewiring never invalidates a plan; RTCore still checks them at
runtime (`home_valid`, SDO readback) exactly as before.

The OLP Workspace presents available robot-setting differences and revision
dates when a program's saved description differs from the current one. The
operator can choose **Accept current settings** without typing or copying a
hash. This records the program's acceptance of that version; it does not
re-plan, verify motion, or make a stale plan executable. Robot identities
remain exact hashes internally and are available under technical details.
Unavailable historical settings or revision dates are identified as such.
The program's pin or acceptance date is provenance for the program, not the
date on which the robot settings changed.

## Robot sha256

The description says which files it is made of, and the identity is computed
over exactly those. `robot_description_manifest.json` in the model directory is that manifest:

```json
{ "schema": "rosie.robot-manifest.v1",
  "model_id": "rosie_1420_v1",
  "files": ["config.json", "meshes/base.dae", "...", "robot.srdf", "robot.urdf",
            "spheres.json"] }
```

The identity is `"sha256:" + hex` of SHA-256 over a versioned big-endian
layout, `robot_description_manifest.json` included as an entry of its own:

```
"rosie.robot-description.identity.v1\x00"
u64 entry_count
per entry, sorted by path bytes, "robot_description_manifest.json" among them:
  u64 len(path); path (UTF-8, relative, '/' separated); sha256(file bytes)
```

Registering the manifest closes the obvious hole: dropping a file from `files`
is itself a change of identity. Paths are relative, `/`-separated, and may not
be absolute or contain `.` or `..`; a listed file that is missing, or that is
not a regular file, makes the identity INVALID rather than something else's.

URDF mesh URIs may retain a different package label from the model id, such as
`package://rosie_1430_v1/meshes/link_1.dae` in a byte-preserved Rosie 1430
description. Native compilation and resource serving resolve this only to the
same description's registered `meshes/link_1.dae`. They never search another
package, filesystem directory or network location. Empty/traversal components,
file/network schemes and encoded/query/fragment paths are refused.

**A file nobody registered does not count.** CAD exports, build scripts,
measurement reports and READMEs may sit in the directory -- they do sit in
both models' -- without making a plan stale. That is the whole reason this is a
manifest and not a hash of the directory: `git rev-parse HEAD:<dir>` would put
a re-exported STEP and a typo in a README into the robot's identity, and the
cell would refuse every plan made before it.

Adding a file the robot needs is a deliberate act: register it, and every plan
made against the old description is refused, which is correct -- the robot
changed. `robot_description/tools/manifest.py` writes and `--check`s the file,
and a test holds every model's manifest to what its URDF references.

## What is in the description

`robot_description/robots/<model>/`, one directory per robot model, shared by every
cell that mounts one:

| File | What |
|---|---|
| `robot.urdf` | kinematics, meshes, design limits (position AND a velocity RATING), torch and positioner |
| `meshes/` | the geometry the URDF names |
| `spheres.json` | the collision model the motion search collides against, with the `provenance` hashes of the `robot.urdf` and `meshes/` it was fitted from |
| `config.json` | `rosie.robot-config.v1`: what the URDF cannot hold, for the planner |
| `rtcore_definition.json` | `rosie.robot-definition.v1`: what rt-core needs of the robot and the planner never reads -- gear ratios, motor encoder counts, encoder batteries, signs, drive profiles, which URDF joint each drive turns, Home policy (`rt-core/config/templates/robot.json` documents every field) |

A bench of motors with no arm -- `bench_one_motor_1to1`, `bench_nine_motors_1to1` --
is a description of the manifest and `rtcore_definition.json` alone: it has an
identity and rt-core runs it, and the OLP neither lists nor plans for it.
`go run rosieos/robotdesc/cmd/identity robot_description/robots/<model>` prints a
description's identity; `robot_description/go` is the Go implementation of this
contract, shared by rt-core's `rtctl` and the OLP.

`config.json` never repeats a URDF value. It carries:

- `kinematic_correction_caps` — the largest per-joint origin correction a
  cell's `machine_planning_calibration.json` may carry, per component. **2 mm and 1 degree, proposed,
  not agreed** (see the open points).
- `planning` — the rate the planner may parametrize against, per joint, plus
  the acceleration the URDF has no field for, and a `speed_ceiling_rad_s` that
  caps every joint. A planning rate above the URDF's rating is refused: a
  description may lower the rate it plans at, never raise it.
- `frames` — which link everything is measured in (`base`), where the torch
  is (`tool`), and what a workpiece is fixtured to (`work`). A URDF has no
  word for the last one. A cell whose work frame is a point ON a link rather
  than the link's origin states `work_surface`, and the planner adds that
  point as a fixed child so everything that walks joints carries it.
- `axes` — which joints the WELD PLANNER searches and which it holds, and at
  what value. Nothing in a URDF distinguishes a station index from a
  positioner. Held is a statement about planning only: the axis stays mapped
  and homed, the dense plan still commands it (its bit is in the header's
  `axis_mask`, held at that value), and the motion server, the pendant and
  RTCore move it as they always did.
- `reset_pose` — every seam is planned from it and back to it.
- `torch` — the tool frame and the direction the electrode runs in it, with
  the evidence sentence it was read from.

**The planner's profile is computed, not stored.** `config.json` merged with
the target's `machine_planning_calibration.json` is what every stage reads
(`weld_motion_planner.robot_cell.robot.profile.for_cell`). The two velocities
it carries mean different things: `velocity_rad_s` is the planning rate from
`config.json`, `enforced_velocity_rad_s` is what the CELL clamps at -- from
`machine_planning_calibration.json`, falling back to the planning rate when the cell states none --
and the gap between them is the joint daemon's follower authority to close a
lag. This replaced three files the planner kept beside itself
(`data/motion/cells/<cell>.{profile,tool,spheres}.json`), which was a second
description of a robot living a directory away from the first.

## machine_planning_calibration.json (`rosie.machine-planning-calibration.v1`)

Lives on the cell, not in RosieOS git: it is per-unit configuration. RosieOS
owns its schema, parser and validation. The OLP fetches the bytes from the
target and hands them to the planner verbatim.

```jsonc
{
  "schema": "rosie.machine-planning-calibration.v1",
  "model_id": "rosie_1420_v1",          // must name the description
  "limits": {                           // optional; narrows only
    "speed_ceiling_rad_s": 0.15,        // optional; every revolute joint
    "joints": {                         // keyed by URDF joint name
      "J2": { "lower_rad": -1.5, "upper_rad": 1.2, "velocity_rad_s": 0.09 }
    },
    "held": { "J8": 0.0 }               // the planner holds it; jog still moves it
  },
  "kinematics": {                       // optional; joint-origin corrections
    "J2": { "xyz_m": [0.0004, 0.0, -0.0002], "rpy_rad": [0.0, 0.0012, 0.0] }
  }
}
```

**Bounded deviation.** Two places check, and they are the two that own a copy
of the file: the CELL, on load, so a `machine_planning_calibration.json` edited by hand never reaches
the drives; and the OLP, when it reads one off a target, so a plan is never
built on a deviation the cell will refuse. Nothing between them re-checks.
Either rejects a document that:

- has an unknown key anywhere, a schema other than `rosie.machine-planning-calibration.v1`, or a
  `model_id` that is not the description's;
- names a joint the URDF does not have, or limits / holds a joint that is not
  revolute;
- widens a limit: `lower_rad` below the URDF's lower, `upper_rad` above its
  upper, `velocity_rad_s` or `speed_ceiling_rad_s` above the URDF velocity (the
  ceiling against the fastest joint), or leaves `lower_rad >= upper_rad`;
- holds a joint at a value outside its (narrowed) limits. The planner also
  refuses to plan a held ARM joint -- its solver has six unknowns -- but that
  is a planning limit rather than a rule about the document, so it is raised
  where the plan is made;
- corrects a joint origin by more than the description's cap in any component:
  `|xyz_m[i]| <= caps.xyz_m`, `|rpy_rad[i]| <= caps.rpy_rad`. Caps come from
  the description's `config.json` (`kinematic_correction_caps`); a description
  without one has caps of zero, so any nonzero correction is refused.

**Applying it.** Limits replace the URDF's for planning (never exceeding them).
A kinematic correction composes on the right of the joint's URDF origin:
`T' = T_origin * Trans(xyz_m) * R(rpy_rad)`, with `R` the URDF fixed-axis
convention `Rz(y) Ry(p) Rx(r)`. Wire joint angles keep their meaning; the
corrected origins are what forward kinematics uses.

**No file.** A cell with no `machine_planning_calibration.json` serves, and hashes, the empty document
for its model, exactly these bytes:

```
{"schema":"rosie.machine-planning-calibration.v1","model_id":"<model_id>"}\n
```

A planner that cannot reach the target plans against the same bytes and says so
in the header (`machine_planning_calibration_source: "empty"`). If the cell has a real `machine_planning_calibration.json`, the
hashes differ and the plan is refused: re-fetch, re-plan.

## Where the identity travels

1. **rt-core** (`rt-core/`): the robot definition pins the description
   directory (`description: {path, robot_description_sha256}`) and the machine config
   pins its `machine_planning_calibration.json` (`robot.machine_planning_calibration: {path, sha256}`); `rtctl compile`
   recomputes both, judges the `machine_planning_calibration.json` by the rules above, and snapshots
   the manifest, every registered file and the `machine_planning_calibration.json` into the compiled
   resources. The adapter admits the set only when the identity over the
   served files equals the pin, lists it in Describe
   (`robot.resources`, by path and sha256) and serves each file under
   `GET /v1/resources/<sha256>`. Changing either pin changes
   `configuration_sha256`.
2. **OLP**: reads the machine's identity off the Describe listing, fetches
   the `machine_planning_calibration.json` bytes at pack time, keeps them with the program's cell
   settings, and compiles the planning context from the description at the
   advertised identity (docs/PLANNING-CONTEXT.md).
3. **Planner** (`weld_planner/v1`): validates `machine_planning_calibration.json` against the URDF,
   applies it to the cell descriptor, stamps `descriptor.robot`, and writes the
   identity into the dense trajectory header.
4. **Header** (`robot_cell`, see `DENSE-JOINT-TRAJECTORY-V1.md`):

   ```json
   "robot_cell": { "model_id": "rosie_1420_v1",
              "robot_description_sha256": "sha256:<64 hex>",
              "machine_planning_calibration_sha256": "sha256:<64 hex>",
              "machine_planning_calibration_source": "target",
              "machine_configuration_sha256": "sha256:<64 hex>" }
   ```

   `machine_configuration_sha256` is optional and a RECORD, not part of the
   fence: `"sha256:"` + the rt-core `configuration_sha256` of the machine the
   plan was made for, read off Describe at pack time and absent on a plan made
   offline. It covers both identities above (the machine config pins them)
   and also everything a plan does not depend on -- a CPU number, a bus
   timeout -- so Load never refuses over it; when it differs from the
   machine's it says so in the load result's `notes`.

5. **Refusal**: at Load, before the machine is acquired, the OLP compares the
   header's `robot_cell` with the machine's identity:

   | reason | when |
   |---|---|
   | `robot_cell_missing` | the header has no `robot_cell` block (a plan from before this contract) |
   | `robot_cell_unavailable` | the machine serves a description without its manifest or without a `machine_planning_calibration.json` (detail says which), or its adapter serves no compiled configuration at all (started without `--compiled-config`), which is indistinguishable from a flat machine and is refused rather than passed |
   | `robot_cell_mismatch` | model, tree or cell hash differ (detail names which, with both values) |

   A machine that binds no robot description at all (a flat-axis simulation
   machine) has nothing to compare with and passes the block unjudged; its
   status says so (`robot_cell.configured: false`).

## Wire shapes

**Dense-execution status** (`robot_cell`, present once the machine has been
described):

```json
"robot_cell": { "configured": true, "valid": true,
           "model_id": "rosie_1420_v1",
           "robot_description_sha256": "<40 hex>",
           "machine_planning_calibration_sha256": "sha256:<64 hex>",
           "machine_planning_calibration_present": false,
           "error": "" }
```

`configured` is false when the machine binds no robot description; `valid` is
false when it does, but serves no manifest or no `machine_planning_calibration.json` (`error` says
why). The hash fields are `""` when unknown.

**`GET .../dense-execution/cell`** answers

```json
{ "robot_cell": { ...the status object above... },
  "machine_planning_calibration_base64": "<the exact bytes the machine hashed>" }
```

or `409 robot_cell_unavailable` with the reason when the identity is not
valid.

**Who reads what.** The OLP is the one place that opens a description
directory and the one place that reads a cell's `machine_planning_calibration.json`
(`internal/robotconfig`, a store pointed at a root the server names -- a
workstation's checkout today, a cell's deployed copy under step 3). It loads the
description (manifest identity, the URDF's own joint limits, `config.json`'s
caps and rate policy), fetches the target's `machine_planning_calibration.json` when it packs a plan,
validates the deviation against the description, and hands on SETTLED NUMBERS.
Nothing downstream re-reads either file or re-applies a rule.

**OLP → seam worker** (`build_plan_request`): `cell_urdf_base64` and one
`cell_settings`:

```json
{ "identity": { "model_id": "rosie_1420_v1",
                "robot_description_sha256": "sha256:<64 hex>",
                "machine_planning_calibration_sha256": "sha256:<64 hex>",
                "machine_planning_calibration_source": "target" },
  "joint_limits": { "J2": { "lower_rad": -1.5, "upper_rad": 1.2,
                            "velocity_rad_s": 0.09 } },
  "joint_corrections": { "J3": { "xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0] } },
  "held": { "J8": 0.0 },
  "speed_ceiling_rad_s": 0.15 }
```

Every joint limit is complete -- what the cell left out comes back as the
URDF's own number -- and keys that do not apply are omitted. The worker writes
those numbers into the descriptor's joints, stamps `descriptor.robot`, and
judges nothing: whether a limit widens or a correction exceeds its cap was
settled before the request was built. No `cell_settings` at all is a request
from before this contract; the model is packed as authored and Load
refuses what it plans.

**OLP → seam worker**, beside the above: `robot_frames`, the description's
`frames` block. The worker resolves nothing by model name -- it used to keep a
table of which link each robot fixtures to, which is a second place to be
wrong.

**Client → OLP** (`GET /api/offline-programming/v1/robots`): the catalogue of
what can be planned for -- every model in the store with its `robot_description_sha256`,
its frames (including the work frame's position at the zero pose, walked out
of the URDF), its driven and held axes and its reset pose. A client picks from
this rather than from a list written into its own source; a description that
will not load is named in `problems` rather than hidden.

**UI → OLP** (`weld-plan` body): nothing about the cell. A browser cannot
state what the robot it is talking to has been calibrated to, so the server
fetches it from the target; the UI keeps the identity only to show it and to
tell the operator when what it is showing is stale.

Not to be confused with the `machine_planning_calibration.json` member of a `.weldplan` container,
which is the cell DESCRIPTOR (`amr-weld-planner-v1.cell.v1`). With this
contract that descriptor carries the applied deviation and a `robot` block
(the identity, plus the rates and holds a later stage reads -- never the
`machine_planning_calibration.json` document itself), so the container's pinned cell sha256 covers
both.

## Local dev cell

The local cell is an rt-core simulation machine. By default
(`simulation-program.json`) it is flat-axis and binds no robot, so it serves
no description: the OLP plans against the workstation's copy and the empty
document, and Load passes the plan's robot block unjudged
(`robot_cell.configured: false` in the status); Home, arm and jog work.
`DEV_STACK_MACHINE_CONFIG=rt-core/config/machines/simulation/simulation-rosie1400.json`
binds `rosie_1400_v3` instead: the cell then serves the description (verified
2026-09-20: Describe lists its 18 files, the OLP's `dense-execution/cell`
answers the identity and the empty document) and Load judges the plan's
robot block. That machine takes the A6-EC drive profile from the definition,
and the simulated bus answers none of the A6 Home objects, so Home is refused
and nothing arms or jogs on it yet; the robot-bound dev cell is opt-in until
the simulator can Home an A6 profile or a robot-bound machine may select a
simulation profile.

## Open points (from the proposal and the 09-13/14 review)

- Model id when the torch is part of the model (`rosie_1420_v1-torchB`?).
- Correction cap numbers. `config.json` now states ±2 mm / ±1° per component
  for both models, which is the proposal's own example and NOT an agreed
  number: it wants whoever runs the probe jig. A description without caps
  admits no correction at all.
- Exact match on both hashes to start; loosen only with a concrete case.
  A review suggested a canonical projection of planner-relevant fields for the
  cell identity; the file hash is used for now because `machine_planning_calibration.json` holds
  nothing but planner inputs.
- Base/workpiece frames and fixtures as `machine_planning_calibration.json` fields (from review);
  today the 1420 fixture placeholder is still a constant in the seam worker.
- Step 3: the runtime loads the description as data, drops the embedded
  factory URDF and its hash constant, the generated URDF, the snapshot,
  receipt and pointer chain, and derives a new pair revision when the
  description identity changes. `robot/v4`, reviewed against the `c482179` checkpoint.
