# `robot_description`

One directory is one robot. Everything intrinsic to a model lives in it and is
shared by every cell that mounts one; how a particular cell deviates from it
lives on that cell, in `machine_planning_calibration.json`, and never here. The contract both halves
answer to is `docs/ROBOT-DESCRIPTION-CONTRACT.md` -- read it before changing
anything in here, because these files are hashed and a plan made against the
old hash is refused by the cell that holds the new one.

```
robot_description/
  robots/<model>/            one directory per robot (rosie_1400_v3, rosie_1420_v1,
                             and the benches of bare motors)
    robot_description_manifest.json            the MANIFEST: which files this robot is made of
    robot.urdf               kinematics, meshes, design limits, the torch
    robot.srdf               the planning group and its named states
    config.json              what URDF cannot hold, for the planner: acceleration,
                             the axis roles, the reset pose, the rate policy, the
                             torch convention, and the caps a cell's kinematic
                             corrections must stay inside
    rtcore_definition.json   what rt-core needs and the planner never reads: gear
                             ratios, motor encoder counts, encoder batteries,
                             signs, drive profiles, which URDF joint each drive
                             turns, Home policy (rt-core/config/templates/robot.json)
    spheres.json             the collision spheres, with the hash of the meshes and
                             URDF they were fitted from
    meshes/                  the link geometry the URDF names, plus the planner's
                             own fixture bodies
    provenance/              how the above was made: the CAD export, the scripts
                             that built the bundle, the measurement reports. NOT
                             registered, so nothing in here is part of the identity
                             and editing it never makes a plan stale
  go/                        module rosieos/robotdesc: the contract in Go (identity,
                             URDF scan, the store, the machine_planning_calibration.json rules), shared by
                             rt-core's rtctl and the OLP; `go run ./cmd/identity`
                             prints a description's identity
  tools/manifest.py          writes and checks manifests
```

A bench of bare motors (`bench_one_motor_1to1`, `bench_nine_motors_1to1`) is a
description of `robot_description_manifest.json` and `rtcore_definition.json` alone: rt-core runs
it, the OLP neither lists nor plans for it.

## Identity

`robot_description_manifest.json` lists the files that are the robot; the identity is a
sha256 over that manifest and each file it names, rendered `sha256:<hex>`.
rt-core's `rtctl compile` computes it from this directory and pins it in the
machine config, the cell serves the files and the identity in Describe, the
OLP computes it from its own copy and from what the cell lists, and a plan
whose header names another one is refused before it is loaded.

So a re-exported STEP, a new measurement report or a fixed typo in a README
does not invalidate a plan, and adding a mesh the robot actually uses does --
once it is registered, which is the deliberate act:

```bash
python3 robot_description/tools/manifest.py --check         # what CI runs
python3 robot_description/tools/manifest.py rosie_1420_v1   # rewrite one
```

## Adding a model

Make the directory under `robots/`, put `robot.urdf` and its meshes in it,
write `config.json` (copy a sibling's and replace every number with this
robot's -- it may not repeat a URDF value), write `rtcore_definition.json`
(every number with its `_source`; `rt-core/config/templates/robot.json` is the
annotated template), fit spheres with `weld_planner/v1/tools/fit_spheres.py`,
then run the manifest tool. The planner finds a model by directory name: that
name is the cell id a program carries and the `model_id` every file in here
states. A machine config in `rt-core/config/machines/` then pins the directory
and the identity the manifest tool (or `go run ./go/cmd/identity`) prints.
