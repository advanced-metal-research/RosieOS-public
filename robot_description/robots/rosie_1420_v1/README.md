# rosie_1420_v1

A six-axis arm on a pedestal, with the `mig_torch_v1` gun on `link_6`. No
positioner: a part is fixtured to the floor, so the redundancy at a weld point
is spin alone. The URDF, its meshes and `config.json` are the same shape as
`rosie_1400_v3`'s; the two share a gun and a mount.

`robot.urdf` is embedded byte for byte in the RTCore bridge and its sha256 is
pinned in the pair config (`robot/v4/realtime-core/natsbridge`). Changing it
desyncs the cell's home profile key until that copy is updated -- which is what
step 3 of `docs/ROBOT-DESCRIPTION-CONTRACT.md` exists to end.

## config.json

What the URDF cannot hold, and never a URDF value repeated.

- **`planning.joints`** is a POLICY, not a rating: the rates the planner
  parametrizes against, under `robot.urdf`'s own rating (the drive cap through each gear ratio, below). The cell's
  enforced rate lives in that cell's `machine_planning_calibration.json`, and the gap between the two
  is the follower's authority to close a lag. Owner update 2026-09-22 sets
  J1-J3 and J5 to 3 rad/s, J4 and J6 to 10 rad/s, with a 10 rad/s global
  planning ceiling. This replaces the inherited 0.09/0.2 rad/s policy and
  its 0.2 rad/s global cap. J5 has a 100:1 gearbox, so its planning rate
  remains below the URDF's 3.14159265 rad/s rating. The acceleration is
  copied from `rosie_1400_v3` and still unconfirmed.
- Every revolute joint's `<limit velocity>` is the drive cap through its gear
  ratio (owner decision 2026-09-21, P8-44). rt-core caps the A6 motors at
  `drive_speed_limit_motor_rpm` = 3000 motor rpm (the machine template's
  default and every machine config's value; the motors are rated 6000), so
  rad/s = 3000 / 60 * 2 * pi / ratio = 314.159265 / ratio, with the ratio
  from `rtcore_definition.json`: J1, J2, J3, J5 (100:1) 3.14159265; J4 (18:1)
  17.4532925; J6 (10:1) 31.4159265. Until 2026-09-18 every joint carried
  0.2 rad/s, an unmeasured design number, and PR 122 briefly carried 180 and
  360 deg/s, also unmeasured. It is a RATING: the bound `rtctl compile` holds
  a machine's `max_velocity` under. What the planner plans at is
  `planning.joints` and what a cell enforces is its own
  `machine_planning_calibration.json`; neither may exceed the rating.
- **`axes`**: J1-J6 driven, nothing held.
- **`reset_pose`** is all zeros, which every joint's `<limit>` admits.
- **`torch`**: `tool0`, electrode along +X. PCA over the gun mesh (2026-09-12)
  puts the tube's centreline 0.061 degrees off tool +X. `tool0` is the base of
  the contact tip, not its outlet: a 17 mm standoff holds `tool0` 17 mm off the
  work and the wire outlet 10.65 mm. How far off the work a weld runs is the
  program's `process.standoff_mm`, not this file's.
  The frame was wrong until 2026-09-12 -- `tool0` sat 55.7 mm off the gun, in
  free air. A pure translation, so it cancelled in every relative move and only
  the planner, which computes a TCP absolutely, noticed.
- **`kinematic_correction_caps`**: 2 mm and 1 degree per component. PROPOSED,
  an open point in the contract for whoever runs the probe jig.

## spheres.json

The collision spheres, and the sha256 of the meshes they were fitted from.
Refit with `weld_planner/v1/tools/fit_spheres.py`; a mesh that changes without
a refit fails the provenance test.

## provenance/

The Fusion export the bundle was built from and the scripts that built it.
Nothing here is registered in `robot_description_manifest.json`, so editing it never changes the
robot's identity. `tools/build_bundle_from_fusion_export.mjs` is stale: its
joint table still carries the pre-measurement limits, so re-running it would
overwrite the measured ones.
