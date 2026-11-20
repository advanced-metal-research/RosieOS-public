# rosie_1400_v3

Simple URDF generated from the active Autodesk Fusion `full_cell_iges` document (v19).

This v3 model uses one visual DAE per visible top-level Fusion occurrence and mesh collisions:

- `floor`
- `link_1` through `link_5` (Fusion `l1`–`l5`)
- `link_6` — the Fusion `mig_torch_v1` welding torch body, which replaced the old `l6` wrist disk as the J6 child
- `end_effector` — the new Fusion `end_effector` TCP cone, fixed to `link_6`; `tool0` is coincident with it
- `h_frame`
- `positioner_table_a`
- `positioner_table_b`

Fusion exported each occurrence as STL in `provenance/fusion_part_export/meshes` via `rosie autodesk v1 export-model`. CadQuery measured those STLs in `provenance/fusion_part_export/cadquery_measurements.json`. The checked viewer assets are staged under `urdf/v1/ui/public/robot/rosie_1400_v3/meshes/*.dae`, and `robot.urdf` references them with `package://rosie_1400_v3/meshes/...`.

Joint origins are full parent-relative Fusion occurrence transforms (translation and rotation), so the URDF zero pose reproduces the exported CAD pose exactly. The exported document carries a small J1 yaw (~1.01°) and J2/J3/J5 pitches; these live in the fixed part of each joint origin.

## Regenerate from a fresh Fusion export

```powershell
.\rosie.ps1 autodesk v1 export-model --root= --parts= --out robot_description/robots/rosie_1400_v3/provenance/fusion_part_export --force --resume=false
.\rosie.ps1 cadquery v1 measure --fusion-manifest robot_description/robots/rosie_1400_v3/provenance/fusion_part_export/export_manifest.json
node robot_description\rosie_1400_v3\provenance\tools\build_bundle_from_fusion_export.mjs
node robot_description\rosie_1400_v3\provenance\tools\verify_bundle_against_fusion.mjs
```

`build_bundle_from_fusion_export.mjs` rebuilds `robot.urdf`, `meshes/*.dae`, `provenance/simple_parts_manifest.json`, and `provenance/ik_fk_origin_verification.json`, then stages the viewer copy. `verify_bundle_against_fusion.mjs` recomputes zero-pose FK from `robot.urdf` and asserts every link's world transform matches the Fusion `export_manifest.json` occurrence transform and that every mesh was CadQuery-measurable.

`provenance/ik_fk_origin_verification.json` verifies the URDF joint origin/RPY values against the Autodesk `export-model` metadata. IK is intentionally marked `not_solved_here` because this bundle demonstrates FK playback only.

## Table calibration overlay

`calibration.json` is the table-corner work-frame calibration record (schema `rosie.table-calibration.v1`). The Steam Deck calibrate chord (LT+RT+SELECT on the table-box program) makes the motion-server fit a least-squares rigid correction from the 4 edited corner points and apply it as an overlay on the `J9` joint origin in `robot.urdf` (and the staged viewer copy under `urdf/v1/ui/public`). It is a rigid work-frame correction — NOT per-joint DH calibration. `records[0]` is active, `records[1]` is the one-step revert target; an identity record (zero translation/rpy, as committed here) means uncalibrated, and applying it restores the factory `nominal_origin_*` attributes byte-for-byte. See `motion-server/v1/src/robot_v4_table_calibration.hpp`.

CAUTION: `build_bundle_from_fusion_export.mjs` rewrites `robot.urdf` from the Fusion export and therefore resets any applied calibration to the factory origin. After regenerating the bundle, re-apply the active record (`robot-v4-cartesiand --self-test table-calibration-apply`) or accept the identity baseline.

## config.json

What the URDF cannot hold, and never a URDF value repeated.

- **`planning.joints`** is a POLICY, not a rating: the rates the planner
  parametrizes against, under `robot.urdf`'s own rating (the drive cap through each gear ratio, below). The cell's enforced
  rate lives in that cell's `machine_planning_calibration.json`, and the gap between the two is the
  follower's authority to close a lag. The planning rates are 0.785398 rad/s
  (45 deg/s) on J1-J3 and the positioner J7-J9 and 1.570796 rad/s (90 deg/s)
  on the wrist J4-J6, with `speed_ceiling_rad_s` at the wrist rate (owner
  decision 2026-09-22: the previous 0.09/0.2 rad/s were the drives' max
  profile velocity read off the nine-motor bench on 2026-09-04, a bench number the
  planner then had to fit every weld under -- a 39 mm arc at 4 mm/s needed
  twice it and could not be planned). Both are under the ratings below. The
  acceleration (5/10/2.5 rad/s^2, each joint reaching its rate in about
  0.3 s) is a policy sized to the new rates and is unconfirmed against
  hardware; M5 only fits velocity, so T2 is what holds a trajectory under it.
- Every revolute joint's `<limit velocity>` is the drive cap through its gear
  ratio (owner decision 2026-09-21, P8-44). rt-core caps the A6 motors at
  `drive_speed_limit_motor_rpm` = 3000 motor rpm (the machine template's
  default and every machine config's value; the motors are rated 6000), so
  rad/s = 3000 / 60 * 2 * pi / ratio = 314.159265 / ratio, with the ratio
  from `rtcore_definition.json`: J1, J2, J3 (100:1) 3.14159265; J4 (18:1)
  17.4532925; J5 (27:2) 23.2710567; J6 (10:1) 31.4159265; J7, J8, J9 (100:1,
  owner direction 2026-09-21, definition revision 7) 3.14159265. Until
  2026-09-18 every joint carried 0.2 rad/s, an unmeasured design number, and
  PR 122 briefly carried 180 and 360 deg/s, also unmeasured. It is a RATING:
  the bound `rtctl compile` holds a machine's `max_velocity` under. What the
  planner plans at is `planning.joints` and what a cell enforces is its own
  `machine_planning_calibration.json`, both far below it.
- **`axes`**: J1-J7 searched by the weld planner; J8 and J9 held. Two bays on
  one H-frame -- J9 turns the frame so one bay faces the robot and the other
  the operator. The held values are both 0.0 as a PLACEHOLDER: J9 in
  particular decides which bay faces the robot, so a real program states it.
  HELD IS ABOUT PLANNING, not about the robot: J8 and J9 stay mapped and
  homed, the dense plan commands them at their held value, and the motion
  server, the pendant and RTCore drive them as they always did.
- **`reset_pose`** is the URDF's own home: every revolute joint carries
  `<calibration rising="0"/>`, so zero is the pose the pendant's home means.
- **`torch`**: `tool0`, electrode along +X. PCA over the gun mesh (2026-09-12)
  puts the tube's centreline 0.061 degrees off tool +X. `tool0` is the base of
  the contact tip, not its outlet: a 17 mm standoff holds `tool0` 17 mm off the
  work and the wire outlet 10.65 mm. How far off the work a weld runs is the
  program's `process.standoff_mm`, not this file's.
- **`kinematic_correction_caps`**: 2 mm and 1 degree per component. PROPOSED,
  an open point in `docs/ROBOT-DESCRIPTION-CONTRACT.md` for whoever runs the
  probe jig.

## spheres.json

The collision spheres, and the sha256 of the meshes they were fitted from.
Refit with `weld_planner/v1/tools/fit_spheres.py`; a mesh that changes without
a refit fails the provenance test.

## provenance/

The Fusion export this bundle was built from, the scripts that built it, and
the measurement reports. Nothing here is registered in `robot_description_manifest.json`, so
editing it never changes the robot's identity.
