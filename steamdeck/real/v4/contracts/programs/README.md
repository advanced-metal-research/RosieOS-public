> **Nine-motor bench, dedicated automatic test bench:** the bench is an unattended test
> resource. No human authorization, approval, confirmation, label, or per-run
> permission exists. Invoke automatic tests directly; machine control state is
> coordination and evidence, not human permission.

# Robot V4 program contracts

These schemas define the shared boundary between Offline Programming,
SteamDeck, Motion Server, and the Motion Server-owned program catalog.

- `robot.v4.program.v1` is editable and revisioned.
- `robot.v4.program_catalog.v1` is the Motion Server-owned catalog envelope.
- `robot.v4.accepted-plan.v1` is immutable and is never edited as a program.
- `robot.v4.program.v2` is the pre-compile weld program and the successor of
  `robot.v4.program.v1`: one row per node (weld, approach, transit, retract,
  home, dwell, io), holding intent only. It is produced by either Offline
  Programming's classic flow or the weld planner's rails flow and lowered by
  a planner. Motion nodes read in order must alternate weld and freespace as
  a single line; the planner never inserts a freespace move, a missing one is
  a validation error. Event nodes (io, dwell) sit on the boundaries of that
  line. A classic weld carries its baked per-point TCP orientation as
  authority; a rails weld carries the weld planner's seam; a hand-placed weld
  (`manual_pose_wp`, the OLP's Waypoint welds tool) carries its torch poses
  and a C0/C1/C2 continuity per pose, in the workpiece frame, with the path
  and orientation interpolation pinned by the schema; `shape: "arc"` makes
  it the circle through exactly three poses instead. The OLP lowers a
  hand-placed weld to a seam before a plan request is packed, and the weld
  planner refuses one that was not lowered (D28). Cell and tool are
  named by id and pinned by sha256, never inlined. `weld_parameters` is
  reserved and must be empty. Unlike the three schemas above it is **closed**:
  a field it does not name is refused, which is the structural half of the
  no-solution rule; the semantic half (the alternation, a deny list of solved
  quantities, curve invariants) is restated by hand in
  `program_v2_test.go`, `offline-programming/v1/ui/src/contracts/robot-v4-program-v2.ts`
  and `weld_planner/v1/python/weldplan/program_v2.py`, all asserting the same
  fixtures. Its seam `$defs` are vendored from the weld planner schema and
  held identical by parity tests in all three languages. Nothing produces or
  consumes it yet; program.v1 remains the executor input until the executors
  move. This directory is the canonical shared contract location.

Canonical joints are always J1 through J9 in radians. Cartesian positions are
metres. A program edit advances `program_revision`, retains the prior accepted
revision in `base_program_revision` until the catalog accepts the update, and
invalidates every prior plan.

The v1 schemas allow additional properties for forward compatibility. Clients
must preserve unknown properties when they load and resave a program.
`robot.v4.program.v2` is closed, and says so above.

## Accepted-plan custody gap

Lifecycle state is `not_evaluated`: the saved `robot.v4.accepted-plan.v1`
artifact does not retain pair revision, pair snapshot hash, generated-URDF
hash, activation epoch, or Home epoch. Runtime gates may check those values,
but the retained accepted-plan artifact alone cannot prove that it is still
compatible with current execution authority.
