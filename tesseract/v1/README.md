# Tesseract v1

Tesseract v1 is the no-ROS Tesseract and TrajOpt worker: a pixi Python
environment run from this directory. The default demo uses the PyPI
`tesseract-robotics-nanobind` bindings to call `TrajOptMotionPlanner` directly
against the local `rosie_1400_v3` URDF/SRDF assets. OLP and the Cartesian
motion server call its collision workers (`python/collision_check.py`,
`python/persistent_collision_check.py`) directly.

Robot v4 planning defaults to `robot_description/robots/rosie_1400_v3/robot.urdf` with the same
DAE meshes used for rendering and collision geometry. The worker materializes a
temporary absolute-mesh URDF before `Environment.init()` so Tesseract can load
the v3 mesh-collision model without a ROS package index.

From `tesseract/v1`:

```sh
pixi install
pixi run python python/ur10_demo.py demo
pixi run python python/ur10_demo.py demo --path wrist-sweep --json exports/wrist_sweep.json
pixi run python python/ur10_demo.py demo --path harmonious --trajectory-method dense-csp --tcp-speed 0.01 --out exports/rosie_1400_v3_dense_csp.json
pixi run self-test
```

## Cartesian planner API

`cartesian-plan` (`python/cartesian_plan.py`) is the generic JSON API. It does
not know about light-box installations, active TCP policies, or UI artifact paths. Callers pass
URDF/SRDF paths, joint names, a start seed, target points, and optional tip
offset metadata:

```sh
pixi run cartesian-plan --spec /tmp/cartesian-plan.json --out /tmp/cartesian-plan-result.json
```

The request schema is `tesseract-v1.cartesian-plan.v1`. The result schema is
`tesseract-v1.cartesian-plan-result.v1`, with an `urdf-viewer-trajectory-v1`
`trajectory[]` payload and summary diagnostics such as `success`,
`max_tracking_error_m`, `moving_joint_count`, and `tool_path_length_m`.

The current backend seeds Cartesian targets with numeric IK and runs TrajOpt
joint-space smoothing over those seeds. It is a planner/diagnostic output, not a
drive-ready program. Robot-specific consumers must still decide how to preview,
hash, and execute motion safely.

## Path presets

`demo` accepts named start/goal presets with `--path`:

```sh
# Compact v3 reach against the mesh-backed rosie_1400_v3 planning URDF.
pixi run python python/ur10_demo.py demo --path reach

# Wrist-heavy six-axis motion, written anywhere using --json <file>.
pixi run python python/ur10_demo.py demo --path wrist-sweep --steps 7 --json /tmp/wrist_sweep.json

# Taller six-axis arc.
pixi run python python/ur10_demo.py demo --path vertical-arc --json exports/vertical_arc.json

# v3 harmonious/coordinated motion: J7 turns from -pi to +pi while the six arm
# joints follow the v3 seed path.
pixi run python python/ur10_demo.py demo --path harmonious --steps 13 --json exports/rosie_1400_v3_path.json

# Dense CSP comparison: resample the TrajOpt path into timed position rows.
pixi run python python/ur10_demo.py demo --path harmonious --steps 17 --trajectory-method dense-csp --tcp-speed 0.01 --out exports/rosie_1400_v3_dense_csp.json

# Spline/time comparison: cubic joint interpolation with time, velocity, and acceleration.
pixi run python python/ur10_demo.py demo --path harmonious --steps 17 --trajectory-method spline-time --out exports/rosie_1400_v3_spline_time.json


```

`--json` without a path prints the full JSON payload to stdout. `--json <file>`
or `--out <file>` writes the trajectory JSON to that location and keeps the
compact human table on stdout.

Notes:

- The pixi workspace pins Python 3.12 because the verified
  `tesseract-robotics-nanobind` wheels include Linux x86_64 and Jetson
  `linux-aarch64` support.
- Collision costs and constraints are enabled for the default
  `rosie_1400_v3` path, and the result metadata reports
  `collision_prevention_model: rosie_1400_v3_mesh_urdf`.
- Keep `--tcp-speed 0.01` when regenerating the canonical dense-CSP export;
  its regression test checks every sample against the velocity limits declared
  by the v3 URDF.
- TrajOpt support for the seven-axis coordinated path is verified by the worker:
  it builds a single `JointWaypoint` containing `turntable_joint` plus the six
  UR-10 joints, and `TrajOptMotionPlanner.checkRequest()` and `solve()` succeed
  with all seven axes moving in the `harmonious` preset. The harmonious preset
  sweeps the turntable from `-pi` to `+pi` and seeds those joint waypoints from
  a small IK loop so `tool0` tracks a TCP target directly above the turntable's
  rotating 12 o'clock marker. The JSON includes `track_point_xyz_m`,
  `target_xyz_m`, and `tracking_error_m` per waypoint.
- Rosie 1400 v3 uses the mesh-backed GradientOS URDF/DAEs. The source URDF
  keeps package mesh paths for Steam Deck packaging; the Python worker
  materializes a temporary absolute-mesh URDF before `Environment.init()` so
  Tesseract can load the same visual and collision geometry without a ROS
  package index.
## Output shape

The current JSON payload is a planner/viewer handoff, not a drive-ready servo
stream. It contains:

- `joint_names`: ordered joints, including the external `turntable_joint`.
- `trajectory[]`: one row per waypoint.
- `positions_rad` and `positions_deg`: the planned joint positions.
- `time_s`: currently `0.0` for every row because this wrapper has not run
  Tesseract time parameterization yet.
- `tool0_xyz_m`, `tool0_matrix`, `target_xyz_m`, `track_point_xyz_m`, and
  `tracking_error_m`: viewer diagnostics for the TCP tracking demo.

The schema has room for future `velocities_rad_s`, `accelerations_rad_s2`,
`torques_nm`, and `torques_raw`, and the URDF viewer path panel will surface
those if present. TrajOpt supports velocity, acceleration, and jerk smoothing
terms, and Tesseract waypoints can carry optional velocity, acceleration, effort,
and time values. This wrapper currently extracts positions and time only.

`--trajectory-method` lets the same TrajOpt result be exported for viewer-side
comparison:

- `waypoints`: raw TrajOpt waypoint positions.
- `dense-csp`: fixed-period CSP-style target positions with estimated
  velocities and accelerations.
- `spline-time`: cubic Hermite joint interpolation with timestamps and
  derivative fields.
- `cartesian-track`: regenerates the harmonious turntable samples from the
  Cartesian weld target and solves IK at each sample.

For welding curves, the next drive-ready output should be a time-parameterized
trajectory such as:

- JSON/CSV: `time_s`, joint positions, joint velocities, joint accelerations,
  TCP pose, TCP speed, path progress, and blend/process metadata.
- ROS-style `JointTrajectory`: positions plus optional velocities,
  accelerations, effort, and `time_from_start`.
- EtherCAT CSP samples: target position counts at the RT cycle rate, with
  optional velocity/torque feedforward where the drive PDO map supports it.

## Servo mode notes

For coordinated welding motion, CSP is the default target mode: the planner or
RT controller owns the multi-axis path, streams synchronized target positions,
and the drive closes its local position/velocity/current loops. CSV is useful
for velocity jogging or a controller that intentionally owns the outer position
loop. CST is for torque/force/compliance experiments and should stay behind a
separate safety and tuning plan before it is used around weld path geometry.
