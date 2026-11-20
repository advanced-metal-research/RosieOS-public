# Configuration fields

Configuration files are organized by layer under [config/](../config/README.md): drive, robot, machine and cell configs.

## Bench brake override

Cell axes may declare `"brake_override": {"present": false, "reason": "..."}`
only in the **bench class**: an explicit `bus.unknown_slaves: "hold"` policy
with its required nonempty notes, plus `hold_on_robot_acknowledged: true` and
`hold_on_robot_note` when binding an assembled-robot definition. This uses the
compiler's existing bench policy rather than a filename, because `Compile`
accepts configuration bytes and packaged configurations may move directories.
Configurations under `config/machines/bench/` must still opt into that policy. All other
configurations are robot/non-bench class and refuse the field with a typed
`BrakeOverrideError` naming the field and class. Robot definition joints cannot
declare an override.

Only boolean `present: false` and a non-whitespace textual `reason` are allowed.
Unknown members, null, missing fields and `present: true` are refused. The
existing refusal of a direct cell-axis `brake` block on robot bindings remains.
Legacy flat-axis brake parsing is unchanged.

The override changes only the effective brake presence. Release/hold delays,
feedback and gravity policy, and the displacement tolerance retain their profile
validation. Gravity or feedback requirements still refuse an absent brake.
Native `present=false` performs no brake output command, release wait or hold
check. Axes without the override retain their existing brake and fault behavior.
Removing an override changes configuration and machine identities; recompile,
deploy matching identities, verify configuration and re-Home before motion.

The compiled per-axis machine record (`axes.conf` and `machine.identity`)
publishes `brake_override_reason_hex`: lowercase hexadecimal UTF-8 bytes of the
reason. It is absent on axes without an override and participates in machine
identity. Native parsing requires bench hold and `brake.present=false` whenever
this metadata is present; it adds no field to the cyclic brake configuration.
The native description and public Describe axis expose the decoded text as the
optional `brake_override_reason` string. Its presence means an explicit disabling
override, not a physically absent brake. Operator surfaces can label that axis
"brake override" and display the reason. Reason changes also change identities.

## Robot-bound bench joint limits

The pinned robot description's `robot.urdf` is the only source of joint travel
limits for a robot-bound bare-motor bench. The bench declares its physical
joint-to-slave mapping and deployment tracking/brake/bus settings; it inherits
joint lower/upper and velocity ratings, encoder mode, gearing and Home policy
from the robot description. It does not carry a second position envelope.

Cell `limits.min`/`limits.max` and robot-definition `mechanical_travel` are
refused for every URDF-backed robot joint, including narrower copied bounds.
Standalone motor definitions without a URDF joint retain their own travel limits.
URDF-backed joints in bench hold configurations also refuse
`coordinate_evidence_mode` overrides and inherit the robot's encoder policy.
The former `bench_position_limits_override` exception has been removed;
old configurations containing it are rejected. Update the robot's URDF and pin
its new description when its design limits change, then recompile and deploy
matching configuration identities. Runtime and planner admission still enforce
the resulting limits.

## Sampled coordinate motion bounds

The drive profile's `maximum_physical_speed_counts_per_second` remains the
inhibited acquisition/backdrive bound. Startup and explicit reference binding
retain the declared acquisition-duration check: the outward-rounded motion
allowance plus feedback error must be strictly less than half an output turn.

For battery-present A6 sampled evidence (vendor 4194304/product 1813), PDO
tracking also covers the existing admitted powered motion. Independent raw
multi-turn mailbox comparisons use this bound after binding. Its counts/s bound
is the greater of the inhibited bound and
`ceil(min(max_velocity * counts_per_radian, max_profile_vel_counts_per_s))`.
The counts scale applies software gearing once; the drive cap is the existing
compiled motor-RPM/native-counts cap. No additional robot speed limit or copied
URDF rating is configured. Neither motion admission nor output limits change.

The compiler rejects such an axis if this tracking speed over one configured
cycle, rounded upward and including feedback error, reaches half its feedback
period (`coordinate_tracking_allowance_exceeds_half_period`). Runtime checks
still use actual elapsed time and refuse ambiguous gaps. Already-bound raw
samples are compared with retained PDO history at the low-word request, using
the actual low-word acquisition interval; the half-period check is retained.
This does not claim a shorter guaranteed SDO duration or relax startup binding.
Battery-absent and other legacy evidence retain their existing bounds and
acquisition checks; this change does not qualify them for higher-speed tracking.

## Compiled cell model resources (P8-28)

`rtctl compile` snapshots the bound robot definition, URDF and referenced meshes
into `<configuration_sha256>/resources/<resource_sha256>`. `resources.json`
records robot identity, original resource names, byte counts, media types and
per-axis drive configuration digests. `configuration.identity` preserves the
canonical digest preimage; configurations with cell I/O also retain the base
preimage in `configuration.base.identity`. These artefact files do not change
the thirteen pinned configuration digests in `docs/CONTRACTS.md`.

A robot's `package://<model_id>/meshes/<file>` meshes are the description's own
registered files (`robot_description/robots/<model_id>/meshes/`); a runtime
package carries every description, so compilation reads them from the package
or from the repository beside the component. Missing, escaping or unsupported references refuse compilation; a running cell
requires only its compiled directory. Mesh content receives a byte digest in
that artefact; existing machine pins bind definition/URDF and drive content,
not independently approved mesh byte pins. Resource integrity does not replace
the configuration approval required for motion.

Pass that directory to `rt-control --compiled-config`, or use the
`ROSIE_RT_COMPILED_CONFIG` emitted by `rtctl control-env`. Identity, file bounds
and hashes are checked before serving; a supplied incomplete or mismatched
directory refuses startup. On deployment, restart the adapter with the new
directory. The public observe surface then describes and serves that snapshot
without opening repository files. An unbound flat-axis simulation has no robot
definition or URDF to serve; its robot block is null, preserving its exact pin.
