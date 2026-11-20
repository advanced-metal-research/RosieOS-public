# Configuration layers

All runtime configuration lives in this tree. Use **drive config**, **robot
config**, **machine config**, and **cell config** to distinguish the layers.
The JSON keys `profile` and `drive_profile`, and existing CLI switches such as
`--profiles` and `--manifest`, retain their wire/API spelling.

| Layer | Directory | Composition |
|---|---|---|
| Drive config | `drives/` | A machine axis selects a basename with `profile`; a robot selects `drive_profile`. |
| Robot description | `../robot_description/robots/<model_id>/` (repository root; a runtime package carries a copy) | A machine pins `robot.robot_description{path, sha256}`: the directory and its manifest identity `robot_description_sha256`. The description's `rtcore_definition.json` maps its joints to mechanics; its `robot.urdf` is the geometry. |
| Machine config | `machines/`, `machines/bench/`, `machines/simulation/` | Binds drives and optional robot mechanics to slave positions, timing, limits, I/O and host runtime policy. |
| Cell config | `cells/` | Binds hosts, package selection, pair identity, ordered application axes and a repository-relative `machine_config` with its compiled pin. |

## Drive config

`templates/drive.json` describes CiA402 drive fields and the alternate terminal
descriptor fields. Drive configs carry EtherCAT identity, PDO layout, scaling
semantics, startup SDO schemas/defaults, Home transactions, brake/jog policy,
coordinate evidence and collision bounds. A drive name resolves only in
`config/drives/`; there is no fallback to the former directory.
`<name>.tuning.json` files are drive tuning catalogs, not profiles: matched to
a profile by EtherCAT vendor/product, they declare the servo tuning objects a
machine may set per axis in `startup` or read with `observe`
([tuning a joint](../docs/drives/a6ec/README.md#tuning-a-joint-2026-10-07)).
A catalog may also declare the drive's `parameter_storage` object (A6-EC
C13.10), which a machine verifies with `verify_parameter_storage` and uses to
write per-axis `startup_trial` keys to drive RAM only
([CONTRACTS](../docs/CONTRACTS.md#startup-write-on-change-and-ram-only-trials-2026-10-09-offline-not-deployed)).

A physical drive descriptor does not define robot joint acceleration. For every
URDF-mapped robot axis, the compiler takes travel and maximum velocity from the
robot's `robot.urdf`, and both trajectory and jog acceleration from
`config.json` → `planning.joints.<urdf_joint>.acceleration_rad_s2`. It requires
that canonical acceleration; it never substitutes a drive-profile value.
Machine declarations of `min`, `max`, `max_velocity`, `jog_acceleration` or an
axis `max_acceleration` cannot override a mapped robot. This applies equally to
an assembled robot, its simulation and a motor bench assigned that robot.

Synthetic flat fixtures and bare-motor descriptions with no robot URDF have no
robot motion parameters to inherit. Their explicit fixture limits are separate
from robot configuration; selecting a robot cannot carry those fixture limits
into it. Standalone flat fixtures can declare an axis `max_acceleration`.
For a mapped robot, the drive's per-axis speed cap is also derived from that
URDF velocity through the robot's gear ratio and motor encoder scale. Neither
`drive_speed_limit_motor_rpm` nor legacy `max_motor_rpm` may redeclare it on the
machine. Native PDO integer ranges and supported drive hardware constraints
still reject an unrepresentable robot configuration rather than changing its
declared limits.

An I/O terminal is also a named drive config: `dio_terminal.json` describes its
identity and wire layout. `cell-io/simulation.json` remains a **complete simulation
machine fixture**, including axes and an `io` block; it is not another terminal
descriptor. `io.profile` selects the terminal from `drives/`. Its bit assignments,
independent readbacks and output expiry belong to the machine. Torch qualification
is explicitly false. Schema 2 I/O profiles add `planet_iecs_1116_do.json` (a
mailbox-free 16-output EtherCAT terminal) and `revpi_dio.json` (the RevPi DIO in
the piControl process image, offsets in the machine `io` block); an output with
no input module may declare `readback: none_unverified`
([CELL_IO.md](../docs/CELL_IO.md)).

## Robot description

A robot lives in `robot_description/robots/<model_id>/` at the repository root
(docs/ROBOT-DESCRIPTION-CONTRACT.md): `robot.urdf`, meshes, the planner's
`config.json` and `spheres.json`, and `rtcore_definition.json`, which
`templates/robot.json` describes: exact gear ratios, encoder counts, direction,
battery facts, travel, Home policy and URDF joint mapping. `robot_description_manifest.json`
registers the files the robot is made of and the identity is a SHA-256 over
them (`go run rosieos/robotdesc/cmd/identity <directory>`). A bench of motors
is a description with no `robot.urdf`. Numeric model values require their
existing `_source` or `_unverified` siblings.

A machine pins the directory and its identity with `robot.robot_description{path,
sha256}`, may pin this machine's planning calibration with
`robot.machine_planning_calibration{path, sha256, source}` (a file beside the machine
config, under `config/machines/`), and maps each axis through `robot_joint`. Robot-bound axes
cannot redeclare flat-axis mechanics. The compiled configuration snapshots the
description whole, plus the machine's `machine_planning_calibration.json`, and the adapter serves it.
Home calibration offsets belong to a separate calibration record; they are not
factory robot facts.

## Machine config

`templates/machine.json` documents flat simulation axes and the alternate
robot-bound axis fields, plus bus policy, timing, telemetry and terminal I/O.
Choose one axis representation. A brake-disabling override requires an explicit
bench hold policy and reason. `single_turn_absolute` requires the qualified A6
sampled-evidence branch; a generic simulation drive uses `multi_turn`.

Templates are annotated class definitions, not deployment approval. They show
mutually exclusive fields together; remove `_doc` siblings and select the desired
branch before compiling. `<required>` marks a value the example cannot choose.
`TestConfigurationTemplateFieldCoverage` checks compiler reader fields against
the templates. `TestConfigurationTemplateSimulation` compiles the flat machine
and CiA402 drive branches; `TestConfigurationTemplateRobot` pins and compiles the
external-joint robot branch. `TestCellConfigTemplate` validates the cell's actual
simulation binding. These checks perform no hardware access.

From `rt-core/`, compile a concrete simulation machine:

```sh
make control
build/rtctl compile --config config/machines/simulation/simulation.json --out build/config
# build/config/8e40d3e2f4161294b77d7d56a309350cf6955c0be3eba1e6bf5419491c61b292
```

## Cell config and pins

`templates/cell.json` describes the native compatibility binding. The complete
deployment/host inventory examples live in `cells/`; their additional deployment
fields are consumed by the deployment CLI. The moved Go package is
`rosieos/rt-core/config/cells`. Its validator recompiles every native binding and
refuses a mismatched digest, axis count, pair or listener. Existing compiler
warnings remain findings, even where a regression test expects them.

To obtain a pin without deploying or contacting a cell:

```sh
compiled=$(build/rtctl compile --config config/machines/bench/one-motor-1to1-lcdjog.json --out build/config)
basename "$compiled"
# baee417704fac7a859300b5b08ed39b27ca4fffde938434655525aa4f52dc90b
go test -count=1 ./config/cells -run TestRepositoryCompatibility
```

Put the printed digest in the selected node's `rt_core.configuration_sha256`;
its `machine_config` is `rt-core/config/machines/bench/one-motor-1to1-lcdjog.json`.
Update every consumer pin when the approved configuration changes.

The existing immutable `configuration_sha256` composes machine content and
resolved drive content; a robot-bound machine also carries the exact robot pin,
which in turn pins URDF resources. The compiler retains its existing normalization
of bus defaults and notes. Moving byte-identical configs does not alter this
identity. A robot or resource change requires updating its parent pin, recompiling
and updating every cell/application pin. A cell's deployment-only fields do not
enter the compiler digest: they select and pin it. Changing that identity boundary
would invalidate existing pins and is outside this filesystem migration.

## Relocation digest check

The current LA bench uses `machines/bench/rosie1420-six-motors.json`, with
joint position limits, speed ratings, coordinate evidence mode and Home policy
inherited from its pinned Rosie 1420 definition. The former nine-axis bench
machine configuration is retired; current cell aliases bind the six-motor bench.
The historical relocation measurements below remain evidence of that migration,
not current deployment pins. Current pins live in `cells/*.json`.

Compiled before and after P8-26 from the same bytes; both columns are identical.

| Machine config | Before | After |
|---|---|---|
| `machines/a6ec-bench.example.json` | `ae560eb81624aeb60a131c3072517a874e3b72026e7581ac2cdaa7baaf6ab10e` | `ae560eb81624aeb60a131c3072517a874e3b72026e7581ac2cdaa7baaf6ab10e` |
| `machines/bench/nine-motors-1to1-ffA.json` | `6b35a500dc4dc68b5fbc01edf1b2e61b7df21c9799c17a86747c606ed05be3ec` | `6b35a500dc4dc68b5fbc01edf1b2e61b7df21c9799c17a86747c606ed05be3ec` |
| `machines/bench/nine-motors-1to1-ffB.json` | `d0694bca23176b1ec58525034548fccf2c94349b84f10823404beebaea345e01` | `d0694bca23176b1ec58525034548fccf2c94349b84f10823404beebaea345e01` |
| `machines/bench/nine-motors-1to1.json` | `1912df7d55763abc8949d31f826c4a86f9ab93b1915ef6b347f5c599aaad3dbe` | `1912df7d55763abc8949d31f826c4a86f9ab93b1915ef6b347f5c599aaad3dbe` |
| `machines/bench/one-motor-1to1-lcdjog.json` | `a576bc117f7dac844d5b773d743e0426c54d5414486ac0fed0a484964d3ecbfe` | `a576bc117f7dac844d5b773d743e0426c54d5414486ac0fed0a484964d3ecbfe` |
| `machines/bench/one-motor-1to1.json` | `1ec357d237e66c446492d2fcdacdd0b63b6e5f6ae0f7f1cb9246f6841dd8fd89` | `1ec357d237e66c446492d2fcdacdd0b63b6e5f6ae0f7f1cb9246f6841dd8fd89` |
| `machines/bench/rosie1420-six-motors.json` | `f51524f0d0cbc0c1cf2fb923c26b5ba276de2c1a09f0a6cc21a4424f0214db40` | `f51524f0d0cbc0c1cf2fb923c26b5ba276de2c1a09f0a6cc21a4424f0214db40` |
| Retired nine-axis bench snapshot | `e7c9015db26100bd3d0895bc8ba58749569c9420bfd2fb480236b947f78fff5b` | `e7c9015db26100bd3d0895bc8ba58749569c9420bfd2fb480236b947f78fff5b` |
| `machines/simulation/simulation-home.json` | `9d34c4ab5614d50479b7aef03730bd4abe5c05830b5141d46df0feebb65b93b0` | `9d34c4ab5614d50479b7aef03730bd4abe5c05830b5141d46df0feebb65b93b0` |
| `machines/simulation/simulation-linear.json` | `72cbd9d018fdca2b111528c96234b226c7f1f204a3433f58c76f5dab88b82931` | `72cbd9d018fdca2b111528c96234b226c7f1f204a3433f58c76f5dab88b82931` |
| `machines/simulation/simulation-program.json` | `835cbc8cd3dc24c47d386756415919a773eb1f31be73a08d5dae4038c32bb8e2` | `835cbc8cd3dc24c47d386756415919a773eb1f31be73a08d5dae4038c32bb8e2` |
| `machines/simulation/simulation-scalars.json` | `d6e61a144f610947adcbe1e717b5f9efd2e7922f62a523e5345ff939fcc2996e` | `d6e61a144f610947adcbe1e717b5f9efd2e7922f62a523e5345ff939fcc2996e` |
| `machines/simulation/simulation.json` | `8e40d3e2f4161294b77d7d56a309350cf6955c0be3eba1e6bf5419491c61b292` | `8e40d3e2f4161294b77d7d56a309350cf6955c0be3eba1e6bf5419491c61b292` |
| `cell-io/simulation.json` | `eb4560eb333a619a521f9468fddbaa21d7cad0a22c17ebf1aa2e3b2af85fe42a` | `eb4560eb333a619a521f9468fddbaa21d7cad0a22c17ebf1aa2e3b2af85fe42a` |

The cell serves its own robot definition: `rtctl compile` carries the definition,
URDF and mesh bytes into the compiled digest directory, and `rt-control` loads
that directory through `--compiled-config` or `ROSIE_RT_COMPILED_CONFIG` to expose
them through `/v1/describe` and `/v1/resources/<sha256>`. Clients can observe a
reachable cell and fetch its model with `rtctl resources fetch <sha> --out <dir>`;
the repository pin is the approval to arm, not the source from which a client
loads the deployed model. A redeployment publishes its new snapshot when the
adapter restarts, while `cell_configuration_mismatch` still prevents arming an
unapproved configuration. Flat configurations without a robot binding report
no robot model; simulation definitions remain repository-owned inputs.
