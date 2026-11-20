# MuJoCo cell simulator v1

A physics stand-in for RTCore and the nine A6-EC drives. It binds RTCore's
UDP port, takes `robot-v4-cartesiand`'s jog stream, forms the CSP target
the way RTCore does every millisecond, runs the drives' position/speed
cascade on a MuJoCo model of the cell, and answers `ServoFeedbackV4` read
off the simulated encoders. Nothing above it changes: cartesiand, the OLP,
the Deck and natsbridge see an RTCore.

```
OLP / Deck  --UDP :49013-->  robot-v4-cartesiand  --CmdJogV2 (qd, 100 Hz)-->  cellsim
                                                  <--ServoFeedbackV4--------  (RTCore mirror, 1 kHz)
                                                                                  |  607A / 60FF
                                                                             A6-EC cascade x9
                                                                                  |  torque
                                                                               MuJoCo cell
```

## Run

```bash
cd mujoco-sim/v1 && pixi install          # mujoco + numpy + pycollada, ~4 s

# behind the OLP's local simulator (the launcher installs this env when asked):
ROSIEOS_OLP_PHYSICS=mujoco bash offline-programming/v1/start-offline-programming.sh

# standalone, RTCore's port, waiting for a pair owner's CmdJointConfigV2:
pixi run sim --status-output /run/robot-v4-rtcore/status.json \
             --telemetry-ring /dev/shm/robot-v4-rtcore-telemetry.ring --viewer

# standalone with an injected authority (what the OLP's cartesiand holds):
pixi run sim --udp-listen 127.0.0.1:49004 --authority-json authority.json
```

`pixi run test` runs the suite (wire ABI, RTCore rules on a kinematic
plant, drive cascade, model FK, and the whole process on its UDP port);
`pixi run model` builds the MJCF and checks its forward kinematics against
the URDF chain; `pixi run drive` steps one drive against a ramp.

## What is mirrored, and from where

| Layer | Source it mirrors | Module |
| --- | --- | --- |
| UDP ABI (JCFG 592, FDBK/MODE 56, SLIM 64, ARM 48/64/296, HM35/GHOM 288, jog 176/408, feedback 392) | `robot_v4_rtcore.cpp:264-526` static_asserts | `wire.py` |
| Dispatch by size then magic, validators, pair-binding byte-exact echo, JCFG home-profile key (FNV-1a) | `robot_v4_rtcore_udp_runtime.hpp:1332-1911`, `robot_v4_rtcore.cpp:1825-1907, 3164-3520` | `rtcore.py` |
| The cycle: stale lease → hold; slew 12.57 rad/s²; per-cycle step cap; target = ACTUAL + 20 ms lead, capped 0.35–0.80° per axis; home-limit clamp; 2° raw-seam hold; rotary-frame wrap; CSV mode for single-axis counts/s jogs | `robot_v4_rtcore_ethercat_runtime.hpp:1883-2042`, `robot_v4_rtcore.cpp:2738-2983` | `rtcore.py::cycle` |
| Units: 131072 counts/motor rev × gear ratio, sign from `ROBOT_V4_ROBOT_AXES`, home at the frame midpoint (607C = RM/2) | `robot_v4_rtcore.cpp:2650`, `a6ec.md` | `rtcore.py`, `plant.py` |
| DS402: statusword 0x0250 disarmed → 0x0231 → 0x0233 → 0x0237 enabled, homed bits 12+15, CSP/CSV/PP mode display | `robot_v4_rtcore.cpp:3576-3607`, `a6ec.md` | `rtcore.py::_ds402_step` |
| A6-EC CSP cascade: position P (C01.00), speed PI (C01.01/02) with PDFF (C01.1B), torque filter (C01.03), speed/torque feedforward (C01.14–18), torque limit (60E0/1), Er47.0 window (6065/6066) | A6-EC manual §4.1.6 fig. 4-39, §7.4–7.8, §11.3 | `drive.py` |
| `status.json` (sim writer keys + pair-config block) and the RTT3 telemetry ring natsbridge reads | `robot_v4_rtcore_telemetry_status.hpp:899-1042`, `robot_v4_rtcore.cpp:531-577` | `status.py` |

The model is built from the same `urdf/v1/<model>/robot.urdf` everything
else reads: bodies from links, hinges from joints with the URDF limits,
Collada meshes converted once to OBJ, mass and inertia from the visual mesh
volume at a configured density, the rotor inertia reflected through the
gearbox as joint `armature`, and the `<transmission>` reduction as the gear
ratio. `model.fk_check` compares every link frame against a numpy chain of
the URDF at random poses (5e-16 m on `rosie_1400_v3`, 2026-08-29).

## Two ways to get a pair configuration

* **`--authority-json`** — the `robot.v4.rtcore.status.v1` document an
  isolated cell injects into cartesiand (`offline-programming/v1/internal/
  simulator/jog_authority.go`). The sim adopts its identity, holds
  `runtime_mode_epoch` / `home_epoch` static (every consumer holds the same
  static document; a moving epoch stops the feedback binding matching),
  and reports home valid at the frame midpoint, as the document claims.
* **nothing** — wait for `robot-v4-pair-owner` / natsbridge to send
  `CmdJointConfigV2`, verify its home-profile key, mint the epochs, and
  publish them through `status.json` and the ring, as RTCore does on a
  Jetson. This is the `run-local-sim-stack.sh` shape with `$RTCORE_BIN`
  replaced; it has been exercised only by the unit tests, not against a
  running pair owner.

## What the mirror predicts, and what it measured

* **A CSP jog runs at 0.8 × the commanded velocity.** RTCore rebuilds 607A
  every cycle as *actual + 20 ms of travel*; a P-only position loop at the
  manual's 40 /s then settles where `40 × 0.020 × qd = 0.8 qd`. Measured
  0.081 rad for 0.05 rad/s × 2 s (`tests/test_sim_udp.py`). Single-axis
  joint jogs go through CSV (mode 9) and run at 1.0 × (0.103 rad measured).
  The real cell's C01.00 and speed feedforward decide the real factor; if
  the real robot jogs at the commanded speed, C01.00 is higher than 40 or
  C01.14 is not zero -- either goes in `DriveGains`.
* **A held axis does not creep** (RTCore aligns the target to the actual
  position; the drives sit on it): < 2 mrad over 1 s in the OLP integration
  test.
* **Brake-release sag** depends on the inertia ratio C00.06 the drive was
  tuned with. At the manual's 100 % J5 dropped 42 mrad when the drives
  enabled; with the ratio auto-derived from the model's mass matrix (what a
  commissioned drive's auto-tune leaves behind: 115 / 233 / 96 / 873 / 3229 /
  1516 % for J1–J6 at the zero pose) it is 2.8 mrad. `CellConfig.
  auto_inertia_ratio` chooses; the real drives' C00.06 has never been read
  back into this repo.
* **Closed-loop playback through cartesiand** (OLP plan of 16 samples over
  30 s, `POST .../jog/play`): 0.6–1.7 mrad tracking error, completed in
  30.2–30.4 s wall, final pose within 3 mrad of home (2026-08-29).
* **Timing.** `mj_step` is 3 µs on this model; the Python cycle is ~200 µs;
  0.3 % of cycles were late on an idle box and 14 % with `go test`,
  cartesiand and the sim sharing cores. A stall over 50 ms resyncs the
  cycle clock instead of replaying the backlog.

## Unverified

Everything in `drive.MOTORS` (which motor frame sits on which joint, rated
torque, rotor inertia), every `DriveGains` default (the manual's, not the
cell's), the brake release time, the mesh density, and the joint damping.
Each is a dataclass field with its provenance in the comment; the honest
next step is an SDO read-back of C01.xx / C00.06 from the cell's drives.

## Layout

```
python/cellsim/   wire.py rtcore.py drive.py plant.py model.py meshes.py status.py main.py
tests/            test_wire test_rtcore test_drive test_model test_sim_udp, support.py (authority, RTCore client, sim process)
.cache/meshes/    OBJ conversions of the cell's .dae (gitignored)
```
