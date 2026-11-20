# Cell I/O (P6-05)

Configuration files are organized by layer under [config/](../config/README.md): drive, robot, machine and cell configs.

Doors, e-stop and light curtains act through the hardware safety chain on drive
torque. Their software observations are supervisory. Fast consent, zone and
weld-ready contacts are read from the EtherCAT terminal process image each native
cycle. Drive mailbox digital inputs remain display signals and cannot provide
these cyclic contacts. This implementation does not qualify a safety function.

`config/drives/dio_terminal.json` describes a generic combined eight-input/eight-output
terminal. Its identity, mapping and channel count are placeholders marked
`_unverified`, not an inventoried EL1008 or EL2008 description. The current
compiler supports one terminal position, distinct from every drive position.
Real separate DI and DO terminals require separate inventoried profiles and
multi-terminal support before use. The simulation example is
`config/cell-io/simulation.json`.

### Schema 2 profiles: mailbox-free terminals and RevPi DIO

A profile with `schema_version: 2` selects its path with `io_backend`:

- `ethercat`: a terminal with no CoE mailbox, such as the PLANET IECS-1116-DO
  (`config/drives/planet_iecs_1116_do.json`, 16 open-collector outputs, no
  inputs). Identity is `vendor_id`, `product_code` and `revision_no`; the layout
  is `output_pdos` and `input_pdos`, at most two fixed one-byte PDOs per
  direction, each on its own sync manager (first byte channels 0-7, second
  8-15). No SDO can read such a terminal's assignment, so bring-up configures
  the declared PDOs explicitly and refuses unless every one matches the
  SII-scanned sync managers (`ecrt_master_get_sync_manager/pdo/pdo_entry`). An
  SII without PDO categories is refused, not assumed to match. The PLANET
  identity and layout come from its ESI (`docs/drives/planet/`), unverified on
  a bus until `ethercat slaves -v` and `ethercat pdos` are read on the cell.
- `picontrol`: the RevPi DIO expansion (`config/drives/revpi_dio.json`, O_1..O_14
  and I_1..I_14 as bits 0..13). The machine `io` block names
  `picontrol_device` (`/dev/piControl0`) and the PiCtory byte offsets
  `picontrol_output_offset` and `picontrol_input_offset` (`piTest -v O_1`,
  `piTest -v I_1`); `slave_position` does not apply and the module is not a bus
  device. The native cycle rewrites the whole output word with `pwrite` every
  cycle, so a second writer (`piTest -w`) is overwritten within a cycle; an
  observation is fresh only after this process's last write succeeded. A clean
  shutdown writes OFF, but a killed or hung rt-core leaves the last word on the
  module: piControl keeps exchanging it, and the DIO's own watchdog (outputs to
  0 after 500 ms without a piBridge exchange, firmware 1.5 or newer) only
  covers a stopped piControl. The bit layout was read on a Rosie 1430 cell's
  RevPi DIO (`piTest -v O_1` offset 83 bit 0, `O_9` offset 84 bit 0). A red IN LED with
  `Status` bit 0 set (no communication to the input IC) and every input
  reading 1 means the inputs' 24 V field supply (X2 pins 1/2) is missing.

Only eight channels per direction carry names (`IoStateV1`, the marker mask);
bits may address any of the profile's channels (at most 16).

### Outputs without wired feedback

While no input module is fitted, an output may declare `"readback":
"none_unverified"` instead of `readback_bit`/`readback_polarity` (the default is
`"readback": "wired"`). Such an output is commanded but never observed: status
reports it `valid: false`, `io_arm` needs no OFF observation for it, and no
readback disagreement can be detected for it. Stop fencing, the fresh exchange
required after a fence, expiry, grant and permit gating are unchanged. This is
JSLOP's posture (it read the output process image back, which proves nothing
about the wire); it is an exception, not a qualification. Fitting an input
module (PLANET IECS-1116-DI) and wiring each output back restores the rule.

The configured terminal is a declared non-axis device with disposition
`io_terminal`. Admission counts nine drives plus one terminal as ten required
responses. Its identity and presence are required under both `hold` and `refuse`;
after activation it must remain OP. Unreadable identity, a replacement, absence
or loss of OP refuses readiness. An undeclared terminal follows
`bus.unknown_slaves`, just like a spare drive, and receives no PDO registration.
Describe publishes the configured `io` policy and the terminal's bus disposition;
status publishes the independent cyclic `IoState`.

The configuration `io` block requires `profile`, `slave_position`, explicit
`torch_qualified: false`, `inputs` and `outputs`. Inputs have a unique `name`,
`bit` (0..7), `polarity` (`active_high` or `active_low`) and `class` (`supervisory`
or `fast`). Outputs have a unique `name`, `bit`, `class` (`process` or `torch`),
`safe_state: false`, `expiry_ns` (1..1000000000), and independent DI
`readback_bit`/`readback_polarity`. Duplicate output bits or feedback bits, missing
expiry, unknown classes and any other safe state are refused. The I/O block and
terminal profile bind the configuration digest; coordinate identity is unchanged.

Only the native cycle writes output PDOs. All configured fast inputs must be
valid and satisfied before ON. Stop sets every process/torch output OFF in the applying cycle,
clears intent and invalidates I/O permission. A fresh grant and explicit `io_arm`
and valid OFF readback observed after the fence cycle are required before another
ON; motion also retains its own Enable/Arm checks.
`io_disarm` clears permission and intent. Exchange loss invalidates inputs and
commands OFF. These software rules cannot establish physical OFF through a lost
bus: terminal watchdog behavior must be qualified independently.

Dense `process_markers` attach `{action, value}` to a source `{segment, sample}`;
the action names an output. Markers survive dwell and collapsed segment endpoints.
Native delivery is bounded to one cycle from the attached sample. Late markers
fault without replay. Two markers for the same output in one exchange also fault
instead of collapsing a physical transition. Each ON starts its own expiry, which neither grant renewal
nor a client worker can extend. Repeated explicit ON markers start new expiries.
Intent, command, independent wired feedback, validity and observation/expiry
clocks are separate in `status.core.io`. Readback disagreement after three
exchanges latches typed `io_readback_disagreement` and `cell_io` fault bit 13;
restart is required after investigation. The configured OFF state, not a software
echo of the output byte, must be observed before arming.

Prepared marker intent survives a definite `not_ready` or `mode_conflict` Start
refusal and restoration under the same identity. Native point refusals stop the
Go upload when observed and return `TrajectoryPointError` with the zero-based
upload point index and the original `RejectedError` receipt. Torch-class ON and
OFF markers both preserve native `io_torch_unqualified` through the adapter's
public refusal mapping; a later COMMIT result cannot replace that diagnosis.

## First terminal hardware qualification

Before any output class is enabled for cell operation, record all of these checks
on the first bench terminal with the hardware safety chain active and controlled
test loads isolated from process equipment:

1. Read and match the terminal's SII position, vendor, product and revision.
2. Read back both PDO assignments, every mapped entry and the scanned SM layout;
   each must match the declared profile before cyclic output admission.
3. Toggle one wired input by hand. Record the physical edge and native cycle
   observation showing the correct value and polarity within one cycle.
4. Command one isolated test output ON with an explicit I/O arm. Record the
   independent wired readback, separately from intent and commanded value.
5. Issue no further output request and record expiry to the declared OFF safe
   state, including physical readback. Grant renewal must not extend expiry.
6. Apply Stop while ON. Record the fence-cycle OFF command and independent safe
   readback; prove the old grant cannot turn it ON and a fresh grant still needs
   explicit `io_arm` after the fence.

Also qualify fast-contact reaction, bus-loss and terminal-watchdog OFF behavior,
readback lag and marker latency under load. Channel count, the 1000000000 ns
maximum expiry, one-cycle delivery and three-exchange settling bounds remain
`_unverified`. No terminal exists on either bench. Simulator and unit-test results
do not qualify torch: torch-bearing admission remains refused until these checks
are recorded and reviewed, including explicit torch-class OFF markers. Cell
configurations retain `torch_qualified: false`; recording checks does
not automatically enable torch. The candidate qualified path requires a reviewed
`torch_qualification_record`, one torch output with independent wired readback,
and a fast permit input. Those declarations are not proof of hardware qualification. See [APPLICATION_API.md](APPLICATION_API.md) for typed
operations and [CONTRACTS.md](CONTRACTS.md) for fencing.


## Sixteen-channel I/O candidate, 2026-10-17

Named capacity is sixteen inputs and sixteen outputs. Native protocol minor 15
changes IoStateV1/StatusSnapshotV1 and extends TrajectoryPointV1 to 312 bytes;
all IPC clients, the adapter, OLP and pendant require coordinated replacement.
The runtime preallocates 234,000,168 bytes for three maximum-size trajectory
slots (24,000,000 bytes more than the previous layout), with no allocation or
unbounded scan added to the cyclic wait/marker path.

`io_test` requires the current lease and application generation, disarmed axes,
idle motion and explicit test permission. Per-channel source sequence fences old
ON after OFF. A renewal cannot restore an expired intent; only a new explicit
ON edge can begin it. Since `bd322d01c` (2026-10-19), manual torch terminal tests use the same
explicit-begin, sequence and expiry rules as other testable outputs. Manual
expiry switches OFF without an automatic-weld marker fault. This does not
qualify programmed weld markers.
The pendant batches fresh samples at 50 ms and ends its test on close, loss of
focus, stale status, motion or L2 Arm. Native expiry remains independent of UI
scheduling. Commanded OFF is not independent physical feedback. In particular,
RevPi piControl may retain its process word if native rt-core is killed: the DIO
piBridge watchdog alone does not cover that failure. No watchdog qualification
is claimed by these software tests.

A prepared point may carry a named-input barrier (`timeout_ms`, `stable_ms` in
its process marker). Admission requires fresh channel definitions and a
stationary next interval. The native clock pauses at the barrier while the
input satisfies its continuous stability interval. Existing non-torch output
intents are refreshed by native only while this active stationary barrier runs.
Timeout or invalid input clears intents and faults execution, preserving Home
when coordinates remain trusted. Stop/disarm cancels rather than resuming a
barrier. Trace and dry run strip the process schedule, including waits.

Taught weld start/stop markers occur on arrival at their annotated waypoints.
Generic I/O rows and manual tests cannot substitute for a torch-class action.
Qualified weld expiry or loss of I/O permission faults execution before another
motion sample; it cannot silently continue the path with its weld output lost.
Physical qualification, real wire timing and weld quality remain untested.


### October 18 candidate: scoped input monitors

Program `monitor` / `end_monitor` events bind named inputs into immutable
`monitor_mask` / `monitor_values` at each trajectory sample (TrajectoryPointV1
320 bytes in protocol minor 15). Pairing, configured names and exact sample
binding are checked before preparation. During execution the native cycle checks
validity and required value, with at most sixteen comparisons and no allocation.
An invalid/mismatched monitored input raises the I/O wait fault and clears output
authority. End monitor removes that requirement. These are operational process
checks, not safety-rated interlocks. Dry-run/Trace strips monitors as well as waits
and output actions. Output authority is requested only for actual output markers.


## October 18 — controller-owned power/arm indicators (candidate)

A green indicator output stays solid when any configured drive reports power
and flashes while the robot is armed; an alarm output pulses for a configured
duration (three seconds in this candidate) on each successful arm edge. The
green output uses a 1 s period, half ON/half OFF. Release returns
green to steady immediately; the alarm completes its pulse even after a short
arm/release. A controller fault cancels the pulse. Lost fresh exchange, I/O loss
or shutdown commands both indications OFF; restored disarmed feedback does not
replay the alarm. This is status indication, not a safety-rated power detector.

Optional output class `indicator` reserves those outputs from manual tests,
process markers and weld assignment. Native owns timing without an application
lease or extra thread/poller. The existing output status reports commands;
none_unverified feedback remains invalid. Other process outputs retain their
expiry, Stop and grant rules. L2, jogging, Home and drive tuning are unchanged.
The period and statusword mask/value live in machine configuration, not code.
A6-EC manual CSP statusword 6041 bit 4 is Voltage enabled (PDF page 109,
printed page 107). The live disarmed sample
reported that bit set. No electrical power-cycle qualification is claimed.

Native unit checks cover steady power, flash boundaries, exact pulse expiry,
repeat arms, short holds, fault/loss/shutdown, recovery without replay and
manual/program rejection. Linux integration, installation and physical light/
alarm acceptance remain pending. No I/O or drive motion test has been performed
for this candidate. The existing piControl hung-owner/watchdog limitation remains.
