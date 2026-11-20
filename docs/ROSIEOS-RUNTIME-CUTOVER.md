# RosieOS runtime cutover

Objective: rebuild the demonstrated native control capability from RosieOS
source, with one release procedure and explicit bench and cell configurations.
No active JSLOP service or source-path dependency may remain after promotion.
Offline backups and attribution are retained; this is not deletion of history.

## Completion scope (September 12)

The finish line is a team-usable release, not just replacing installed paths.

- [x] Commit and build the integrated RosieOS runtime; retain exact artifacts.
- [ ] Run the native app in simulation with this release, including joystick
  motion, release-to-stop, Arm/disarm transitions and reconnect behavior.
- [ ] Deploy the same source release to the bench and verify bench jog and Stop.
- [ ] Verify Rosie 1420 jog and Stop from the Deck after this cutover.
- [ ] Package all required source, SDK/environment and asset references so
  the team can reproduce without this Mac or agent history.
- [ ] Prove candidate deployment and rollback with the existing daemon path,
  including effective systemd configuration, not only desired-state acceptance.
- [ ] Reconcile the branch stack against current main and domain work, review
  the combined candidate, then merge the qualified PR(s).
- [ ] Establish component owners and focused contribution instructions that
  contributors can use directly.

Every run identifies source, artifacts, target, model/configuration and result.
Testbench and Rosie are distinct targets, even when they select the same model.
The last successful installation remains available during candidate evaluation.
No merge or physical-success claim follows from component tests alone.

### Fast changes without private forks

A rapid edit/deploy/test loop is a supported workflow. Keep candidate
builds isolated by source revision; stage while someone else uses the hardware,
then claim a target for activation and testing. Do not replace another active
operator's installation or leave ad-hoc overrides that defeat the next deploy.
Release the target and retain the exact result/rollback at the end. Reuse the
existing lease/deployment mechanisms; repair specific gaps rather than building
another scheduler or runtime.

Separate review and regression surfaces:

| Segment | Change boundary | Minimum relevant check |
| --- | --- | --- |
| Native/web UI and input | Presentation, navigation and input interpretation; no hidden controller-policy changes | Real input/sim behavior, Stop and input-loss tests |
| Motion Server / planner integration | Commands, plans, cancellation and authority protocol | Actual-loop tests, planner interface and simulator replay |
| RTCore / bridge | Drive output, timing, units and lease enforcement | Controller regressions, bench replay, then cell replay when affected |
| Cell configuration | Target identity, model, gearing, calibration and effective limits | Exact target/config comparison; preserve Home and explicit changes |
| Build/deploy/evidence | Build identity, install/rollback and retained results | Clean reproduction and effective loaded-state verification |

Ownership must be agreed with the team before adding CODEOWNERS; there is
currently no CODEOWNERS file on this candidate. Contributors can work against
these boundaries without owning an entire runtime fork.

Prefer one behavioral change per PR. Splitting by component is useful when
independently testable; genuinely coupled protocol changes belong together.
The current recovery import is a convergence checkpoint, not the template for
future large PRs. Shared hardware is exclusive during activation/control;
source editing, builds and simulation need not wait for it.

This is a reviewed component-source integration, not a merge of unrelated Git
histories. Motion comes from the reconstructed 558f09af source plus named-Deck
support; RTCore from the reconstructed e7dd1a39 source; bridge/PairOwner from
the verified Go closure plus the explicit eligible-Deck-set repair. The native
client preserves ca17095. Source and regression provenance stay recorded.

Preserve current physical configuration, Home, calibration, command authority,
limits and recovery behavior. Do not raise either target's effective limits.
Keep deployment disconnected from live control until source tests and staged
binary/configuration checks pass. Stop/disarm and retain rollback before target
activation. No rehoming of assembled Rosie; physical replay needs current
on-site readiness.

Judges: common nine-case planning/control fixture; JSLOP command-result and
tracking fixtures; RTCore counts/radians, Stop, limit and lead tests; full bridge
suite; native six/nine-axis Arm test; targeted deployment readback and real Deck
replay on each target. Inventory all active runtime units and loaded paths;
renaming a service or copying the previous executable is not completion.

Current disposition: source-built runtime installed on the Rosie 1420 cell's
Jetson and RevPi; post-cutover operator jog not evaluated because the cell's
Deck is offline.

## Source checkpoint checks (September 12)

- Full bridge package and LIVE observer-recovery regression passed.
- Native configured-axis Arm readback passed for six and nine axes. Integration
  required restoring the client's status-sequence header and JSON-scanner-only
  include boundary, plus supplying the scanner's optional-presence output.
- Nine actual Motion Server loop cases passed, including owned cold-command
  results, duplicate-command behavior, Stop/Disarm cancellation, epoch/Home/model
  changes, collision rejection, busy retry, and complete authority publication.
- Reconstructed RTCore counts and speed-scaled-lead fixtures passed. These are
  offline tests, not hardware tracking qualification.
- The accepted-plan dry-run fixture still fails on both unchanged parent and
  candidate. Do not report complete spreadsheet playback qualification.

Evidence is retained in `output/rosieos-cutover-20260912` in the AMR workspace.
The following installation record supersedes the source-only checkpoint.

## Installed release: c482179

Source: `c4821793ad0bf92a7af5f45766567c5d6a66a92f`.
The release directory is `~/.rosie/releases/rosieos-c482179` on the cell's
Jetson and RevPi. `release.json` identifies all four binaries and their rollback paths.

Motion Server and RTCore rebuilt byte-for-byte to the working binaries; bridge
and PairOwner were rebuilt from the integrated Go source. `/proc/PID/exe` paths
and hashes were checked after switching, not inferred from desired records.
The collision service now loads both wrapper and checker from this release's
RosieOS source. It retains the installed Pixi Python environment under
the predecessor monorepo's `tesseract/v1`; that is a toolchain dependency, not JSLOP code.

The JSLOP telemetry service is disabled. Its archive and source remain untouched
for custody/rollback. Historic schema names and source attribution are retained.
There is no active JSLOP executable or collision-source dependency in the
qualified control chain. This is source ownership and a reproducible build,
not a claim that the inherited implementation has been redesigned.

### Actual cutover and repeatable path

1. Freeze source; test the combined native, bridge, Motion Server and RTCore
   candidate. Build ARM C++ on the bench Jetson with the RevPi's pinned
   EtherCAT SDK, cross-build Go, and build headed native on a build host. Keep artifact hashes.
2. Stage without activation. Check archive hashes and target dynamic linking.
   Run collision-free and collision-positive fixtures with the staged checker.
3. Capture actual process configuration and rollback intents. Observe SIM,
   disarmed, zero enabled axes and motion output disabled. Preserve Home/pair.
4. Submit exact release paths, hashes and existing configuration through
   `/api/processes/desired` for RTCore, bridge, PairOwner and Motion Server.
5. **Inspect effective systemd configuration.** On this installation, the old
   `99-v3-normal-pdo-command-invalidation.conf` overrode the daemon's new
   `25-daemon-intent.conf`. Its old executable path remained active despite an
   applied intent. Move the override into rollback, reload and restart.
   Redundant RTCore `40-integrated-csp.conf`, `41-jslop-speed-scaled-lead.conf`
   and bridge `40-integrated-csp.conf` were also moved to rollback; their exact
   settings are already in the desired record. Do not blindly repeat these
   filenames on another machine: compare effective configuration first.
6. Switch the collision unit to the source-owned checker and disable the old
   read-only JSLOP recorder. Keep archive data and unit backups.
7. Verify loaded hashes, active service paths, fresh NATS status and exact pair
   identity. Exercise the installed collision service with both fixtures.
8. With the Deck awake and the operator at Rosie, install the built native app,
   enter LIVE, take authority, Arm, jog and Stop/disarm using the established
   workflow. This final step remains outstanding for this release.

Post-install checks passed: six-axis Home mask `63`, model `rosie_1420_v1`,
pair hash `857bdccac00fabb115fb7ac2ec76489498b5dc7690f5106be6fa1a7a5534af47`,
fresh Motion Server/RTCore status, pair authority observed, zero send errors,
SIM/disarmed with output disabled. Installed collision probes checked three
clear poses and five poses yielding four contacts. No rehoming or motion was
performed during this cutover.

Deck binary SHA-256:
`ace9095f2b5d8b85382f53a99528fb2b42ecef2535a2bc2bd4170370f8370909`.
Built successfully with Vulkan/X11 and packet/joystick-release self-tests;
not yet installed on the cell's Deck. Tailnet and saved LAN SSH were unavailable.

Rollback: restore the saved desired records, collision unit and moved systemd
drop-ins; reload and restart while disarmed, then verify loaded artifacts and
pair/Home readback. Enable the old recorder only if restoring that capture lane.
Do not delete or reset `/var/lib/robot-v4/urdf-pairs` or the Home profile.

Verdict: **passed** for build, installation and passive/runtime collision checks;
**capability_unavailable** for post-cutover Deck/physical replay. The earlier
operator-confirmed jog belongs to the prior installation, not this receipt.
