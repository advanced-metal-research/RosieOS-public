# Qualification pipeline

Status: Current design and implementation authority for qualification work.
Reconciled against RosieOS main at e7258d4b on 2026-11-10.

## Goal and model

For one immutable candidate, determine the evidence required for its intended use.
Run cheap evidence first. Use physical hardware only for claims that need it.
Derive qualification from policy and immutable scenario receipts.

Source diff → plan → PR checks → main → build-host candidate → simulation → servo HIL → robot → welding.
The same candidate ID moves through each environment. A new binary is a new candidate.
An exploratory test does not qualify a candidate.

## Current implementation and corrected assumptions

| Surface | Verified state on main | Qualification consequence |
| --- | --- | --- |
| PR admission | The policy is in shadow mode. The PR admission workflow runs for every pull request. Several source areas still have pending proof mappings. | A green job means the report ran. It does not mean the change is admitted for release. |
| Managed gate coverage | The release-test coverage file maps 12 managed suites to PR or build-host owners. | This is a release-gate map, not a scenario catalog or physical receipt. |
| GitHub workflows | 17 YAML workflows cover source, package, client, planner, preflight, build-host, and candidate simulation contracts. | A published RTCore candidate triggers simulation. No workflow promotes it through servo HIL or robot evidence. |
| Build host | The worker builds a fixed RTCore candidate for one servo-bench target profile. It records source, recipe, environment, artifact hashes, and checks. Publication is separate. | Use its rosie.release-candidate.v1 ID as qualification identity. |
| Build-host combination qualifier | The existing qualifier runs a fixed isolated software matrix for main, a reference, and reviewed PR pins. It has its own config/qualification-policy.json. | This tests source combinations. It does not implement this document's claim policy or physical qualification. |
| Managed release | The deploy Go module verifies a candidate, binds it to a target, installs, activates, verifies, journals, and rolls back. | Use this transaction for physical qualification. |
| Simulation | Native RTCore and composed motion simulation tests exist. The MuJoCo README describes a V4 mirror that reads urdf/v1. | Start with native simulation. Review MuJoCo model ownership before it can qualify current robot descriptions. |
| Configuration | Robot descriptions own model manifests. Machine configs pin description identity. Cell configs bind hosts and runtime. | Derive declared capability here. A config does not prove live physical readiness. |
| Agent instructions | CLAUDE.md exists on main. Root AGENTS.md did not exist at this commit. | Both agents and humans need a pointer to this contract. |

The first build-host candidate recipe covers RTCore for one target profile.
Other component recipes remain future work.
The existing build-host qualifier's name is an overlap, not this policy's owner.

### Direct-release suite comparison, 2026-11-11

At source commit `2a86fef8`, the direct managed full gate names 12 suites in
`steamdeck/real/v4/release_tests.go`. The coverage map assigns eight to PR CI
and four to the build host. This audit compares test commands and selectors. It does
not claim that a particular candidate passed them.

| Direct gate entries | PR or build-host command | Comparison |
| --- | --- | --- |
| `make_test`, `make_test_serial` | The standalone RTCore native job runs `make test`. The Makefile calls both `test-runs` and `test-serial`. | Same test targets. The direct gate splits them so its serial tail runs alone. |
| `motion_test` | The Cartesian job runs `make all test test-composed`. | Includes the direct `make test` target and one composed test. |
| `ui_test`, `ui_npm_ci` | The OLP job runs clean `npm ci`, then `npm run lint && npm test`. | Includes both direct commands. The direct gate can reuse cached dependencies when its lock hash matches. |
| `olp` | The OLP job runs `go test -race -count=1 ./...`. | Includes the direct `go test -count=1 ./internal/...` packages, with race checks. |
| `pendant_olp_core`, `cells` | The OLP job runs the same Node test selector. The RTCore native job runs the same cell package command. | Same selectors. |
| `rtctl`, `faultmatrix_shutdown`, `sdk_path_jog`, `olp_composed` | The build-host worker runs the same Go test selectors in four serial gates. Its wrapper refuses an empty or entirely skipped selection. | Same selected tests. The build host also builds the native inputs each gate needs. |

The map validator checks that each suite has an owner and that its command
appears in the named source or workflow. It cannot prove equivalent runtime
conditions or a passing execution. The publisher checks four successful
workflows for the exact merged SHA. The build host records four gate results and
their result digest in the staged build log. Keep those results together
before calling a candidate software-vetted.

The build host published the `2a86fef8` prerelease on 2026-11-11 with installed
worker `34b1ef79`. That publisher enforced only the release-request and RTCore
workflows. The admission and consumer workflows also passed for the same SHA,
but the publication receipt does not bind them. The build host now runs worker
`2a86fef8`, which requires all four workflows. Treat this prerelease as a
pipeline probe, not as four-check publication evidence. A new operation ID
must produce a new immutable release after the simulation result is reviewed.

The release simulation run `43114261805` stopped in the source stage. The
`operator-runtime-contract` Go test made a PKI fixture under Go's temp root,
but did not pass that root as `TMPDIR` to the PKI script. The PR OLP job sets
`TMPDIR`, so it did not expose this failure. The fixture now passes its own
temp root, and a Linux test covers an unset caller `TMPDIR`. This run did not
produce simulation evidence. Its source evaluation also lacked four declared
claims: compatibility, resources, security, and timing. Those claims remain
open; the temp-root fix does not satisfy them.

PR RTCore workflows use path filters, while their required `main` push runs
have no path filters. This can expose a source test after merge that did not
run on the PR. Align the applicable PR failure surface with the publication
gate before enforcing green PR checks. Do not remove a repeated run until its
distinct proof, such as the merged SHA or target package, has another owner.

A motion-free servo-bench install can support a scoped RTCore bench-install claim.
It does not meet this document's robot-qualified beta floor. Keep these names
and promotion rules separate; do not relabel a bench receipt as robot evidence.

The local managed-release journal records the selected source, configuration,
package, and candidate IDs with the command result. It does not prove what the
device loaded. The operations dashboard does not ingest it yet.
Keep live readback and physical receipts as separate requirements.
The native `rt-core.yml` job runs the focused managed-release transaction tests.
The publisher requires that workflow to pass on the candidate's exact main
commit. This covers source behavior, not a live install or rollback drill.

## Sources of truth

| Source | Owns |
| --- | --- |
| This document | Qualification design and phase boundaries |
| quality/qualification-policy.json | Classes, modifiers, path rules, claims, environments, beta floor, and exception shape |
| quality/qualification-comparison-starts.json | Reviewed source comparison start for each candidate target profile |
| Scenario declarations beside tests | Stable facts about executable scenarios |
| Robot descriptions, machine configs, cell configs | Declared hardware and configuration identity |
| Build-host candidate manifest | Candidate identity and artifact evidence |
| Scenario receipts | Observed execution results |

The catalog, plan, capability view, and qualification summary are generated outputs.
Do not maintain another test catalog or hardware inventory.
Live health, lease state, and availability are runtime facts.

## Release and classification policy

One candidate can gain source, simulation, servo, robot, and welding evidence.
Compute state from the candidate, policy identity, required claims, receipts, and approved exceptions.
Never edit a failed receipt into a pass.

Robot beta is a manual assignment to a robot-qualified candidate.
Every robot beta candidate must pass the common robot floor.
Robot beta can include an accepted defect through a scoped reviewed exception.
An RTCore-only candidate can instead receive a component-scoped
`bench-install-tested RTCore beta` label after a motion-free bench receipt.
That label grants no motion, assembled-robot, client, or welding claim.
Do not create a stable channel until it changes an operating decision.
Nightly and weekly can later describe schedules, not different binaries.

### Separate install and qualification verdicts

The original 12 direct-release suites are the software floor for a motion-free
bench install. The build-host result must pass its four fixed suites. The
publication receipt must bind successful runs of the four required GitHub
workflows to the same merged commit. The candidate archive and its target must
verify. Candidate-bound simulation receipts must pass before host mutation.

The install verdict checks that floor and reports every broader source claim
that remains incomplete. It does not call those claims passed. The current
`source-qualified` and `hil-inputs` verdicts still require the broader claims;
they must not be reused as the install verdict or weakened for motion.

| Verdict | Evidence it can support | Evidence it cannot support |
| --- | --- | --- |
| `install-inputs-checked` | The sealed candidate passed the software floor and its candidate-bound simulation. The reviewed bench selection, reservation, live identity, and rollback checks still gate host mutation. | The candidate loaded on hardware, motion works, or the robot is ready. |
| `bench-install-tested` | A motion-free receipt records the exact RTCore package and configuration on the RevPi, the Jetson relay's active component identities, start and health results, final-safe state, and rollback result. | Motor direction, motion limits, assembled-robot behavior, clients, or welding. |
| `motion-qualified` | Approved physical motion scenarios pass on the selected hardware. | An assembled robot or welding process is qualified. |
| `robot-qualified` | The common robot floor and the selected robot scenarios pass. | Welding is qualified without welding-cell evidence. |

`install-inputs` is a separate, read-only preflight. It verifies the sealed
candidate, the 12-suite coverage map, four passed build-host gate results in the
sealed build log, and the publisher's local receipt for four successful GitHub
workflows at the exact source commit. It also requires candidate-bound
simulation, the selected target profile, and a motion-free observation
scenario. It reports broader source claims without calling them passed.
The receipt is copied from the build host through a trusted operator path; the
preflight checks its content against all seven candidate assets but does not
authenticate the receipt against GitHub. Confirm that trust boundary before
routine or automatic use.

`hil-inputs` keeps its broader source and servo requirements. The motion-peer
observer accepts `--install-only --publication-receipt PATH` for a motion-free
readback after installation. Its receipt still says `not_qualified`; it does
not prove rollback or motion. It reads the Jetson boot ID, but not the active
Jetson OLP or planner build. It reads the RevPi RTCore package, unit status,
and controller state. A `bench-install-tested` verdict still needs the managed
release journal, Jetson component readback, and a tested rollback result. The
RTCore-only candidate has no Jetson application asset, so Jetson readback
records the relay environment, not another copy of the candidate package.
The managed release command does not call the new preflight automatically.
For the first supervised run, the operator must
run it before the existing `--plan` and installation steps. Enforce it in the
managed command before routine deployment. A candidate built before this
policy change cannot use the new verdict from its exact source checkout.

The policy distinguishes fleet-coordination from deployment-runtime.
Fleet reservation and snapshot code needs source contract checks. It does not
change on-device restart behavior. Other classes cover realtime-control,
motion-execution, operator-programming, planning, robot-cell-config,
welding-process, and tooling-docs. A change can have several classes.
The qualifier adds qualification-infrastructure for policy, scenario metadata,
and proof workflows. Those changes need source checks, not physical runs.
Security, timing, compatibility, and resource-impact are modifiers.
Requirements are the union of all matching classes and modifiers.

Path rules provide a deterministic minimum. Changed diff content can add
modifiers and the welding-process class. Python test files do not add product
claims from fixture text.
The plan names each matching rule and reason.
This first classifier does not compute a full import dependency graph.
Unknown paths are visible and would block under enforcement.
A `deleted_only` rule covers a retired tree (`robot/`, `urdf/`, the old
pendants, `nats/`, dated records): it matches only a deletion the diff
itself shows, keeps the classes the tree had, and adds compatibility for
code, because a live consumer may still read it. A path list without the
diff, or a file added back under a retired tree, stays unknown.
A developer can broaden a local plan with --add-class or --add-modifier.
Ordinary PR metadata cannot reduce required evidence.
Keep PR integration in shadow mode until representative changes classify correctly.

The shadow review used real historical commits:

| Commit | Change | Plan after rule review |
| --- | --- | --- |
| 00bf23c4 | PR proof workflows and coverage tests | Source qualification checks; no servo claim |
| ab3329ad | Drive timeout, cell config, and OLP test | Cell-config and timing claims; the OLP test adds no robot claim |
| 1a4f24ac | Robot limit and model pins | Cell-config and planning claims; the Tesseract path is recognized |
| fdc8556c | Build-host worker and release path | Deployment, runtime-restart simulation, and several modifier claims; review the broad modifier match |
| 602dec57 | Weld output selection and Trace behavior | Operator-programming, planning, and welding-process claims |

The weld-output text rule catches commit 602dec57. It does not prove that all
semantic weld changes have matching text. The broad modifier matches on
fdc8556c also need review. Keep the policy in shadow mode. Do not use these
rules alone to schedule physical work.

Run the planner with node quality/qualification.mjs plan --base origin/main --head HEAD.
Use --commit SHA to plan one commit, --out PATH to save JSON, and catalog to list declarations.
Use capabilities --cell rt-core/config/cells/<cell>.json to inspect declared capability.
The plan records paths, classes, modifiers, claims, reasons, environments,
selected scenarios, uncovered claims, and missing receipts.
The output is deterministic for the same source, policy, and declarations.

## Scenario declarations and current mapping

A scenario is the smallest independently routable test.
Put its metadata beside the test in a .scenarios.json file.
The catalog discovers these files from repository paths. Do not edit a generated catalog.
Each declaration names an ID, claim, environment, test, capability, motion permission,
timeout, evidence output, final-safe rule, and owner.
It does not name a machine, release channel, or shell command.
The runner ID selects a fixed command in quality/qualification.mjs.
The policy decides if a claim is required.

| Scenario | Environment | Claim | Existing test |
| --- | --- | --- | --- |
| pr-admission-contract | source | admission-contract | quality/pr-admission.test.mjs |
| qualification-policy-contract | source | qualification-contract | quality/qualification.test.mjs |
| cell-config-compatibility | source | config-identity | rt-core/config/cells/compatibility_test.go |
| native-sim-home-arm | simulation | sim-home-arm | rt-core/adapters/rosie/control/simulated_a6ec_home_linux_test.go |
| native-runtime-restart | simulation | runtime-restart | rt-core/ipcclient/restart_fencing_linux_test.go |
| motion-composed-ordering | simulation | motion-command-ordering | motion-server/v1/tests/rt_core_composed_test.go |
| installed-identity-observation | servo HIL rehearsal | installed-identity | quality/hil_admission.test.py |

This first mapping does not cover all source or simulation behavior.
The plan reports uncovered claims.
The existing release-test coverage file still owns the 12 managed gate suites.

## Evidence environments

| Level | Environment | Main claim |
| --- | --- | --- |
| 0 | PR and host | Fast software, schema, package, and interface checks |
| 1 | Simulation | Composed workflows, transitions, faults, ordering, and planning |
| 2 | Representative servo HIL | Jetson, RevPi, matching drives, EtherCAT, Stop, Home, recovery, timing |
| 3 | Robot bay | Assembled mechanics, workspace, calibration, load, operator behavior |
| 4 | Welding cell | Process I/O, fixtures, interlocks, sensors, weld evidence |

Running-cell-affecting changes generally need simulation.
Pure documentation and unrelated tooling do not.
Simulation receipts cannot prove servo or robot behavior.
The second bench has a different servo profile and cannot silently replace the matching bench.

## Physical resources and leases

Reserve the whole servo bench, robot bay, or welding cell.
Do not reserve the Jetson, RevPi, and drives independently.
Use development, qualification, and soak modes.
A resource cannot be in development and qualification at the same time.

The qualification transition stops declared development work and clears declared temporary roots.
It installs the exact candidate through managed release, verifies identities,
audits owned services and paths, and checks health.
Only clean can enter qualification. Dirty needs repair. Unknown needs review.
Lost network or a crashed runner must not release a resource before local final-safe evidence.

Scenarios request capabilities, not named hosts.
Use existing robot, machine, and cell files for declared capability.
Combine that declaration with runtime-observed identity and health.
The first capability view reports physicalRobot and observedHealth as unknown.
Start with a visible queue. Do not preempt active motion automatically.
Local RosieOS control remains the motion and safety authority.

### Selected first step: cooperative busy signal

This is the selected design, not a deployed qualification workflow.
Use the existing Fleet Board hold to show that someone reserved the whole bench.
Use the cell's control grant to show who has motion control now. These are
different facts. Present both in RosieOS clients and deployment preflight;
do not create a second lease database. Show `unknown`, not `free`, when a source
cannot answer. A new managed deployment must warn and refuse while an active
hold or control session conflicts with it. The owner can end the hold through
its existing workflow. Emergency Stop and read-only status must still work.

The OLP maintenance notice already announces an update and blocks normal
robot actions. Do not use it as the HIL busy signal: it would also block test
actions that use OLP. The OLP control view already shows another controller.
The Fleet Board already shows reservations. The managed RTCore release already
serializes release runs and refuses an armed or granted controller. It does not
check Fleet Board holds. The remaining work is to present reservation and
control state at the deployment and client entry points used for this bench.

This is a cooperative protocol. It does not intercept direct SSH, existing
shells, service APIs, or agents that skip RosieOS commands. Keep present sudo
rights and the managed deploy path unchanged. Do not require a new emergency
admin login for this first step. A physical receipt must record that host
writes were not restricted. A passing run can support a supervised behavior
claim, but it cannot claim a clean, unchanged host. Any observed drift makes
the run incomplete. Automatic clean-bench qualification and promotion remain
disabled; a reviewer must judge supervised physical evidence explicitly.
This first step admits motion-free observation only. Unattended motion needs
an enforced host-write boundary or an independently verified safety system
that contains direct host changes. A busy signal and drift audit do not meet
that requirement.

The draft implementation adds `GET /api/busy` to the existing operations
dashboard. It reports `busy`, `free`, or `unknown` for each configured cell.
It combines Fleet Board holds with the dashboard's fresh controller observation.
The response contains no Fleet credential, owner name, or motion command.
The dashboard's sessions view shows both source states. If either source is
unavailable, the combined state is `unknown` unless the other proves `busy`.
The route is read-only and coalesces simultaneous reads. Keep it behind the
dashboard's tailnet access. Do not expose it on the public Internet.

Candidate deployment through `realtime-core release` can use
`--busy-status-url` for a cooperative preflight. Set it to the dashboard's
exact `/api/busy` URL. When set, the command refuses a busy, unknown, stale,
malformed, or unreachable view before it starts the release transaction.
The ordinary commit-based development path can use the same check. The flag
stays optional because Fleet Board does not yet declare every managed release
target, including some robot cells. The existing release lock and idle wait remain
the deployment gate for those targets. HIL qualification still requires a
separate cell-local admission gate.
This is a point-in-time cooperative check. A direct SSH write or a new hold
after the check can still change the cell. The OLP and Steam Deck clients do
not yet consume this route, and the running dashboard has not been updated
with this draft code.
Do not mark the client rollout or busy protocol complete from this PR.

### Physical admission authority

GitHub may request qualification, and the build host may build its candidate. Neither
owns a physical lease. A developer computer is also a requester, not an owner.
Fleet Board may reserve time, but its hold grants no motion or host-write
authority. The cell-local RosieOS gate owns one lease for the whole bench. It
must coordinate the motion-authority peer and its paired RTCore peer. The first
target is the servo bench's Jetson and RevPi pair. Select the peers from the cell
config, not from scenario code.

Before admission, compare the declaration with live observations. The existing
`rtctl inventory --config FILE --json` reads EtherCAT identities, drive values,
and bus topology from the RevPi. `rtctl hostcheck --expected-release FILE --json`
checks host prerequisites and the loaded RTCore release. The local gate must
also observe the active pair and configuration identity, installed candidate,
runtime health, and a final-safe starting state. Record each command's exit
status, output digest, host identity, boot identity, and observation time.
Unknown or stale observations block admission. A matching file in Git does not
prove that the attached hardware or loaded process matches it.

In the selected first step, the local motion grant fences control requests.
Fleet Board and clients show the busy state. Managed deployment checks it before
changing either host. A direct SSH write can still bypass the busy signal.
Audit owned state before and after the run, and check for drift during it.
Observed drift makes the physical receipt incomplete and requests immediate
Stop. The audit cannot prove that no temporary or unobserved write occurred.

For unattended motor tests, the owner selected temporary developer-write
restrictions on both hosts. This requires separate service owners, narrow
deploy rights, and a tested emergency admin path. A Fleet Board hold, UI
warning, or passing drift audit does not enforce the restriction. The
independent-safety-system path is not selected. No unattended scenario gets
motion authority until the selected path passes on both hosts.

### Servo-bench host access baseline

A read-only host check is not proof that either host is ready for qualification.
Record who can write what on both hosts before changing accounts or rights.

If LAN SSH also accepts connections, a tailnet rule alone cannot enforce the
qualification write boundary. Where RosieOS services and SSH sessions share one
account with passwordless sudo, and that account owns parts of the release
root, a developer using that identity can change those files without sudo.
Service identity and release ownership must change before that account can
become read-only to managed state.

On the RevPi, RTCore and rt-control run under separate service identities from
a root-owned `/opt/rosie-rt-core` tree. The managed RTCore release path still
calls `sudo -n` for install, activation, verification, and rollback. Removing
the RevPi login's sudo now would break that path.

A named account whose `sudo -n` asks for a locked password is not an admin
path; neither is a locked root account. An enabled console login service does
not prove that a person can recover the host at a physical console. Until an
admin path independent of the tailnet and the motion peer is verified, the
cooperative busy signal can proceed, but changes that could remove the only
tested admin route are blocked. Keep developer accounts and sudo rights
unchanged until a separate route reaches both hosts, runs Stop, and restores a
known release.

The owner selected temporary developer-write restrictions on both hosts for
unattended qualification. No restriction is installed yet. The emergency
admin route is also unverified. Use this order:

1. Give managed releases, services, and configs dedicated owners on the motion
   peer. Keep developer checkouts in a separate writable root.
2. Replace broad deploy-time sudo on both hosts with fixed, authorized operations.
   Preserve install, activation, verification, rollback, and emergency Stop.
3. Establish and test a break-glass administrator path before changing SSH rights.
4. During a qualification window, remove developer sudo and managed-path writes
   on both hosts. Cover LAN SSH, local login, service APIs, and tailnet access.
5. Test each developer identity on each host. Attempts to change the active
   release, config, systemd units, service APIs, or device settings must fail.
   Test an existing SSH session as well as a new one. Developer checkouts stay
   writable outside managed paths.
6. Test the emergency admin path during the restriction window. It must reach
   both hosts without a developer identity, run Stop, inspect state, restore
   the previous package and config, and recover access if the gate fails.
7. Confirm that the managed release can still install and roll back through
   its narrow authority. Recheck service ownership and loaded identities.
   Admit clean-bench qualification only after these checks and a clean baseline.
8. Restore normal developer access after final-safe state and receipt capture.
   Verify that the temporary restriction left no stale account or unit state.

A unit or directory name is not proof of the selected profile. Read the
unit's description and the loaded identity. Do not delete old deployment
directories until the active, rollback, and evidence references are
identified. Review the effective settings of every live drop-in before
changing service ownership or the bench profile.

Before changing the hosts, keep a verified copy of the active package and a
private runtime backup off the pair: the loaded compiled configuration,
runtime environment file, systemd units and drop-ins, and every referenced
Home anchor directory. Check each copied file's hash against the host. A copied
`rt-package verify` run in an ARM64 container checks package integrity; its
host-prerequisite result does not describe the RevPi. A sandbox install of
the saved package and runtime backup, and a sandbox `--rollback` from a newer
candidate, test filesystem reconstruction and the installer's `current` and
`previous` selection. They do not start systemd, restore the live control
environment, or test the Jetson side. The full pair rollback gate stays open
until those are tested.

Do not revoke access on the live pair until the current users, service owners,
deploy path, and maintenance window are confirmed.

## Exploratory development

An agent can edit, build, deploy, and probe broadly in simulation.
A development lease can permit bounded experiments on the servo bench.
Do not run a general autonomous edit-to-physical-motion loop on a full robot yet.
An exploratory result is not qualification evidence. Save its trace and action
sequence, connect it to a Linear issue, then turn important repeatable failures
into deterministic scenarios. The issue's status does not qualify a candidate.

## Results and receipts

Use pass, fail, incomplete, not_applicable, and quarantined.
A required scenario must pass.
Runner failure, timeout, missing evidence, or lost connection is incomplete.
An assertion failure is fail.
A quarantined required scenario needs replacement evidence or blocks the claim.
Do not retry a product failure until it passes.
One classified infrastructure failure can receive one controlled retry; save both attempts.

Physical scenarios have separate behavior and final-safe verdicts.
Both must pass. Observe through the declared post-command period.
Stop, disarm, lease loss, and hardware protection remain immediate.

Each receipt binds candidate ID, source SHA, policy digest, scenario identity,
target and configuration identity, runner identity, times, result, and evidence digests.
Store large logs and traces as separate content-addressed assets.
A rerun writes a new receipt.
The first simulation runner writes content-addressed local receipts and logs.
Central retention and synchronization remain open.

Reuse evidence only when relevant candidate, scenario, policy, environment,
configuration, description, hardware, and producer inputs remain unchanged.
An exception needs issue, claim, reason, approver, scope, and review condition.
It never changes fail into pass.

## First simulation path

The runner accepts a verified build-host candidate directory.
It calls the existing verifier and checks the candidate, source checkout, and plan identities.
The source runner executes selected source scenarios from that checkout.
It writes candidate-bound, content-addressed receipts and logs.
The HIL input check requires a passing receipt for every required source claim.
A test declaration alone does not count as passing evidence.
For candidate commands, the default plan compares the candidate commit with the
reviewed start in quality/qualification-comparison-starts.json. The runner
requires that start to be an earlier ancestor of the candidate. It rejects an
unknown target or a supplied plan with a different start. A PR plan may still
use --commit to inspect one commit; that is not a candidate qualification plan.

The first start for the servo bench's target profile is 7f66ed7f. Its passing simulation
receipt does not make it stable or qualified for hardware use. The start only
sets which source changes the plan inspects. Advance it only after a separate
review of the evidence and the remaining claims. An older candidate keeps its
original source and result; this change needs a new candidate to test the fixed
runner path.

It runs selected fixed simulation commands and writes one receipt per attempt.
Evaluation returns simulation-qualified only when all required simulation claims have matching pass receipts.
This state covers the simulation layer. It does not prove source, servo, or robot claims.

The PR fixture job uses the existing build-host candidate format and a small synthetic
archive. It runs the selected native simulation scenarios on Linux, then checks that
receipt tampering fails verification. The runtime-restart scenario kills and restarts
the simulated RTCore process. It checks that old permissions and plans do not return.
This proves the handoff and runner path for mapped claims. If the source plan
requires an unmapped claim, the fixture reports simulation incomplete. The fixture
does not prove the cross-build, GitHub release event, or target installation.
The separate cartesian CI job runs composed motion tests. A real candidate
whose plan requires motion-command-ordering also runs that scenario.

Run simulate or evaluate with --candidate-dir PATH and --receipts-dir PATH.
The native simulator builds source from the candidate's exact commit.
It does not execute the ARM64 hardware binary.
The receipt must not claim hardware package execution.
Publication of an rt-core-candidate release triggers the first simulation workflow.
The workflow verifies the downloaded candidate and retains receipts as an Actions artifact.
This is the first simulation proof, not long-term durable receipt storage.

### Published candidate evidence, 2026-11-10

The build host staged candidate `sha256:c4eeaebb6054af88104708b2ad7b558d10de0cd0c23422feeca0623333dc99a8`
from main commit `e7258d4bb22b3f1a654e65e1fe21b3169294dcb2` for
the servo bench's target profile. The package hash is
`a66cecc3ea01725bc075fdd059d68dbabc381efad92159e0f630dff5a7840d94`.
The local candidate verifier accepted the copied, staged assets. The build host then
published the exact candidate as a GitHub prerelease (tag `rt-core-candidate-main-e7258d4bb22b3f1a654e65e1fe21b3169294dcb2`).
Its publication receipt binds the source commit to passing release-request run
`42857759653` and RTCore run `42857759619`. GitHub reports the expected package
hash and seven uploaded assets. Publication did not install the package.

An offline Linux run used a clean checkout at the source commit. It had no
network or device access. The qualification unit and Linux integration fixture
tests passed 23 of 23 cases. That fixture uses a synthetic candidate. The
admission tests passed 9 of 9.
The cumulative plan from `7f66ed7f7a1430b5a2da4e818326f15f424d634b`
has no unknown paths. It still has four uncovered source claims, three
servo-HIL claims, and one robot claim.

The separate `node quality/qualification.mjs simulate --candidate-dir /candidate --receipts-dir /evidence`
run used the staged build-host candidate for the selected `native-sim-home-arm`
scenario. Its content-addressed receipt and log are kept in the private
repository. Independent `evaluate` returned `simulation-qualified` with
zero missing simulation claims under policy hash
`5d13e07c67aa49c556ab62d1476bc58c4a74439b73b880b08d886e1d7396a7af`.
This receipt covers Home and Arm simulation only. It does not cover joint or
Cartesian jogging, release-triggered simulation, target installation, or
physical behavior.
This older candidate has no candidate-bound source scenario receipts.
It cannot pass the newer HIL input check without those receipts.
`hil-inputs` remains blocked with `hil_scenarios_missing`.

The release-triggered simulation run (`42875435272`)
passed after GitHub fetched the published package. Its retained receipt has
hash `66661584ad57085c98a4ca371f307e53f83b5b2f476fddb24f1619ffaf55f863`;
its log has hash
`5262b2309ad8bf123e3b890afac20dc244fd554774d71676f3e9ebe6ef70a450`.
The receipt names the same candidate ID, source SHA, target, scenario, and
policy hash. Its evaluation is `simulation-qualified` with no missing
simulation claim. The Actions artifact is retained by GitHub's artifact
policy; it is not a permanent qualification store.

### Published candidate evidence, 2026-11-11

The build host published candidate `6c3e0438` (tag `rt-core-candidate-main-6c3e04387915197107a27067cfe67916b24051dd`)
as a GitHub prerelease for the servo bench's target profile. GitHub reports seven assets,
including package SHA-256
`34f7b0c86840d02bea9e512f7e051298b09662ab0468247a3fe8a224c84026ae`.
The publication receipt binds four passing GitHub workflows to the same main
commit. The release-triggered simulation run (`43118220151`)
passed. Independent evaluation of its content-addressed receipts returned
`simulation-qualified` with zero missing simulation claims.

The cumulative plan from reviewed start `7f66ed7f7a1430b5a2da4e818326f15f424d634b`
has zero unknown paths. Its selected source scenarios passed, but
`evaluate-source` remains `incomplete` for `compatibility-contract`,
`resource-budget`, `security-boundary`, and `timing-budget`. The current
`hil-inputs` command therefore reports `blocked`; it must not be called an
install-only verdict. The candidate has not been installed on the servo bench.

## Prepare physical qualification

A published candidate at 7f66ed7f passed simulation under the earlier
one-commit comparison rule. A later candidate exposed a classification
defect. The e7258d4b candidate includes fixes from #228 and #229 and has a
passing release-triggered simulation receipt. This proves that the published
candidate path works under the cumulative comparison rule. It does not prove
physical qualification or a stable fallback.

Run this command to compare a source plan with a declared cell profile:

```sh
node quality/qualification.mjs hil-fit --commit HEAD \
  --add-class robot-cell-config \
  --cell rt-core/config/cells/<cell>.json
```

The report uses the existing cell, machine, and robot description files.
It can distinguish the 1400 and 1420 bench profiles when a scenario requires
an exact machine profile. `match` means only that declared capabilities match.
`unknown` means a required HIL claim has no scenario.
The report always sets `admission` to `not_evaluated`; it does not contact
hardware or authorize a run.

Before a servo-HIL scenario contacts the host, run `hil-inputs` with
`--candidate-dir`, `--plan`,
`--receipts-dir`, and `--cell`. It verifies the local candidate bundle,
checks that the plan and checkout name the candidate's source commit, checks
content-addressed simulation receipts, and compares the candidate's target
profile with the cell profile. The output separates failed input checks from
physical admission. It always sets `admission` to `not_evaluated`.
This command does not publish a candidate or contact a cell. Source PR checks
remain a separate requirement.

The managed release command already holds a relay maintenance lock during
installation. That lock refuses Arm, so it cannot be the lease for a moving
HIL scenario. `rtctl hostcheck --expected-release FILE --json` can compare
the installed RTCore services, loaded binaries, and EtherCAT owner with an
expected release. It does not audit every RosieOS-owned path or prove a
qualification lease. The dashboard checks a terminal controller observation,
but its remote snapshot is not local HIL admission evidence.

Before an automatic HIL run, add a local admission adapter for the whole
bench. It must prove the active motion grant, the busy signal, the baseline of
owned surfaces, the exact installed candidate and configuration, live health,
and a final-safe starting state. It must label host-write assurance as
`cooperative`, not `enforced`.
Then add one bounded HIL scenario and a local receipt with separate behavior
and final-safe verdicts. A remote job or receipt must not release a lease
when local final-safe evidence is missing. Keep target installation in the
existing managed release transaction. Do not add a second deploy path.
Before the first physical motion run, prove the published candidate path.
Require passing simulation receipts for that same candidate.

The draft busy view does not supply that local lease or final-safe proof.
No HIL motion command or physical qualification receipt is in this slice.
A read-only busy view cannot authorize a moving scenario.

### Motion-free bench admission rehearsal

The first physical scenario is `installed-identity-observation`. It sends no
motion or drive command. Its cell-local command checks one current Fleet Board
hold for the `motor-testbench` resource on the cell's motion peer. It uses a
snapshot-only credential. It verifies the candidate, matching simulation
evidence, the target profile, the installed RTCore package hash, two active
control services, and two disarmed controller observations. The publisher
service may be inactive in this read-only rehearsal. The observations must have
no active grant, enabled axis, fault, or commanded cell I/O output. A local
file lock stops two copies of this command from running together.

The command returns `motion_free_observation_only` or `blocked`. It writes a
content-addressed local rehearsal receipt. The receipt always says
`qualificationState: not_qualified` and `hostWriteAssurance: cooperative`.
Offline tests reject missing controller incarnation, malformed grant and queue
fields, a controller restart during observation, unsafe outputs, a lost hold,
and a second local rehearsal that finds the lock held.
It cannot authorize motion, prove an unchanged host, or promote a candidate.
It does not verify the Jetson application or client generation. The first
build-host candidate covers RTCore only.
The Fleet hold does not prove the requester's identity. A developer with the
shared Jetson account can still bypass this command. A process crash can omit
the final-safe receipt; only the motion-free read is allowed in this phase.

Run `quality/hil_admission.py` on the motion peer from a clean RosieOS checkout
at the candidate source commit. Supply the verified candidate directory, its
simulation receipts, the cell file, one manually reserved Fleet job, and a
private file containing only `FLEET_SNAPSHOT_TOKEN`. The token file must have
mode 0600. The command refuses stale or conflicting reservations. It also
requires strict SSH host-key checking when it reads the paired RevPi.
It checks the local checkout before it runs qualification code or reads Fleet
and the device. A dirty checkout or a different commit blocks the rehearsal.
This check detects drift; it does not stop a privileged user from changing the
host after the check.
The cell's existing `expected_slaves` field sets the total bus count.
The live bus view must account for each drive beyond the selected axes as a
safe held slave. No second hardware inventory is maintained for this check.

Replace `<cell>` below with the reviewed bench cell file. When that file is a
new source selection, the live bench remains on its old configuration until
managed installation. Review the drive map, configuration pin, reservation,
and rollback first.

The operator must first publish a candidate from a merge to `main`. A PR does
not publish a candidate. Download that candidate and its simulation receipts.
Use the existing managed release command to install the exact candidate on
the bench's Jetson and RevPi. Reserve `<motion-peer>:motor-testbench`
manually in Fleet Board. Use the deployed Fleet Worker and its distinct
snapshot-only secret. Then run this read-only command on the motion peer:

```sh
python3 quality/hil_admission.py \
  --cell rt-core/config/cells/<cell>.json \
  --candidate-dir /path/to/published-candidate \
  --receipts-dir /path/to/simulation-receipts \
  --rehearsal-dir /path/to/local-rehearsal-receipts \
  --install-only \
  --publication-receipt /path/to/build1-publication-receipt.json \
  --fleet-url https://YOUR-FLEET-HOST/api/snapshot \
  --fleet-token-file /path/to/private-snapshot-token \
  --job-id job-YOUR-RESERVATION
```

Run it from the candidate's clean RosieOS checkout. Keep the candidate and
receipt directories outside that checkout. A blocked result is expected until
all prerequisites exist. The receipt records a failed or incomplete attempt;
it does not grant a later retry. The operator must review each receipt and
the final disarmed state before releasing the manual reservation.

`rtctl inventory` requires an unowned EtherCAT master. It cannot scan the bus
while RTCore owns that master. The rehearsal reads live `rtctl status` and
`describe` instead. Those readings and package verification do not replace
`rtctl hostcheck --expected-release` or a full owned-path baseline. The
expected-release manifest still needs an independent candidate-derived owner.
Do not call this rehearsal clean-bench qualification.

The local gate compares the cell file's configuration pin with the
configuration digest the RevPi reports, and blocks a mismatch. Do not change
the cell pin or running configuration just to make the check pass. First
determine which configuration is intended for the candidate.

To read the installed state without changing it, check that
`rosie-rt-core.service` and `rosie-rt-control.service` are active, and read
the installed source from `readlink -f /opt/rosie-rt-core/current` and its
`rt_package.json`. The running RTCore and RTControl commands report the axis
count, machine digest, configuration digest, and pair label. The pair label
alone does not identify the model. If `rtctl describe --json` cannot read the
control socket as the RevPi login, run the installed `rtctl` with `sudo -n`,
`describe --json`, and the socket `/run/rosie-rt-core/public/control.sock`
to read the model, description digest, bus positions, signs, travel limits,
and brake wiring state.

Compare each axis's software scale with the robot definition: motor counts
per turn times gear reduction, per radian. Agreement between source and
controller readback does not verify which physical motor is attached at each
position, its actual gearing, direction, or brake wiring. One read-only
`rtctl status --json` snapshot is not the two-observation motion-free
rehearsal for a new candidate. Neither API response proves the physical motor
labels, direction, scaling, or brake wiring.

## Servo-bench acceptance contract

This contract covers only one servo bench: its Jetson and the attached RevPi.
It does not qualify a robot cell, another test bench, an assembled robot, a
client, or welding. The first candidate recipe contains RTCore only. Use one candidate
ID and one source SHA through the gates. Save failed and incomplete evidence.
Do not convert a motion-free receipt into a servo-HIL pass.

| Gate | Pass threshold and evidence | If the threshold is not met |
| --- | --- | --- |
| Source and candidate | The candidate comes from merged `main`. The publication tag, source SHA, build-host recipe and environment, the bench's target profile, package SHA-256, and candidate ID agree. Four exact-SHA publication checks pass: release request, admission, RTCore, and RTCore consumers. Keep the release assets and worker receipt. | Block installation. A green PR or local build is not a published candidate. |
| Cumulative plan | The candidate plan compares the candidate SHA with reviewed start `7f66ed7f7a1430b5a2da4e818326f15f424d634b`. It has zero unknown paths. Every selected source scenario and every required simulation scenario has a passing candidate receipt. Save the JSON plan, policy digest, and list of broader claims still incomplete. | A failed selected scenario or missing simulation receipt blocks installation. Uncovered source, servo, and robot claims remain incomplete; they block the claims that need them, not a scoped motion-free install. Do not drop a claim to make the plan pass. |
| Simulation | `evaluate` reports `simulation-qualified` and zero `missingClaims` for this candidate ID. Every required scenario has a matching content-addressed pass receipt. Keep the Actions run URL, receipts, and logs. | Stop before host mutation. A fixture or an older candidate's receipt does not count. |
| Bench selection | One reviewed cell file selects the six-of-nine bench option. Its machine file, six drive positions, signs, scaling, brake wiring, robot description, and compiled configuration digest agree with the attached hardware. Record the owner's on-site attestation, reviewed source diff, and live readback. | Keep the option inactive. A source selection alone does not change the live configuration or prove that a new package installed. |
| Reservation and rollback | One current Fleet hold names `<motion-peer>:motor-testbench` and the job ID. The snapshot is no more than 30 seconds old and the hold has at least 60 seconds left. Preserve a verified package and configuration that can restore the pre-run bench state. | Block mutation or admission. A saved config alone is not a complete rollback package. |
| Managed installation | Run the existing candidate release command from a clean checkout at the candidate SHA. Its read-only `--plan` passes first. For a reviewed configuration switch, name the exact live pin with `--accept-config-drift` and keep `--rollback auto`. This authorizes the switch; it does not satisfy the bench-selection or health checks. After owner approval, the transaction verifies the package hash, installs, starts, checks the paired services, and writes a journal. The loaded RTCore and both host identities match the candidate receipt. | Stop and use the transaction's rollback path. Do not use `--accept-config-drift` or `--accept-hostcheck` to turn an unreviewed mismatch or failed check into a pass. |
| Motion-free rehearsal | `hil_admission.py` returns `motion_free_observation_only` with zero blockers and writes a content-addressed receipt marked `not_qualified`. Two observations, at least one second apart, keep the same boot, controller incarnation, and bus inventory. They show six selected axes, verified mask 63, no active grant, no enabled axis, no pending motion, and every output off. Each extra drive must appear as a distinct, present held slave in INIT or PREOP with a readable identity. | Keep the hold and inspect the fault. An unknown or stale value blocks; it is not a zero. Review final-safe state before release. |
| Commissioning for autonomous motion | Use the source robot model's main-branch travel, speed, acceleration, and following-error values for J1-J6. A person at the bench records the six drive identities, motor direction, scaling, brake behavior, guarding, Stop, lease loss, and fault recovery. The reviewer approves bounded test commands within those values and records a measured Stop-time limit. Temporary developer-write restrictions pass on both hosts, with a tested emergency admin path. Keep the signed review and measured baseline. | Do not run unattended motion. The owner's reported checks are not the measured baseline. Disabled collision-watchdog warnings in the profile need a documented reason or an independent guard, and the busy signal does not restrict host writes. |
| Autonomous joint-jog qualification | A cell-local runner holds motion authority and checks the same candidate, configuration, and reservation. It jogs each of six selected joints in both directions, one joint at a time. Every command stays inside the approved travel, speed, acceleration, following-error, and time limits. Require zero unexpected movement, zero active faults, and separate `pass` results for behavior and final-safe state. Save traces and one immutable receipt per scenario attempt. | Any missing limit, changed identity, lost hold, Stop failure, timeout, or absent final-safe observation blocks the next command. It cannot pass from a motion-free receipt. |
| Cartesian command check | In simulation, check positive and negative X, Y, Z, roll, pitch, and yaw commands against the selected robot model. After joint-jog qualification, a reviewed cell-local scenario may send bounded Cartesian commands to the bench. Compare planned joint paths with encoder traces and forward-kinematics results under pinned numeric tolerances. | The bench has no assembled arm or end-effector measurement. This check cannot claim physical Cartesian accuracy, collision clearance, or robot qualification. Do not interpret "moderate" as a limit. |
| Operations view | The operations dashboard's `rosie-observability.service` reports this pair's last observed candidate, source SHA, package and config digests, deployment journal result, receipt type, qualification state, and observation time. Values match local receipts and live host readback. A live daemon report older than 30 seconds appears as `unknown`, not `qualified`. | Do not show a green status from daemon metadata alone. Keep deployment and qualification authority on the cell. |

The six-of-nine bench has motors but no assembled arm. A servo-HIL pass proves
only the measured motor and controller behavior. It cannot satisfy the robot
floor in Phase 4. Phase 5 soak and Phase 6 welding need separate approved
scenarios and thresholds. Do not infer either from a bench receipt.

The reviewed motion profile must name each motor and direction. It must set
numeric maximum travel, speed, acceleration, following error, command time,
Stop time, and settle time. The Cartesian profile must also set numeric
translation, rotation, speed, and model-error bounds for each direction.
Every measured value must stay within its bound. The owner must document why
the disabled collision watchdogs are safe for this fixture, or add an
independent guard. Missing values or guard evidence block unattended motion.
The owner selected an enforced host-write boundary with a tested emergency
admin path. That selection authorizes design work, not an untested sudo or SSH
change. Keep unattended motion blocked until the boundary passes the tests
above on both hosts.

Check both hosts with `sudo -n -l` and a read-only `getent group sudo`. A
shared account with `NOPASSWD: ALL` can bypass a busy notice or lease. Do not
restrict such accounts until a separate administrator login and recovery
procedure work on both hosts.

Use the existing `rt-core/tools/benchdrive` directional jog path for each
single-axis attempt. It checks the live description, Home, coordinates, and a
conservative travel envelope before it enables. Its JSONL evidence records
measured travel. A qualification runner still needs to bind twelve attempts
to one candidate, hold, profile, and final-safe receipt. Do not treat the
tool's `--confirm-motion` flag as a commissioning or admission check.
OLP's Cartesian hold requires the machine-level `jog_jerk`. A bench machine
without it cannot run the hold as-is. Add and qualify a reviewed machine
profile, or use a different bounded Cartesian command contract. Do not quietly
substitute simulation success for bench execution.

A Stop measurement from another bench configuration is not a measured Stop
result for this profile. Neither is a source revision that removes a brake
override and restores the A6-EC software brake wait and hold check. Source
facts, such as a robot definition and URDF that agree on a reduction, do not
verify the attached motors.

### Bench review values from source

The bench cell's machine file maps the selected joints to EtherCAT positions
and declares their signs. The gear ratios and motor encoder counts per
revolution come from the source robot's `rtcore_definition.json`. The travel
bounds come from its URDF, not from a measurement on a bare motor bench. A
`following_error` the bench machine config labels unverified stays unverified
until measured. Do not copy another robot's bound into the bench profile, even
when the two definitions share ratios and signs.

The owner selected the source robot's main-branch values for physical bench
motion. Use them as the configured limits; do not require another table of
those values. Then compare the bench machine with the robot cell's machine for
the same robot, setting by setting: the robot description digest, axes, and
cycle; the bus and network (extra slaves held or refused, control profile);
scheduling; the motion guards (`jog_jerk`, tracking-lag allowance, drive
supervision, joint tolerances, and brake guards); and per-joint startup tuning
measured for another bench.

The shared description supports a source-level model and scaling comparison.
It does not establish matching Stop, tracking, or brake behavior. Review each
machine difference before claiming representative control behavior. Keep
bench-specific settings when the physical hardware requires them, and record
why.

The source values are not a motion approval. An owner's on-site report of
motor order, held positions, direction, scaling, and brakes, with no motor
serial table or measured trace, is not a measurement. Stop time, lease-loss
behavior, and a safe first-motion sequence need their own results. Keep all
motion blocked until a reviewer records those results and approves bounded
commands within the source limits.

A bench option that uses a robot cell's model as its source model has its own
cell file. Source selects the option for an install-only run. The live RevPi
does not change until a managed install. When the offline-programming target
catalogue does not expose the option as a client target, an RTCore-only
release does not qualify a client or its connection to the new pair.

Managed `release --plan` refuses a cell file before host contact when it has
no `cell_deploy` block. The bench option's block names the bench's existing
OLP and planner units, release directory, and listener; its program store is
separate from other profiles' stores. The managed release command accepts a
source-only dry run. The `release_plan_only` guard refuses apply before host
contact. This flag guards only the managed release command; it never
restricted other deploy commands or direct SSH. A profile selected for a
release sets the flag to false. Its `rt_core.activate=true` selects the paired
service, not motion approval.

Selectable bench profiles share one runtime slot on the bench host. Legacy
unit and directory names remain to preserve the existing rollback lineage. A
later profile switch must replace that slot and verify the loaded identity; it
must not leave a second OLP or planner pair running. Selecting a profile in
source permits a managed release attempt. Admission, a current hold, live
identity checks, and rollback remain separate gates. Build a new candidate
from the selected source commit. An earlier candidate's package remains
simulation evidence, not the bench candidate.

The cumulative plan through this PR now has no unknown paths. A narrow policy
rule classifies hash-named qualification receipts and logs under `docs/evidence`;
other unknown files still block. It has nine uncovered claims: four source,
three servo-HIL, and two robot. The existing managed-release
and candidate tests declare the source `deploy-contract` claim. They run in the
native `rt-core.yml` job. The publisher requires a successful push run of that
workflow for the candidate's exact source commit. The cell template selects
configuration claims; RT control source still selects control claims. These
tests check ordering, identity, and rollback with a fake remote; they do not
prove an installation. Before this PR, the publisher did not require the
`rt-core-consumers.yml` push run, although this PR changes operator code that
workflow tests. This PR adds that exact-commit publication check and runs the
consumer workflow on every main push. The source poller can admit its initial
commit, and manual intake can admit a docs-only commit, so a path filter would
leave a candidate without an exact-commit consumer run.
The PR admission workflow also runs on every main push, and publication now
requires its successful exact-commit run. This binds the admission and
qualification contract tests to the candidate source. The admission policy
remains in shadow mode: a successful workflow can report `wouldBlock`.
This PR also adds a source runner to the release-triggered qualification job.
It runs selected source scenarios from the candidate checkout and saves
candidate-bound receipts. `hil-inputs` rejects a missing or failed source
receipt. A passing workflow still cannot cover a source claim with no scenario.
The cumulative plan has four such source claims. Add focused tests or keep
those claims blocked; do not treat existing narrow tests as broad proof.
Receipt evaluation requires every selected scenario for a claim to pass.
One passing receipt cannot hide a missing or failed sibling scenario.
Existing publisher and plan-only tests check private credentials,
exact-commit publication, and release refusal. They do not prove that the
new bench manifest's authorization and credential settings are enforced.
Keep `security-boundary` uncovered until a source check proves that contract.
The motion-free HIL admission command refuses any uncovered source claim
before it reads the RevPi. Missing later servo-HIL scenarios remain visible;
they do not turn a motion-free observation into physical qualification.
The native cell compiler now refuses an unsupported deployment schema and
tests compiled configuration pins. This improves source validation but does
not prove the broader `compatibility-contract` claim. Map source checks to
the four remaining source claims.
Add physical scenarios for the three servo-HIL claims.
Keep robot acceptance incomplete after a bench pass. A physical receipt can
record a servo-HIL pass without making the candidate robot-qualified or beta.
Four remaining source claims come from broad text matches in the new cell and
machine files. Existing tests do not prove the selected authorization,
resource, timing, or cross-component compatibility contracts. The motion hold
synchronization fix also selects `operator-contract`, `operator-workflow`, and
`operator-input`. The composed browser and Deck test now declares the
simulation `operator-workflow` scenario. It tests source behavior on loopback,
not a packaged client or physical control. Require a matching candidate
receipt before counting that scenario as passed. The existing `denseexec`
package tests now declare the source `operator-contract` scenario. They test
operator commands and result handling against simulated RTCore. The source
runner must produce a matching candidate receipt before that claim passes.
Robot operator input stays open until an assembled robot is tested. Do not
use either software test as physical input evidence.

Add negative tests at each boundary. Wrong candidate or package hashes, a
changed policy, a missing simulation receipt, and an unknown path must block
before installation. A stale or duplicate hold, changed controller incarnation,
wrong configuration digest, active output, or lost local lock must block HIL
admission. During a motor scenario, Stop, lease loss, timeout,
drive fault, or an absent final-safe observation must fail or mark the receipt
incomplete. The runner must not resume the next direction after such a fault.
A stale dashboard report, conflicting receipts, or an unverified
loaded binary must show `unknown` or `mismatch`, never `qualified`.

### Commands and stop points

Run these source checks before a live attempt. The first two commands are
internal: their tests use the real bench cells, so the public mirror leaves
them out.

```sh
node --test quality/qualification.test.mjs quality/qualification.integration.test.mjs
python3 quality/hil_admission.test.py
node quality/qualification.mjs plan --base 7f66ed7f7a1430b5a2da4e818326f15f424d634b --head HEAD
node quality/qualification.mjs capabilities --cell rt-core/config/cells/<cell>.json
```

The plan command inspects the checkout commit. It does not verify a candidate.
The integration fixture skips on macOS; require its Linux CI run to pass.
For a published candidate, use the registry fetch and managed release commands
in `tools/build1-release-worker/README.md`. Run `simulate`, `evaluate`, and
`hil-inputs` with `--candidate-dir` and `--receipts-dir` from a clean checkout
at the candidate SHA. `hil-inputs` also needs `--cell`. Require matching
candidate, source, policy, target, and scenario identities, with no blockers.
Run `source` and `evaluate-source` before `hil-inputs` with the same candidate
and receipt directory. Check `missingClaims` in the source result. The source
runner exits nonzero if a selected test fails. An uncovered claim still reports
`incomplete`; the runner does not invent a test for it.
Run the managed release with `--plan` before removing that flag. Keep the
read-only plan output, install journal, service status, package verification,
and rollback result. The source-selected profile and live state must be
reviewed before either command can justify a configuration change.

Run the install-only preflight from a clean checkout at the candidate commit:

```sh
node quality/qualification.mjs install-inputs \
  --candidate-dir /path/to/verified-candidate \
  --receipts-dir /path/to/qualification-receipts \
  --publication-receipt /path/to/build1-publication-receipt.json \
  --cell rt-core/config/cells/<cell>.json
```

Require `inputStatus: install-inputs-checked` and no blockers. This result
permits a supervised install attempt after separate bench checks. It does not
admit motion or prove that installation succeeded.

For the motion-free gate, run the documented `hil_admission.py` command on
the motion peer after the install and manual reservation. Add `--install-only` and
`--publication-receipt PATH` for this scoped observation. Its tests enforce
the 30-second snapshot age, 60-second hold margin, two controller reads, and
the disarmed six-axis values in the table. A servo-HIL motion runner and true
qualification receipt do not exist yet. Build and test that local gate before
claiming a physical pass. The approved numeric motion envelope is an open
input, not a value to guess in code or in this document. Keep the first live
installation and observation motion-free. The autonomous scenarios can follow
only after commissioning and an offline fault-injection proof of their stop
behavior. An operator need not stand at the bench only after the owner verifies
that guarding, local Stop, and the selected containment path keep the fixture
safe without a person present.

After the run, check the operations dashboard's user service on its host and
read its local API:

```sh
systemctl --user is-active rosie-observability.service
curl --silent --show-error http://<dashboard-host>:18324/api/state
curl --silent --show-error --include http://<dashboard-host>:18324/api/busy
```

The service can listen on the host's tailnet address rather than on loopback.
Check the listener before using these commands on another host or after a
network change.
The managed release writes `cell-deploy.jsonl` under `~/.rosie/deploy-journal`
on the machine that runs the CLI. It is not a cell-local or dashboard-host
journal. A dashboard join must receive that exact result and compare it with
the candidate, live readback, and physical receipt. Daemon metadata cannot
replace the journal or the physical receipt.

Compare the displayed identity
and status with the candidate manifest, managed release journal, local HIL
receipt, and live paired-host readback. Run the dashboard's own tests. The
dashboard does not yet join candidate and qualification receipts. Until that
join is built, its qualification state must remain `unknown`.
A running dashboard without the busy route answers `/api/busy` with 404.
Update the running dashboard before using its busy view as deployment input.
Its ownership probe must look for `rosie-rt-core.service`, not the obsolete
`robot-v4-rtcore.service` name. A probe that reports a RevPi RTCore process
with `expectedPid: 0` and `serviceState: inactive/dead`, while a direct RevPi
check reports both required services active, disagrees with the host. Resolve
that disagreement before the dashboard reports a verified installed state. A
corrected probe reports complete coverage, the service's PID as
`expectedPid`, that PID as the one holder of `/dev/EtherCAT0`, and
`serviceState: active/running`. This checks the probe path, not the candidate
or physical qualification state. Test a dashboard update before the live view
uses it.

## Beta and later physical stages

The common robot floor proves installed identity, control ownership, Stop,
one bounded safe motion, fault-free observation, and final-safe state.
Change-specific scenarios add the claims selected by policy.
Run full robot qualification on one representative compatible bay.
Each other bay needs a smaller target-specific acceptance check before it follows beta.
Keep beta assignment manual initially.

Servo HIL needs a matching Jetson, RevPi, and servo bench.
Admission needs a motion grant, visible reservation, baseline check, and exact
candidate installation. The run needs capability matching, bounded scenarios,
post-command observation, final-safe verification, local receipts, and rollback.
Report clean/dirty/unknown state.
Do not start automatic servo HIL until those boundaries exist. Do not infer
clean-bench qualification from a cooperative busy signal.

Soak repeats approved behavior against a selected beta candidate.
It reaches final-safe state between cycles and can yield the resource.
Soak adds evidence to the candidate; it does not create another release.
Welding graduation is separate and manual after robot qualification.

## Historical requirements and open gaps

Robot V5 testplan and Gradient CLI Robot V4 are one-time research inputs.
Useful patterns include session identity, replay, recovery sequences,
operator input, physical acceptance, and final-safe evidence.
The old seven-stage Gradient CLI controller is not a compatibility target.
Current scenarios must use current RosieOS control and deployment interfaces.

An internal release-readiness review of 2026-09-30 identified target compatibility, transitions, late faults, measured endpoint
criteria, and release ownership as gaps.
The November 2 review named in the input plan was not found in the checked
repository or supplied files. Add it to the gap review when available.
Linear AMR-406 keeps real consumer and cell I/O qualification open.
AMR-718 records a J6 completion timeout and measured endpoint residual.
AMR-405 records bounded bench motion and recovery evidence.
These issues help select deterministic scenarios. Linear status never controls qualification.

The largest gaps are Stop, real-drive recovery, operator input, installed identity,
final-safe state, and welding I/O scenarios.
The source coverage map lacks claim-level equivalence for every suite.
PR and build-host checks alone do not qualify a robot candidate.

## Phases and acceptance

Phase 0 is the current-state audit and gap list.
Phase 1 is the central policy, colocated declarations, generated catalog, and shadow plan.
It succeeds when one command classifies representative commits and explains missing evidence.

Phase 2 runs deterministic simulation scenarios for one verified candidate.
It succeeds when a merged candidate becomes simulation-qualified without manual receipts.
This first slice supplies a runner, evaluator, and publication trigger.
Only mapped simulation claims can pass.

Phase 3 adds representative servo HIL. The servo-bench acceptance contract defines
its first motion-free gate and the later autonomous motor gates. Motion-free
observation alone leaves Phase 3 incomplete.
Phase 4 adds robot qualification and manual beta assignment.
Phase 5 adds beta soak.
Phase 6 adds manual welding graduation.

The policy, receipt rules, and test declarations for later phases can be built
before the first live run. Do not enable automatic robot motion, beta promotion,
soak, or welding from an unproven Phase 3 authority path. Test each source
contract as it is added; use representative PRs later to prove the full path.

Before admission enforcement, compare historical and current changes with expected classes.
Prove that unknown paths and failed policy checks would block.
Keep branch protection unchanged until shadow results are reliable.
