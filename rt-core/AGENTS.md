# RT-core work tracking

Home is established only by drive homing (HM35), which writes 607C; the saved
v3 HM35 anchor is a check and backup, restored only through direct drive checks,
RestoreAnchor or verified-delta for v3. There is no software current-pose zero;
refuse `rosie-anchor-v5` files. Software must never route around a drive or
hardware failure (for example a drive that fails HM35): surface the failure and
fix it at the drive rather than adding a capture, override or alternate reference.
Never re-home to clear a fault. HM35 makes the current position Home, so it is
run only by an operator at the joint's mechanical home pose to establish Home,
never as a recovery shortcut. A fault that leaves encoder coordinates trusted
(e.g. target_lead, AMR-720) must not invalidate Home; it resets once its
condition clears. Only genuine reference loss (coordinate_reference, untrusted
evidence, encoder/battery alarm) may require a new HM35.
Every A6-EC axis uses absolute linear mode
(`C00.07 = 1`) with the motor-to-axis gear ratio applied in software and
standard `0x6091:01/02 = 1/1`. The A6-EC HM35 transaction writes `607C = 0`;
its other vendor-confirmed writes and ordering stay fixed. Do not add dwells,
preconditioning stages or per-drive variants. Rotation mode, wrapped feedback,
wrapped commands, batteryless single-turn mode 3 and public `feedback_wrap`,
`command_wrap` and `anchor_period_counts` fields are retired. Coordinate
evidence, retained Home and verified-delta compare complete signed counts
exactly; a whole-turn disagreement must refuse. This migration stays on its
branch until the single-axis mini bench passes servo-rate qualification and
the target cell is ready. A migrated drive needs a power cycle and operator HM35 on every
joint after the approved full release.
Nothing is joint-specific: joints differ only by gear ratio, so every limit,
tolerance, timeout and check is one joint-unit value applied to all axes and
converted to counts by gear ratio; if one joint needs more, all joints get it.
Exception (owner decision): drive servo tuning is per-joint machine
configuration, limited to the drive's tuning catalog (config/drives/
a6ec.tuning.json), read at every start, written only where it differs (each
A6-EC SDO write is an EEPROM write), read back and part of the configuration
hash; drive auto-tuning may only measure. Safety bounds stay one
joint-unit value for every joint.
`max_target_lead` is one joint-angle value for all joints. A cell whose machine
configuration still lists it per joint (values copied from bench and v4
constants) measures it on the loaded arm and replaces it with one joint-angle
value for all joints before release. Do not tune it per joint in the meantime
(GitHub issue #158). The target-lead and following-error bounds also grow with
commanded speed by the machine-level `tracking_lag_allowance` (one time for
every joint). The machine-level `jog_jerk` is one value for every joint and
also bounds OLP Move-to and Cartesian steps via `Compiled.JogJerk`. Drive speed
feedforward (C01.13/14/15) is a per-joint tuning key. Measured on a Rosie 1430
arm: with 0/0/318 on every joint, every joint read 3-9 permille 80-500 Hz
torque, lag ~27 ms; with 1/500/318 J2 oscillated at 100 Hz up to 557 permille.
A configuration may omit tuning startup overrides to retain drive-held values.
Deploy permits removal only for keys classified as tuning by the new compiled
catalog; other removals remain refused. At startup rt-core observes the tuning
objects a configuration does not write (docs/drives/a6ec/README.md, "Tuning a
joint").
Deploys and any build or test on a cell's hosts use only the repo commands
under the cell lock: ../CLAUDE.md, "Deploying and testing on a robot cell",
and ../docs/DEPLOY_AND_TEST_WORKFLOW.md.
No drive-, robot- or machine-specific number may be hard-coded in rt-core
source. Limits, tolerances, timeouts and drive transactions live only in config
files (drive profile, robot definition, machine config) and reach the runtime
through the rtctl compiler; code holds only generic mechanics such as unit
conversion. Remaining unrelated legacy A6-EC literals are tracked in
issue #160 (internal tracker)
and P4-18; do not add new ones or fold that wider cleanup into P8-45.

Read README.md and docs/KICKOFF.md before implementation. The authoritative
backlog is docs/IMPLEMENTATION_PLAN.md; docs/tracker.html contains its more granular
work packages and preserves their existing IDs. Phase numbers are not execution
order: minimum P0 decisions, P2 corrections using the original harness, P3 lanes,
P1 test parity and port, then incremental P4/P5, P6/P7 and P8.

When a work package advances or completes, update its committed ITEMS data in
docs/tracker.html with stage, owner role/person, evidence and implementation commit
where applicable. Keep the matching plan, docs/ARCHITECTURE.md, docs/APPLICATION_API.md and the
tracker evidence consistent; the validation record is archived under docs/history/.
Do not mark a fix Verified merely because a design was written or a commit landed.
Preserve existing stages, IDs, evidence and browser draft storage. Browser-only
edits are not repository updates; export the data block and commit it.

Keep Linear project `rt-core: standalone realtime core` synchronized with the
committed tracker. Match issues by the existing work-package ID (for example,
P8-27), preserve history, and mirror stage, owner, evidence, implementation
commits and dependencies. Check for an existing issue before creating one.
Update both records when work changes; report any failed synchronization.

Keep the existing Python tests as the oracle through P2/P3. Port against the same
corrected daemon/configuration and demonstrate parity before removing them. Final
G2 requires no owned Python in the new component/control path/tools/tests; G0/G1
do not depend on language removal. The static HTML/JavaScript documentation
tracker is not a runtime or control-path component.

Do not give time estimates, work-duration estimates, ETAs or time-based sizing.
Express sequence through dependencies and concrete acceptance criteria. Keep
execution latency and other measured engineering budgets explicit; these are
not work-duration estimates.

For gates in a Windows-created sparse worktree under WSL, the Windows `gitdir:` path in the worktree's `.git` file prevents Linux Git from reading the pinned Jetson URDFs through the existing sparse-entry fallback. Use an executable, ignored `rt-core/build/test-bin/git` bridge: resolve the effective directory including `-C`, find its enclosing `.git` pointer, translate a Windows drive path with `wslpath -u`, then exec `/usr/bin/git --git-dir="$translated_directory" --work-tree="$worktree_root" "$@"`; pass ordinary Linux repositories, explicit `GIT_DIR`, and `init`/`clone` fixture commands directly to `/usr/bin/git` unchanged. The bridge is local validation tooling and must never be committed or rewrite Git metadata. From `rt-core` in WSL, export `PATH="$PWD/build/test-bin:/home/operator/.rosie/go-1.26.2/bin:$PATH" GOCACHE="$PWD/build/go-cache" GOMODCACHE="$PWD/build/go-mod-cache" GOTMPDIR="$PWD/build/go-tmp" GOFLAGS=-mod=mod TMPDIR=/dev/shm/rt-core/drivealarm`, create those cache and temporary directories, confirm `git check-ignore build/test-bin/git`, and run `make control sim && make test && make test-go && make test-sanitize && make check-protocol && make check-api`; `go test -race -count=1 ./tools/rtctl -run 'TestRobotMigrationParity|TestRobotResourceRepositoryBoundary'` exercises the sparse-resource fallback and repository-boundary regressions with this bridge.
