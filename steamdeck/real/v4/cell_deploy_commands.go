package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"os"
	"path"
	"path/filepath"
	"strings"
	"time"
)

// cellDeployRealtimeCoreCommands are the realtime-core subcommands driven by
// the cell manifest's cell_deploy block (runRealtimeCoreCell).
var cellDeployRealtimeCoreCommands = []string{"release", "build", "precheck", "wait-idle", "copy-dumps", "verify", "restart", "rollback", "reboot", "resume-after-reboot", "lock", "prune"}

const cellDeployRealtimeCoreUsage = `Cell-driven steps (need the manifest's cell_deploy block; see README "Adding a new robot cell"):
  release One command from a commit to a verified cell, under the cell lock
          throughout: reads what the cell runs, picks the smallest tier
          (config, olp, full), runs its steps, one timing table
          --commit <sha> [--tier auto|config|olp|full] [--test fast|full]
          [--plan] [--rollback auto|never] [--accept-hostcheck ids] [--regate]
          [--rollback-to-configuration sha256] [--clean]
          [--install-planner-env] [--idle-wait 10m] [--quiet 2m] [--dry-run]
          Run again for the same commit: resumes a stopped full release and
          reuses a passed test gate (docs/DEPLOY_AND_TEST_WORKFLOW.md)
  build   Build one commit into a deployment tree on the relay, in the cell's
          persistent build tree (exactly the commit, clean): rt-core
          live/control/sim, cell compile = pin, package + verify, OLP, UI
          stub, Cartesian resolver, planner env; installs nothing. Runs under
          the cell lock at normal priority on every core: waits for the cell
          to be idle, then both maintenance notices are up (no arming) until
          it ends [--commit HEAD] [--previous dir] [--test] [--regate]
          [--test-level full|fast] [--test-since c1,c2] [--clean]
          [--known-failures f] [--install-planner-env] [--allow-compiled-drift]
          [--fetch-package] [--idle-wait 10m] [--quiet 2m] [--interval 15s]
          [--maintenance-eta d]
          [--low-priority (no lock, nice 15; a relay serving no robot only)]
          [--dry-run]
  lock    Run any command under the cell lock (agent tests on the relay):
          --reason TEXT [--eta 30m] [--on local|relay|servo] [--idle-wait 10m]
          [--dry-run] -- COMMAND...
  precheck
          Read-only gate before stopping a running cell: disarmed, no lease,
          no faults, OLP idle, stop dumps copied, nothing in D state
          [--quiet 2m] [--dry-run]
  wait-idle
          Read-only: poll until disarmed, unleased and no OLP activity for
          --quiet [--quiet 10m] [--limit 25m] [--interval 60s] [--notice]
          [--dry-run]
  copy-dumps
          Copy stop dumps off the servo host (nice 19, ionice idle, throttled,
          sha256-checked; never decoded there) [--to relay|mac] [--dry-run]
  verify  Read-only post-start acceptance: pin, every axis and startup
          readback verified, Home restored, disarmed, no faults, anchors
          unchanged since precheck [--home-wait 20s] [--expect-capabilities d]
          [--expect-installed (the control.env pin, not the manifest's)]
          [--dry-run]
  restart Controlled restart of the installed release and configuration
          (never changes either): cell lock and idle wait, copy-dumps,
          refusals (armed, leased, in use, D state, uncopied dumps, faults),
          clean systemctl stop of rt-control then rosie-rt-core, start on
          the installed pin, Home restored (never re-homes), verify
          --reason TEXT [--clear-drive-alarms (owner inspected the cause)]
          [--home-wait 45s] [--idle-wait 10m] [--quiet 2m] [--dry-run]
  rollback
          Tier 3 rollback: stop, install.sh --rollback, restore the control.env
          backup of the configuration that ran before, start disarmed, verify
          --to-configuration <sha256> [--home-wait 20s] [--dry-run]
  prune   Delete old rt-core releases on the servo host with the installed
          install.sh --prune: keeps current, previous and pinned known-good
          releases; never touches units [--pin rev,...] [--dry-run]
          (a full release runs it after verify)
  reboot  Reboot the servo host: ONLY with the owner's approval
          --owner-approved --reason TEXT [--dry-run]
  resume-after-reboot
          Start disarmed after an approved reboot: refuses unless booted after
          --booted-after, units inactive, master Idle with the machine's slaves;
          then verify [--home-wait 20s] [--expect-capabilities d] [--dry-run]
All take --manifest cell.json and --transport tailscale|ssh; <command> --help
lists a command's flags.
`

// runRealtimeCoreCell runs one of cellDeployRealtimeCoreCommands.
func runRealtimeCoreCell(command string, args []string, stdout, stderr io.Writer) error {
	switch command {
	case "release":
		return runRealtimeCoreRelease(args, stdout, stderr)
	case "rollback":
		return runCellDeployRollback(args, stdout, stderr)
	case "build":
		return runCellDeployBuild(args, stdout, stderr)
	case "precheck":
		return runCellDeployPrecheck(args, stdout, stderr)
	case "wait-idle":
		return runCellDeployWaitIdle(args, stdout, stderr)
	case "copy-dumps":
		return runCellDeployCopyDumps(args, stdout, stderr)
	case "verify":
		return runCellDeployVerify(args, stdout, stderr)
	case "restart":
		return runCellDeployRestart(args, stdout, stderr)
	case "reboot":
		return runCellDeployReboot(args, stdout, stderr)
	case "resume-after-reboot":
		return runCellDeployResume(args, stdout, stderr)
	case "lock":
		return runCellLock(args, stdout, stderr)
	case "prune":
		return runCellDeployPrune(args, stdout, stderr)
	}
	return fmt.Errorf("unknown realtime-core command %q", command)
}

// Expected run times shown on the maintenance overlay once the notice is up,
// the idle wait excluded. Unverified: install-olp.sh restarted a Rosie 1430 cell's OLP
// and passed its checks within minutes (2026-10-06/07, not timed); a
// RevPi servo host reboot came back within reboot.sh's 600 s budget (2026-10-07,
// not timed), and resume then starts and verifies (start waits up to 60 s).
const (
	cellDeployOLPEstimate    = 5 * time.Minute
	cellDeployRebootEstimate = 15 * time.Minute
)

// cellDeployNotice is this run's maintenance notice (rt_core_maintenance.go);
// rtCoreMaintenanceWrap sets it before anything else and clears it on exit.
func cellDeployNotice(ctx cellDeployContext, command, reason string, f rtCoreMaintenanceFlags, estimate time.Duration) rtCoreMaintenance {
	m := newRTCoreMaintenance(command, ctx.CellID, f, estimate)
	m.Reason = reason + " on " + ctx.CellID
	m.SetBy = strings.ReplaceAll(command, "-", " ") + " (" + rtCoreMaintenanceOperator() + ")"
	return m
}

func cellDeploySeconds(name string, d time.Duration, min, max time.Duration) (int, error) {
	if d < min || d > max || d%time.Second != 0 {
		return 0, fmt.Errorf("%s must be whole seconds between %s and %s", name, min, max)
	}
	return int(d / time.Second), nil
}

func cellDeployAbsFlag(name, value string) error {
	if value == "" {
		return nil
	}
	return cellDeployCleanAbs(name, value)
}

// --- build -----------------------------------------------------------------

type cellDeployBuildProbeFacts struct {
	Previous           string            `json:"previous"`
	PreviousHead       string            `json:"previous_head"`
	Deployment         string            `json:"deployment"`
	DeploymentExists   bool              `json:"deployment_exists"`
	DeploymentComplete bool              `json:"deployment_complete"`
	DeploymentHead     string            `json:"deployment_head"`
	IgHPkgconfig       bool              `json:"igh_pkgconfig"`
	Tools              map[string]string `json:"tools"`
	FreeMiB            int64             `json:"free_mib"`
	Package            string            `json:"package"`
	PackageSHA256      string            `json:"package_sha256"`
}

// cellDeployFetchPackage copies the built live package to the Mac, where
// `realtime-core deploy --package` takes it (that command refuses a package
// not built from this checkout's clean HEAD).
func cellDeployFetchPackage(run *cellDeployRun, transport, remote, sha string) (string, error) {
	if remote == "" || !rtCoreHexSHA256.MatchString(sha) {
		return "", cellDeployExit(cellDeployExitFailed, "the build reported no package to fetch")
	}
	home, err := os.UserHomeDir()
	if err != nil {
		return "", err
	}
	dir := filepath.Join(home, ".rosie", "rt-core", "packages")
	if err := os.MkdirAll(dir, 0o700); err != nil {
		return "", err
	}
	local := filepath.Join(dir, sha+".tar.gz")
	if got, err := cellDeployFileSHA256(local); err == nil && got == sha {
		run.p.Step("package", "already on this machine: "+local)
		return local, nil
	}
	out, err := os.Create(local + ".part")
	if err != nil {
		return "", err
	}
	argv := rtCoreConfigRemoteArgv(transport, run.ctx.RelayUser, run.ctx.RelayPeer, run.ctx.RelayHost, "cat "+shellQuote(remote))
	run.begin("package fetch")
	code, err := cellDeployFetch(argv, out, run.p)
	run.end("package fetch", err)
	out.Close()
	got, _ := cellDeployFileSHA256(local + ".part")
	if err != nil || code != 0 || got != sha {
		os.Remove(local + ".part")
		return "", cellDeployExit(cellDeployExitFailed, "package fetch failed (exit %d, %v, sha256 %s want %s)", code, err, got, sha)
	}
	if err := os.Rename(local+".part", local); err != nil {
		return "", err
	}
	run.p.Step("package", fmt.Sprintf("fetched %s (sha256 verified)", local))
	return local, nil
}

// cellDeployBuildMinFreeMiB refuses a build that would fill the relay's disk.
// A deployment tree (source, rt-core build, package, OLP, resolver objects) was
// 1.5-2.5 GiB on one relay in October 2026; unverified on other relays.
const cellDeployBuildMinFreeMiB = 4096

// cellDeployEvaluateBuildProbe decides before any transfer: skip (already
// built), refuse, or build.
func cellDeployEvaluateBuildProbe(commit string, f cellDeployBuildProbeFacts, installEnv, test bool) (skip bool, refuse []string) {
	if f.DeploymentComplete {
		if f.DeploymentHead == commit {
			return true, nil
		}
		return false, []string{fmt.Sprintf("%s is complete but holds %s, not %s", f.Deployment, f.DeploymentHead, commit)}
	}
	if f.DeploymentExists && f.DeploymentHead != "" && f.DeploymentHead != commit {
		refuse = append(refuse, fmt.Sprintf("%s holds an unfinished build of %s, not %s; move it aside", f.Deployment, f.DeploymentHead, commit))
	}
	if !f.IgHPkgconfig {
		refuse = append(refuse, "the cell's igh_deps_dir has no igh-prefix/lib/pkgconfig/libethercat.pc on the relay")
	}
	needed := []string{"go", "make", "pkg-config"}
	if installEnv {
		needed = append(needed, "pixi")
	}
	if test {
		needed = append(needed, "node")
	}
	for _, tool := range needed {
		if f.Tools[tool] == "" {
			refuse = append(refuse, tool+" not found on the relay's cell_deploy path")
		}
	}
	if f.FreeMiB >= 0 && f.FreeMiB < cellDeployBuildMinFreeMiB {
		refuse = append(refuse, fmt.Sprintf("only %d MiB free under the deployments directory (need %d)", f.FreeMiB, cellDeployBuildMinFreeMiB))
	}
	return false, refuse
}

func runCellDeployBuild(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var commit, previous, knownPath, testLevel, testSince, testTier string
	var test, installEnv, allowDrift, fetchPackage, lowPriority, clean, regate bool
	var maintenanceETA time.Duration
	var wait cellLockWait
	ctx, restore, err := cellDeploySetup("realtime-core build", args, stderr, func(fs *flag.FlagSet) {
		fs.BoolVar(&lowPriority, "low-priority", false, "no cell lock and no notice, at nice 15 / ionice best-effort 7: only for a relay that serves no robot in use")
		fs.DurationVar(&maintenanceETA, "maintenance-eta", 0, "expected duration on the overlays (default: from this machine's build history)")
		wait.register(fs)
		fs.StringVar(&commit, "commit", "HEAD", "commit to build (must be on a local ref; uncommitted work never ships)")
		fs.StringVar(&previous, "previous", "", "relay tree to clone from and reuse the planner env of (default: the tree the OLP unit runs)")
		fs.BoolVar(&test, "test", false, "then run the test gate in the build tree (--test-level)")
		fs.StringVar(&testLevel, "test-level", "full", "full: every suite; fast: only the suites whose inputs changed since their last pass recorded on this machine for the cell")
		fs.StringVar(&testSince, "test-since", "", "fast: comma-separated commits the cell runs (default: the previous tree's commit); with --regate fast diffs against them")
		fs.StringVar(&testTier, "test-tier", "", "fast: the release tier (olp or config: servo-only inputs, which the tier does not ship, select nothing; full or empty: every input counts)")
		fs.BoolVar(&regate, "regate", false, "ignore this machine's gate records: no reuse of a pass of the same commit, and fast diffs against --test-since instead of each suite's last pass")
		fs.BoolVar(&clean, "clean", false, "delete the build host's persistent tree first: a from-scratch build")
		fs.StringVar(&knownPath, "known-failures", cellDeployKnownFailuresPath, "known pre-existing gate failures (repository path; rt-core's CI driver validates it)")
		fs.BoolVar(&installEnv, "install-planner-env", false, "run pixi install for the planner env when the previous one cannot be reused")
		fs.BoolVar(&allowDrift, "allow-compiled-drift", false, "accept compiled outputs that differ from the previous tree's under the same pin")
		fs.BoolVar(&fetchPackage, "fetch-package", false, "copy the built rosie-rt-core package to ~/.rosie/rt-core/packages on this machine for realtime-core deploy --package")
	}, &common, true, "--commit", "--previous", "--test", "--test-level", "--test-since", "--test-tier", "--regate", "--clean", "--known-failures", "--install-planner-env", "--allow-compiled-drift", "--fetch-package",
		"--low-priority", "--maintenance-eta", "--idle-wait", "--quiet", "--interval")
	if err != nil {
		return err
	}
	defer restore()
	if err := cellDeployAbsFlag("--previous", previous); err != nil {
		return err
	}
	if err := wait.validate(); err != nil {
		return err
	}
	if maintenanceETA != 0 && (maintenanceETA < time.Minute || maintenanceETA > 12*time.Hour) {
		return fmt.Errorf("--maintenance-eta must be between 1m and 12h")
	}
	repo, err := repoRoot()
	if err != nil {
		return err
	}
	full, ref, err := cellDeployResolveCommit(repo, commit)
	if err != nil {
		return err
	}
	var known map[string]map[string]string
	if test {
		raw, err := os.ReadFile(filepath.Join(repo, filepath.FromSlash(knownPath)))
		if err != nil {
			return fmt.Errorf("--known-failures: %w", err)
		}
		if known, err = cellDeployKnownFailures(raw); err != nil {
			return err
		}
	}
	deployment := ctx.DeploymentDir(full)
	plan := cellDeployBuildPlan{Commit: full, Ref: ref, Previous: previous, BundlePath: path.Join(ctx.Deploy.ScratchDir, "bundles", full+".bundle"),
		InstallPlannerEnv: installEnv, AllowDrift: allowDrift, Locked: !lowPriority, Clean: clean, FetchPackage: fetchPackage}
	if testLevel != "fast" && testLevel != "full" {
		return fmt.Errorf("--test-level must be fast or full")
	}
	lock := newCellDeployBuildLock(ctx, full, test, testLevel, lowPriority, maintenanceETA, wait)
	switch releaseTier(testTier) {
	case "", releaseTierOLP, releaseTierConfig, releaseTierFull:
	default:
		return fmt.Errorf("--test-tier must be olp, config or full")
	}
	var since []string
	for _, c := range strings.Split(testSince, ",") {
		if c = strings.TrimSpace(c); c != "" {
			since = append(since, c)
		}
	}
	// selectGate picks the suites: fast against each suite's last recorded
	// pass on the cell, or with --regate against the base commits (the
	// previous tree's commit comes from the probe).
	selectGate := func(previousHead string) (gateSelection, error) {
		if testLevel == "full" {
			return gateSelect("full", nil, true, gateMeasuredSeconds())
		}
		bases := since
		if len(bases) == 0 && previousHead != "" {
			bases = []string{previousHead}
		}
		return gateFastSelection(gateHistory{Repo: repo, Cell: ctx.CellID, Commit: full, Tier: testTier, Since: bases, Known: known}, regate, gateMeasuredSeconds())
	}
	uploadScript := rtCoreConfigReplace(`set -eu
install -d -m 0700 @DIR@
cat > @FILE@.part
mv -f @FILE@.part @FILE@
printf '@@step 0 upload stored %s sha256=%s\n' @FILE@ "$(sha256sum < @FILE@ | cut -d' ' -f1)"
`, "@DIR@", shellQuote(path.Dir(plan.BundlePath)), "@FILE@", shellQuote(plan.BundlePath))
	if common.DryRun {
		plan.BundleSHA256 = "<bundle-sha256>"
		fmt.Fprintf(stdout, "realtime-core build cell=%s commit=%s ref=%s deployment=%s relay=%s@%s pin=%s test=%t (dry run: nothing contacted)\n",
			ctx.CellID, full, ref, deployment, ctx.RelayUser, ctx.RelayPeer, ctx.RTCore.ConfigurationSHA256, test)
		switch {
		case lock.Script != "":
			fmt.Fprintf(stdout, "# 0. cell lock: relay holder session for the whole build (notice ETA %s, %s)\n%s\n", lock.ETA, lock.ETAWhy, lock.Script)
		case lock.Inherited != "":
			fmt.Fprintf(stdout, "# 0. cell lock: held by the outer run %s\n", lock.Inherited)
		default:
			fmt.Fprintln(stdout, "# 0. --low-priority: no cell lock, no notice")
		}
		fmt.Fprintf(stdout, "# 1. probe (read-only)\n%s\n# 2. git bundle create <tmp> %s [^<commit of the previous tree>], streamed to %s\n%s\n# 3. build in %s\n%s\n",
			cellDeployBuildProbe(ctx, full, previous), ref, plan.BundlePath, uploadScript, ctx.TreeDir(), cellDeployBuildScript(ctx, plan))
		if test {
			sel, err := selectGate("")
			if err != nil {
				return err
			}
			if testLevel == "fast" && !regate {
				fmt.Fprintf(stdout, "# 4. test gate, level fast, tier %s: each suite runs unless a pass this machine recorded for %s covers its inputs at %s (--regate: diff against --test-since instead)\n", firstNonEmpty(testTier, "any (every input counts)"), ctx.CellID, cellDeployShort(full))
			} else {
				fmt.Fprintf(stdout, "# 4. test gate, level %s (fast without --test-since: the suites are chosen after the probe reads the previous tree's commit); skipped when this machine recorded a pass of these suites for the same commit, tree and gate definition %.12s (--regate runs it anyway)\n", testLevel, gateDefinitionDigest())
			}
			for _, line := range sel.Describe() {
				fmt.Fprintf(stdout, "#    %s\n", line)
			}
			fmt.Fprintf(stdout, "#    %s\n", sel.Summary())
			fmt.Fprintf(stdout, "%s\n", cellDeployGateScript(ctx, full, sel, plan.Locked))
		}
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core build", common.Transport, stdout)
	if err != nil {
		return err
	}
	fail := func(result string, e error) error {
		run.finish(result, cellDeployExitFailed, e.Error())
		return e
	}
	if err := lock.acquire(run, common.Transport); err != nil {
		return fail("refused", err)
	}
	defer lock.release(run)
	progress := lock.progress(run)
	run.p.onStep = progress.ObserveGate
	run.p.Step("build", fmt.Sprintf("commit %s (%s) -> %s", full, ref, deployment))
	code, probe, _, refused, err := run.session("build probe", cellDeployBuildProbe(ctx, full, previous))
	if e := sessionOutcome("build probe", code, nil, refused, err); e != nil {
		return fail("refused", e)
	}
	var pf cellDeployBuildProbeFacts
	if err := json.Unmarshal(probe, &pf); err != nil {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "build probe facts unreadable: %v", err))
	}
	run.p.Step("probe", fmt.Sprintf("previous=%s (%.12s) deployment exists=%t complete=%t free=%d MiB tools=%v", firstNonEmpty(pf.Previous, "none"), pf.PreviousHead, pf.DeploymentExists, pf.DeploymentComplete, pf.FreeMiB, pf.Tools))
	skip, refuse := cellDeployEvaluateBuildProbe(full, pf, installEnv, test)
	if len(refuse) > 0 {
		run.report(refuse, nil)
		return fail("refused", cellDeployExit(cellDeployExitRefused, "build refused (nothing was changed):\n  - %s", strings.Join(refuse, "\n  - ")))
	}
	pkg, pkgSHA := pf.Package, pf.PackageSHA256
	if skip {
		run.p.Step("build", deployment+" is already built from "+full+"; nothing to build")
	} else {
		tmp, err := os.CreateTemp("", "rosie-cell-build-*.bundle")
		if err != nil {
			return fail("failed", err)
		}
		tmp.Close()
		os.Remove(tmp.Name())
		defer os.Remove(tmp.Name())
		run.begin("bundle")
		kind, err := cellDeployBundle(repo, ref, pf.PreviousHead, tmp.Name())
		if err != nil {
			return fail("failed", err)
		}
		if plan.BundleSHA256, err = cellDeployFileSHA256(tmp.Name()); err != nil {
			return fail("failed", err)
		}
		run.end("bundle", nil)
		info, _ := os.Stat(tmp.Name())
		run.p.Step("bundle", fmt.Sprintf("%s bundle of %s: %d bytes sha256=%.16s; uploading to %s", kind, ref, info.Size(), plan.BundleSHA256, plan.BundlePath))
		run.p.Record("upload script (relay)", uploadScript)
		run.begin("upload")
		code, err := cellDeployUpload(rtCoreConfigRemoteArgv(common.Transport, ctx.RelayUser, ctx.RelayPeer, ctx.RelayHost, uploadScript), tmp.Name(), run.p)
		run.end("upload", err)
		if err != nil || code != 0 {
			return fail("failed", cellDeployExit(cellDeployExitFailed, "bundle upload failed (exit %d, %v)", code, err))
		}
		code, _, result, refused, err := run.watchedSession("build", cellDeployBuildScript(ctx, plan), remoteWatchBuildLimit)
		if e := sessionOutcome("build", code, result, refused, err); e != nil {
			run.finish("failed", code, e.Error())
			return e
		}
		var built struct {
			Package       string `json:"package"`
			PackageSHA256 string `json:"package_sha256"`
		}
		_ = json.Unmarshal(result, &built)
		pkg, pkgSHA = built.Package, built.PackageSHA256
	}
	run.p.Step("package", fmt.Sprintf("%s:%s sha256=%s", ctx.RelayPeer, firstNonEmpty(pkg, "(none)"), pkgSHA))
	if fetchPackage {
		local, err := cellDeployFetchPackage(run, common.Transport, pkg, pkgSHA)
		if err != nil {
			return fail("failed", err)
		}
		run.p.Step("package", fmt.Sprintf("tier 3 install from a checkout at %s: go run . realtime-core deploy --manifest %s --package %s", cellDeployShort(full), ctx.Manifest, local))
	}
	if !test {
		run.p.Step("done", fmt.Sprintf("built %s; next: go run . olp install --manifest %s --deployment %s", deployment, ctx.Manifest, cellDeployShort(full)))
		run.finish("ok", 0, deployment)
		return nil
	}
	sel, err := selectGate(pf.PreviousHead)
	if err != nil {
		return fail("failed", err)
	}
	if regate && testLevel == "fast" {
		run.p.Step("tests", "--regate: the gate records are ignored; fast takes the suites whose inputs changed since "+firstNonEmpty(strings.Join(since, ","), pf.PreviousHead, "(unknown: every suite)"))
	}
	for _, line := range sel.Describe() {
		run.p.Step("tests", line)
	}
	run.p.Step("tests", sel.Summary())
	if len(sel.Suites) == 0 {
		run.p.Step("done", fmt.Sprintf("built %s; no test suite needs to run (--test-level fast: every suite is unaffected or covered by a recorded pass, above)", deployment))
		run.finish("ok", 0, deployment)
		return nil
	}
	tree, treeErr := gateTreeOf(repo, full)
	if treeErr != nil {
		run.p.Step("gate-reuse", fmt.Sprintf("tree of %s unreadable (%v): no gate reuse", cellDeployShort(full), treeErr))
	} else if regate {
		run.p.Step("gate-reuse", "--regate: running the gate whatever this machine recorded")
	} else if rec, ok := gateFindPass(ctx.CellID, full, tree, sel); ok {
		_, knownFails, newFails := cellDeployClassifyGate(rec.Steps, known)
		if len(newFails) == 0 {
			run.p.Step("gate-reuse", fmt.Sprintf("reused: the %s gate of %s (tree %.12s, gate definition %.12s) passed at %s with suites %s; --regate runs it again",
				rec.Level, cellDeployShort(full), tree, rec.Gate, rec.At, strings.Join(rec.Suites, ", ")))
			for _, line := range knownFails {
				run.p.Step("known", "pre-existing: "+line)
			}
			run.p.Step("done", fmt.Sprintf("built %s; gate passed earlier (reused, %d known pre-existing failures)", deployment, len(knownFails)))
			run.finish("ok", 0, deployment)
			return nil
		}
		run.p.Step("gate-reuse", fmt.Sprintf("the recorded pass of %s at %s now has %d failure(s) the known-failures list no longer excuses; running the gate", cellDeployShort(full), rec.At, len(newFails)))
	}
	run.p.Step("tests", fmt.Sprintf("watched: a stalled stream is re-attached once; the gate is stopped past %s", gateWatchLimit(sel).Round(time.Minute)))
	progress.GateSelected(gateNamesOf(sel))
	code, _, result, refused, err := run.watchedSession("test gate", cellDeployGateScript(ctx, full, sel, plan.Locked), gateWatchLimit(sel))
	if e := sessionOutcome("test gate", code, result, refused, err); e != nil {
		return fail("failed", e)
	}
	var gate struct {
		Steps []cellDeployGateStep `json:"steps"`
	}
	if err := json.Unmarshal(result, &gate); err != nil || len(gate.Steps) == 0 {
		return fail("failed", cellDeployExit(cellDeployExitFailed, "test gate result unreadable: %v", err))
	}
	for _, s := range gate.Steps {
		run.p.timer.attachSuites("test gate/"+s.Step, s.Suites)
	}
	passed, knownFails, newFails := cellDeployClassifyGate(gate.Steps, known)
	for _, line := range passed {
		run.p.Step("gate", line)
	}
	for _, line := range knownFails {
		run.p.Step("known", "pre-existing: "+line)
	}
	for _, line := range newFails {
		run.p.Step("NEW", "failure: "+line)
	}
	if len(newFails) > 0 {
		if treeErr == nil {
			partial, steps := gateSuccessfulSubset(sel, result)
			inputs := gateSuiteInputHashes(repo, full, partial)
			if len(partial.Suites) > 0 && len(inputs) == len(partial.Suites) {
				if err := gateRecordPass(gatePassRecord{Cell: ctx.CellID, Commit: full, Tree: tree, Level: testLevel,
					Partial: true, Suites: gateNamesOf(partial), Steps: steps, Deployment: deployment, SuiteInputs: inputs}); err != nil {
					run.p.Step("gate-reuse", fmt.Sprintf("WARNING successful suite results were not retained (%v)", err))
				} else {
					run.p.Step("gate-reuse", fmt.Sprintf("retained %d successful suite result(s) for fast retry; this gate remains failed", len(partial.Suites)))
				}
			}
		}
		return fail("gate_failed", cellDeployExit(cellDeployExitFailed, "test gate: %d new failure(s) beyond the %d known pre-existing ones; the tree is built but do not deploy it until they are explained", len(newFails), len(knownFails)))
	}
	if treeErr == nil {
		if err := gateRecordPass(gatePassRecord{Cell: ctx.CellID, Commit: full, Tree: tree, Level: testLevel, Suites: gateNamesOf(sel), Steps: gate.Steps, Deployment: deployment,
			SuiteInputs: gateSuiteInputHashes(repo, full, sel)}); err != nil {
			run.p.Step("gate-reuse", fmt.Sprintf("WARNING the pass was not recorded (%v): a retry runs the gate again", err))
		}
	}
	run.p.Step("done", fmt.Sprintf("built %s; gate passed (%d known pre-existing failures)", deployment, len(knownFails)))
	run.finish("ok", 0, deployment)
	return nil
}

// --- precheck, wait-idle, copy-dumps, verify ----------------------------------

func runCellDeployPrecheck(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var quiet time.Duration
	ctx, restore, err := cellDeploySetup("realtime-core precheck", args, stderr, func(fs *flag.FlagSet) {
		fs.DurationVar(&quiet, "quiet", 2*time.Minute, "no OLP operator activity in this window")
	}, &common, true, "--quiet")
	if err != nil {
		return err
	}
	defer restore()
	q, err := cellDeploySeconds("--quiet", quiet, 10*time.Second, time.Hour)
	if err != nil {
		return err
	}
	script := cellDeployPrecheckScript(ctx, q)
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core precheck cell=%s (read-only; dry run: nothing contacted)\n%s\n", ctx.CellID, script)
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core precheck", common.Transport, stdout)
	if err != nil {
		return err
	}
	code, probe, result, refused, err := run.session("precheck", script)
	if e := sessionOutcome("precheck", code, result, refused, err); e != nil {
		run.finish("refused", cellDeployExitRefused, e.Error())
		return e
	}
	facts, err := decodeCellDeployFacts(probe)
	if err != nil {
		run.finish("refused", cellDeployExitRefused, err.Error())
		return cellDeployExit(cellDeployExitRefused, "precheck facts unreadable: %v", err)
	}
	facts.markMacCopies(ctx.CellID)
	refuse, notes := cellDeployEvaluatePrecheck(ctx, facts)
	run.report(refuse, notes)
	if len(refuse) > 0 {
		run.finish("refused", cellDeployExitRefused, strings.Join(refuse, "; "))
		return cellDeployExit(cellDeployExitRefused, "PRECHECK REFUSED:\n  - %s", strings.Join(refuse, "\n  - "))
	}
	run.p.Step("done", "PRECHECK OK (facts kept on the relay under "+path.Join(ctx.Deploy.ScratchDir, "precheck")+" for verify)")
	run.finish("ok", 0, "")
	return nil
}

func runCellDeployWaitIdle(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var quiet, limit, interval time.Duration
	var notice bool
	var maint rtCoreMaintenanceFlags
	ctx, restore, err := cellDeploySetup("realtime-core wait-idle", args, stderr, func(fs *flag.FlagSet) {
		fs.DurationVar(&quiet, "quiet", 10*time.Minute, "no OLP operator activity for this long")
		fs.DurationVar(&limit, "limit", 25*time.Minute, "give up after this long")
		fs.DurationVar(&interval, "interval", time.Minute, "poll interval")
		fs.BoolVar(&notice, "notice", false, "put up the maintenance notice while waiting (operators see it); cleared at the end unless --keep-maintenance hands it to the next step")
		maint.register(fs, false)
	}, &common, true, "--quiet", "--limit", "--interval", "--notice", "--maintenance-eta", "--keep-maintenance")
	if err != nil {
		return err
	}
	defer restore()
	if err := maint.validate(); err != nil {
		return err
	}
	q, err := cellDeploySeconds("--quiet", quiet, 10*time.Second, 2*time.Hour)
	if err != nil {
		return err
	}
	l, err := cellDeploySeconds("--limit", limit, 0, 12*time.Hour)
	if err != nil {
		return err
	}
	i, err := cellDeploySeconds("--interval", interval, 5*time.Second, 10*time.Minute)
	if err != nil {
		return err
	}
	script := cellDeployWaitIdleScript(ctx, q, l, i)
	if notice {
		m := cellDeployNotice(ctx, "wait-idle", "waiting for the cell to go idle before an update", maint, limit)
		m.IdleWait = 0
		script = rtCoreMaintenanceWrap(m, ctx.ServoTarget, "waiting for operators to finish: Stop, disarm and release", script)
	}
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core wait-idle cell=%s notice=%t (read-only on the cell; dry run: nothing contacted)\n%s\n", ctx.CellID, notice, script)
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core wait-idle", common.Transport, stdout)
	if err != nil {
		return err
	}
	code, _, result, refused, err := run.session("wait-idle", script)
	if err == nil && code == 3 {
		run.finish("busy", cellDeployExitRefused, string(result))
		return cellDeployExit(cellDeployExitRefused, "NOT IDLE within %s: %s", limit, string(result))
	}
	if e := sessionOutcome("wait-idle", code, result, refused, err); e != nil {
		run.finish("failed", cellDeployExitFailed, e.Error())
		return e
	}
	run.p.Step("done", "IDLE")
	run.finish("ok", 0, "")
	return nil
}

func runCellDeployCopyDumps(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var to string
	ctx, restore, err := cellDeploySetup("realtime-core copy-dumps", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&to, "to", "relay", "relay: the cell's dump_dir on the relay; mac: then also to ~/.rosie/stop-dumps/<cell> on this machine")
	}, &common, true, "--to")
	if err != nil {
		return err
	}
	defer restore()
	if to != "relay" && to != "mac" {
		return fmt.Errorf("--to must be relay or mac")
	}
	script := cellDeployCopyDumpsScript(ctx)
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core copy-dumps cell=%s to=%s rate=%d B/s dump_dir=%s (dry run: nothing contacted)\n%s\n", ctx.CellID, to, ctx.Deploy.DumpCopyRate, ctx.Deploy.DumpDir, script)
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core copy-dumps", common.Transport, stdout)
	if err != nil {
		return err
	}
	result, code, err := cellDeployCopyDumpsSession(run, ctx, common.Transport, to == "mac")
	if err != nil {
		run.finish("failed", code, err.Error())
		return err
	}
	run.finish("ok", 0, result)
	return nil
}

// cellDeployCopyDumpsSession copies every stop dump to the relay's dump_dir
// (and with toMac to this machine) and returns the session's result and, on
// failure, the exit code to journal. The caller finishes the run.
func cellDeployCopyDumpsSession(run *cellDeployRun, ctx cellDeployContext, transport string, toMac bool) (string, int, error) {
	code, _, result, refused, err := run.session("copy-dumps", cellDeployCopyDumpsScript(ctx))
	if e := sessionOutcome("copy-dumps", code, result, refused, err); e != nil {
		return "", code, e
	}
	var copied struct {
		Dir   string `json:"dir"`
		Files []struct {
			Name   string `json:"name"`
			Bytes  int64  `json:"bytes"`
			SHA256 string `json:"sha256"`
		} `json:"files"`
	}
	if err := json.Unmarshal(result, &copied); err != nil {
		return "", cellDeployExitFailed, cellDeployExit(cellDeployExitFailed, "copy-dumps result unreadable: %v", err)
	}
	if toMac {
		home, err := os.UserHomeDir()
		if err != nil {
			return "", cellDeployExitFailed, err
		}
		local := filepath.Join(home, ".rosie", "stop-dumps", ctx.CellID)
		if err := os.MkdirAll(local, 0o700); err != nil {
			return "", cellDeployExitFailed, err
		}
		for _, f := range copied.Files {
			dst := filepath.Join(local, f.Name)
			if sum, err := cellDeployFileSHA256(dst); err == nil && sum == f.SHA256 {
				run.p.Step("mac", "already here "+dst)
				continue
			}
			out, err := os.Create(dst + ".part")
			if err != nil {
				return "", cellDeployExitFailed, err
			}
			argv := rtCoreConfigRemoteArgv(transport, ctx.RelayUser, ctx.RelayPeer, ctx.RelayHost, "cat "+shellQuote(path.Join(copied.Dir, f.Name)))
			code, err := cellDeployFetch(argv, out, run.p)
			out.Close()
			sum, _ := cellDeployFileSHA256(dst + ".part")
			if err != nil || code != 0 || sum != f.SHA256 {
				os.Remove(dst + ".part")
				return "", cellDeployExitFailed, cellDeployExit(cellDeployExitFailed, "fetch of %s to the Mac failed (exit %d, %v, sha256 %s want %s)", f.Name, code, err, sum, f.SHA256)
			}
			if err := os.Rename(dst+".part", dst); err != nil {
				return "", cellDeployExitFailed, err
			}
			run.p.Step("mac", fmt.Sprintf("copied %s (%d bytes, sha256 verified)", dst, f.Bytes))
		}
	}
	run.p.Step(map[bool]string{true: "done", false: "dumps"}[run.command == "realtime-core copy-dumps"],
		fmt.Sprintf("%d dump file(s) held in %s:%s; decode them off the servo host", len(copied.Files), ctx.RelayPeer, copied.Dir))
	return string(result), 0, nil
}

func runCellDeployVerify(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var homeWait time.Duration
	var caps string
	var installed bool
	ctx, restore, err := cellDeploySetup("realtime-core verify", args, stderr, func(fs *flag.FlagSet) {
		fs.DurationVar(&homeWait, "home-wait", 20*time.Second, "wait this long for home_valid_mask to settle after a start")
		fs.StringVar(&caps, "expect-capabilities", "", "also require this Describe capabilities digest")
		fs.BoolVar(&installed, "expect-installed", false, "verify against the configuration control.env pins (what is installed) instead of the manifest's pin; Describe must run exactly that one")
	}, &common, true, "--home-wait", "--expect-capabilities", "--expect-installed")
	if err != nil {
		return err
	}
	defer restore()
	w, err := cellDeploySeconds("--home-wait", homeWait, 0, 10*time.Minute)
	if err != nil {
		return err
	}
	if caps != "" && !rtCoreHexSHA256.MatchString(caps) {
		return fmt.Errorf("--expect-capabilities must be a lowercase 64-hex digest")
	}
	script := cellDeployVerifyScript(ctx, w)
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core verify cell=%s expect=%s (read-only; dry run: nothing contacted)\n%s\n%s\n", ctx.CellID,
			map[bool]string{true: "installed (control.env) pin", false: "cell pin " + ctx.RTCore.ConfigurationSHA256}[installed], cellDeployDCPlan(ctx), script)
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core verify", common.Transport, stdout)
	if err != nil {
		return err
	}
	_, err = cellDeployVerifySessionExpect(run, ctx, w, cellDeployVerifyExpect{Installed: installed, Capabilities: caps}, true)
	return err
}

func cellDeployVerifySession(run *cellDeployRun, ctx cellDeployContext, homeWait int, caps string) error {
	_, err := cellDeployVerifySessionExpect(run, ctx, homeWait, cellDeployVerifyExpect{Capabilities: caps}, true)
	return err
}

// cellDeployVerifySessionExpect runs verify against e and returns its facts;
// with finish it also ends the run (restart reports before it does).
func cellDeployVerifySessionExpect(run *cellDeployRun, ctx cellDeployContext, homeWait int, e cellDeployVerifyExpect, finish bool) (cellDeployFacts, error) {
	end := func(result string, code int, detail string) {
		if finish {
			run.finish(result, code, detail)
		}
	}
	home := ctx.FullAxisMask()
	if e.HomeMask != nil {
		home = *e.HomeMask
	}
	code, probe, result, refused, err := run.session("verify", cellDeployVerifyScriptFor(ctx, homeWait, home))
	if e := sessionOutcome("verify", code, result, refused, err); e != nil {
		end("failed", cellDeployExitFailed, e.Error())
		return cellDeployFacts{}, e
	}
	facts, err := decodeCellDeployFacts(probe)
	if err != nil {
		end("failed", cellDeployExitFailed, err.Error())
		return facts, cellDeployExit(cellDeployExitFailed, "verify facts unreadable: %v", err)
	}
	refuse, notes := cellDeployEvaluateVerifyExpect(ctx, facts, e)
	run.report(refuse, notes)
	if facts.Verify != nil {
		for _, line := range facts.Verify.JournalLines {
			run.p.Step("journal", line)
		}
	}
	if len(refuse) > 0 {
		end("failed", cellDeployExitFailed, strings.Join(refuse, "; "))
		return facts, cellDeployExit(cellDeployExitFailed, "VERIFY FAILED (nothing was changed; report it, never re-home to fix it):\n  - %s", strings.Join(refuse, "\n  - "))
	}
	what := "cell pin"
	switch {
	case e.Installed:
		what = "installed pin " + cellDeployShort(facts.ControlEnvPin)
	case e.Pin != "":
		what = "configuration " + cellDeployShort(e.Pin)
	}
	homeResult := "Home restored"
	if e.RequireHomeInvalidated {
		homeResult = "old Home invalidated"
	}
	run.p.Step(map[bool]string{true: "done", false: "verify"}[finish], "VERIFIED: "+what+" running, every axis and readback verified, "+homeResult+", disarmed, no faults, DC clocks locked"+map[bool]string{true: "", false: " (no DC: bus_bringup.no_dc)"}[len(ctx.DCSlaves) > 0])
	end("ok", 0, "")
	return facts, nil
}

// --- reboot and resume -----------------------------------------------------

const cellDeployRebootWarning = `WARNING: this reboots the SERVO HOST of a robot cell. CLAUDE.md: a reboot of
the servo host needs the owner's approval, even disarmed. It empties /run
(stop dumps), drops the EtherCAT bus, and the controller stays stopped until
realtime-core resume-after-reboot. Use it only for what a restart cannot fix,
for example an IgH master left reserved by a core stuck in a mailbox call.
`

func runCellDeployReboot(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var approved bool
	var reason string
	var maint rtCoreMaintenanceFlags
	ctx, restore, err := cellDeploySetup("realtime-core reboot", args, stderr, func(fs *flag.FlagSet) {
		fs.BoolVar(&approved, "owner-approved", false, "the owner approved this reboot (required)")
		fs.StringVar(&reason, "reason", "", "why, as approved by the owner (recorded in the journal)")
		maint.register(fs, false)
	}, &common, true, "--owner-approved", "--reason", "--maintenance-eta", "--keep-maintenance")
	if err != nil {
		return err
	}
	defer restore()
	if err := maint.validate(); err != nil {
		return err
	}
	fmt.Fprint(stdout, cellDeployRebootWarning)
	m := cellDeployNotice(ctx, "reboot", "servo host reboot (owner-approved)", maint, cellDeployRebootEstimate)
	m.IdleWait = 0
	script := rtCoreMaintenanceWrap(m, ctx.ServoTarget, "rebooting the servo host (owner-approved); the controller is stopped until resume-after-reboot",
		cellDeployRebootScript(ctx, firstNonEmpty(reason, "<reason>")))
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core reboot cell=%s servo=%s owner_approved=%t (dry run: nothing contacted)\n# 1. facts (read-only)\n%s\n# 2. reboot\n%s\n", ctx.CellID, ctx.ServoTarget, approved, cellDeployFactsScript(ctx, false), script)
		return nil
	}
	if !approved || strings.TrimSpace(reason) == "" {
		return cellDeployExit(cellDeployExitRefused, "reboot refused: pass --owner-approved and --reason with the owner's approval; nothing was contacted")
	}
	if rtCoreConfigStdinIsTTY() {
		answer, err := rtCoreConfigPrompt(fmt.Sprintf("type the servo host peer name (%s) to reboot it: ", ctx.ServoPeer))
		if err != nil || strings.TrimSpace(answer) != ctx.ServoPeer {
			return cellDeployExit(cellDeployExitRefused, "reboot not confirmed; nothing was contacted")
		}
	}
	run, err := newCellDeployRun(ctx, "realtime-core reboot", common.Transport, stdout)
	if err != nil {
		return err
	}
	run.p.Step("reboot", "owner-approved: "+reason)
	code, probe, result, refused, err := run.session("reboot facts", cellDeployFactsScript(ctx, false))
	if e := sessionOutcome("reboot facts", code, result, refused, err); e != nil {
		run.finish("refused", cellDeployExitRefused, e.Error())
		return e
	}
	facts, err := decodeCellDeployFacts(probe)
	if err != nil {
		run.finish("refused", cellDeployExitRefused, err.Error())
		return cellDeployExit(cellDeployExitRefused, "reboot facts unreadable: %v", err)
	}
	facts.markMacCopies(ctx.CellID)
	var refuse []string
	if facts.busy() {
		refuse = append(refuse, "controller armed, enabled or leased: disarm and release first")
	}
	if missing := facts.uncopiedDumps(); len(missing) > 0 {
		refuse = append(refuse, "stop dumps without a copy: "+strings.Join(missing, ", ")+"; run realtime-core copy-dumps first")
	}
	for _, e := range facts.Errors {
		run.p.Step("fact", "servo host read: "+e+" (a wedged controller is a reason to reboot; continuing)")
	}
	if len(facts.DState) > 0 {
		run.p.Step("fact", fmt.Sprintf("processes in D/Z state: %v", facts.DState))
	}
	if len(refuse) > 0 {
		run.report(refuse, nil)
		run.finish("refused", cellDeployExitRefused, strings.Join(refuse, "; "))
		return cellDeployExit(cellDeployExitRefused, "reboot refused (nothing was changed):\n  - %s", strings.Join(refuse, "\n  - "))
	}
	requested := time.Now().UTC().Add(-time.Second)
	code, _, result, refused, err = run.session("reboot", script)
	if err == nil && code == cellDeployExitNotBack {
		run.finish("not_back", code, string(result))
		return cellDeployExit(code, "the servo host did not come back within 600 s; check it on site")
	}
	if e := sessionOutcome("reboot", code, result, refused, err); e != nil {
		run.finish("failed", code, e.Error())
		return e
	}
	run.p.Step("done", fmt.Sprintf("servo host rebooted; next: go run . realtime-core resume-after-reboot --manifest %s --booted-after %s", ctx.Manifest, requested.Format(time.RFC3339)))
	run.finish("ok", 0, string(result))
	return nil
}

func runCellDeployResume(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var after string
	var homeWait time.Duration
	var caps string
	var maint rtCoreMaintenanceFlags
	ctx, restore, err := cellDeploySetup("realtime-core resume-after-reboot", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&after, "booted-after", "", "RFC3339 time the reboot was requested; the servo host must have booted after it (required)")
		fs.DurationVar(&homeWait, "home-wait", 20*time.Second, "wait this long for home_valid_mask to settle after the start")
		fs.StringVar(&caps, "expect-capabilities", "", "also require this Describe capabilities digest")
		maint.register(fs, false)
	}, &common, true, "--booted-after", "--home-wait", "--expect-capabilities", "--maintenance-eta", "--keep-maintenance")
	if err != nil {
		return err
	}
	defer restore()
	if err := maint.validate(); err != nil {
		return err
	}
	bootedAfter, err := time.Parse(time.RFC3339, after)
	if err != nil {
		return fmt.Errorf("--booted-after must be an RFC3339 time (for example %s)", time.Now().UTC().Format(time.RFC3339))
	}
	w, err := cellDeploySeconds("--home-wait", homeWait, 0, 10*time.Minute)
	if err != nil {
		return err
	}
	start, err := rtCoreStartCommand(ctx.RTCore)
	if err != nil {
		return err
	}
	m := cellDeployNotice(ctx, "resume-after-reboot", "rt-core controller start after a servo host reboot", maint, rtCoreMaintenanceStartEstimate)
	m.IdleWait = 0
	startScript := rtCoreMaintenanceWrap(m, ctx.ServoTarget, "starting the controller after the servo host reboot (stays disarmed)", cellDeployStartScript(ctx, start))
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core resume-after-reboot cell=%s booted_after=%s (dry run: nothing contacted)\n# 1. facts (read-only, EtherCAT master read only with both units inactive)\n%s\n# 2. start disarmed\n%s\n# 3. verify (read-only)\n%s\n",
			ctx.CellID, bootedAfter.Format(time.RFC3339), cellDeployFactsScript(ctx, true), startScript, cellDeployVerifyScript(ctx, w))
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core resume-after-reboot", common.Transport, stdout)
	if err != nil {
		return err
	}
	code, probe, result, refused, err := run.session("resume facts", cellDeployFactsScript(ctx, true))
	if e := sessionOutcome("resume facts", code, result, refused, err); e != nil {
		run.finish("refused", cellDeployExitRefused, e.Error())
		return e
	}
	facts, err := decodeCellDeployFacts(probe)
	if err != nil {
		run.finish("refused", cellDeployExitRefused, err.Error())
		return cellDeployExit(cellDeployExitRefused, "resume facts unreadable: %v", err)
	}
	if facts.EtherCAT != nil {
		run.p.Step("fact", fmt.Sprintf("booted %s; EtherCAT master phase=%s slaves=%d; units %s/%s", facts.Boot, facts.EtherCAT.Phase, facts.EtherCAT.Slaves,
			facts.unitActive("rosie-rt-core.service"), facts.unitActive("rosie-rt-control.service")))
	}
	if refuse := cellDeployEvaluateResume(ctx, facts, bootedAfter); len(refuse) > 0 {
		run.report(refuse, nil)
		run.finish("refused", cellDeployExitRefused, strings.Join(refuse, "; "))
		return cellDeployExit(cellDeployExitRefused, "resume refused (nothing was changed):\n  - %s", strings.Join(refuse, "\n  - "))
	}
	code, _, result, refused, err = run.session("start", startScript)
	if e := sessionOutcome("start", code, result, refused, err); e != nil {
		run.finish("failed", cellDeployExitFailed, e.Error())
		return e
	}
	return cellDeployVerifySession(run, ctx, w, caps)
}

// --- olp ---------------------------------------------------------------------

const cellDeployOLPUsage = `usage: olp <install|rollback|status|ui> --manifest cell.json [flags]

Tier 2 (OLP-only) on the cell's relay; every value comes from the manifest's
cell_deploy.olp block. Touches no controller; target select is Describe only.

  install   Repoint the OLP unit at a built tree (realtime-core build):
            --deployment <commit|dir> [--previous dir] [--expect-capabilities d]
            Refuses while the OLP is in use; re-pins olp-cells.json from the
            live Describe; backs up the unit; a failed check restores it (exit 5).
  rollback  Reinstall the unit backup saved by the last install [--to dir]
  status    Read-only: unit, tree, catalogue pins, health, /targets
All take --transport tailscale|ssh and --dry-run.

  ui        This Mac's browser UI against the cell: start|stop|restart|status|logs
            (a self-healing tunnel to the OLP plus Vite, under launchd);
            go run . olp ui --help
`

func runOLP(args []string, stdout, stderr io.Writer) error {
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, cellDeployOLPUsage)
		return nil
	}
	switch args[0] {
	case "install":
		return runCellDeployOLPInstall(args[1:], stdout, stderr)
	case "rollback":
		return runCellDeployOLPRollback(args[1:], stdout, stderr)
	case "status":
		return runCellDeployOLPStatus(args[1:], stdout, stderr)
	case "ui":
		return runOLPUI(args[1:], stdout, stderr)
	}
	return fmt.Errorf("unknown olp command %q", args[0])
}

// cellDeployResolveDeployment accepts a deployment directory under the cell's
// deployments_dir or a commit of this checkout.
func cellDeployResolveDeployment(ctx cellDeployContext, value string) (string, error) {
	if strings.HasPrefix(value, "/") {
		if err := cellDeployCleanAbs("--deployment", value); err != nil {
			return "", err
		}
		if path.Dir(value) != ctx.Deploy.DeploymentsDir || !strings.HasPrefix(path.Base(value), ctx.Deploy.DeploymentPrefix) {
			return "", fmt.Errorf("--deployment %s is not %s/%s<commit>", value, ctx.Deploy.DeploymentsDir, ctx.Deploy.DeploymentPrefix)
		}
		return value, nil
	}
	repo, err := repoRoot()
	if err != nil {
		return "", err
	}
	full, err := robotV4GitOutput(repo, "rev-parse", "--verify", value+"^{commit}")
	full = strings.TrimSpace(full)
	if err != nil || !cellDeployHexCommit.MatchString(full) {
		return "", fmt.Errorf("--deployment %q is neither a deployment directory nor a commit of this checkout", value)
	}
	return ctx.DeploymentDir(full), nil
}

func runCellDeployOLPInstall(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var deployment, previous, caps string
	var pairMigration bool
	var maint rtCoreMaintenanceFlags
	ctx, restore, err := cellDeploySetup("olp install", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&deployment, "deployment", "", "built tree: a commit of this checkout or the deployment directory (required)")
		fs.StringVar(&previous, "previous", "", "tree to take the client PKI, OLP state and catalogue from (default: the unit's current tree)")
		fs.StringVar(&caps, "expect-capabilities", "", "refuse unless the live Describe has this capabilities digest")
		fs.BoolVar(&pairMigration, "pair-migration", false, "use staged client PKI for the manifest pair and update the OLP binding")
		maint.register(fs, true)
	}, &common, true, "--deployment", "--previous", "--expect-capabilities", "--pair-migration", "--maintenance-eta", "--idle-wait", "--keep-maintenance")
	if err != nil {
		return err
	}
	defer restore()
	if err := maint.validate(); err != nil {
		return err
	}
	if deployment == "" {
		return fmt.Errorf("olp install requires --deployment")
	}
	if err := cellDeployAbsFlag("--previous", previous); err != nil {
		return err
	}
	if caps != "" && !rtCoreHexSHA256.MatchString(caps) {
		return fmt.Errorf("--expect-capabilities must be a lowercase 64-hex digest")
	}
	if pairMigration && !common.DryRun && os.Getenv(cellLockEnv) == "" {
		return fmt.Errorf("OLP pair migration is only available inside a locked realtime-core release")
	}
	dir, err := cellDeployResolveDeployment(ctx, deployment)
	if err != nil {
		return err
	}
	release := strings.TrimPrefix(path.Base(dir), ctx.Deploy.DeploymentPrefix)
	// Alone, its estimate and a one-step progress come from this machine's
	// journal; inside a run's lock (release) the notice keeps that run's.
	estimate := releaseJournalHistory(ctx.CellID).stepEstimate("olp install", releaseGatePlan{})
	m := cellDeployNotice(ctx, "olp-install", "OLP server update to "+release, maint, releaseETAWithMargin(estimate.Expected))
	if m.LockInherit == "" {
		m.Progress = noticeStaticProgress(estimate)
	}
	script := rtCoreMaintenanceWrap(m, ctx.ServoTarget, "installing the OLP server "+release+"; it restarts shortly",
		cellDeployOLPInstallScript(ctx, cellDeployOLPPlan{Deployment: dir, Previous: previous, ExpectCapabilities: caps, PairMigration: pairMigration}))
	if common.DryRun {
		planner := "# planner unit: provisioned by hand (no cell_deploy.olp.planner); must be active\n"
		if ctx.Deploy.OLP.Planner != nil {
			planner = "# planner unit to install (" + cellDeployPlannerRevisionToken + " = the tree's commit)\n" + cellDeployPlannerUnit(ctx, dir, release)
		}
		fmt.Fprintf(stdout, "olp install cell=%s unit=%s deployment=%s (dry run: nothing contacted)\n# unit to install\n%s%s# relay script\n%s\n", ctx.CellID, ctx.Deploy.OLP.Unit, dir, cellDeployOLPUnit(ctx, dir, release), planner, script)
		return nil
	}
	run, err := newCellDeployRun(ctx, "olp install", common.Transport, stdout)
	if err != nil {
		return err
	}
	code, _, result, refused, err := run.session("olp install", script)
	if err == nil && code == cellDeployExitRestored {
		run.finish("restored", code, string(result))
		return cellDeployExit(code, "olp install failed a check and the previous unit was restored: %s (the new tree keeps private/ and olp-state/: move them aside before retrying)", string(result))
	}
	if e := sessionOutcome("olp install", code, result, refused, err); e != nil {
		run.finish("failed", code, e.Error())
		return e
	}
	run.p.Step("done", fmt.Sprintf("OLP runs %s; re-select the cell in the UI. Rollback: go run . olp rollback --manifest %s", dir, ctx.Manifest))
	run.finish("ok", 0, string(result))
	return nil
}

func runCellDeployOLPRollback(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var to string
	var maint rtCoreMaintenanceFlags
	ctx, restore, err := cellDeploySetup("olp rollback", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&to, "to", "", "refuse unless the saved unit returns to this tree")
		maint.register(fs, true)
	}, &common, true, "--to", "--maintenance-eta", "--idle-wait", "--keep-maintenance")
	if err != nil {
		return err
	}
	defer restore()
	if err := maint.validate(); err != nil {
		return err
	}
	if err := cellDeployAbsFlag("--to", to); err != nil {
		return err
	}
	m := cellDeployNotice(ctx, "olp-rollback", "OLP server rollback", maint, cellDeployOLPEstimate)
	script := rtCoreMaintenanceWrap(m, ctx.ServoTarget, "rolling the OLP server back; it restarts shortly", cellDeployOLPRollbackScript(ctx, to))
	if common.DryRun {
		fmt.Fprintf(stdout, "olp rollback cell=%s unit=%s (dry run: nothing contacted)\n%s\n", ctx.CellID, ctx.Deploy.OLP.Unit, script)
		return nil
	}
	run, err := newCellDeployRun(ctx, "olp rollback", common.Transport, stdout)
	if err != nil {
		return err
	}
	code, _, result, refused, err := run.session("olp rollback", script)
	if e := sessionOutcome("olp rollback", code, result, refused, err); e != nil {
		run.finish("failed", code, e.Error())
		return e
	}
	run.p.Step("done", "rolled back: "+string(result)+"; re-select the cell in the UI")
	run.finish("ok", 0, string(result))
	return nil
}

func runCellDeployOLPStatus(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	ctx, restore, err := cellDeploySetup("olp status", args, stderr, nil, &common, true)
	if err != nil {
		return err
	}
	defer restore()
	script := cellDeployOLPStatusScript(ctx)
	if common.DryRun {
		fmt.Fprintf(stdout, "olp status cell=%s (read-only; dry run: nothing contacted)\n%s\n", ctx.CellID, script)
		return nil
	}
	p := newRTCoreProgress(stdout, nil)
	code, err := rtCoreConfigRunRemote(rtCoreConfigRemoteArgv(common.Transport, ctx.RelayUser, ctx.RelayPeer, ctx.RelayHost, script), p)
	return sessionOutcome("olp status", code, nil, nil, err)
}
