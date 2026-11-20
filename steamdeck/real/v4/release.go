package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"path"
	"reflect"
	"regexp"
	"strconv"
	"strings"
	"time"
)

// `realtime-core release`: one command from a commit to a verified cell.
// Under the cell lock for the whole run (the maintenance notice is up from
// the moment the lock is held until the run ends), it reads what the cell
// runs, picks the smallest tier (release_tier.go), and runs that tier's
// existing steps in process, in order, each with its own journal and
// timings:
//
//	config: build, test, copy-dumps, config (--olp restart), [olp install]
//	olp:    build, test, olp install
//	full:   build --fetch-package, test, precheck, copy-dumps, deploy --package,
//	        start, verify, olp install
//
// Each step's own refusals stand: release adds no override. Rollback per
// tier: config and olp install restore themselves on a failed check; a full
// release whose start or verify fails copies the dumps off and runs
// `realtime-core rollback` to the release that ran before (--rollback never
// leaves the failure in place for investigation). The pendant is flagged,
// never rebuilt. The run ends with one timing table, one row per step.

const releaseUsage = `usage: realtime-core release --manifest cell.json (--commit <sha> | --candidate <dir> --candidate-id sha256:<digest>) [flags]

One command from a commit to a verified cell, under the cell lock throughout:
reads what the cell runs (read-only), picks the smallest tier that carries
the change, then runs that tier's steps and prints one timing table.

  --tier auto|config|olp|full   default auto (a smaller forced tier is refused
                                when it cannot carry the change)
  --candidate DIR               verified local build-host candidate bundle; full
                                RTCore release only, without rebuilding or OLP
  --candidate-id SHA256         exact candidate identity expected in DIR
  --busy-status-url URL          optional cooperative preflight against the
                                Rosie dashboard /api/busy endpoint
  --test fast|full              default fast for config/olp, full for full
  --plan                        read what the cell runs, print the tier and the
                                steps; change nothing (read-only on the relay
                                and servo host)
  --dry-run                     print the probe and every tier's steps;
                                contact nothing
  --rollback auto|never         full tier: roll back after a failed start or
                                verify; pair migration also covers OLP install
  --pair-migration-from ID      full source release only: replace the installed
                                pair PKI with staged credentials for the new
                                manifest pair; automatic rollback required
  --drive-transition prepare    full source release only: stop both units,
                                install, then change declared A6-EC tracking
                                modes 4 to 1 while stopped. Leave maintenance
                                up and require a physical drive power cycle
  --drive-transition resume     after that cycle, verify modes directly,
                                start and verify disarmed with Home invalid
  --power-cycled                resume only: operator confirms the physical
                                cycle; software rollback is disabled
  --rollback-to-configuration SHA
                                resume only: the configuration that ran before
                                the release when this machine has no record
                                of it
  --rollback-to-axes N          resume only: the previous Describe axis count
                                for a paired migration on another machine

A release of a commit whose rt-core package is already current resumes: both
units stopped -> start, verify, olp install; both running -> verify, olp
install (olp install only when the OLP tree is not the commit).
  --accept-hostcheck LIST       full tier: deploy's health gate failures the
                                owner accepts (comma-separated hostcheck ids,
                                e.g. igh_master on a servo host); default none
  --regate                      ignore this machine's gate records: run the gate
                                even when a pass of the same commit is recorded,
                                and let fast tests diff against the installed
                                commits instead of each suite's last pass
                                (default: fast runs only the suites whose inputs
                                changed since their last recorded pass on the
                                cell; servo-only inputs do not count for the
                                olp and config tiers)
  --accept-config-drift PIN     the cell runs configuration PIN, which no
                                commit on the branch pins, and the commit would
                                change drive values it carries (printed per key,
                                running -> commit): the owner accepts replacing
                                it. Without it such a release is refused
  --clean                       from-scratch build of the build host's tree
  --install-planner-env         pixi install when the planner env cannot be reused
  --idle-wait 10m --quiet 2m    how long to wait for the cell lock and an idle cell
  --transport tailscale|ssh

The working tree's cell manifest and machine config must equal the commit's
(the pins come from them), and nothing under rt-core/config,
robot_description or the cell manifest may be uncommitted (owner rule
2026-10-11: a cell never runs a configuration git does not hold); a full
release runs from a clean checkout at the commit (deploy --package refuses
any other). When the cell runs a configuration no commit pins, the release
(and --plan) says so and lists every drive value the commit would change.
`

type releaseOptions struct {
	Commit, Candidate, CandidateID            string
	Tier, Test, Rollback, AcceptHostcheck     string
	RollbackTo, AcceptConfigDrift             string
	BusyStatusURL                             string
	PairMigrationFrom                         string
	DriveTransition                           string
	RollbackToAxes                            int
	CandidatePackage, CandidateArtifactSHA256 string
	CandidatePackageManifestSHA256            string
	AcceptHomeInvalidation                    bool
	Plan, Clean, InstallEnv, Regate           bool
	PowerCycled                               bool
	Wait                                      cellLockWait
}

// releaseProbeFacts is the read-only probe: the servo facts merged with the
// relay's (dumps copied, OLP session), the OLP tree's commit and the running
// compiled axes.conf (the drift guard's running drive values, config_drift.go).
type releaseProbeFacts struct {
	cellDeployFacts
	OLPHead               string `json:"olp_head"`
	LiveAxesConf          string `json:"live_axes_conf"`
	DriveTransitionNotice bool   `json:"drive_transition_notice"`
	MaintenanceRunID      string `json:"maintenance_run_id"`
	ServoMaintenanceRunID string `json:"servo_maintenance_run_id"`
}

// releaseProbeScript is read-only on both hosts.
func releaseProbeScript(ctx cellDeployContext) string {
	return cellDeployRelayPrologue(ctx) + rtCoreConfigReplace(`DUMPS=@DUMPS@; UNIT=@UNIT@; API=@API@
w=$(mktemp -d) || refuse "mktemp failed"
wd=$(systemctl show "$UNIT" -p WorkingDirectory --value 2> /dev/null)
OLP_DIR=""; case "$wd" in */source/offline-programming/v1) OLP_DIR=${wd%/source/offline-programming/v1} ;; esac
OLP_HEAD=$(git -C "$OLP_DIR/source" rev-parse HEAD 2> /dev/null)
step "release probe relay=$(hostname) olp=${OLP_DIR:-none} at ${OLP_HEAD:-unknown}; reading the servo host (read-only)"
servos > "$w/servo.json" 2> "$w/servo.err" <<'ROSIE_SERVO_FACTS'
@FACTS@
ROSIE_SERVO_FACTS
python3 -c 'import json,sys; json.load(open(sys.argv[1]))' "$w/servo.json" 2> /dev/null || refuse "servo host facts unreadable: $(tail -n 3 "$w/servo.err" | tr '\n' ' ')"
curl -fsS -m 5 "$API/dense-execution/status" > "$w/olp-status.json" 2> /dev/null || : > "$w/olp-status.json"
sudo -n sh -c 'if [ -e "$1" ]; then cat "$1"; else printf "{}\n"; fi' sh @MAINT@ > "$w/maintenance.json" || refuse "maintenance notice unreadable"
servos > "$w/servo-maintenance.json" <<'ROSIE_SERVO_MAINT' || refuse "servo maintenance notice unreadable"
sudo -n sh -c 'if [ -e "$1" ]; then cat "$1"; else printf "{}\n"; fi' sh /etc/rosie-rt-core/maintenance.json
ROSIE_SERVO_MAINT
: > "$w/activity"
python3 - "$w/servo.json" "$w/olp-status.json" "$w/activity" "$DUMPS" "$UNIT" "$OLP_DIR" > "$w/probe.line" <<'ROSIE_MERGE_PY'
@MERGE@
ROSIE_MERGE_PY
cfg=$(python3 -c 'import json,sys; print((json.load(open(sys.argv[1])).get("describe") or {}).get("configuration_sha256") or "")' "$w/servo.json" 2> /dev/null)
: > "$w/axes.conf"
if printf '%s' "$cfg" | grep -Eqx '[0-9a-f]{64}'; then
  servo "sudo -n cat /etc/rosie-rt-core/compiled/$cfg/axes.conf" > "$w/axes.conf" 2> /dev/null || : > "$w/axes.conf"
fi
python3 - "$w/probe.line" "$OLP_HEAD" "$w/axes.conf" "$w/maintenance.json" "$w/servo-maintenance.json" @CELL@ <<'ROSIE_RELEASE_PY' || refuse "maintenance notice or release facts invalid"
import json, sys
line, head, axes, maintenance, servo_maintenance, cell = sys.argv[1:7]
facts = json.loads(open(line).read().split(" ", 1)[1])
facts["olp_head"] = head
facts["live_axes_conf"] = open(axes).read()
notice = json.load(open(maintenance))
servo_notice = json.load(open(servo_maintenance))
facts["drive_transition_notice"] = isinstance(notice, dict) and notice.get("active") is True and str(notice.get("run_id") or "").startswith("drive-transition-" + cell + "-")
facts["maintenance_run_id"] = str(notice.get("run_id") or "") if isinstance(notice, dict) and notice.get("active") is True else ""
facts["servo_maintenance_run_id"] = str(servo_notice.get("run_id") or "") if isinstance(servo_notice, dict) and servo_notice.get("active") is True else ""
print("@@probe " + json.dumps(facts))
ROSIE_RELEASE_PY
rm -rf -- "$w"
`, "@DUMPS@", shellQuote(ctx.Deploy.DumpDir), "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit),
		"@API@", shellQuote("http://"+ctx.Deploy.OLP.Listen+"/api/offline-programming/v1"),
		"@MAINT@", shellQuote(rtCoreMaintenanceFile),
		"@CELL@", shellQuote(ctx.CellID),
		"@FACTS@", cellDeployServoFacts(false), "@MERGE@", cellDeployRelayFactsPy)
}

// releaseInstalledFrom reads what the cell runs from the probe.
func releaseInstalledFrom(f releaseProbeFacts) (releaseInstalled, error) {
	in := releaseInstalled{RTCoreCommit: path.Base(f.Current), OLPCommit: f.OLPHead}
	if f.Describe == nil {
		return in, fmt.Errorf("the controller's Describe is unavailable (rosie-rt-control not active?): the release cannot tell what the cell runs; check with realtime-core precheck")
	}
	in.Configuration, in.Capabilities, in.Protocol, in.Axes = f.Describe.ConfigurationSHA256, f.Describe.CapabilitiesDigest, f.Describe.Protocol, f.Describe.Axes
	if !cellDeployHexCommit.MatchString(in.RTCoreCommit) {
		return in, fmt.Errorf("installed release %q is not a commit-named release directory", f.Current)
	}
	return in, nil
}

// releaseConfirmLockedProbe refuses a candidate install or resume if the
// installed identity changed between the first read and the cell lock.
func releaseConfirmLockedProbe(run *cellDeployRun, script string, before releaseProbeFacts, checkDrives bool) error {
	run.begin("locked probe")
	code, raw, result, refused, err := run.session("release locked probe", script)
	run.end("locked probe", err)
	if e := sessionOutcome("release locked probe", code, result, refused, err); e != nil {
		return e
	}
	var after releaseProbeFacts
	if err := json.Unmarshal(raw, &after); err != nil {
		return cellDeployExit(cellDeployExitRefused, "release locked probe facts unreadable: %v", err)
	}
	return releaseProbeUnchanged(before, after, checkDrives)
}

// releaseProbeUnchanged checks the release identity across a maintenance or
// lock handoff. Only the notice may change while preparing a transition.
func releaseProbeUnchanged(before, after releaseProbeFacts, checkDrives bool) error {
	if (before.Describe == nil) != (after.Describe == nil) ||
		(before.Describe != nil && *after.Describe != *before.Describe) ||
		after.Current != before.Current || after.Previous != before.Previous ||
		after.Boot != before.Boot || after.ControlEnvPin != before.ControlEnvPin ||
		after.PackageManifestSHA256 != before.PackageManifestSHA256 ||
		after.OLPHead != before.OLPHead || after.LiveAxesConf != before.LiveAxesConf ||
		(before.DriveTransitionNotice && (!after.DriveTransitionNotice || after.MaintenanceRunID != before.MaintenanceRunID || after.ServoMaintenanceRunID != before.ServoMaintenanceRunID)) ||
		(checkDrives && !reflect.DeepEqual(after.DriveReadbacks, before.DriveReadbacks)) ||
		after.unitActive("rosie-rt-core.service") != before.unitActive("rosie-rt-core.service") ||
		after.unitActive("rosie-rt-control.service") != before.unitActive("rosie-rt-control.service") {
		return cellDeployExit(cellDeployExitRefused, "cell identity changed after the first probe; retry from a fresh read, before any install")
	}
	return nil
}

// releaseStep is one existing command run in process.
type releaseStep struct {
	Name string
	Args []string // the command line after `go run .`
	run  func(args []string, stdout, stderr io.Writer) error
}

func (s releaseStep) line() string {
	return "go run . " + strings.Join(s.Args, " ")
}

// releaseSteps renders a tier's steps. caps is the commit's capabilities
// digest (verify and olp install require it after a full release). Every step
// reaches the relay over the release's --transport.
func releaseSteps(tier releaseTier, plan releasePlan, ctx cellDeployContext, commit, pin, caps, test, transport string, since []string, o releaseOptions) []releaseStep {
	m := []string{"--manifest", ctx.Manifest}
	t := []string{"--transport", transport}
	core := func(name string, transported bool, args ...string) releaseStep {
		a := append(append([]string{"realtime-core"}, args...), m...)
		if transported {
			a = append(a, t...)
		}
		return releaseStep{Name: name, Args: a, run: func(a []string, out, errw io.Writer) error { return runRealtimeCore(a[1:], out, errw) }}
	}
	build := []string{"build", "--commit", commit, "--test", "--test-level", test}
	if test == "fast" && len(since) > 0 {
		build = append(build, "--test-since", strings.Join(since, ","))
	}
	if test == "fast" {
		build = append(build, "--test-tier", string(tier))
	}
	if o.Clean {
		build = append(build, "--clean")
	}
	if o.InstallEnv {
		build = append(build, "--install-planner-env")
	}
	if o.Regate {
		build = append(build, "--regate")
	}
	olpInstall := releaseStep{Name: "olp install", Args: append(append([]string{"olp", "install", "--deployment", commit}, m...), t...),
		run: func(a []string, out, errw io.Writer) error { return runOLP(a[1:], out, errw) }}
	if o.PairMigrationFrom != "" {
		olpInstall.Args = append(olpInstall.Args, "--pair-migration")
	}
	var steps []releaseStep
	switch tier {
	case releaseTierConfig:
		config := []string{"config", "--accept-configuration", pin, "--olp", "restart"}
		if o.AcceptConfigDrift != "" {
			config = append(config, "--accept-config-drift", o.AcceptConfigDrift)
		}
		if o.AcceptHomeInvalidation {
			config = append(config, "--accept-home-invalidation")
		}
		steps = append(steps, core("build+test", true, build...), core("copy-dumps", true, "copy-dumps"), core("config", true, config...))
		if plan.OLP {
			steps = append(steps, olpInstall)
		}
	case releaseTierOLP:
		steps = append(steps, core("build+test", true, build...), olpInstall)
	case releaseTierFull:
		if o.CandidatePackage == "" {
			steps = append(steps, core("build+test", true, append(build, "--fetch-package")...))
		}
		packagePath := "<fetched package>"
		deploy := []string{"deploy", "--package", packagePath}
		if o.CandidatePackage != "" {
			packagePath = o.CandidatePackage
			deploy = []string{"deploy", "--package", packagePath, "--expect-package-sha256", o.CandidateArtifactSHA256}
		}
		if o.PairMigrationFrom != "" {
			deploy = append(deploy, "--pair-migration-from", o.PairMigrationFrom)
		}
		steps = append(steps, core("copy-dumps", true, "copy-dumps"), core("precheck", true, "precheck"),
			core("deploy", true, deploy...), core("start", true, "start"),
			core("verify", true, "verify", "--expect-capabilities", caps))
		if plan.OLP {
			olpInstall.Args = append(olpInstall.Args, "--expect-capabilities", caps)
			steps = append(steps, olpInstall)
		}
		steps = append(steps, core("prune", true, "prune"))
	}
	return steps
}

// releaseWorkingTreeMatches refuses when this checkout's copies of the files
// the steps read locally differ from the commit's: the cell manifest (the
// pin), the machine config (config compiles it), any uncommitted change under
// rt-core/config, robot_description or the cell manifest (config_drift.go),
// and for a full release the whole rt-core and robot_description trees at
// HEAD (deploy --package).
func releaseWorkingTreeMatches(repo, commit, manifest, machine string, full bool) error {
	var problems []string
	for _, p := range []string{manifest, machine} {
		if out, err := robotV4GitOutput(repo, "diff", "--name-only", commit, "--", p); err != nil || strings.TrimSpace(out) != "" {
			problems = append(problems, p+" differs from "+cellDeployShort(commit))
		}
	}
	if dirty, err := cellDeployUncommittedConfig(repo, manifest); err != nil {
		problems = append(problems, "git status failed: "+err.Error())
	} else if len(dirty) > 0 {
		problems = append(problems, cellDeployUncommittedMessage(dirty))
	}
	if full {
		head, _ := robotV4GitOutput(repo, "rev-parse", "HEAD")
		if strings.TrimSpace(head) != commit {
			problems = append(problems, fmt.Sprintf("HEAD is %.8s, not %.8s (deploy --package takes only a package built from this checkout's clean HEAD)", strings.TrimSpace(head), commit))
		}
		if dirty, _ := robotV4GitOutput(repo, "status", "--porcelain", "--", "rt-core", "robot_description"); strings.TrimSpace(dirty) != "" {
			problems = append(problems, "rt-core or robot_description has uncommitted changes")
		}
	}
	if len(problems) > 0 {
		return cellDeployExit(cellDeployExitRefused, "release refused before contacting anything:\n  - %s\nRun it from a clean checkout at the commit (git clone --branch/checkout <sha>; nothing else may differ)", strings.Join(problems, "\n  - "))
	}
	return nil
}

// releaseHostcheckFailures reads the hostcheck ids a deploy health gate
// failed on from its output. The gate's python assertion prints every check;
// only the required ones (and 'deployment') decide the gate, so only those
// can need the owner's acceptance. The first live release (one cell, 2026-10-07)
// refused on isolated, nohz_full and the /run directories, which the gate
// never required.
func releaseHostcheckFailures(output string, required []string) []string {
	var failed []string
	i := strings.Index(output, "AssertionError: {")
	if i < 0 {
		return nil
	}
	body := output[i+len("AssertionError: "):]
	if j := strings.Index(body, "}"); j >= 0 {
		body = body[:j+1]
	}
	var checks map[string]string
	if json.Unmarshal([]byte(strings.ReplaceAll(body, "'", `"`)), &checks) != nil {
		return nil
	}
	for id, status := range checks {
		if status != "pass" && (id == "deployment" || containsString(required, id)) {
			failed = append(failed, id)
		}
	}
	return failed
}

func runRealtimeCoreRelease(args []string, stdout, stderr io.Writer) error {
	if len(args) > 0 && isHelp(args[0]) {
		fmt.Fprint(stdout, releaseUsage)
		return nil
	}
	var common cellDeployCommonFlags
	var o releaseOptions
	ctx, restore, err := cellDeploySetup("realtime-core release", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&o.Commit, "commit", "", "commit to release (default HEAD; excludes --candidate)")
		fs.StringVar(&o.Candidate, "candidate", "", "verified local build-host candidate directory")
		fs.StringVar(&o.CandidateID, "candidate-id", "", "exact sha256 identity required with --candidate")
		fs.StringVar(&o.BusyStatusURL, "busy-status-url", "", "read-only Rosie dashboard /api/busy endpoint; optional cooperative preflight")
		fs.StringVar(&o.Tier, "tier", "auto", "auto, config, olp or full")
		fs.StringVar(&o.Test, "test", "", "fast or full (default: fast for config and olp, full for full)")
		fs.StringVar(&o.Rollback, "rollback", "auto", "full tier: auto rolls back to the previous release when start or verify fails; never leaves it")
		fs.StringVar(&o.PairMigrationFrom, "pair-migration-from", "", "old installed pair ID; full source release with staged PKI and automatic rollback")
		fs.StringVar(&o.DriveTransition, "drive-transition", "", "explicit stopped-drive transition: prepare or resume")
		fs.BoolVar(&o.PowerCycled, "power-cycled", false, "resume: operator confirms physical drive power cycle after prepare")
		fs.StringVar(&o.RollbackTo, "rollback-to-configuration", "", "resume: the configuration that ran before the release, when this machine has no record of it (automatic rollback target)")
		fs.IntVar(&o.RollbackToAxes, "rollback-to-axes", 0, "resume: previous Describe axis count for a paired migration without a local release record")
		fs.StringVar(&o.AcceptHostcheck, "accept-hostcheck", "", "full tier: comma-separated hostcheck ids whose failure in deploy's health gate the owner accepts")
		fs.BoolVar(&o.Plan, "plan", false, "read what the cell runs and print the tier and steps; change nothing")
		fs.BoolVar(&o.Clean, "clean", false, "from-scratch build of the build host's tree")
		fs.BoolVar(&o.InstallEnv, "install-planner-env", false, "pixi install when the planner env cannot be reused")
		fs.BoolVar(&o.Regate, "regate", false, "ignore this machine's gate records: no reuse of a pass of the same commit, and fast tests diff against the installed commits instead of each suite's last pass")
		fs.StringVar(&o.AcceptConfigDrift, "accept-config-drift", "", "the running configuration sha256 that no commit pins, whose drive values the owner accepts replacing")
		fs.BoolVar(&o.AcceptHomeInvalidation, "accept-home-invalidation", false, "config tier: owner accepts a coordinate or anchor tolerance change; the changed axes lose Home (verified) and need operator HM35")
		o.Wait.register(fs)
	}, &common, true, "--commit", "--candidate", "--candidate-id", "--busy-status-url", "--tier", "--test", "--rollback", "--pair-migration-from", "--drive-transition", "--power-cycled", "--rollback-to-configuration", "--rollback-to-axes", "--accept-hostcheck", "--accept-config-drift", "--accept-home-invalidation", "--plan", "--clean", "--install-planner-env", "--regate", "--idle-wait", "--quiet", "--interval")
	if err != nil {
		return err
	}
	defer restore()
	if err := o.Wait.validate(); err != nil {
		return err
	}
	if ctx.Deploy.ReleasePlanOnly && !o.Plan && !common.DryRun {
		return cellDeployExit(cellDeployExitRefused, "cell %s is release plan-only; review and select a deployable profile before apply", ctx.CellID)
	}
	if o.Test != "" && o.Test != "fast" && o.Test != "full" {
		return fmt.Errorf("--test must be fast or full")
	}
	if o.Rollback != "auto" && o.Rollback != "never" {
		return fmt.Errorf("--rollback must be auto or never")
	}
	if o.DriveTransition != "" {
		if o.DriveTransition != "prepare" && o.DriveTransition != "resume" {
			return fmt.Errorf("--drive-transition must be prepare or resume")
		}
		if o.Candidate != "" || o.Tier != "full" || o.AcceptConfigDrift == "" {
			return fmt.Errorf("drive transition requires a full source release and --accept-config-drift for the exact running pin")
		}
		if o.DriveTransition == "prepare" && (o.PowerCycled || o.Rollback != "auto") {
			return fmt.Errorf("drive transition prepare requires --rollback auto before any drive write and no --power-cycled")
		}
		if o.DriveTransition == "resume" && (!o.PowerCycled || o.Rollback != "never") {
			return fmt.Errorf("drive transition resume requires --power-cycled and --rollback never; software rollback cannot precede drive reversal")
		}
	} else if o.PowerCycled {
		return fmt.Errorf("--power-cycled requires --drive-transition resume")
	}
	if o.PairMigrationFrom != "" {
		if !regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]*$`).MatchString(o.PairMigrationFrom) || o.PairMigrationFrom == ctx.RTCore.PairID {
			return fmt.Errorf("--pair-migration-from must name a valid old pair distinct from the manifest pair")
		}
		if (o.Rollback != "auto" && o.DriveTransition != "resume") || ctx.RTCore.RemotePKI != rtCoreRemotePKIInstalled || o.Candidate != "" || (o.Tier != "auto" && o.Tier != "full") {
			return fmt.Errorf("pair migration requires --rollback auto, rt_core.remote_pki installed, and a full source release (not --candidate)")
		}
	}
	if o.RollbackTo != "" && !rtCoreHexSHA256.MatchString(o.RollbackTo) {
		return fmt.Errorf("--rollback-to-configuration must be a lowercase 64-hex configuration sha256")
	}
	if o.RollbackToAxes != 0 && (o.RollbackTo == "" || o.RollbackToAxes < 1 || o.RollbackToAxes > 63) {
		return fmt.Errorf("--rollback-to-axes requires --rollback-to-configuration and an axis count from 1 to 63")
	}
	if o.AcceptConfigDrift != "" && !rtCoreHexSHA256.MatchString(o.AcceptConfigDrift) {
		return fmt.Errorf("--accept-config-drift must be the running configuration's lowercase 64-hex sha256")
	}
	if _, err := releaseChoose(releaseInstalled{}, releaseTarget{}, o.Tier); err != nil && strings.Contains(err.Error(), "--tier must be") {
		return err
	}
	repo, err := repoRoot()
	if err != nil {
		return err
	}
	var candidate *releaseCandidate
	if o.Candidate != "" || o.CandidateID != "" {
		if o.Candidate == "" || o.CandidateID == "" {
			return fmt.Errorf("--candidate and --candidate-id are required together")
		}
		if o.Commit != "" {
			return fmt.Errorf("--candidate and --commit exclude each other")
		}
		if o.Tier != "auto" && o.Tier != string(releaseTierFull) {
			return fmt.Errorf("--candidate supports only --tier auto or full")
		}
		if o.Test != "" || o.Clean || o.InstallEnv || o.Regate || o.AcceptHomeInvalidation {
			return fmt.Errorf("--candidate uses the build host's sealed full release; build and config-tier flags do not apply")
		}
		if o.AcceptConfigDrift != "" && o.Rollback != "auto" {
			return fmt.Errorf("candidate configuration transition requires --rollback auto")
		}
		selected, err := releaseCandidateLoad(repo, o.Candidate, o.CandidateID)
		if err != nil {
			return err
		}
		if err := selected.validateTarget(ctx); err != nil {
			return err
		}
		candidate = &selected
		o.Commit = selected.Commit
		o.CandidatePackage = selected.Package
		o.CandidateArtifactSHA256 = selected.ArtifactSHA256
		o.CandidatePackageManifestSHA256 = selected.PackageManifestSHA256
	} else if o.Commit == "" {
		o.Commit = "HEAD"
	}
	full, _, err := cellDeployResolveCommit(repo, o.Commit)
	if err != nil {
		return err
	}
	caps, err := releaseCapabilitiesAt(repo, full)
	if err != nil {
		return err
	}
	raw, err := robotV4GitOutput(repo, "show", full+":"+ctx.Manifest)
	if err != nil {
		return fmt.Errorf("%s is not in %s: %w", ctx.Manifest, cellDeployShort(full), err)
	}
	pin, err := releasePinIn([]byte(raw))
	if err != nil {
		return err
	}
	probe := releaseProbeScript(ctx)
	if common.DryRun {
		identity := "commit"
		if candidate != nil {
			identity = "candidate " + candidate.ID
		}
		fmt.Fprintf(stdout, "realtime-core release cell=%s %s commit=%s pin=%.12s capabilities=%.12s tier=%s (dry run: nothing contacted)\n", ctx.CellID, identity, full, pin, caps, o.Tier)
		fmt.Fprintf(stdout, "# 0. cell lock: realtime-core lock holder on %s@%s (waits up to %s for another run and an idle cell, then both notices go up)\n", ctx.RelayUser, ctx.RelayPeer, o.Wait.Limit)
		fmt.Fprintf(stdout, "# 1. probe (read-only)\n%s\n", probe)
		tiers := []releaseTier{releaseTierConfig, releaseTierOLP, releaseTierFull}
		if candidate != nil {
			tiers = []releaseTier{releaseTierFull}
		} else if o.PairMigrationFrom != "" {
			tiers = []releaseTier{releaseTierFull}
		} else if o.Tier != "auto" {
			tiers = []releaseTier{releaseTier(o.Tier)}
		}
		history := releaseJournalHistory(ctx.CellID)
		for _, t := range tiers {
			test := firstNonEmpty(o.Test, releaseDefaultTestLevel(t))
			label := "tests " + test
			plan := releasePlan{OLP: true}
			if candidate != nil {
				label = "verified build-host candidate"
				plan.OLP = false
			}
			fmt.Fprintf(stdout, "# 2. tier %s (%s):\n", t, label)
			steps := releaseSteps(t, plan, ctx, full, pin, caps, test, common.Transport, []string{"<installed rt-core commit>", "<OLP tree commit>"}, o)
			if o.DriveTransition == "prepare" {
				for i, step := range steps {
					if step.Name == "deploy" {
						steps = steps[:i+1]
						break
					}
				}
			}
			for i, s := range steps {
				fmt.Fprintf(stdout, "#    %d. %s\n", i+1, s.line())
			}
			if o.DriveTransition == "prepare" {
				fmt.Fprintln(stdout, "#    next: apply only declared stopped-drive mode changes; stop before start, verify, OLP install and prune; require physical power cycle")
			}
			gate := releaseGatePlan{Level: test}
			var sel gateSelection
			if candidate == nil {
				sel, err = releaseGateSelection(repo, ctx, full, t, test, nil, o.Regate, history.Means)
				if err != nil {
					return err
				}
				gate = releaseGatePlanOf(repo, ctx, full, sel, o.Regate)
			}
			if candidate == nil && test == "fast" {
				fmt.Fprintf(stdout, "#    fast gate from this machine's records for %s (build chooses again after its probe):\n", ctx.CellID)
				for _, line := range append(sel.Describe(), sel.Summary()) {
					fmt.Fprintf(stdout, "#      %s\n", line)
				}
			}
			for _, line := range releaseEstimateSteps(history, steps, gate).lines() {
				fmt.Fprintf(stdout, "#    eta %s\n", line)
			}
		}
		return nil
	}
	if err := releaseCheckTree(repo, full, ctx.Manifest, ctx.RTCore.MachineConfig, candidate != nil || o.Tier == string(releaseTierFull)); err != nil {
		return err
	}
	if !o.Plan && o.BusyStatusURL != "" {
		if err := releaseBusyPreflight(o.BusyStatusURL, ctx.CellID); err != nil {
			return fmt.Errorf("bench busy preflight: %w", err)
		}
	}
	run, err := newCellDeployRun(ctx, "realtime-core release", common.Transport, stdout)
	if err != nil {
		return err
	}
	run.release = &releaseJournalIdentity{SourceCommit: full, ConfigurationSHA256: pin}
	if candidate != nil {
		run.release.CandidateID = candidate.ID
		run.release.TargetProfile = candidate.TargetProfile
		run.release.ArtifactSHA256 = candidate.ArtifactSHA256
		run.release.PackageManifestSHA256 = candidate.PackageManifestSHA256
	}
	fail := func(result string, e error) error {
		run.finish(result, deployExitCode(e), e.Error())
		return e
	}
	run.begin("probe")
	code, rawProbe, result, refused, err := run.session("release probe", probe)
	run.end("probe", err)
	if e := sessionOutcome("release probe", code, result, refused, err); e != nil {
		return fail("refused", e)
	}
	var facts releaseProbeFacts
	if err := json.Unmarshal(rawProbe, &facts); err != nil {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "release probe facts unreadable: %v", err))
	}
	if candidate != nil && path.Base(facts.Current) == full && facts.PackageManifestSHA256 != candidate.PackageManifestSHA256 {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "installed release %s has package manifest %.12s, not candidate %.12s; the same source commit cannot name different artifacts", cellDeployShort(full), facts.PackageManifestSHA256, candidate.PackageManifestSHA256))
	}
	saved, hasSaved := releaseLoadState(ctx.CellID)
	pending := hasSaved && saved.DriveTransitionPhase != "" && saved.DriveTransitionPhase != "verified"
	preinstallRetry := pending && o.DriveTransition == "prepare" && releaseDrivePreInstallRetry(saved, facts, full)
	if facts.DriveTransitionNotice && !pending {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "the cell has a drive-transition maintenance notice but this machine has no matching pending record; inspect the notice and cell before release"))
	}
	if pending && !facts.DriveTransitionNotice && !preinstallRetry {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "the local drive-transition record is pending but the cell's maintenance notice is missing; do not start or write drives"))
	}
	if pending && !preinstallRetry && (saved.DriveTransitionRunID == "" || saved.DriveTransitionRunID != facts.MaintenanceRunID || saved.DriveTransitionRunID != facts.ServoMaintenanceRunID) {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "the cell's drive-transition notice does not match this machine's pending record; do not start or write drives"))
	}
	if o.DriveTransition == "prepare" && (!pending || preinstallRetry) && (facts.MaintenanceRunID != "" || facts.ServoMaintenanceRunID != "") {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "another maintenance notice (%s on relay, %s on servo) is active; clear it only after checking its owner and cell state", facts.MaintenanceRunID, facts.ServoMaintenanceRunID))
	}
	if pending && !preinstallRetry {
		if o.DriveTransition == "prepare" && (saved.DriveTransitionPhase == "applying" || saved.DriveTransitionPhase == "preparing") && saved.Commit == full {
			if from, _, refuse := releaseResumePoint(facts, full); refuse != "" || from != "start" || facts.ControlEnvPin != pin || len(saved.DriveTransitionSlaves) == 0 || saved.FromConfiguration != o.AcceptConfigDrift ||
				!rtCoreHexSHA256.MatchString(facts.PackageManifestSHA256) || (saved.DriveTransitionPackageSHA256 != "" && saved.DriveTransitionPackageSHA256 != facts.PackageManifestSHA256) {
				return fail("refused", cellDeployExit(cellDeployExitRefused, "cannot retry drive writes: exact release, stopped units, target pin, previous pin and saved slave list are required (%s)", refuse))
			}
			if o.Plan {
				run.p.Step("plan", fmt.Sprintf("retry mode writes on slaves %v; both units stay stopped", saved.DriveTransitionSlaves))
				run.finish("ok", 0, "plan retry drive writes")
				return nil
			}
			unlock, err := releaseLock(run, ctx, common.Transport, full, releaseTierFull, 5*time.Minute, o.Wait)
			if err != nil {
				return fail("refused", err)
			}
			if err := releaseConfirmLockedProbe(run, probe, facts, false); err != nil {
				unlock()
				return fail("refused", err)
			}
			saved.DriveTransitionPackageSHA256 = facts.PackageManifestSHA256
			saved.DriveTransitionPhase = "applying"
			if err := releaseSaveState(saved); err != nil {
				unlock()
				return fail("refused", err)
			}
			code, _, result, refused, writeErr := run.session("drive mode apply retry", releaseDriveModeScript(ctx, full, pin, saved.DriveTransitionSlaves, "apply"))
			err = sessionOutcome("drive mode apply retry", code, result, refused, writeErr)
			if err == nil {
				saved.DriveTransitionPhase = "awaiting_power_cycle"
				err = releaseSaveState(saved)
			}
			unlock()
			if err != nil {
				return fail("failed", err)
			}
			run.p.Step("done", "drive writes complete; physically power-cycle drives, then resume; maintenance stays up")
			run.finish("awaiting_power_cycle", 0, "drive transition prepared after retry")
			return nil
		}
		if o.DriveTransition != "resume" || saved.DriveTransitionPhase != "awaiting_power_cycle" || saved.Commit != full {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "drive transition for %s is %s; do not start or roll back either controller until its drive state is checked", cellDeployShort(saved.Commit), saved.DriveTransitionPhase))
		}
	} else if o.DriveTransition == "resume" {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "no matching prepared drive transition is recorded on this machine"))
	}
	if from, why, refuse := releaseResumePoint(facts, full); refuse != "" {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "%s", refuse))
	} else if from != "" && o.Tier != string(releaseTierConfig) && o.Tier != string(releaseTierOLP) {
		if o.DriveTransition == "prepare" || (o.DriveTransition == "resume" && from != "start") {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "drive transition requires prepare on the old running release, then resume with the new release installed and both units stopped"))
		}
		if candidate != nil && o.AcceptConfigDrift != "" {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "candidate is already installed; resume without --accept-config-drift, using the saved rollback target"))
		}
		return releaseResumeRun(run, ctx, facts, from, why, full, caps, common, o, stdout, stderr)
	}
	if o.DriveTransition == "resume" {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "drive transition resume requires the selected release installed with both units stopped"))
	}
	in, err := releaseInstalledFrom(facts)
	if err != nil {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "%v", err))
	}
	target, err := releaseTargetFor(repo, full, ctx.Manifest, in)
	if err != nil {
		return fail("refused", err)
	}
	drift := releaseConfigDriftFor(repo, full, ctx.RTCore.MachineConfig, facts.LiveAxesConf, in, target)
	for _, line := range drift.lines(o.AcceptConfigDrift) {
		run.p.Step("DRIFT", line)
	}
	if err := drift.refusal(o.AcceptConfigDrift); err != nil {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "%v", err))
	}
	if candidate != nil && o.AcceptConfigDrift != "" {
		if o.AcceptConfigDrift != in.Configuration {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "--accept-config-drift %s does not name the running configuration %s", o.AcceptConfigDrift, in.Configuration))
		}
		if drift.Unproven != "" {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "candidate configuration transition cannot compare the running drive values: %s", drift.Unproven))
		}
	}
	plan, err := releaseChoose(in, target, o.Tier)
	if err != nil {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "%v", err))
	}
	if candidate != nil {
		plan.Tier = releaseTierFull
		plan.OLP = false
		plan.Reasons = append(plan.Reasons, "registered candidate selects an exact RTCore package; the active Jetson application generation is preserved")
	}
	if o.PairMigrationFrom != "" && plan.Tier != releaseTierFull {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "pair migration requires a full release; use --tier full"))
	}
	if o.PairMigrationFrom != "" {
		if in.OLPCommit == full {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "pair migration requires a new OLP tree; the active tree already names this commit"))
		}
		plan.OLP = true
		plan.Reasons = append(plan.Reasons, "pair credentials change on both controller and OLP; install the OLP even if its source is unchanged")
	}
	if plan.Tier == releaseTierFull {
		if err := releaseCheckTree(repo, full, ctx.Manifest, ctx.RTCore.MachineConfig, true); err != nil {
			return fail("refused", err)
		}
	}
	run.p.Step("installed", fmt.Sprintf("rt-core %s, OLP %s, configuration %.12s, protocol %s, capabilities %.12s", cellDeployShort(in.RTCoreCommit), cellDeployShort(in.OLPCommit), in.Configuration, in.Protocol, in.Capabilities))
	run.p.Step("commit", fmt.Sprintf("%s: configuration %.12s, capabilities %.12s", cellDeployShort(full), target.Configuration, target.Capabilities))
	for _, r := range plan.Reasons {
		run.p.Step("tier", r)
	}
	for _, n := range plan.Notes {
		run.p.Step("note", n)
	}
	if plan.Pendant {
		run.p.Step("PENDANT", fmt.Sprintf("Steam Deck v5 rebuild needed (%s); not done by release: go run . pendant build --manifest %s --commit %s, then pendant install --host <deck>",
			releaseSummarizePaths(plan.PendantReasons, 3), ctx.Manifest, cellDeployShort(full)))
	}
	test := firstNonEmpty(o.Test, releaseDefaultTestLevel(plan.Tier))
	since := releaseSince(in)
	steps := releaseSteps(plan.Tier, plan, ctx, full, pin, caps, test, common.Transport, since, o)
	if o.DriveTransition == "prepare" {
		var prepared []releaseStep
		for _, step := range steps {
			prepared = append(prepared, step)
			if step.Name == "deploy" {
				break
			}
		}
		if !releaseHasStep(prepared, "deploy") {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "drive transition requires the full deploy step"))
		}
		steps = prepared
	}
	if plan.Tier == releaseTierNone && !plan.OLP {
		run.p.Step("done", "nothing to release")
		run.finish("ok", 0, "nothing to release")
		return nil
	}
	if plan.Tier == releaseTierNone {
		// Only the OLP tree changed: the olp tier.
		plan.Tier = releaseTierOLP
		steps = releaseSteps(plan.Tier, plan, ctx, full, pin, caps, test, common.Transport, since, o)
	}
	gateLabel := "tests " + test
	if candidate != nil {
		gateLabel = "verified build-host candidate"
	}
	run.p.Step("plan", fmt.Sprintf("tier %s, %s: %d step(s)", plan.Tier, gateLabel, len(steps)))
	for i, s := range steps {
		run.p.Step("plan", fmt.Sprintf("%d. %s", i+1, s.line()))
	}
	history := releaseJournalHistory(ctx.CellID)
	gate := releaseGatePlan{Level: test}
	if candidate == nil {
		sel, err := releaseGateSelection(repo, ctx, full, plan.Tier, test, since, o.Regate, history.Means)
		if err != nil {
			return fail("refused", err)
		}
		gate = releaseGatePlanOf(repo, ctx, full, sel, o.Regate)
		if test == "fast" {
			for _, line := range append(sel.Describe(), sel.Summary()) {
				run.p.Step("gate", line)
			}
		}
	}
	eta := releaseEstimateSteps(history, steps, gate)
	for _, line := range eta.lines() {
		run.p.Step("eta", line)
	}
	var transitionSlaves []int
	if plan.Tier == releaseTierFull && target.Configuration != in.Configuration {
		compiled, compiledPin, err := releaseCompileTarget(repo, full, ctx.RTCore.MachineConfig)
		if err != nil {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "drive preflight cannot compile the target configuration: %v", err))
		}
		if compiledPin != target.Configuration {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "drive preflight compiled configuration %s, not manifest pin %s", compiledPin, target.Configuration))
		}
		if o.DriveTransition == "prepare" {
			settings, err := releaseDriveSettings(compiled, facts)
			if err != nil {
				return fail("refused", cellDeployExit(cellDeployExitRefused, "%v", err))
			}
			transitionSlaves, err = releaseDriveModeTransitionSettings(settings)
			if err != nil {
				return fail("refused", cellDeployExit(cellDeployExitRefused, "%v", err))
			}
			run.p.Step("drive transition", fmt.Sprintf("mode 4 to 1 on slaves %v; deploy stops both units, then write and verify each drive; physical drive power cycle and a separate resume are required", transitionSlaves))
		} else if err := releasePowerCyclePreflight(compiled, facts); err != nil {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "%v", err))
		} else {
			run.p.Step("drive preflight", "power-cycle startup settings match the running controller's readbacks; activation will check the drives again")
		}
	}
	if o.DriveTransition == "prepare" && len(transitionSlaves) == 0 {
		return fail("refused", cellDeployExit(cellDeployExitRefused, "drive transition prepare requires a changed power-cycle drive setting"))
	}
	if o.Plan {
		run.p.Step("done", "--plan: nothing changed")
		run.finish("ok", 0, "plan "+string(plan.Tier))
		return nil
	}

	candidateTransition := candidate != nil && o.AcceptConfigDrift != ""
	strictTransition := candidateTransition || o.PairMigrationFrom != "" || o.DriveTransition == "prepare"
	state := releaseState{Cell: ctx.CellID, Commit: full, FromRTCommit: in.RTCoreCommit, FromConfiguration: in.Configuration, FromPairID: o.PairMigrationFrom, FromAxes: in.Axes,
		FromCapabilities: in.Capabilities, FromOLPCommit: in.OLPCommit}
	if o.DriveTransition == "prepare" {
		state.DriveTransitionPhase = "preparing"
		state.DriveTransitionSlaves = transitionSlaves
		if err := runRealtimeCoreMaintenance([]string{"set", "--manifest", ctx.Manifest, "--transport", common.Transport, "--drive-transition", "--reason", "stopped drive mode transition for " + ctx.CellID, "--phase", "preparing release; no motion", "--eta", "12h"}, stdout, stderr); err != nil {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "cannot set persistent transition notice: %v", err))
		}
		code, raw, result, refused, probeErr := run.session("transition notice probe", probe)
		if err := sessionOutcome("transition notice probe", code, result, refused, probeErr); err != nil {
			return fail("refused", err)
		}
		var noticed releaseProbeFacts
		if err := json.Unmarshal(raw, &noticed); err != nil || !noticed.DriveTransitionNotice || noticed.ServoMaintenanceRunID != noticed.MaintenanceRunID {
			return fail("refused", cellDeployExit(cellDeployExitRefused, "persistent transition notice not confirmed on the relay; no install started"))
		}
		if err := releaseProbeUnchanged(facts, noticed, true); err != nil {
			return fail("refused", err)
		}
		state.DriveTransitionRunID = noticed.MaintenanceRunID
		facts = noticed
	}
	if plan.Tier == releaseTierFull && !strictTransition {
		if err := releaseSaveState(state); err != nil {
			run.p.Step("state", fmt.Sprintf("WARNING the pre-release state was not recorded (%v): a resume of this release will need --rollback-to-configuration %s", err, in.Configuration))
		}
	}
	unlock, err := releaseLock(run, ctx, common.Transport, full, plan.Tier, eta.remaining(0), o.Wait)
	if err != nil {
		return fail("refused", err)
	}
	if strictTransition {
		if err := releaseConfirmLockedProbe(run, probe, facts, o.DriveTransition == "prepare"); err != nil {
			unlock()
			return fail("refused", err)
		}
		if err := releaseSaveState(state); err != nil {
			unlock()
			return fail("refused", cellDeployExit(cellDeployExitRefused, "configuration or pair transition cannot record rollback target: %v", err))
		}
	}
	err = releaseRunSteps(run, ctx, steps, plan, in, o, eta, stdout, stderr)
	if err == nil && o.DriveTransition == "prepare" {
		code, raw, result, refused, probeErr := run.session("installed transition probe", probe)
		err = sessionOutcome("installed transition probe", code, result, refused, probeErr)
		if err == nil {
			var installed releaseProbeFacts
			if json.Unmarshal(raw, &installed) != nil || path.Base(installed.Current) != full || installed.ControlEnvPin != pin || !installed.DriveTransitionNotice || installed.ServoMaintenanceRunID != state.DriveTransitionRunID ||
				installed.unitActive("rosie-rt-core.service") != "inactive" || installed.unitActive("rosie-rt-control.service") != "inactive" ||
				!rtCoreHexSHA256.MatchString(installed.PackageManifestSHA256) {
				err = cellDeployExit(cellDeployExitFailed, "installed transition identity is not the exact stopped release; no drive write started")
			} else {
				state.DriveTransitionPackageSHA256 = installed.PackageManifestSHA256
				state.DriveTransitionPhase = "applying"
				err = releaseSaveState(state)
			}
		}
	}
	if err == nil && o.DriveTransition == "prepare" {
		code, _, result, refused, writeErr := run.session("drive mode apply", releaseDriveModeScript(ctx, full, pin, transitionSlaves, "apply"))
		err = sessionOutcome("drive mode apply", code, result, refused, writeErr)
		if err == nil {
			state.DriveTransitionPhase = "awaiting_power_cycle"
			err = releaseSaveState(state)
		}
	}
	unlock()
	if err != nil {
		return fail("failed", err)
	}
	if o.DriveTransition == "prepare" {
		run.p.Step("done", "drives written and read back; both units remain stopped. Physically power-cycle drives before --drive-transition resume --power-cycled --rollback never. Maintenance stays up.")
		run.finish("awaiting_power_cycle", 0, "drive transition prepared")
		return nil
	}
	// olp install leaves its Describe-only select to the end: the OLP
	// refuses selection while the lock's notice is up.
	if releaseHasStep(steps, "olp install") {
		err := releaseSelect(run, ctx, ctx.DeploymentDir(full))
		if err != nil {
			if o.PairMigrationFrom != "" {
				return fail("failed", fmt.Errorf("paired release installed, but post-notice OLP selection failed (%v); do not use the cell until its controller and OLP identities are checked", err))
			}
			return fail("failed", fmt.Errorf("released %s, but the new OLP's Describe-only select failed after the notice cleared (%v); olp status, then olp rollback if it persists", cellDeployShort(full), err))
		}
	}
	run.p.Step("done", fmt.Sprintf("released %s on %s (tier %s); operators re-select the cell", cellDeployShort(full), ctx.CellID, plan.Tier))
	run.finish("ok", 0, string(plan.Tier))
	return nil
}

// Seams for tests: what the cell's commit carries, the working-tree check,
// the cell lock, each step and the core commands the rollback runs.
var (
	releaseTargetFor     = releaseTargetAt
	releaseCheckTree     = releaseWorkingTreeMatches
	releaseCandidateLoad = loadReleaseCandidate
	releaseLock          = releaseTakeLock
	releaseLockLost      = cellLockLost
	// releaseNoticeProgress sends the release's progress and ETA to its
	// lock's holder (notice_progress.go).
	releaseNoticeProgress = cellLockNoticeProgress
	releaseRunStep        = func(s releaseStep, args []string, stdout, stderr io.Writer) error { return s.run(args, stdout, stderr) }
	// releaseCore is nil outside tests: runRealtimeCore (a direct reference
	// would be an initialization cycle through release itself).
	releaseCore func(args []string, stdout, stderr io.Writer) error
)

// releaseSince is what fast tests diff against: the installed rt-core and
// OLP commits.
func releaseSince(in releaseInstalled) []string {
	var out []string
	for _, c := range []string{in.RTCoreCommit, in.OLPCommit} {
		if cellDeployHexCommit.MatchString(c) && !containsString(out, c) {
			out = append(out, c)
		}
	}
	return out
}

// releaseSelectScript is a Describe-only select of the cell on the relay's
// OLP, then its jog state once the Describe reached it (olp install's own
// check, run after the release's notice cleared).
func releaseSelectScript(ctx cellDeployContext, deployment string) string {
	return "set -uo pipefail\nexport LC_ALL=C\n" + rtCoreConfigStepFunc + "\n" + rtCoreConfigReplace(`CELL=@CELL@; MODEL=@MODEL@; UNIT=@UNIT@; API=@API@
W=$(mktemp -d) || exit 1
`, "@CELL@", shellQuote(ctx.CellID), "@MODEL@", shellQuote(ctx.RobotModel), "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit),
		"@API@", shellQuote("http://"+ctx.Deploy.OLP.Listen+"/api/offline-programming/v1")) + cellDeployOLPCheckShell +
		"OLP_FAIL=\"\"\nolp_select_now " + shellQuote(deployment) + " || { printf '@@refused %s\\n' \"$OLP_FAIL\"; rm -rf \"$W\"; exit 2; }\nrm -rf \"$W\"\n"
}

func releaseSelect(run *cellDeployRun, ctx cellDeployContext, deployment string) error {
	code, _, result, refused, err := run.session("olp select", releaseSelectScript(ctx, deployment))
	return sessionOutcome("olp select", code, result, refused, err)
}

// releaseTakeLock holds the cell lock for the whole release (cellLockForRun);
// eta is the notice's first ETA (release_eta.go).
func releaseTakeLock(run *cellDeployRun, ctx cellDeployContext, transport, commit string, tier releaseTier, eta time.Duration, w cellLockWait) (func(), error) {
	reason := fmt.Sprintf("release %s (%s tier)", cellDeployShort(commit), tier)
	return cellLockForRun(run.p, ctx, transport, "release", reason, eta, w)
}

// releaseRunSteps runs the steps in order, one timing span each, and rolls a
// full release back when its start or verify fails. Every step's start and
// end (and the gate's suites, from the build inside it) update the notice's
// progress and ETA (notice_progress.go).
func releaseRunSteps(run *cellDeployRun, ctx cellDeployContext, steps []releaseStep, plan releasePlan, in releaseInstalled, o releaseOptions, eta releaseETA, stdout, stderr io.Writer) error {
	accepted := map[string]bool{}
	for _, id := range strings.Split(o.AcceptHostcheck, ",") {
		if id = strings.TrimSpace(id); id != "" {
			accepted[id] = true
		}
	}
	tracker := newNoticeTracker(eta.History, eta, eta.Level, releaseNoticeProgress, func(line string) { run.p.Step("progress", line) })
	defer noticeTrackerRegister(tracker)()
	var pkg string
	for i, s := range steps {
		args := append([]string(nil), s.Args...)
		for j, a := range args {
			if a == "<fetched package>" {
				args[j] = pkg
			}
		}
		if err := releaseLockLost(); err != nil {
			return cellDeployExit(cellDeployExitFailed, "%v; stopped before step %d/%d (%s), nothing more changed. Run the same release again: it resumes (realtime-core maintenance show first)", err, i+1, len(steps), s.Name)
		}
		tracker.Begin(i)
		run.p.Step("step", fmt.Sprintf("%d/%d %s: go run . %s", i+1, len(steps), s.Name, strings.Join(args, " ")))
		var out strings.Builder
		w := io.MultiWriter(stdout, &out)
		run.begin(s.Name)
		err := releaseRunStep(s, args, w, io.MultiWriter(stderr, &out))
		run.end(s.Name, err)
		if err != nil && s.Name == "start" && releaseStartStalled(out.String()) {
			for _, line := range releaseBringupSamples(out.String()) {
				run.p.Step("stall", line)
			}
			run.p.Step("start", "the bus bring-up did not converge and start stopped both units; starting once more before any rollback (drives in OP over time above)")
			tracker.Begin(i)
			out.Reset()
			run.begin("start retry")
			err = releaseRunStep(s, args, w, io.MultiWriter(stderr, &out))
			run.end("start retry", err)
			if err != nil {
				for _, line := range releaseBringupSamples(out.String()) {
					run.p.Step("stall", "retry: "+line)
				}
			}
		}
		if s.Name == "build+test" && err == nil {
			pkg = releaseFetchedPackage(out.String())
			if pkg == "" && releaseHasStep(steps, "deploy") {
				return cellDeployExit(cellDeployExitFailed, "build reported no fetched package")
			}
		}
		if err == nil {
			tracker.End(i, true)
			continue
		}
		if s.Name == "prune" {
			tracker.End(i, false)
			run.p.Step("prune", fmt.Sprintf("WARNING old releases were not pruned (%v); the release itself is done. Retry: go run . realtime-core prune --manifest %s", err, ctx.Manifest))
			continue
		}
		rollback := func() error {
			if o.Rollback == "never" {
				if o.DriveTransition == "resume" {
					return fmt.Errorf("%s failed after the drive transition: maintenance stays up; do not start the old release or clear the notice. Drive reversal and a second physical power cycle must precede software rollback: %w", s.Name, err)
				}
				return fmt.Errorf("%s failed; --rollback never: the new release stays installed for investigation (rollback: go run . realtime-core rollback --manifest %s --to-configuration %s): %w", s.Name, ctx.Manifest, in.Configuration, err)
			}
			run.p.Step("ROLLBACK", fmt.Sprintf("%s failed: copying the dumps off, then rolling back to %s with configuration %.12s", s.Name, cellDeployShort(in.RTCoreCommit), in.Configuration))
			back := tracker.Add("rollback", eta.Rollback.Expected)
			tracker.Begin(back)
			run.begin("rollback")
			rerr := releaseRollback(ctx, in, o, stdout, stderr)
			run.end("rollback", rerr)
			tracker.End(back, rerr == nil)
			if rerr != nil {
				return fmt.Errorf("%s failed (%v) and the rollback failed too (%v): inspect both units and pair identities on host; never SIGKILL rosie-rt-core or re-home", s.Name, err, rerr)
			}
			if s.Name == "olp install" && o.PairMigrationFrom != "" {
				return fmt.Errorf("OLP install failed; controller rolled back to %s (configuration %.12s). Check the OLP unit and old client credentials before use: %w", cellDeployShort(in.RTCoreCommit), in.Configuration, err)
			}
			return fmt.Errorf("%s failed and the cell was rolled back to %s (configuration %.12s, verified): %w", s.Name, cellDeployShort(in.RTCoreCommit), in.Configuration, err)
		}
		if s.Name == "deploy" && strings.Contains(out.String(), "rt-core health failed on") {
			failed := releaseHostcheckFailures(out.String(), rtCoreHealthRequired(ctx.RTCore.EtherCATDeviceGroup))
			var unaccepted []string
			for _, id := range failed {
				if !accepted[id] {
					unaccepted = append(unaccepted, id)
				}
			}
			if len(failed) > 0 && len(unaccepted) == 0 {
				tracker.End(i, true)
				run.p.Step("deploy", fmt.Sprintf("health gate failed only on owner-accepted hostcheck(s) %v (--accept-hostcheck): installed and stopped; continuing with start", failed))
				continue
			}
			tracker.End(i, false)
			if ((o.CandidatePackage != "" && o.AcceptConfigDrift != "") || o.PairMigrationFrom != "") && releaseDeployReached(out.String()) == releaseDeployInstalled {
				return rollback()
			}
			return cellDeployExit(deployExitCode(err), "deploy health gate failed on %v (accepted: %v): the package is installed and both units are stopped. Fix the host, or if the owner accepts these checks re-run with --accept-hostcheck; then go run . realtime-core start, verify and olp install. Never re-home", failed, o.AcceptHostcheck)
		}
		tracker.End(i, false)
		if (s.Name == "start" || s.Name == "verify" || (s.Name == "olp install" && o.PairMigrationFrom != "")) && plan.Tier == releaseTierFull {
			return rollback()
		}
		if s.Name == "deploy" && plan.Tier == releaseTierFull {
			switch releaseDeployReached(out.String()) {
			case releaseDeployInstalled:
				return rollback()
			case releaseDeployStopped:
				return cellDeployExit(deployExitCode(err), "deploy failed after stopping the controller: both units are stopped on the unchanged release %s with control.env %.12s; start them with go run . realtime-core restart --manifest %s --reason \"deploy failed\" (it starts the installed pin), then verify. Never re-home: %v", cellDeployShort(in.RTCoreCommit), in.Configuration, ctx.Manifest, err)
			}
		}
		return fmt.Errorf("%s: %w", s.Name, err)
	}
	return nil
}

const (
	releaseDeployUntouched = iota
	releaseDeployStopped
	releaseDeployInstalled
)

// The deploy session runs under set -eu and announces each phase before it
// runs (rt_core_relay_deploy.go): reaching "phase control-env" means
// host/install.sh completed and selected the new release as current with both
// units stopped; reaching "phase stop" alone means the units were stopped on
// the release that ran. Rolling back in the second case would select the
// release before the running one.
var releaseDeployPhase = regexp.MustCompile(`(?m)^\[[^\]]*\] phase +(stop|install|control-env|v4-closure|health|end)\s*$`)

func releaseDeployReached(out string) int {
	reached := releaseDeployUntouched
	for _, m := range releaseDeployPhase.FindAllStringSubmatch(out, -1) {
		switch m[1] {
		case "stop", "install":
			reached = max(reached, releaseDeployStopped)
		default:
			reached = releaseDeployInstalled
		}
	}
	return reached
}

func releaseHasStep(steps []releaseStep, name string) bool {
	for _, s := range steps {
		if s.Name == name {
			return true
		}
	}
	return false
}

// releaseStartStalled: start failed because the bus bring-up did not converge
// (rtCoreStartCommand's kind=bringup), the one failure a second clean start
// may clear; a controller running another configuration, or one that came
// up armed or granted, is never started again.
func releaseStartStalled(output string) bool {
	return strings.Contains(output, "start stall kind=bringup")
}

// releaseBringupSamples are start's bring-up lines (drives in OP per axis
// over time) and its stall summary.
func releaseBringupSamples(output string) []string {
	var out []string
	for _, line := range strings.Split(output, "\n") {
		if i := strings.Index(line, "start bring-up t="); i >= 0 {
			out = append(out, line[i:])
		} else if i := strings.Index(line, "start stall "); i >= 0 {
			out = append(out, line[i:])
		}
	}
	return out
}

// releaseFetchedPackage finds the package path build --fetch-package printed.
func releaseFetchedPackage(output string) string {
	const marker = " --package "
	for _, line := range strings.Split(output, "\n") {
		if i := strings.LastIndex(line, marker); i >= 0 && strings.Contains(line, "realtime-core deploy") {
			return strings.Fields(line[i+len(marker):])[0]
		}
	}
	return ""
}

// releaseRollback is a full release's automatic rollback: dumps off the servo
// host first (the rollback restarts rt-core), then realtime-core rollback to
// the configuration that ran before.
func releaseRollback(ctx cellDeployContext, in releaseInstalled, o releaseOptions, stdout, stderr io.Writer) error {
	m := []string{"--manifest", ctx.Manifest}
	core := releaseCore
	if core == nil {
		core = runRealtimeCore
	}
	if err := core(append([]string{"copy-dumps"}, m...), stdout, stderr); err != nil {
		return fmt.Errorf("copy-dumps before the rollback: %w", err)
	}
	args := []string{"rollback", "--to-configuration", in.Configuration}
	if o.PairMigrationFrom != "" {
		if in.Axes <= 0 || in.Axes >= 64 {
			return fmt.Errorf("cannot verify paired rollback: previous Describe axis count %d is missing or invalid", in.Axes)
		}
		args = append(args, "--restore-pair-pki-from", o.PairMigrationFrom, "--to-axes", strconv.Itoa(in.Axes))
	}
	return core(append(args, m...), stdout, stderr)
}
