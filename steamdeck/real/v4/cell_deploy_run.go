package main

import (
	"bufio"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"regexp"
	"strings"
	"sync"
	"time"
)

// Exit codes shared by the cell deploy commands; 2 and 5 match `realtime-core
// config` (refused with nothing changed; OLP restored after a failed check).
const (
	cellDeployExitFailed   = 1
	cellDeployExitRefused  = 2
	cellDeployExitNotBack  = 4
	cellDeployExitRestored = 5
)

func cellDeployExit(code int, format string, args ...any) error {
	return &rtCoreConfigExit{Code: code, Message: fmt.Sprintf(format, args...)}
}

// cellDeployCommonFlags are on every cell deploy command.
type cellDeployCommonFlags struct {
	Manifest  string
	DryRun    bool
	Transport string
}

func (c *cellDeployCommonFlags) register(fs *flag.FlagSet, dryRun bool) {
	fs.StringVar(&c.Manifest, "manifest", "", "cell manifest (rt-core/config/cells/<cell>.json) with a cell_deploy block")
	fs.StringVar(&c.Transport, "transport", "tailscale", "how the Mac reaches the relay: tailscale or ssh")
	if dryRun {
		fs.BoolVar(&c.DryRun, "dry-run", false, "print the rendered relay scripts without contacting any host")
	}
}

// cellDeploySetup parses flags, selects the manifest and loads the context.
// The returned restore must be called when the command ends.
func cellDeploySetup(name string, args []string, stderr io.Writer, define func(*flag.FlagSet), common *cellDeployCommonFlags, dryRun bool, allowed ...string) (cellDeployContext, func(), error) {
	fs := flag.NewFlagSet(name, flag.ContinueOnError)
	fs.SetOutput(stderr)
	common.register(fs, dryRun)
	if define != nil {
		define(fs)
	}
	allowed = append(allowed, "--manifest", "--transport")
	if dryRun {
		allowed = append(allowed, "--dry-run")
	}
	if err := rejectRetiredRealtimeCoreFlags(args, allowed...); err != nil {
		return cellDeployContext{}, nil, err
	}
	if err := fs.Parse(args); err != nil {
		return cellDeployContext{}, nil, err
	}
	if fs.NArg() != 0 {
		return cellDeployContext{}, nil, fmt.Errorf("%s takes no positional arguments", name)
	}
	if common.Transport != "tailscale" && common.Transport != "ssh" {
		return cellDeployContext{}, nil, fmt.Errorf("--transport must be tailscale or ssh")
	}
	restore, err := selectRealtimeCoreManifest(common.Manifest)
	if err != nil {
		return cellDeployContext{}, nil, err
	}
	ctx, err := loadCellDeployContext()
	if err != nil {
		restore()
		return cellDeployContext{}, nil, err
	}
	return ctx, restore, nil
}

// cellDeployRun is one command's progress printer and journal: the transcript
// (rendered scripts included) in ~/.rosie/deploy-journal/<run>.log, its phase
// timings in <run>.timings.json (deploy_timings.go) and one JSON line per run
// in cell-deploy.jsonl.
type cellDeployRun struct {
	ctx       cellDeployContext
	transport string
	p         *rtCoreProgress
	command   string
	id        string
	logPath   string
	journal   string
	release   *releaseJournalIdentity
	close     func()
}

// releaseJournalIdentity records the selected release input, not a live readback.
type releaseJournalIdentity struct {
	SourceCommit          string `json:"source_commit"`
	ConfigurationSHA256   string `json:"configuration_sha256"`
	CandidateID           string `json:"candidate_id,omitempty"`
	TargetProfile         string `json:"target_profile,omitempty"`
	ArtifactSHA256        string `json:"artifact_sha256,omitempty"`
	PackageManifestSHA256 string `json:"package_manifest_sha256,omitempty"`
}

func newCellDeployRun(ctx cellDeployContext, command, transport string, stdout io.Writer) (*cellDeployRun, error) {
	r, err := newDeployJournalRun(ctx.CellID, command, stdout)
	if err != nil {
		return nil, err
	}
	r.ctx, r.transport = ctx, transport
	r.p.Step(strings.Fields(command)[0], fmt.Sprintf("%s cell=%s relay=%s@%s (%s) servo=%s", command, ctx.CellID, ctx.RelayUser, ctx.RelayPeer, transport, ctx.ServoTarget))
	return r, nil
}

// newDeployJournalRun opens the journal of one run for cell without a relay
// context; realtime-core deploy and start use it directly.
func newDeployJournalRun(cell, command string, stdout io.Writer) (*cellDeployRun, error) {
	home, err := os.UserHomeDir()
	if err != nil {
		return nil, err
	}
	dir := filepath.Join(home, ".rosie", "deploy-journal")
	if err := os.MkdirAll(dir, 0o700); err != nil {
		return nil, err
	}
	id := time.Now().UTC().Format("20060102T150405Z") + "-" + strings.ReplaceAll(command, " ", "-") + "-" + cell
	logPath := filepath.Join(dir, id+".log")
	f, err := os.OpenFile(logPath, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o600)
	if err != nil {
		return nil, err
	}
	r := &cellDeployRun{ctx: cellDeployContext{CellID: cell}, p: newRTCoreProgress(stdout, f), command: command, id: id, logPath: logPath,
		journal: filepath.Join(dir, "cell-deploy.jsonl"), close: func() { f.Close() }}
	r.p.timer = newDeployTimer(id, command, cell, r.p.start)
	return r, nil
}

func (r *cellDeployRun) finish(result string, code int, detail string) {
	timings := r.p.timer.finish(result, code, r.p.now(), r.logPath)
	entry := map[string]any{"time": time.Now().UTC().Format(time.RFC3339), "run_id": r.id, "command": r.command, "cell": r.ctx.CellID,
		"result": result, "exit": code, "detail": detail, "duration_s": float64(time.Since(r.p.start).Milliseconds()) / 1000, "log": r.logPath}
	if r.release != nil {
		entry["release"] = r.release
	}
	if err := writeDeployTimings(deployTimingsPath(r.logPath), timings); err == nil {
		entry["timings"] = deployTimingsPath(r.logPath)
	} else {
		r.p.Step("timings", "not written: "+err.Error())
	}
	line, _ := json.Marshal(entry)
	if f, err := os.OpenFile(r.journal, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o600); err == nil {
		fmt.Fprintln(f, string(line))
		f.Close()
	}
	for _, line := range deployTimingSummary(timings) {
		r.p.Step("timings", line)
	}
	r.p.Step("journal", r.logPath)
	r.close()
}

// begin and end time one Mac-side phase of the run (bundle, upload, fetch).
func (r *cellDeployRun) begin(name string) { r.p.timer.begin(name, r.p.now()) }

func (r *cellDeployRun) end(name string, err error) {
	r.p.timer.end(name, deployTimingResult(0, err), r.p.now())
}

// session runs one relay script and returns its exit code with the @@probe
// and @@result payloads it printed. The session is one timing span; the
// script's step lines become its phases.
func (r *cellDeployRun) session(label, script string) (int, []byte, []byte, []string, error) {
	return r.runSession(label, script, true)
}

// runSession is session; reset false keeps the payloads an earlier session
// of the same work printed (a watched session's re-attach).
func (r *cellDeployRun) runSession(label, script string, reset bool) (int, []byte, []byte, []string, error) {
	r.p.Record(label+" (relay script)", script)
	r.p.mu.Lock()
	if reset {
		r.p.probe, r.p.result, r.p.refused = nil, nil, nil
	}
	r.p.mu.Unlock()
	r.p.timer.begin(label, r.p.now())
	code, err := rtCoreConfigRunRemote(rtCoreConfigRemoteArgv(r.transport, r.ctx.RelayUser, r.ctx.RelayPeer, r.ctx.RelayHost, script), r.p)
	r.p.timer.end(label, deployTimingResult(code, err), r.p.now())
	r.p.mu.Lock()
	defer r.p.mu.Unlock()
	return code, r.p.probe, r.p.result, append([]string(nil), r.p.refused...), err
}

// sessionOutcome maps a session's exit to the command's error: 0 ok, 2 refused
// (nothing changed), anything else a failure carrying the result reason.
func sessionOutcome(what string, code int, result []byte, refused []string, err error) error {
	switch {
	case err != nil:
		return cellDeployExit(cellDeployExitFailed, "%s: relay session failed: %v", what, err)
	case code == 0:
		return nil
	case code == cellDeployExitRefused:
		return cellDeployExit(cellDeployExitRefused, "%s refused (nothing was changed): %s", what, strings.Join(refused, "; "))
	default:
		return cellDeployExit(code, "%s failed (exit %d): %s %s", what, code, strings.Join(refused, "; "), string(result))
	}
}

// --- facts ---------------------------------------------------------------

type cellDeployFacts struct {
	Boot                  string                         `json:"boot"`
	BootEpoch             int64                          `json:"boot_epoch"`
	NowEpoch              int64                          `json:"now_epoch"`
	Current               string                         `json:"current"`
	Previous              string                         `json:"previous"`
	ControlEnvPin         string                         `json:"control_env_pin"`
	PackageManifestSHA256 string                         `json:"package_manifest_sha256"`
	Units                 map[string]cellDeployUnitFacts `json:"units"`
	Status                *cellDeployStatusFacts         `json:"status"`
	DriveReadbacks        []cellDeployDriveReadback      `json:"drive_readbacks"`
	Describe              *cellDeployDescribeFacts       `json:"describe"`
	Anchor                rtCoreConfigAnchor             `json:"anchor"`
	Dumps                 []cellDeployDump               `json:"dumps"`
	DState                []string                       `json:"dstate"`
	EtherCAT              *cellDeployEtherCATFacts       `json:"ethercat"`
	OLP                   *cellDeployOLPFacts            `json:"olp"`
	Errors                []string                       `json:"errors"`
	Verify                *cellDeployVerifyFacts         `json:"verify"`
	PrecheckFiles         *[]string                      `json:"precheck_anchor_files"`
}

// cellDeployDriveReadback is the last startup SDO proof reported by rt-core.
// It is read-only evidence, not a new EtherCAT read.
type cellDeployDriveReadback struct {
	Axis          int    `json:"axis"`
	SlavePosition int    `json:"slave_position"`
	Key           string `json:"setting_key"`
	Type          string `json:"type"`
	Index         int    `json:"index"`
	Subindex      int    `json:"subindex"`
	Readback      int64  `json:"readback"`
	ReadbackValid bool   `json:"readback_valid"`
	Verified      bool   `json:"verified"`
}

type cellDeployUnitFacts struct {
	Active  string `json:"active"`
	Enabled string `json:"enabled"`
	MainPID string `json:"main_pid"`
}

type cellDeployStatusFacts struct {
	Armed                 uint32   `json:"armed"`
	AxisEnableMask        uint32   `json:"axis_enable_mask"`
	LeaseValid            uint32   `json:"lease_valid"`
	ConfigVerifiedMask    uint64   `json:"config_verified_mask"`
	HomeValidMask         uint64   `json:"home_valid_mask"`
	SafetyFaultMask       uint32   `json:"safety_fault_mask"`
	ExecutionFaultReasons uint32   `json:"execution_fault_reasons"`
	Alarms                []string `json:"alarms"`
	// DriveAlarms are the same alarms with the drive's 0x603F/0x203F codes,
	// which the drive profile's fault_reset list classifies (restart).
	DriveAlarms []cellDeployDriveAlarm `json:"drive_alarms"`
	// RecoveryFaults are the latched native faults with rt-core's recovery
	// class (rtctl status core.recovery.faults): restart reads them for
	// --clear-rehome-faults.
	RecoveryFaults []cellDeployRecoveryFault `json:"recovery_faults"`
	// VerifiedReferenceMask is the axes whose position is trusted, audit
	// verified, coordinate source valid and Home valid right now: a
	// coordinate fault latched on them is stale (issue #177).
	VerifiedReferenceMask uint32 `json:"verified_reference_mask"`
}

// cellDeployRecoveryFault is one latched native fault: its
// execution_fault_reasons bit, the axes it latched and the axes whose Home a
// recovery needs re-established (rt-core sdk control Recovery.Faults).
type cellDeployRecoveryFault struct {
	Bit            uint32 `json:"bit"`
	Name           string `json:"name"`
	AxisMask       uint32 `json:"axis_mask"`
	Recovery       string `json:"recovery"`
	RehomeAxisMask uint32 `json:"rehome_axis_mask"`
}

type cellDeployDriveAlarm struct {
	Axis      int    `json:"axis"`
	ErrorCode uint32 `json:"error_code"`
	Alarm     string `json:"drive_alarm"`
	Text      string `json:"text"`
	AuxCode   uint32 `json:"aux_code"`
	AuxValid  bool   `json:"aux_valid"`
}

func (a cellDeployDriveAlarm) String() string {
	aux := "0x203F unread"
	if a.AuxValid {
		aux = fmt.Sprintf("0x%04x", a.AuxCode)
	}
	return strings.TrimSpace(fmt.Sprintf("axis %d %s 0x%04x/%s %s", a.Axis, firstNonEmpty(a.Alarm, "(unnamed)"), a.ErrorCode, aux, strings.Join(strings.Fields(a.Text), " ")))
}

type cellDeployDescribeFacts struct {
	ConfigurationSHA256 string `json:"configuration_sha256"`
	MachineSHA256       string `json:"machine_sha256"`
	DeploymentSHA256    string `json:"deployment_sha256"`
	CapabilitiesDigest  string `json:"capabilities_digest"`
	Backend             string `json:"backend"`
	Axes                int    `json:"axes"`
	Protocol            string `json:"protocol"`
	PathJog             bool   `json:"path_jog"`
}

type cellDeployDump struct {
	Name   string `json:"name"`
	Bytes  int64  `json:"bytes"`
	Copied bool   `json:"copied"`
	// RecordedSHA256 is the hash copy-dumps verified against the servo host
	// (dump_dir/SHA256SUMS); Where is "relay" or "mac" for a copy that has it.
	RecordedSHA256 string `json:"recorded_sha256,omitempty"`
	Where          string `json:"where,omitempty"`
}

// cellDeployMacDumpDir is where copy-dumps --to mac keeps a cell's dumps.
func cellDeployMacDumpDir(cell string) string {
	home, err := os.UserHomeDir()
	if err != nil {
		return ""
	}
	return filepath.Join(home, ".rosie", "stop-dumps", cell)
}

// markMacCopies counts a dump as copied when this machine holds a file of its
// name and size whose sha256 is the one copy-dumps recorded on the relay (the
// relay copy may have been removed since).
func (f *cellDeployFacts) markMacCopies(cell string) {
	dir := cellDeployMacDumpDir(cell)
	for i, d := range f.Dumps {
		if d.Copied || d.RecordedSHA256 == "" || dir == "" {
			continue
		}
		p := filepath.Join(dir, d.Name)
		if info, err := os.Stat(p); err != nil || info.Size() != d.Bytes {
			continue
		}
		if sum, err := cellDeployFileSHA256(p); err == nil && sum == d.RecordedSHA256 {
			f.Dumps[i].Copied, f.Dumps[i].Where = true, "mac"
		}
	}
}

type cellDeployEtherCATFacts struct {
	Unit      string `json:"unit"`
	Phase     string `json:"phase"`
	Slaves    int    `json:"slaves"`
	MasterRaw string `json:"master_raw"`
}

type cellDeployOLPFacts struct {
	Unit           string `json:"unit"`
	Deployment     string `json:"deployment"`
	Reachable      bool   `json:"reachable"`
	Session        string `json:"session"`
	Held           bool   `json:"held"`
	RecentActivity int    `json:"recent_activity"`
}

type cellDeployVerifyFacts struct {
	Since               string   `json:"since"`
	ReadbacksVerified   int      `json:"readbacks_verified"`
	ReadbacksUnverified int      `json:"readbacks_unverified"`
	UnverifiedLines     []string `json:"unverified_lines"`
	JournalLines        []string `json:"journal_lines"`
	AnchorFiles         []string `json:"anchor_files"`
	// DC is the distributed-clock lock read (cell_deploy_dc_lock.go); nil
	// when the servo host read nothing.
	DC *cellDeployDCFacts `json:"dc,omitempty"`
}

func (f cellDeployFacts) unitActive(name string) string {
	return f.Units[name].Active
}

func (f cellDeployFacts) busy() bool {
	return f.Status != nil && f.Status.busy()
}

// busy: an operator holds the controller (armed, an axis enabled, or a
// lease). The host scripts use the same predicate (rtCoreBusyPy).
func (s cellDeployStatusFacts) busy() bool {
	return s.Armed != 0 || s.AxisEnableMask != 0 || s.LeaseValid != 0
}

func (f cellDeployFacts) uncopiedDumps() []string {
	var out []string
	for _, d := range f.Dumps {
		if !d.Copied {
			out = append(out, fmt.Sprintf("%s (%d bytes)", d.Name, d.Bytes))
		}
	}
	return out
}

// cellDeployEvaluatePrecheck is the gate before any stop of a running cell
// (tier 3 deploy, config deploy, a planned reboot): everything that would make
// a stop unsafe or lose evidence. It returns refusals and informational notes.
func cellDeployEvaluatePrecheck(ctx cellDeployContext, f cellDeployFacts) (refuse, notes []string) {
	for _, e := range f.Errors {
		refuse = append(refuse, "servo host read failed: "+e)
	}
	for _, u := range rtCoreConfigUnitNames {
		if a := f.unitActive(u); a != "active" {
			refuse = append(refuse, fmt.Sprintf("%s is %q: precheck is for a running cell (after a servo-host reboot use resume-after-reboot)", u, a))
		}
	}
	var full uint64
	if f.Describe == nil {
		refuse = append(refuse, "running Describe unavailable: cannot verify the installed axis count")
	} else {
		if f.Describe.Axes <= 0 || f.Describe.Axes > 32 {
			refuse = append(refuse, fmt.Sprintf("Describe reports %d axes", f.Describe.Axes))
		} else {
			full = uint64(1)<<uint(f.Describe.Axes) - 1
		}
		if f.ControlEnvPin == "" || f.Describe.ConfigurationSHA256 != f.ControlEnvPin {
			refuse = append(refuse, fmt.Sprintf("running Describe configuration %.12s differs from control.env %.12s", f.Describe.ConfigurationSHA256, f.ControlEnvPin))
		}
	}
	if f.Status == nil {
		refuse = append(refuse, "controller status unavailable")
	} else {
		s := f.Status
		if f.busy() {
			refuse = append(refuse, fmt.Sprintf("controller armed=%d axis_enable_mask=%d lease_valid=%d: an operator holds the cell; finish, Stop, disarm and release", s.Armed, s.AxisEnableMask, s.LeaseValid))
		}
		if s.SafetyFaultMask != 0 || s.ExecutionFaultReasons != 0 || len(s.Alarms) > 0 {
			refuse = append(refuse, fmt.Sprintf("faults present (safety_fault_mask=%d execution_fault_reasons=%d alarms=%v): a restart would clear them; report and fix at the cause first", s.SafetyFaultMask, s.ExecutionFaultReasons, s.Alarms))
		}
		if full != 0 && s.ConfigVerifiedMask != full {
			refuse = append(refuse, fmt.Sprintf("config_verified_mask=%d, not %d", s.ConfigVerifiedMask, full))
		}
		if full != 0 {
			notes = append(notes, fmt.Sprintf("home_valid_mask=%d of %d", s.HomeValidMask, full))
		}
	}
	if o := f.OLP; o != nil {
		if o.Session == "playing" || o.Session == "paused" || o.Held {
			refuse = append(refuse, fmt.Sprintf("OLP session %q held=%t: an operator is running the cell", o.Session, o.Held))
		}
		if o.RecentActivity != 0 {
			refuse = append(refuse, fmt.Sprintf("OLP journal shows %d operator action line(s) in the quiet window (or could not be read: -1)", o.RecentActivity))
		}
		notes = append(notes, fmt.Sprintf("OLP %s runs %s reachable=%t session=%q", o.Unit, firstNonEmpty(o.Deployment, "(unknown)"), o.Reachable, o.Session))
	}
	if missing := f.uncopiedDumps(); len(missing) > 0 {
		refuse = append(refuse, fmt.Sprintf("stop dump(s) without a hash-verified copy in %s or %s: %s; run realtime-core copy-dumps first (any rt-core restart or reboot deletes %s)", ctx.Deploy.DumpDir, cellDeployMacDumpDir(ctx.CellID), strings.Join(missing, ", "), rtCoreConfigTelemetryDir))
	}
	if len(f.DState) > 0 {
		refuse = append(refuse, fmt.Sprintf("rosie processes in D/Z state %v: a stop may hang in an IgH call; read /proc/<pid>/task/*/stack and ask the owner (never SIGKILL rosie-rt-core)", f.DState))
	}
	if f.ControlEnvPin != ctx.RTCore.ConfigurationSHA256 {
		notes = append(notes, fmt.Sprintf("control.env pins %.12s, the cell pins %.12s", f.ControlEnvPin, ctx.RTCore.ConfigurationSHA256))
	}
	if f.Describe != nil {
		notes = append(notes, fmt.Sprintf("Describe configuration=%.12s protocol=%s capabilities=%.12s path_jog=%t", f.Describe.ConfigurationSHA256, f.Describe.Protocol, f.Describe.CapabilitiesDigest, f.Describe.PathJog))
	}
	notes = append(notes, fmt.Sprintf("release current=%s previous=%s boot=%s", path.Base(f.Current), path.Base(f.Previous), f.Boot),
		fmt.Sprintf("anchors %s layout=%q files=%d", firstNonEmpty(f.Anchor.Dir, "(none)"), f.Anchor.Layout, len(f.Anchor.Files)))
	return refuse, notes
}

// cellDeployEvaluateResume gates a start after a servo-host reboot: the host
// must have booted after the given time, both units inactive, the EtherCAT
// master idle with exactly the machine's slaves, and nothing stuck.
func cellDeployEvaluateResume(ctx cellDeployContext, f cellDeployFacts, bootedAfter time.Time) (refuse []string) {
	for _, e := range f.Errors {
		refuse = append(refuse, "servo host read failed: "+e)
	}
	if f.BootEpoch <= bootedAfter.Unix() {
		refuse = append(refuse, fmt.Sprintf("servo host booted %s (epoch %d), not after %s: it has not rebooted since", f.Boot, f.BootEpoch, bootedAfter.UTC().Format(time.RFC3339)))
	}
	for _, u := range rtCoreConfigUnitNames {
		if a := f.unitActive(u); a != "inactive" {
			refuse = append(refuse, fmt.Sprintf("%s is %q, not inactive (enabled at boot? then use realtime-core verify)", u, a))
		}
	}
	if f.EtherCAT == nil {
		refuse = append(refuse, "EtherCAT master not read (units not inactive)")
	} else {
		if f.EtherCAT.Phase != "Idle" {
			refuse = append(refuse, fmt.Sprintf("EtherCAT master phase %q, not Idle (ethercat.service %s)", f.EtherCAT.Phase, f.EtherCAT.Unit))
		}
		if f.EtherCAT.Slaves != ctx.Slaves {
			refuse = append(refuse, fmt.Sprintf("EtherCAT lists %d slave(s), the cell expects %d", f.EtherCAT.Slaves, ctx.Slaves))
		}
	}
	if len(f.DState) > 0 {
		refuse = append(refuse, fmt.Sprintf("rosie processes in D/Z state after boot: %v", f.DState))
	}
	if f.ControlEnvPin != ctx.RTCore.ConfigurationSHA256 {
		refuse = append(refuse, fmt.Sprintf("control.env pins %q, not the cell pin %s", f.ControlEnvPin, ctx.RTCore.ConfigurationSHA256))
	}
	if f.Current == "" {
		refuse = append(refuse, "no /opt/rosie-rt-core/current release")
	}
	return refuse
}

// cellDeployVerifyExpect is what verify accepts. The zero value is the cell
// after a deploy: the manifest's pin and Home on every axis.
type cellDeployVerifyExpect struct {
	// Pin, set, is the configuration that must run instead of the
	// manifest's (restart: the one that ran before it stopped the cell).
	Pin string
	// Installed takes the pin from control.env (verify --expect-installed):
	// a cell that runs a configuration no commit pins (a Rosie 1430 arm's 66c7300b on
	// 2026-10-11) is verified against what is installed, not refused on it.
	Installed bool
	// HomeMask, set, is the Home that must be restored (every bit of it);
	// restart never homes, so it expects the Home that was valid before.
	HomeMask     *uint64
	Capabilities string
	// A stopped drive-mode change invalidates the old coordinate reference.
	// It may remove old anchors, but must not create or change an anchor.
	RequireHomeInvalidated bool
	AllowAnchorRemoval     bool
}

// cellDeployEvaluateVerify is the post-start acceptance: the cell pin running
// on ethercat with every axis verified, every startup readback verified, Home
// restored (never re-home to fix it), disarmed, no faults, the anchor files
// unchanged since the last precheck, and every DC slave's clock locked within
// cell_deploy.dc_lock.
func cellDeployEvaluateVerify(ctx cellDeployContext, f cellDeployFacts, expectCapabilities string) (refuse, notes []string) {
	return cellDeployEvaluateVerifyExpect(ctx, f, cellDeployVerifyExpect{Capabilities: expectCapabilities})
}

func cellDeployEvaluateVerifyExpect(ctx cellDeployContext, f cellDeployFacts, e cellDeployVerifyExpect) (refuse, notes []string) {
	full := ctx.FullAxisMask()
	expectCapabilities := e.Capabilities
	for _, msg := range f.Errors {
		refuse = append(refuse, "servo host read failed: "+msg)
	}
	for _, u := range rtCoreConfigUnitNames {
		if a := f.unitActive(u); a != "active" {
			refuse = append(refuse, fmt.Sprintf("%s is %q", u, a))
		}
	}
	pin, what := ctx.RTCore.ConfigurationSHA256, "the cell pin"
	switch {
	case e.Installed:
		pin, what = f.ControlEnvPin, "the installed pin"
		if !rtCoreHexSHA256.MatchString(pin) {
			refuse = append(refuse, fmt.Sprintf("control.env pins %q: no installed configuration to verify against", pin))
		} else if pin != ctx.RTCore.ConfigurationSHA256 {
			notes = append(notes, fmt.Sprintf("--expect-installed: control.env pins %.12s, the manifest pins %.12s; verifying the installed configuration (release or config changes it)", pin, ctx.RTCore.ConfigurationSHA256))
		}
	case e.Pin != "":
		pin, what = e.Pin, "the configuration that ran before"
	}
	if f.ControlEnvPin != pin {
		refuse = append(refuse, fmt.Sprintf("control.env pins %q, not %s %s", f.ControlEnvPin, what, pin))
	}
	home := full
	if e.HomeMask != nil {
		home = *e.HomeMask
	}
	if d := f.Describe; d == nil {
		refuse = append(refuse, "Describe unavailable")
	} else {
		if d.ConfigurationSHA256 != pin || d.Backend != "ethercat" || d.Axes != ctx.MachineAxes {
			refuse = append(refuse, fmt.Sprintf("Describe configuration=%s backend=%s axes=%d, want %s ethercat %d", d.ConfigurationSHA256, d.Backend, d.Axes, pin, ctx.MachineAxes))
		}
		if expectCapabilities != "" && d.CapabilitiesDigest != expectCapabilities {
			refuse = append(refuse, fmt.Sprintf("capabilities %s, expected %s", d.CapabilitiesDigest, expectCapabilities))
		}
		notes = append(notes, fmt.Sprintf("Describe machine=%.12s deployment=%.12s protocol=%s capabilities=%s path_jog=%t", d.MachineSHA256, d.DeploymentSHA256, d.Protocol, d.CapabilitiesDigest, d.PathJog))
	}
	if s := f.Status; s == nil {
		refuse = append(refuse, "status unavailable")
	} else {
		if s.ConfigVerifiedMask != full {
			refuse = append(refuse, fmt.Sprintf("config_verified_mask=%d, not %d", s.ConfigVerifiedMask, full))
		}
		if s.HomeValidMask&home != home {
			refuse = append(refuse, fmt.Sprintf("home_valid_mask=%d, not %d: report it; do NOT re-home or restore anchors to clear it", s.HomeValidMask, home))
		} else if s.HomeValidMask != full {
			notes = append(notes, fmt.Sprintf("home_valid_mask=%d of %d, as before (nothing here homes)", s.HomeValidMask, full))
		}
		if e.RequireHomeInvalidated && s.HomeValidMask != 0 {
			refuse = append(refuse, fmt.Sprintf("home_valid_mask=%d, expected 0 after drive mode transition; old Home must not be reused", s.HomeValidMask))
		}
		if f.busy() {
			refuse = append(refuse, "controller armed, enabled or leased after start")
		}
		if s.SafetyFaultMask != 0 || s.ExecutionFaultReasons != 0 || len(s.Alarms) > 0 {
			refuse = append(refuse, fmt.Sprintf("faults safety=%d execution=%d alarms=%v", s.SafetyFaultMask, s.ExecutionFaultReasons, s.Alarms))
		}
	}
	if v := f.Verify; v == nil {
		refuse = append(refuse, "startup journal not read")
	} else {
		if v.ReadbacksUnverified != 0 {
			refuse = append(refuse, fmt.Sprintf("%d startup readback(s) verified=0: %v", v.ReadbacksUnverified, v.UnverifiedLines))
		}
		if v.ReadbacksVerified == 0 {
			refuse = append(refuse, "no startup readback lines (verified=1) since rosie-rt-core started at "+v.Since)
		}
		notes = append(notes, fmt.Sprintf("startup readbacks verified=1: %d since %s", v.ReadbacksVerified, v.Since))
		dcRefuse, dcNotes := cellDeployEvaluateDCLock(ctx, v.DC)
		refuse, notes = append(refuse, dcRefuse...), append(notes, dcNotes...)
	}
	if f.Anchor.Layout != rtCoreConfigAnchorLayout {
		refuse = append(refuse, fmt.Sprintf("anchor dir %s is %q, not %q", f.Anchor.Dir, f.Anchor.Layout, rtCoreConfigAnchorLayout))
	}
	if f.PrecheckFiles == nil {
		if e.RequireHomeInvalidated {
			refuse = append(refuse, "no precheck record on the relay: cannot prove which old Home anchors were removed")
		} else {
			notes = append(notes, "no precheck record on the relay: anchor files not compared")
		}
	} else if strings.Join(*f.PrecheckFiles, "\n") != strings.Join(f.Anchor.Files, "\n") {
		old := make(map[string]bool, len(*f.PrecheckFiles))
		for _, file := range *f.PrecheckFiles {
			old[file] = true
		}
		removedOnly := len(f.Anchor.Files) < len(*f.PrecheckFiles)
		for _, file := range f.Anchor.Files {
			removedOnly = removedOnly && old[file]
		}
		if !e.AllowAnchorRemoval || !removedOnly {
			refuse = append(refuse, fmt.Sprintf("anchor files differ from the last precheck: before %v, now %v", *f.PrecheckFiles, f.Anchor.Files))
		} else {
			notes = append(notes, "old anchor files were removed during the drive mode transition; Home remains invalid")
		}
	} else {
		notes = append(notes, fmt.Sprintf("anchor files unchanged since the last precheck (%d)", len(f.Anchor.Files)))
	}
	if len(f.DState) > 0 {
		refuse = append(refuse, fmt.Sprintf("rosie processes in D/Z state: %v", f.DState))
	}
	notes = append(notes, fmt.Sprintf("release current=%s previous=%s", path.Base(f.Current), path.Base(f.Previous)))
	return refuse, notes
}

func decodeCellDeployFacts(raw []byte) (cellDeployFacts, error) {
	var f cellDeployFacts
	if len(raw) == 0 {
		return f, fmt.Errorf("no facts received")
	}
	err := json.Unmarshal(raw, &f)
	return f, err
}

func (r *cellDeployRun) report(refuse, notes []string) {
	for _, n := range notes {
		r.p.Step("fact", n)
	}
	for _, n := range refuse {
		r.p.Step("REFUSE", n)
	}
}

// --- test gate classification ---------------------------------------------

type cellDeployGateStep struct {
	Step     string   `json:"step"`
	RC       int      `json:"rc"`
	Seconds  int      `json:"seconds"`
	Failures []string `json:"failures"`
	Log      string   `json:"log"`
	// Suites are the step's slowest parts (make recipe lines, Go packages,
	// node tests), longest first.
	Suites []deployTimingSuite `json:"suites"`
}

// cellDeployKnownFailuresPath is the one list of pre-existing gate failures,
// owned and validated by rt-core's CI driver (tools/cidriver
// known_failures.go): suite, test, reason, evidence.
const cellDeployKnownFailuresPath = "rt-core/tools/cidriver/known_failures.json"

// cellDeployKnownFailures reads that list as suite -> test -> reason.
func cellDeployKnownFailures(raw []byte) (map[string]map[string]string, error) {
	var entries []struct {
		Suite, Test, Reason, Evidence string
	}
	if err := json.Unmarshal(raw, &entries); err != nil {
		return nil, fmt.Errorf("known failures: %w", err)
	}
	known := map[string]map[string]string{}
	for _, e := range entries {
		if e.Suite == "" || e.Test == "" {
			return nil, fmt.Errorf("known failures: an entry without suite or test")
		}
		if known[e.Suite] == nil {
			known[e.Suite] = map[string]string{}
		}
		known[e.Suite][e.Test] = e.Reason
	}
	return known, nil
}

// cellDeployClassifyGate splits failures into known pre-existing ones and new
// ones. A step that failed without a parsable test name is new: the gate
// cannot tell it from a regression.
func cellDeployClassifyGate(steps []cellDeployGateStep, known map[string]map[string]string) (passed, knownFails, newFails []string) {
	for _, s := range steps {
		if s.RC == 0 && len(s.Failures) == 0 {
			passed = append(passed, fmt.Sprintf("%s ok (%ds)", s.Step, s.Seconds))
			continue
		}
		if len(s.Failures) == 0 {
			newFails = append(newFails, fmt.Sprintf("%s exit %d with no failing test named (log %s)", s.Step, s.RC, s.Log))
			continue
		}
		for _, name := range s.Failures {
			parent, _, _ := strings.Cut(name, "/")
			if why, ok := known[s.Step][name]; ok {
				knownFails = append(knownFails, fmt.Sprintf("%s %s: %s", s.Step, name, why))
			} else if why, ok := known[s.Step][parent]; ok {
				knownFails = append(knownFails, fmt.Sprintf("%s %s: %s", s.Step, name, why))
			} else {
				newFails = append(newFails, fmt.Sprintf("%s %s (log %s)", s.Step, name, s.Log))
			}
		}
	}
	return passed, knownFails, newFails
}

// --- git and transfer ------------------------------------------------------

var cellDeployHexCommit = regexp.MustCompile(`^[0-9a-f]{40}$`)

// cellDeployResolveCommit returns the full commit and a local ref it is
// reachable from (git bundle refuses a bare commit). HEAD is preferred; the
// bundle may carry commits after the one built, which the build ignores.
func cellDeployResolveCommit(repo, commit string) (string, string, error) {
	full, err := robotV4GitOutput(repo, "rev-parse", "--verify", firstNonEmpty(commit, "HEAD")+"^{commit}")
	full = strings.TrimSpace(full)
	if err != nil || !cellDeployHexCommit.MatchString(full) {
		return "", "", fmt.Errorf("--commit %q is not a commit in this checkout", commit)
	}
	if exec.Command("git", "-C", repo, "merge-base", "--is-ancestor", full, "HEAD").Run() == nil {
		return full, "HEAD", nil
	}
	refs, _ := robotV4GitOutput(repo, "for-each-ref", "--contains", full, "--format=%(refname)", "refs/heads", "refs/remotes", "refs/tags")
	for _, ref := range strings.Fields(refs) {
		return full, ref, nil
	}
	return "", "", fmt.Errorf("commit %s is on no local ref; fetch or create a branch for it first", full)
}

// cellDeployBundle writes a bundle of ref, incremental over base when base is a
// local ancestor of ref (base is the commit the relay's previous tree holds).
var cellDeployBundle = func(repo, ref, base, out string) (string, error) {
	args := []string{"-C", repo, "bundle", "create", out, ref}
	kind := "full"
	if base != "" && exec.Command("git", "-C", repo, "cat-file", "-e", base+"^{commit}").Run() == nil &&
		exec.Command("git", "-C", repo, "merge-base", "--is-ancestor", base, ref).Run() == nil {
		if count, err := robotV4GitOutput(repo, "rev-list", "--count", base+".."+ref); err == nil && strings.TrimSpace(count) != "0" {
			args = append(args, "^"+base)
			kind = "incremental over " + base[:12]
		}
	}
	if output, err := exec.Command("git", args...).CombinedOutput(); err != nil {
		return "", fmt.Errorf("git bundle: %v: %s", err, strings.TrimSpace(string(output)))
	}
	return kind, nil
}

// cellDeployUpload streams a local file to the relay as stdin of argv (data,
// not a script); the remote command stores it and prints its sha256.
var cellDeployUpload = func(argv []string, file string, p *rtCoreProgress) (int, error) {
	return cellDeployExec(argv, file, nil, p)
}

// cellDeployFetch streams a relay file (stdout of argv) into a local file.
var cellDeployFetch = func(argv []string, out io.Writer, p *rtCoreProgress) (int, error) {
	return cellDeployExec(argv, "", out, p)
}

func cellDeployExec(argv []string, stdinFile string, stdout io.Writer, p *rtCoreProgress) (int, error) {
	cmd := exec.Command(argv[0], argv[1:]...)
	if stdinFile != "" {
		f, err := os.Open(stdinFile)
		if err != nil {
			return -1, err
		}
		defer f.Close()
		cmd.Stdin = f
	}
	var wg sync.WaitGroup
	if stdout != nil {
		cmd.Stdout = stdout
	} else {
		pipe, err := cmd.StdoutPipe()
		if err != nil {
			return -1, err
		}
		wg.Add(1)
		go func() {
			defer wg.Done()
			s := bufio.NewScanner(pipe)
			for s.Scan() {
				p.Remote(s.Text())
			}
		}()
	}
	stderr, err := cmd.StderrPipe()
	if err != nil {
		return -1, err
	}
	if err := cmd.Start(); err != nil {
		return -1, err
	}
	wg.Add(1)
	go func() {
		defer wg.Done()
		s := bufio.NewScanner(stderr)
		for s.Scan() {
			p.Step("stderr", s.Text())
		}
	}()
	wg.Wait()
	err = cmd.Wait()
	var exit *exec.ExitError
	if errors.As(err, &exit) {
		return exit.ExitCode(), nil
	}
	if err != nil {
		return -1, err
	}
	return 0, nil
}

func cellDeployFileSHA256(file string) (string, error) {
	f, err := os.Open(file)
	if err != nil {
		return "", err
	}
	defer f.Close()
	h := sha256.New()
	if _, err := io.Copy(h, f); err != nil {
		return "", err
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}
