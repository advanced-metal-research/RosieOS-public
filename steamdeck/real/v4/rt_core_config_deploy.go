package main

import (
	"bytes"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"time"
)

// A configuration-only deploy changes exactly one input, the cell's machine
// JSON. Binaries, drive profiles, robot descriptions, units, drop-ins, PKI and
// anchors stay the installed release's, so the only compiler that may judge the
// JSON is that release's own rtctl: configuration_sha256 hashes the input bytes,
// not compiler semantics, and an older compiler folds keys it does not know into
// deployment metadata that the running binary silently ignores (jog_jerk on
// 6c6f97c0, proven offline 2026-10-06).

const (
	rtCoreConfigExitRefused        = 2
	rtCoreConfigExitRolledBack     = 3
	rtCoreConfigExitRollbackFailed = 4
	// The robot runs the new configuration but the OLP update failed and was
	// restored; the OLP then refuses the cell on its pin check (fails closed).
	rtCoreConfigExitOLPFailed = 5
)

// rtCoreConfigHomeTimeout bounds how long apply waits, after every axis has
// verified its configuration, for home_valid_mask to return to its pre-deploy
// value. UNCONFIRMED: the 2026-10-06 6c6f97c0 and cd8c5b4c starts reported
// home_valid_mask 2 and 6 mid-restore and 63 a few seconds later; nothing was
// timed. Measure on the first run and replace this note.
const rtCoreConfigHomeTimeout = 45 * time.Second

const (
	rtCoreConfigLockPath     = "/run/lock/rosie-rt-config-deploy.lock"
	rtCoreConfigHistoryDir   = "/etc/rosie-rt-core/config-history"
	rtCoreConfigTelemetryDir = "/run/rosie-rt-core/native/telemetry"
	// Adapter-write/native-read anchor layout fixed on a RevPi servo host 2026-10-06
	// 16:12 (anchors/store.go writes 0640 only into a setgid 0750 directory).
	rtCoreConfigAnchorLayout = "rosie-ctl:rosie-rt 2750"
	rtCoreConfigAnchorMode   = "640"
)

var rtCoreHexSHA256 = regexp.MustCompile(`^[0-9a-f]{64}$`)

type rtCoreConfigOptions struct {
	ManifestPath        string
	DryRun              bool
	AcceptConfiguration string
	// AcceptConfigDrift names the running configuration when no commit pins
	// it and this run changes drive values it carries (config_drift.go).
	AcceptConfigDrift string
	// AcceptHomeInvalidation is the owner's acceptance that a coordinate or
	// anchor tolerance change refuses the saved anchors of the changed axes:
	// the apply verifies exactly those axes lose Home and they need operator
	// HM35. Without it such a change is refused as not configuration-only.
	AcceptHomeInvalidation bool
	OLP                    string
	ReadyTimeout           time.Duration
	HomeTimeout            time.Duration
	Transport              string
	Maintenance            rtCoreMaintenanceFlags
	// notice is this run's maintenance notice once it is up (RunID set); the
	// apply script then updates its phase and clears it on exit.
	notice rtCoreMaintenance
}

// rtCoreConfigBusy: an operator holds the controller (armed, enabled or a lease).
func rtCoreConfigBusy(s rtCoreConfigStatus) bool { return s.busy() }

func (o rtCoreConfigOptions) validate() error {
	if err := o.Maintenance.validate(); err != nil {
		return err
	}
	if o.OLP != "restart" && o.OLP != "skip" {
		return fmt.Errorf("--olp must be restart or skip")
	}
	if o.Transport != "tailscale" && o.Transport != "ssh" {
		return fmt.Errorf("--transport must be tailscale or ssh")
	}
	if o.AcceptConfigDrift != "" && !rtCoreHexSHA256.MatchString(o.AcceptConfigDrift) {
		return fmt.Errorf("--accept-config-drift must be the running configuration's lowercase 64-hex sha256")
	}
	for name, d := range map[string]time.Duration{"--ready-timeout": o.ReadyTimeout, "--home-timeout": o.HomeTimeout} {
		if d < time.Second || d > 10*time.Minute || d%time.Second != 0 {
			return fmt.Errorf("%s must be whole seconds between 1s and 10m", name)
		}
	}
	if o.AcceptConfiguration != "" && !rtCoreHexSHA256.MatchString(o.AcceptConfiguration) {
		return fmt.Errorf("--accept-configuration must be a lowercase 64-hex configuration sha256")
	}
	return nil
}

// rtCoreConfigExit carries the documented exit status through Run.
type rtCoreConfigExit struct {
	Code    int
	Message string
}

func (e *rtCoreConfigExit) Error() string             { return e.Message }
func (e *rtCoreConfigExit) RealtimeCoreExitCode() int { return e.Code }

func rtCoreConfigRefuse(reasons ...string) *rtCoreConfigExit {
	return &rtCoreConfigExit{Code: rtCoreConfigExitRefused, Message: "configuration deploy refused (nothing on the robot was changed):\n  - " + strings.Join(reasons, "\n  - ")}
}

// rtCoreConfigSource is what the Mac sends: always the worktree bytes of the
// cell's machine_config, plus the git facts that decide the run's label.
type rtCoreConfigSource struct {
	Repo            string
	CellManifest    string // repository-relative
	CellID          string
	MachinePath     string // repository-relative, == rt_core.machine_config
	MachineBytes    []byte
	BytesSHA256     string
	GitHead         string
	HeadBytesSame   bool   // worktree bytes == git show HEAD:<machine>
	HeadCellPin     string // HEAD's cell manifest pin ("" when absent)
	WorktreeCellPin string
	// DumpDir is the cell's relay dump directory (cell_deploy.dump_dir; ""
	// without the block): a Stop dump with a verified copy there does not
	// block the restart.
	DumpDir string
	// Uncommitted lists git status entries under rt-core/config,
	// robot_description and the cell manifest: apply refuses any.
	Uncommitted []string
	// LiveUnknown: no commit reachable from HEAD pins the running
	// configuration (git log -S, config_drift.go).
	LiveUnknown bool
}

// label is "committed" only when the bytes are HEAD's and compile to HEAD's
// cell pin; anything else runs on the robot as a trial that git does not hold.
func (s rtCoreConfigSource) label(newConfiguration string) string {
	if s.HeadBytesSame && newConfiguration != "" && newConfiguration == s.HeadCellPin {
		return "committed"
	}
	return "trial"
}

func rtCoreConfigLocalSource(repo, manifestPath string, c robotV4RTCoreConfig) (rtCoreConfigSource, error) {
	src := rtCoreConfigSource{Repo: repo, MachinePath: c.MachineConfig, WorktreeCellPin: c.ConfigurationSHA256}
	abs, err := filepath.Abs(resolveRobotV4ManifestPath(manifestPath))
	if err != nil {
		return src, err
	}
	rel, err := filepath.Rel(repo, abs)
	if err != nil || strings.HasPrefix(rel, "..") {
		return src, fmt.Errorf("cell manifest %s is outside the repository %s", abs, repo)
	}
	src.CellManifest = filepath.ToSlash(rel)
	src.CellID = strings.TrimSuffix(filepath.Base(abs), ".json")
	src.MachineBytes, err = os.ReadFile(filepath.Join(repo, filepath.FromSlash(c.MachineConfig)))
	if err != nil {
		return src, err
	}
	var header struct {
		Backend string `json:"backend"`
	}
	if err := json.Unmarshal(src.MachineBytes, &header); err != nil {
		return src, rtCoreConfigRefuse(fmt.Sprintf("%s is not valid JSON: %v", c.MachineConfig, err))
	}
	if header.Backend != "live" {
		return src, rtCoreConfigRefuse(fmt.Sprintf("%s backend is %q; a configuration-only deploy installs only a live machine configuration", c.MachineConfig, header.Backend))
	}
	sum := sha256.Sum256(src.MachineBytes)
	src.BytesSHA256 = hex.EncodeToString(sum[:])
	if src.GitHead, err = robotV4GitOutput(repo, "rev-parse", "HEAD"); err != nil {
		return src, err
	}
	if head, err := exec.Command("git", "-C", repo, "show", "HEAD:"+c.MachineConfig).Output(); err == nil {
		src.HeadBytesSame = bytes.Equal(head, src.MachineBytes)
	}
	if raw, err := exec.Command("git", "-C", repo, "show", "HEAD:"+src.CellManifest).Output(); err == nil {
		src.HeadCellPin = rtCoreConfigCellPin(raw, c.MachineConfig)
	}
	return src, nil
}

// rtCoreConfigCellPin returns the rt_core pin bound to machineConfig in a cell
// manifest, or "" when the manifest has none.
func rtCoreConfigCellPin(raw []byte, machineConfig string) string {
	var manifest struct {
		Nodes []struct {
			RTCore *struct {
				MachineConfig       string `json:"machine_config"`
				ConfigurationSHA256 string `json:"configuration_sha256"`
			} `json:"rt_core"`
		} `json:"nodes"`
	}
	if json.Unmarshal(raw, &manifest) != nil {
		return ""
	}
	for _, node := range manifest.Nodes {
		if node.RTCore != nil && node.RTCore.MachineConfig == machineConfig {
			return node.RTCore.ConfigurationSHA256
		}
	}
	return ""
}

// Probe facts, as emitted by the relay's read-only session (@@probe line).

type rtCoreCompiledFiles struct {
	Configuration string            `json:"configuration"`
	Files         map[string]string `json:"files"`
}

type rtCoreConfigCompile struct {
	Where string `json:"where"`
	RC    int    `json:"rc"`
	Error string `json:"error"`
	rtCoreCompiledFiles
}

type rtCoreConfigControlEnv struct {
	Configuration   string `json:"configuration"`
	PairID          string `json:"pair_id"`
	PairRevision    string `json:"pair_revision"`
	RemoteArgsLines int    `json:"remote_args_lines"`
}

type rtCoreConfigDescribe struct {
	ConfigurationSHA256 string `json:"configuration_sha256"`
	MachineSHA256       string `json:"machine_sha256"`
	DeploymentSHA256    string `json:"deployment_sha256"`
	CapabilitiesDigest  string `json:"capabilities_digest"`
	Backend             string `json:"backend"`
	Axes                int    `json:"axes"`
}

// rtCoreConfigStatus is rtctl status's core object: the same struct every
// cell command decodes (cellDeployStatusFacts).
type rtCoreConfigStatus = cellDeployStatusFacts

type rtCoreConfigUnit struct {
	Active    string `json:"active"`
	Enabled   string `json:"enabled"`
	NRestarts string `json:"n_restarts"`
}

type rtCoreConfigAnchor struct {
	Dir    string   `json:"dir"`
	Layout string   `json:"layout"`
	Files  []string `json:"files"` // "<name> <owner>:<group> <mode> <sha256>  <path>", sorted
}

type rtCoreConfigOLP struct {
	Unit                  string `json:"unit"`
	Dir                   string `json:"dir"`
	Source                string `json:"source"`
	Catalogue             string `json:"catalogue"`
	Listen                string `json:"listen"`
	PackageManifestSHA256 string `json:"package_manifest_sha256"`
	CellPin               string `json:"cell_pin"`
	Error                 string `json:"error"`
}

type rtCoreConfigProbe struct {
	Release               string                      `json:"release"`
	PackageManifestSHA256 string                      `json:"package_manifest_sha256"`
	ControlEnv            rtCoreConfigControlEnv      `json:"control_env"`
	Describe              rtCoreConfigDescribe        `json:"describe"`
	Status                rtCoreConfigStatus          `json:"status"`
	Units                 map[string]rtCoreConfigUnit `json:"units"`
	Anchor                rtCoreConfigAnchor          `json:"anchor"`
	StopDumps             []string                    `json:"stop_dumps"`
	StopDumpsCopied       []string                    `json:"stop_dumps_copied"`
	Locked                bool                        `json:"locked"`
	Current               rtCoreCompiledFiles         `json:"current"`
	Compile               rtCoreConfigCompile         `json:"compile"`
	OLP                   rtCoreConfigOLP             `json:"olp"`
	Errors                []string                    `json:"errors"`
}

var rtCoreConfigUnitNames = []string{"rosie-rt-core.service", "rosie-rt-control.service"}

// Compiled-output comparison (pure).

type rtCoreStartupChange struct {
	Axis int    `json:"axis"`
	Key  string `json:"key"`
	Old  string `json:"old"` // "" = absent
	New  string `json:"new"` // "" = absent
}

type rtCoreConfigDiff struct {
	OldMachine, NewMachine       string
	OldDeployment, NewDeployment string
	CoordinateIdentical          bool
	CoordinateAxes               []string // axes.conf coordinate_identity / coordinate_evidence lines that changed
	AxisCountChanged             bool
	DeploymentDrift              []string // non-"_" deployment identity keys added (+), removed (-) or changed (~)
	DeploymentNotes              []string // "_"-prefixed keys: informational only
	StartupChanges               []rtCoreStartupChange
	OtherAxesChanges             []string // reported, not refused (limits, tracking allowance, ...)
}

// rtCoreParseAxesConf splits axes.conf into lines before the first axis and
// one map per "axis=" block. Repeated keys are kept, newline-joined.
func rtCoreParseAxesConf(text string) (map[string]string, []map[string]string) {
	global := map[string]string{}
	var axes []map[string]string
	current := global
	for _, line := range strings.Split(strings.TrimSuffix(text, "\n"), "\n") {
		if line == "" {
			continue
		}
		key, value, _ := strings.Cut(line, "=")
		if key == "axis" {
			current = map[string]string{}
			axes = append(axes, current)
		}
		if previous, ok := current[key]; ok {
			value = previous + "\n" + value
		}
		current[key] = value
	}
	return global, axes
}

// rtCoreParseStartup maps a startup_sdos value
// ("key|type|index|sub|value[|power_cycle];...") to key -> value.
func rtCoreParseStartup(value string) map[string]string {
	out := map[string]string{}
	for _, item := range strings.Split(value, ";") {
		if item == "" {
			continue
		}
		parts := strings.Split(item, "|")
		if len(parts) >= 5 {
			out[parts[0]] = parts[4]
		}
	}
	return out
}

// rtCoreTuningKeys reads the new compile's tuning.json: the startup keys its
// drive tuning catalog classes as per-joint servo tuning (owner decision
// 2026-10-07). Only these may differ between axes or be removed to retain
// drive-held tuning. Missing or malformed catalog data grants neither exception.
func rtCoreTuningKeys(text string) map[string]bool {
	keys := map[string]bool{}
	var doc struct {
		TuningKeys []string `json:"tuning_keys"`
	}
	if json.Unmarshal([]byte(text), &doc) == nil {
		for _, key := range doc.TuningKeys {
			keys[key] = true
		}
	}
	return keys
}

// rtCoreIdentityKey splits a deployment.identity line at the first '=' outside a
// JSON-quoted key segment and returns the full key and its top-level segment.
func rtCoreIdentityKey(line string) (key, top string) {
	quoted, escaped := false, false
	end := len(line)
	for i := 0; i < len(line); i++ {
		ch := line[i]
		switch {
		case escaped:
			escaped = false
		case quoted && ch == '\\':
			escaped = true
		case ch == '"':
			quoted = !quoted
		case !quoted && ch == '=':
			end = i
			i = len(line)
		}
	}
	key = line[:end]
	top = key
	if strings.HasPrefix(key, `"`) {
		quoted, escaped = true, false
		for i := 1; i < len(key); i++ {
			if escaped {
				escaped = false
			} else if key[i] == '\\' {
				escaped = true
			} else if key[i] == '"' {
				top = key[:i+1]
				break
			}
		}
	} else if dot := strings.IndexByte(key, '.'); dot >= 0 {
		top = key[:dot]
	}
	return key, top
}

func rtCoreParseIdentity(text string) map[string]string {
	out := map[string]string{}
	for _, line := range strings.Split(strings.TrimSuffix(text, "\n"), "\n") {
		if line == "" {
			continue
		}
		key, _ := rtCoreIdentityKey(line)
		out[key] = strings.TrimPrefix(line[len(key):], "=")
	}
	return out
}

func rtCoreArgvValue(argvJSON, flag string) string {
	var argv []string
	if json.Unmarshal([]byte(argvJSON), &argv) != nil {
		return ""
	}
	for i := 0; i+1 < len(argv); i++ {
		if argv[i] == flag {
			return argv[i+1]
		}
	}
	return ""
}

func rtCoreConfigCompare(current, next rtCoreCompiledFiles) rtCoreConfigDiff {
	d := rtCoreConfigDiff{
		OldMachine: rtCoreArgvValue(current.Files["argv.json"], "--machine-sha256"), NewMachine: rtCoreArgvValue(next.Files["argv.json"], "--machine-sha256"),
		OldDeployment: rtCoreArgvValue(current.Files["argv.json"], "--deployment-sha256"), NewDeployment: rtCoreArgvValue(next.Files["argv.json"], "--deployment-sha256"),
		CoordinateIdentical: current.Files["coordinate.identity.json"] != "" && current.Files["coordinate.identity.json"] == next.Files["coordinate.identity.json"],
	}
	oldGlobal, oldAxes := rtCoreParseAxesConf(current.Files["axes.conf"])
	newGlobal, newAxes := rtCoreParseAxesConf(next.Files["axes.conf"])
	d.AxisCountChanged = len(oldAxes) != len(newAxes)
	for _, key := range rtCoreUnionKeys(oldGlobal, newGlobal) {
		if oldGlobal[key] != newGlobal[key] {
			d.OtherAxesChanges = append(d.OtherAxesChanges, fmt.Sprintf("global %s: %s -> %s", key, rtCoreShort(oldGlobal[key]), rtCoreShort(newGlobal[key])))
		}
	}
	for i := 0; i < len(oldAxes) || i < len(newAxes); i++ {
		o, n := map[string]string{}, map[string]string{}
		if i < len(oldAxes) {
			o = oldAxes[i]
		}
		if i < len(newAxes) {
			n = newAxes[i]
		}
		for _, key := range rtCoreUnionKeys(o, n) {
			if o[key] == n[key] {
				continue
			}
			switch key {
			case "coordinate_identity", "coordinate_evidence":
				d.CoordinateAxes = append(d.CoordinateAxes, fmt.Sprintf("axis %d %s", i, key))
			case "startup_sdos":
				oldSDO, newSDO := rtCoreParseStartup(o[key]), rtCoreParseStartup(n[key])
				for _, sk := range rtCoreUnionKeys(oldSDO, newSDO) {
					if oldSDO[sk] != newSDO[sk] {
						d.StartupChanges = append(d.StartupChanges, rtCoreStartupChange{Axis: i, Key: sk, Old: oldSDO[sk], New: newSDO[sk]})
					}
				}
			default:
				d.OtherAxesChanges = append(d.OtherAxesChanges, fmt.Sprintf("axis %d %s: %s -> %s", i, key, rtCoreShort(o[key]), rtCoreShort(n[key])))
			}
		}
	}
	oldIdentity, newIdentity := rtCoreParseIdentity(current.Files["deployment.identity"]), rtCoreParseIdentity(next.Files["deployment.identity"])
	for _, key := range rtCoreUnionKeys(oldIdentity, newIdentity) {
		ov, oldOK := oldIdentity[key]
		nv, newOK := newIdentity[key]
		if oldOK && newOK && ov == nv {
			continue
		}
		marker := "~"
		if !oldOK {
			marker = "+"
		} else if !newOK {
			marker = "-"
		}
		if _, top := rtCoreIdentityKey(key); strings.HasPrefix(top, `"_`) {
			d.DeploymentNotes = append(d.DeploymentNotes, marker+key)
		} else {
			d.DeploymentDrift = append(d.DeploymentDrift, marker+key)
		}
	}
	return d
}

func rtCoreUnionKeys(a, b map[string]string) []string {
	seen := map[string]bool{}
	var keys []string
	for _, m := range []map[string]string{a, b} {
		for k := range m {
			if !seen[k] {
				seen[k] = true
				keys = append(keys, k)
			}
		}
	}
	sort.Strings(keys)
	return keys
}

func rtCoreShort(value string) string {
	if value == "" {
		return "(absent)"
	}
	if len(value) > 72 {
		return value[:69] + "..."
	}
	return value
}

// rtCoreConfigCoordinateMask is the axes whose saved anchors a coordinate
// change refuses: the axes named in CoordinateAxes ("axis N key"), or every
// axis when coordinate.identity.json changed without per-axis lines.
func rtCoreConfigCoordinateMask(d rtCoreConfigDiff, axes int) uint32 {
	var mask uint32
	for _, line := range d.CoordinateAxes {
		var axis int
		if _, err := fmt.Sscanf(line, "axis %d", &axis); err == nil && axis >= 0 && axis < 32 {
			mask |= 1 << axis
		}
	}
	if mask == 0 || !d.CoordinateIdentical && len(d.CoordinateAxes) == 0 {
		for i := 0; i < axes && i < 32; i++ {
			mask |= 1 << i
		}
	}
	return mask
}

// rtCoreConfigPlan is everything the apply session re-checks and needs.
type rtCoreConfigPlan struct {
	Old, New string
	Label    string
	Noop     bool
	Diff     rtCoreConfigDiff
	Axes     int
	HomeMask uint32
	// NewHomeMask is the home_valid_mask the new configuration must settle at:
	// HomeMask, less the axes an accepted coordinate change invalidates.
	NewHomeMask      uint32
	Anchors          string // probe's anchor lines, compared byte-for-byte on the servo host
	AnchorDir        string
	EnablementBefore string // "<core>/<control>" systemctl is-enabled
	CoreRestarts     string
	ControlRestarts  string
	// CopiedDumps are the Stop dumps with a verified copy: the servo host
	// refuses any other dump before the restart deletes it.
	CopiedDumps []string
	// Promotions are trial keys stored unchanged by this configuration; Bridge
	// is the machine JSON the servo host starts first so their stored write
	// happens (rt_core_config_promote.go).
	Promotions   []rtCorePromotion
	Bridge       []byte
	BridgeSHA256 string
	// Drift is the running configuration's provenance and the drive values
	// this run would change when git does not hold it (config_drift.go).
	Drift configDrift
}

// rtCoreConfigEvaluate is the whole pre-robot decision. It never touches a host;
// every refusal is collected so the operator sees all blockers at once.
func rtCoreConfigEvaluate(c robotV4RTCoreConfig, src rtCoreConfigSource, p rtCoreConfigProbe, opts rtCoreConfigOptions) (rtCoreConfigPlan, error) {
	plan := rtCoreConfigPlan{Old: p.ControlEnv.Configuration, New: p.Compile.Configuration, Axes: p.Describe.Axes, HomeMask: uint32(p.Status.HomeValidMask),
		Anchors: strings.Join(p.Anchor.Files, "\n"), AnchorDir: p.Anchor.Dir,
		CoreRestarts: p.Units["rosie-rt-core.service"].NRestarts, ControlRestarts: p.Units["rosie-rt-control.service"].NRestarts}
	plan.NewHomeMask = plan.HomeMask
	plan.EnablementBefore = p.Units["rosie-rt-core.service"].Enabled + "/" + p.Units["rosie-rt-control.service"].Enabled
	var refuse []string
	for _, e := range p.Errors {
		refuse = append(refuse, "probe could not read servo/relay facts: "+e)
	}
	if p.Locked {
		refuse = append(refuse, "another configuration deploy holds "+rtCoreConfigLockPath+" on the servo host")
	}
	if !rtCoreHexSHA256.MatchString(p.ControlEnv.Configuration) {
		refuse = append(refuse, fmt.Sprintf("servo host control.env carries no configuration sha256 (%q)", p.ControlEnv.Configuration))
	}
	if p.Compile.RC != 0 || p.Compile.Error != "" || !rtCoreHexSHA256.MatchString(p.Compile.Configuration) {
		refuse = append(refuse, fmt.Sprintf("installed release %s cannot compile this machine config (%s): %s; it needs compiler or native support that release lacks: full deploy (realtime-core deploy)",
			p.Release, p.Compile.Where, strings.TrimSpace(firstNonEmpty(p.Compile.Error, fmt.Sprintf("rtctl exit %d", p.Compile.RC)))))
		return plan, rtCoreConfigRefuse(refuse...)
	}
	plan.Label = src.label(plan.New)
	plan.Diff = rtCoreConfigCompare(p.Current, p.Compile.rtCoreCompiledFiles)
	if plan.New == p.ControlEnv.Configuration && plan.New == p.Describe.ConfigurationSHA256 && len(refuse) == 0 {
		plan.Noop = true
		return plan, nil
	}
	d := plan.Diff
	if p.Current.Files["axes.conf"] == "" || p.Current.Files["coordinate.identity.json"] == "" || p.Current.Files["deployment.identity"] == "" {
		refuse = append(refuse, "could not read the running compiled configuration /etc/rosie-rt-core/compiled/"+p.ControlEnv.Configuration+" to compare against")
	} else {
		if d.AxisCountChanged {
			refuse = append(refuse, "axis count changes: full deploy")
		}
		if !d.CoordinateIdentical || len(d.CoordinateAxes) > 0 {
			what := "coordinate.identity.json"
			if len(d.CoordinateAxes) > 0 {
				what = strings.Join(d.CoordinateAxes, ", ")
			}
			if opts.AcceptHomeInvalidation {
				plan.NewHomeMask = plan.HomeMask &^ rtCoreConfigCoordinateMask(d, plan.Axes)
			} else {
				refuse = append(refuse, "coordinate identity or anchor tolerance changes ("+what+"): the saved anchors would be refused and this would invalidate Home; those joints would need operator HM35 at the mechanical home pose. Not a configuration-only change without --accept-home-invalidation (owner's call)")
			}
		}
		if len(d.DeploymentDrift) > 0 {
			refuse = append(refuse, fmt.Sprintf("deployment identity changes %v: config-only allows only \"_\"-prefixed notes there. Keys the installed compiler does not treat as machine semantics land in deployment identity and are silently ignored by the running binary (for example jog_jerk on 6c6f97c0); robot block, runtime, control, telemetry and host placement edits also need a full deploy (realtime-core deploy)", d.DeploymentDrift))
		}
		tuning := rtCoreTuningKeys(p.Compile.Files["tuning.json"])
		changed := map[string]bool{}
		for _, ch := range d.StartupChanges {
			if ch.New == "" && !tuning[ch.Key] {
				refuse = append(refuse, fmt.Sprintf("startup key %s removed on axis %d: the drive keeps its last written value (A6-EC stores SDO writes in EEPROM), so removing it neither reverts the drive nor rolls back; write the intended value explicitly", ch.Key, ch.Axis))
			}
			changed[ch.Key] = true
		}
		_, newAxes := rtCoreParseAxesConf(p.Compile.Files["axes.conf"])
		for _, key := range rtCoreSortedKeys(changed) {
			if tuning[key] {
				continue // per-joint servo tuning: values may differ between axes
			}
			values := map[string]bool{}
			present := 0
			for _, axis := range newAxes {
				if v, ok := rtCoreParseStartup(axis["startup_sdos"])[key]; ok {
					values[v] = true
					present++
				}
			}
			if len(values) > 1 || (present > 0 && present != len(newAxes)) {
				refuse = append(refuse, fmt.Sprintf("startup key %s would differ between axes (%d distinct values on %d of %d axes): it is not a per-joint tuning key of the drive's tuning catalog (compiled tuning.json), and nothing else is joint-specific (CLAUDE.md); set one value for every joint", key, len(values), present, len(newAxes)))
			}
		}
	}
	s := p.Status
	if p.Describe.Backend != "ethercat" {
		refuse = append(refuse, fmt.Sprintf("running backend is %q, not ethercat", p.Describe.Backend))
	}
	if s.Armed != 0 || s.AxisEnableMask != 0 {
		refuse = append(refuse, fmt.Sprintf("controller armed=%d axis_enable_mask=%d: disarm first; config deploy never stops a controller under an operator", s.Armed, s.AxisEnableMask))
	}
	if s.LeaseValid != 0 {
		refuse = append(refuse, "a control lease is held (lease_valid=1): an idle OLP or Steam Deck session holding the lease blocks the deploy — release it in that client and re-run")
	}
	if s.SafetyFaultMask != 0 || s.ExecutionFaultReasons != 0 {
		refuse = append(refuse, fmt.Sprintf("faults present (safety_fault_mask=%d execution_fault_reasons=%d): a restart is never used to clear a fault; resolve it at its cause first", s.SafetyFaultMask, s.ExecutionFaultReasons))
	}
	for _, alarm := range s.Alarms {
		refuse = append(refuse, "drive alarm or error: "+alarm+" (fix it at the drive; config deploy does not restart through it)")
	}
	if p.Describe.Axes <= 0 || p.Describe.Axes > 32 {
		refuse = append(refuse, fmt.Sprintf("Describe reports %d axes", p.Describe.Axes))
	} else if full := uint64(1)<<p.Describe.Axes - 1; s.ConfigVerifiedMask != full {
		refuse = append(refuse, fmt.Sprintf("config_verified_mask=%d, not %d: the running controller has not verified every axis", s.ConfigVerifiedMask, full))
	}
	if p.Describe.ConfigurationSHA256 != p.ControlEnv.Configuration {
		refuse = append(refuse, fmt.Sprintf("running Describe configuration %s differs from control.env %s", rtCoreShort(p.Describe.ConfigurationSHA256), rtCoreShort(p.ControlEnv.Configuration)))
	}
	if p.ControlEnv.PairID != c.PairID || p.ControlEnv.PairRevision != strconv.FormatUint(c.PairRevision, 10) {
		refuse = append(refuse, fmt.Sprintf("control.env pair %s/%s differs from the cell's %s/%d", p.ControlEnv.PairID, p.ControlEnv.PairRevision, c.PairID, c.PairRevision))
	}
	if c.RemotePKI == rtCoreRemotePKIInstalled && p.ControlEnv.RemoteArgsLines != 1 {
		refuse = append(refuse, fmt.Sprintf("control.env carries %d ROSIE_RT_REMOTE_ARGS lines; --keep-remote-args needs exactly one", p.ControlEnv.RemoteArgsLines))
	}
	for _, name := range rtCoreConfigUnitNames {
		u := p.Units[name]
		if u.Active != "active" {
			refuse = append(refuse, fmt.Sprintf("%s is %q: config-only restarts a running controller; start it with realtime-core start first", name, u.Active))
		}
		if u.NRestarts == "" {
			refuse = append(refuse, name+" NRestarts unreadable")
		}
	}
	if p.Anchor.Dir == "" {
		refuse = append(refuse, "rosie-rt-control has no ROSIE_RT_ANCHOR_DIR; this tool cannot prove Home anchors survive the restart")
	} else if p.Anchor.Layout != rtCoreConfigAnchorLayout {
		refuse = append(refuse, fmt.Sprintf("anchor dir %s is %q, not %q; this tool does not repair it", p.Anchor.Dir, p.Anchor.Layout, rtCoreConfigAnchorLayout))
	}
	for _, line := range p.Anchor.Files {
		if f := strings.Fields(line); len(f) < 4 || f[2] != rtCoreConfigAnchorMode {
			refuse = append(refuse, "anchor file is not mode 0640: "+line)
		}
	}
	var uncopied []string
	for _, d := range p.StopDumps {
		if containsString(p.StopDumpsCopied, d) {
			plan.CopiedDumps = append(plan.CopiedDumps, d)
		} else {
			uncopied = append(uncopied, d)
		}
	}
	if len(uncopied) > 0 {
		refuse = append(refuse, fmt.Sprintf("%d stop dump file(s) in %s without a verified copy in the cell's dump_dir (%s): run realtime-core copy-dumps first (throttled; it never decodes or deletes on the servo host); restarting deletes /run/rosie-rt-core (RuntimeDirectoryPreserve=no)", len(uncopied), rtCoreConfigTelemetryDir, strings.Join(uncopied, ", ")))
	}
	if opts.OLP == "restart" {
		if p.OLP.Error != "" {
			refuse = append(refuse, "OLP deployment not usable for --olp restart: "+p.OLP.Error+" (use --olp skip and update the OLP yourself)")
		} else if p.OLP.CellPin != p.ControlEnv.Configuration {
			refuse = append(refuse, fmt.Sprintf("OLP tree %s pins %s, not the running %s: the OLP update replaces exactly that pin (use --olp skip)", p.OLP.Source, rtCoreShort(p.OLP.CellPin), rtCoreShort(p.ControlEnv.Configuration)))
		}
	}
	if !opts.DryRun && len(src.Uncommitted) > 0 {
		refuse = append(refuse, cellDeployUncommittedMessage(src.Uncommitted))
	}
	switch {
	case opts.DryRun || plan.Label == "committed":
	case !src.HeadBytesSame && len(src.Uncommitted) == 0:
		refuse = append(refuse, fmt.Sprintf("%s differs from HEAD: config applies only a committed configuration (owner rule 2026-10-11)", src.MachinePath))
	case src.HeadBytesSame:
		refuse = append(refuse, fmt.Sprintf("%s compiles to %.12s with the installed release, but HEAD's %s pins %.12s: config applies only a committed configuration (owner rule 2026-10-11). Commit the pin the dry run prints, then run again", src.MachinePath, plan.New, src.CellManifest, src.HeadCellPin))
	}
	plan.Drift = configDrift{Live: p.ControlEnv.Configuration, Commit: plan.New, Unknown: src.LiveUnknown}
	if plan.Drift.Unknown {
		if p.Current.Files["axes.conf"] == "" {
			plan.Drift.Unproven = "the running compiled axes.conf (/etc/rosie-rt-core/compiled/" + p.ControlEnv.Configuration + "/axes.conf) was not readable"
		} else {
			plan.Drift.Changes = configTuningDrift(p.Current.Files["axes.conf"], p.Compile.Files["axes.conf"])
		}
		if err := plan.Drift.refusal(opts.AcceptConfigDrift); err != nil {
			refuse = append(refuse, err.Error())
		}
	}
	if p.Current.Files["axes.conf"] != "" {
		if promotions, err := rtCoreConfigPromotions(p.Current, p.Compile.rtCoreCompiledFiles); err != nil {
			refuse = append(refuse, "trial promotion: "+err.Error())
		} else if len(promotions) > 0 {
			ranges, err := rtCoreLoadTuningRanges(src.Repo)
			if err == nil {
				plan.Bridge, plan.Promotions, err = rtCoreConfigBridge(src.MachineBytes, promotions, ranges)
			}
			if err != nil {
				refuse = append(refuse, "keys leave startup_trial unchanged, so their stored write needs a bridge configuration, and it cannot be built: "+err.Error())
			} else {
				plan.BridgeSHA256 = rtCoreSHA256Hex(plan.Bridge)
			}
		}
	}
	if len(refuse) > 0 {
		return plan, rtCoreConfigRefuse(refuse...)
	}
	return plan, nil
}

func rtCoreSortedKeys(m map[string]bool) []string {
	keys := make([]string, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return keys
}

// rtCoreConfigAccept requires the operator to name the exact new digest. A
// non-interactive run must pass it; a terminal may type its first 12 characters.
func rtCoreConfigAccept(newConfiguration, accepted string, tty bool, prompt func(string) (string, error)) error {
	if accepted != "" {
		if accepted != newConfiguration {
			return rtCoreConfigRefuse(fmt.Sprintf("--accept-configuration %s does not match the configuration this run would apply, %s", accepted, newConfiguration))
		}
		return nil
	}
	if !tty || prompt == nil {
		return rtCoreConfigRefuse("applying needs --accept-configuration " + newConfiguration + " (non-interactive run)")
	}
	answer, err := prompt(fmt.Sprintf("apply configuration %s? type its first 12 characters to accept: ", newConfiguration))
	if err != nil {
		return rtCoreConfigRefuse("acceptance not given: " + err.Error())
	}
	if strings.TrimSpace(answer) != newConfiguration[:12] {
		return rtCoreConfigRefuse("acceptance did not match " + newConfiguration[:12])
	}
	return nil
}
