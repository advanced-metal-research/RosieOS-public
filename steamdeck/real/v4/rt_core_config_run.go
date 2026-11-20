package main

import (
	"bufio"
	"bytes"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"time"
)

func runRealtimeCoreConfig(args []string, stdout, stderr io.Writer) error {
	fs := flag.NewFlagSet("realtime-core config", flag.ContinueOnError)
	fs.SetOutput(stderr)
	opts := rtCoreConfigOptions{}
	fs.StringVar(&opts.ManifestPath, "manifest", "", "approved native cell config")
	fs.BoolVar(&opts.DryRun, "dry-run", false, "probe, precompile and evaluate only (read-only on both hosts)")
	fs.StringVar(&opts.AcceptConfiguration, "accept-configuration", "", "the new configuration sha256 this run is allowed to apply")
	fs.StringVar(&opts.AcceptConfigDrift, "accept-config-drift", "", "the running configuration sha256 that no commit pins, whose drive values the owner accepts replacing")
	fs.BoolVar(&opts.AcceptHomeInvalidation, "accept-home-invalidation", false, "owner accepts a coordinate or anchor tolerance change: the changed axes lose Home (verified) and need operator HM35")
	fs.StringVar(&opts.OLP, "olp", "restart", "restart: update the relay OLP pins and restart it after the robot verifies; skip: leave the OLP untouched")
	fs.DurationVar(&opts.ReadyTimeout, "ready-timeout", time.Duration(rtCoreStartReadySeconds)*time.Second, "wait for every axis to verify the new configuration")
	fs.DurationVar(&opts.HomeTimeout, "home-timeout", rtCoreConfigHomeTimeout, "then wait for home_valid_mask to return to its pre-deploy value (UNCONFIRMED default)")
	fs.StringVar(&opts.Transport, "transport", "tailscale", "how the Mac reaches the relay: tailscale or ssh")
	opts.Maintenance.register(fs, true)
	if err := rejectRetiredRealtimeCoreFlags(args, "--manifest", "--dry-run", "--accept-configuration", "--accept-config-drift", "--accept-home-invalidation", "--olp", "--ready-timeout", "--home-timeout", "--transport",
		"--maintenance-eta", "--idle-wait", "--keep-maintenance"); err != nil {
		return err
	}
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() != 0 {
		return &retiredRuntimeError{Selection: "realtime-core config arguments"}
	}
	if err := opts.validate(); err != nil {
		return err
	}
	restore, err := selectRealtimeCoreManifest(opts.ManifestPath)
	if err != nil {
		return err
	}
	defer restore()
	opts.ManifestPath = selectedRobotV4ManifestPath()
	return runRTCoreConfig(opts, stdout, stderr)
}

// rtCoreProgress prints "[  12.3s +1.2s] phase detail" lines to the terminal
// and the run transcript, and captures the remote @@ result lines.
type rtCoreProgress struct {
	mu          sync.Mutex
	start, last time.Time
	now         func() time.Time
	out, log    io.Writer
	probe       []byte
	result      []byte
	refused     []string
	olp         []string
	timer       *deployTimer
	// watch is set while a watched session runs (remote_watch.go).
	watch *remoteWatch
	// onStep, set, sees every progress line after it is printed (the
	// notice's gate progress reads the gate's suite lines from it).
	onStep func(phase, detail string)
}

func newRTCoreProgress(out, log io.Writer) *rtCoreProgress {
	p := &rtCoreProgress{now: time.Now, out: out, log: log}
	p.start = p.now()
	p.last = p.start
	return p
}

func (p *rtCoreProgress) Step(phase, detail string) {
	p.stepAt(phase, detail, nil)
}

// stepAt prints one progress line; remoteMS is the relay script's own clock
// for an "@@step" line, kept in the run's timings.
func (p *rtCoreProgress) stepAt(phase, detail string, remoteMS *int64) {
	p.mu.Lock()
	now := p.now()
	line := fmt.Sprintf("[%6.1fs %+5.1fs] %-8s %s\n", now.Sub(p.start).Seconds(), now.Sub(p.last).Seconds(), phase, detail)
	p.last = now
	fmt.Fprint(p.out, line)
	if p.log != nil {
		fmt.Fprint(p.log, line)
	}
	p.timer.observe(phase, detail, remoteMS, now)
	hook := p.onStep
	p.mu.Unlock()
	if hook != nil {
		hook(phase, detail)
	}
}

// Record writes to the transcript only (rendered scripts, raw probe JSON).
func (p *rtCoreProgress) Record(label, text string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.log != nil {
		fmt.Fprintf(p.log, "----- %s -----\n%s\n----- end %s -----\n", label, strings.TrimRight(text, "\n"), label)
	}
}

// Remote handles one stdout line from the relay session.
func (p *rtCoreProgress) Remote(line string) {
	tag, rest, _ := strings.Cut(line, " ")
	if tag == "@@watch" {
		p.watchTouch("")
		p.watchLine(rest)
		return
	}
	p.watchTouch(line)
	switch tag {
	case "@@step":
		ms, rest, _ := strings.Cut(rest, " ") // remote milliseconds; the Mac re-stamps
		phase, detail, _ := strings.Cut(rest, " ")
		p.stepAt(phase, detail, deployTimingParseMS(ms))
	case "@@probe":
		p.mu.Lock()
		p.probe = []byte(rest)
		p.mu.Unlock()
		p.Record("probe facts", rest)
		p.Step("probe", fmt.Sprintf("facts received (%d bytes)", len(rest)))
	case "@@result":
		p.mu.Lock()
		p.result = []byte(rest)
		p.mu.Unlock()
		p.Step("result", rest)
	case "@@refused":
		p.mu.Lock()
		p.refused = append(p.refused, rest)
		p.mu.Unlock()
		p.Step("refused", rest)
	case "@@olp":
		p.mu.Lock()
		p.olp = append(p.olp, rest)
		p.mu.Unlock()
		p.Step("olp", rest)
	default:
		p.Step("remote", line)
	}
}

// rtCoreConfigRunRemote runs one relay session and streams its output. Tests
// replace it; nothing else in this command opens a connection.
var rtCoreConfigRunRemote = rtCoreRunRemoteProcess

// remoteCommandStdinBytes is the largest remote command a session puts on the
// ssh command line. sshd hands that command to the login shell as one
// argument, and Linux refuses a single argument over MAX_ARG_STRLEN (32 pages,
// 131072 bytes) with E2BIG, which ssh reports only as exit 1 with no message:
// on the relay 2026-10-10 a 132852-byte configuration apply script (one arm's
// feedforward notes on six axes in three machine files) failed twice before
// the servo host saw it; a 120000-byte `bash -c` ran and a 133000-byte one
// exited 1. Larger scripts go on stdin instead.
const remoteCommandStdinBytes = 96 * 1024

// remoteStdinRunner saves the script from stdin to a file before running it,
// so a command inside it that reads stdin (ssh, cat) sees end of input, as
// with the command-line form, instead of eating the rest of the script.
const remoteStdinRunner = `f=$(mktemp) || exit 1; trap "rm -f \"$f\"" EXIT; cat >"$f" || exit 1; bash "$f"`

// remoteScriptViaStdin rewrites a session whose remote command is
// `bash -c <shellQuote(script)>` longer than remoteCommandStdinBytes into the
// short remoteStdinRunner plus the script for stdin. Anything else, or a
// command that does not unquote back to itself, is returned unchanged.
func remoteScriptViaStdin(argv []string) ([]string, string, bool) {
	last := argv[len(argv)-1]
	quoted, ok := strings.CutPrefix(last, "bash -c ")
	if !ok || len(last) <= remoteCommandStdinBytes || len(quoted) < 2 || quoted[0] != '\'' || quoted[len(quoted)-1] != '\'' {
		return argv, "", false
	}
	script := strings.ReplaceAll(quoted[1:len(quoted)-1], `'"'"'`, `'`)
	if shellQuote(script) != quoted {
		return argv, "", false
	}
	out := append(append([]string{}, argv[:len(argv)-1]...), "bash -c "+shellQuote(remoteStdinRunner))
	return out, script, true
}

// rtCoreRunRemoteProcess is the real session runner. Under a watched session
// (p.watch) the ssh runs in its own process group (`tailscale ssh` is a
// wrapper around the real ssh), only complete stdout lines count as
// delivered, and a watchdog ends the session with a remoteStallError when it
// stays silent past Silence or runs past Deadline.
func rtCoreRunRemoteProcess(argv []string, p *rtCoreProgress) (int, error) {
	argv, script, viaStdin := remoteScriptViaStdin(argv)
	cmd := exec.Command(argv[0], argv[1:]...)
	cmd.Stdin = nil
	if viaStdin {
		cmd.Stdin = strings.NewReader(script)
	}
	p.mu.Lock()
	watched := p.watch != nil
	if watched {
		p.watch.activity = p.now()
		cellLockDetach(cmd)
	}
	p.mu.Unlock()
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		return -1, err
	}
	stderr, err := cmd.StderrPipe()
	if err != nil {
		return -1, err
	}
	if err := cmd.Start(); err != nil {
		return -1, err
	}
	var wg sync.WaitGroup
	wg.Add(2)
	go func() {
		defer wg.Done()
		scanner := bufio.NewScanner(stdout)
		scanner.Buffer(make([]byte, 0, 64*1024), 16*1024*1024)
		if watched {
			scanner.Split(remoteScanCompleteLines)
		}
		for scanner.Scan() {
			line := scanner.Text()
			if watched {
				p.mu.Lock()
				if p.watch != nil {
					p.watch.delivered += int64(len(line)) + 1
				}
				p.mu.Unlock()
			}
			p.Remote(line)
		}
	}()
	go func() {
		defer wg.Done()
		scanner := bufio.NewScanner(stderr)
		scanner.Buffer(make([]byte, 0, 64*1024), 1024*1024)
		for scanner.Scan() {
			line := scanner.Text()
			if watched && strings.HasPrefix(line, "@@watch ") {
				p.Remote(line)
				continue
			}
			p.watchTouch("")
			p.Step("stderr", line)
		}
	}()
	var stall *remoteStallError
	var escalate *time.Timer
	if watched {
		done := make(chan struct{})
		defer close(done)
		var once sync.Once
		go func() {
			tick := time.NewTicker(remoteWatchTick)
			defer tick.Stop()
			for {
				select {
				case <-done:
					return
				case <-tick.C:
				}
				p.mu.Lock()
				reason, overrun := "", false
				if p.watch != nil {
					reason, overrun = p.watch.expired(p.now())
				}
				p.mu.Unlock()
				if reason == "" {
					continue
				}
				once.Do(func() {
					p.mu.Lock()
					stall = &remoteStallError{Reason: reason, Overrun: overrun}
					remoteKillGroup(cmd.Process, false)
					escalate = time.AfterFunc(5*time.Second, func() { remoteKillGroup(cmd.Process, true) })
					p.mu.Unlock()
				})
				return
			}
		}()
	}
	wg.Wait()
	err = cmd.Wait()
	p.mu.Lock()
	stalled := stall
	if escalate != nil {
		escalate.Stop()
	}
	p.mu.Unlock()
	if stalled != nil {
		return -1, stalled
	}
	var exit *exec.ExitError
	if errors.As(err, &exit) {
		return exit.ExitCode(), nil
	}
	if err != nil {
		return -1, err
	}
	return 0, nil
}

// remoteScanCompleteLines is bufio.ScanLines without the unterminated last
// line at EOF: a watched stream cut mid-line is resent whole by the
// re-attach, which resumes at the last complete line.
func remoteScanCompleteLines(data []byte, atEOF bool) (int, []byte, error) {
	if i := bytes.IndexByte(data, '\n'); i >= 0 {
		return i + 1, bytes.TrimSuffix(data[:i], []byte("\r")), nil
	}
	if atEOF {
		return len(data), nil, nil
	}
	return 0, nil, nil
}

var rtCoreConfigStdinIsTTY = func() bool {
	info, err := os.Stdin.Stat()
	return err == nil && info.Mode()&os.ModeCharDevice != 0
}

var rtCoreConfigPrompt = func(question string) (string, error) {
	fmt.Fprint(os.Stderr, question)
	line, err := bufio.NewReader(os.Stdin).ReadString('\n')
	if err != nil && line == "" {
		return "", err
	}
	return line, nil
}

type rtCoreConfigJournalLine struct {
	Time                  string                `json:"time"`
	RunID                 string                `json:"run_id"`
	Cell                  string                `json:"cell"`
	Label                 string                `json:"label,omitempty"`
	GitHead               string                `json:"git_head"`
	MachinePath           string                `json:"machine_path"`
	MachineBytesSHA256    string                `json:"machine_bytes_sha256"`
	Old                   string                `json:"old,omitempty"`
	New                   string                `json:"new,omitempty"`
	Release               string                `json:"release,omitempty"`
	PackageManifestSHA256 string                `json:"package_manifest_sha256,omitempty"`
	StartupChanges        []rtCoreStartupChange `json:"startup_changes,omitempty"`
	DryRun                bool                  `json:"dry_run"`
	Result                string                `json:"result"`
	Exit                  int                   `json:"exit"`
	Detail                string                `json:"detail,omitempty"`
	DurationSeconds       float64               `json:"duration_s"`
	Transport             string                `json:"transport"`
	Log                   string                `json:"log"`
	Timings               string                `json:"timings,omitempty"`
}

func runRTCoreConfig(opts rtCoreConfigOptions, stdout, stderr io.Writer) error {
	node, target, via, viaPeer, err := rtCoreRelayRoute()
	if err != nil {
		return err
	}
	c := *node.RTCore
	if err := c.validate(); err != nil {
		return err
	}
	relayUser, relayHost, _ := strings.Cut(via, "@")
	if relayHost == "" {
		relayUser, relayHost = "", via
	}
	repo, err := repoRoot()
	if err != nil {
		return err
	}
	src, err := rtCoreConfigSourceFor(repo, opts.ManifestPath, c)
	if err != nil {
		return err
	}
	if raw, err := os.ReadFile(filepath.Join(repo, filepath.FromSlash(src.CellManifest))); err == nil {
		if d, err := decodeCellDeployConfig(raw); err == nil {
			src.DumpDir = d.DumpDir
		}
	}
	if src.Uncommitted, err = rtCoreConfigUncommitted(repo, src.CellManifest); err != nil {
		return err
	}
	if len(src.Uncommitted) > 0 && !opts.DryRun {
		return rtCoreConfigRefuse(cellDeployUncommittedMessage(src.Uncommitted) + " (refused before contacting anything; --dry-run still probes and prints the pin an edit compiles to)")
	}
	olpUnit, err := rtCoreConfigOLPUnit(filepath.Join(repo, filepath.FromSlash(src.CellManifest)), opts.OLP)
	if err != nil {
		return err
	}

	runID := time.Now().UTC().Format("20060102T150405Z") + "-" + src.CellID
	home, err := os.UserHomeDir()
	if err != nil {
		return err
	}
	journalDir := filepath.Join(home, ".rosie", "deploy-journal")
	if err := os.MkdirAll(journalDir, 0o700); err != nil {
		return err
	}
	logPath := filepath.Join(journalDir, runID+".log")
	logFile, err := os.OpenFile(logPath, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o600)
	if err != nil {
		return err
	}
	defer logFile.Close()
	p := newRTCoreProgress(stdout, logFile)
	p.timer = newDeployTimer(runID, "realtime-core config", src.CellID, p.start)
	entry := rtCoreConfigJournalLine{Time: time.Now().UTC().Format(time.RFC3339), RunID: runID, Cell: src.CellID, GitHead: src.GitHead,
		MachinePath: src.MachinePath, MachineBytesSHA256: src.BytesSHA256, DryRun: opts.DryRun, Transport: opts.Transport, Log: logPath}
	finish := func(result string, exit int, detail string) {
		entry.Result, entry.Exit, entry.Detail = result, exit, detail
		entry.DurationSeconds = float64(time.Since(p.start).Milliseconds()) / 1000
		timings := p.timer.finish(result, exit, p.now(), logPath)
		if err := writeDeployTimings(deployTimingsPath(logPath), timings); err == nil {
			entry.Timings = deployTimingsPath(logPath)
		}
		if raw, err := json.Marshal(entry); err == nil {
			if f, err := os.OpenFile(filepath.Join(journalDir, "rt-core-config.jsonl"), os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o600); err == nil {
				fmt.Fprintln(f, string(raw))
				f.Close()
			}
		}
		for _, line := range deployTimingSummary(timings) {
			p.Step("timings", line)
		}
		p.Step("journal", logPath)
	}
	relay := func(label, script string) (int, error) {
		p.timer.begin(label, p.now())
		code, err := rtCoreConfigRunRemote(rtCoreConfigRemoteArgv(opts.Transport, relayUser, viaPeer, relayHost, script), p)
		p.timer.end(label, deployTimingResult(code, err), p.now())
		return code, err
	}

	p.Step("config", fmt.Sprintf("cell=%s machine=%s bytes_sha256=%s head=%.12s", src.CellID, src.MachinePath, src.BytesSHA256[:12], src.GitHead))
	p.Step("config", fmt.Sprintf("worktree bytes == HEAD: %t; HEAD cell pin %.12s; worktree cell pin %.12s", src.HeadBytesSame, src.HeadCellPin, src.WorktreeCellPin))
	relayShown := remoteTarget(relayUser, relayHost)
	if opts.Transport == "tailscale" {
		relayShown = remoteTarget(relayUser, viaPeer)
	}
	p.Step("config", fmt.Sprintf("relay %s via %s; servo host %s; olp=%s (%s); dry_run=%t", relayShown, opts.Transport, target, opts.OLP, firstNonEmpty(olpUnit, "none"), opts.DryRun))

	probeScript := rtCoreConfigProbeCommand(src, target, runID, olpUnit)
	p.Record("probe script (relay)", probeScript)
	p.Step("probe", "connecting (read-only session)")
	code, err := relay("probe", probeScript)
	if err != nil || code != 0 || len(p.probe) == 0 {
		detail := fmt.Sprintf("probe session failed (exit %d, err %v): %s", code, err, strings.Join(p.refused, "; "))
		finish("probe_failed", rtCoreConfigExitRefused, detail)
		return rtCoreConfigRefuse(detail)
	}
	var probe rtCoreConfigProbe
	if err := json.Unmarshal(p.probe, &probe); err != nil {
		finish("probe_failed", rtCoreConfigExitRefused, err.Error())
		return rtCoreConfigRefuse("probe facts unreadable: " + err.Error())
	}
	entry.Old, entry.New, entry.Release, entry.PackageManifestSHA256 = probe.ControlEnv.Configuration, probe.Compile.Configuration, probe.Release, probe.PackageManifestSHA256
	if live := probe.ControlEnv.Configuration; rtCoreHexSHA256.MatchString(live) && live != probe.Compile.Configuration {
		known, err := rtCoreConfigPinCommits(repo, "HEAD", live)
		src.LiveUnknown = err != nil || len(known) == 0
	}
	plan, evalErr := rtCoreConfigEvaluate(c, src, probe, opts)
	// Busy is the one refusal worth waiting out: with the maintenance notice up,
	// operators are told to finish, and the servo host's own preflight re-checks
	// disarmed and no lease before anything changes. Every other refusal stands
	// at once, with no notice.
	waitForIdle := false
	if evalErr != nil && !opts.DryRun && opts.Maintenance.IdleWait > 0 && rtCoreConfigBusy(probe.Status) {
		idle := probe
		idle.Status.Armed, idle.Status.AxisEnableMask, idle.Status.LeaseValid = 0, 0, 0
		if idlePlan, err := rtCoreConfigEvaluate(c, src, idle, opts); err == nil && !idlePlan.Noop {
			plan, evalErr, waitForIdle = idlePlan, nil, true
		}
	}
	entry.Label, entry.StartupChanges = plan.Label, plan.Diff.StartupChanges
	rtCoreConfigReport(p, src, probe, plan)
	if evalErr != nil {
		for _, line := range strings.Split(evalErr.Error(), "\n") {
			p.Step("refused", strings.TrimSpace(line))
		}
		finish("refused", rtCoreConfigExitRefused, evalErr.Error())
		return evalErr
	}
	if plan.Noop {
		p.Step("done", fmt.Sprintf("no-op: the robot already runs %s (%s); nothing restarted", plan.New, plan.Label))
		finish("noop", 0, "")
		return nil
	}
	if opts.DryRun {
		if len(src.Uncommitted) > 0 || plan.Label != "committed" {
			p.Step("WARNING", fmt.Sprintf("%s (%s) is not committed: an apply refuses it (owner rule 2026-10-11). Commit the machine JSON and pin %s in %s and its mirrors, then release", plan.New, plan.Label, plan.New, src.CellManifest))
		}
		p.Step("dry-run", fmt.Sprintf("would apply %s (%s) replacing %s; nothing was changed", plan.New, plan.Label, plan.Old))
		p.Step("dry-run", fmt.Sprintf("apply with: go run . realtime-core config --manifest %s --accept-configuration %s", src.CellManifest, plan.New))
		finish("dry-run", 0, "")
		return nil
	}
	if err := rtCoreConfigAccept(plan.New, opts.AcceptConfiguration, rtCoreConfigStdinIsTTY(), rtCoreConfigPrompt); err != nil {
		finish("refused", rtCoreConfigExitRefused, err.Error())
		return err
	}
	p.Step("accept", fmt.Sprintf("configuration %s accepted (%s)", plan.New, plan.Label))

	// One cell lock for the notice, apply and clear sessions: taken per
	// session, another run could take the cell between them.
	if ctx, err := loadCellDeployContext(); err != nil {
		p.Step("lock", fmt.Sprintf("no cell_deploy block (%v): the lock is taken per relay session", err))
	} else {
		w := cellLockWait{Limit: opts.Maintenance.IdleWait, Quiet: cellLockDefaultQuiet, Interval: cellLockDefaultInterval}
		estimate := releaseJournalHistory(ctx.CellID).stepEstimate("config", releaseGatePlan{})
		p.Step("eta", fmt.Sprintf("config %s: %s", formatDeployDuration(estimate.Expected.Seconds()), estimate.Basis))
		unlock, err := cellLockForRun(p, ctx, opts.Transport, "config", fmt.Sprintf("configuration %.12s (%s)", plan.New, plan.Label),
			firstNonZeroDuration(opts.Maintenance.ETA, releaseETAWithMargin(estimate.Expected)), w)
		if err != nil {
			finish("refused", rtCoreConfigExitRefused, err.Error())
			return rtCoreConfigRefuse(err.Error())
		}
		defer unlock()
		_, stopProgress := noticeRunProgress(p, ctx.CellID, "config")
		defer stopProgress()
	}

	notice := newRTCoreMaintenance("config", src.CellID, opts.Maintenance, rtCoreMaintenanceConfigEstimate)
	notice.ServoRunner = "rsshs" // rt-control's copy on the servo host too
	if !waitForIdle && opts.Maintenance.ETA == 0 {
		notice.ETA = rtCoreMaintenanceConfigEstimate
	}
	if waitForIdle {
		p.Step("busy", fmt.Sprintf("controller armed, enabled or leased: the notice asks operators to Stop, disarm and release; waiting up to %s, then the servo host re-checks", opts.Maintenance.IdleWait))
	}
	p.refused = nil
	setScript := rtCoreConfigMaintenanceScript(target, notice, `maint_set "starting: configuration `+plan.New[:12]+`" || { printf '@@refused %s\n' "maintenance notice could not be written to $MAINT_FILE on $(hostname); nothing was changed"; exit 2; }
maint_wait_idle rsshs
maint_phase "applying configuration `+plan.New[:12]+`: the controller restarts next"
`)
	p.Record("maintenance notice script (relay)", setScript)
	p.Step("notice", fmt.Sprintf("setting the maintenance notice on the relay (eta %s, keep=%t)", notice.ETA, notice.Keep))
	code, err = relay("notice", setScript)
	if err != nil || code != 0 {
		detail := fmt.Sprintf("maintenance notice could not be set (exit %d, err %v): %s", code, err, strings.Join(p.refused, "; "))
		finish("refused", rtCoreConfigExitRefused, detail)
		return rtCoreConfigRefuse(detail)
	}
	defer func() {
		if notice.Keep {
			p.Step("notice", "left up (--keep-maintenance): the next step takes it over, or clear it with realtime-core maintenance clear")
			return
		}
		clear := rtCoreConfigMaintenanceScript(target, notice, "MAINT_SERVO_WRITTEN=1\nmaint_clear\n")
		if code, err := rtCoreConfigRunRemote(rtCoreConfigRemoteArgv(opts.Transport, relayUser, viaPeer, relayHost, clear), p); err != nil || code != 0 {
			p.Step("notice", fmt.Sprintf("could not confirm the notice was cleared (exit %d, %v); check: go run . realtime-core maintenance show --manifest %s", code, err, src.CellManifest))
		}
	}()
	opts.notice = notice

	p.result, p.refused, p.olp = nil, nil, nil
	applyScript := rtCoreConfigApplyCommand(c, src, probe, plan, target, runID, olpUnit, opts)
	p.Record("apply script (relay)", applyScript)
	p.Step("apply", "connecting (servo host apply session)")
	code, err = relay("apply", applyScript)
	var result map[string]string
	_ = json.Unmarshal(p.result, &result)
	summary := rtCoreConfigSummary(src, probe, plan, result, opts, target, viaPeer, relayUser, olpUnit)
	switch {
	case err == nil && code == 0:
		for _, line := range summary {
			p.Step("done", line)
		}
		finish("ok", 0, strings.Join(p.olp, "; "))
		return nil
	case err == nil && code == rtCoreConfigExitOLPFailed:
		for _, line := range summary {
			p.Step("done", line)
		}
		detail := "robot runs " + plan.New + " and is verified, but the OLP update failed and was restored: " + strings.Join(p.olp, "; ") + ". The OLP now refuses the cell on its pin check until its tree carries " + plan.New + "; the robot was not rolled back."
		finish("olp_failed", code, detail)
		return &rtCoreConfigExit{Code: code, Message: detail}
	case err == nil && code == rtCoreConfigExitRefused:
		detail := "servo host refused before any change: " + strings.Join(p.refused, "; ")
		finish("refused", code, detail)
		return rtCoreConfigRefuse(detail)
	case err == nil && code == rtCoreConfigExitRolledBack:
		detail := fmt.Sprintf("apply failed after the stop and was rolled back; the robot runs the old configuration %s, verified (%s). Backup %s", plan.Old, result["reason"], result["backup"])
		finish("rolled_back", code, detail)
		return &rtCoreConfigExit{Code: code, Message: detail}
	case err == nil && code == rtCoreConfigExitRollbackFailed:
		detail := fmt.Sprintf("ROLLBACK FAILED (%s): rosie-rt-core and rosie-rt-control are STOPPED, bus owner none. control.env backup %s. Read the servo-host output above; never re-home or restore anchors to recover", result["reason"], result["backup"])
		finish("rollback_failed", code, detail)
		return &rtCoreConfigExit{Code: code, Message: detail}
	default:
		detail := fmt.Sprintf("apply session ended unexpectedly (exit %d, err %v). The servo-host script ignores hangups and finishes or rolls back on its own; read the state with --dry-run and %s/journal.jsonl on the servo host before doing anything else", code, err, rtCoreConfigHistoryDir)
		finish("connection_lost", 1, detail)
		return errors.New(detail)
	}
}

// rtCoreConfigReport prints what the probe found and what would change.
func rtCoreConfigReport(p *rtCoreProgress, src rtCoreConfigSource, probe rtCoreConfigProbe, plan rtCoreConfigPlan) {
	s := probe.Status
	p.Step("probe", fmt.Sprintf("servo host release=%.12s package=%.12s control.env=%.12s describe=%.12s backend=%s axes=%d", probe.Release, probe.PackageManifestSHA256, probe.ControlEnv.Configuration, probe.Describe.ConfigurationSHA256, probe.Describe.Backend, probe.Describe.Axes))
	p.Step("probe", fmt.Sprintf("armed=%d enable=%d lease=%d config_verified=%d home_valid=%d safety=%d execution=%d alarms=%d", s.Armed, s.AxisEnableMask, s.LeaseValid, s.ConfigVerifiedMask, s.HomeValidMask, s.SafetyFaultMask, s.ExecutionFaultReasons, len(s.Alarms)))
	for _, name := range rtCoreConfigUnitNames {
		u := probe.Units[name]
		p.Step("probe", fmt.Sprintf("%s active=%s enabled=%s NRestarts=%s", name, u.Active, u.Enabled, u.NRestarts))
	}
	p.Step("probe", fmt.Sprintf("anchors %s layout=%q files=%d; stop dumps=%d; lock held=%t", firstNonEmpty(probe.Anchor.Dir, "(none)"), probe.Anchor.Layout, len(probe.Anchor.Files), len(probe.StopDumps), probe.Locked))
	p.Step("probe", fmt.Sprintf("olp %s pin=%.12s package=%.12s %s", firstNonEmpty(probe.OLP.Source, "(none)"), probe.OLP.CellPin, probe.OLP.PackageManifestSHA256, probe.OLP.Error))
	p.Step("compile", fmt.Sprintf("%s -> %s", probe.Compile.Where, firstNonEmpty(probe.Compile.Configuration, "error")))
	if plan.New == "" {
		return
	}
	d := plan.Diff
	p.Step("diff", fmt.Sprintf("configuration %.12s -> %.12s; machine %.12s -> %.12s; deployment %.12s -> %.12s; label=%s", plan.Old, plan.New, d.OldMachine, d.NewMachine, d.OldDeployment, d.NewDeployment, plan.Label))
	p.Step("diff", fmt.Sprintf("coordinate identity unchanged: %t; coordinate lines changed: %d; deployment drift: %d; deployment notes: %d", d.CoordinateIdentical, len(d.CoordinateAxes), len(d.DeploymentDrift), len(d.DeploymentNotes)))
	tuning := rtCoreTuningKeys(probe.Compile.Files["tuning.json"])
	for _, ch := range d.StartupChanges {
		next := rtCoreShort(ch.New)
		if ch.New == "" && tuning[ch.Key] {
			next = "no longer managed; current drive value retained (not reset to a default; RAM-only trials remain volatile)"
		}
		p.Step("diff", fmt.Sprintf("axis %d startup %s: %s -> %s", ch.Axis, ch.Key, rtCoreShort(ch.Old), next))
	}
	for _, line := range d.OtherAxesChanges {
		p.Step("diff", line)
	}
	for _, key := range d.DeploymentNotes {
		p.Step("diff", "deployment note "+key)
	}
	for _, key := range d.DeploymentDrift {
		p.Step("diff", "deployment DRIFT "+key)
	}
	for _, line := range plan.Drift.lines("") {
		p.Step("DRIFT", line)
	}
	for _, line := range src.Uncommitted {
		p.Step("uncommitted", line)
	}
	for _, pr := range plan.Promotions {
		p.Step("promote", fmt.Sprintf("axis %d (%s) %s leaves startup_trial at %d, which drive RAM already holds while EEPROM keeps the pre-trial value: a bridge configuration writes %d RAM-only first, so this run stores %d (C13.10 = 1)", pr.Axis, pr.Name, pr.Key, pr.Raw, pr.Bridge, pr.Raw))
	}
	if len(plan.Promotions) > 0 {
		p.Step("promote", fmt.Sprintf("bridge machine sha256 %.12s; one extra controller start before %.12s, disarmed under the lock", plan.BridgeSHA256, plan.New))
	}
}

// rtCoreConfigOLPUnit is the relay OLP unit the cell manifest's cell_deploy
// block names (olp.unit, the unit `olp install` writes). --olp restart needs
// it; with --olp skip a manifest without the block reads none, and the
// precompile falls back to the servo host.
func rtCoreConfigOLPUnit(manifestPath, olp string) (string, error) {
	raw, err := os.ReadFile(manifestPath)
	if err != nil {
		return "", err
	}
	deploy, err := decodeCellDeployConfig(raw)
	if err != nil {
		if olp == "skip" {
			return "", nil
		}
		return "", rtCoreConfigRefuse(fmt.Sprintf("%s: %v; --olp restart needs its cell_deploy olp.unit (or pass --olp skip)", manifestPath, err))
	}
	return deploy.OLP.Unit, nil
}

// rtCoreConfigSummary is the operator's after-action text: what runs now, which
// pins went stale, and how to get back.
func rtCoreConfigSummary(src rtCoreConfigSource, probe rtCoreConfigProbe, plan rtCoreConfigPlan, result map[string]string, opts rtCoreConfigOptions, target, viaPeer, relayUser, olpUnit string) []string {
	backup := firstNonEmpty(result["backup"], rtCoreConfigHistoryDir+"/control-env/<UTC>-"+plan.Old+".env")
	lines := []string{fmt.Sprintf("robot runs %s (%s) on release %.12s; previous configuration %s", plan.New, plan.Label, probe.Release, plan.Old)}
	if plan.Label == "trial" || src.HeadCellPin != plan.New || src.WorktreeCellPin != plan.New {
		lines = append(lines,
			fmt.Sprintf("repo cell pin is stale: %s pins %.12s in the worktree and %.12s at HEAD; realtime-core start and deploy refuse until this value is committed (machine JSON plus re-pins of the cell manifest and offline-programming/v1/internal/targets/cells_test.go) or rolled back", src.CellManifest, src.WorktreeCellPin, src.HeadCellPin),
			"the next `olp install` overwrites the OLP edit (it repoints the OLP unit at a fresh deployment tree and rebuilds olp-cells.json)",
			"Steam Deck v5 pins are stale (packaged tree and olp-cells.json): Deck Connect fails cell_configuration_mismatch until it is rebuilt from the deployed value")
	}
	if opts.OLP == "skip" {
		lines = append(lines, "OLP untouched (--olp skip): it still pins "+plan.Old+" and refuses the cell until updated")
	}
	tuning := rtCoreTuningKeys(probe.Compile.Files["tuning.json"])
	for _, ch := range plan.Diff.StartupChanges {
		if ch.New == "" && tuning[ch.Key] {
			lines = append(lines, fmt.Sprintf("axis %d %s: no longer managed; current drive value retained. Rolling back to a configuration that writes this key manages it again; RAM-only trials remain volatile.", ch.Axis, ch.Key))
		}
	}
	lines = append(lines,
		"startup applied the configured SDO settings; omitted tuning keys were not written. The A6-EC stores SDO writes in EEPROM by default (see README)",
		"rollback, preferred: put the previous machine JSON back (the OLP backup holds it as <OLP deployment>/config-history/<UTC>-"+plan.Old+"/machine.json) and run config again with --accept-configuration "+plan.Old,
		"rollback, by hand on the servo host (tailscale ssh "+remoteTarget(relayUser, viaPeer)+", then ssh -6 "+target+"):",
		"  sudo -n systemctl stop rosie-rt-control.service rosie-rt-core.service",
		"  sudo -n install -m 0640 -o root -g rosie-ctl "+backup+" /etc/rosie-rt-core/.control.env.manual",
		"  sudo -n mv -f /etc/rosie-rt-core/.control.env.manual /etc/rosie-rt-core/control.env",
		"  sudo -n systemctl start rosie-rt-core.service rosie-rt-control.service",
		"  sudo -n /opt/rosie-rt-core/current/bin/rtctl describe --socket /run/rosie-rt-core/control.sock --json   # expect "+plan.Old,
		"  then restore the OLP files from its config-history backup and sudo -n systemctl restart "+firstNonEmpty(olpUnit, "<olp unit>"),
		"rolling back does not revert drive parameters the old configuration does not write; drives keep EEPROM values")
	return lines
}

func firstNonZeroDuration(values ...time.Duration) time.Duration {
	for _, v := range values {
		if v != 0 {
			return v
		}
	}
	return 0
}
