package main

import (
	"bufio"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/exec"
	"os/signal"
	"strings"
	"sync"
	"syscall"
	"time"
)

// The cell lock. Owner rule (2026-10-07 16:50 PDT): building, testing or
// updating on a cell's hosts and using its robot are mutually exclusive. No
// build, test or update starts while the robot is in use, and nobody can arm
// while one runs.
//
// The lock is a flock on the relay (the cell's OLP host), held by one process
// per run: ~/.rosie/locks/<cell>.lock. Every maintenance wrapper takes it with
// its first notice write (rt_core_maintenance.go maint_lock), so two runs on
// one cell never overlap. `realtime-core build` and `realtime-core lock` hold
// it in a holder session that
//
//  1. takes the lock (waiting up to --idle-wait for another run's),
//  2. waits, with no notice yet, until the controller is disarmed with no
//     lease and the OLP journal shows no robot request for --quiet, printing
//     who is connected at every poll, and refuses when --idle-wait runs out,
//  3. sets the maintenance notice on both hosts (OLP 503 and rt-control busy
//     refuse arming from every client from then on), re-checks that the cell
//     is still idle, and only then lets the work start,
//  4. clears both notices when the work ends: success, failure, Ctrl-C,
//     SIGTERM or a dropped connection (the relay script's EXIT trap; stdin
//     EOF, or no client heartbeat for cellLockClientTimeout, is "the client
//     is gone").
//
// The holder's ssh carries the keepalives (sshRelayAliveArgs), so the Mac sees
// a dead path within ~60 s; the relay's sshd may not see it for hours, so the
// client also writes a heartbeat line to the holder's stdin every
// cellLockHeartbeat and the holder releases when they stop. A healthy idle
// holder answers both and stays up for as long as the run takes. A run that
// holds the lock for several sessions (cellLockForRun) checks before each
// step that its holder is still connected (cellLockLost).
//
// Relay sessions that do heavy work under the lock (build, test gate) take
// <cell>.work shared and refuse unless the lock is held. A holder that ends
// while such work still runs (the build ignores hangups) keeps the notice up
// until that work finishes. The notice records its owner process; `realtime-
// core maintenance show` flags one whose run is gone and never clears it.
//
// Builds and tests on hosts outside a cell (this Mac, CI, a bench) need no
// lock.

// cellLockEnv tells a command run under `realtime-core lock` that the lock of
// one cell is already held: "<cell> <run id>". Deploy commands of that cell
// then run inside it instead of waiting for it.
const cellLockEnv = "ROSIE_CELL_LOCK"

// Lock runs are not "realtime-core-" runs: a deploy step inside one updates
// its notice but never takes it over or clears it (as for an operator's).
const cellLockRunPrefix = "cell-lock-"

// cellLockFile is the relay's lock file as a shell word; $HOME is the relay
// user's, which every cell command runs as.
func cellLockFile(cell string) string {
	return `"$HOME/.rosie/locks/"` + shellQuote(cell+".lock")
}

// cellLockInherited returns the run id of an outer lock of this cell held for
// this process, or "".
func cellLockInherited(cell string) string {
	c, id, ok := strings.Cut(os.Getenv(cellLockEnv), " ")
	if !ok || c != cell || id == "" {
		return ""
	}
	return id
}

// cellLockWait is the idle wait before the notice goes up.
type cellLockWait struct {
	Quiet, Limit, Interval time.Duration
}

// Defaults for the idle wait. Unverified: the precheck's 2 min quiet window,
// and a limit long enough for an operator to finish a hold, Stop and disarm.
const (
	cellLockDefaultQuiet    = 2 * time.Minute
	cellLockDefaultLimit    = 10 * time.Minute
	cellLockDefaultInterval = 15 * time.Second
)

// The holder's client heartbeat. Chosen, not measured (2026-10-08,
// unverified): a line every 10 s; the relay holder treats 120 s without one
// as a client that is gone (12 missed lines; the Mac's own ssh gives up on a
// dead path after ~60 s).
var (
	cellLockHeartbeat     = 10 * time.Second
	cellLockClientTimeout = 120 * time.Second
)

func (w *cellLockWait) register(fs *flag.FlagSet) {
	fs.DurationVar(&w.Limit, "idle-wait", cellLockDefaultLimit, "wait this long for another run's cell lock, then for the cell to be idle (disarmed, no lease, no OLP robot request for --quiet); then refuse")
	fs.DurationVar(&w.Quiet, "quiet", cellLockDefaultQuiet, "the OLP journal must show no robot request for this long")
	fs.DurationVar(&w.Interval, "interval", cellLockDefaultInterval, "idle poll interval; each poll prints who is connected")
}

func (w cellLockWait) validate() error {
	if _, err := cellDeploySeconds("--idle-wait", w.Limit, 0, 2*time.Hour); err != nil {
		return err
	}
	if _, err := cellDeploySeconds("--quiet", w.Quiet, 10*time.Second, 2*time.Hour); err != nil {
		return err
	}
	_, err := cellDeploySeconds("--interval", w.Interval, 5*time.Second, 10*time.Minute)
	return err
}

// cellLockShell is the holder session's part after the maintenance shell:
// the idle checks, the command's stop and the release, then steps 1-3 of the
// lock. Its EXIT trap stops a command still running, waits for work sessions
// under the lock and clears the notice; signals cannot cut that short.
const cellLockShell = `CELL_OLP_UNIT=@UNIT@; CELL_ACTIVITY=@ACTIVITY@; OLP_LISTEN=@LISTEN@
CELL_QUIET_S=@QUIET@; CELL_LIMIT_S=@LIMIT@; CELL_EVERY_S=@EVERY@; CELL_CMD=@CMD@; CELL_CLIENT_S=@CLIENT@
CELL_WORK="${MAINT_LOCK%.lock}.work"; CELL_PID=""
cell_client_wait(){ # until the client's stdin ends, or its heartbeat lines stop for CELL_CLIENT_S; "eta N" re-estimates the notice to N s from now, "progress N <base64 JSON>" also sets its progress (notice_progress.go)
  local t l r e b
  while :; do
    t=$SECONDS; IFS= read -r -t "$CELL_CLIENT_S" l <&6 || break
    case "$l" in
      "eta "|"eta "*[!0-9]*) ;;
      "eta "*) MAINT_ETA_S=${l#eta }; MAINT_ETA_NOW=1; maint_phase ""; MAINT_ETA_NOW=0 ;;
      "progress "*)
        r=${l#progress }; e=${r%% *}; b=${r#* }
        case "$e" in ""|*[!0-9]*) continue ;; esac
        case "$b" in ""|*[!0-9A-Za-z+/=]*) continue ;; esac
        MAINT_ETA_S=$e; MAINT_ETA_NOW=1; MAINT_PROGRESS=$b; maint_phase ""; MAINT_ETA_NOW=0; MAINT_PROGRESS="" ;;
    esac
  done
  if [ $((SECONDS - t)) -ge "$CELL_CLIENT_S" ]; then maint_say "no heartbeat from the client for ${CELL_CLIENT_S}s: its connection is dead; releasing the lock"; fi
}
cell_idle_once(){ maint_servo <<'ROSIE_CELL_IDLE'
@IDLE@
ROSIE_CELL_IDLE
}
cell_other_run(){ # the run id of another deploy or lock run whose notice is up (between its sessions it holds no lock)
  python3 -c 'import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    sys.exit()
r = str(d.get("run_id") or "") if isinstance(d, dict) and d.get("active", True) is not False else ""
if r and r != sys.argv[2] and r.startswith(("realtime-core-", "cell-lock-")):
    print("%s (%s)" % (r, d.get("reason") or "no reason"))' "$MAINT_FILE" "$MAINT_RUN" 2> /dev/null
}
cell_wait_idle(){ # before the notice: read-only on both hosts
  local start=$SECONDS line act other
  maint_say "waiting up to ${CELL_LIMIT_S}s for the cell to be idle: disarmed, no lease, no OLP robot request for ${CELL_QUIET_S}s (no notice yet)"
  while :; do
    line=$(cell_idle_once 2> /dev/null | head -n 1); [ -n "$line" ] || line="unknown servo host unreachable"
    other=$(cell_other_run)
    [ -z "$other" ] || line="busy another run's maintenance notice is up: $other (if that run is gone: realtime-core maintenance show, then clear)"
    act=$( { sudo -n journalctl -u "$CELL_OLP_UNIT" --since "-$CELL_QUIET_S s" --no-pager -o cat 2> /dev/null || echo "arm= (OLP journal unreadable)"; } | grep -cE "$CELL_ACTIVITY" || true)
    case "$line" in idle*) if [ "${act:-1}" = 0 ]; then maint_say "cell idle (${line#idle }); no OLP robot request in ${CELL_QUIET_S}s"; return 0; fi ;; esac
    maint_say "cell in use: ${line} olp_robot_requests_last_${CELL_QUIET_S}s=${act:-?}; waited $((SECONDS - start))s of ${CELL_LIMIT_S}s"
    maint_who
    [ $((SECONDS - start)) -lt "$CELL_LIMIT_S" ] || return 3
    sleep "$CELL_EVERY_S"
  done
}
cell_stop(){ [ -n "$CELL_PID" ] || return 0; maint_say "$1: stopping the command (TERM to its process group)"; kill -TERM -- "-$CELL_PID" 2> /dev/null || kill -TERM "$CELL_PID" 2> /dev/null; }
cell_release(){
  if [ -n "$CELL_PID" ] && kill -0 "$CELL_PID" 2> /dev/null; then cell_stop "the lock is ending"; wait "$CELL_PID" 2> /dev/null; fi
  [ "${MAINT_LOCKED:-0}" = 1 ] || return 0
  exec 5>> "$CELL_WORK" || return 0
  if ! flock -n 5; then
    maint_phase "waiting for relay work started under this lock (a build or test gate) to finish"
    maint_say "relay work started under this lock is still running; the notice stays up until it ends"
    flock 5
  fi
}
trap 'rc=$?; trap - EXIT; trap "" HUP INT TERM PIPE USR1; set +e; cell_release; maint_clear; exit $rc' EXIT
trap 'exit 129' HUP; trap 'exit 130' INT; trap 'exit 143' TERM; trap '' PIPE
exec 6<&0
maint_lock || { printf '@@refused %s\n' "the cell is locked by $(maint_holder); nothing was started"; exit 2; }
cell_wait_idle || { printf '@@refused %s\n' "the cell stayed in use for ${CELL_LIMIT_S}s (armed, leased or OLP robot requests); nothing was started"; exit 3; }
maint_set @PHASE@ || { printf '@@refused %s\n' "the maintenance notice could not be written on $(hostname) or the servo host; nothing was started"; exit 2; }
line=$(cell_idle_once 2> /dev/null | head -n 1)
case "$line" in idle*) ;; *) printf '@@refused %s\n' "the cell became busy before the notice took effect (${line:-servo host unreachable}); nothing was started"; exit 3 ;; esac
printf '@@locked %s\n' "$MAINT_RUN"
`

// cellLockHolderScript renders the holder session. on: "local" holds until
// stdin closes (the client runs the work, or a build drives its own relay
// sessions); "relay" runs command on the relay and "servo" on the servo host
// (from the relay), each in its own process group, stopped with TERM when
// stdin closes (client gone or interrupted).
func cellLockHolderScript(ctx cellDeployContext, m rtCoreMaintenance, w cellLockWait, phase, on, command string) string {
	m.ServoRunner = "maint_servo"
	m.IdleWait = 0
	m.LockInherit = ""
	body := `maint_say "holding the cell lock for the client; arming stays refused until it releases"
cell_client_wait
`
	// The USR1 trap is set before the stdin watcher exists: an untrapped
	// USR1 would end this shell without its EXIT trap, leaving the notices.
	start := `set +e
trap 'cell_stop "the client went away or was interrupted"' USR1
`
	switch on {
	case "relay":
		body = start + `maint_say "running on $(hostname): $CELL_CMD"
setsid bash -c "$CELL_CMD" < /dev/null & CELL_PID=$!
`
	case "servo":
		body = start + `maint_say "running on the servo host $MAINT_SERVO (nothing heavy there): $CELL_CMD"
setsid ssh -o BatchMode=yes ` + sshAliveShell(sshConnectTimeout) + ` -- "$MAINT_SERVO" "$CELL_CMD" < /dev/null & CELL_PID=$!
`
	}
	if on != "local" {
		body += `( cell_client_wait; kill -USR1 $$ 2> /dev/null ) > /dev/null 2>&1 & CELL_WATCH=$!
rc=0
while kill -0 "$CELL_PID" 2> /dev/null; do wait "$CELL_PID"; rc=$?; done
CELL_PID=""
kill "$CELL_WATCH" 2> /dev/null
maint_say "command exit $rc"
exit "$rc"
`
	}
	return "set -eu\n" + rtCoreMaintenanceShell(m) +
		"MAINT_SERVO=" + shellQuote(ctx.ServoTarget) + "\n" +
		rtCoreMaintenanceServoFunc +
		rtCoreConfigReplace(cellLockShell, "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit), "@ACTIVITY@", shellQuote(cellDeployOLPActivity),
			"@LISTEN@", shellQuote(ctx.Deploy.OLP.Listen), "@QUIET@", fmt.Sprint(int(w.Quiet.Seconds())), "@LIMIT@", fmt.Sprint(int(w.Limit.Seconds())),
			"@EVERY@", fmt.Sprint(int(w.Interval.Seconds())), "@CMD@", shellQuote(command), "@PHASE@", shellQuote(phase),
			"@CLIENT@", fmt.Sprint(int(cellLockClientTimeout.Seconds())),
			"@IDLE@", strings.ReplaceAll(rtCoreMaintenanceIdleScript, "@IDLE_S@", "0")) + body
}

// cellLockWorkShell renders a relay work session's priority function (prio)
// and, under the lock, its check: take <cell>.work shared (a holder that is
// releasing holds it exclusively) and refuse unless the lock is held. Under
// the lock the work runs at normal priority: nobody uses the robot. Without
// it (--low-priority) it runs at low CPU and I/O priority and checks nothing.
func cellLockWorkShell(cell string, locked bool) (prio, check string) {
	if !locked {
		return `prio(){ nice -n 15 ionice -c2 -n7 "$@"; }`, ""
	}
	return `prio(){ "$@"; } # under the cell lock: normal priority`,
		`CELL_LOCK=` + cellLockFile(cell) + `
{ mkdir -p "$(dirname "$CELL_LOCK")" && exec 8>> "${CELL_LOCK%.lock}.work" && flock -n -s 8; } || refuse "the cell lock is being released; nothing was started"
if flock -n "$CELL_LOCK" true; then refuse "the cell lock is not held: run realtime-core build without --low-priority, or under realtime-core lock; nothing was started"; fi
`
}

// cellLockSession is one holder session on the relay.
type cellLockSession struct {
	process *os.Process
	stdin   io.WriteCloser
	locked  chan struct{}
	done    chan struct{}
	exit    int
	err     error
	mu      sync.Mutex // stdin: heartbeat writes and the close
	closed  bool
}

// closeStdin tells the holder the client is done (stdin EOF).
func (s *cellLockSession) closeStdin() {
	s.mu.Lock()
	defer s.mu.Unlock()
	if !s.closed {
		s.closed = true
		s.stdin.Close()
	}
}

// sendETA asks the holder to rewrite its notice's ETA to d from now, keeping
// the notice's start and phase: an "eta <seconds>" line on the stdin the
// heartbeat uses. False once the client closed it.
func (s *cellLockSession) sendETA(d time.Duration) bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed {
		return false
	}
	_, err := fmt.Fprintf(s.stdin, "eta %d\n", int(d.Round(time.Second)/time.Second))
	return err == nil
}

// sendProgress is sendETA with the run's progress (notice_progress.go).
func (s *cellLockSession) sendProgress(eta time.Duration, p noticeProgress) bool {
	line, err := noticeProgressLine(eta, p)
	if err != nil {
		return false
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.closed {
		return false
	}
	_, err = io.WriteString(s.stdin, line)
	return err == nil
}

// heartbeat writes one line to the holder's stdin every cellLockHeartbeat
// until the session ends or the client closes it.
func (s *cellLockSession) heartbeat() {
	tick := time.NewTicker(cellLockHeartbeat)
	defer tick.Stop()
	for {
		select {
		case <-s.done:
			return
		case <-tick.C:
		}
		s.mu.Lock()
		if s.closed {
			s.mu.Unlock()
			return
		}
		_, _ = io.WriteString(s.stdin, "\n")
		s.mu.Unlock()
	}
}

// cellLockArgv reaches the relay; tests run the holder under a local bash.
var cellLockArgv = func(transport string, ctx cellDeployContext, script string) []string {
	return rtCoreConfigRemoteArgv(transport, ctx.RelayUser, ctx.RelayPeer, ctx.RelayHost, script)
}

// cellLockStart starts a holder session. Its ssh runs in its own process
// group, so a Ctrl-C reaches this process (which then releases the lock in
// order) and not the session itself.
var cellLockStart = func(argv []string, p *rtCoreProgress) (*cellLockSession, error) {
	cmd := exec.Command(argv[0], argv[1:]...)
	cellLockDetach(cmd)
	stdin, err := cmd.StdinPipe()
	if err != nil {
		return nil, err
	}
	stdout, err := cmd.StdoutPipe()
	if err != nil {
		return nil, err
	}
	stderr, err := cmd.StderrPipe()
	if err != nil {
		return nil, err
	}
	if err := cmd.Start(); err != nil {
		return nil, err
	}
	s := &cellLockSession{process: cmd.Process, stdin: stdin, locked: make(chan struct{}), done: make(chan struct{})}
	var once sync.Once
	var wg sync.WaitGroup
	wg.Add(2)
	go func() {
		defer wg.Done()
		scanner := bufio.NewScanner(stdout)
		scanner.Buffer(make([]byte, 0, 64*1024), 16*1024*1024)
		for scanner.Scan() {
			line := scanner.Text()
			if run, ok := strings.CutPrefix(line, "@@locked "); ok {
				once.Do(func() { close(s.locked) })
				p.Step("lock", "held by run "+run+": both maintenance notices are up; arming is refused until this run ends")
				continue
			}
			p.Remote(line)
		}
	}()
	go func() {
		defer wg.Done()
		scanner := bufio.NewScanner(stderr)
		for scanner.Scan() {
			p.Step("stderr", scanner.Text())
		}
	}()
	go func() {
		wg.Wait()
		err := cmd.Wait()
		var exit *exec.ExitError
		switch {
		case errors.As(err, &exit):
			s.exit = exit.ExitCode()
		case err != nil:
			s.exit, s.err = -1, err
		}
		close(s.done)
	}()
	go s.heartbeat()
	return s, nil
}

// await reports whether the session holds the lock; false once it ended
// without taking it.
func (s *cellLockSession) await() bool {
	select {
	case <-s.locked:
		return true
	case <-s.done:
		select {
		case <-s.locked:
			return true
		default:
			return false
		}
	}
}

// release tells the holder the client is done (stdin EOF) and waits up to
// timeout for it to clear the notices; false when it is still waiting for
// relay work under the lock.
func (s *cellLockSession) release(timeout time.Duration) bool {
	s.closeStdin()
	select {
	case <-s.done:
		return true
	case <-time.After(timeout):
		return false
	}
}

// cellLockHold starts a holder session and waits until it holds the lock.
func cellLockHold(p *rtCoreProgress, argv []string) (*cellLockSession, error) {
	s, err := cellLockStart(argv, p)
	if err != nil {
		return nil, err
	}
	if s.await() {
		return s, nil
	}
	p.mu.Lock()
	refused := strings.Join(p.refused, "; ")
	p.mu.Unlock()
	if s.err != nil {
		return nil, cellDeployExit(cellDeployExitFailed, "cell lock session failed: %v", s.err)
	}
	return nil, cellDeployExit(cellDeployExitRefused, "cell lock not taken (exit %d): %s", s.exit, firstNonEmpty(refused, "see the lines above"))
}

// cellLockOnSignal calls handle for SIGINT and SIGTERM until stopped.
func cellLockOnSignal(handle func(os.Signal)) func() {
	ch := make(chan os.Signal, 2)
	signal.Notify(ch, os.Interrupt, syscall.SIGTERM)
	stop := make(chan struct{})
	go func() {
		for {
			select {
			case sig := <-ch:
				handle(sig)
			case <-stop:
				return
			}
		}
	}()
	return func() {
		signal.Stop(ch)
		close(stop)
	}
}

// How long a released holder may take to clear before the client stops
// waiting for it (it then still clears, after relay work under the lock).
const cellLockReleaseWait = 2 * time.Minute

func cellLockReleaseOrReport(p *rtCoreProgress, s *cellLockSession) {
	if !s.release(cellLockReleaseWait) {
		p.Step("lock", "the holder is still waiting for relay work under the lock; it clears both notices when that ends (check: realtime-core maintenance show)")
	}
}

// A build's and a build-and-gate's estimates when the cell has no build or
// gate in this machine's deploy journal (release_eta.go uses them too).
// Unverified: the one measured build (9a48c367, a Rosie 1430 cell's relay, nice 15,
// planner env reused, no --test) took 186 s on 2026-10-07; the gate had not
// been timed then (since: 13-16.5 min, 2026-10-08/-11).
const (
	cellDeployBuildDefaultETA     = 10 * time.Minute
	cellDeployBuildTestDefaultETA = 60 * time.Minute
)

// cellDeployBuildEstimate is a standalone build's step from this machine's
// journal (release_eta.go): the cell's median build, with --test plus the
// gate of every suite of the level (the build narrows it to the suites it
// chooses once it has; notice_progress.go).
func cellDeployBuildEstimate(h releaseHistory, test bool, level string) releaseStepETA {
	b, why := h.buildEstimate()
	if !test {
		return releaseStepETA{Name: "build", Expected: b, Basis: "build " + formatDeployDuration(b.Seconds()) + " (" + why + ")"}
	}
	all, _ := gateSelect("full", nil, true, h.Means)
	plan := releaseGatePlan{Level: level}
	for _, s := range all.Suites {
		plan.Suites = append(plan.Suites, s.Name)
	}
	return h.stepEstimate("build+test", plan)
}

// cellDeployBuildLock is `realtime-core build`'s cell lock: a holder session
// from before the probe to after the test gate. Script is empty when the
// build runs at --low-priority (no lock) or inside an outer lock.
type cellDeployBuildLock struct {
	Script, Inherited, ETAWhy string
	Low                       bool
	ETA                       time.Duration
	// Step and History seed the notice's progress (notice_progress.go).
	Step        releaseStepETA
	History     releaseHistory
	Level       string
	session     *cellLockSession
	stopSignals func()
	held        time.Time
}

func newCellDeployBuildLock(ctx cellDeployContext, commit string, test bool, level string, low bool, eta time.Duration, w cellLockWait) cellDeployBuildLock {
	l := cellDeployBuildLock{Low: low, Inherited: cellLockInherited(ctx.CellID), ETA: eta, ETAWhy: "--maintenance-eta", Level: level}
	if low || l.Inherited != "" {
		return l
	}
	l.History = releaseJournalHistory(ctx.CellID)
	l.Step = cellDeployBuildEstimate(l.History, test, level)
	if l.ETA == 0 {
		l.ETA, l.ETAWhy = releaseETAWithMargin(l.Step.Expected), l.Step.Basis+" + 20% (at least 1 min), from this Mac's journal"
	}
	what := "build"
	if test {
		what = "build and test gate"
	}
	m := cellDeployNotice(ctx, "build", what+" "+cellDeployShort(commit)+" in progress", rtCoreMaintenanceFlags{ETA: l.ETA}, 0)
	m.LockWait = w.Limit
	l.Script = cellLockHolderScript(ctx, m, w, what+" of "+cellDeployShort(commit)+" running on the OLP host; the robot cannot be armed until it ends", "local", "")
	return l
}

// acquire takes the lock before any relay work. A SIGINT or SIGTERM then
// releases it and exits 130: relay work already started finishes first (the
// build ignores hangups) and the notice stays up until it ends.
func (l *cellDeployBuildLock) acquire(run *cellDeployRun, transport string) error {
	switch {
	case l.Low:
		run.p.Step("lock", "--low-priority: no cell lock and no maintenance notice; nice 15 / ionice best-effort 7. Only for a relay that serves no robot in use (CLAUDE.md)")
		return nil
	case l.Inherited != "":
		run.p.Step("lock", "running under the cell lock of "+l.Inherited+" (normal priority)")
		return nil
	}
	run.p.Step("lock", fmt.Sprintf("taking the cell lock; notice ETA %s (%s)", l.ETA, l.ETAWhy))
	run.p.Record("cell lock (relay holder)", l.Script)
	s, err := cellLockHold(run.p, cellLockArgv(transport, run.ctx, l.Script))
	if err != nil {
		return err
	}
	l.session, l.held = s, time.Now()
	l.stopSignals = cellLockOnSignal(func(sig os.Signal) {
		run.p.Step("interrupt", fmt.Sprintf("%v: releasing the cell lock; relay work already started finishes first and the notice stays up until it ends (realtime-core maintenance show)", sig))
		s.release(10 * time.Second)
		os.Exit(130)
	})
	return nil
}

// progress is the notice progress for the build: a release's when the build
// runs inside one, else one step of its own over this lock's holder; nil at
// --low-priority (no notice).
func (l *cellDeployBuildLock) progress(run *cellDeployRun) *noticeTracker {
	if t := noticeTrackerActive(); t != nil && l.Inherited != "" {
		return t
	}
	if l.session == nil {
		return nil
	}
	t := newNoticeTracker(l.History, releaseETA{Steps: []releaseStepETA{l.Step}}, l.Level, l.session.sendProgress,
		func(line string) { run.p.Step("progress", line) })
	t.Begin(0)
	return t
}

func (l *cellDeployBuildLock) release(run *cellDeployRun) {
	if l.session == nil {
		return
	}
	l.stopSignals()
	cellLockReleaseOrReport(run.p, l.session)
}

const cellLockUsage = `usage: realtime-core lock --manifest cell.json --reason TEXT [--eta 30m]
          [--on local|relay|servo] [--idle-wait 10m] [--quiet 2m] [--interval 15s]
          [--transport tailscale|ssh] [--dry-run] -- COMMAND...

Runs COMMAND under the cell lock: waits for another run's lock and for the
cell to be idle (printing who is connected), sets the maintenance notice on
the OLP host and the servo host (arming refused from every client), runs
COMMAND, and clears both notices when it ends, however it ends.

  --on local   COMMAND runs on this machine (default); ROSIE_CELL_LOCK tells
               deploy commands of this cell inside it to run under this lock
  --on relay   COMMAND runs on the cell's OLP host as bash -c COMMAND, in its
               own process group; Ctrl-C or a dropped connection stops it (TERM)
               before the notices are cleared
  --on servo   COMMAND runs on the servo host over ssh from the relay. Nothing
               heavy runs there; a dropped connection may leave it running

Use it for any build, test or heavy job on a cell's hosts that is not a
realtime-core/olp command (those take the lock themselves). Builds and tests
on hosts outside the cell need no lock.
`

// runCellLock is `realtime-core lock`.
func runCellLock(args []string, stdout, stderr io.Writer) error {
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, cellLockUsage)
		return nil
	}
	flags, command := args, []string(nil)
	for i, a := range args {
		if a == "--" {
			flags, command = args[:i], args[i+1:]
			break
		}
	}
	var common cellDeployCommonFlags
	var reason, on string
	var eta time.Duration
	var w cellLockWait
	ctx, restore, err := cellDeploySetup("realtime-core lock", flags, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&reason, "reason", "", "what runs, shown to operators on the OLP and Steam Deck overlays (required)")
		fs.DurationVar(&eta, "eta", 30*time.Minute, "expected duration shown on the overlays, from when the notice is set")
		fs.StringVar(&on, "on", "local", "where COMMAND runs: local, relay or servo")
		w.register(fs)
	}, &common, true, "--reason", "--eta", "--on", "--idle-wait", "--quiet", "--interval")
	if err != nil {
		return err
	}
	defer restore()
	switch {
	case strings.TrimSpace(reason) == "":
		return fmt.Errorf("realtime-core lock requires --reason")
	case eta < time.Minute || eta > 12*time.Hour:
		return fmt.Errorf("--eta must be between 1m and 12h")
	case on != "local" && on != "relay" && on != "servo":
		return fmt.Errorf("--on must be local, relay or servo")
	case len(command) == 0:
		return fmt.Errorf("realtime-core lock needs a command after --")
	}
	if err := w.validate(); err != nil {
		return err
	}
	text := command[0]
	if len(command) > 1 {
		quoted := make([]string, len(command))
		for i, c := range command {
			quoted[i] = shellQuote(c)
		}
		text = strings.Join(quoted, " ")
	}
	if outer := cellLockInherited(ctx.CellID); outer != "" {
		if on != "local" {
			return fmt.Errorf("already under the cell lock of %s (%s): run the command directly, not a nested lock --on %s", outer, cellLockEnv, on)
		}
		fmt.Fprintf(stdout, "realtime-core lock: already under the cell lock of %s; running the command directly\n", outer)
		return cellLockRunLocal(command, stdout, stderr, os.Getenv(cellLockEnv))
	}
	m := cellDeployNotice(ctx, "lock", reason, rtCoreMaintenanceFlags{ETA: eta}, 0)
	m.RunID = cellLockRunPrefix + time.Now().UTC().Format("20060102T150405Z") + "-" + ctx.CellID
	m.SetBy = "realtime-core lock (" + rtCoreMaintenanceOperator() + ")"
	m.LockWait = w.Limit
	where := map[string]string{"local": "on " + rtCoreMaintenanceOperator(), "relay": "on the OLP host", "servo": "on the servo host"}[on]
	short := text
	if len(short) > 160 {
		short = short[:157] + "..."
	}
	script := cellLockHolderScript(ctx, m, w, "running "+where+": "+short, on, text)
	if common.DryRun {
		fmt.Fprintf(stdout, "realtime-core lock cell=%s on=%s run=%s (dry run: nothing contacted)\n# command\n%s\n# relay holder session\n%s\n", ctx.CellID, on, m.RunID, text, script)
		return nil
	}
	run, err := newCellDeployRun(ctx, "realtime-core lock", common.Transport, stdout)
	if err != nil {
		return err
	}
	run.p.Record("command", text)
	session, err := cellLockHold(run.p, cellLockArgv(common.Transport, ctx, script))
	if err != nil {
		run.finish("refused", cellDeployExitRefused, err.Error())
		return err
	}
	code := 0
	if on == "local" {
		code = cellLockExitCode(cellLockRunLocal(command, stdout, stderr, ctx.CellID+" "+m.RunID))
		run.p.Step("lock", fmt.Sprintf("command exit %d; releasing the cell lock", code))
		cellLockReleaseOrReport(run.p, session)
	} else {
		stop := cellLockOnSignal(func(sig os.Signal) {
			run.p.Step("interrupt", fmt.Sprintf("%v: stopping the command on the %s host, then clearing the notices", sig, on))
			session.closeStdin()
		})
		<-session.done
		stop()
		code = session.exit
	}
	run.finish(map[bool]string{true: "ok", false: "failed"}[code == 0], code, text)
	if code != 0 {
		return cellDeployExit(code, "realtime-core lock: the command exited %d (the notices are cleared)", code)
	}
	return nil
}

// cellLockRunLocal runs command here with the lock named in its environment,
// forwarding SIGINT and SIGTERM to it.
func cellLockRunLocal(command []string, stdout, stderr io.Writer, lock string) error {
	child := exec.Command(command[0], command[1:]...)
	child.Stdin, child.Stdout, child.Stderr = os.Stdin, stdout, stderr
	child.Env = append(os.Environ(), cellLockEnv+"="+lock)
	if err := child.Start(); err != nil {
		return err
	}
	stop := cellLockOnSignal(func(sig os.Signal) { _ = child.Process.Signal(sig) })
	defer stop()
	return child.Wait()
}

func cellLockExitCode(err error) int {
	var exit *exec.ExitError
	switch {
	case err == nil:
		return 0
	case errors.As(err, &exit):
		if code := exit.ExitCode(); code >= 0 {
			return code
		}
		return 130
	default:
		return 127
	}
}

// cellLockRunSessions are the holder sessions cellLockForRun holds in this
// process, by their cellLockEnv value.
var cellLockRunSessions sync.Map

// cellLockLost reports a run lock of this process whose holder session has
// ended: its connection died (keepalives or the relay's heartbeat timeout)
// or the relay ended it. The relay clears the notices once it sees the client
// gone, so nothing more may start under that lock. Nil when the process holds
// no run lock or its holder is connected.
func cellLockLost() error {
	held := os.Getenv(cellLockEnv)
	v, ok := cellLockRunSessions.Load(held)
	if !ok {
		return nil
	}
	s := v.(*cellLockSession)
	select {
	case <-s.done:
		return fmt.Errorf("the cell lock's holder session (run %s) ended with exit %d %v: the lock and the notices are no longer this run's", held, s.exit, s.err)
	default:
		return nil
	}
}

// cellLockNoticeETA re-estimates the notice of the run lock this process
// holds (cellLockForRun) to d from now. False when this process holds none
// (under an outer lock, whose run owns its ETA) or its holder is gone.
func cellLockNoticeETA(d time.Duration) bool {
	v, ok := cellLockRunSessions.Load(os.Getenv(cellLockEnv))
	if !ok {
		return false
	}
	s := v.(*cellLockSession)
	select {
	case <-s.done:
		return false
	default:
	}
	return s.sendETA(d)
}

// cellLockForRun holds the cell lock for the whole of a command that opens
// several relay sessions (release, config) and names it in this process's
// environment, so every session and in-process step runs inside it
// (cellLockInherited): their notices update the lock's notice and never
// clear it, and no other run can take the cell between two sessions. A
// command already under an outer lock of the cell keeps that one.
func cellLockForRun(p *rtCoreProgress, ctx cellDeployContext, transport, command, reason string, eta time.Duration, w cellLockWait) (func(), error) {
	if outer := cellLockInherited(ctx.CellID); outer != "" {
		p.Step("lock", "running under the cell lock of "+outer)
		return func() {}, nil
	}
	m := cellDeployNotice(ctx, "lock", reason, rtCoreMaintenanceFlags{ETA: eta}, 0)
	m.RunID = cellLockRunPrefix + time.Now().UTC().Format("20060102T150405Z") + "-" + command + "-" + ctx.CellID
	m.SetBy = "realtime-core " + command + " (" + rtCoreMaintenanceOperator() + ")"
	m.LockWait = w.Limit
	script := cellLockHolderScript(ctx, m, w, reason+" in progress; the robot cannot be armed until it ends", "local", "")
	p.Record("cell lock (relay holder)", script)
	p.timer.begin("lock", p.now())
	session, err := cellLockHold(p, cellLockArgv(transport, ctx, script))
	p.timer.end("lock", deployTimingResult(0, err), p.now())
	if err != nil {
		return nil, err
	}
	previous, had := os.LookupEnv(cellLockEnv)
	held := ctx.CellID + " " + m.RunID
	os.Setenv(cellLockEnv, held)
	cellLockRunSessions.Store(held, session)
	stop := cellLockOnSignal(func(sig os.Signal) {
		p.Step("interrupt", fmt.Sprintf("%v: releasing the cell lock; relay work already started finishes first and the notice stays up until it ends", sig))
		session.release(10 * time.Second)
		os.Exit(130)
	})
	return func() {
		stop()
		cellLockRunSessions.Delete(held)
		if had {
			os.Setenv(cellLockEnv, previous)
		} else {
			os.Unsetenv(cellLockEnv)
		}
		p.Step("lock", "releasing the cell lock (both notices clear)")
		cellLockReleaseOrReport(p, session)
	}, nil
}
