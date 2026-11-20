package main

import (
	"errors"
	"fmt"
	"strconv"
	"strings"
	"time"
)

// Watched relay sessions: the long ones that stream progress (the build and
// the test gate). The relay runs the work detached from the connection, in
// its own process group, with its output in ~/.rosie/watched/<run>-<label>/out
// and its exit status written to exit when it ends; the session only follows
// that file. So a connection that dies (ssh exit 255 after the keepalives,
// sshAliveArgs) or goes silent leaves the work running, and a fresh
// connection can tell a dead stream from a slow suite: it re-attaches from
// the last byte the Mac received and either streams the rest and the exit
// status or reports that the work is gone.
//
// On 2026-10-08 the fast gate of an OLP-tier release of ae985a83 finished on
// the relay at ~13:02 PDT while the Mac's ssh, half-open, never saw it end;
// the release held the cell lock until 13:49, when the coordinator sent
// SIGTERM to that ssh, and another release waited on the lock meanwhile.
//
// Like the release's one retry of a stalled start, a stalled stream is
// re-attached once; a re-attach that delivers nothing new fails the step. A
// session still running past its limit is stopped on the relay (TERM to its
// process group, KILL after remoteWatchStopWaitS) and fails: it holds the
// work lock, which keeps the cell lock and the notice up.

// The thresholds. Chosen, not measured (2026-10-08, unverified): the relay
// prints a heartbeat every remoteWatchHeartbeatS while the work prints
// nothing, so remoteWatchSilence (four missed heartbeats) of no line at all
// means the stream is dead, not a quiet suite.
var (
	remoteWatchSilence    = 2 * time.Minute
	remoteWatchHeartbeatS = 30
	remoteWatchTick       = time.Second
	remoteWatchStopWaitS  = 30
	// remoteWatchBuildLimit bounds one build session: measured builds took
	// 98-186 s (2026-10-07, one relay); a planner env install (pixi) has not
	// been timed, hence the margin. Unverified.
	remoteWatchBuildLimit = 90 * time.Minute
)

const remoteWatchReattachMax = 5

// gateWatchLimit bounds one test gate session: twice the selected suites run
// one at a time on the recorded (journal) or catalogue seconds, plus 15 min,
// and at least 30 min. The full gate's catalogue sum is 1770 s (74 min
// limit); the four bc423a86 full gates took 11.4-11.7 min on lanes.
// Unverified.
func gateWatchLimit(sel gateSelection) time.Duration {
	serial := 0.0
	for _, s := range sel.Suites {
		serial += s.Seconds
	}
	return max(30*time.Minute, time.Duration(2*serial)*time.Second+15*time.Minute)
}

// remoteWatch is one watched session's state, under rtCoreProgress.mu.
type remoteWatch struct {
	Silence  time.Duration
	Deadline time.Time // zero: none
	activity time.Time
	// delivered counts the bytes of complete stdout lines received: where a
	// re-attach resumes.
	delivered int64
	last      string
	exit      *int
	lost      string
}

// remoteStallError is the runner's report that it ended a watched session
// itself: no line within Silence, or past Deadline (Overrun).
type remoteStallError struct {
	Reason  string
	Overrun bool
}

func (e *remoteStallError) Error() string { return "stalled: " + e.Reason }

// expired returns why a watched session must end now, or "".
func (w *remoteWatch) expired(now time.Time) (string, bool) {
	if !w.Deadline.IsZero() && now.After(w.Deadline) {
		return fmt.Sprintf("still running past its limit (deadline %s)", w.Deadline.Format("15:04:05")), true
	}
	if quiet := now.Sub(w.activity); quiet > w.Silence {
		return fmt.Sprintf("no output and no relay heartbeat for %s (heartbeat every %ds)", quiet.Round(time.Second), remoteWatchHeartbeatS), false
	}
	return "", false
}

// watchLine handles a "@@watch" line from the relay's follower (stderr).
func (p *rtCoreProgress) watchLine(rest string) {
	kind, detail, _ := strings.Cut(rest, " ")
	switch kind {
	case "alive":
		return
	case "exit":
		if code, err := strconv.Atoi(strings.TrimSpace(detail)); err == nil {
			p.mu.Lock()
			if p.watch != nil {
				p.watch.exit = &code
			}
			p.mu.Unlock()
		}
		return
	case "lost":
		p.mu.Lock()
		if p.watch != nil {
			p.watch.lost = detail
		}
		p.mu.Unlock()
	}
	p.Step("watch", kind+" "+detail)
}

// watchTouch marks activity on the watched stream; line is a work line
// (kept as the last one) or "" for follower lines.
func (p *rtCoreProgress) watchTouch(line string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	if p.watch == nil {
		return
	}
	p.watch.activity = p.now()
	if line != "" {
		p.watch.last = line
	}
}

// remoteWatchID names a watched session's directory on the relay.
func remoteWatchID(run, label string) string {
	return run + "-" + strings.Join(strings.Fields(label), "-")
}

// remoteWatchDirShell is the session directory as a shell word.
func remoteWatchDirShell(id string) string {
	return `"$HOME/.rosie/watched/"` + shellQuote(id)
}

// remoteWatchFollowShell defines watch_follow OFFSET: stream $W/out from
// byte OFFSET, then exit with the work's status. Its own lines go to stderr
// (@@watch alive|exit|lost) so stdout carries exactly the work's bytes. A
// dead connection ends it on the next write (SIGPIPE); the work, in another
// process group and writing to a file, keeps running.
func remoteWatchFollowShell() string {
	return strings.ReplaceAll(`watch_follow(){
  local off=$1 size quiet=0 done pid rc
  pid=$(cat "$W/pid" 2> /dev/null)
  while :; do
    done=0; [ -e "$W/exit" ] && done=1
    size=$(wc -c < "$W/out" | tr -d ' ')
    if [ "$size" -gt "$off" ]; then
      tail -c +$((off + 1)) "$W/out" | head -c $((size - off)) || exit 141
      off=$size; quiet=0
    fi
    if [ "$done" = 1 ]; then
      if [ "$size" -gt 0 ] && [ -n "$(tail -c 1 "$W/out")" ]; then printf '\n'; fi
      rc=$(cat "$W/exit"); printf '@@watch exit %s\n' "$rc" >&2; exit "$rc"
    fi
    if ! kill -0 "$pid" 2> /dev/null && [ ! -e "$W/exit" ]; then
      printf '@@watch lost %s\n' "the relay work (pid $pid) ended without recording an exit status; its output is in $W/out" >&2; exit 255
    fi
    quiet=$((quiet + 1))
    if [ "$quiet" -ge @HB@ ]; then printf '@@watch alive %s\n' "$pid" >&2 || exit 141; quiet=0; fi
    sleep 1
  done
}
`, "@HB@", strconv.Itoa(remoteWatchHeartbeatS))
}

// remoteWatchStartScript starts script detached in its watched directory
// and follows it. The work runs under setsid (its own session and process
// group, the group the stop signals) with no descriptor of the connection:
// a work that inherited the session's stdout kept the client reading until
// the work itself ended. Directories older than 7 days are removed.
func remoteWatchStartScript(id, script string) string {
	if !strings.HasSuffix(script, "\n") {
		script += "\n"
	}
	return "set -u\nW=" + remoteWatchDirShell(id) + `
mkdir -p "$W" && chmod 0700 "$W" || { printf '@@refused %s\n' "cannot create $W on $(hostname)"; exit 2; }
find "$(dirname "$W")" -mindepth 1 -maxdepth 1 -type d -mtime +7 -exec rm -rf {} + 2> /dev/null
cat > "$W/script" <<'ROSIE_WATCHED_SCRIPT' || { printf '@@refused %s\n' "cannot write $W/script"; exit 2; }
` + script + `ROSIE_WATCHED_SCRIPT
: > "$W/out"
command -v setsid > /dev/null 2>&1 || { printf '@@refused %s\n' "setsid not found on $(hostname)"; exit 2; }
setsid bash -c 'bash "$1/script" > "$1/out" 2>&1; echo $? > "$1/exit.tmp"; mv -f "$1/exit.tmp" "$1/exit"' watched "$W" < /dev/null > /dev/null 2>&1 &
echo $! > "$W/pid"
printf '@@watch started %s\n' "pid $! on $(hostname) in $W: it keeps running if this connection drops" >&2
` + remoteWatchFollowShell() + "watch_follow 0\n"
}

// remoteWatchAttachScript re-attaches to a watched session from byte offset.
func remoteWatchAttachScript(id string, offset int64) string {
	return "set -u\nW=" + remoteWatchDirShell(id) + `
[ -s "$W/pid" ] || { printf '@@watch lost %s\n' "no watched session in $W on $(hostname)" >&2; exit 255; }
state=running
if [ -e "$W/exit" ]; then state="exited with status $(cat "$W/exit")"
elif ! kill -0 "$(cat "$W/pid")" 2> /dev/null; then state="gone without an exit status"; fi
printf '@@watch attach %s\n' "on $(hostname): the work is $state; resuming at byte ` + strconv.FormatInt(offset, 10) + `" >&2
` + remoteWatchFollowShell() + "watch_follow " + strconv.FormatInt(offset, 10) + "\n"
}

// remoteWatchStopScript stops a watched session's process group: TERM, then
// KILL after remoteWatchStopWaitS. It runs on the relay only (the build host;
// nothing here signals a process on the servo host).
func remoteWatchStopScript(id string) string {
	return "set -u\nW=" + remoteWatchDirShell(id) + `
pid=$(cat "$W/pid" 2> /dev/null)
[ -n "$pid" ] || { printf '@@result %s\n' '{"stopped":"none","reason":"no pid file"}'; exit 0; }
if [ -e "$W/exit" ] || ! kill -0 "$pid" 2> /dev/null; then printf '@@result {"stopped":"none","reason":"already ended"}\n'; exit 0; fi
kill -TERM -- "-$pid" 2> /dev/null || kill -TERM "$pid"
for i in $(seq 1 ` + strconv.Itoa(remoteWatchStopWaitS) + `); do kill -0 "$pid" 2> /dev/null || break; sleep 1; done
if kill -0 "$pid" 2> /dev/null; then
  kill -KILL -- "-$pid" 2> /dev/null; printf '@@result {"stopped":"KILL","pid":%s}\n' "$pid"
else
  printf '@@result {"stopped":"TERM","pid":%s}\n' "$pid"
fi
`
}

// watchedSession runs a long relay script as a watched session (above) and
// returns like session. A stream that ends without the work's exit status
// (ssh exit 255, the keepalives) or goes silent is re-attached once over a
// fresh connection; one past limit is stopped on the relay and fails.
func (r *cellDeployRun) watchedSession(label, script string, limit time.Duration) (int, []byte, []byte, []string, error) {
	id := remoteWatchID(r.id, label)
	w := &remoteWatch{Silence: remoteWatchSilence, Deadline: r.p.now().Add(limit)}
	r.p.mu.Lock()
	r.p.watch = w
	r.p.mu.Unlock()
	defer func() {
		r.p.mu.Lock()
		r.p.watch = nil
		r.p.mu.Unlock()
	}()
	snapshot := func() (exit *int, lost, last string, delivered int64) {
		r.p.mu.Lock()
		defer r.p.mu.Unlock()
		return w.exit, w.lost, w.last, w.delivered
	}
	where := "~/.rosie/watched/" + id + " on " + r.ctx.RelayPeer
	code, probe, result, refused, err := r.runSession(label, remoteWatchStartScript(id, script), true)
	resumedAt := int64(-1)
	for n := 0; ; n++ {
		exit, lost, last, delivered := snapshot()
		if exit != nil {
			return *exit, probe, result, refused, nil
		}
		if lost != "" {
			return code, probe, result, refused, fmt.Errorf("%s: %s (last line %q)", label, lost, last)
		}
		var stall *remoteStallError
		stalled := errors.As(err, &stall)
		if !stalled && (err != nil || code != 255) {
			return code, probe, result, refused, err
		}
		reason := "the connection ended (ssh exit 255) before the work's exit status arrived"
		if stalled {
			reason = stall.Reason
		}
		if stalled && stall.Overrun {
			r.p.Step("STALL", fmt.Sprintf("%s %s; last line: %q; stopping the relay work (%s)", label, reason, last, where))
			r.p.mu.Lock()
			r.p.watch = nil
			r.p.mu.Unlock()
			scode, _, sresult, _, serr := r.runSession(label+" (stop)", remoteWatchStopScript(id), false)
			return -1, probe, result, refused, fmt.Errorf("%s %s (limit %s); stop on the relay: exit %d %s %v; last line %q; output in %s",
				label, reason, limit.Round(time.Second), scode, strings.TrimSpace(string(sresult)), serr, last, where)
		}
		if delivered == resumedAt || n >= remoteWatchReattachMax {
			return code, probe, result, refused, fmt.Errorf("%s: %s, again with no new output since the re-attach (last line %q); the relay work may still run (output in %s): realtime-core maintenance show",
				label, reason, last, where)
		}
		r.p.Step("stall", fmt.Sprintf("%s %s; last line: %q; re-attaching over a fresh connection at byte %d (%s)", label, reason, last, delivered, where))
		resumedAt = delivered
		code, probe, result, refused, err = r.runSession(label+" (re-attach)", remoteWatchAttachScript(id, delivered), false)
	}
}
