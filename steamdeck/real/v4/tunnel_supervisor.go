package main

import (
	"bytes"
	"encoding/xml"
	"fmt"
	"os"
	"sort"
	"strconv"
	"strings"
)

// A supervised ssh forward to a cell's relay, shared by the operator Mac's
// `olp ui` (olp_ui.go) and the Steam Deck's `pendant tunnel`
// (pendant_tunnel.go). Two layers keep it up. The host's service manager owns
// a rendered bash supervisor and restarts it if it exits: launchd (KeepAlive,
// gui/<uid>) on the Mac, a systemd --user unit (Restart=always, WantedBy
// default.target) on the Deck. Inside, the supervisor restarts ssh whenever it
// exits, with a doubling backoff, and records why in tunnel.state.
//
// Until 2026-10-08 both ends were hand setups: the Mac's tunnel died twice in
// ~12 h on "Timeout, server <relay> not responding" and stayed dead; the
// Deck's hand unit (rosie-v5-<robot>-tunnel) restarted ssh every 5 s with
// ServerAlive 15 x 3 and kept no record of why.

// The tunnel's dead-link detection and restart pacing. Chosen, not measured
// (2026-10-08, unverified): the hand tunnels ran ServerAliveInterval=15 x 3
// and sat dead for 45 s before ssh noticed. 5 s x 3 (sshAliveArgs, shared
// with every deploy session) notices a dead path in ~15 s; the first retry
// after a run of at least 30 s comes 1 s later, then the delay doubles to
// 30 s while the relay stays unreachable. On the Mac a killed ssh was back
// through the tunnel in 1.36 s (a Rosie 1430 cell, 2026-10-08).
const (
	tunnelBackoffMinMS  = 1000
	tunnelBackoffMaxMS  = 30000
	tunnelHealthyAfterS = 30
	tunnelLogMaxBytes   = 8 << 20
	// The service manager's own restart pacing for a supervisor that exits
	// (launchd ThrottleInterval, systemd RestartSec).
	tunnelServiceRestartS = 5
)

// tunnelForward is one -L: 127.0.0.1:LocalPort on this host to Remote
// (host:port as the relay resolves it).
type tunnelForward struct {
	LocalPort int
	Remote    string
}

func (f tunnelForward) arg() string { return fmt.Sprintf("127.0.0.1:%d:%s", f.LocalPort, f.Remote) }

// tunnelSpec is one supervised forward: the ssh that reaches the relay, its
// forwards (the first is the one the health check uses and the orphan check
// matches) and where the supervisor keeps its log and state.
type tunnelSpec struct {
	Title      string   // "OLP UI tunnel for <cell>"
	RenderedBy string   // the command that renders it
	SSH        []string // the client up to and including the target: tailscale ssh user@host, /usr/bin/ssh user@host
	Options    []string // extra -o / -i arguments after the common ones
	Forwards   []tunnelForward
	LogPath    string
	StatePath  string

	backoffMinMS, backoffMaxMS, healthyAfterS int
}

// tunnelArgv is the forwarding ssh. ExitOnForwardFailure makes a taken local
// port an exit (and a recorded reason) instead of a tunnel that forwards
// nothing; ControlPath=none keeps a user's ssh multiplexing from turning -N
// into a client of some other connection. OpenSSH and tailscale ssh both read
// options after the destination.
func tunnelArgv(s tunnelSpec) []string {
	argv := append(append([]string{}, s.SSH...), "-N", "-o", "ExitOnForwardFailure=yes")
	argv = append(argv, sshAliveArgs(sshConnectTimeout)...)
	argv = append(argv, "-o", "BatchMode=yes", "-o", "ControlMaster=no", "-o", "ControlPath=none")
	argv = append(argv, s.Options...)
	for _, f := range s.Forwards {
		argv = append(argv, "-L", f.arg())
	}
	return argv
}

// tunnelSupervisorScript renders the supervisor. `set -m` gives every attempt
// its own process group: `tailscale ssh` is a /bin/sh wrapper whose child is
// the real ssh, so killing only the wrapper would orphan an ssh that keeps the
// port and fails every later attempt with "Address already in use".
func tunnelSupervisorScript(s tunnelSpec) string {
	quoted := make([]string, 0, 32)
	for _, a := range tunnelArgv(s) {
		quoted = append(quoted, shellQuote(a))
	}
	minMS, maxMS, healthy := s.backoffMinMS, s.backoffMaxMS, s.healthyAfterS
	if minMS == 0 {
		minMS, maxMS, healthy = tunnelBackoffMinMS, tunnelBackoffMaxMS, tunnelHealthyAfterS
	}
	return fmt.Sprintf(`#!/bin/bash
# %[1]s, rendered by `+"`%[10]s`"+`
# (steamdeck/real/v4/tunnel_supervisor.go); edits are overwritten. The service
# manager keeps this script alive; this loop restarts ssh whenever it exits,
# the delay doubling from backoff_min_ms to backoff_max_ms and resetting after
# a run of at least healthy_after_s. tunnel.state carries the last restart
# reason.
set -u -m
log=%[2]s
state=%[3]s
cmd=(%[4]s)
backoff_min_ms=%[5]d
backoff_max_ms=%[6]d
healthy_after_s=%[7]d
log_max_bytes=%[8]d

child=""
attempt=0
restarts=0
backoff_ms=$backoff_min_ms
phase=starting
up_since=""
last_exit="" last_rc="" last_reason=""

now() { date '+%%Y-%%m-%%dT%%H:%%M:%%S%%z'; }
write_state() {
  {
    printf 'supervisor_pid=%%s\n' "$$"
    printf 'phase=%%s\n' "$phase"
    printf 'ssh_pid=%%s\n' "$child"
    printf 'attempt=%%s\n' "$attempt"
    printf 'up_since=%%s\n' "$up_since"
    printf 'restarts=%%s\n' "$restarts"
    printf 'backoff_ms=%%s\n' "$backoff_ms"
    printf 'last_exit=%%s\n' "$last_exit"
    printf 'last_rc=%%s\n' "$last_rc"
    printf 'last_reason=%%s\n' "$last_reason"
  } >"$state.tmp" && mv -f "$state.tmp" "$state"
}
stop() {
  if [ -n "$child" ]; then
    kill -TERM -- "-$child" 2>/dev/null
    wait "$child" 2>/dev/null
  fi
  child="" phase=stopped up_since=""
  write_state
  printf '%%s @@tunnel supervisor %%s stopped\n' "$(now)" "$$" >>"$log"
  exit 0
}
trap stop TERM INT HUP

printf '%%s @@tunnel supervisor %%s started\n' "$(now)" "$$" >>"$log"
# A supervisor that was killed outright leaves its attempt (its own process
# group) holding the port; end it if its command is still this forward.
old=$(sed -n 's/^ssh_pid=//p' "$state" 2>/dev/null)
if [ -n "$old" ] && ps -o command= -p "$old" 2>/dev/null | grep -qF -- %[9]s; then
  printf '%%s @@tunnel ending attempt process group %%s left by a previous supervisor\n' "$(now)" "$old" >>"$log"
  kill -TERM -- "-$old" 2>/dev/null
  sleep 0.5
fi
while :; do
  size=$(wc -c <"$log" 2>/dev/null || echo 0)
  if [ $((size)) -gt "$log_max_bytes" ]; then mv -f "$log" "$log.1"; fi
  attempt=$((attempt + 1))
  started=$(date +%%s)
  printf '%%s @@tunnel attempt %%d: %%s\n' "$(now)" "$attempt" "${cmd[*]}" >>"$log"
  offset=$(wc -c <"$log")
  "${cmd[@]}" </dev/null >>"$log" 2>&1 &
  child=$!
  phase=up up_since=$(now)
  write_state
  wait "$child"
  rc=$?
  child=""
  ran=$(( $(date +%%s) - started ))
  reason=$(tail -c +$((offset + 1)) "$log" | tr -d '\r' | grep -Ev '^(channel [0-9]+: open failed|$)' | tail -n 1)
  if [ -z "$reason" ]; then
    if [ "$rc" -gt 128 ]; then reason="ssh ended by signal $((rc - 128))"; else reason="ssh exited without a message"; fi
  fi
  if [ "$ran" -ge "$healthy_after_s" ]; then backoff_ms=$backoff_min_ms; fi
  restarts=$((restarts + 1))
  last_exit=$(now) last_rc=$rc last_reason=$reason
  phase=backoff up_since=""
  write_state
  printf '%%s @@tunnel attempt %%d ended rc=%%d after %%ds: %%s; retry in %%d ms\n' "$(now)" "$attempt" "$rc" "$ran" "$reason" "$backoff_ms" >>"$log"
  sleep "$(printf '%%d.%%03d' $((backoff_ms / 1000)) $((backoff_ms %% 1000)))" &
  child=$!
  wait "$child"
  child=""
  backoff_ms=$((backoff_ms * 2))
  if [ "$backoff_ms" -gt "$backoff_max_ms" ]; then backoff_ms=$backoff_max_ms; fi
done
`, s.Title, shellQuote(s.LogPath), shellQuote(s.StatePath), strings.Join(quoted, " "),
		minMS, maxMS, healthy, tunnelLogMaxBytes, shellQuote("-L "+s.Forwards[0].arg()), s.RenderedBy)
}

// --- tunnel state -----------------------------------------------------------

type tunnelState struct {
	SupervisorPID int
	Phase         string
	SSHPID        int
	Attempt       int
	UpSince       string
	Restarts      int
	BackoffMS     int
	LastExit      string
	LastRC        string
	LastReason    string
}

func readTunnelState(p string) (tunnelState, error) {
	raw, err := os.ReadFile(p)
	if err != nil {
		return tunnelState{}, err
	}
	return parseTunnelState(string(raw)), nil
}

// parseTunnelState reads tunnel.state's key=value lines (the Mac reads the
// file, the Deck's status sends its text).
func parseTunnelState(raw string) tunnelState {
	var s tunnelState
	for _, line := range strings.Split(raw, "\n") {
		k, v, ok := strings.Cut(line, "=")
		if !ok {
			continue
		}
		n, _ := strconv.Atoi(v)
		switch k {
		case "supervisor_pid":
			s.SupervisorPID = n
		case "phase":
			s.Phase = v
		case "ssh_pid":
			s.SSHPID = n
		case "attempt":
			s.Attempt = n
		case "up_since":
			s.UpSince = v
		case "restarts":
			s.Restarts = n
		case "backoff_ms":
			s.BackoffMS = n
		case "last_exit":
			s.LastExit = v
		case "last_rc":
			s.LastRC = v
		case "last_reason":
			s.LastReason = v
		}
	}
	return s
}

func (s tunnelState) lastRestart() string {
	if s.Restarts == 0 {
		return "no restart since the supervisor started"
	}
	return fmt.Sprintf("%d restarts; last at %s rc=%s: %s", s.Restarts, s.LastExit, s.LastRC, s.LastReason)
}

// --- service managers -------------------------------------------------------

// serviceJob is one long-running process the host's service manager keeps
// alive: a launchd agent on the Mac, a systemd --user unit on the Deck.
type serviceJob struct {
	Label       string
	Description string
	Argv        []string
	Dir         string
	Env         map[string]string
	Log         string // launchd only: stdout and stderr (systemd: the journal)
}

func xmlText(s string) string {
	var b bytes.Buffer
	_ = xml.EscapeText(&b, []byte(s))
	return b.String()
}

// launchdPlist renders a launchd agent that starts at bootstrap and is
// restarted whenever it exits (at most every tunnelServiceRestartS).
// Interactive keeps App Nap from throttling the operator's UI.
func launchdPlist(j serviceJob) string {
	var b strings.Builder
	b.WriteString(`<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
`)
	fmt.Fprintf(&b, "\t<key>Label</key>\n\t<string>%s</string>\n\t<key>ProgramArguments</key>\n\t<array>\n", xmlText(j.Label))
	for _, a := range j.Argv {
		fmt.Fprintf(&b, "\t\t<string>%s</string>\n", xmlText(a))
	}
	fmt.Fprintf(&b, "\t</array>\n\t<key>WorkingDirectory</key>\n\t<string>%s</string>\n\t<key>EnvironmentVariables</key>\n\t<dict>\n", xmlText(j.Dir))
	for _, k := range sortedKeys(j.Env) {
		fmt.Fprintf(&b, "\t\t<key>%s</key>\n\t\t<string>%s</string>\n", xmlText(k), xmlText(j.Env[k]))
	}
	fmt.Fprintf(&b, "\t</dict>\n\t<key>StandardOutPath</key>\n\t<string>%[1]s</string>\n\t<key>StandardErrorPath</key>\n\t<string>%[1]s</string>\n", xmlText(j.Log))
	fmt.Fprintf(&b, "\t<key>RunAtLoad</key>\n\t<true/>\n\t<key>KeepAlive</key>\n\t<true/>\n\t<key>ThrottleInterval</key>\n\t<integer>%d</integer>\n\t<key>ProcessType</key>\n\t<string>Interactive</string>\n</dict>\n</plist>\n", tunnelServiceRestartS)
	return b.String()
}

// systemdUserUnit renders a systemd --user service restarted whenever it
// exits and never given up on (StartLimitIntervalSec=0: a relay unreachable
// for hours must not leave the unit failed). WantedBy=default.target starts it
// with the user's manager: at login, which SteamOS does at boot, and it runs
// through sleep and resume. Stopping kills the whole cgroup, the supervisor's
// ssh attempts included.
func systemdUserUnit(j serviceJob) string {
	var b strings.Builder
	fmt.Fprintf(&b, "[Unit]\nDescription=%s\nAfter=network-online.target\nStartLimitIntervalSec=0\n\n[Service]\nType=simple\n", strings.ReplaceAll(j.Description, "%", "%%"))
	quoted := make([]string, 0, len(j.Argv))
	for _, a := range j.Argv {
		quoted = append(quoted, systemdQuote(a))
	}
	fmt.Fprintf(&b, "ExecStart=%s\n", strings.Join(quoted, " "))
	if j.Dir != "" {
		fmt.Fprintf(&b, "WorkingDirectory=%s\n", systemdQuote(j.Dir))
	}
	for _, k := range sortedKeys(j.Env) {
		fmt.Fprintf(&b, "Environment=%s\n", systemdQuote(k+"="+j.Env[k]))
	}
	fmt.Fprintf(&b, "Restart=always\nRestartSec=%d\nKillMode=control-group\nTimeoutStopSec=10\n\n[Install]\nWantedBy=default.target\n", tunnelServiceRestartS)
	return b.String()
}

// systemdQuote quotes one word for a unit file when it needs it; `%` is
// doubled so systemd's specifier expansion leaves the text alone.
func systemdQuote(s string) string {
	s = strings.ReplaceAll(s, "%", "%%")
	if s != "" && !strings.ContainsAny(s, " \t\"'\\$;") {
		return s
	}
	r := strings.NewReplacer(`\`, `\\`, `"`, `\"`, "$", "$$")
	return `"` + r.Replace(s) + `"`
}

func sortedKeys(m map[string]string) []string {
	keys := make([]string, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return keys
}

// splitAction finds a subcommand's action as its first or last argument, so
// both `olp ui start --manifest x` and `olp ui --manifest x start` work.
func splitAction(actions, args []string) (string, []string) {
	if len(args) > 0 && containsString(actions, args[0]) {
		return args[0], args[1:]
	}
	if n := len(args); n > 0 && containsString(actions, args[n-1]) {
		return args[n-1], args[:n-1]
	}
	return "", args
}
