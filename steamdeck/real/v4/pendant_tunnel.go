package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"os"
	"path"
	"regexp"
	"strconv"
	"strings"
	"time"
)

// pendant tunnel keeps a Steam Deck's ssh tunnel to its cell's relay up: the
// pendant reaches the cell host's OLP server through it (pendant.env
// ROSIE_PENDANT_OLP_ORIGIN=http://127.0.0.1:<olp_port>), because SteamOS runs
// Tailscale in userspace-networking mode and apps on the Deck cannot reach
// tailnet addresses. It is the Mac's `olp ui` tunnel (tunnel_supervisor.go:
// ServerAlive 5 x 3, retry 1 s doubling to 30 s, restart reason in
// tunnel.state, orphan reaping) under a systemd --user unit, rendered from the
// Deck's pendant_deploy.decks[].tunnel block:
//
//	"tunnel": {"relay_host": "192.168.1.10",
//	           "identity_file": "/home/deck/.ssh/rosie-<robot>-tunnel",
//	           "known_hosts": "/home/deck/.ssh/rosie-<robot>-known_hosts",
//	           "olp_port": 18794,
//	           "forwards": [{"local_port": 18443, "remote": "127.0.0.1:18443"}],
//	           "replaces": ["rosie-v5-<robot>-tunnel.service"]}
//
// The relay user and the OLP's address on the relay come from cell_deploy
// (relay ssh_user, olp.listen). The key, its authorized_keys restriction on
// the relay (permitopen, from=) and the pinned host key are per-Deck
// credentials provisioned by hand; this command only names them. It replaces
// the units in "replaces" (the hand-made unit of 2026-10-06) and keeps their
// files for rollback.
//
// "relay_auth": "tailscale_ssh" is a Deck whose Tailscale runs with a kernel
// tun device (one Deck), or in userspace mode with "tailscale_nc" naming
// the Deck's tailscale CLI (another Deck: apps cannot reach a tailnet address
// there, so ssh connects through "tailscale nc" as its ProxyCommand): the
// ssh reaches the relay's MagicDNS name and the relay's Tailscale SSH server
// admits it by the Deck's tailnet identity and the tailnet's SSH policy, so
// there is no key file and relay_user is the account that policy maps the
// Deck to. The relay's host key is still pinned (known_hosts). The policy, not
// an authorized_keys line, decides what the session may do: a full login, not
// forwards only.
//
// Taking the tunnel down while the Deck's pendant has the robot armed or holds
// the session lease lapses the pendant's heartbeat (1 s against the OLP's 5 s
// deadman) and the OLP Stops the robot, so every action that interrupts it
// refuses then; the Deck reads the OLP through the tunnel itself. The pendant
// itself needs no restart when the tunnel comes back: it polls status every
// 250 ms (2 s timeout) and the cell list every 2 s (steamdeck/real/v5
// src/machine.cpp).

const pendantTunnelUsage = `usage: pendant tunnel <install|status|logs|restart|stop|rollback> --host <deck> --manifest cell.json [flags]

A Deck's self-healing ssh tunnel to its cell's relay, rendered from the Deck's
pendant_deploy.decks[].tunnel block: a systemd --user unit (Restart=always,
started with the user's manager at login/boot, running through sleep) around
the supervisor the Mac's olp ui uses (ServerAlive 5 s x 3, retry 1 s doubling
to 30 s, restart reason in tunnel.state). The pendant's pendant.env origin
points at its OLP forward.

  install   Render and install the unit and supervisor; replace the units the
            block lists (hand-made), keeping their files for rollback; start it
            and wait for OLP health through the tunnel [--wait 30s]. Leaves a
            current, healthy tunnel alone and restarts a dead one at once. A
            switch that loses health the old tunnel had is undone (exit 5).
  status    Read-only: unit, supervisor phase and last restart reason, OLP
            health and build through the tunnel, robot use, the pendant
            package, pendant.env origin. Exit 1 when unhealthy.
  logs      The supervisor's tunnel.log and the unit's journal [--lines 40]
            [--follow].
  restart   Restart the unit and wait for health [--wait 30s].
  stop      Stop and disable the unit (the pendant loses its OLP).
  rollback  Put back what ran before the last install (the hand-made unit after
            the first one).
install, restart, stop and rollback refuse (exit 2, nothing changed) while
this Deck's pendant has the robot armed, holds the session lease or runs a
program. All take --dry-run (prints the session and rendered files, contacts
nothing). pendant status reports the tunnel too.
`

var pendantTunnelActions = []string{"install", "status", "logs", "restart", "stop", "rollback"}

type pendantTunnel struct {
	RelayHost    string           `json:"relay_host"`
	IdentityFile string           `json:"identity_file,omitempty"`
	KnownHosts   string           `json:"known_hosts"`
	OLPPort      int              `json:"olp_port"`
	Forwards     []pendantForward `json:"forwards,omitempty"`
	Replaces     []string         `json:"replaces,omitempty"`
	StateDir     string           `json:"state_dir,omitempty"`
	// RelayAuth is "key" (default: identity_file, restricted on the relay)
	// or "tailscale_ssh" (no key; the relay's Tailscale SSH policy admits
	// the Deck). RelayUser overrides cell_deploy's relay user.
	RelayAuth string `json:"relay_auth,omitempty"`
	RelayUser string `json:"relay_user,omitempty"`
	// TailscaleNC is the absolute path of the Deck's tailscale CLI when its
	// Tailscale runs in userspace mode: ssh then connects through
	// "<cli> nc %h %p". Only with relay_auth tailscale_ssh.
	TailscaleNC string `json:"tailscale_nc,omitempty"`
}

const pendantRelayAuthTailscaleSSH = "tailscale_ssh"

type pendantForward struct {
	LocalPort int    `json:"local_port"`
	Remote    string `json:"remote"`
}

var (
	pendantForwardRemote = regexp.MustCompile(`^[A-Za-z0-9._-]+:[0-9]{1,5}$`)
	pendantUnitName      = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9@._-]*\.service$`)
)

func pendantValidPort(p int) bool { return p >= 1024 && p <= 65535 }

func (t pendantTunnel) validate(field string) error {
	switch t.RelayAuth {
	case "", "key":
		if !pendantHost.MatchString(t.RelayHost) {
			return fmt.Errorf("%s.relay_host must be the relay's address as the Deck reaches it (the LAN: userspace Tailscale cannot carry the forward), got %q", field, t.RelayHost)
		}
		if err := cellDeployCleanAbs(field+".identity_file", t.IdentityFile); err != nil {
			return err
		}
	case pendantRelayAuthTailscaleSSH:
		if !pendantHost.MatchString(t.RelayHost) {
			return fmt.Errorf("%s.relay_host must be the relay's tailnet name (MagicDNS), got %q", field, t.RelayHost)
		}
		if t.IdentityFile != "" {
			return fmt.Errorf("%s: relay_auth tailscale_ssh uses no key; drop identity_file %q or use relay_auth key", field, t.IdentityFile)
		}
		if t.RelayUser == "" {
			return fmt.Errorf("%s: relay_auth tailscale_ssh needs relay_user, the relay account the tailnet's SSH policy admits this Deck as", field)
		}
		if t.TailscaleNC != "" {
			if err := cellDeployCleanAbs(field+".tailscale_nc", t.TailscaleNC); err != nil {
				return err
			}
		}
	default:
		return fmt.Errorf("%s.relay_auth must be key or %s, got %q", field, pendantRelayAuthTailscaleSSH, t.RelayAuth)
	}
	if t.TailscaleNC != "" && t.RelayAuth != pendantRelayAuthTailscaleSSH {
		return fmt.Errorf("%s.tailscale_nc is only for relay_auth %s (a key relay is reached over the LAN)", field, pendantRelayAuthTailscaleSSH)
	}
	if t.RelayUser != "" && !cellDeployToken.MatchString(t.RelayUser) {
		return fmt.Errorf("%s.relay_user must be an account name, got %q", field, t.RelayUser)
	}
	if err := cellDeployCleanAbs(field+".known_hosts", t.KnownHosts); err != nil {
		return err
	}
	if t.StateDir != "" {
		if err := cellDeployCleanAbs(field+".state_dir", t.StateDir); err != nil {
			return err
		}
	}
	if !pendantValidPort(t.OLPPort) {
		return fmt.Errorf("%s.olp_port must be 1024..65535 (the port pendant.env's ROSIE_PENDANT_OLP_ORIGIN names), got %d", field, t.OLPPort)
	}
	seen := map[int]bool{t.OLPPort: true}
	for i, f := range t.Forwards {
		if !pendantValidPort(f.LocalPort) || seen[f.LocalPort] {
			return fmt.Errorf("%s.forwards[%d].local_port must be a unique 1024..65535 port, got %d", field, i, f.LocalPort)
		}
		seen[f.LocalPort] = true
		if !pendantForwardRemote.MatchString(f.Remote) {
			return fmt.Errorf("%s.forwards[%d].remote must be host:port as the relay resolves it, got %q", field, i, f.Remote)
		}
	}
	for i, u := range t.Replaces {
		if !pendantUnitName.MatchString(u) {
			return fmt.Errorf("%s.replaces[%d] must be a systemd user unit name ending .service, got %q", field, i, u)
		}
	}
	return nil
}

// pendantTunnelPlan is one Deck's tunnel as the manifest renders it.
type pendantTunnelPlan struct {
	Cell, Unit, Dir string
	Deck            pendantDeck
	Relay           string // user@host as the Deck reaches it
	OLPListen       string
	Spec            tunnelSpec
	Ports           []int
}

func pendantTunnelResolve(ctx cellDeployContext, d pendantDeck) (pendantTunnelPlan, error) {
	t := d.Tunnel
	if t == nil {
		return pendantTunnelPlan{}, fmt.Errorf("deck %s has no tunnel block in %s's pendant_deploy; add one (steamdeck/real/v4/README.md, \"Deck tunnel\")", d.Name, ctx.CellID)
	}
	if !pendantForwardRemote.MatchString(ctx.Deploy.OLP.Listen) {
		return pendantTunnelPlan{}, fmt.Errorf("cell_deploy.olp.listen %q is not host:port", ctx.Deploy.OLP.Listen)
	}
	dir := t.StateDir
	if dir == "" {
		dir = path.Join("/home", d.SSHUser, ".rosie", "pendant-tunnel", ctx.CellID)
	}
	user := ctx.RelayUser
	if t.RelayUser != "" {
		user = t.RelayUser
	}
	p := pendantTunnelPlan{Cell: ctx.CellID, Unit: "rosie-pendant-tunnel-" + ctx.CellID + ".service", Dir: dir, Deck: d,
		Relay: remoteTarget(user, t.RelayHost), OLPListen: ctx.Deploy.OLP.Listen}
	forwards := []tunnelForward{{LocalPort: t.OLPPort, Remote: p.OLPListen}}
	for _, f := range t.Forwards {
		forwards = append(forwards, tunnelForward{LocalPort: f.LocalPort, Remote: f.Remote})
	}
	for _, f := range forwards {
		p.Ports = append(p.Ports, f.LocalPort)
	}
	auth := []string{"-i", t.IdentityFile, "-o", "IdentitiesOnly=yes"}
	if t.RelayAuth == pendantRelayAuthTailscaleSSH {
		// The Tailscale SSH server admits by tailnet identity ("none"
		// authentication); no key is offered.
		auth = []string{"-o", "PubkeyAuthentication=no"}
		if t.TailscaleNC != "" {
			auth = append(auth, "-o", "ProxyCommand="+t.TailscaleNC+" nc %h %p")
		}
	}
	p.Spec = tunnelSpec{Title: "Pendant tunnel supervisor for " + ctx.CellID + " on " + d.Name, RenderedBy: "go run . pendant tunnel install",
		SSH:      []string{"/usr/bin/ssh", p.Relay},
		Options:  append(auth, "-o", "StrictHostKeyChecking=yes", "-o", "UserKnownHostsFile="+t.KnownHosts),
		Forwards: forwards, LogPath: path.Join(dir, "tunnel.log"), StatePath: path.Join(dir, "tunnel.state")}
	return p, nil
}

func (p pendantTunnelPlan) OLPOrigin() string {
	return p.Deck.tunnelOrigin()
}

func (p pendantTunnelPlan) describe() string {
	var extra []string
	for _, f := range p.Spec.Forwards[1:] {
		extra = append(extra, fmt.Sprintf("%d -> %s", f.LocalPort, f.Remote))
	}
	s := fmt.Sprintf("127.0.0.1:%d -> %s %s", p.Deck.Tunnel.OLPPort, p.Relay, p.OLPListen)
	if len(extra) > 0 {
		s += " (also " + strings.Join(extra, ", ") + ")"
	}
	return s
}

func (p pendantTunnelPlan) unitFile() string {
	return systemdUserUnit(serviceJob{Label: p.Unit, Description: fmt.Sprintf("Rosie pendant tunnel for %s: %s (go run . pendant tunnel)", p.Cell, p.describe()),
		Argv: []string{"/bin/bash", path.Join(p.Dir, "tunnel.sh")}, Dir: p.Dir})
}

// pendantTunnelOpts are an action's knobs as the Deck script reads them.
type pendantTunnelOpts struct {
	Action string
	Wait   time.Duration
	Lines  int
	Follow bool
}

// pendantTunnelIdlePy decides whether interrupting the tunnel can stop the
// robot: argv status.json, control.json (both read through the tunnel; empty
// when the OLP did not answer) and this Deck's pendant label as the OLP shows
// it ("rosie-pendant-v5 (<hostname>)", steamdeck/real/v5 src/olp_client.cpp).
// Exit 1 while the robot is armed, leased, running a program or held in a jog
// by this pendant or a client the OLP does not name; another named client
// (a browser on the Mac) does not use this tunnel.
const pendantTunnelIdlePy = `import json, sys
def load(p):
    try:
        with open(p) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None
st, ctl, me = load(sys.argv[1]), load(sys.argv[2]), sys.argv[3]
if not isinstance(st, dict):
    print("OLP status unreadable through the tunnel: nothing goes through it to interrupt")
    sys.exit(0)
ctl = ctl if isinstance(ctl, dict) else {}
s = st.get("session") or {}
busy = []
if st.get("armed"):
    busy.append("armed")
if (s.get("lease") or {}).get("held") and s.get("state") != "stopped":
    busy.append("session lease held")
if s.get("state") in ("playing", "paused"):
    busy.append("program " + s["state"])
if st.get("held"):
    busy.append("jog held")
if not busy:
    print("robot not in use through this OLP (session %s, disarmed, no lease)" % (s.get("state") or "none"))
    sys.exit(0)
if ctl.get("state") == "other_server":
    print("robot %s through another server, not this tunnel" % ", ".join(busy))
    sys.exit(0)
label = (ctl.get("owner") or {}).get("label") or ""
if label and label != me:
    print("robot %s by %s, not this Deck's pendant (%s)" % (", ".join(busy), label, me))
    sys.exit(0)
print("robot %s by %s" % (", ".join(busy), label or "a client the OLP does not name"))
sys.exit(1)
`

// pendantTunnelProbePy prints the @@probe line status reads. argv: unit,
// state dir, legacy units, ports open ("18794=1 18443=0"), files current
// (tunnel.sh and unit: 1/0), health file, health error, health ms, idle text,
// app dir, launcher (the deck's, may be empty). The pendant's origin is read
// as run.sh resolves it: pendant.env, else the launcher's export.
const pendantTunnelProbePy = `import json, os, subprocess, sys
unit, d, legacy, ports, files, hfile, herr, hms, idle, app, launcher = sys.argv[1:12]
def sh(*a):
    try:
        return subprocess.run(a, capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception:
        return ""
def show(u):
    out = sh("systemctl", "--user", "show", u, "-p", "LoadState,ActiveState,SubState,UnitFileState,NRestarts,MainPID,ActiveEnterTimestamp")
    return dict(l.split("=", 1) for l in out.splitlines() if "=" in l)
def read(p):
    try:
        with open(p) as f:
            return f.read()
    except OSError:
        return None
units = os.path.join(os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config"), "systemd", "user")
health = {"ok": False, "error": herr}
body = read(hfile)
if body:
    try:
        health = {"ok": True, "ms": int(hms or 0), "body": json.loads(body)}
    except ValueError:
        health = {"ok": False, "error": "not JSON: " + body[:120]}
def origin_in(name):
    found = None
    for line in (read(os.path.join(app, name)) or "").splitlines():
        line = line.strip()
        if line.startswith("export "):
            line = line[len("export "):].lstrip()
        if line.startswith("ROSIE_PENDANT_OLP_ORIGIN="):
            found = line.split("=", 1)[1].strip().strip("'\"")
    return found
origin, origin_from = origin_in("pendant.env"), "pendant.env"
if not origin and launcher:
    origin, origin_from = origin_in(launcher), launcher
running = subprocess.run(["pgrep", "-f", "bin/rosie-pendant-v[5]"], capture_output=True).returncode == 0
user = sh("id", "-un")
probe = {
    "unit": show(unit),
    "unit_file": os.path.exists(os.path.join(units, unit)),
    "files_current": files,
    "legacy": [dict(show(u), unit=u, file=os.path.exists(os.path.join(units, u))) for u in legacy.split()],
    "state": read(os.path.join(d, "tunnel.state")),
    "ports": dict((p.split("=")[0], p.split("=")[1] == "1") for p in ports.split()),
    "health": health,
    "idle": idle,
    "pendant": {"commit": (read(os.path.join(app, "SOURCE_COMMIT")) or "unknown").strip(), "running": running, "origin": origin, "origin_from": origin_from},
    "linger": sh("loginctl", "show-user", user, "-p", "Linger", "--value"),
    "replaced": sorted(os.listdir(os.path.join(d, "replaced"))) if os.path.isdir(os.path.join(d, "replaced")) else [],
    "rollback": os.path.isfile(os.path.join(d, "rollback", "units")),
}
print("@@probe " + json.dumps(probe, separators=(",", ":")))
`

// pendantTunnelDeckScript is the Deck half of every action, sent on stdin
// after its variables and the rendered files. Progress: @@step, @@refused,
// @@result and one @@probe line (status's facts).
const pendantTunnelDeckScript = `set -uo pipefail
export LC_ALL=C
TT0=$(date +%s%N)
tstep(){ printf '@@step %d %s\n' $(( ($(date +%s%N) - TT0) / 1000000 )) "$*"; }
trefuse(){ printf '@@refused %s\n' "$*"; exit 2; }
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
export DBUS_SESSION_BUS_ADDRESS=${DBUS_SESSION_BUS_ADDRESS:-unix:path=$XDG_RUNTIME_DIR/bus}
TUN_UNITS_DIR=${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user
TUN_ROLLBACK=$TUN_DIR/rollback
TUN_API=http://127.0.0.1:$TUN_OLP_PORT/api/offline-programming/v1
TW=$(mktemp -d) || trefuse "mktemp failed"
trap 'rm -rf -- "$TW"' EXIT
mkdir -p "$TW/new" && tun_render "$TW/new"

tun_active(){ systemctl --user -q is-active "$1" 2> /dev/null; }
tun_enabled(){ systemctl --user -q is-enabled "$1" 2> /dev/null; }
tun_port_open(){ (exec 3<> "/dev/tcp/127.0.0.1/$1") 2> /dev/null; }
tun_files_current(){ cmp -s "$TW/new/tunnel.sh" "$TUN_DIR/tunnel.sh" && cmp -s "$TW/new/unit" "$TUN_UNITS_DIR/$TUN_UNIT"; }
tun_any_active(){ local u; for u in "$TUN_UNIT" $TUN_LEGACY; do tun_active "$u" && return 0; done; return 1; }
tun_health(){ # OLP health through the tunnel into $TW/health.json; TUN_HERR on failure
  rm -f "$TW/health.json"
  if curl -fsS -m 3 -o "$TW/health.json.part" "$TUN_API/health" 2> "$TW/health.err"; then mv -f "$TW/health.json.part" "$TW/health.json"; TUN_HERR=""; return 0; fi
  TUN_HERR=$(tail -n 1 "$TW/health.err" | tr -d '\r'); TUN_HERR=${TUN_HERR:-no answer}; return 1
}
tun_wait_health(){ # $1 seconds; TUN_HMS: ms until health answered
  local t0 end
  t0=$(date +%s%N); end=$(( $(date +%s) + $1 ))
  while :; do
    if tun_health; then TUN_HMS=$(( ($(date +%s%N) - t0) / 1000000 )); return 0; fi
    [ "$(date +%s)" -lt "$end" ] || return 1
    sleep 0.2
  done
}
tun_idle_text(){ # sets TUN_IDLE; returns 1 while this Deck's pendant uses the robot
  curl -fsS -m 3 -o "$TW/status.json" "$TUN_API/dense-execution/status" 2> /dev/null || : > "$TW/status.json"
  curl -fsS -m 3 -o "$TW/control.json" "$TUN_API/control" 2> /dev/null || : > "$TW/control.json"
  TUN_IDLE=$(python3 -c "$TUN_IDLE_PY" "$TW/status.json" "$TW/control.json" "rosie-pendant-v5 ($(uname -n))")
}
tun_require_idle(){ # $1 what would interrupt the tunnel
  tun_idle_text; local rc=$?
  tstep "check $TUN_IDLE"
  [ $rc -eq 0 ] || trefuse "$1: $TUN_IDLE. Taking the tunnel down would lapse the pendant's heartbeat and the OLP would Stop the robot; finish, Stop or disarm on the Deck first"
}
tun_probe(){
  local ports="" p files=0 hms
  for p in $TUN_PORTS; do if tun_port_open "$p"; then ports="$ports $p=1"; else ports="$ports $p=0"; fi; done
  tun_files_current && files=1
  hms=$(date +%s%N); tun_health; hms=$(( ($(date +%s%N) - hms) / 1000000 ))
  [ -n "${TUN_IDLE:-}" ] || tun_idle_text
  python3 -c "$TUN_PROBE_PY" "$TUN_UNIT" "$TUN_DIR" "$TUN_LEGACY" "$ports" "$files" "$TW/health.json" "${TUN_HERR:-}" "$hms" "$TUN_IDLE" "$TUN_APP" "$TUN_LAUNCHER"
}
tun_await_ports_free(){ # after a stop: the forwards' ports must close
  local i p held
  for i in $(seq 1 50); do
    held=""; for p in $TUN_PORTS; do tun_port_open "$p" && held="$held $p"; done
    [ -z "$held" ] && return 0
    sleep 0.2
  done
  TUN_HELD=$held; return 1
}

# Puts back the unit files and states saved in $TUN_ROLLBACK and removes the
# rendered unit when the snapshot did not have it.
tun_restore(){ # $1 why
  local u en ac
  tstep "tunnel RESTORE: $1"
  systemctl --user disable --now "$TUN_UNIT" > /dev/null 2>&1
  if grep -q "^$TUN_UNIT " "$TUN_ROLLBACK/units" 2> /dev/null; then :; else rm -f "$TUN_UNITS_DIR/$TUN_UNIT"; fi
  [ ! -e "$TUN_ROLLBACK/tunnel.sh" ] || install -m 0700 "$TUN_ROLLBACK/tunnel.sh" "$TUN_DIR/tunnel.sh"
  while read -r u en ac; do
    install -m 0644 "$TUN_ROLLBACK/$u" "$TUN_UNITS_DIR/$u"
  done < "$TUN_ROLLBACK/units"
  systemctl --user daemon-reload
  if ! tun_await_ports_free; then tstep "tunnel ports$TUN_HELD still held after stopping $TUN_UNIT"; fi
  while read -r u en ac; do
    [ "$en" != enabled ] || systemctl --user enable "$u" > /dev/null 2>&1
    [ "$ac" != active ] || systemctl --user restart "$u"
    tstep "tunnel restored $u ($en, $ac)"
  done < "$TUN_ROLLBACK/units"
  if tun_wait_health "$TUN_WAIT"; then tstep "tunnel OLP health ok through 127.0.0.1:$TUN_OLP_PORT after the restore ($TUN_HMS ms)"; else tstep "tunnel no OLP health after the restore: $TUN_HERR"; fi
}

tun_install(){
  local legacy_live="" u before
  for u in $TUN_LEGACY; do
    if [ -e "$TUN_UNITS_DIR/$u" ] || tun_active "$u"; then legacy_live="$legacy_live $u"; fi
  done
  if tun_health; then before=ok; else before="no health ($TUN_HERR)"; fi
  tstep "tunnel now: $TUN_UNIT $(systemctl --user is-active "$TUN_UNIT" 2> /dev/null), files $(tun_files_current && echo current || echo differ from this rendering); hand-made:${legacy_live:- none}; OLP through 127.0.0.1:$TUN_OLP_PORT: $before"
  if tun_files_current && [ -z "$legacy_live" ] && tun_enabled "$TUN_UNIT" && tun_active "$TUN_UNIT"; then
    if [ "$before" = ok ]; then
      tstep "tunnel current and healthy; left alone"
      tun_probe; printf '@@result {"status":"unchanged"}\n'; return 0
    fi
    tstep "tunnel unit active but no OLP health: restarting it now instead of waiting out the backoff"
    systemctl --user restart "$TUN_UNIT"
    if tun_wait_health "$TUN_WAIT"; then tstep "tunnel OLP health ok through 127.0.0.1:$TUN_OLP_PORT $TUN_HMS ms after the restart"; tun_probe; printf '@@result {"status":"restarted"}\n'; return 0; fi
    tstep "tunnel no OLP health within $TUN_WAIT s: $TUN_HERR (the supervisor keeps retrying)"
    tun_probe; printf '@@result {"status":"unhealthy","reason":"no OLP health through the tunnel"}\n'; exit 1
  fi
  if ! tun_any_active; then
    for u in $TUN_PORTS; do
      tun_port_open "$u" && trefuse "127.0.0.1:$u is held by a process outside $TUN_UNIT${TUN_LEGACY:+ and }$TUN_LEGACY; stop it first (ss -ltnp 'sport = :$u' on the Deck)"
    done
  else
    tun_require_idle "replacing the tunnel"
  fi
  # What runs now, for rollback: unit files with their enabled and active
  # states, and the supervisor script.
  rm -rf -- "$TUN_ROLLBACK.new"; mkdir -p "$TUN_ROLLBACK.new" "$TUN_DIR" "$TUN_UNITS_DIR" || trefuse "cannot create $TUN_ROLLBACK.new"
  : > "$TUN_ROLLBACK.new/units"
  for u in "$TUN_UNIT" $TUN_LEGACY; do
    [ -e "$TUN_UNITS_DIR/$u" ] || continue
    cp -p "$TUN_UNITS_DIR/$u" "$TUN_ROLLBACK.new/$u" || trefuse "cannot keep $u"
    printf '%s %s %s\n' "$u" "$(tun_enabled "$u" && echo enabled || echo disabled)" "$(tun_active "$u" && echo active || echo inactive)" >> "$TUN_ROLLBACK.new/units"
  done
  [ ! -e "$TUN_DIR/tunnel.sh" ] || cp -p "$TUN_DIR/tunnel.sh" "$TUN_ROLLBACK.new/tunnel.sh"
  rm -rf -- "$TUN_ROLLBACK" && mv -- "$TUN_ROLLBACK.new" "$TUN_ROLLBACK" || trefuse "cannot keep the rollback copy in $TUN_ROLLBACK"
  tstep "tunnel rollback copy: $(tr '\n' ';' < "$TUN_ROLLBACK/units")"
  install -m 0700 "$TW/new/tunnel.sh" "$TUN_DIR/tunnel.sh" && install -m 0644 "$TW/new/unit" "$TUN_UNITS_DIR/$TUN_UNIT" \
    || { tun_restore "cannot write the rendered files"; printf '@@result {"status":"restored","reason":"cannot write the rendered files"}\n'; exit 5; }
  local down
  down=$(date +%s%N)
  for u in $legacy_live; do
    systemctl --user disable --now "$u" > /dev/null 2>&1
    mkdir -p "$TUN_DIR/replaced"
    [ -e "$TUN_DIR/replaced/$u" ] || [ ! -e "$TUN_UNITS_DIR/$u" ] || cp -p "$TUN_UNITS_DIR/$u" "$TUN_DIR/replaced/$u"
    rm -f "$TUN_UNITS_DIR/$u"
    tstep "tunnel hand-made $u stopped and disabled; its unit file kept in $TUN_DIR/replaced and the rollback copy"
  done
  systemctl --user daemon-reload
  if tun_active "$TUN_UNIT"; then
    systemctl --user stop "$TUN_UNIT"
  fi
  tun_await_ports_free || { tun_restore "ports$TUN_HELD still held after the old tunnel stopped"; printf '@@result {"status":"restored","reason":"ports%s still held"}\n' "$TUN_HELD"; exit 5; }
  tstep "tunnel old tunnel down, ports free"
  systemctl --user enable "$TUN_UNIT" > /dev/null 2>&1 && systemctl --user start "$TUN_UNIT" \
    || { tun_restore "systemctl could not start $TUN_UNIT"; printf '@@result {"status":"restored","reason":"start failed"}\n'; exit 5; }
  tstep "tunnel $TUN_UNIT enabled and started"
  if tun_wait_health "$TUN_WAIT"; then
    tstep "tunnel OLP health ok through 127.0.0.1:$TUN_OLP_PORT $TUN_HMS ms after the start; the pendant had no OLP for $(( ($(date +%s%N) - down) / 1000000 )) ms"
    tun_probe; printf '@@result {"status":"ok","unit":"%s"}\n' "$TUN_UNIT"; return 0
  fi
  if [ "$before" = ok ]; then
    tun_restore "no OLP health through the new tunnel within $TUN_WAIT s ($TUN_HERR); the old one had it"
    tun_probe; printf '@@result {"status":"restored","reason":"no OLP health through the new tunnel"}\n'; exit 5
  fi
  tstep "tunnel no OLP health within $TUN_WAIT s ($TUN_HERR), as before the switch: the relay or its OLP is down; the new tunnel keeps retrying"
  tun_probe; printf '@@result {"status":"unhealthy","reason":"no OLP health before or after"}\n'; exit 1
}

tun_restart(){
  [ -e "$TUN_UNITS_DIR/$TUN_UNIT" ] || trefuse "$TUN_UNIT is not installed: run pendant tunnel install"
  tun_require_idle "restarting the tunnel"
  systemctl --user restart "$TUN_UNIT" || { printf '@@result {"status":"failed","reason":"systemctl restart failed"}\n'; exit 1; }
  tstep "tunnel $TUN_UNIT restarted"
  if tun_wait_health "$TUN_WAIT"; then
    tstep "tunnel OLP health ok through 127.0.0.1:$TUN_OLP_PORT $TUN_HMS ms after the restart"
    tun_probe; printf '@@result {"status":"ok"}\n'; return 0
  fi
  tstep "tunnel no OLP health within $TUN_WAIT s: $TUN_HERR (the supervisor keeps retrying)"
  tun_probe; printf '@@result {"status":"unhealthy","reason":"no OLP health through the tunnel"}\n'; exit 1
}

tun_stop(){
  tun_require_idle "stopping the tunnel"
  systemctl --user disable --now "$TUN_UNIT" > /dev/null 2>&1
  tun_await_ports_free || { tun_probe; printf '@@result {"status":"failed","reason":"ports%s still held after the stop"}\n' "$TUN_HELD"; exit 1; }
  tstep "tunnel $TUN_UNIT stopped and disabled; ports free (the pendant has no OLP until pendant tunnel install)"
  tun_probe; printf '@@result {"status":"stopped"}\n'
}

tun_rollback(){
  [ -f "$TUN_ROLLBACK/units" ] || trefuse "no rollback copy in $TUN_ROLLBACK (pendant tunnel install keeps one)"
  tun_any_active && tun_require_idle "rolling the tunnel back"
  tun_restore "rollback to $(tr '\n' ';' < "$TUN_ROLLBACK/units")"
  tun_probe
  if [ -n "$TUN_HERR" ]; then printf '@@result {"status":"rolled_back","health":"not ok"}\n'; exit 1; fi
  printf '@@result {"status":"rolled_back"}\n'
}

tun_logs(){
  echo "== $TUN_DIR/tunnel.log"
  tail -n "$TUN_LINES" "$TUN_DIR/tunnel.log" 2> /dev/null || echo "(none)"
  echo "== journalctl --user -u $TUN_UNIT"
  journalctl --user -u "$TUN_UNIT" -n "$TUN_LINES" --no-pager 2> /dev/null | cut -c1-300
  [ "$TUN_FOLLOW" != 1 ] || exec tail -n 0 -F "$TUN_DIR/tunnel.log"
}

case ${TUN_ACTION:-} in
  install) tun_install ;;
  restart) tun_restart ;;
  stop) tun_stop ;;
  rollback) tun_rollback ;;
  status) tun_probe ;;
  logs) tun_logs ;;
  *) echo "pendant tunnel: TUN_ACTION must be install, status, logs, restart, stop or rollback" >&2; exit 2 ;;
esac < /dev/null
`

// pendantTunnelHeredoc writes content to $1/name on the Deck, with a quoted
// delimiter the content does not contain (nothing in it is expanded).
func pendantTunnelHeredoc(name, content string) string {
	delim := "ROSIE_TUNNEL_EOF"
	for i := 0; strings.Contains(content, delim); i++ {
		delim = "ROSIE_TUNNEL_EOF_" + strconv.Itoa(i)
	}
	return "cat > \"$1/" + name + "\" <<'" + delim + "'\n" + content + delim + "\n"
}

// pendantTunnelScript is the whole Deck session: variables, the rendered
// supervisor and unit (tun_render), then pendantTunnelDeckScript.
func pendantTunnelScript(p pendantTunnelPlan, o pendantTunnelOpts) string {
	follow := "0"
	if o.Follow {
		follow = "1"
	}
	ports := make([]string, 0, len(p.Ports))
	for _, port := range p.Ports {
		ports = append(ports, strconv.Itoa(port))
	}
	vars := []string{
		"TUN_ACTION=" + o.Action, "TUN_UNIT=" + shellQuote(p.Unit), "TUN_DIR=" + shellQuote(p.Dir),
		"TUN_LEGACY=" + shellQuote(strings.Join(p.Deck.Tunnel.Replaces, " ")), "TUN_PORTS=" + shellQuote(strings.Join(ports, " ")),
		"TUN_OLP_PORT=" + strconv.Itoa(p.Deck.Tunnel.OLPPort), "TUN_WAIT=" + strconv.Itoa(int(o.Wait.Seconds())),
		"TUN_APP=" + shellQuote(p.Deck.AppDir), "TUN_LAUNCHER=" + shellQuote(p.Deck.Launcher), "TUN_LINES=" + strconv.Itoa(o.Lines), "TUN_FOLLOW=" + follow,
		"TUN_IDLE_PY=" + shellQuote(pendantTunnelIdlePy), "TUN_PROBE_PY=" + shellQuote(pendantTunnelProbePy),
	}
	render := "tun_render(){\n" + pendantTunnelHeredoc("tunnel.sh", tunnelSupervisorScript(p.Spec)) +
		pendantTunnelHeredoc("unit", p.unitFile()) + "}\n"
	return strings.Join(vars, "\n") + "\n" + render + pendantTunnelDeckScript
}

// --- report -----------------------------------------------------------------

type pendantTunnelProbe struct {
	Unit         map[string]string `json:"unit"`
	UnitFile     bool              `json:"unit_file"`
	FilesCurrent string            `json:"files_current"`
	Legacy       []map[string]any  `json:"legacy"`
	State        *string           `json:"state"`
	Ports        map[string]bool   `json:"ports"`
	Health       struct {
		OK    bool        `json:"ok"`
		MS    int         `json:"ms"`
		Error string      `json:"error"`
		Body  olpUIHealth `json:"body"`
	} `json:"health"`
	Idle    string `json:"idle"`
	Pendant struct {
		Commit     string  `json:"commit"`
		Running    bool    `json:"running"`
		Origin     *string `json:"origin"`
		OriginFrom string  `json:"origin_from"`
	} `json:"pendant"`
	Linger   string   `json:"linger"`
	Replaced []string `json:"replaced"`
	Rollback bool     `json:"rollback"`
}

// pendantTunnelReport prints the probe and returns what is unhealthy.
func pendantTunnelReport(p *rtCoreProgress, plan pendantTunnelPlan, raw []byte) []string {
	problems, pendant := pendantTunnelFaults(p, plan, raw)
	return append(problems, pendant...)
}

// pendantTunnelFaults prints the probe and returns the tunnel's own faults
// and, apart, a pendant whose origin is not this tunnel. Install and restart
// fail only on the first: on a Deck's first deploy the tunnel goes in before
// the pendant install that writes its origin (pendant install needs the
// tunnel's health for its relaunch check), a Steam Deck, 2026-11-03.
func pendantTunnelFaults(p *rtCoreProgress, plan pendantTunnelPlan, raw []byte) (problems, pendant []string) {
	bad := func(phase, msg string) {
		if phase == "pendant" {
			pendant = append(pendant, msg)
		} else {
			problems = append(problems, msg)
		}
		p.Step(phase, "DOWN: "+msg)
	}
	var pr pendantTunnelProbe
	if err := json.Unmarshal(raw, &pr); err != nil || pr.Unit == nil {
		bad("tunnel", fmt.Sprintf("the Deck sent no readable tunnel facts (%v)", err))
		return problems, pendant
	}
	u := pr.Unit
	st := tunnelState{}
	if pr.State != nil {
		st = parseTunnelState(*pr.State)
	}
	var closed []string
	for _, port := range plan.Ports {
		if !pr.Ports[strconv.Itoa(port)] {
			closed = append(closed, strconv.Itoa(port))
		}
	}
	unit := fmt.Sprintf("%s %s/%s (%s), systemd restarts %s", plan.Unit, u["ActiveState"], u["SubState"], u["UnitFileState"], firstNonEmpty(u["NRestarts"], "0"))
	switch {
	case !pr.UnitFile:
		bad("tunnel", plan.Unit+" not installed on "+plan.Deck.Name+" (run pendant tunnel install)")
	case u["ActiveState"] != "active":
		bad("tunnel", fmt.Sprintf("%s; %s", unit, st.lastRestart()))
	case st.Phase != "up" || len(closed) > 0:
		bad("tunnel", fmt.Sprintf("%s; supervisor pid %d phase %s, 127.0.0.1 port(s) %s not listening; %s", unit, st.SupervisorPID, st.Phase, strings.Join(closed, ","), st.lastRestart()))
	default:
		p.Step("tunnel", fmt.Sprintf("up: %s; %s; supervisor pid %d, ssh pid %d, attempt %d up since %s; %s",
			plan.describe(), unit, st.SupervisorPID, st.SSHPID, st.Attempt, st.UpSince, st.lastRestart()))
	}
	if pr.UnitFile && pr.FilesCurrent != "1" {
		p.Step("tunnel", "installed files differ from this checkout's rendering: pendant tunnel install re-renders (while the robot is idle)")
	}
	if pr.Health.OK {
		h := pr.Health.Body
		p.Step("health", fmt.Sprintf("through the tunnel %s: ok, OLP build %s started %s, maintenance notice %v (%d ms)",
			plan.OLPOrigin(), h.BuildID, h.StartedAt, h.Maintenance.Active, pr.Health.MS))
	} else {
		bad("health", fmt.Sprintf("OLP health through the tunnel %s: %s", plan.OLPOrigin(), firstNonEmpty(pr.Health.Error, "no answer")))
	}
	p.Step("robot", pr.Idle)
	running := "not running"
	if pr.Pendant.Running {
		running = "running"
	}
	p.Step("pendant", fmt.Sprintf("package %s, app %s", pr.Pendant.Commit, running))
	from := firstNonEmpty(pr.Pendant.OriginFrom, "pendant.env")
	switch {
	case pr.Pendant.Origin == nil && plan.Deck.Launcher != "":
		bad("pendant", "neither pendant.env nor "+plan.Deck.Launcher+" sets ROSIE_PENDANT_OLP_ORIGIN: the pendant uses the Deck-local OLP, not this tunnel")
	case pr.Pendant.Origin == nil:
		bad("pendant", "pendant.env sets no ROSIE_PENDANT_OLP_ORIGIN: the pendant uses the Deck-local OLP, not this tunnel")
	case *pr.Pendant.Origin != plan.OLPOrigin():
		bad("pendant", fmt.Sprintf("%s ROSIE_PENDANT_OLP_ORIGIN=%s, the tunnel forwards %s", from, *pr.Pendant.Origin, plan.OLPOrigin()))
	default:
		p.Step("pendant", from+" origin "+*pr.Pendant.Origin+" is this tunnel")
	}
	for _, l := range pr.Legacy {
		name, _ := l["unit"].(string)
		file, _ := l["file"].(bool)
		active, _ := l["ActiveState"].(string)
		switch {
		case active == "active" || active == "activating":
			bad("legacy", fmt.Sprintf("hand-made %s is %s beside %s (pendant tunnel install replaces it)", name, active, plan.Unit))
		case file:
			p.Step("legacy", fmt.Sprintf("hand-made %s installed but %s (pendant tunnel install replaces it)", name, firstNonEmpty(active, "inactive")))
		case containsString(pr.Replaced, name):
			p.Step("legacy", fmt.Sprintf("hand-made %s replaced; its unit file is kept in %s/replaced", name, plan.Dir))
		}
	}
	p.Step("deck", fmt.Sprintf("linger %s (the unit starts with deck's login at boot); rollback copy %v", firstNonEmpty(pr.Linger, "unknown"), pr.Rollback))
	return problems, pendant
}

// --- command ----------------------------------------------------------------

// pendantRunDeckTo runs a Deck script with its stdout to out (logs), never
// through the progress printer.
var pendantRunDeckTo = func(d pendantDeck, script string, out io.Writer, p *rtCoreProgress) (int, error) {
	f, err := os.CreateTemp("", "rosie-pendant-deck-*.sh")
	if err != nil {
		return -1, err
	}
	defer os.Remove(f.Name())
	if _, err := f.WriteString(script); err != nil {
		f.Close()
		return -1, err
	}
	if err := f.Close(); err != nil {
		return -1, err
	}
	return cellDeployExec(pendantDeckArgv(d), f.Name(), out, p)
}

func runPendantTunnel(args []string, stdout, stderr io.Writer) error {
	action, rest := splitAction(pendantTunnelActions, args)
	if action == "" {
		if len(args) == 0 || isHelp(args[0]) {
			fmt.Fprint(stdout, pendantTunnelUsage)
			return nil
		}
		return fmt.Errorf("pendant tunnel needs one of %s\n\n%s", strings.Join(pendantTunnelActions, "|"), pendantTunnelUsage)
	}
	var common cellDeployCommonFlags
	var host string
	o := pendantTunnelOpts{Action: action}
	name := "pendant tunnel " + action
	ctx, cfg, restore, err := pendantSetup(name, rest, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&host, "host", "", "Deck name from the manifest's pendant_deploy.decks (required)")
		fs.DurationVar(&o.Wait, "wait", 30*time.Second, "install/restart/rollback: how long to wait for OLP health through the tunnel")
		fs.IntVar(&o.Lines, "lines", 40, "logs: lines of tunnel.log and of the journal")
		fs.BoolVar(&o.Follow, "follow", false, "logs: keep following tunnel.log")
	}, &common, "--host", "--wait", "--lines", "--follow")
	if err != nil {
		return err
	}
	defer restore()
	deck, err := cfg.deck(host)
	if err != nil {
		return err
	}
	plan, err := pendantTunnelResolve(ctx, deck)
	if err != nil {
		return err
	}
	if o.Wait < time.Second || o.Lines < 1 {
		return fmt.Errorf("--wait must be at least 1s and --lines at least 1")
	}
	script := pendantTunnelScript(plan, o)
	if common.DryRun {
		fmt.Fprintf(stdout, "%s cell=%s deck=%s unit=%s (dry run: nothing contacted)\n%s\n# Deck session: %s, stdin: variables, tun_render (below), the action script\n# %s/tunnel.sh\n%s# ~/.config/systemd/user/%s\n%s",
			name, ctx.CellID, deck.Name, plan.Unit, plan.describe(), strings.Join(pendantDeckArgv(deck), " "),
			plan.Dir, tunnelSupervisorScript(plan.Spec), plan.Unit, plan.unitFile())
		return nil
	}
	if err := pendantDeckOnline(deck); err != nil {
		return err
	}
	switch action {
	case "logs":
		code, err := pendantRunDeckTo(deck, script, stdout, newRTCoreProgress(stderr, nil))
		return sessionOutcome(name, code, nil, nil, err)
	case "status":
		p := newRTCoreProgress(stdout, nil)
		p.Step("tunnel", fmt.Sprintf("cell=%s deck=%s %s", ctx.CellID, deck.Name, plan.describe()))
		code, err := pendantRunDeck(deck, script, p)
		if e := sessionOutcome(name, code, nil, nil, err); e != nil {
			return e
		}
		if problems := pendantTunnelReport(p, plan, p.probe); len(problems) > 0 {
			return cellDeployExit(cellDeployExitFailed, "pendant tunnel on %s unhealthy:\n  - %s", deck.Name, strings.Join(problems, "\n  - "))
		}
		p.Step("status", "healthy")
		return nil
	}
	run, err := newCellDeployRun(ctx, name, common.Transport, stdout)
	if err != nil {
		return err
	}
	run.p.Step("tunnel", fmt.Sprintf("deck %s (%s@%s, %s) unit %s: %s", deck.Name, deck.SSHUser, deck.Host, deck.Transport, plan.Unit, plan.describe()))
	run.p.Record(name+" (Deck script)", script)
	run.begin("deck session")
	code, err := pendantRunDeck(deck, script, run.p)
	run.end("deck session", err)
	result := string(run.p.result)
	var problems, pendant []string
	if run.p.probe != nil {
		problems, pendant = pendantTunnelFaults(run.p, plan, run.p.probe)
	}
	if len(pendant) > 0 && (action == "install" || action == "restart") {
		// The tunnel is judged on its own; the origin is pendant install's.
		run.p.Step("pendant", fmt.Sprintf("not yet this tunnel: go run . pendant install --manifest %s --host %s writes ROSIE_PENDANT_OLP_ORIGIN=%s where the Deck's site files name no origin", ctx.Manifest, deck.Name, plan.OLPOrigin()))
	} else {
		problems = append(problems, pendant...)
	}
	switch {
	case err == nil && code == cellDeployExitRestored:
		run.finish("restored", code, result)
		return cellDeployExit(code, "%s failed and the previous tunnel was restored: %s", name, result)
	case err == nil && code == 0 && len(problems) > 0 && (action == "install" || action == "restart"):
		run.finish("failed", cellDeployExitFailed, strings.Join(problems, "; "))
		return cellDeployExit(cellDeployExitFailed, "%s: %s\n  - %s", name, result, strings.Join(problems, "\n  - "))
	}
	if e := sessionOutcome(name, code, run.p.result, run.p.refused, err); e != nil {
		outcome := "failed"
		if code == cellDeployExitRefused {
			outcome = "refused"
		}
		run.finish(outcome, code, e.Error())
		return e
	}
	run.p.Step("done", result)
	run.finish("ok", 0, result)
	return nil
}
