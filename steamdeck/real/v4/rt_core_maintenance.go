package main

import (
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"strconv"
	"strings"
	"time"
)

// The maintenance notice on the OLP host. The relay that every realtime-core
// command already runs on is the host of the cell's OLP
// server, which reads this file (offline-programming/v1 maintenance.go):
// while it exists every browser and Steam Deck shows a blocking "Update in
// progress" overlay with a countdown, and the server refuses arm, jog, Move-to,
// Home, load, play and target selection with 503 "maintenance". Stop, Disarm
// and Disconnect still work; the notice never disarms or stops anything.
//
// deploy, start and config set the notice before they wait for the cell to
// go idle, so operators are told to finish up, update its phase at the major
// steps, and clear it when they end, failures included. --keep-maintenance
// leaves it up for the next step of a session; a notice an operator set with
// `realtime-core maintenance set` is updated but never cleared by a deploy.
// The existing armed/enabled/lease gates are unchanged: the idle wait only
// gives operators time before those gates decide.

const rtCoreMaintenanceFile = "/var/lib/rosie-olp/maintenance.json"

// The servo host's copy, read by rt-control (rt-core adapters/rosie/control
// maintenance.go), which refuses new authority and motion starts with "busy"
// while it exists, for clients that reach the cell without the relay's OLP.
const (
	rtCoreMaintenanceServoFile   = "/etc/rosie-rt-core/maintenance.json"
	rtCoreMaintenanceServoSchema = "rosie.rt-control.maintenance.v1"
)

// Expected run times once the notice is up, the idle wait excluded. The owner's
// estimates (2026-10-07): a configuration-only deploy about 2 min, a full
// deploy session 30-60 min. Start is unverified: its ready wait is 60 s.
const (
	rtCoreMaintenanceConfigEstimate = 2 * time.Minute
	rtCoreMaintenanceDeployEstimate = 45 * time.Minute
	rtCoreMaintenanceStartEstimate  = 5 * time.Minute
	// How long a run waits, with the notice up, for operators to finish, Stop
	// and disarm before the armed/lease gates refuse. Unverified.
	rtCoreMaintenanceIdleWait = 2 * time.Minute
)

// Deploy steps own run ids with this prefix; a later step takes over such a
// kept notice and clears it. Any other live notice belongs to an operator.
const rtCoreMaintenanceRunPrefix = "realtime-core-"

type rtCoreMaintenance struct {
	RunID, Reason, Release, SetBy string
	// ETA counts from when the notice is set and includes IdleWait.
	ETA      time.Duration
	IdleWait time.Duration
	Keep     bool
	// ServoRunner is the relay-script function that runs its stdin as a
	// script on the servo host; set, every write and clear also goes to the
	// servo host's notice. Empty: the relay's notice only.
	ServoRunner string
	// Cell, set, makes the first notice write take the cell lock on the
	// relay (cell_lock.go): one maintenance run per cell at a time. An
	// operator's notice (`realtime-core maintenance set`) takes none.
	Cell string
	// LockInherit is the run id of an outer `realtime-core lock` of this
	// cell that already holds the lock (ROSIE_CELL_LOCK); the run then
	// requires that lock to be held instead of taking it.
	LockInherit string
	// LockWait is how long the first write waits for another run's lock
	// before it refuses (0: refuse at once).
	LockWait time.Duration
	// Progress, set, is a progress (base64 JSON, noticeStaticProgress) every
	// write of this run carries; a run inside another's notice never sets it.
	Progress string
}

type rtCoreMaintenanceFlags struct {
	ETA      time.Duration
	IdleWait time.Duration
	Keep     bool
}

func (f *rtCoreMaintenanceFlags) register(fs *flag.FlagSet, idle bool) {
	fs.DurationVar(&f.ETA, "maintenance-eta", 0, "expected duration shown on the OLP and Steam Deck overlays, counted from when the notice is set (default: the command's estimate plus the idle wait)")
	fs.BoolVar(&f.Keep, "keep-maintenance", false, "leave the maintenance notice up when this command ends, for the next step of a session")
	if idle {
		fs.DurationVar(&f.IdleWait, "idle-wait", rtCoreMaintenanceIdleWait, "with the notice up, wait this long for the controller to be disarmed with no lease before the existing gates refuse (0: refuse at once, no notice for a busy cell)")
	}
}

func (f rtCoreMaintenanceFlags) validate() error {
	if f.ETA != 0 && (f.ETA < time.Minute || f.ETA > 12*time.Hour) {
		return fmt.Errorf("--maintenance-eta must be between 1m and 12h")
	}
	if f.IdleWait < 0 || f.IdleWait > 30*time.Minute || f.IdleWait%time.Second != 0 {
		return fmt.Errorf("--idle-wait must be whole seconds between 0s and 30m")
	}
	return nil
}

func newRTCoreMaintenance(command, cell string, f rtCoreMaintenanceFlags, estimate time.Duration) rtCoreMaintenance {
	m := rtCoreMaintenance{
		RunID:    rtCoreMaintenanceRunPrefix + command + "-" + time.Now().UTC().Format("20060102T150405Z") + "-" + cell,
		Reason:   map[string]string{"deploy": "rt-core full deploy", "start": "rt-core controller start", "config": "rt-core configuration deploy"}[command] + " on " + cell,
		SetBy:    "realtime-core " + command + " (" + rtCoreMaintenanceOperator() + ")",
		ETA:      f.ETA,
		IdleWait: f.IdleWait,
		Keep:     f.Keep,
		Cell:     cell,
	}
	m.LockInherit = cellLockInherited(cell)
	if m.ETA == 0 {
		m.ETA = f.IdleWait + estimate
	}
	if repo, err := repoRoot(); err == nil {
		if head, err := robotV4GitOutput(repo, "rev-parse", "--short=12", "HEAD"); err == nil {
			m.Release = strings.TrimSpace(head)
		}
	}
	return m
}

func rtCoreMaintenanceOperator() string {
	user := firstNonEmpty(os.Getenv("USER"), "unknown")
	host, _ := os.Hostname()
	return user + "@" + firstNonEmpty(strings.TrimSuffix(host, ".local"), "unknown")
}

// rtCoreMaintenanceWriterPy writes the notice JSON to stdout. A live notice
// keeps its start (elapsed time stays true across steps); an operator's
// notice keeps its run id, so this run will not clear it. The ETA is the
// notice owner's: its own later writes keep it unless they re-estimate it
// (eta_now=1, a release after each step), and a step inside another run's
// notice (a cell lock's or an operator's) never moves it -- on 2026-10-10 a
// release's inner steps would each have reset the lock's ETA to their own
// fixed estimate (deploy: 47 min). The progress (notice_progress.go, base64
// JSON) follows the same ownership: a write that carries one replaces it
// (keeping updated_at when nothing changed), any other write by the owner or
// inside its notice keeps it; its step line becomes the phase when the write
// names none. An empty phase keeps the current one. The
// notice names the process that owns it (host, pid, run id, cell lock) so
// `realtime-core maintenance show` can flag one whose run is gone; an adopted
// notice keeps its owner.
const rtCoreMaintenanceWriterPy = `import base64, datetime, json, socket, sys
path, run, prefix, reason, phase, eta_s, set_by, release = sys.argv[1:9]
owner_pid, lock, eta_now, progress_b64 = (sys.argv[9:13] + ["", "", "", ""])[:4]
now = datetime.datetime.now(datetime.timezone.utc).replace(microsecond=0)
fmt = lambda t: t.strftime("%Y-%m-%dT%H:%M:%SZ")
try:
    old = json.load(open(path))
except (OSError, ValueError):
    old = {}
if not isinstance(old, dict):
    old = {}
live = old.get("active", True) is not False and bool(old.get("started_at"))
owner = run
if live and old.get("run_id") and old.get("run_id") != run and not str(old.get("run_id")).startswith(prefix):
    owner = old["run_id"]
eta = fmt(now + datetime.timedelta(seconds=int(eta_s)))
owned = live and (old.get("run_id") == run or owner != run)
if owned and old.get("eta") and eta_now != "1":
    eta = old["eta"]
progress = None
if progress_b64 and owner == run:
    try:
        progress = json.loads(base64.b64decode(progress_b64, validate=True))
    except Exception:
        progress = None
    if not isinstance(progress, dict):
        progress = None
kept = old.get("progress") if owned and isinstance(old.get("progress"), dict) else None
if progress is not None:
    step_line = str(progress.pop("phase", "") or "")
    if not phase:
        phase = step_line
    progress["updated_at"] = fmt(now)
    if kept is not None and {k: v for k, v in kept.items() if k != "updated_at"} == {k: v for k, v in progress.items() if k != "updated_at"}:
        progress["updated_at"] = kept.get("updated_at", progress["updated_at"])
else:
    progress = kept
if live and not phase:
    phase = old.get("phase") or ""
notice = {"schema": "rosie.olp.maintenance.v1", "active": True, "reason": reason, "phase": phase,
          "started_at": old["started_at"] if live else fmt(now), "eta": eta, "set_by": set_by,
          "release": release, "run_id": owner, "updated_at": fmt(now)}
if progress is not None:
    notice["progress"] = progress
if owner == run and owner_pid:
    notice["owner"] = {"host": socket.gethostname(), "pid": int(owner_pid), "run_id": run, "lock": lock}
elif live and isinstance(old.get("owner"), dict):
    notice["owner"] = old["owner"]
print(json.dumps(notice, indent=2))`

// rtCoreMaintenanceOwnerPy reports, read-only, whether the process that set a
// notice still runs: "owner: ... running", "STALE: ..." when its pid is gone
// or the cell lock it held is free, or "owner: none recorded". It never
// clears anything: an operator checks the cell and runs maintenance clear.
const rtCoreMaintenanceOwnerPy = `import json, os, socket, subprocess, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    print("owner: the notice is not readable JSON (it still blocks arming)")
    sys.exit()
o = d.get("owner") if isinstance(d, dict) else None
if not isinstance(o, dict) or not o.get("pid"):
    print("owner: none recorded (run %s: an operator's notice or an older tool's); it stays until realtime-core maintenance clear" % d.get("run_id"))
    sys.exit()
run, host, pid, lock = o.get("run_id"), o.get("host"), int(o.get("pid")), o.get("lock") or ""
if host != socket.gethostname():
    print("owner: run %s pid %d on %s (another host; not checked here)" % (run, pid, host))
    sys.exit()
try:
    os.kill(pid, 0)
    alive = True
except ProcessLookupError:
    alive = False
except PermissionError:
    alive = True
held = None
if lock:
    try:
        held = subprocess.run(["flock", "-n", lock, "true"], capture_output=True).returncode != 0
    except OSError:
        held = None
if alive and held is not False:
    print("owner: run %s pid %d on %s is running%s" % (run, pid, host, "; it holds the cell lock" if held else ""))
else:
    print("STALE: the run that set this notice (%s, pid %d on %s) is gone%s; nothing clears a notice by itself: check the cell and that run's journal, then realtime-core maintenance clear" % (run, pid, host, "" if held is None else " and the cell lock is free"))`

// rtCoreBusyPy defines rosie_busy(core), the one predicate every host script
// uses for "an operator holds the controller": armed, any axis enabled, or a
// valid lease, read from rtctl status's core object (Go: cellDeployStatusFacts
// .busy). A missing field counts as busy, so a status a script cannot read
// never lets a stop, restart or reboot through. It was six inline variants,
// two of which read a missing field as idle.
const rtCoreBusyPy = `def rosie_busy(c):
    return any(int(c.get(k, 1) or 0) != 0 for k in ("armed", "axis_enable_mask")) or bool(c.get("lease_valid", True))
`

// rtCoreMaintenanceIdleScript runs on the servo host: one line per check,
// "idle ..." (then exit 0) or "busy ..." / "unknown ...", every 2 s until the
// deadline. Read-only: rtctl status, as every probe already reads it.
const rtCoreMaintenanceIdleScript = `set -u
deadline=$((SECONDS + @IDLE_S@))
while :; do
  if [ "$(systemctl show rosie-rt-control.service -p ActiveState --value 2>/dev/null)" != active ]; then echo "idle controller adapter not running"; exit 0; fi
  s=$(sudo -n /opt/rosie-rt-core/current/bin/rtctl status --socket /run/rosie-rt-core/control.sock --json 2>/dev/null | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
    c = d.get("data", d)["core"]
except Exception:
    print("unknown status unavailable")
    sys.exit()
facts = " ".join("%s=%s" % (k, c.get(k)) for k in ("armed", "axis_enable_mask", "lease_valid"))
` + rtCoreBusyPy + `print(("busy " if rosie_busy(c) else "idle ") + facts)')
  # The holder rt-control journals when a grant changes hands (rt-core with the
  # maintenance notice; older releases log nothing, so nothing is shown).
  holder=$(sudo -n journalctl -u rosie-rt-control.service -n 400 --no-pager -o cat 2> /dev/null | grep -E 'rt-control: grant_(acquired|released|expired|revoked)' | tail -n 1)
  case "$holder" in *grant_acquired*) holder=" lease holder:${holder#*grant_acquired}" ;; *) holder="" ;; esac
  echo "${s:-unknown status unavailable}${holder}"
  case "$s" in idle*) exit 0 ;; esac
  [ "$SECONDS" -ge "$deadline" ] && exit 3
  sleep 2
done`

// rtCoreMaintenanceWhoPy prints one line per client of the relay's OLP (its
// GET .../maintenance clients and machine), for the deployer to know who to
// wait for. Labels are the OLP's own: unknown clients stay unknown.
const rtCoreMaintenanceWhoPy = `import datetime, json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit()
now = datetime.datetime.now(datetime.timezone.utc)
def ago(t):
    try:
        return "%ds ago" % (now - datetime.datetime.strptime(t, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=datetime.timezone.utc)).total_seconds()
    except Exception:
        return "at an unknown time"
m = d.get("machine") or {}
if m:
    print("olp machine %s armed=%s lease=%s session=%s" % (m.get("cell"), m.get("armed"), m.get("lease_valid"), m.get("session_state") or "none"))
for c in d.get("clients") or []:
    parts = ["olp client", c.get("label") or "unknown", "[%s]" % c.get("kind"), "from %s%s" % (c.get("address"), " (this host or a tunnel)" if c.get("loopback") else ""), "seen " + ago(c.get("last_seen"))]
    if c.get("last_action"):
        parts.append("last robot request %s %s" % (c["last_action"], ago(c.get("last_action_at"))))
    if c.get("session_holder"):
        parts.append("HOLDS the execution session")
    print(" ".join(parts))`

// rtCoreMaintenanceShell defines, for a relay script: maint_set PHASE (fails
// when the notice cannot be written), maint_phase PHASE (never fails the
// run), maint_clear (this run's notice only, nothing with --keep-maintenance)
// and maint_wait_idle RUNNER (RUNNER runs its stdin as a script on the servo
// host).
func rtCoreMaintenanceShell(m rtCoreMaintenance) string {
	keep := "0"
	if m.Keep {
		keep = "1"
	}
	lock := ""
	if m.Cell != "" {
		lock = cellLockFile(m.Cell)
	}
	return rtCoreConfigReplace(`MAINT_FILE=@FILE@; MAINT_RUN=@RUN@; MAINT_REASON=@REASON@; MAINT_RELEASE=@RELEASE@
MAINT_SET_BY=@SET_BY@; MAINT_ETA_S=@ETA_S@; MAINT_IDLE_S=@IDLE_S@; MAINT_KEEP=@KEEP@; MAINT_PROGRESS=@PROGRESS@
MAINT_SERVO_FILE=@SERVO_FILE@; MAINT_SERVO_RUNNER=@SERVO_RUNNER@
MAINT_LOCK=@LOCK@; MAINT_LOCK_INHERIT=@LOCK_INHERIT@; MAINT_LOCK_WAIT_S=@LOCK_WAIT_S@
maint_say(){ if type step > /dev/null 2>&1; then step "maintenance $*"; else printf 'maintenance: %s\n' "$*"; fi; }
maint_holder(){ head -n 1 "$MAINT_LOCK.owner" 2> /dev/null || echo "an unknown run"; }
maint_lock(){ # the cell lock (fd 7, held until this script exits): one maintenance run per cell
  [ -n "$MAINT_LOCK" ] && [ "${MAINT_LOCKED:-0}" = 0 ] || return 0
  command -v flock > /dev/null 2>&1 || { maint_say "flock not found on $(hostname): the cell lock cannot be taken"; return 1; }
  mkdir -p "$(dirname "$MAINT_LOCK")" || return 1
  if [ -n "$MAINT_LOCK_INHERIT" ]; then
    if flock -n "$MAINT_LOCK" true; then maint_say "the cell lock of $MAINT_LOCK_INHERIT is not held on $(hostname); refusing"; return 1; fi
    MAINT_LOCKED=inherited; maint_say "running under the cell lock of $MAINT_LOCK_INHERIT"; return 0
  fi
  exec 7>> "$MAINT_LOCK" || return 1
  if ! flock -n 7; then
    maint_say "the cell is locked by $(maint_holder); waiting up to ${MAINT_LOCK_WAIT_S}s"
    if [ "$MAINT_LOCK_WAIT_S" = 0 ] || ! flock -w "$MAINT_LOCK_WAIT_S" 7; then
      maint_say "the cell is locked by $(maint_holder); refusing"; exec 7>&-; return 1
    fi
  fi
  printf '%s pid %s on %s: %s (%s) since %s\n' "$MAINT_RUN" "$$" "$(hostname)" "$MAINT_REASON" "$MAINT_SET_BY" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" > "$MAINT_LOCK.owner"
  MAINT_LOCKED=1; maint_say "cell lock taken ($MAINT_LOCK)"
}
maint_owner(){ # $1 notice file: is the run that set it still alive (read-only)
  python3 - "$1" <<'ROSIE_MAINT_OWNER_PY'
@OWNER_PY@
ROSIE_MAINT_OWNER_PY
}
maint_servo_write(){ # $1: the relay notice; the servo host gets the same notice for rt-control
  local b64
  b64=$(python3 -c 'import json, sys
d = json.load(open(sys.argv[1]))
d["schema"] = "@SERVO_SCHEMA@"
sys.stdout.write(json.dumps(d))' "$1" | base64 | tr -d '\n') || return 1
  printf 'set -eu\nt=$(mktemp)\nprintf %%s %s | base64 -d > "$t"\n[ -d "$(dirname %s)" ] || sudo -n install -d -m 0755 "$(dirname %s)"\nnotice_tmp=%s.tmp.${t##*/}\nsudo -n install -m 0644 "$t" "$notice_tmp"\nsudo -n mv -f "$notice_tmp" %s\nrm -f "$t"\n' \
    "$b64" "$MAINT_SERVO_FILE" "$MAINT_SERVO_FILE" "$MAINT_SERVO_FILE" "$MAINT_SERVO_FILE" | "$MAINT_SERVO_RUNNER"
}
maint_set(){
  local tmp notice_tmp
  maint_lock || return 1
  tmp=$(mktemp) || return 1
  notice_tmp="$MAINT_FILE.tmp.${tmp##*/}"
  if python3 - "$MAINT_FILE" "$MAINT_RUN" @PREFIX@ "$MAINT_REASON" "$1" "$MAINT_ETA_S" "$MAINT_SET_BY" "$MAINT_RELEASE" "$$" "$MAINT_LOCK" "${MAINT_ETA_NOW:-0}" "${MAINT_PROGRESS:-}" > "$tmp" <<'ROSIE_MAINT_PY' &&
@WRITER@
ROSIE_MAINT_PY
    sudo -n install -d -m 0755 "$(dirname "$MAINT_FILE")" &&
    sudo -n install -m 0644 "$tmp" "$notice_tmp" && sudo -n mv -f "$notice_tmp" "$MAINT_FILE" &&
    { [ -z "$MAINT_SERVO_RUNNER" ] || { maint_servo_write "$tmp" && MAINT_SERVO_WRITTEN=1; }; }; then
    rm -f "$tmp"
    maint_say "notice on $(hostname)${MAINT_SERVO_RUNNER:+ and the servo host}: ${1:-ETA re-estimated to ${MAINT_ETA_S}s from now}"
    return 0
  fi
  rm -f "$tmp"
  return 1
}
maint_phase(){ maint_set "$1" || maint_say "could not update the notice ($1)"; }
# Inside a cell lock the notice is the lock run's, which clears it when it
# ends; calling it an operator's invited a hand clear mid-release.
maint_owner_words(){
  if [ -n "${MAINT_LOCK_INHERIT:-}" ]; then echo "belongs to the cell lock run $MAINT_LOCK_INHERIT, which clears it when it ends"
  else echo "belongs to another run (an operator's)"; fi
}
maint_servo_clear(){
  local out
  out=$(mktemp) || return 0
  if "$MAINT_SERVO_RUNNER" > "$out" 2>&1 <<'ROSIE_MAINT_SERVO_CLEAR'
F=@SERVO_FILE@
if python3 -c 'import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    sys.exit(1)
sys.exit(0 if isinstance(d, dict) and d.get("run_id") == sys.argv[2] else 1)' "$F" @RUN@; then sudo -n rm -f "$F" && echo cleared
elif [ -e "$F" ]; then echo kept
else echo absent
fi
ROSIE_MAINT_SERVO_CLEAR
  then
    case "$(tail -n 1 "$out")" in
      cleared) maint_say "servo host notice cleared" ;;
      kept) maint_say "servo host notice $(maint_owner_words); left in place" ;;
    esac
  else
    maint_say "could not clear the servo host notice ($(tail -n 1 "$out")); clear it with realtime-core maintenance clear"
  fi
  rm -f "$out"
}
maint_clear(){
  if [ "$MAINT_KEEP" = 1 ]; then maint_say "notice left up (--keep-maintenance); the next step takes it over, or: realtime-core maintenance clear"; return 0; fi
  if python3 -c 'import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    sys.exit(1)
sys.exit(0 if isinstance(d, dict) and d.get("run_id") == sys.argv[2] else 1)' "$MAINT_FILE" "$MAINT_RUN"; then
    sudo -n rm -f "$MAINT_FILE" && maint_say "notice cleared"
  elif [ -e "$MAINT_FILE" ]; then
    maint_say "notice $(maint_owner_words); left in place"
  fi
  if [ -n "$MAINT_SERVO_RUNNER" ] && [ "${MAINT_SERVO_WRITTEN:-0}" = 1 ]; then maint_servo_clear; fi
}
maint_who(){ # who is connected, from the relay's OLP (needs its maintenance route)
  curl -fsS -m 2 "http://${OLP_LISTEN:-127.0.0.1:8794}/api/offline-programming/v1/maintenance" 2> /dev/null | python3 -c '@WHO@' | while IFS= read -r who; do maint_say "$who"; done
}
maint_wait_idle(){
  [ "$MAINT_IDLE_S" -gt 0 ] || return 0
  maint_say "waiting up to ${MAINT_IDLE_S}s for the controller to be disarmed with no lease"
  "$@" <<'ROSIE_MAINT_IDLE' | while IFS= read -r line; do
@IDLE@
ROSIE_MAINT_IDLE
    case "$line" in
      idle*) maint_say "cell idle (${line#idle })"; maint_who; maint_phase "cell idle; continuing" ;;
      *) maint_say "cell busy (${line#* })"; maint_who
        if [ "$line" != "${prev:-}" ]; then maint_phase "waiting for operators to finish: Stop, disarm and release (${line#* })"; prev=$line; fi ;;
    esac
  done || true
}
`, "@FILE@", rtCoreMaintenanceFile, "@RUN@", shellQuote(m.RunID), "@REASON@", shellQuote(m.Reason), "@RELEASE@", shellQuote(m.Release),
		"@SET_BY@", shellQuote(m.SetBy), "@ETA_S@", strconv.Itoa(int(m.ETA.Seconds())), "@IDLE_S@", strconv.Itoa(int(m.IdleWait.Seconds())), "@PROGRESS@", shellQuote(m.Progress),
		"@KEEP@", keep, "@PREFIX@", shellQuote(rtCoreMaintenanceRunPrefix), "@WRITER@", rtCoreMaintenanceWriterPy,
		"@SERVO_FILE@", rtCoreMaintenanceServoFile, "@SERVO_RUNNER@", m.ServoRunner, "@SERVO_SCHEMA@", rtCoreMaintenanceServoSchema,
		"@WHO@", rtCoreMaintenanceWhoPy, "@OWNER_PY@", rtCoreMaintenanceOwnerPy,
		"@LOCK@", firstNonEmpty(lock, "''"), "@LOCK_INHERIT@", shellQuote(m.LockInherit), "@LOCK_WAIT_S@", strconv.Itoa(int(m.LockWait.Seconds())),
		"@IDLE@", strings.ReplaceAll(rtCoreMaintenanceIdleScript, "@IDLE_S@", strconv.Itoa(int(m.IdleWait.Seconds()))))
}

// rtCoreMaintenanceServoFunc defines maint_servo: a script on stdin, run on
// the servo host ($MAINT_SERVO) from the relay.
var rtCoreMaintenanceServoFunc = `maint_servo(){ ssh -o BatchMode=yes ` + sshAliveShell(sshConnectTimeout) +
	` -- "$MAINT_SERVO" 'f=$(mktemp) && cat > "$f" && bash "$f" < /dev/null; rc=$?; rm -f "$f"; exit $rc'; }` + "\n"

// rtCoreMaintenanceWrap runs a relay script under the notice: set it before
// anything else (refusing, with nothing changed, if it cannot be written),
// clear it on every exit, wait for the servo host to be idle, then body.
// body's maint_phase calls update the phase.
func rtCoreMaintenanceWrap(m rtCoreMaintenance, servoTarget, firstPhase, body string) string {
	m.ServoRunner = "maint_servo"
	// The trap is armed before the first write, so a notice written on one
	// host before the other refused is cleared again.
	return "set -eu\n" + rtCoreMaintenanceShell(m) +
		"MAINT_SERVO=" + shellQuote(servoTarget) + "\n" +
		rtCoreMaintenanceServoFunc +
		`trap 'rc=$?; trap - EXIT; maint_clear; exit $rc' EXIT` + "\n" +
		`trap 'exit 129' HUP; trap 'exit 130' INT; trap 'exit 143' TERM` + "\n" +
		"maint_set " + shellQuote(firstPhase) + ` || { printf 'maintenance notice could not be written on %s or the servo host; nothing was changed\n' "$(hostname)" >&2; exit 2; }` + "\n" +
		"maint_wait_idle maint_servo\n" + body
}

// rtCoreMaintenanceShowBody prints both notices, whether the run that set the
// relay's is still alive (STALE when it is gone), a servo-host notice that
// does not match the relay's, and who is connected. Read-only: a stale notice
// is reported, never cleared.
const rtCoreMaintenanceShowBody = `if [ -e "$MAINT_FILE" ]; then cat "$MAINT_FILE"; maint_owner "$MAINT_FILE" | while IFS= read -r l; do maint_say "$l"; done
else maint_say "no notice on $(hostname)"; fi
servo_notice=$(printf 'if [ -e %s ]; then cat %s; fi\n' "$MAINT_SERVO_FILE" "$MAINT_SERVO_FILE" | maint_servo) || maint_say "could not read the servo host notice"
if [ -n "$servo_notice" ]; then printf '%s\n' "$servo_notice"; else maint_say "no notice on the servo host"; fi
printf '%s' "$servo_notice" | python3 -c 'import json, sys
def run(text):
    try:
        return json.loads(text).get("run_id") or "?"
    except Exception:
        return "?" if text.strip() else None
try:
    relay = run(open(sys.argv[1]).read())
except OSError:
    relay = None
servo = run(sys.stdin.read())
if servo and not relay:
    print("ORPHAN: the servo host has a notice (run %s) but this host has none; rt-control refuses arming until realtime-core maintenance clear" % servo)
elif servo and relay and servo != relay:
    print("MISMATCH: relay notice run %s, servo host notice run %s" % (relay, servo))
elif relay and not servo:
    print("the servo host has no notice: clients that reach rt-control without this OLP are not refused")' "$MAINT_FILE" | while IFS= read -r l; do maint_say "$l"; done
maint_who
`

// runRealtimeCoreMaintenance is `realtime-core maintenance set|clear|show`:
// an operator's notice for a session of several steps, or clearing one a lost
// connection left behind. It runs on the relay like config.
func runRealtimeCoreMaintenance(args []string, stdout, stderr io.Writer) error {
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, `usage: realtime-core maintenance <set|clear|show> --manifest cell.json [flags]

  set    --reason TEXT [--eta 30m] [--phase TEXT] [--transport tailscale|ssh] [--dry-run]
  clear  [--transport tailscale|ssh] [--dry-run]   (any notice, whoever set it)
  show   [--transport tailscale|ssh]   (both notices; STALE when the run that set
         it is gone, ORPHAN for a servo-host notice without a relay one; never clears)

The notice is `+rtCoreMaintenanceFile+` on the cell's relay (its OLP host).
While it exists the OLP refuses robot actions and every client shows a
blocking overlay; Stop, Disarm and Disconnect still work.
`)
		return nil
	}
	action := args[0]
	fs := flag.NewFlagSet("realtime-core maintenance "+action, flag.ContinueOnError)
	fs.SetOutput(stderr)
	manifest := fs.String("manifest", "", "approved native cell config")
	transport := fs.String("transport", "tailscale", "how the Mac reaches the relay: tailscale or ssh")
	dryRun := fs.Bool("dry-run", false, "print the relay script without contacting the relay")
	driveTransition := fs.Bool("drive-transition", false, "set the persistent marker for a stopped drive-mode transition")
	reason := fs.String("reason", "", "what is being updated (shown to operators)")
	phase := fs.String("phase", "operator maintenance", "the current step (shown to operators)")
	eta := fs.Duration("eta", 30*time.Minute, "expected duration from now")
	if err := rejectRetiredRealtimeCoreFlags(args[1:], "--manifest", "--transport", "--dry-run", "--drive-transition", "--reason", "--phase", "--eta"); err != nil {
		return err
	}
	if err := fs.Parse(args[1:]); err != nil {
		return err
	}
	if fs.NArg() != 0 || (*transport != "tailscale" && *transport != "ssh") {
		return errors.New("realtime-core maintenance takes no positional arguments and --transport tailscale|ssh")
	}
	restore, err := selectRealtimeCoreManifest(*manifest)
	if err != nil {
		return err
	}
	defer restore()
	var transitionCell string
	if *driveTransition {
		ctx, err := loadCellDeployContext()
		if err != nil {
			return err
		}
		if action != "set" || *reason != "stopped drive mode transition for "+ctx.CellID {
			return errors.New("--drive-transition requires maintenance set with the exact cell transition reason")
		}
		transitionCell = ctx.CellID
	}
	_, target, via, viaPeer, err := rtCoreRelayRoute()
	if err != nil {
		return err
	}
	relayUser, relayHost, _ := strings.Cut(via, "@")
	if relayHost == "" {
		relayUser, relayHost = "", via
	}
	m := rtCoreMaintenance{RunID: "operator-" + time.Now().UTC().Format("20060102T150405Z"), Reason: *reason,
		SetBy: "realtime-core maintenance (" + rtCoreMaintenanceOperator() + ")", ETA: *eta, ServoRunner: "maint_servo"}
	if *driveTransition {
		m.RunID = "drive-transition-" + transitionCell + "-" + time.Now().UTC().Format("20060102T150405Z")
	}
	var body string
	switch action {
	case "set":
		if strings.TrimSpace(*reason) == "" || *eta < time.Minute || *eta > 12*time.Hour {
			return errors.New("maintenance set requires --reason and an --eta between 1m and 12h")
		}
		if repo, err := repoRoot(); err == nil {
			if head, err := robotV4GitOutput(repo, "rev-parse", "--short=12", "HEAD"); err == nil {
				m.Release = strings.TrimSpace(head)
			}
		}
		body = "maint_set " + shellQuote(*phase) + " || exit 2\n"
	case "clear":
		// Any notice, whoever set it, on both hosts.
		body = `sudo -n rm -f "$MAINT_FILE" && maint_say "notice cleared on $(hostname)"
printf 'sudo -n rm -f %s && echo cleared\n' "$MAINT_SERVO_FILE" | maint_servo > /dev/null && maint_say "notice cleared on the servo host" || maint_say "could not reach the servo host to clear its notice"
`
	case "show":
		body = rtCoreMaintenanceShowBody
	default:
		return fmt.Errorf("unknown realtime-core maintenance command %q", action)
	}
	script := "set -u\n" + rtCoreMaintenanceShell(m) + "MAINT_SERVO=" + shellQuote(target) + "\n" + rtCoreMaintenanceServoFunc + body
	fmt.Fprintf(stdout, "maintenance %s relay=%s file=%s, servo host %s file=%s\n", action, remoteTarget(relayUser, viaPeer), rtCoreMaintenanceFile, target, rtCoreMaintenanceServoFile)
	if *dryRun {
		fmt.Fprintln(stdout, script)
		return nil
	}
	p := newRTCoreProgress(stdout, nil)
	code, err := rtCoreConfigRunRemote(rtCoreConfigRemoteArgv(*transport, relayUser, viaPeer, relayHost, script), p)
	if err != nil || code != 0 {
		return fmt.Errorf("maintenance %s failed on the relay (exit %d, %v)", action, code, err)
	}
	return nil
}
