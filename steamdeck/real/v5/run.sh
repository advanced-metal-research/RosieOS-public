#!/usr/bin/env bash
# Steam shortcut entry point. The pendant is the native Qt app in bin/; its
# backend is an OLP server, which owns the machine session. By default that is
# the server beside it on loopback, and closing the pendant stops it, whose
# shutdown Stops any machine it armed before releasing authority. With
# ROSIE_PENDANT_OLP_ORIGIN (site file pendant.env) the backend is a cell host's
# OLP server, deployed with its cell, and closing the pendant sends that server
# Stop instead, except while fresh server observations prove an unattended
# program loop owns the machine. Nothing here arms, homes or moves.
#
# One pendant per Deck user (README "One pendant per Deck"): every pendant
# reads the same controller, and on 2026-10-11 a hidden pendant for one arm
# armed it with presses meant for a visible one for another arm. This launch takes the
# user's pendant lock for its whole life, its exit Stop included; a launch
# that finds the lock held asks that launch to quit (its app closes, its EXIT
# trap sends its Stop) and starts once it has. ROSIE_PENDANT_EXPECT_CELL (the
# launcher's, or pendant.env's) is the cell this launch is for: the app
# refuses to arm or move while its OLP server reports another.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
base=$PWD
unit=rosie-v5-olp
origin=http://127.0.0.1:8794
if [[ -f "$base/pendant.env" ]]; then
  set -a
  # shellcheck source=/dev/null
  . "$base/pendant.env"
  set +a
fi
[[ -z "${ROSIE_PENDANT_OLP_ORIGIN:-}" ]] || origin=$ROSIE_PENDANT_OLP_ORIGIN
expect=${ROSIE_PENDANT_EXPECT_CELL:-}

runtime=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}
lock=$runtime/rosie-pendant-v5.lock
holder=$runtime/rosie-pendant-v5.holder
app_pattern='bin/rosie-pendant-v[5]'
say(){ printf 'run.sh: %s\n' "$*" >&2; }
# Unverified bounds: 25 s covers the previous launch's close and its exit
# Stop (5 s, below) or local server stop (25 s TimeoutStopSec, rarely
# reached); 15 s for a pendant started outside the lock to exit on SIGTERM.
takeover_wait=${ROSIE_PENDANT_TAKEOVER_WAIT:-25}
stray_wait=${ROSIE_PENDANT_STRAY_WAIT:-15}
exec 9>> "$lock"
if ! flock -n 9; then
  read -r old_pid old_cell old_origin < "$holder" 2> /dev/null || true
  say "another pendant launch holds $lock (run.sh pid ${old_pid:-?}, cell ${old_cell:--}, OLP ${old_origin:-?}); asking it to quit"
  if [[ ${old_pid:-} =~ ^[0-9]+$ ]] && tr '\0' ' ' < "/proc/$old_pid/cmdline" 2> /dev/null | grep -q 'run\.sh'; then
    # Its app first: the launch waits on it, then runs its EXIT trap.
    pkill -TERM -P "$old_pid" -f "$app_pattern" 2> /dev/null || true
    kill -TERM "$old_pid" 2> /dev/null || true
  fi
  if ! flock -w "$takeover_wait" 9; then
    say "refused: the pendant launch that holds $lock (run.sh pid ${old_pid:-?}) did not quit within ${takeover_wait} s; quit it from Steam, then start this one again"
    exit 3
  fi
  say "the previous pendant launch quit; starting this one"
fi
printf '%s %s %s\n' "$$" "${expect:--}" "$origin" > "$holder"
# Pendants started outside the lock (an older package, another app directory):
# asked to quit the same way, so their own launch sends its Stop.
mapfile -t strays < <(pgrep -u "$(id -u)" -f "$app_pattern" || true)
if (( ${#strays[@]} )); then
  say "closing ${#strays[@]} pendant(s) started outside the lock: $(for p in "${strays[@]}"; do tr '\0' ' ' < "/proc/$p/cmdline" 2> /dev/null | cut -c1-160; printf '(pid %s); ' "$p"; done)"
  kill -TERM "${strays[@]}" 2> /dev/null || true
  for _ in $(seq $(( stray_wait * 5 ))); do pgrep -u "$(id -u)" -f "$app_pattern" > /dev/null || break; sleep 0.2; done
  if pgrep -u "$(id -u)" -f "$app_pattern" > /dev/null; then
    say "refused: pendant(s) $(pgrep -u "$(id -u)" -f "$app_pattern" | tr '\n' ' ')did not quit within ${stray_wait} s; quit them from Steam, then start this one again"
    exit 3
  fi
fi
# Steam's injected library path selects an incompatible libcurl; drop it.
curl_clean() { env -u LD_LIBRARY_PATH -u LD_PRELOAD curl "$@"; }

# A running unattended loop belongs to the cell, not the departing pendant.
# Never infer this from a saved UI setting or a previous Run reply: one fresh
# read decides. OLP answers status from a live rt-core status call (any failed
# call sets rt_core.reason), and the native core time must lie within 1 s of
# the native host clock in the same answer, so one read is current evidence.
# Absent Python, silence, an attended loop (every loop on a cell whose
# manifest does not set unattended_loop_allowed), stale or malformed status
# all keep the exit Stop. The 2 s HTTP bound and the 1 s native-age check are
# unverified cleanup budgets, not motion deadlines. python3 is the same
# reader deck-install.sh uses for its idle check.
unattended_loop_active() {
  [[ -n "$expect" ]] || return 1
  local observed
  observed=$(curl_clean -sf -m 2 -H 'Cache-Control: no-cache' "$origin/api/offline-programming/v1/dense-execution/status") || return 1
  printf '%s' "$observed" | python3 -c '
import json, sys
try:
    s = json.load(sys.stdin)
    target, loop, session, rt = s["target"], s["loop"], s["session"], s["rt_core"]
    status, core = rt["status"], rt["status"]["core"]
    native, now = core["time_ns"], status["grant"]["NowHostNs"]
    valid = (s["available"] is True and s["armed"] is True and target["cell_id"] == sys.argv[1]
             and target["selection_generation"] and status["daemon_incarnation"] and status["adapter_incarnation"]
             and not s.get("daemon_error") and rt["inhibited"] is False and rt["stop_uncertain"] is False
             and not rt.get("reason") and loop["attended"] is False and loop["mode"] in ("count", "loop")
             and loop["state"] in ("running", "finishing") and not loop.get("end_reason")
             and loop["trajectory_digest"] and loop["trajectory_digest"] == session["trajectory_digest"]
             and session["lease"]["held"] is True and session["state"] in ("playing", "loaded", "done")
             and core["armed"] == 1 and core["lease_valid"] == 1 and core["safety_fault_mask"] == 0
             and all(type(v) is int and v > 0 for v in (status["time_ns"], native, now))
             and 0 <= now - native <= 1000000000)
    sys.exit(0 if valid else 1)
except (ValueError, KeyError, TypeError, IndexError, AttributeError):
    sys.exit(1)
' "$expect" 2>/dev/null
}

# TERM may arrive after Bash has entered EXIT but before its first command.
# Exiting from that TERM handler does not run EXIT again. Both handlers must
# perform the same cleanup explicitly; ignoring later TERM and the guard keep
# a repeated quit from truncating or duplicating the bounded Stop.
exit_cleanup_started=0
exit_cleanup() {
  trap '' TERM
  (( exit_cleanup_started == 0 )) || return 0
  exit_cleanup_started=1
  if [[ -n "${ROSIE_PENDANT_OLP_ORIGIN:-}" ]]; then
    if unattended_loop_active; then
      say "unattended program loop remains running on $expect; STOP or FINISH CYCLE is available from a connected client"
      return 0
    fi
    # The remote OLP outlives this app. Unverified bound: 5 s over the tunnel.
    curl_clean -sf -m 5 -o /dev/null -X POST -H "Content-Type: application/json" -d "{}" "$origin/api/offline-programming/v1/dense-execution/stop" || true
  else
    systemctl --user stop "$unit"
  fi
}
trap exit_cleanup EXIT
trap 'exit_cleanup; exit 143' TERM

if [[ -z "${ROSIE_PENDANT_OLP_ORIGIN:-}" ]]; then
  if ! systemctl --user -q is-active "$unit"; then
    systemctl --user reset-failed "$unit" 2>/dev/null || true
    # Unverified bound: 25 s covers the server's 20 s execution-gateway close.
    systemd-run --user --quiet --collect --unit "$unit" -p TimeoutStopSec=25 "$base/start-olp.sh"
  fi
fi

# Unverified bound: 30 s; the API answered within 2 s of start on the Deck.
for _ in $(seq 150); do
  curl_clean -sf -o /dev/null "$origin/api/offline-programming/v1/health" && break
  sleep 0.2
done

# The app gets system Qt 5.15 and only this app's lib/ (Assimp and its two
# dependencies, which SteamOS does not ship), never Steam's runtime paths or
# overlay preload. gamescope runs it under XWayland, so Qt uses xcb, at the
# layout's own 1280 x 800 scale. Not exec: the EXIT trap must still run.
expect_args=()
[[ -z $expect ]] || expect_args=(--expect-cell "$expect")
env -u LD_PRELOAD LD_LIBRARY_PATH="$base/lib" \
  QT_QPA_PLATFORM=xcb QT_AUTO_SCREEN_SCALE_FACTOR=0 QT_SCALE_FACTOR=1 \
  "$base/bin/rosie-pendant-v5" --olp "$origin" --bundle "$base/bin/olp-core.js" \
  --robots "$base/olp/robot_description/robots" "${expect_args[@]}" "$@"
