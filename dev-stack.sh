#!/usr/bin/env bash
# The local dev stack, as one thing to start, stop, restart and look at.
#
# Each member runs in its own session under a pid file in
# $STACK_DIR with its own log, so `stop` kills exactly what `start` began
# and nothing else, and `restart` is the two in order.
# rt-sim and rt-control provide the native local motion backend, and OLP owns
# its Cartesian intent process. MuJoCo
# and planning remain independent tools; no retired controller is launched.
# Native configuration and binaries live under rt-core/build.
#
# Open http://127.0.0.1:5189/offline-programming/v1/ui/ -- 5189, not 8794.
#
#   ./dev-stack.sh start|stop|restart|status|logs [name ...]
#
# With no names every process is meant; with names only those. `logs` tails
# the named logs (default: all). The OLP launcher is reused as-is (it builds
# what is missing and holds serve in the foreground), which is why olp takes
# the longest and why its log is the one to read when something is off.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Unverified local directory defaults; private per-user names prevent shared sockets.
RUN_BASE="${XDG_RUNTIME_DIR:-${TMPDIR:-/tmp}}"
STACK_DIR="${ROSIE_DEV_STACK_DIR:-$RUN_BASE/rosie-stack-$UID}"
CATALOG_SCRIPT="${CATALOG_DAEMON_SCRIPT:-$HOME/.cache/rosieos-olp/catalog-daemon/start-catalog-daemon.sh}"
export PATH="$HOME/.rosie/bin:$HOME/.rosie/go-1.26.2/bin:$HOME/.rosie/node-22.13.1/bin:$PATH"
# Unverified local selection defaults; deployed releases remain manifest-owned.
BACKEND="${DEV_STACK_BACKEND:-rt_core}"
[[ "$BACKEND" == rt_core ]] || { echo 'backend_retired: DEV_STACK_BACKEND requires rt_core' >&2; exit 2; }
# Refuse obsolete members before cache directories, discovery, or sockets.
for member in "${@:2}"; do
  case "$member" in
    sim|controller|cell) echo "service_retired: $member; use rt-sim and rt-control" >&2; exit 2 ;;
    pendant) echo 'service_retired: pendant; the virtual Deck is retired, use the Steam Deck v5 pendant' >&2; exit 2 ;;
  esac
done
export ROSIE_LOCAL_RT_DIR="${ROSIE_LOCAL_RT_DIR:-$RUN_BASE/rosie-dev-rt-$UID}"
export ROSIE_LOCAL_RT_BUILD="${ROSIE_LOCAL_RT_BUILD:-$ROOT/rt-core/build/dev-stack}"
RT_HELPER="$ROOT/motion-server/v1/local-rt-core.sh"

# Remote access over Tailscale. The Vite dev server proxies the API and
# accepts any host, so binding it on every interface is all a remote browser
# needs; the servers behind it stay on loopback. Set DEV_STACK_UI_HOST to
# 127.0.0.1 to keep the UI local even when Tailscale is up. Note the project
# store is per browser origin: programs authored at 127.0.0.1 are not visible
# from the Tailscale address and vice versa (export/import or the catalogue
# moves them).
TS_IP=""
TS_NAME=""
if [[ "${DEV_STACK_UI_HOST:-}" != 127.0.0.1 ]]; then
  # Unverified discovery ceiling: 3 s; local startup never waits for Tailscale.
  TS_IP="$(timeout 3 tailscale ip -4 2>/dev/null | head -n 1 || true)"
  TS_NAME="$(timeout 3 tailscale status --self --peers=false 2>/dev/null | awk 'NR==1 {print $2}' || true)"
fi
UI_HOST="${DEV_STACK_UI_HOST:-$([[ -n "$TS_IP" ]] && echo 0.0.0.0 || echo 127.0.0.1)}"
# How the UI is served. `dev` is Vite's dev server: instant reload on edit,
# but every source module is its own request and nothing is compressed --
# 125 requests and 9 MB per load, which over a Tailscale round trip is the
# wait. `built` builds once and serves the bundle compressed (57 requests,
# 3 MB, most of it the catalogue); edits need a `restart ui`. Built is the
# default when Tailscale is up, dev otherwise; DEV_STACK_UI_MODE overrides.
UI_MODE="${DEV_STACK_UI_MODE:-$([[ -n "$TS_IP" ]] && echo built || echo dev)}"
CATALOG_ORIGIN="http://127.0.0.1:8787"
MOTION_ORIGIN="http://127.0.0.1:8796"
OLP_ORIGIN="http://127.0.0.1:8794"
UI_URL="http://127.0.0.1:5189/offline-programming/v1/ui/"

NATS_PORT=14222
NATS_MONITOR=18222
DAEMON_INGEST="${DEV_STACK_DENSE_INGEST:-127.0.0.1:8797}"
NATS_URL="${DEV_STACK_DENSE_NATS_URL:-nats://127.0.0.1:$NATS_PORT}"
MS_BIN="${DEV_STACK_DENSE_BIN_DIR:-$(make -s --no-print-directory -C "$ROOT/motion-server/joint-trajectory/v1" print-bin-dir)}"
NATS_BIN="${DEV_STACK_NATS_BIN:-$HOME/.rosie/bin/nats-server}"

ALL=(catalog nats rt-sim rt-control daemon motion olp ui)
DENSE_PLANS="$ROSIE_LOCAL_RT_BUILD/dense-plans"
umask 077
for directory in "$STACK_DIR"; do
  [[ ! -L "$directory" && ( ! -e "$directory" || -O "$directory" ) ]] || { echo "dev-stack: refusing unowned runtime directory $directory" >&2; exit 1; }
  mkdir -p "$directory"
  chmod 0700 "$directory"
done

log() { printf 'dev-stack: %s\n' "$*"; }

health_url() {
  case "$1" in
    rt-sim) echo "$ROSIE_LOCAL_RT_DIR/ipc.sock" ;;
    rt-control) echo "$ROSIE_LOCAL_RT_DIR/control.sock (jog $ROSIE_LOCAL_RT_DIR/jog.sock)" ;;
    catalog)   echo "$CATALOG_ORIGIN/api/state" ;;
    nats)      echo "http://127.0.0.1:$NATS_MONITOR/varz" ;;
    motion)    echo "$MOTION_ORIGIN/api/motion/health" ;;
    olp)       echo "$OLP_ORIGIN/api/offline-programming/v1/health" ;;
    ui)        echo "$UI_URL" ;;
  esac
}

# A file the process writes once its socket is bound; a stale one from an
# earlier run is removed by start_one so it cannot answer for a dead process.
ready_file() {
  case "$1" in
    rt-sim) echo "$ROSIE_LOCAL_RT_DIR/metrics.json" ;;
    rt-control) echo "$STACK_DIR/rt-control-ready.json" ;;
    daemon)     echo "$STACK_DIR/daemon-ready.json" ;;
  esac
}

# The daemon's TCP status request: [u64 BE size][u32 BE json_len][json].
daemon_status_ok() {
  local address=127.0.0.1:8797
  if [[ "$BACKEND" == rt_core ]]; then
    address="$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(str(d["host"])+":"+str(d["port"]))' "$(ready_file daemon)" 2>/dev/null)" || return 1
  fi
  python3 - "$address" <<'PY' 2>/dev/null
import json, socket, struct, sys
host, port = sys.argv[1].rsplit(":", 1)
body = json.dumps({"kind": "status"}).encode()
frame = struct.pack(">Q", 4 + len(body)) + struct.pack(">I", len(body)) + body
with socket.create_connection((host, int(port)), timeout=3) as s:
    s.sendall(frame)
    head = b""
    while len(head) < 12:
        chunk = s.recv(12 - len(head))
        if not chunk:
            sys.exit(1)
        head += chunk
    size, json_len = struct.unpack(">Q", head[:8])[0], struct.unpack(">I", head[8:12])[0]
    data = b""
    while len(data) < json_len:
        chunk = s.recv(json_len - len(data))
        if not chunk:
            sys.exit(1)
        data += chunk
sys.exit(0 if json.loads(data).get("ok") else 1)
PY
}

# Require advancing native observations; stale files cannot make a paused core ready.
rt_sim_advancing() {
  pid_of rt-sim >/dev/null || return 1
  [[ -S "$ROSIE_LOCAL_RT_DIR/ipc.sock" ]] || return 1
  python3 - "$(ready_file rt-sim)" <<'PY' 2>/dev/null
import json, sys, time
# Unverified readiness polling bounds, in seconds; success requires native progress.
budget, poll = 1.0, 0.02
def read():
    with open(sys.argv[1]) as stream:
        return json.load(stream)
first = read()
assert first['time_ns'] > 0 and first['daemon_incarnation']
deadline = time.monotonic() + budget
while time.monotonic() < deadline:
    current = read()
    if current['daemon_incarnation'] != first['daemon_incarnation']:
        sys.exit(1)
    if current['time_ns'] > first['time_ns']:
        sys.exit(0)
    time.sleep(poll)
sys.exit(1)
PY
}

healthy() {
  case "$1" in
    rt-sim) rt_sim_advancing ;;
    rt-control)
      pid_of rt-control >/dev/null && rt_sim_advancing &&
        curl -fsS --max-time 3 --unix-socket "$ROSIE_LOCAL_RT_DIR/control.sock" http://localhost/v1/status |
        python3 -c 'import json,sys; d=json.load(sys.stdin)["data"]; c=d["core"]; assert d["daemon_incarnation"] and c["daemon_incarnation"] == d["daemon_incarnation"] and c["time_ns"] > 0 and c["backend"] == "simulation"; print(json.dumps(d))' > "$(ready_file rt-control)" ;;
    daemon)         pid_of daemon >/dev/null && daemon_status_ok ;;
    *) curl -fsS -m 3 -o /dev/null "$(health_url "$1")" 2>/dev/null ;;
  esac
}

# The command each process runs, as a bash -c string; cwd set per process.
command_for() {
  local olp_backend_env=""
  if [[ "$BACKEND" == rt_core ]]; then
    olp_backend_env="DEV_STACK_BACKEND=rt_core OFFLINE_PROGRAMMING_EXECUTION_BACKEND=rt_core OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core "
    case "$1" in
      rt-sim) printf 'exec bash %q sim\n' "$RT_HELPER"; return ;;
      rt-control) printf 'exec bash %q control\n' "$RT_HELPER"; return ;;
      daemon)
        printf 'mkdir -p %q && exec %q --backend rt_core --config %q --dense-ingest-listen %q --plan-store-dir %q --ready-file %q --nats-url %q --robot-cell dev-cell --robot-command-subject robot/v4/robot.dev-cell.command --status-publish-subject robot/v4/robot.dev-cell.status\n' "$DENSE_PLANS" "$MS_BIN/joint_trajectory_daemon" "$ROSIE_LOCAL_RT_BUILD/daemon-rt-core.json" "$DAEMON_INGEST" "$DENSE_PLANS" "$(ready_file daemon)" "$NATS_URL"
        return ;;
    esac
  fi
  if [[ -n "${DEV_STACK_OLP_REMOTE:-}" ]]; then
    local remote_binding
    printf -v remote_binding '%q' "$DEV_STACK_OLP_REMOTE"
    olp_backend_env="DEV_STACK_BACKEND=$BACKEND OFFLINE_PROGRAMMING_EXECUTION_BACKEND=rt_core OFFLINE_PROGRAMMING_JOG_BACKEND=$BACKEND OFFLINE_PROGRAMMING_RT_CORE_REMOTE=$remote_binding "
  fi
  if [[ -n "${DEV_STACK_OLP_CELLS:-}" ]]; then
    local cells_binding
    printf -v cells_binding '%q' "$DEV_STACK_OLP_CELLS"
    olp_backend_env+="DEV_STACK_OLP_CELLS=$cells_binding "
  fi
  case "$1" in
    catalog)   echo "exec \"$CATALOG_SCRIPT\"" ;;
    nats)      echo "exec \"$NATS_BIN\" -a 127.0.0.1 -p $NATS_PORT -m $NATS_MONITOR" ;;
    motion)    echo "cd \"$ROOT/weld_planner/v1\" && exec pixi run -e motion motion-serve" ;;
    olp)       echo "cd \"$ROOT/offline-programming/v1\" && ${olp_backend_env}OFFLINE_PROGRAMMING_PROGRAM_CATALOG_ORIGIN=\"$CATALOG_ORIGIN\" OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN=\"$MOTION_ORIGIN\" OLP_RESTART_SERVE=1 exec bash ./start-offline-programming.sh" ;;
    ui)
      if [[ "$UI_MODE" == "built" ]]; then
        echo "cd \"$ROOT/offline-programming/v1/ui\" && npm run -s stage:assets && npx vite build --outDir .staged-dist --logLevel warn && exec npx vite preview --outDir .staged-dist --host $UI_HOST --port 5189 --strictPort"
      else
        echo "cd \"$ROOT/offline-programming/v1/ui\" && exec npm run dev -- --host $UI_HOST --port 5189"
      fi
      ;;
  esac
}

process_start_ticks() {
  local stat remainder ticks
  stat="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  # comm may contain spaces and ')': fields begin after the final ') '.
  remainder="${stat##*) }"
  [[ "${remainder%% *}" != Z && "${remainder%% *}" != X ]] || return 1
  ticks="$(awk '{print $20}' <<< "$remainder")"
  [[ "$ticks" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "$ticks"
}

recorded_pid() {
  local file="$STACK_DIR/$1.pid"
  [[ -f "$file" ]] || return 1
  local pid ticks
  pid="$(cat "$file")"
  [[ "$pid" =~ ^[0-9]+$ && -f "$file.start" ]] || return 1
  kill -0 "$pid" 2>/dev/null || return 1
  ticks="$(process_start_ticks "$pid")" || return 1
  [[ "$(cat "$file.start")" == "$ticks" ]] || return 1
  echo "$pid"
}

pid_of() {
  local pid
  pid="$(recorded_pid "$1")" || return 1
  grep -zFxq "ROSIE_DEV_STACK_OWNER=$STACK_DIR" "/proc/$pid/environ" 2>/dev/null || return 1
  echo "$pid"
}

start_one() {
  local name="$1"
  if pid_of "$name" >/dev/null; then
    log "$name already running (pid $(pid_of "$name"))"
    return 0
  fi
  if healthy "$name"; then
    log "$name is healthy at $(health_url "$name") but not ours -- leaving it; stop it yourself to let this script own it"
    return 0
  fi
  if [[ "$name" == "catalog" && ! -x "$CATALOG_SCRIPT" ]]; then
    log "catalog: $CATALOG_SCRIPT is missing; skipping (set CATALOG_DAEMON_SCRIPT)"
    return 0
  fi
  case "$name" in
    rt-sim)
      if [[ ! -x "$ROOT/rt-core/build/rt-control" || ! -x "$ROOT/rt-core/build/rtctl" || ! -x "$ROOT/rt-core/build/rosie-rt-core-sim" ]]; then
        make -C "$ROOT/rt-core" control sim
      fi
      bash "$RT_HELPER" prepare ;;
    daemon|olp)
      if [[ "$BACKEND" == rt_core ]]; then
        healthy rt-control || { log "$name requires healthy rt-control"; return 1; }
        bash "$RT_HELPER" bind
      fi ;;
  esac
  case "$name" in
    nats)
      if [[ ! -x "$NATS_BIN" ]]; then
        log "nats: $NATS_BIN is missing; skipping (install nats-server there, v2.14 or later)"
        return 0
      fi ;;
    daemon)
      local binary="$MS_BIN/joint_trajectory_daemon"
      if [[ ! -x "$binary" ]]; then
        log "$name: $binary is missing; skipping (cd motion-server/joint-trajectory/v1 && make)"
        return 0
      fi ;;
  esac
  local ready; ready="$(ready_file "$name")"
  [[ -n "$ready" ]] && rm -f "$ready"
  local logfile="$STACK_DIR/$name.log"
  : > "$logfile"
  local command
  command="$(command_for "$name")" || return 1
  # setsid: its own session, so stop can kill the whole tree (pixi -> python,
  # npm -> vite, the launcher -> serve -> cartesiand) by process group.
  ROSIE_DEV_STACK_OWNER="$STACK_DIR" setsid bash -c "$command" >"$logfile" 2>&1 < /dev/null &
  echo $! > "$STACK_DIR/$name.pid"
  process_start_ticks "$!" > "$STACK_DIR/$name.pid.start"
  log "$name starting (pid $!) -> $logfile"
}

stop_one() {
  local name="$1"
  local pid
  if ! pid="$(pid_of "$name")"; then
    rm -f "$STACK_DIR/$name.pid" "$STACK_DIR/$name.pid.start"
    return 0
  fi
  log "stopping $name (pgid $pid)"
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.5
  done
  if kill -0 "$pid" 2>/dev/null; then
    log "$name did not exit; killing"
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$STACK_DIR/$name.pid" "$STACK_DIR/$name.pid.start"
}

# Remote access needs one host firewall rule: ufw is active on this machine
# and drops inbound tailnet traffic to the UI port. Added on the first start
# with the UI bound remotely (sudo asks for the password in your terminal),
# scoped to the Tailscale interface so the LAN side stays closed, then
# remembered by a marker so later starts never ask again. `ufw allow` is
# idempotent, so an already-present rule is fine. DEV_STACK_FIREWALL=0 skips.
ensure_firewall() {
  [[ "$UI_HOST" != "127.0.0.1" ]] || return 0
  [[ "${DEV_STACK_FIREWALL:-1}" == "1" ]] || return 0
  [[ "$(systemctl is-active ufw 2>/dev/null)" == "active" ]] || return 0
  local marker="$STACK_DIR/ufw-5189.allowed"
  [[ -f "$marker" ]] && return 0
  log "ufw is active; allowing tailnet access to the UI port once (sudo may ask for your password)"
  if sudo ufw allow in on tailscale0 to any port 5189 proto tcp; then
    touch "$marker"
    log "ufw: tailscale0 -> :5189 allowed"
  else
    log "ufw rule not added (no sudo?); the remote address will not connect until you run:"
    log "  sudo ufw allow in on tailscale0 to any port 5189 proto tcp"
  fi
  if tailscale serve status 2>/dev/null | grep -q ":5189"; then
    log "tailscale serve still owns tailnet port 5189 and answers 404 on the raw IP; to use the IP run:"
    log "  sudo tailscale serve --http=5189 off"
  fi
}

wait_healthy() {
  local name
  local names=("$@") deadline=$((SECONDS + ${STACK_WAIT_SECS:-300}))
  local pending=("${names[@]}")
  while (( ${#pending[@]} > 0 && SECONDS < deadline )); do
    local still=()
    for name in "${pending[@]}"; do
      if healthy "$name"; then
        log "$name healthy: $(health_url "$name")"
      # A live recorded child may still be entering exec, before its owner
      # environment is visible. Only healthy/stop require full ownership.
      elif ! recorded_pid "$name" >/dev/null; then
        log "$name exited before it was healthy -- see $STACK_DIR/$name.log"
      else
        still+=("$name")
      fi
    done
    pending=("${still[@]:-}")
    [[ -n "${pending[0]:-}" ]] || pending=()
    (( ${#pending[@]} == 0 )) || sleep 2
  done
  for name in "${pending[@]:-}"; do
    [[ -n "$name" ]] && log "$name not healthy after ${STACK_WAIT_SECS:-300}s -- tail -f $STACK_DIR/$name.log"
  done
  return 0
}

status_all() {
  local names=("$@")
  printf '  backend: %s\n' "$(cat "$STACK_DIR/backend" 2>/dev/null || echo "$BACKEND")"
  printf '  rt-core sockets: native=%s control=%s jog=%s\n' "$ROSIE_LOCAL_RT_DIR/ipc.sock" "$ROSIE_LOCAL_RT_DIR/control.sock" "$ROSIE_LOCAL_RT_DIR/jog.sock"
  for name in "${names[@]}"; do
    local pid state
    pid="$(pid_of "$name" || echo -)"
    if healthy "$name"; then state="healthy"; else state="down"; fi
    printf '  %-10s pid %-8s %-8s %s\n' "$name" "$pid" "$state" "$(health_url "$name")"
  done
  echo "  olp target: rt_core control=$ROSIE_LOCAL_RT_DIR/control.sock"
  if [[ -n "${DEV_STACK_OLP_REMOTE:-}" ]]; then
    olp_remote_target "$DEV_STACK_OLP_REMOTE" || return 1
  fi
  echo "  app:    $UI_URL  (ui mode: $UI_MODE$([[ "$UI_MODE" == "built" ]] && echo ", restart ui after editing the UI" || echo ", live reload"))"
  if [[ "$UI_HOST" == "127.0.0.1" ]]; then
    echo "  remote: off (UI bound to 127.0.0.1; unset DEV_STACK_UI_HOST or set it to 0.0.0.0)"
  elif [[ -n "$TS_IP" ]]; then
    if tailscale serve status 2>/dev/null | grep -q "^https://"; then
      # HTTPS through tailscale serve: the browser's HTTPS-first upgrade and
      # the ts.net HSTS preload both want this, and the cert is Tailscale's.
      echo "  remote: $(tailscale serve status 2>/dev/null | grep -m1 '^https://' | awk '{print $1}')/offline-programming/v1/ui/  (tailscale serve, https)"
    elif tailscale serve status 2>/dev/null | grep -q ":5189"; then
      # tailscale serve owns tailnet port 5189: it matches by host name, so
      # the raw IP answers its own 404. Give the names.
      echo "  remote: http://$TS_NAME:5189/offline-programming/v1/ui/  (tailscale serve; use the name, the raw IP answers 404)"
    else
      echo "  remote: http://$TS_IP:5189/offline-programming/v1/ui/  (tailscale)"
      [[ -n "$TS_NAME" ]] && echo "          http://$TS_NAME:5189/offline-programming/v1/ui/  (magicdns)"
    fi
    echo "          the project store is per origin: use export/import or the catalogue to move programs between addresses"
    if [[ "$(systemctl is-active ufw 2>/dev/null)" == "active" && ! -f "$STACK_DIR/ufw-5189.allowed" ]]; then
      echo "          ufw is active and the tailnet rule is not recorded yet; the next 'start ui' adds it (sudo)"
    fi
  else
    echo "  remote: UI bound to $UI_HOST but no tailscale address found"
  fi
}

# Display only the selected origin and pair, preventing credential paths or URL
# userinfo from appearing in stack status.
olp_remote_target() {
  python3 - "$1" <<'PY'
import json, re, sys, urllib.parse
try:
    cfg = json.load(open(sys.argv[1]))
    u = urllib.parse.urlsplit(cfg['address'])
    pair = cfg['binding']['pair_id']
    if (u.scheme != 'https' or not u.hostname or u.username is not None or u.password is not None or
            u.path not in ('', '/') or u.query or u.fragment or
            any(c.isspace() for c in cfg['address']) or not re.fullmatch(r'[A-Za-z0-9_.:-]+', pair)):
        raise ValueError()
    print('  olp remote: https://%s pair %s · jog local' % (u.netloc, pair))
except (OSError, ValueError, KeyError, TypeError):
    raise SystemExit('olp_remote_binding_invalid')
PY
}

main() {
  local action="${1:-status}"; shift || true
  case "$action" in
    stop|status|logs|restart)
      if [[ -f "$STACK_DIR/backend" && "$(cat "$STACK_DIR/backend")" != "$BACKEND" ]]; then
        DEV_STACK_BACKEND="$(cat "$STACK_DIR/backend")" exec bash "$0" "$action" "$@"
      fi
      if [[ -f "$STACK_DIR/rt-dir" && -f "$STACK_DIR/rt-build" ]]; then
        export ROSIE_LOCAL_RT_DIR="$(cat "$STACK_DIR/rt-dir")"
        export ROSIE_LOCAL_RT_BUILD="$(cat "$STACK_DIR/rt-build")"
      fi
      if [[ -f "$STACK_DIR/olp-remote" ]]; then
        export DEV_STACK_OLP_REMOTE="$(cat "$STACK_DIR/olp-remote")"
      fi ;;
  esac
  local names=("$@")
  (( ${#names[@]} > 0 )) || names=("${ALL[@]}")
  case "$action" in
    start)
      if [[ " ${names[*]} " == *' olp '* ]]; then
        if [[ -n "${DEV_STACK_OLP_REMOTE:-}" ]]; then
          export DEV_STACK_OLP_REMOTE="$(realpath -e -- "$DEV_STACK_OLP_REMOTE")"
          olp_remote_target "$DEV_STACK_OLP_REMOTE" || exit 1
        fi
        if pid_of olp >/dev/null && [[ "$(cat "$STACK_DIR/olp-remote" 2>/dev/null || true)" != "${DEV_STACK_OLP_REMOTE:-}" ]]; then
          log 'stop olp before changing its remote binding'; exit 1
        fi
        if [[ -n "${DEV_STACK_OLP_REMOTE:-}" ]]; then
          printf '%s\n' "$DEV_STACK_OLP_REMOTE" > "$STACK_DIR/olp-remote"
        else
          rm -f "$STACK_DIR/olp-remote"
        fi
      fi
      if [[ -f "$STACK_DIR/backend" && "$(cat "$STACK_DIR/backend")" != "$BACKEND" ]]; then
        log "stop the active backend before changing DEV_STACK_BACKEND"; exit 1
      fi
      if [[ "$BACKEND" == rt_core ]]; then
        if [[ -f "$STACK_DIR/rt-dir" ]] && { [[ "$(cat "$STACK_DIR/rt-dir")" != "$ROSIE_LOCAL_RT_DIR" ]] || [[ "$(cat "$STACK_DIR/rt-build")" != "$ROSIE_LOCAL_RT_BUILD" ]]; }; then
          log 'stop the active stack before changing its simulation paths'; exit 1
        fi
        printf '%s\n' "$ROSIE_LOCAL_RT_DIR" > "$STACK_DIR/rt-dir"
        printf '%s\n' "$ROSIE_LOCAL_RT_BUILD" > "$STACK_DIR/rt-build"
      fi
      printf '%s\n' "$BACKEND" > "$STACK_DIR/backend"
      for name in "${names[@]}"; do
        [[ "$name" == "ui" ]] && ensure_firewall
        start_one "$name"
        if [[ "$name" == rt-sim || "$name" == rt-control ]]; then
          wait_healthy "$name"
          healthy "$name" || exit 1
        fi
      done
      wait_healthy "${names[@]}"
      status_all "${names[@]}"
      ;;
    stop)
      # Reverse order: the UI and OLP before the servers they talk to.
      for (( i=${#names[@]}-1; i>=0; i-- )); do stop_one "${names[$i]}"; done
      if ! pid_of olp >/dev/null; then rm -f "$STACK_DIR/olp-remote"; fi
      local active=0
      for name in "${ALL[@]}"; do if pid_of "$name" >/dev/null; then active=1; fi; done
      (( active != 0 )) || rm -f "$STACK_DIR/backend" "$STACK_DIR/rt-dir" "$STACK_DIR/rt-build"
      ;;
    restart)
      "$0" stop "${names[@]}"
      "$0" start "${names[@]}"
      ;;
    status)
      status_all "${names[@]}"
      ;;
    logs)
      local files=()
      for name in "${names[@]}"; do files+=("$STACK_DIR/$name.log"); done
      exec tail -n 40 -F "${files[@]}"
      ;;
    *)
      sed -n 2,20p "$0"
      exit 2
      ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
