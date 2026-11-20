#!/usr/bin/env bash
# Official Offline Programming V1 suite launcher for WSL (the planner host).
#
# Brings up the loopback Motion Server planner + pixi Tesseract worker by
# starting (or reusing) OLP `serve` on 127.0.0.1:8794 with local simulation.
# Reuses the Vite UI on 127.0.0.1:5189 when it is already running. Does not
# enable connected execution or physical arming. A missing robot does not
# block local Tesseract simulation.
#
# Lower-level Linux server helper: ./serve-wsl.sh
# Windows helper: ./start-offline-programming.ps1
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OLP="$ROOT/offline-programming/v1"
LISTEN="${OLP_LISTEN:-127.0.0.1:8794}"
UI_HOST="${OLP_UI_HOST:-127.0.0.1}"
UI_PORT="${OLP_UI_PORT:-5189}"
UI_PATH="/offline-programming/v1/ui/"
APP_URL="http://${UI_HOST}:${UI_PORT}${UI_PATH}"
API_ORIGIN="http://${LISTEN}"
TES="$ROOT/tesseract/v1"
PLANNER_SRC="$ROOT/motion-server/v1"
PLANNER_DEST="$PLANNER_SRC/bin/robot-v4-cartesiand"
SERVER_DEST="$OLP/bin/offline-programming-linux"
PY_DEST="$TES/.pixi/envs/default/bin/python3"
PIXILIB="$TES/.pixi/envs/default/lib"
DIST_INDEX="$OLP/ui/dist/index.html"
SERVE_PID=""
STARTED_SERVE=0
OLP_REBUILT=0
SIM_WAIT_SECS="${OLP_SIM_WAIT_SECS:-270}"
# Native selection is shared by execution and local Cartesian jog.
RT_REMOTE_JOG_EXPLICIT="${OFFLINE_PROGRAMMING_JOG_BACKEND:+1}"
export OFFLINE_PROGRAMMING_EXECUTION_BACKEND="${OFFLINE_PROGRAMMING_EXECUTION_BACKEND:-${DEV_STACK_BACKEND:-rt_core}}"
export OFFLINE_PROGRAMMING_JOG_BACKEND="${OFFLINE_PROGRAMMING_JOG_BACKEND:-${DEV_STACK_BACKEND:-rt_core}}"
# Refuse obsolete selections before creating caches, binding sockets, or children.
for backend in "$OFFLINE_PROGRAMMING_EXECUTION_BACKEND" "$OFFLINE_PROGRAMMING_JOG_BACKEND"; do
  [[ "$backend" == rt_core ]] || { printf 'backend_retired: select rt_core\n' >&2; exit 1; }
done
RT_CHILDREN=()

# Under WSL HOME can point elsewhere; use the login's Linux home when it exists.
user_home="/home/$(id -un)"
if [[ -d "$user_home" ]]; then
  export HOME="$user_home"
fi
export PATH="${HOME}/.rosie/bin:${HOME}/.rosie/go-1.26.2/bin:${HOME}/.pixi/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin${PATH:+:$PATH}"

CACHE="${ROSIEOS_OLP_CACHE:-}"
if [[ -z "$CACHE" ]]; then
  for candidate in "${HOME}/.cache/rosieos-olp" "$user_home/.cache/rosieos-olp"; do
    if [[ -d "$candidate" ]]; then
      CACHE="$candidate"
      break
    fi
  done
fi
if [[ -z "$CACHE" ]]; then
  CACHE="${HOME}/.cache/rosieos-olp"
fi
mkdir -p "$CACHE"

log() { printf 'olp-start: %s\n' "$*"; }
err() { printf 'olp-start: %s\n' "$*" >&2; }
die() {
  err "$*"
  exit 1
}

# Reuse only the explicitly bound stack adapter; standalone launch owns its
# simulation children and tears them down with serve.
ensure_local_rt_core() {
  case "$OFFLINE_PROGRAMMING_JOG_BACKEND" in rt_core) ;; *) die 'backend_retired: jog requires rt_core' ;; esac
  case "$OFFLINE_PROGRAMMING_EXECUTION_BACKEND" in rt_core) ;; *) die 'backend_retired: execution requires rt_core' ;; esac
  local helper="$ROOT/motion-server/v1/local-rt-core.sh"
  if [[ -z "${ROSIE_LOCAL_RT_BUILD:-}" && -z "${ROSIE_RT_CONTROL_SOCKET:-}" && -f "$ROOT/rt-core/build/dev-stack/binding.env" ]]; then
    if (source "$ROOT/rt-core/build/dev-stack/binding.env"; curl -fsS --max-time 3 --unix-socket "$ROSIE_RT_CONTROL_SOCKET" http://localhost/v1/describe >/dev/null 2>&1); then
      export ROSIE_LOCAL_RT_BUILD="$ROOT/rt-core/build/dev-stack"
    fi
  fi
  if [[ -n "${ROSIE_LOCAL_RT_BUILD:-}" && -f "$ROSIE_LOCAL_RT_BUILD/binding.env" ]]; then
    source "$ROSIE_LOCAL_RT_BUILD/binding.env"
    curl -fsS --unix-socket "$ROSIE_RT_CONTROL_SOCKET" http://localhost/v1/describe >/dev/null || die 'configured stack adapter is unavailable'
  elif [[ -n "${ROSIE_RT_CONTROL_SOCKET:-}" ]]; then
    curl -fsS --unix-socket "$ROSIE_RT_CONTROL_SOCKET" http://localhost/v1/describe >/dev/null || die 'configured simulation adapter is unavailable'
  else
    export ROSIE_LOCAL_RT_DIR="$(mktemp -d "${TMPDIR:-/tmp}/olp-rt.XXXXXX")"
    export ROSIE_LOCAL_RT_BUILD="$ROOT/rt-core/build/olp-local"
    mkdir -p "$ROSIE_LOCAL_RT_BUILD"
    make -C "$ROOT/rt-core" control sim
    bash "$helper" prepare
    bash "$helper" sim >"$ROSIE_LOCAL_RT_BUILD/sim.log" 2>&1 &
    RT_CHILDREN+=("$!")
    echo "$!" > "$ROSIE_LOCAL_RT_BUILD/sim.pid"
    bash "$helper" control >"$ROSIE_LOCAL_RT_BUILD/control.log" 2>&1 &
    RT_CHILDREN+=("$!")
    echo "$!" > "$ROSIE_LOCAL_RT_BUILD/control.pid"
    # Unverified startup observation ceiling; successful Describe is required.
    local deadline=$((SECONDS + 30))
    until curl -fsS --unix-socket "$ROSIE_LOCAL_RT_DIR/control.sock" http://localhost/v1/describe >"$ROSIE_LOCAL_RT_BUILD/control-ready.json" 2>/dev/null; do
      (( SECONDS < deadline )) || die "simulation adapter not ready; see $ROSIE_LOCAL_RT_BUILD/control.log"
      kill -0 "${RT_CHILDREN[0]}" && kill -0 "${RT_CHILDREN[1]}" || die 'simulation child exited'
      sleep 0.1
    done
    bash "$helper" bind
    source "$ROSIE_LOCAL_RT_BUILD/binding.env"
    # A newly owned adapter must also own the serve binding, preventing reuse
    # of a server still attached to a different standalone simulation.
    export OLP_RESTART_SERVE=1
  fi
}

# Resolve only the live remote identity; failed discovery cannot leave serve
# using a local compile or a previous remote configuration.
ensure_remote_rt_core() {
  [[ -n "${OFFLINE_PROGRAMMING_RT_CORE_REMOTE:-}" ]] || return 0
  [[ "$OFFLINE_PROGRAMMING_EXECUTION_BACKEND" == rt_core ]] || die 'remote_execution_backend_required: select rt_core'
  [[ "$RT_REMOTE_JOG_EXPLICIT" == 1 ]] || die 'remote_jog_backend_required: explicitly select rt_core (local simulation)'
  local destination
  mkdir -p "$ROOT/rt-core/build/olp-remote"
  destination="$(mktemp -d "$ROOT/rt-core/build/olp-remote/binding.XXXXXX")/olp-rt-core.json"
  RT_REMOTE_SUMMARY="$(python3 - "$OFFLINE_PROGRAMMING_RT_CORE_REMOTE" "$destination" <<'PY'
import json, pathlib, re, subprocess, sys, urllib.parse

def fail(reason):
    raise SystemExit(reason)

try:
    source = pathlib.Path(sys.argv[1]).resolve(strict=True)
    cfg = json.loads(source.read_text())
    address = cfg['address']
    url = urllib.parse.urlsplit(address)
    if (url.scheme != 'https' or not url.hostname or url.username is not None or
            url.password is not None or url.path not in ('', '/') or url.query or url.fragment or
            any(c.isspace() for c in address)):
        fail('remote_address_invalid')
    binding = cfg['binding']
    if (not re.fullmatch(r'[A-Za-z0-9_.:-]+', binding['pair_id']) or
            type(binding['revision']) is not int or binding['revision'] < 1 or
            type(cfg['axis_mask']) is not int or not 1 <= cfg['axis_mask'] <= 511):
        fail('remote_binding_invalid')
    expected = cfg.get('expected_backend')
    if expected not in (None, 'ethercat', 'simulation'):
        fail('remote_expected_backend_invalid')
    resolved = {}
    for field in ('ca_file', 'certificate_file', 'key_file'):
        value = cfg.get(field)
        if not isinstance(value, str) or not value:
            fail('remote_credential_missing: ' + field)
        path = source.parent / value
        if not path.is_file():
            fail('remote_credential_missing: ' + field)
        resolved[field] = str(path.resolve())
    san = subprocess.run(['openssl', 'x509', '-in', resolved['certificate_file'],
        '-noout', '-ext', 'subjectAltName'], capture_output=True, text=True)
    pairs = re.findall(r'URI:rosie-pair:([^,\s]+)', san.stdout)
    if san.returncode or pairs != [binding['pair_id']]:
        fail('remote_certificate_pair_mismatch')
    # Unverified discovery ceiling; an authenticated Describe response establishes identity.
    response = subprocess.run(['curl', '-q', '--fail', '--silent', '--show-error', '--max-time', '10',
        '--noproxy', '*', '--cacert', resolved['ca_file'], '--cert', resolved['certificate_file'],
        '--key', resolved['key_file'], address.rstrip('/') + '/v1/describe'], capture_output=True)
    if response.returncode:
        fail('remote_describe_failed: HTTPS request refused')
    try:
        envelope = json.loads(response.stdout)
        d = envelope['data']
        if envelope.get('ok') is not True:
            fail('remote_describe_failed: response refused')
        if expected and d['backend'] != expected:
            fail('remote_backend_mismatch: expected ' + expected)
        if d['backend'] not in ('ethercat', 'simulation') or d['contract_version'] != 2:
            fail('remote_description_invalid: backend or contract')
        if not 1 <= len(d['axes']) <= 9 or cfg['axis_mask'] >> len(d['axes']):
            fail('remote_description_invalid: axis_mask')
        for field in ('machine_sha256', 'configuration_sha256', 'deployment_sha256', 'capabilities_digest'):
            if not re.fullmatch(r'[a-f0-9]{64}', d[field]):
                fail('remote_description_invalid: ' + field)
    except (KeyError, TypeError, ValueError):
        fail('remote_describe_failed: invalid description')
    result = dict(resolved, address=address.rstrip('/'), axis_mask=cfg['axis_mask'],
        binding=dict(pair_id=binding['pair_id'], revision=binding['revision'],
            machine_sha256=d['machine_sha256'], configuration_sha256=d['configuration_sha256']),
        deployment_sha256=d['deployment_sha256'], capabilities_digest=d['capabilities_digest'])
    if expected:
        result['expected_backend'] = expected
    pathlib.Path(sys.argv[2]).write_text(json.dumps(result) + '\n')
    print('dense backend: rt_core (remote %s pair %s configuration %s)' %
        (result['address'], binding['pair_id'], d['configuration_sha256']))
except (OSError, KeyError, TypeError, ValueError):
    fail('remote_binding_invalid: unreadable or malformed binding')
PY
  )" || die 'remote_binding_refused'
  RT_REMOTE_CONFIG="$destination"
  # Reused serve processes cannot prove that they hold these credentials/bindings.
  export OLP_RESTART_SERVE=1
}

http_get() {
  local url="$1"
  python3 - "$url" <<'PY'
import sys, urllib.error, urllib.request
url = sys.argv[1]
try:
    with urllib.request.urlopen(url, timeout=3) as response:
        sys.stdout.write(response.read().decode("utf-8", "replace"))
except Exception:
    sys.exit(1)
PY
}

port_open() {
  local host="$1" port="$2"
  python3 - "$host" "$port" <<'PY'
import socket, sys
host, port = sys.argv[1], int(sys.argv[2])
sock = socket.socket()
sock.settimeout(1)
try:
    sock.connect((host, port))
except OSError:
    sys.exit(1)
finally:
    sock.close()
PY
}

link_if_missing() {
  local dest="$1" src="$2"
  if [[ -e "$dest" || -L "$dest" ]]; then
    return 0
  fi
  if [[ -e "$src" ]]; then
    mkdir -p "$(dirname "$dest")"
    ln -sfn "$src" "$dest"
    log "linked $dest -> $src"
  fi
}

ensure_planner() {
  if [[ "$OFFLINE_PROGRAMMING_JOG_BACKEND" == rt_core ]]; then
    export OFFLINE_PROGRAMMING_JOG_BINARY="$ROOT/rt-core/build/cartesian/robot-v4-cartesiand"
    make -C "$PLANNER_SRC" all BIN_DIR="$ROOT/rt-core/build/cartesian" OBJECT_CACHE_DIR="$ROOT/rt-core/build/cartesian-cache"
  fi
  link_if_missing "$PLANNER_DEST" "$CACHE/motion-server-bin/robot-v4-cartesiand"
  if [[ -x "$PLANNER_DEST" ]]; then
    log "planner ready: $PLANNER_DEST"
    return 0
  fi
  if ! command -v g++ >/dev/null 2>&1 && ! command -v c++ >/dev/null 2>&1; then
    die "planner missing and no C++ compiler. Next: sudo apt install g++ make, then re-run this script."
  fi
  if ! command -v make >/dev/null 2>&1; then
    die "planner missing and make is unavailable. Next: sudo apt install make g++, then re-run this script."
  fi
  log "building robot-v4-cartesiand"
  mkdir -p "$CACHE/motion-server-bin" "$CACHE/motion-cache" "$PLANNER_SRC/bin"
  (
    cd "$PLANNER_SRC"
    BIN_DIR="$CACHE/motion-server-bin" \
      OBJECT_CACHE_DIR="$CACHE/motion-cache" \
      make -j"$(nproc)" all
  )
  link_if_missing "$PLANNER_DEST" "$CACHE/motion-server-bin/robot-v4-cartesiand"
  if [[ ! -x "$PLANNER_DEST" && -x "$CACHE/motion-server-bin/robot-v4-cartesiand" ]]; then
    ln -sfn "$CACHE/motion-server-bin/robot-v4-cartesiand" "$PLANNER_DEST"
  fi
  [[ -x "$PLANNER_DEST" ]] || die "planner build finished but $PLANNER_DEST is not executable"
  log "planner ready: $PLANNER_DEST"
}

wait_for_pixi_install() {
  local waited=0
  while pgrep -af 'pixi install' | grep -v "pgrep -af" >/dev/null 2>&1; do
    if (( waited >= 1200 )); then
      die "timed out waiting for an in-flight pixi install. Next: cd tesseract/v1 && pixi install --locked"
    fi
    if (( waited % 30 == 0 )); then
      log "waiting for in-flight pixi install (${waited}s)"
    fi
    sleep 5
    waited=$((waited + 5))
  done
}

ensure_pixi() {
  mkdir -p "$CACHE/tesseract-pixi"
  if [[ ! -e "$TES/.pixi" && -d "$CACHE/tesseract-pixi" ]]; then
    ln -sfn "$CACHE/tesseract-pixi" "$TES/.pixi"
    log "linked $TES/.pixi -> $CACHE/tesseract-pixi"
  fi
  if [[ -x "$PY_DEST" && -d "$PIXILIB" ]] && "$PY_DEST" -c 'import tesseract_robotics' >/dev/null 2>&1; then
    log "Tesseract pixi ready: $PY_DEST"
    return 0
  fi
  wait_for_pixi_install
  if [[ -x "$PY_DEST" && -d "$PIXILIB" ]] && "$PY_DEST" -c 'import tesseract_robotics' >/dev/null 2>&1; then
    log "Tesseract pixi ready: $PY_DEST"
    return 0
  fi
  if ! command -v pixi >/dev/null 2>&1; then
    die "Tesseract pixi env missing. Next: curl -fsSL https://pixi.sh/install.sh | bash ; then cd tesseract/v1 && pixi install --locked"
  fi
  log "installing Tesseract pixi env (pixi install --locked)"
  (
    cd "$TES"
    pixi install --locked
  )
  [[ -x "$PY_DEST" && -d "$PIXILIB" ]] || die "pixi install finished but $PY_DEST is missing"
  "$PY_DEST" -c 'import tesseract_robotics' >/dev/null || die "pixi python cannot import tesseract_robotics"
  log "Tesseract pixi ready: $PY_DEST"
}

ensure_olp_linux() {
  link_if_missing "$SERVER_DEST" "$CACHE/olp-bin/offline-programming-linux"
  local need_build=0
  if [[ ! -x "$SERVER_DEST" ]]; then
    need_build=1
  elif find "$OLP" -name '*.go' ! -path '*/ui/*' -newer "$SERVER_DEST" -print -quit | grep -q .; then
    need_build=1
    log "Go sources newer than $SERVER_DEST; rebuilding so jog uses motion-server/v1"
  fi
  if [[ "$need_build" -eq 0 ]]; then
    log "OLP linux binary ready: $SERVER_DEST"
    return 0
  fi
  if command -v go >/dev/null 2>&1; then
    log "building offline-programming-linux"
    mkdir -p "$OLP/bin" "$CACHE/olp-bin"
    (
      cd "$OLP"
      CGO_ENABLED=0 GOOS=linux GOARCH="$(go env GOHOSTARCH)" go build -o "$SERVER_DEST" .
    )
    cp -f "$SERVER_DEST" "$CACHE/olp-bin/offline-programming-linux" 2>/dev/null || true
    [[ -x "$SERVER_DEST" ]] || die "go build finished but $SERVER_DEST is not executable"
    OLP_REBUILT=1
    log "OLP linux binary ready: $SERVER_DEST"
    return 0
  fi
  die "OLP linux binary missing and WSL has no go. Next (Windows PowerShell):
  cd offline-programming\\v1
  \$env:GOOS='linux'; \$env:GOARCH='amd64'; \$env:CGO_ENABLED='0'; go build -o bin/offline-programming-linux .
  or run .\\offline-programming\\v1\\start-offline-programming.ps1"
}

ensure_dist() {
  if [[ -f "$DIST_INDEX" ]]; then
    log "built UI present at $DIST_INDEX"
    return 0
  fi
  log "ui/dist missing; writing a stub so serve can bind :8794 (open the Vite app URL)"
  mkdir -p "$OLP/ui/dist"
  cat >"$DIST_INDEX" <<EOF
<!DOCTYPE html>
<meta charset="utf-8">
<title>Offline Programming V1</title>
<meta http-equiv="refresh" content="0; url=${APP_URL}">
<p>API is on ${API_ORIGIN}. Open the app at <a href="${APP_URL}">${APP_URL}</a>.</p>
EOF
}

wait_for_health() {
  local waited=0
  while (( waited < 60 )); do
    if http_get "${API_ORIGIN}/api/offline-programming/v1/health" >/dev/null; then
      return 0
    fi
    sleep 1
    waited=$((waited + 1))
  done
  die "OLP API did not become healthy at ${API_ORIGIN}/api/offline-programming/v1/health"
}

start_or_reuse_serve() {
  if [[ "${OLP_RESTART_SERVE:-}" == "1" || "${OLP_REBUILT:-0}" == "1" ]]; then
    if http_get "${API_ORIGIN}/api/offline-programming/v1/health" >/dev/null || port_open 127.0.0.1 8794; then
      log "restarting OLP serve so jog auto-start and motion-server/v1 path take effect"
      stop_olp_serve
    fi
  fi
  if http_get "${API_ORIGIN}/api/offline-programming/v1/health" >/dev/null; then
    local actual
    actual="$(http_get "${API_ORIGIN}/api/offline-programming/v1/dense-execution/capabilities" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("backend", "unavailable"))')" || die 'cannot verify existing serve backend'
    [[ "$actual" == "$OFFLINE_PROGRAMMING_EXECUTION_BACKEND" ]] || die "existing serve uses $actual; restart it with the selected $OFFLINE_PROGRAMMING_EXECUTION_BACKEND backend"
    log "reusing OLP serve already healthy on ${LISTEN}"
    return 0
  fi
  if port_open 127.0.0.1 8794; then
    log "port 8794 is in use; waiting for health instead of starting another server"
    wait_for_health
    return 0
  fi
  log "starting OLP serve on ${LISTEN} with local Tesseract simulator"
  mkdir -p "$CACHE"
  local log_file="$CACHE/serve-wsl.log"
  nohup bash "$OLP/serve-wsl.sh" "$LISTEN" >"$log_file" 2>&1 &
  SERVE_PID=$!
  STARTED_SERVE=1
  log "serve pid ${SERVE_PID}; log ${log_file}"
  wait_for_health
}

stop_olp_serve() {
  pkill -f "offline-programming-linux serve" >/dev/null 2>&1 || true
  local waited=0
  while port_open 127.0.0.1 8794 && (( waited < 20 )); do
    sleep 1
    waited=$((waited + 1))
  done
}

wait_for_simulator() {
  local waited=0
  local body=""
  log "waiting up to ${SIM_WAIT_SECS}s for local Tesseract/Motion Server readiness"
  while (( waited <= SIM_WAIT_SECS )); do
    body="$(http_get "${API_ORIGIN}/api/offline-programming/v1/local-simulator/status" || true)"
    if [[ -n "$body" ]]; then
      if printf '%s' "$body" | python3 -c 'import json,sys
payload=json.load(sys.stdin)
status=payload.get("status") if isinstance(payload.get("status"), dict) else payload
ready=bool((status.get("readiness") or {}).get("ready"))
avail=bool((status.get("availability") or {}).get("available"))
sys.exit(0 if ready and avail else 1)'; then
        log "Tesseract / local simulator: READY (phase ready, no physical output)"
        return 0
      fi
      if (( waited % 15 == 0 )); then
        printf '%s' "$body" | python3 -c 'import json,sys
payload=json.load(sys.stdin)
status=payload.get("status") if isinstance(payload.get("status"), dict) else payload
avail=status.get("availability") or {}
ready=status.get("readiness") or {}
print("olp-start: simulator phase=%s available=%s ready=%s code=%s" % (
    status.get("phase"), avail.get("available"), ready.get("ready"), avail.get("code")))' || true
      fi
    fi
    sleep 3
    waited=$((waited + 3))
  done
  err "Tesseract / local simulator did not become ready within ${SIM_WAIT_SECS}s; API stays up for local sim once the worker finishes"
  if [[ -n "$body" ]]; then
    err "last local-simulator/status: $body"
  fi
  return 0
}

wait_for_jog() {
  local waited=0
  local body=""
  log "waiting up to 45s for isolated Motion Server jog (robot-v4-cartesiand)"
  while (( waited <= 45 )); do
    body="$(http_get "${API_ORIGIN}/api/offline-programming/v1/local-simulator/jog/state" || true)"
    if [[ -n "$body" ]]; then
      if printf '%s' "$body" | python3 -c 'import json,sys
payload=json.load(sys.stdin)
state=payload.get("state") if isinstance(payload.get("state"), dict) else {}
sys.exit(0 if payload.get("ok") and state.get("running") else 1)'; then
        log "Motion Server jog: LIVE (isolated cartesiand, backend $OFFLINE_PROGRAMMING_JOG_BACKEND, simulation)"
        return 0
      fi
    fi
    sleep 2
    waited=$((waited + 2))
  done
  err "Motion Server jog is not live yet; cartesian BASE/TOOL stays disabled until POST .../jog/start succeeds"
  if [[ -n "$body" ]]; then
    err "last jog/state: $body"
  fi
  return 0
}

jog_backend_summary() {
  log "jog backend: rt_core${ROSIE_RT_CONTROL_SOCKET:+ ($ROSIE_RT_CONTROL_SOCKET)}"
}

probe_robot() {
  jog_backend_summary
  log "${RT_REMOTE_SUMMARY:-dense backend: $OFFLINE_PROGRAMMING_EXECUTION_BACKEND}"
  local daemon_url="${OFFLINE_PROGRAMMING_DAEMON_STATE_URL:-}"
  if [[ -n "$daemon_url" ]]; then
    if http_get "$daemon_url" >/dev/null; then
      log "daemon: reachable at $daemon_url"
    else
      log "daemon: configured but not reachable at $daemon_url (local Tesseract sim continues)"
    fi
  else
    log "daemon: not configured (OFFLINE_PROGRAMMING_DAEMON_STATE_URL unset)"
  fi

  # Rails authoring ("Define welds from edges") proxies to the weld planner's
  # own dev servers. The API comes up healthy without them and every rails
  # action then fails in the UI, so say so here rather than let `api health ok`
  # read as "everything works".
  # Rails authoring is served from inside this process now: the seam worker
  # runs as one subprocess per request out of `weld_planner/v1`, so what can
  # be missing is that environment rather than another server. The capability
  # says which, because `api health ok` on its own used to read as "everything
  # works" while every rails action failed in the UI.
  local wp_motion="${OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN:-}"
  if python3 -c "
import json, sys, urllib.request
with urllib.request.urlopen('$API_ORIGIN/api/offline-programming/v1/capabilities', timeout=5) as answer:
    sys.exit(0 if json.load(answer).get('weld_planner', {}).get('enabled') else 1)
" 2>/dev/null; then
    log "rails: seam worker LIVE (weld_planner/v1, one process per request)"
  else
    log "rails: seam worker UNAVAILABLE -- provision the weld planner" \
        "environment: cd weld_planner/v1 && pixi install -e default." \
        "'Define welds from edges' will fail until then."
  fi
  if [[ -n "$wp_motion" ]]; then
    log "motion: $wp_motion"
  else
    log "motion: DISABLED -- set OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN" \
        "(usually http://127.0.0.1:8796) to plan and to play a dense trajectory."
  fi

  python3 - "$API_ORIGIN" <<'PY'
import json, sys, urllib.error, urllib.request

origin = sys.argv[1]
base = origin.rstrip("/") + "/api/offline-programming/v1"

def get(path):
    with urllib.request.urlopen(base + path, timeout=3) as response:
        return json.loads(response.read().decode("utf-8"))

try:
    health = get("/health")
except Exception as exc:
    print("olp-start: robot: no robot (health probe failed: %s)" % exc)
    print("olp-start: physical arming: DISARMED (connected execution is not enabled by this launcher)")
    sys.exit(0)

print(
    "olp-start: api health ok connected=%s local_ready=%s execution_enabled=%s"
    % (health.get("connected"), health.get("local_ready"), health.get("execution_enabled"))
)

try:
    discovery = get("/connected-targets")
except Exception as exc:
    print("olp-start: robot: no robot (connected-targets unavailable: %s)" % exc)
    print("olp-start: physical arming: DISARMED (this launcher never enables connected execution)")
    sys.exit(0)

targets = discovery.get("targets") or []
live = []
armed = []
for target in targets:
    profile = target.get("profile_id") or ""
    if not profile:
        continue
    try:
        status = get("/connected-targets/%s/status" % profile)
    except Exception:
        continue
    connected = bool(status.get("connected")) and status.get("state") == "read_only"
    if connected:
        live.append(profile)
    output = status.get("output") or {}
    fact = output.get("actual_servo_armed") or {}
    if fact.get("known") is True and fact.get("value") is True:
        armed.append(profile)

if live:
    print("olp-start: robot: CONNECTED (%s)" % ", ".join(live))
else:
    print("olp-start: robot: no robot (local Tesseract simulation continues)")

if armed:
    print(
        "olp-start: physical arming: observed ARMED on %s — this launcher still does not enable connected execution"
        % ", ".join(armed)
    )
else:
    print("olp-start: physical arming: DISARMED (connected execution is off; fail closed)")
PY
}

windows_host() {
  awk '/^nameserver / { print $2; exit }' /etc/resolv.conf 2>/dev/null || true
}

print_ui_url() {
  local win_host
  win_host="$(windows_host)"
  if port_open "$UI_HOST" "$UI_PORT" || http_get "$APP_URL" >/dev/null; then
    log "UI: Vite reachable from WSL at $APP_URL"
  elif [[ -n "$win_host" ]] && port_open "$win_host" "$UI_PORT"; then
    log "UI: Vite is listening on the Windows host (WSL loopback does not see :${UI_PORT})"
  elif ! grep -qi microsoft /proc/version 2>/dev/null; then
    # Native Linux desktop: there is no Windows host to be hiding Vite, so a
    # closed :5189 means it simply is not running. Say that instead of the WSL
    # advice, and do NOT offer the API origin as a fallback -- `ui/dist` is a
    # redirect stub back to :5189, so it bounces to this same dead port.
    log "UI: NOT RUNNING on :${UI_PORT} -- the app lives on Vite and the link below will refuse"
    log "UI: start it in another terminal: cd offline-programming/v1/ui && npm run dev"
  else
    log "UI: WSL cannot see :${UI_PORT}; if Vite is already running on Windows, use the URL below"
    log "UI: otherwise start it with .\\offline-programming\\v1\\start-offline-programming.ps1"
    log "UI: fallback PWA (if dist is built): ${API_ORIGIN}${UI_PATH}"
  fi
  printf '\n'
  log "Open the app: $APP_URL"
  log "API / Tesseract: ${API_ORIGIN}/api/offline-programming/v1/health"
}

cleanup() {
  if [[ "$STARTED_SERVE" -eq 1 && -n "$SERVE_PID" ]]; then
    if kill -0 "$SERVE_PID" >/dev/null 2>&1; then
      log "stopping serve we started (pid $SERVE_PID)"
      kill "$SERVE_PID" >/dev/null 2>&1 || true
      wait "$SERVE_PID" >/dev/null 2>&1 || true
    fi
  fi
  for (( i=${#RT_CHILDREN[@]}-1; i>=0; i-- )); do
    kill "${RT_CHILDREN[$i]}" 2>/dev/null || true
    wait "${RT_CHILDREN[$i]}" 2>/dev/null || true
  done
  if (( ${#RT_CHILDREN[@]} > 0 )); then
    rm -f "$ROSIE_LOCAL_RT_BUILD/sim.pid" "$ROSIE_LOCAL_RT_BUILD/control.pid"
  fi
}

usage() {
  cat <<EOF
usage: start-offline-programming.sh [--help]

Official WSL entry point for Offline Programming V1.
Starts or reuses OLP serve on ${LISTEN} with the local Tesseract simulator,
probes robot connection without blocking simulation, and prints $APP_URL.

This launcher never passes --enable-connected-execution.
Remote dense execution: OFFLINE_PROGRAMMING_RT_CORE_REMOTE=<binding.json>.
Select OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core (local simulation) explicitly.
EOF
}

main() {
  if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
  fi
  trap cleanup EXIT
  log "repository $ROOT"
  ensure_remote_rt_core
  ensure_local_rt_core
  if [[ -z "${OFFLINE_PROGRAMMING_CELLS:-}" && -z "${OFFLINE_PROGRAMMING_RT_CORE_REMOTE:-}" && "$OFFLINE_PROGRAMMING_EXECUTION_BACKEND" == rt_core && -f "${ROSIE_LOCAL_RT_BUILD:-}/olp-cells.json" ]]; then
    export OFFLINE_PROGRAMMING_CELLS="$ROSIE_LOCAL_RT_BUILD/olp-cells.json"
  fi
  if [[ -n "${RT_REMOTE_CONFIG:-}" ]]; then export OFFLINE_PROGRAMMING_RT_CORE_CONFIG="$RT_REMOTE_CONFIG"; fi
  ensure_planner
  ensure_pixi
  ensure_olp_linux
  ensure_dist
  start_or_reuse_serve
  wait_for_simulator
  wait_for_jog
  probe_robot
  print_ui_url
  if [[ "$STARTED_SERVE" -eq 1 && -n "$SERVE_PID" ]]; then
    log "serve running in foreground (Ctrl+C stops the server this script started; reused UI is left running)"
    wait "$SERVE_PID"
  fi
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then main "$@"; fi
