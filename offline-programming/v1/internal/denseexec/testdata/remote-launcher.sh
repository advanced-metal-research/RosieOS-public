#!/usr/bin/env bash
# Assert launcher output and independent jog binding; TLS discovery is supplied by the Go harness.
set -euo pipefail
root="$1" build="$2" binding="$3" scenario="$4"
export ROSIEOS_OLP_CACHE="$build/cache" OFFLINE_PROGRAMMING_EXECUTION_BACKEND=rt_core
export OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core OFFLINE_PROGRAMMING_RT_CORE_REMOTE="$binding"
[[ "$scenario" != implicit_jog ]] || unset OFFLINE_PROGRAMMING_JOG_BACKEND
source "$root/offline-programming/v1/start-offline-programming.sh"
ROOT="$build"
# Keep discovery/binding startup real while excluding unrelated planner and UI processes.
for helper in ensure_planner ensure_pixi ensure_olp_linux ensure_dist start_or_reuse_serve wait_for_simulator wait_for_jog probe_robot print_ui_url; do
  eval "$helper() { :; }"
done
if [[ "$scenario" == success ]]; then
  export ROSIE_LOCAL_RT_BUILD="$build/local"
  mkdir -p "$ROSIE_LOCAL_RT_BUILD"
  printf 'export ROSIE_RT_CONTROL_SOCKET=/local/control.sock\nexport OFFLINE_PROGRAMMING_RT_CORE_CONFIG=/local/olp-rt-core.json\n' > "$ROSIE_LOCAL_RT_BUILD/binding.env"
  curl() { [[ "$*" == *'/local/control.sock'* ]]; }
  main
  [[ "$RT_REMOTE_SUMMARY" == "dense backend: rt_core (remote https://"*" pair bench-rosie1420 configuration "* ]]
  [[ "$ROSIE_RT_CONTROL_SOCKET" == /local/control.sock && "$OLP_RESTART_SERVE" == 1 ]]
  python3 - "$OFFLINE_PROGRAMMING_RT_CORE_CONFIG" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
assert 'socket' not in d
assert d['axis_mask'] == 63 and d['binding']['revision'] == 1
assert d['binding']['machine_sha256'] == 'a' * 64
assert d['binding']['configuration_sha256'] == 'b' * 64
assert d['deployment_sha256'] == 'c' * 64
assert d['capabilities_digest'] == 'd' * 64
assert d['expected_backend'] == 'ethercat'
PY
else
  case "$scenario" in
    pair_mismatch) reason=remote_certificate_pair_mismatch ;;
    hostile_curlrc) reason=remote_describe_failed ;;
    describe_failure) reason=remote_describe_failed ;;
    backend_mismatch) reason=remote_backend_mismatch ;;
    missing_credential) reason='remote_credential_missing: key_file' ;;
    implicit_jog) reason=remote_jog_backend_required ;;
  esac
  if (main) >"$build/result" 2>&1; then exit 1; fi
  grep -F "$reason" "$build/result"
  [[ -z "$(find "$build" -name olp-rt-core.json -print)" ]]
fi
