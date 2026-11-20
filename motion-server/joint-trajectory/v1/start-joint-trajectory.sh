#!/usr/bin/env bash
# A backend change retains the explicitly configured deployment identity.
set -euo pipefail
# Unverified repository default; native identity is supplied explicitly below.
backend="${JOINT_TRAJECTORY_BACKEND:-rt_core}"
refuse_backend() { echo "reason=backend_retired selection=$1; select rt_core" >&2; exit 2; }
[ "$backend" = rt_core ] || refuse_backend "$backend"
backend_value=0
for arg in "$@"; do
  if [ "$backend_value" = 1 ]; then
    [ "$arg" = rt_core ] || refuse_backend "$arg"
    backend_value=0
  else
    case "$arg" in
      --backend) backend_value=1 ;;
      --backend=*) [ "${arg#--backend=}" = rt_core ] || refuse_backend "${arg#--backend=}" ;;
    esac
  fi
done
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${JOINT_TRAJECTORY_BINARY:-$ROOT/build/bin/joint_trajectory_daemon}" \
  --config "${JOINT_TRAJECTORY_DAEMON_CONFIG:-$ROOT/config/joint_trajectory_daemon.config.json}" \
  --backend "$backend" "$@"
