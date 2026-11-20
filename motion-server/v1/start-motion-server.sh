#!/usr/bin/env bash
# Explicit launch bindings prevent a repository default from guessing a cell identity.
set -euo pipefail
# Unverified repository default; native identity is supplied explicitly below.
backend="${MOTION_SERVER_BACKEND:-rt_core}"
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
binary="${MOTION_SERVER_BINARY:-$HOME/.rosie/motion-server/v1/bin/robot-v4-cartesiand}"
args=(--backend "$backend")
case "$backend" in
  rt_core)
    args+=(--rt-control-socket "${ROSIE_RT_CONTROL_SOCKET:?required}" --rt-jog-socket "${ROSIE_RT_JOG_SOCKET:?required}"
      --rt-pair-id "${ROSIE_RT_PAIR_ID:?required}" --rt-pair-revision "${ROSIE_RT_PAIR_REVISION:?required}"
      --rt-configuration-sha256 "${ROSIE_RT_CONFIGURATION_SHA256:?required}"
      --rt-axis-ids "${ROSIE_RT_AXIS_IDS:?required}" --rt-urdf "${ROSIE_RT_URDF:?required}"
      --rt-urdf-sha256 "${ROSIE_RT_URDF_SHA256:?required}") ;;
esac
exec "$binary" "${args[@]}" "$@"
