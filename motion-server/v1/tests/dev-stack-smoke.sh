#!/usr/bin/env bash
# Composed local cutover smoke. Native observations, not process uptime, prove
# one OLP Cartesian intent and one immutable dense program reached the simulator.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
: "${TMPDIR:?set TMPDIR to a Linux filesystem such as /dev/shm/rt-core/defaultflips}"
mkdir -p "$TMPDIR" "$ROOT/rt-core/build"
export DEV_STACK_BACKEND="${DEV_STACK_BACKEND:-rt_core}"
[[ "$DEV_STACK_BACKEND" == rt_core ]] || { echo 'backend_retired: select rt_core' >&2; exit 2; }
build="$(mktemp -d "$ROOT/rt-core/build/defaultflips-smoke.XXXXXX")"
export DEV_STACK_SMOKE=1 DEV_STACK_FIREWALL=0 DEV_STACK_UI_HOST=127.0.0.1
export ROSIE_LOCAL_RT_DIR="$(mktemp -d "$TMPDIR/ds.XXXXXX")"
export ROSIE_LOCAL_RT_BUILD="$build/runtime" ROSIE_DEV_STACK_DIR="${ROSIE_DEV_STACK_DIR:-$build/stack}"
export DEV_STACK_DENSE_BIN_DIR="$ROOT/rt-core/build/joint-trajectory"
export DEV_STACK_DENSE_INGEST=127.0.0.1:0
export OFFLINE_PROGRAMMING_JOG_BINARY="$ROOT/rt-core/build/cartesian/robot-v4-cartesiand"
cleanup() { bash "$ROOT/dev-stack.sh" stop rt-sim rt-control daemon; }
trap cleanup EXIT
make -C "$ROOT/rt-core" control sim
make -C "$ROOT/motion-server/v1" all BIN_DIR="$ROOT/rt-core/build/cartesian" OBJECT_CACHE_DIR="$ROOT/rt-core/build/cartesian-cache"
make -C "$ROOT/motion-server/joint-trajectory/v1" all BIN_DIR="$DEV_STACK_DENSE_BIN_DIR"
bash "$ROOT/dev-stack.sh" start rt-sim rt-control
bash "$ROOT/motion-server/v1/local-rt-core.sh" bind
source "$ROSIE_LOCAL_RT_BUILD/binding.env"
cd "$ROOT/offline-programming/v1"
# The reference dense program starts at zero. Execute it before jog changes the
# simulator pose; admission must keep rejecting mismatched starting positions.
go test -race -count=1 ./internal/simulator -run '^TestDevStackDenseProgram$' -v
go test -race -count=1 ./internal/simulator -run '^TestLocalRTCoreJog$' -v
echo 'dev-stack smoke: rt_core PASS (public jog motion and dense completion)'
