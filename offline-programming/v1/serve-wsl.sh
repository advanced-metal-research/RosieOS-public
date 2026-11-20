#!/usr/bin/env bash
# Serve Offline Programming V1 under WSL with the Linux Motion Server planner
# and pixi Tesseract collision worker. Native Windows `go run` cannot execute
# the ELF planner binary.
#
# Suite entry point (UI + serve + robot probe):
#   ./start-offline-programming.sh
#   .\start-offline-programming.ps1
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OLP="$ROOT/offline-programming/v1"
LISTEN="${1:-127.0.0.1:8794}"
SERVER_BIN="$OLP/bin/offline-programming-linux"
PLANNER_BIN="$ROOT/motion-server/v1/bin/robot-v4-cartesiand"
PIXILIB="$ROOT/tesseract/v1/.pixi/envs/default/lib"
PY="$ROOT/tesseract/v1/.pixi/envs/default/bin/python3"

# Under WSL HOME can point elsewhere; use the login's Linux home when it exists.
user_home="/home/$(id -un)"
if [[ -d "$user_home" ]]; then
  export HOME="$user_home"
fi
CACHE="${ROSIEOS_OLP_CACHE:-}"
if [[ -z "$CACHE" ]]; then
  for candidate in "${HOME}/.cache/rosieos-olp" "$user_home/.cache/rosieos-olp"; do
    if [[ -d "$candidate" ]]; then
      CACHE="$candidate"
      break
    fi
  done
fi

if [[ ! -x "$PLANNER_BIN" && -n "${CACHE:-}" && -x "$CACHE/motion-server-bin/robot-v4-cartesiand" ]]; then
  mkdir -p "$(dirname "$PLANNER_BIN")"
  ln -sfn "$CACHE/motion-server-bin/robot-v4-cartesiand" "$PLANNER_BIN"
fi

if [[ ! -x "$PLANNER_BIN" ]]; then
  echo "planner missing; building robot-v4-cartesiand" >&2
  (
    cd "$ROOT/motion-server/v1"
    BIN_DIR="$ROOT/motion-server/v1/bin" \
      OBJECT_CACHE_DIR="$OLP/.tmp-motion-cache" \
      make -j"$(nproc)" all
  )
fi

if [[ ! -x "$SERVER_BIN" && -n "${CACHE:-}" && -x "$CACHE/olp-bin/offline-programming-linux" ]]; then
  mkdir -p "$(dirname "$SERVER_BIN")"
  ln -sfn "$CACHE/olp-bin/offline-programming-linux" "$SERVER_BIN"
fi

if [[ ! -x "$SERVER_BIN" ]] && command -v go >/dev/null 2>&1; then
  echo "building offline-programming-linux" >&2
  mkdir -p "$(dirname "$SERVER_BIN")"
  (cd "$OLP" && CGO_ENABLED=0 GOOS=linux GOARCH="$(go env GOHOSTARCH)" go build -o "$SERVER_BIN" .)
fi

if [[ ! -x "$SERVER_BIN" ]]; then
  echo "cross-compile offline-programming-linux from Windows first:" >&2
  echo "  cd offline-programming/v1" >&2
  echo "  \$env:GOOS='linux'; \$env:GOARCH='amd64'; \$env:CGO_ENABLED='0'; go build -o bin/offline-programming-linux ." >&2
  echo "or run ./start-offline-programming.sh / .\\start-offline-programming.ps1" >&2
  exit 1
fi

if [[ ! -x "$PY" || ! -d "$PIXILIB" ]]; then
  echo "Tesseract pixi env missing under tesseract/v1; run:" >&2
  echo "  cd tesseract/v1 && pixi install --locked" >&2
  echo "or ./start-offline-programming.sh which installs pixi if needed" >&2
  exit 1
fi

export LD_LIBRARY_PATH="$PIXILIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cd "$OLP"
exec "$SERVER_BIN" serve --listen "$LISTEN" --enable-local-simulator
