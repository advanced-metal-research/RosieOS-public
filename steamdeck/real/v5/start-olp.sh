#!/usr/bin/env bash
# The OLP server for the pendant: the same binary and planner routes as the
# desktop, headless, against the machines in olp-cells.json. ROSIE_RT_* stays
# unset on purpose: it enables OLP's disconnected local jog, whose daemon would
# own the simulation adapter's only controller slot and lock the pendant's
# machine connection out.
set -euo pipefail
base=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export ROSIE_REPO="$base/olp"
# Only a Local simulation entry needs the local binding. With a catalogue,
# serve reads any configured path and refuses to start when it is missing, so
# a Deck whose catalogue lists only remote cells must not name one.
if [[ -f "$base/olp-rt-core.json" ]]; then
  export OFFLINE_PROGRAMMING_RT_CORE_CONFIG="$base/olp-rt-core.json"
fi
export OFFLINE_PROGRAMMING_CELLS="$base/olp-cells.json"
export OFFLINE_PROGRAMMING_EXECUTION_BACKEND=rt_core
export OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core
export OFFLINE_PROGRAMMING_JOG_BINARY="$base/bin/robot-v4-cartesiand"
export LD_LIBRARY_PATH="$ROSIE_REPO/tesseract/v1/.pixi/envs/default/lib:$base/lib"
# Site file, never packaged: where planning and Load reach the weld planner's
# CUDA motion server (OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN), and how
# the seam worker is launched (SEAM_WORKER_PYTHON, SEAM_WORKER_SPARES). Without
# it the Run page shows the server's refusal and nothing is planned or loaded.
if [[ -f "$base/planner.env" ]]; then
  set -a
  # shellcheck source=/dev/null
  . "$base/planner.env"
  set +a
fi
cd "$ROSIE_REPO/offline-programming/v1"
exec ./bin/offline-programming-linux serve --listen 127.0.0.1:8794 --enable-local-simulator
