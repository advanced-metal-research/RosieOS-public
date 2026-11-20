#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd "$ROOT/../.." && pwd)"
IMAGE="${ROSIE_THOR_IMAGE:-nvcr.io/nvidia/pytorch@sha256:edbbf7492e7a9111974a0e58debbc51371b131e19acf4aa033577cc39d4399a5}"
PIXI_BIN="${PIXI_BIN:-pixi}"
TF32_OVERRIDE="${TORCH_ALLOW_TF32_CUBLAS_OVERRIDE:-0}"

if [[ "$(uname -m)" != "aarch64" ]]; then
  echo "Thor motion runner requires linux-aarch64" >&2
  exit 2
fi
if ! command -v "$PIXI_BIN" >/dev/null 2>&1; then
  echo "Pixi is required to install the locked motion-thor prefix" >&2
  exit 2
fi
if [[ $# -eq 0 ]]; then
  echo "usage: tools/run_thor_motion.sh <python arguments...>" >&2
  echo "example: tools/run_thor_motion.sh -m weld_motion_planner.utils.device" >&2
  exit 2
fi
if [[ "$TF32_OVERRIDE" != "0" && "$TF32_OVERRIDE" != "1" ]]; then
  echo "TORCH_ALLOW_TF32_CUBLAS_OVERRIDE must be 0 or 1" >&2
  exit 2
fi

"$PIXI_BIN" install --manifest-path "$ROOT/pixi.toml" -e motion-thor --locked

docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then
  docker_cmd=(sudo -n docker)
fi
if ! "${docker_cmd[@]}" image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Pinned NVIDIA Thor image is not present locally; refusing to pull: $IMAGE" >&2
  exit 3
fi

docker_args=(
  run --rm --pull=never --runtime=nvidia --ipc=host
  -v "$REPO_ROOT:/repo:ro"
  -v "$ROOT:/repo/weld_planner/v1"
  -w /repo/weld_planner/v1
  -e PYTHONPATH=/repo/weld_planner/v1/python:/usr/local/lib/python3.12/dist-packages
  -e WELD_REPO_ROOT=/repo
  -e TORCH_ALLOW_TF32_CUBLAS_OVERRIDE="$TF32_OVERRIDE"
)
if [[ -n "${ROSIE_THOR_EVIDENCE_ROOT:-}" ]]; then
  if [[ "${ROSIE_THOR_EVIDENCE_ROOT}" != /* ]]; then
    echo "ROSIE_THOR_EVIDENCE_ROOT must be absolute" >&2
    exit 2
  fi
  mkdir -p "$ROSIE_THOR_EVIDENCE_ROOT"
  docker_args+=( -v "$ROSIE_THOR_EVIDENCE_ROOT:/evidence" )
fi

python=/repo/weld_planner/v1/.pixi/envs/motion-thor/bin/python
"${docker_cmd[@]}" "${docker_args[@]}" "$IMAGE" "$python" -c \
  'import torch; x=torch.ones((8,8),device="cuda"); y=x@x; torch.cuda.synchronize(); assert float(y[0,0]) == 8.0'

exec "${docker_cmd[@]}" "${docker_args[@]}" "$IMAGE" "$python" "$@"
