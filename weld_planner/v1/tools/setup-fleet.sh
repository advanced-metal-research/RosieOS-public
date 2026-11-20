#!/usr/bin/env bash
# Replicate the motion planner onto a fleet machine (Jetson Thor / jetson-N).
#
# Run FROM a tailnet workstation with the repo checked out:
#
#     weld_planner/v1/tools/setup-fleet.sh root@<host>
#
# The target is required: the fleet's host names are site data, so the
# script names none.
#
# The fleet answers TAILSCALE SSH AS root@ ONLY -- any other user is refused
# with "tailnet policy does not permit...", which reads like a missing
# account and is not. No password, no key: the tailnet identity is the
# authentication.
#
# These are SHARED machines (other people's homes and long-running jobs live
# there), so everything this script touches is contained: the repo subtree
# under /root/RosieOS, pixi under /root/.rosie, package cache under
# /root/.cache/rattler. Nothing system-wide is installed or upgraded.
#
# Pixi installs the source-owned `motion-thor` dependency prefix. The pinned
# NVIDIA container owns the sm_110 Torch build; conda Torch does not.
set -euo pipefail

if [[ $# -lt 1 || $1 != *@* ]]; then
  echo "usage: $0 root@<host>   (a Jetson Thor or Jetson fleet machine on the tailnet)" >&2
  exit 2
fi
TARGET=$1
DEST=/root/RosieOS
HERE=$(cd "$(dirname "$0")/.." && pwd)          # weld_planner/v1
REPO=$(cd "$HERE/../.." && pwd)                  # RosieOS
SSH=(ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=15)

say() { printf '\nsetup-fleet: %s\n' "$*"; }

say "target $TARGET, dest $DEST"
"${SSH[@]}" "$TARGET" "uname -m | grep -q aarch64" \
  || { echo "setup-fleet: $TARGET is not aarch64; this script ships the aarch64 lock" >&2; exit 1; }

say "syncing weld_planner/v1 and urdf (env, caches and scratch excluded)"
"${SSH[@]}" "$TARGET" "mkdir -p $DEST/weld_planner/v1 $DEST/urdf"
RSYNC=(rsync -az --info=progress2 -e "${SSH[*]}")
"${RSYNC[@]}" --delete \
  --exclude '.pixi' --exclude '__pycache__' --exclude '.pytest_cache' \
  --exclude 'workspace/' --exclude '*.pth' \
  "$HERE/" "$TARGET:$DEST/weld_planner/v1/"
"${RSYNC[@]}" --delete "$REPO/urdf/" "$TARGET:$DEST/urdf/"

say "pixi (static aarch64 binary under /root/.rosie/bin; nothing system-wide)"
"${SSH[@]}" "$TARGET" '
  set -euo pipefail
  mkdir -p /root/.rosie/bin
  if ! /root/.rosie/bin/pixi --version >/dev/null 2>&1; then
    curl -fsSL -o /root/.rosie/bin/pixi \
      https://github.com/prefix-dev/pixi/releases/latest/download/pixi-aarch64-unknown-linux-musl
    chmod +x /root/.rosie/bin/pixi
  fi
  /root/.rosie/bin/pixi --version'

say "installing the motion-thor dependency prefix from the shipped lock"
"${SSH[@]}" "$TARGET" \
  "cd $DEST/weld_planner/v1 && /root/.rosie/bin/pixi install -e motion-thor --locked"

say "smoke: what this machine offers the planner"
"${SSH[@]}" "$TARGET" \
  "cd $DEST/weld_planner/v1 && PIXI_BIN=/root/.rosie/bin/pixi tools/run_thor_motion.sh -m weld_motion_planner.utils.device"

say "smoke: the torch-free workers"
"${SSH[@]}" "$TARGET" \
  "cd $DEST/weld_planner/v1 && PIXI_BIN=/root/.rosie/bin/pixi tools/run_thor_motion.sh -m weld_motion_planner.io.plan_request --self-test" \
  || say "worker self-test FAILED -- read the output above before benchmarking"

cat <<NEXT

setup-fleet: done. On $TARGET:
  cd $DEST/weld_planner/v1
  PIXI_BIN=/root/.rosie/bin/pixi tools/run_thor_motion.sh -m pytest -q tests/motion_planner
  PIXI_BIN=/root/.rosie/bin/pixi tools/run_thor_motion.sh tools/bench_plan.py \\
      data/motion/bracket_b_2x.weldplan --runs 5           # the standing report
NEXT
