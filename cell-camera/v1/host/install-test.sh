#!/usr/bin/env bash
# install.sh in a throwaway container, systemd stubbed. Every camera instance
# shares /opt/rosie-cell-camera/current, so installing a camera without video
# must keep the tree's video sender in the revision it selects: otherwise the
# cell camera's rosie-cell-video@ loses its binary at its next start.
#   docker build -t rosie-camera-video cell-camera/v1/tests/video
#   docker run --rm -v "$PWD/cell-camera/v1":/src:ro rosie-camera-video bash /src/host/install-test.sh
set -euo pipefail
fail() { printf 'install-test: %s\n' "$*" >&2; exit 1; }
[[ ${EUID} == 0 && -d /src/host ]] || fail 'run in the container (see the header)'
d=$(mktemp -d)
mkdir -p "$d/tree/cell-camera" "$d/bin" "$d/stub"
cp -r /src "$d/tree/cell-camera/v1"
printf '#!/bin/sh\nexit 0\n' > "$d/bin/cell-video"
printf '#!/bin/sh\nexit 0\n' > "$d/stub/systemctl"
chmod +x "$d/bin/cell-video" "$d/stub/systemctl"
cat > "$d/wrist.env" <<'E'
CELL_CAMERA_DEVICE=/dev/v4l/by-id/usb-Arducam_Arducam_B0495__USB3_2.3MP__Arducam_202500915_0001-video-index0
CELL_CAMERA_CAPTURE=1920x1200@30
CELL_CAMERA_LISTEN=127.0.0.1:8089
E

PATH="$d/stub:$PATH" bash "$d/tree/cell-camera/v1/host/install.sh" wrist "$d/wrist.env" >/dev/null
revision=$(readlink /opt/rosie-cell-camera/current)
[[ -x /opt/rosie-cell-camera/$revision/bin/cell-video ]] ||
  fail "a camera without video selected revision $revision, which has no bin/cell-video"
echo 'install-test: ok'
