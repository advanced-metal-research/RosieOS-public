#!/usr/bin/env bash
# Install the cell camera on a cell host, and one camera instance from a
# settings file. Run as root on the host, from a checkout or a copied tree:
#   cell-camera/v1/host/install.sh <name> <settings.env> [--start] [--realsense-sdk DIR]
# It copies the package to /opt/rosie-cell-camera/<revision>, selects it as
# `current`, installs the template unit and /etc/rosie-cell-camera/<name>.env,
# and refuses settings the service itself would refuse. With video settings
# (CELL_CAMERA_VIDEO_ADDRESS) it also installs the WebRTC sender, the tree's
# bin/cell-video built by `realtime-core build`, as rosie-cell-video@<name>;
# without them that unit is stopped and disabled. The revision carries the
# tree's sender either way, for the other instances. Without --start the
# instances are installed and enabled but not (re)started. A RealSense
# (CELL_CAMERA_SOURCE=realsense) needs --realsense-sdk: the pyrealsense2
# package host/build-pyrealsense2.sh built for this host, copied into the
# revision beside the service, and the udev rule that lets the service's
# user open the camera.
set -euo pipefail
fail() { printf 'cell-camera install: %s\n' "$*" >&2; exit 1; }
usage='usage: install.sh <name> <settings.env> [--start] [--realsense-sdk DIR]'
[[ $# -ge 2 ]] || fail "$usage"
name=$1 settings=$2 start='' sdk=''
shift 2
while (( $# )); do
  case $1 in
    --start) start=--start; shift ;;
    --realsense-sdk) [[ $# -ge 2 && -n $2 ]] || fail "$usage"; sdk=$2; shift 2 ;;
    *) fail "$usage" ;;
  esac
done
[[ $name =~ ^[a-z0-9][a-z0-9-]{0,31}$ ]] || fail "name must be lower-case letters, digits and dashes, got '$name'"
[[ -f $settings && -r $settings ]] || fail "settings file not readable: $settings"
[[ ${EUID} == 0 ]] || fail 'run as root'
component=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
command -v python3 >/dev/null || fail 'python3 not found'
python3 -c 'import gi; gi.require_version("Gst", "1.0")' 2>/dev/null || fail 'python3 GStreamer bindings missing (python3-gi, gir1.2-gstreamer-1.0)'

# The service's own check, before anything on the host changes.
( cd "$component" && python3 -B - "$settings" <<'PY'
import sys
from cell_camera.config import ConfigError, load, read_env_file
try:
    c = load(read_env_file(open(sys.argv[1]).read()))
except ConfigError as error:
    sys.exit(f"cell-camera install: {sys.argv[1]}: {error}")
print(f"settings ok: {c.name}, {c.capture[0]}x{c.capture[1]}@{c.capture[2]} -> {c.output[0]}x{c.output[1]}, "
      f"http://{c.listen[0]}:{c.listen[1]}{c.path_prefix}/")
PY
) || exit 1

video=$(grep -q '^CELL_CAMERA_VIDEO_ADDRESS=.' "$settings" && echo yes || true)
sender="$component/../../../bin/cell-video"
if [[ -n $video ]]; then
  [[ -x $sender ]] || fail "video is configured but $sender is missing: build a tree with cell-camera/v1/video (realtime-core build)"
  # The sender's own check: its address must be one of this host's.
  # Only the keys it reads, which hold no spaces (NAME and TITLE may).
  env -i $(grep -E '^CELL_CAMERA_(VIDEO_[A-Z]+|LISTEN|PATH_PREFIX)=' "$settings") "$sender" -check || fail "cell-video refused $settings"
fi

realsense=$(grep -q '^CELL_CAMERA_SOURCE=realsense$' "$settings" && echo yes || true)
if [[ -n $realsense ]]; then
  [[ -n $sdk ]] || fail "a RealSense camera needs --realsense-sdk DIR, the pyrealsense2 package host/build-pyrealsense2.sh built on this host"
  [[ -f $sdk/BUILT_FROM && -f $sdk/pyrealsense2/__init__.py ]] || fail "$sdk is not a package from host/build-pyrealsense2.sh (no BUILT_FROM or pyrealsense2/)"
  # The service's own interpreter loads it here, so a package for another
  # Python or architecture, or a host without libusb, fails before anything changes.
  rs_version=$(cd "$sdk" && /usr/bin/python3 -B -c 'import pyrealsense2.pyrealsense2 as rs; print(rs.__version__)' 2>&1) ||
    fail "/usr/bin/python3 cannot load the pyrealsense2 in $sdk: $rs_version"
  printf 'realsense sdk ok: pyrealsense2 %s from %s (%s)\n' "$rs_version" "$sdk" "$(cat "$sdk/BUILT_FROM")"
elif [[ -n $sdk ]]; then
  fail "--realsense-sdk is only for a camera with CELL_CAMERA_SOURCE=realsense"
fi

revision=$(git -C "$component" rev-parse --short=12 HEAD 2>/dev/null || true)
[[ -n $revision ]] || revision=$(cd "$component" && find cell_camera -type f \( -name '*.py' -o -name '*.html' \) | sort | xargs sha256sum | sha256sum | cut -c1-12)
store=/opt/rosie-cell-camera
install -d -m 0755 "$store" "$store/$revision" "$store/$revision/cell_camera" /etc/rosie-cell-camera
install -m 0644 "$component"/cell_camera/*.py "$component"/cell_camera/index.html "$store/$revision/cell_camera/"
# The SDK goes beside the service (its working directory, the only tree the
# unit reads). Like the sender, a revision keeps the SDK the selected one
# had, so installing another camera does not take it from a RealSense.
prev=$(readlink "$store/current" 2>/dev/null || true)
if [[ -n $realsense ]]; then
  rm -rf "$store/$revision/pyrealsense2.new"
  cp -a "$sdk/pyrealsense2" "$store/$revision/pyrealsense2.new"
  install -m 0644 "$sdk/BUILT_FROM" "$store/$revision/pyrealsense2.new/BUILT_FROM"
  rm -rf "$store/$revision/pyrealsense2" && mv -T "$store/$revision/pyrealsense2.new" "$store/$revision/pyrealsense2"
elif [[ ! -d $store/$revision/pyrealsense2 && -n $prev && $prev != "$revision" && -d $store/$prev/pyrealsense2 ]]; then
  cp -a "$store/$prev/pyrealsense2" "$store/$revision/pyrealsense2"
fi
ln -sfn "$revision" "$store/current.new" && mv -Tf "$store/current.new" "$store/current"
install -m 0644 "$component/host/rosie-cell-camera@.service" /etc/systemd/system/rosie-cell-camera@.service
install -m 0644 "$component/host/rosie-cell-video@.service" /etc/systemd/system/rosie-cell-video@.service
# Every instance shares `current`, so the revision carries the sender whenever
# the tree has one, not only when this instance has video: installing a
# camera without video must not take the sender from one that has it.
if [[ -x $sender ]]; then
  install -d -m 0755 "$store/$revision/bin" && install -m 0755 "$sender" "$store/$revision/bin/cell-video"
fi
install -m 0644 "$settings" "/etc/rosie-cell-camera/$name.env"
if [[ -n $realsense ]]; then
  install -m 0644 "$component/host/99-rosie-realsense.rules" /etc/udev/rules.d/99-rosie-realsense.rules
  udevadm control --reload-rules
  udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=8086 --attr-match=idProduct=0b3a
  udevadm settle --timeout=5 || true
fi
systemctl daemon-reload
systemctl enable "rosie-cell-camera@$name.service" >/dev/null
if [[ -n $video ]]; then
  systemctl enable "rosie-cell-video@$name.service" >/dev/null
else
  systemctl disable --now "rosie-cell-video@$name.service" >/dev/null 2>&1 || true
fi
if [[ $start == --start ]]; then
  systemctl restart "rosie-cell-camera@$name.service"
  [[ -z $video ]] || systemctl restart "rosie-cell-video@$name.service"
  sleep 1
  systemctl is-active --quiet "rosie-cell-camera@$name.service" || fail "rosie-cell-camera@$name did not stay up: journalctl -u rosie-cell-camera@$name"
  [[ -z $video ]] || systemctl is-active --quiet "rosie-cell-video@$name.service" || fail "rosie-cell-video@$name did not stay up: journalctl -u rosie-cell-video@$name"
fi
printf 'installed %s as rosie-cell-camera@%s (%s)\n' "$revision" "$name" "$(systemctl is-active "rosie-cell-camera@$name.service" || true)"
[[ -z $video ]] || printf 'installed the video sender as rosie-cell-video@%s (%s)\n' "$name" "$(systemctl is-active "rosie-cell-video@$name.service" || true)"
