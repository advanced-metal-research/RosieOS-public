#!/usr/bin/env bash
# Produce one identified package from the source bundle; never build on the servo host.
set -euo pipefail
export LC_ALL=C
cd "${DAEMON_V1_BUILD_WORKSPACE:?}/rt-core"
read -r ROSIE_RT_SOURCE_SHA SOURCE_DATE_EPOCH < SOURCE_IDENTITY
[[ "${ROSIE_RT_SOURCE_SHA}" =~ ^[0-9a-f]{40}$ && "${SOURCE_DATE_EPOCH}" =~ ^[0-9]+$ ]]
export ROSIE_RT_SOURCE_SHA SOURCE_DATE_EPOCH
SR="${ROSIE_RT_SYSROOT:?manifest target sysroot required}"
export SR
export PATH="${HOME}/.rosie/bin:${PATH}"
export PKG_CONFIG_SYSROOT_DIR="${SR}"
export PKG_CONFIG_LIBDIR="${SR}/usr/local/lib/pkgconfig:${SR}/usr/lib/aarch64-linux-gnu/pkgconfig:${SR}/usr/share/pkgconfig"
pkg-config --exists 'libethercat = 1.6.9'
# Unverified build host prerequisite: the native rt-package manifest command still
# executes the target core for its source identity. Never register binfmt here.
test -r /proc/sys/fs/binfmt_misc/qemu-aarch64
grep -q '^enabled$' /proc/sys/fs/binfmt_misc/qemu-aarch64
export QEMU_LD_PREFIX="${SR}"
export QEMU_SET_ENV="LD_LIBRARY_PATH=${SR}/usr/local/lib:${SR}/usr/lib/aarch64-linux-gnu"
export CGO_ENABLED=0 GOOS=linux GOARCH=arm64
make live control ROSIE_RT_VERSION="${ROSIE_RT_SOURCE_SHA}" \
  CXX="aarch64-linux-gnu-g++-11 --sysroot=$SR -nostdinc++ -isystem $SR/usr/include/c++/12 -isystem $SR/usr/include/aarch64-linux-gnu/c++/12 -isystem $SR/usr/include/c++/12/backward -isystem $SR/usr/include -isystem $SR/usr/include/aarch64-linux-gnu -L$SR/usr/lib/aarch64-linux-gnu -B$SR/usr/lib/aarch64-linux-gnu -static-libgcc" \
  ECRT_LIBDIR= ECRT_LIBS="-L$SR/usr/local/lib -lethercat"
aarch64-linux-gnu-readelf -h build/rosie-rt-core | grep -F 'Machine:' | grep -F AArch64 >/dev/null
aarch64-linux-gnu-strings build/rosie-rt-core | grep -Fx "${ROSIE_RT_SOURCE_SHA}" >/dev/null
aarch64-linux-gnu-readelf -d build/rosie-rt-core | grep -F 'Shared library: [libethercat.so.1]' >/dev/null

scratch="$(mktemp -d "$PWD/build/package.XXXXXX")"
trap 'rm -rf -- "$scratch"' EXIT
mkdir -p "$scratch/helpers"
# Unverified provisioning prerequisite: the source bundle carries the two
# notices from rt-core's pinned IgH source tree; this recipe fetches nothing.
licenses="${ECRT_LICENSE_DIR:-$PWD/build/deps/ethercat-1.6.9}"
for notice in COPYING COPYING.LESSER; do
  test -s "$licenses/$notice"
done
go build -trimpath -o build/rt-package ./tools/packageinfo
GOOS="$(go env GOHOSTOS)" GOARCH="$(go env GOHOSTARCH)" go build -trimpath -o "$scratch/rt-package-host" ./tools/packageinfo

# The bundle carries the source commit and epoch, not a mutable .git directory.
# Restrict git answers to the packager's read-only identity queries.
cat > "$scratch/helpers/git" <<'GIT'
#!/usr/bin/env bash
set -eu
case "$*" in
  'rev-parse --git-dir') printf '%s\n' source-bundle ;;
  'rev-parse HEAD'|'describe --always --dirty') printf '%s\n' "$ROSIE_RT_SOURCE_SHA" ;;
  'status --porcelain --untracked-files=normal') ;;
  *) exit 1 ;;
esac
GIT
cat > "$scratch/helpers/uname" <<'UNAME'
#!/usr/bin/env bash
set -eu
if [[ "$*" == -m ]]; then printf '%s\n' aarch64; else exec /usr/bin/uname "$@"; fi
UNAME
cat > "$scratch/helpers/g++" <<'CXX'
#!/usr/bin/env bash
exec aarch64-linux-gnu-g++-11 "$@"
CXX
chmod 0755 "$scratch/helpers/"*
export PATH="$scratch/helpers:$PATH"
# Stage the package-runtime.sh payload explicitly: its GO override builds one
# executable for both shipping and execution, which cannot serve both hosts.
destination="$scratch/package"
install -d -m 0755 "$destination/bin" "$destination/lib" "$destination/licenses/igh" "$destination/host" "$destination/tools"
install -m 0755 build/rosie-rt-core build/rt-control build/rtctl build/rt-package build/rt-natspublisher "$destination/bin/"
install -m 0644 "$SR/usr/local/lib/libethercat.so.1" "$destination/lib/libethercat.so.1"
install -m 0644 "$licenses/COPYING" "$licenses/COPYING.LESSER" "$destination/licenses/igh/"
install -m 0755 tools/verify-package.sh "$destination/tools/verify-package.sh"
for script in install.sh generate-control-env.sh remote-pki.sh ethercat-foundation.sh run-core; do
  install -m 0755 "host/$script" "$destination/host/"
done
for unit in rosie-rt-core rosie-rt-control rosie-rt-natspublisher rosie-rt-crl; do
  sed -e 's|/opt/rosie/rt-core/build/|/opt/rosie-rt-core/current/bin/|g' \
      -e 's|/opt/rosie/rt-core|/opt/rosie-rt-core/current|g' "host/$unit.service" > "$destination/host/$unit.service"
  chmod 0644 "$destination/host/$unit.service"
done
install -m 0644 host/rosie-rt-crl.timer "$destination/host/rosie-rt-crl.timer"
for directory in config; do
  while IFS= read -r -d '' file; do
    install -D -m 0644 "$file" "$destination/$file"
  done < <(find "$directory" -type f ! -path '*/__pycache__/*' -print0 | sort -z)
done
# Match package-runtime.sh: include only each robot manifest and its registered files.
while IFS= read -r -d '' file; do
  install -D -m 0644 "${file}" "${destination}/${file#../}"
done < <(python3 - "$PWD/.." <<'PY'
import json, pathlib, sys
repository = pathlib.Path(sys.argv[1]).resolve()
for manifest in sorted((repository / "robot_description" / "robots").glob("*/robot_description_manifest.json")):
    model = manifest.parent
    for name in ["robot_description_manifest.json", *json.loads(manifest.read_text())["files"]]:
        sys.stdout.write("../robot_description/robots/%s/%s\0" % (model.name, name))
PY
)
"$scratch/rt-package-host" manifest "$destination" "$PWD" live
chmod 0644 "$destination/rt_package.json"
# Numeric ownership and fixed source epoch make the archive independent of
# traversal order, local accounts, staging paths and gzip header timestamps.
tar --sort=name --format=gnu --mtime="@$SOURCE_DATE_EPOCH" --owner=0 --group=0 --numeric-owner \
  -C "$scratch/package" -cf - . | gzip -n > "${DAEMON_V1_BUILD_OUTPUT:?}.tmp"
chmod 0755 "${DAEMON_V1_BUILD_OUTPUT}.tmp"
mv -f "${DAEMON_V1_BUILD_OUTPUT}.tmp" "${DAEMON_V1_BUILD_OUTPUT}"
