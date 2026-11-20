#!/usr/bin/env bash
# Stage identified userspace artifacts; never change the kernel, services or network.
set -euo pipefail
export LC_ALL=C
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fail() { printf 'package error=%s\n' "$*" >&2; exit 1; }
[[ $# == 1 ]] || fail 'usage: [ROSIE_RT_PACKAGE_SLOT_ROOT=/absolute/slot] [ROSIE_RT_PACKAGE_BUILD_LOG=producer.log] package-runtime.sh DESTINATION'
[[ ! -L "$1" ]] || fail 'destination must not be a symlink'
destination="$(realpath -m -- "$1")"
[[ ! -e "${destination}" || ( -d "${destination}" && -z "$(ls -A -- "${destination}")" ) ]] || fail 'destination must be empty'
assets="${destination}.components"
[[ ! -e "${assets}" && ! -L "${assets}" ]] || fail 'component output already exists'
# unverified deployment default: existing single-directory installations select current.
slot_root="${ROSIE_RT_PACKAGE_SLOT_ROOT:-/opt/rosie-rt-core/current}"
[[ "${slot_root}" =~ ^(/[A-Za-z0-9_.-]+)+$ && "${slot_root}" != */. && "${slot_root}" != */.. && "${slot_root}" != */./* && "${slot_root}" != */../* ]] || fail 'slot root must be an absolute normalized path with safe systemd characters'
# Reject hidden executables before staging; keep the units' home isolation intact.
case "${slot_root}" in
  /home|/home/*|/root|/root/*|/run/user|/run/user/*)
    fail "slot root ${slot_root} is hidden by ProtectHome=yes" ;;
esac
if [[ -n "${SOURCE_DATE_EPOCH:-}" ]]; then
  [[ "${SOURCE_DATE_EPOCH}" =~ ^[0-9]+$ ]] || fail 'SOURCE_DATE_EPOCH must be integer seconds'
fi
producer_log="${ROSIE_RT_PACKAGE_BUILD_LOG:-}"
if [[ -n "${producer_log}" ]]; then
  producer_log="$(realpath -e -- "${producer_log}")"
  [[ -s "${producer_log}" && -f "${producer_log}" ]] || fail 'producer build log must be a nonempty regular file'
fi
core="${ROSIE_RT_PACKAGE_CORE:-${component}/build/rosie-rt-core}"
[[ "${core}" == /* ]] || core="${component}/${core}"
backend=live
if [[ -n "${ROSIE_RT_PACKAGE_CORE:-}" ]]; then
  [[ "$(realpath -e -- "${core}")" == "${component}/build/rosie-rt-core-sim" ]] || fail 'only build/rosie-rt-core-sim is an allowed substitution'
  backend=simulation
fi
for binary in "${core}" "${component}/build/rt-control" "${component}/build/rtctl"; do
  [[ -x "${binary}" ]] || fail 'build make live (or sim) and make control first'
done
if [[ "${backend}" == live ]]; then
  library="$(ldd "${core}" | awk '$1 == "libethercat.so.1" && $2 == "=>" && $3 ~ /^\// { print $3 }')"
else
  library="${component}/build/deps/igh-prefix/lib/libethercat.so.1"
fi
[[ -f "${library}" ]] || fail 'cannot resolve IgH libethercat.so.1; run tools/build-igh-userlib.sh'
# Read metadata beside the selected binary, preventing an unrelated prefix on
# PKG_CONFIG_PATH from labeling an external live library with the default pin.
library_pc="$(dirname "${library}")/pkgconfig/libethercat.pc"
[[ -f "${library_pc}" ]] || fail 'selected IgH library lacks adjacent pkg-config version metadata'
version="$(pkg-config --modversion "${library_pc}")"
bash "${component}/tools/build-igh-userlib.sh" --print-pin "${version}" >/dev/null || fail 'unsupported IgH library version'
export ROSIE_RT_IGH_VERSION="${version}"
licenses="${ECRT_LICENSE_DIR:-${component}/build/deps/ethercat-${version}}"
for notice in COPYING COPYING.LESSER; do
  [[ -f "${licenses}/${notice}" ]] || fail "missing IgH ${notice}; set ECRT_LICENSE_DIR to its source directory"
done
cd "${component}"
CGO_ENABLED=0 "${GO:-go}" build -trimpath -o build/rt-package ./tools/packageinfo
install -d -m 0755 "${destination}/bin" "${destination}/lib" "${destination}/licenses/igh" "${destination}/host" "${destination}/tools"
install -m 0755 "${core}" "${destination}/bin/rosie-rt-core"
install -m 0755 build/rt-control build/rtctl build/rt-package "${destination}/bin/"
if [[ -x build/rt-natspublisher ]]; then
  install -m 0755 build/rt-natspublisher "${destination}/bin/"
fi
install -m 0644 "${library}" "${destination}/lib/libethercat.so.1"
install -m 0644 "${licenses}/COPYING" "${licenses}/COPYING.LESSER" "${destination}/licenses/igh/"
install -m 0755 tools/verify-package.sh "${destination}/tools/verify-package.sh"
for script in install.sh generate-control-env.sh remote-pki.sh ethercat-foundation.sh run-core; do
  install -m 0755 "host/${script}" "${destination}/host/"
done
for unit in rosie-rt-core rosie-rt-control rosie-rt-natspublisher rosie-rt-crl; do
  sed -e 's|/opt/rosie/rt-core/build/|@ROSIE_RT_SLOT@/bin/|g' \
      -e 's|/opt/rosie/rt-core|@ROSIE_RT_SLOT@|g' \
      -e "s|@ROSIE_RT_SLOT@|${slot_root}|g" "host/${unit}.service" > "${destination}/host/${unit}.service"
  chmod 0644 "${destination}/host/${unit}.service"
done
install -m 0644 host/rosie-rt-crl.timer "${destination}/host/rosie-rt-crl.timer"
for directory in config; do
  while IFS= read -r -d '' file; do
    install -D -m 0644 "${file}" "${destination}/${file}"
  done < <(find "${directory}" -type f ! -path '*/__pycache__/*' -print0 | sort -z)
done
# Every robot description, exactly the files its manifest registers: a cell
# compiles its machine config from the package, without a checkout, and the
# identity over these files is what the machine config pins.
while IFS= read -r -d '' file; do
  install -D -m 0644 "${file}" "${destination}/${file#../}"
done < <(python3 - "${component}/.." <<'PY'
import json, pathlib, sys
repository = pathlib.Path(sys.argv[1]).resolve()
for manifest in sorted((repository / "robot_description" / "robots").glob("*/robot_description_manifest.json")):
    model = manifest.parent
    for name in ["robot_description_manifest.json", *json.loads(manifest.read_text())["files"]]:
        sys.stdout.write("../robot_description/robots/%s/%s\0" % (model.name, name))
PY
)
build/rt-package manifest "${destination}" "${component}" "${backend}"
if [[ -n "${SOURCE_DATE_EPOCH:-}" ]]; then
  find "${destination}" -exec touch -h -d "@${SOURCE_DATE_EPOCH}" {} +
fi
build/rt-package component "${destination}" "${assets}" "${producer_log}"
printf 'package status=staged backend=%s\n' "${backend}"
printf 'component status=staged output=%s\n' "${assets}"
