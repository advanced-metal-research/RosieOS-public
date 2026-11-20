#!/usr/bin/env bash
# Build the pinned userspace dependency under this component, without installing
# a kernel module, changing network configuration, or starting a service.
set -euo pipefail
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# A single profile table binds userspace to the kernel archive; unknown versions fail closed.
source "${component}/host/ethercat-foundation.sh"
print_pin=0
if [[ "${1:-}" == --print-pin ]]; then print_pin=1; shift; fi
[[ $# -le 1 ]] || { echo 'usage: build-igh-userlib.sh [--print-pin] [1.6.9|1.6.10|1.6.11|1.6.12]' >&2; exit 1; }
# unverified compatibility default: existing builds continue to select RevPi IgH 1.6.9.
foundation_profile "${ROSIE_RT_FOUNDATION_PROFILE:-revpi-debian12}"
version="${1:-${ROSIE_RT_IGH_VERSION:-${PINNED_IGH_VERSION}}}"
if [[ -n "${ROSIE_RT_FOUNDATION_PROFILE:-}" && "${version}" != "${PINNED_IGH_VERSION}" ]]; then
  echo 'IgH version disagrees with selected host profile' >&2; exit 1
fi
igh_pin "${version}"
deps="${component}/build/deps"
archive="${deps}/igh.tar.gz"
if [[ "${version}" != 1.6.9 ]]; then archive="${deps}/igh-${version}.tar.gz"; fi
source_dir="${deps}/ethercat-${version}"
prefix="${deps}/igh-prefix"
expected_sha="${PINNED_IGH_SHA256}"
if [[ "${print_pin}" == 1 ]]; then
  printf '%s %s %s\n' "${version}" "${expected_sha}" "${archive}"
  exit 0
fi
mkdir -p "${deps}"
if [[ ! -f "${archive}" ]]; then
  curl -fL --retry 3 "${PINNED_IGH_SOURCE_URL}" -o "${archive}.partial"
  printf '%s  %s\n' "${expected_sha}" "${archive}.partial" | sha256sum --check --status
  mv "${archive}.partial" "${archive}"
fi
printf '%s  %s\n' "${expected_sha}" "${archive}" | sha256sum --check --status
if [[ ! -d "${source_dir}" ]]; then
  tar -xzf "${archive}" -C "${deps}"
fi
cd "${source_dir}"
if [[ ! -x configure ]]; then bash bootstrap; fi
./configure --prefix="${prefix}" --disable-kernel --enable-userlib
make -C lib -j2
make -C lib install
make -C include install
printf '\nUserspace library installed under %s\n' "${prefix}"
printf 'Build: PKG_CONFIG_PATH="%s/lib/pkgconfig" make -C "%s" live\n' "${prefix}" "${component}"
