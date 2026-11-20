#!/usr/bin/env bash

# Bind selectable archives to measured digests; unknown versions cannot reach a build.
igh_pin() {
  PINNED_IGH_VERSION="$1"
  # Measured SHA-256 values of the upstream GitLab release archives.
  case "$1" in
    1.6.9) PINNED_IGH_SHA256=2bc4d6b69ed980b896ecae9ea6603b327562dfacd3b8c1ed4e9e599a4cc4dfc3 ;;
    1.6.10) PINNED_IGH_SHA256=b31387b19f8e1d8ced5d4c43f818ffa77a6333c2627782490d1dd1799391d1cc ;;
    1.6.11) PINNED_IGH_SHA256=a68e79603a227ad8f3aca75cdf6abca663a3b7ca50a96faaf1ba020772fde558 ;;
    1.6.12) PINNED_IGH_SHA256=6aa49381da3daf08131913c3fade8fd24b650836567459ef1d35991da1549310 ;;
    *) printf 'unsupported IgH version %s\n' "$1" >&2; return 1 ;;
  esac
  PINNED_IGH_SOURCE_URL="https://gitlab.com/etherlab.org/ethercat/-/archive/${PINNED_IGH_VERSION}/ethercat-${PINNED_IGH_VERSION}.tar.gz"
}

# Select only reviewed tuples; sourcing exposes pins without provisioning a host.
foundation_profile() {
  # Measured archive SHA-256 values from the upstream GitLab tag archives.
  case "$1" in
    revpi-debian12|rosie-rt-core-revpi-igh-v1)
      FOUNDATION_PROFILE=revpi-debian12
      PINNED_OS_VERSION=12
      igh_pin 1.6.9
      HEADER_PACKAGE="linux-headers-${2:-}"
      # Measured 2026-11-13: under 6.12.91-revpi0 two RevPi servo hosts
      # ran the 1 ms cycle with 0.89-1.1 ms worst jitter and lost a process
      # data frame about hourly (29 "datagrams UNMATCHED" in 21 h on
      # one of them); each loss latched coordinate_reference on every joint
      # (issue #177). A third servo host on 6.12.56-revpi1: 65 us, 3 in 20 days.
      PINNED_KERNEL=6.12.56-revpi1-rpi-v8
      PINNED_KERNEL_VERSION=6.12.56-revpi1-1+deb12+1
      # piControl ships one module package per kernel, and the picontrol
      # metapackage depends on exactly one: unpinned, 2.8.2 pulled in
      # 6.18.57 and its kernel (a RevPi servo host, 2026-11-13). The 6.12.56 host's set.
      PINNED_PICONTROL_VERSION=2.5.0-1+deb12+1
      CONFIGURE_DRIVERS=(--enable-generic)
      ;;
    rpi5-debian13)
      FOUNDATION_PROFILE=rpi5-debian13
      PINNED_OS_VERSION=13
      igh_pin 1.6.12
      # Measured package name: Raspberry Pi trixie arm64 Packages index, 2026-09-15.
      HEADER_PACKAGE=linux-headers-rpi-v8-rt
      PINNED_KERNEL=""
      PINNED_KERNEL_VERSION=""
      PINNED_PICONTROL_VERSION=""
      # Measured provisioned Pi flags: hrtimer enabled; cycles fails on arm64 (cpu_khz).
      CONFIGURE_DRIVERS=(--enable-generic --enable-hrtimer)
      if [[ "${ROSIE_RT_FOUNDATION_DEVICE_MODULES:-generic}" == macb ]]; then
        CONFIGURE_DRIVERS+=(--enable-macb)
      fi
      ;;
    *) printf 'unsupported foundation profile %s\n' "$1" >&2; return 1 ;;
  esac
}

# Match OS and kernel together so an unrelated Debian host cannot be provisioned.
foundation_detect_profile() {
  case "$1:$2:$3" in
    debian:12:*-revpi*-rpi-v8) printf 'revpi-debian12\n' ;;
    debian:13:6.18.*-rpi-v8-rt) printf 'rpi5-debian13\n' ;;
    *) printf 'unsupported host %s %s %s\n' "$1" "$2" "$3" >&2; return 1 ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then return 0; fi

set -Eeuo pipefail
umask 077
export LC_ALL=C

readonly RECEIPT_SCHEMA="rosie-rt-core.ethercat-foundation.receipt.v1"
# unverified device-group default; install.sh uses the same deployment variable.
readonly ETHERCAT_DEVICE_GROUP="${ROSIE_RT_ETHERCAT_GROUP:-ethercat}"
# A RevPi's host services and EtherCAT drop-in, from foundation/ beside this
# script. Every realtime-core deploy refuses a servo host without the drop-in
# (steamdeck/real/v4 rt_core_relay_deploy.go preflight). A deployed servo host runs
# byte-identical copies, installed there by the retired v4 stack (checked
# 2026-11-13); before this, nothing installed them on a new RevPi.
readonly FOUNDATION_FILES_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/foundation"
readonly REVPI_HOST_SERVICES=(robot-v4-cpu-performance robot-v4-ethercat-nic-tune robot-v4-irq-affinity)
readonly REVPI_DEVICE_DROPIN=ethercat.service.d/20-robot-v4-device-permissions.conf

fail() {
  printf 'ethercat-foundation error=%s\n' "$*" >&2
  exit 1
}

# Grant access only to the selected device group; repeated installs leave a matching rule intact.
configure_ethercat_device_access() {
  [[ "${ETHERCAT_DEVICE_GROUP}" =~ ^[a-z_][a-z0-9_-]*$ ]] || fail "invalid ROSIE_RT_ETHERCAT_GROUP"
  getent group "${ETHERCAT_DEVICE_GROUP}" >/dev/null || groupadd --system "${ETHERCAT_DEVICE_GROUP}"
  local rule content
  rule="$(host_path /etc/udev/rules.d/99-rosie-rt-core-ethercat.rules)"
  # unverified permission policy: only root and the fieldbus group can open the master device.
  content="KERNEL==\"EtherCAT0\", GROUP=\"${ETHERCAT_DEVICE_GROUP}\", MODE=\"0660\""
  if [[ -f "${rule}" ]] && cmp -s "${rule}" <(printf '%s\n' "${content}"); then
    return
  fi
  # unverified permission policy: udev configuration is root-owned and publicly readable.
  install -d -m 0755 "$(dirname "${rule}")"
  install -m 0644 -o root -g root <(printf '%s\n' "${content}") "${rule}"
  udevadm control --reload-rules
  udevadm trigger --action=change --sysname-match=EtherCAT0
}

require_env() {
  local name="$1"
  [[ -n "${!name:-}" ]] || fail "missing required environment variable ${name}"
}

ACTION="${1:-}"
[[ $# -eq 1 ]] || fail "usage: ethercat-foundation.sh inspect|install|verify|kernel"
case "${ACTION}" in
  inspect|install|verify|kernel) ;;
  *) fail "action must be inspect, install, verify, or kernel" ;;
esac

# The alternate root is a deliberately narrow test seam. A non-root process can
# only use it beneath a temporary directory, and an install that would mutate
# that sandbox is rejected. Production always uses the real host root as root.
TESTING="${ROSIE_RT_FOUNDATION_TESTING:-0}"
HOST_ROOT="/"
if [[ "${TESTING}" == "1" ]]; then
  HOST_ROOT="${ROSIE_RT_FOUNDATION_TEST_ROOT:-}"
  [[ "${HOST_ROOT}" == "${TMPDIR:-/tmp}/rosie-rt-core-foundation-test."* ]] || fail "test root must be a dedicated ${TMPDIR:-/tmp}/rosie-rt-core-foundation-test.* directory"
  [[ -d "${HOST_ROOT}" ]] || fail "test root does not exist"
  HOST_ROOT="${HOST_ROOT%/}"
elif [[ -n "${ROSIE_RT_FOUNDATION_TEST_ROOT:-}" ]]; then
  fail "alternate host root requires ROSIE_RT_FOUNDATION_TESTING=1"
fi
if [[ ${EUID} -ne 0 && "${TESTING}" != "1" ]]; then
  fail "must run as root"
fi

host_path() {
  local path="$1"
  [[ "${path}" == /* ]] || fail "internal host path must be absolute: ${path}"
  if [[ "${HOST_ROOT}" == "/" ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s%s\n' "${HOST_ROOT}" "${path}"
  fi
}

# The firmware boots /boot/firmware/kernel8.img and initramfs8, byte copies
# of /boot/vmlinuz-<release> and /boot/initrd.img-<release> (two RevPi
# servo hosts, 2026-11-13); a kernel_2712.img would take precedence on a CM5.
boot_kernel_is_pinned() {
  [[ ! -e "$(host_path /boot/firmware/kernel_2712.img)" ]] &&
    cmp -s "$(host_path /boot/firmware/kernel8.img)" "$(host_path "/boot/vmlinuz-${PINNED_KERNEL}")" &&
    cmp -s "$(host_path /boot/firmware/initramfs8)" "$(host_path "/boot/initrd.img-${PINNED_KERNEL}")"
}

# Install the pinned kernel and headers, hold them, and make them the boot
# kernel. The raspi-firmware hook copies only the highest installed version to
# /boot/firmware, so every other RevPi kernel package is purged; the running
# kernel keeps running until the reboot, which needs the owner's approval.
install_pinned_kernel() {
  [[ -n "${PINNED_KERNEL}" ]] || fail "profile ${FOUNDATION_PROFILE} pins no kernel"
  [[ "${TESTING}" != "1" ]] || fail "test mode refuses a kernel install"
  local image="linux-image-${PINNED_KERNEL}" headers="linux-headers-${PINNED_KERNEL}"
  local module="picontrol-${PINNED_KERNEL}" others
  local held=(linux-image-revpi-v8 "${image}" "${headers}" picontrol picontrol-common "${module}")
  export DEBIAN_FRONTEND=noninteractive
  apt-mark unhold "${held[@]}" >/dev/null 2>&1 || true
  apt-get update
  apt-get install -y --allow-downgrades --no-install-recommends \
    "${image}=${PINNED_KERNEL_VERSION}" "${headers}=${PINNED_KERNEL_VERSION}" \
    "linux-image-revpi-v8=${PINNED_KERNEL_VERSION}" "picontrol=${PINNED_PICONTROL_VERSION}" \
    "picontrol-common=${PINNED_PICONTROL_VERSION}" "${module}=${PINNED_PICONTROL_VERSION}"
  apt-mark hold "${held[@]}" >/dev/null
  others="$(dpkg-query -W -f='${Package} ${Status}\n' 'linux-image-*-revpi*' 'linux-headers-*-revpi*' 'picontrol-*-revpi*' 2>/dev/null |
    awk -v image="${image}" -v headers="${headers}" -v module="${module}" \
      '$NF == "installed" && $1 != image && $1 != headers && $1 != module && $1 != "linux-image-revpi-v8" {print $1}' || true)"
  if [[ -n "${others}" ]]; then
    # shellcheck disable=SC2086 # one package name per word
    apt-get purge -y ${others}
  fi
  # The pinned image's hooks ran while a higher version was installed and
  # skipped the boot partition; run them again now that it is the highest.
  dpkg-reconfigure -f noninteractive "${image}"
  boot_kernel_is_pinned || fail "/boot/firmware does not hold ${PINNED_KERNEL} after the install"
  others="$(dpkg-query -W -f='${Package} ${Status}\n' 'linux-image-*-revpi*' 2>/dev/null |
    awk -v image="${image}" '$NF == "installed" && $1 != image && $1 != "linux-image-revpi-v8" {print $1}' || true)"
  [[ -z "${others}" ]] || fail "another kernel is still installed: ${others}"
  local running
  running="$(uname -r)"
  if [[ "${running}" == "${PINNED_KERNEL}" ]]; then
    printf 'ethercat_foundation_kernel=current pinned=%s running=%s held=1\n' "${PINNED_KERNEL}" "${running}"
  else
    printf 'ethercat_foundation_kernel=staged pinned=%s running=%s held=1 reboot_required=1 next="reboot (owner approval), then ethercat-foundation.sh install"\n' "${PINNED_KERNEL}" "${running}"
  fi
}

OS_RELEASE_PATH="$(host_path /etc/os-release)"
[[ -r "${OS_RELEASE_PATH}" ]] || fail "cannot read ${OS_RELEASE_PATH}"
ACTUAL_OS_ID="$(. "${OS_RELEASE_PATH}"; printf '%s' "${ID:-}")"
ACTUAL_OS_VERSION="$(. "${OS_RELEASE_PATH}"; printf '%s' "${VERSION_ID:-}")"
ACTUAL_KERNEL="$(uname -r)"
DETECTED_PROFILE="$(foundation_detect_profile "${ACTUAL_OS_ID}" "${ACTUAL_OS_VERSION}" "${ACTUAL_KERNEL}")" || fail "unsupported host"
PROFILE="${ROSIE_RT_FOUNDATION_PROFILE:-${DETECTED_PROFILE}}"
foundation_profile "${PROFILE}" "${ACTUAL_KERNEL}" || fail "unsupported profile"
[[ "${FOUNDATION_PROFILE}" == "${DETECTED_PROFILE}" ]] || fail "profile does not match detected host"
if [[ "${ACTION}" == kernel ]]; then
  install_pinned_kernel
  exit 0
fi
KERNEL_PIN_REFUSAL=""
if [[ -n "${PINNED_KERNEL}" && "${ACTUAL_KERNEL}" != "${PINNED_KERNEL}" ]]; then
  KERNEL_PIN_REFUSAL="running kernel ${ACTUAL_KERNEL} is not the pinned ${PINNED_KERNEL}: run 'ethercat-foundation.sh kernel', reboot the servo host (owner approval), then install"
  printf 'ethercat-foundation warning=%s\n' "${KERNEL_PIN_REFUSAL}" >&2
fi
for required_name in NO_REAL_MOVEMENT ROSIE_RT_FOUNDATION_ETHERCAT_INTERFACE \
  ROSIE_RT_FOUNDATION_ETHERCAT_MAC ROSIE_RT_FOUNDATION_ETHERCAT_PERMANENT_MAC \
  ROSIE_RT_FOUNDATION_UPLINK_INTERFACE ROSIE_RT_FOUNDATION_UPLINK_MAC \
  ROSIE_RT_FOUNDATION_UPLINK_PERMANENT_MAC ROSIE_RT_FOUNDATION_EXPECTED_SLAVES \
  ROSIE_RT_FOUNDATION_RECEIPT_PATH; do
  require_env "${required_name}"
done
EXPECTED_OS_ID="${ROSIE_RT_FOUNDATION_OS_ID:-${ACTUAL_OS_ID}}"
EXPECTED_OS_VERSION="${ROSIE_RT_FOUNDATION_OS_VERSION:-${ACTUAL_OS_VERSION}}"
EXPECTED_ARCH="${ROSIE_RT_FOUNDATION_ARCH:-$(dpkg --print-architecture)}"
EXPECTED_KERNEL="${ROSIE_RT_FOUNDATION_KERNEL:-${ACTUAL_KERNEL}}"
IGH_VERSION="${ROSIE_RT_FOUNDATION_IGH_VERSION:-${PINNED_IGH_VERSION}}"
IGH_SOURCE_URL="${ROSIE_RT_FOUNDATION_IGH_SOURCE_URL:-${PINNED_IGH_SOURCE_URL}}"
IGH_SOURCE_SHA256="${ROSIE_RT_FOUNDATION_IGH_SOURCE_SHA256:-${PINNED_IGH_SHA256}}"
IGH_SOURCE_SHA256="${IGH_SOURCE_SHA256#sha256:}"
ETHERCAT_INTERFACE="${ROSIE_RT_FOUNDATION_ETHERCAT_INTERFACE}"
ETHERCAT_MAC="$(printf '%s' "${ROSIE_RT_FOUNDATION_ETHERCAT_MAC}" | tr '[:upper:]' '[:lower:]')"
ETHERCAT_PERMANENT_MAC="$(printf '%s' "${ROSIE_RT_FOUNDATION_ETHERCAT_PERMANENT_MAC}" | tr '[:upper:]' '[:lower:]')"
UPLINK_INTERFACE="${ROSIE_RT_FOUNDATION_UPLINK_INTERFACE}"
UPLINK_MAC="$(printf '%s' "${ROSIE_RT_FOUNDATION_UPLINK_MAC}" | tr '[:upper:]' '[:lower:]')"
UPLINK_PERMANENT_MAC="$(printf '%s' "${ROSIE_RT_FOUNDATION_UPLINK_PERMANENT_MAC}" | tr '[:upper:]' '[:lower:]')"
# unverified driver qualification: generic is the default at the 1,000,000 ns cycle.
DEVICE_MODULES="${ROSIE_RT_FOUNDATION_DEVICE_MODULES:-generic}"
EXPECTED_SLAVES="${ROSIE_RT_FOUNDATION_EXPECTED_SLAVES}"
RECEIPT_PATH="${ROSIE_RT_FOUNDATION_RECEIPT_PATH}"

logical_receipt_path="${RECEIPT_PATH}"
RECEIPT_PATH="$(host_path "${logical_receipt_path}")"
OS_RELEASE_PATH="$(host_path /etc/os-release)"
ETHERCAT_CONF_PATH="$(host_path /etc/ethercat.conf)"
NM_UNMANAGED_PATH="$(host_path /etc/NetworkManager/conf.d/10-rosie-rt-core-ethercat-unmanaged.conf)"
SERVICE_PATH="$(host_path /etc/systemd/system/ethercat.service)"
SERVICE_DROPIN_PATH="$(host_path /etc/systemd/system/ethercat.service.d/10-rosie-rt-core-foundation.conf)"
ECRT_HEADER_PATH="$(host_path /usr/local/include/ecrt.h)"
LIBETHERCAT_STATIC_PATH="$(host_path /usr/local/lib/libethercat.a)"
LIBETHERCAT_SHARED_PATH="$(host_path /usr/local/lib/libethercat.so)"
LIBETHERCAT_PC_PATH="$(host_path /usr/local/lib/pkgconfig/libethercat.pc)"
ETHERCAT_CLI_PATH="$(host_path /usr/local/bin/ethercat)"
ETHERCATCTL_PATH="$(host_path /usr/local/sbin/ethercatctl)"
ETHERCAT_DEVICE_PATH="$(host_path /dev/EtherCAT0)"
KERNEL_HEADERS_PATH="$(host_path "/lib/modules/${EXPECTED_KERNEL}/build")"
PROC_MODULES_PATH="$(host_path /proc/modules)"

validate_simple_token() {
  local label="$1" value="$2"
  [[ "${value}" =~ ^[A-Za-z0-9_.:/+-]+$ ]] || fail "${label} contains unsupported characters"
}

validate_interface() {
  local label="$1" value="$2"
  [[ "${value}" =~ ^[A-Za-z0-9_.-]+$ ]] || fail "${label} is not a literal interface name"
}

validate_mac() {
  local label="$1" value="$2"
  [[ "${value}" =~ ^([0-9a-f]{2}:){5}[0-9a-f]{2}$ ]] || fail "${label} is not a canonical MAC address"
  [[ "${value}" != "00:00:00:00:00:00" ]] || fail "${label} cannot be all zeroes"
}

render_contract() {
  printf '%s\n' \
    "schema=${RECEIPT_SCHEMA}" \
    "profile=${PROFILE}" \
    "os_id=${EXPECTED_OS_ID}" \
    "os_version=${EXPECTED_OS_VERSION}" \
    "architecture=${EXPECTED_ARCH}" \
    "kernel_release=${EXPECTED_KERNEL}" \
    "igh_version=${IGH_VERSION}" \
    "igh_source_url=${IGH_SOURCE_URL}" \
    "igh_source_sha256=${IGH_SOURCE_SHA256}" \
    "ethercat_interface=${ETHERCAT_INTERFACE}" \
    "ethercat_mac=${ETHERCAT_MAC}" \
    "ethercat_permanent_mac=${ETHERCAT_PERMANENT_MAC}" \
    "uplink_interface=${UPLINK_INTERFACE}" \
    "uplink_mac=${UPLINK_MAC}" \
    "uplink_permanent_mac=${UPLINK_PERMANENT_MAC}" \
    "device_modules=${DEVICE_MODULES}" \
    "expected_slaves=${EXPECTED_SLAVES}" \
    "receipt_path=${logical_receipt_path}"
}

sha256_stream() {
  sha256sum | awk '{print $1}'
}

sha256_file() {
  sha256sum "$1" | awk '{print $1}'
}

contract_sha256() {
  render_contract | sha256_stream
}

render_ethercat_conf() {
  printf 'MASTER0_DEVICE="%s"\nDEVICE_MODULES="%s"\n' "${ETHERCAT_MAC}" "${DEVICE_MODULES}"
}

render_nm_unmanaged() {
  printf '[keyfile]\nunmanaged-devices=interface-name:%s\n' "${ETHERCAT_INTERFACE}"
}

render_service_dropin() {
  cat <<'EOF'
[Service]
ExecStart=
ExecStart=/usr/local/sbin/ethercatctl -c /etc/ethercat.conf start
ExecStop=
ExecStop=/usr/local/sbin/ethercatctl -c /etc/ethercat.conf stop
EOF
}

actual_permanent_mac() {
  local interface="$1" output mac
  output="$(ethtool -P "${interface}" 2>/dev/null)" || return 1
  mac="$(awk '{print tolower($NF)}' <<<"${output}")"
  validate_mac "observed permanent MAC for ${interface}" "${mac}" >/dev/null
  printf '%s\n' "${mac}"
}

# Native macb keeps its claimed device out of Linux's netdev list. Require the
# selected MAC attached with link UP in the master, preventing an absent NIC pass.
native_macb_link_matches() {
  [[ "${FOUNDATION_PROFILE}:${DEVICE_MODULES}" == rpi5-debian13:macb ]] || return 1
  grep -q '^ec_macb ' "${PROC_MODULES_PATH}" || return 1
  "${ETHERCAT_CLI_PATH}" master | awk -v mac="${ETHERCAT_MAC}" '
    $1 == "Main:" && $2 == mac && $3 == "(attached)" {main=1; next}
    main {if ($1 == "Link:" && $2 == "UP") good=1; main=0}
    END {exit !good}'
}

# The RevPi host services, by name; nothing for other profiles.
revpi_host_services() {
  [[ "${FOUNDATION_PROFILE}" == revpi-debian12 ]] || return 0
  printf '%s\n' "${REVPI_HOST_SERVICES[@]}"
}

# "<source> <destination> <mode>" for each RevPi host file; nothing for other profiles.
revpi_host_files() {
  local service
  while read -r service; do
    printf '%s %s 0644\n' "${FOUNDATION_FILES_DIR}/${service}.service" "/etc/systemd/system/${service}.service"
    printf '%s %s 0755\n' "${FOUNDATION_FILES_DIR}/${service}.sh" "/usr/local/sbin/${service}.sh"
  done < <(revpi_host_services)
  if [[ "${FOUNDATION_PROFILE}" == revpi-debian12 ]]; then
    printf '%s %s 0644\n' "${FOUNDATION_FILES_DIR}/${REVPI_DEVICE_DROPIN}" "/etc/systemd/system/${REVPI_DEVICE_DROPIN}"
  fi
}

validate_platform_and_contract() {
  [[ "${NO_REAL_MOVEMENT}" == "1" ]] || fail "NO_REAL_MOVEMENT=1 is required"
  if [[ "${FOUNDATION_PROFILE}" == revpi-debian12 ]]; then
    # The drop-in hands /dev/EtherCAT0 to picontrol; a second group would race it.
    [[ "${ETHERCAT_DEVICE_GROUP}" == picontrol ]] || fail "revpi-debian12 gives /dev/EtherCAT0 to picontrol (${REVPI_DEVICE_DROPIN}); set ROSIE_RT_ETHERCAT_GROUP=picontrol"
    local host_source host_destination host_mode
    while read -r host_source host_destination host_mode; do
      [[ -f "${host_source}" ]] || fail "missing RevPi host file ${host_source}; copy rt-core/host/foundation beside the installer"
    done < <(revpi_host_files)
  fi
  [[ "${EXPECTED_OS_ID}" == "debian" && "${EXPECTED_OS_VERSION}" == "${PINNED_OS_VERSION}" ]] || fail "OS does not match profile"
  [[ "${EXPECTED_ARCH}" == "arm64" ]] || fail "foundation is pinned to arm64"
  [[ "${IGH_VERSION}" == "${PINNED_IGH_VERSION}" ]] || fail "IgH version must be ${PINNED_IGH_VERSION}"
  [[ "${IGH_SOURCE_URL}" == "${PINNED_IGH_SOURCE_URL}" ]] || fail "IgH source URL does not match the pinned ${PINNED_IGH_VERSION} archive"
  [[ "${IGH_SOURCE_SHA256}" == "${PINNED_IGH_SHA256}" ]] || fail "IgH source SHA-256 does not match profile pin"
  case "${FOUNDATION_PROFILE}:${DEVICE_MODULES}" in
    revpi-debian12:generic|rpi5-debian13:generic|rpi5-debian13:macb) ;;
    *) fail "unsupported DEVICE_MODULES for profile" ;;
  esac
  if [[ "${FOUNDATION_PROFILE}" == rpi5-debian13 ]]; then
    [[ "${ETHERCAT_INTERFACE}" == eth0 ]] || fail "Pi EtherCAT interface must be eth0; wlan0/usb0 are protected"
    [[ "$(uname -v)" == *PREEMPT_RT* ]] || fail "Pi kernel must report PREEMPT_RT"
  fi
  [[ "${EXPECTED_SLAVES}" =~ ^[1-9][0-9]*$ ]] || fail "expected slave count must be positive"
  (( EXPECTED_SLAVES <= 256 )) || fail "expected slave count is unreasonably large"
  validate_simple_token "profile" "${PROFILE}"
  validate_simple_token "kernel" "${EXPECTED_KERNEL}"
  validate_interface "EtherCAT interface" "${ETHERCAT_INTERFACE}"
  validate_interface "uplink interface" "${UPLINK_INTERFACE}"
  [[ "${ETHERCAT_INTERFACE}" != "${UPLINK_INTERFACE}" ]] || fail "EtherCAT and uplink interfaces must be distinct"
  validate_mac "EtherCAT current MAC" "${ETHERCAT_MAC}"
  validate_mac "EtherCAT permanent MAC" "${ETHERCAT_PERMANENT_MAC}"
  validate_mac "uplink current MAC" "${UPLINK_MAC}"
  validate_mac "uplink permanent MAC" "${UPLINK_PERMANENT_MAC}"
  [[ "${ETHERCAT_MAC}" != "${UPLINK_MAC}" ]] || fail "EtherCAT and uplink current MACs must be distinct"
  [[ "${ETHERCAT_PERMANENT_MAC}" != "${UPLINK_PERMANENT_MAC}" ]] || fail "EtherCAT and uplink permanent MACs must be distinct"
  [[ "${logical_receipt_path}" =~ ^/var/lib/rosie-rt-core/host-provision/[A-Za-z0-9_.-]+$ ]] || fail "receipt must be a direct file under /var/lib/rosie-rt-core/host-provision"

  [[ -r "${OS_RELEASE_PATH}" ]] || fail "cannot read ${OS_RELEASE_PATH}"
  local actual_os_id actual_os_version actual_arch actual_kernel
  actual_os_id="$(. "${OS_RELEASE_PATH}"; printf '%s' "${ID:-}")"
  actual_os_version="$(. "${OS_RELEASE_PATH}"; printf '%s' "${VERSION_ID:-}")"
  actual_arch="$(dpkg --print-architecture)"
  actual_kernel="$(uname -r)"
  [[ "${actual_os_id}" == "${EXPECTED_OS_ID}" ]] || fail "OS mismatch expected=${EXPECTED_OS_ID} observed=${actual_os_id}"
  [[ "${actual_os_version}" == "${EXPECTED_OS_VERSION}" ]] || fail "OS version mismatch expected=${EXPECTED_OS_VERSION} observed=${actual_os_version}"
  [[ "${actual_arch}" == "${EXPECTED_ARCH}" ]] || fail "architecture mismatch expected=${EXPECTED_ARCH} observed=${actual_arch}"
  [[ "${actual_kernel}" == "${EXPECTED_KERNEL}" ]] || fail "kernel mismatch expected=${EXPECTED_KERNEL} observed=${actual_kernel}; explicit reprovisioning is required"

  local interface expected_current expected_permanent observed_current observed_permanent carrier_path
  while IFS='|' read -r interface expected_current expected_permanent; do
    if [[ "${interface}" == "${ETHERCAT_INTERFACE}" && ! -d "$(host_path "/sys/class/net/${interface}")" ]]; then
      native_macb_link_matches || fail "missing fieldbus netdev without matching macb master link"
      continue
    fi
    [[ -d "$(host_path "/sys/class/net/${interface}")" ]] || fail "manifest interface ${interface} does not exist"
    observed_current="$(tr '[:upper:]' '[:lower:]' < "$(host_path "/sys/class/net/${interface}/address")")"
    observed_permanent="$(actual_permanent_mac "${interface}")" || fail "cannot read permanent MAC for ${interface}"
    [[ "${observed_current}" == "${expected_current}" ]] || fail "current MAC mismatch interface=${interface} expected=${expected_current} observed=${observed_current}"
    [[ "${observed_permanent}" == "${expected_permanent}" ]] || fail "permanent MAC mismatch interface=${interface} expected=${expected_permanent} observed=${observed_permanent}"
    carrier_path="$(host_path "/sys/class/net/${interface}/carrier")"
    # A reboot into a kernel with no IgH module leaves the unmanaged fieldbus NIC
    # down (2026-11-02, mini bench 6.18.34 -> 6.18.50), and a down link reads no
    # carrier: install raises it below anyway, so raise it here rather than
    # refuse the reinstall that repairs it. Other actions stay read-only.
    if [[ "${ACTION}" == install && "${TESTING}" != "1" && "${interface}" == "${ETHERCAT_INTERFACE}" ]] &&
      [[ "$(cat "${carrier_path}" 2>/dev/null)" != "1" ]]; then
      ip link set dev "${interface}" up
      # unverified bound: 10 s for the drive link to negotiate.
      for _ in $(seq 1 50); do
        [[ "$(cat "${carrier_path}" 2>/dev/null)" == "1" ]] && break
        sleep 0.2
      done
    fi
    [[ -r "${carrier_path}" && "$(cat "${carrier_path}" 2>/dev/null)" == "1" ]] || fail "interface ${interface} has no carrier"
  done <<EOF
${ETHERCAT_INTERFACE}|${ETHERCAT_MAC}|${ETHERCAT_PERMANENT_MAC}
${UPLINK_INTERFACE}|${UPLINK_MAC}|${UPLINK_PERMANENT_MAC}
EOF
}

verify_error() {
  local quiet="$1"
  shift
  if [[ "${quiet}" != "1" ]]; then
    printf 'ethercat-foundation verify_error=%s\n' "$*" >&2
  fi
  return 1
}

module_path() {
  modinfo -n "$1" 2>/dev/null
}

module_vermagic_kernel() {
  modinfo -F vermagic "$1" 2>/dev/null | awk '{print $1}'
}

verify_runtime() {
  local quiet="${1:-0}" path module module_file module_kernel pkg_version slave_output slave_count
  for path in \
    "${ECRT_HEADER_PATH}" \
    "${LIBETHERCAT_STATIC_PATH}" \
    "${LIBETHERCAT_SHARED_PATH}" \
    "${LIBETHERCAT_PC_PATH}" \
    "${ETHERCAT_CLI_PATH}" \
    "${ETHERCATCTL_PATH}" \
    "${SERVICE_PATH}" \
    "${ETHERCAT_CONF_PATH}" \
    "${NM_UNMANAGED_PATH}" \
    "${SERVICE_DROPIN_PATH}"; do
    [[ -e "${path}" ]] || { verify_error "${quiet}" "missing ${path}"; return 1; }
  done
  [[ -x "${ETHERCAT_CLI_PATH}" && -x "${ETHERCATCTL_PATH}" ]] || { verify_error "${quiet}" "EtherCAT CLI/controller are not executable"; return 1; }
  cmp -s "${ETHERCAT_CONF_PATH}" <(render_ethercat_conf) || { verify_error "${quiet}" "/etc/ethercat.conf differs from the manifest contract"; return 1; }
  cmp -s "${NM_UNMANAGED_PATH}" <(render_nm_unmanaged) || { verify_error "${quiet}" "NetworkManager EtherCAT-only unmanaged rule differs"; return 1; }
  cmp -s "${SERVICE_DROPIN_PATH}" <(render_service_dropin) || { verify_error "${quiet}" "EtherCAT service override differs"; return 1; }
  if grep -Fq "${UPLINK_INTERFACE}" "${NM_UNMANAGED_PATH}"; then
    verify_error "${quiet}" "uplink interface appears in the unmanaged rule"
    return 1
  fi
  pkg_version="$(PKG_CONFIG_PATH="$(host_path /usr/local/lib/pkgconfig)${PKG_CONFIG_PATH:+:${PKG_CONFIG_PATH}}" pkg-config --modversion libethercat 2>/dev/null)" || { verify_error "${quiet}" "pkg-config cannot resolve libethercat"; return 1; }
  [[ "${pkg_version}" == "${IGH_VERSION}" ]] || { verify_error "${quiet}" "libethercat version mismatch expected=${IGH_VERSION} observed=${pkg_version}"; return 1; }
  for module in ec_master "ec_${DEVICE_MODULES}"; do
    module_file="$(module_path "${module}")" || { verify_error "${quiet}" "missing kernel module ${module}"; return 1; }
    [[ -f "${module_file}" ]] || { verify_error "${quiet}" "module ${module} path is missing: ${module_file}"; return 1; }
    case "${module_file}" in
      "$(host_path "/lib/modules/${EXPECTED_KERNEL}")"/*) ;;
      *) verify_error "${quiet}" "module ${module} is not installed for ${EXPECTED_KERNEL}"; return 1 ;;
    esac
    module_kernel="$(module_vermagic_kernel "${module}")" || { verify_error "${quiet}" "cannot read ${module} vermagic"; return 1; }
    [[ "${module_kernel}" == "${EXPECTED_KERNEL}" ]] || { verify_error "${quiet}" "module ${module} vermagic mismatch expected=${EXPECTED_KERNEL} observed=${module_kernel}"; return 1; }
    grep -q "^${module} " "${PROC_MODULES_PATH}" || { verify_error "${quiet}" "module ${module} is not loaded"; return 1; }
  done
  systemctl is-enabled --quiet ethercat.service || { verify_error "${quiet}" "ethercat.service is not enabled"; return 1; }
  systemctl is-active --quiet ethercat.service || { verify_error "${quiet}" "ethercat.service is not active"; return 1; }
  local host_source host_destination host_mode host_service
  while read -r host_source host_destination host_mode; do
    cmp -s "$(host_path "${host_destination}")" "${host_source}" || { verify_error "${quiet}" "${host_destination} differs from ${host_source}"; return 1; }
    [[ "${host_mode}" != 0755 || -x "$(host_path "${host_destination}")" ]] || { verify_error "${quiet}" "${host_destination} is not executable"; return 1; }
  done < <(revpi_host_files)
  while read -r host_service; do
    systemctl is-enabled --quiet "${host_service}.service" || { verify_error "${quiet}" "${host_service}.service is not enabled"; return 1; }
  done < <(revpi_host_services)
  if [[ -d "$(host_path "/sys/class/net/${ETHERCAT_INTERFACE}")" ]]; then
    nmcli -t -f GENERAL.STATE device show "${ETHERCAT_INTERFACE}" 2>/dev/null | grep -Eq '^GENERAL.STATE:10' || { verify_error "${quiet}" "${ETHERCAT_INTERFACE} is not unmanaged by NetworkManager"; return 1; }
  else
    native_macb_link_matches || { verify_error "${quiet}" "missing macb master link"; return 1; }
  fi
  if [[ "${TESTING}" == "1" ]]; then
    [[ -e "${ETHERCAT_DEVICE_PATH}" ]] || { verify_error "${quiet}" "missing ${ETHERCAT_DEVICE_PATH}"; return 1; }
  else
    [[ -c "${ETHERCAT_DEVICE_PATH}" ]] || { verify_error "${quiet}" "missing EtherCAT character device ${ETHERCAT_DEVICE_PATH}"; return 1; }
  fi
  "${ETHERCAT_CLI_PATH}" master >/dev/null 2>&1 || { verify_error "${quiet}" "EtherCAT master query failed"; return 1; }
  slave_output="$("${ETHERCAT_CLI_PATH}" slaves 2>/dev/null)" || { verify_error "${quiet}" "EtherCAT slave query failed"; return 1; }
  slave_count="$(awk '/^[[:space:]]*[0-9]+[[:space:]]/ {count++} END {print count+0}' <<<"${slave_output}")"
  [[ "${slave_count}" == "${EXPECTED_SLAVES}" ]] || { verify_error "${quiet}" "visible slave mismatch expected=${EXPECTED_SLAVES} observed=${slave_count}"; return 1; }
  return 0
}

render_receipt() {
  local master_module generic_module
  master_module="$(module_path ec_master)"
  generic_module="$(module_path "ec_${DEVICE_MODULES}")"
  render_contract
  printf '%s\n' \
    "contract_sha256=$(contract_sha256)" \
    "ethercat_conf_sha256=$(sha256_file "${ETHERCAT_CONF_PATH}")" \
    "networkmanager_conf_sha256=$(sha256_file "${NM_UNMANAGED_PATH}")" \
    "service_unit_sha256=$(sha256_file "${SERVICE_PATH}")" \
    "service_dropin_sha256=$(sha256_file "${SERVICE_DROPIN_PATH}")" \
    "ecrt_header_sha256=$(sha256_file "${ECRT_HEADER_PATH}")" \
    "libethercat_static_sha256=$(sha256_file "${LIBETHERCAT_STATIC_PATH}")" \
    "libethercat_shared_sha256=$(sha256_file "${LIBETHERCAT_SHARED_PATH}")" \
    "libethercat_pc_sha256=$(sha256_file "${LIBETHERCAT_PC_PATH}")" \
    "ethercat_cli_sha256=$(sha256_file "${ETHERCAT_CLI_PATH}")" \
    "ethercatctl_sha256=$(sha256_file "${ETHERCATCTL_PATH}")" \
    "ec_master_module_path=${master_module}" \
    "ec_master_module_sha256=$(sha256_file "${master_module}")" \
    "ec_master_vermagic=$(module_vermagic_kernel ec_master)" \
    "ec_${DEVICE_MODULES}_module_path=${generic_module}" \
    "ec_${DEVICE_MODULES}_module_sha256=$(sha256_file "${generic_module}")" \
    "ec_${DEVICE_MODULES}_vermagic=$(module_vermagic_kernel "ec_${DEVICE_MODULES}")"
  local host_source host_destination host_mode
  while read -r host_source host_destination host_mode; do
    printf 'host_file_sha256:%s=%s\n' "${host_destination}" "$(sha256_file "$(host_path "${host_destination}")")"
  done < <(revpi_host_files)
}

receipt_matches() {
  [[ -f "${RECEIPT_PATH}" ]] || return 1
  verify_runtime 1 || return 1
  cmp -s "${RECEIPT_PATH}" <(render_receipt)
}

install_file_from_renderer() {
  local destination="$1" mode="$2" renderer="$3" temp
  mkdir -p "$(dirname "${destination}")"
  temp="$(mktemp "${destination}.tmp.XXXXXX")"
  "${renderer}" > "${temp}"
  chmod "${mode}" "${temp}"
  chown root:root "${temp}"
  mv -f "${temp}" "${destination}"
}

prepare_running_kernel_headers() {
  [[ -d "${KERNEL_HEADERS_PATH}" && -f "${KERNEL_HEADERS_PATH}/Makefile" ]] || fail "exact running-kernel headers are missing at ${KERNEL_HEADERS_PATH}"
  local boot_config
  boot_config="$(host_path "/boot/config-${EXPECTED_KERNEL}")"
  if [[ -f "${boot_config}" && ! -f "${KERNEL_HEADERS_PATH}/.config" ]]; then
    install -m 0644 "${boot_config}" "${KERNEL_HEADERS_PATH}/.config"
  fi
  if [[ -f "${KERNEL_HEADERS_PATH}/include/config/auto.conf" && ! -f "${KERNEL_HEADERS_PATH}/include/config/auto.conf.cmd" ]]; then
    printf '%s\n' '# generated for external module build' > "${KERNEL_HEADERS_PATH}/include/config/auto.conf.cmd"
  fi
  if [[ -f "${KERNEL_HEADERS_PATH}/include/generated/autoconf.h" && ! -f "${KERNEL_HEADERS_PATH}/include/generated/autoconf.h.cmd" ]]; then
    printf '%s\n' '# generated for external module build' > "${KERNEL_HEADERS_PATH}/include/generated/autoconf.h.cmd"
  fi
  # RevPi's Debian header package carries the package release in the generated
  # headers used by external-module builds, while `make kernelrelease` rebuilds
  # a shorter value from CONFIG_LOCALVERSION (for example 6.12.56-v8). Trust
  # only the generated inputs that actually become UTS_RELEASE, then keep the
  # post-build modinfo vermagic check below as the final authority.
  local configured_kernel uts_release
  configured_kernel="$(tr -d '[:space:]' < "${KERNEL_HEADERS_PATH}/include/config/kernel.release" 2>/dev/null || true)"
  uts_release="$(sed -n 's/^#define[[:space:]]\+UTS_RELEASE[[:space:]]\+"\([^"]\+\)"$/\1/p' "${KERNEL_HEADERS_PATH}/include/generated/utsrelease.h" 2>/dev/null)"
  [[ "${configured_kernel}" == "${EXPECTED_KERNEL}" ]] || fail "kernel header configured release mismatch expected=${EXPECTED_KERNEL} observed=${configured_kernel:-missing}"
  [[ "${uts_release}" == "${EXPECTED_KERNEL}" ]] || fail "kernel header UTS release mismatch expected=${EXPECTED_KERNEL} observed=${uts_release:-missing}"
}

install_foundation() {
  [[ "${TESTING}" != "1" ]] || fail "test mode refuses a mutating foundation install"
  # A receipt that failed live/hash verification must not survive a partial
  # reprovision and masquerade as current to another deployment process.
  rm -f "${RECEIPT_PATH}"
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y --no-install-recommends \
    ca-certificates curl build-essential autoconf automake libtool pkg-config \
    bison flex bc libssl-dev libelf-dev xz-utils kmod ethtool iproute2 \
    "${HEADER_PACKAGE}"
  prepare_running_kernel_headers

  local build_root tarball partial source_dir jobs entry
  build_root="$(host_path "/var/cache/rosie-rt-core/ethercat-foundation/${IGH_VERSION}-${IGH_SOURCE_SHA256}")"
  tarball="${build_root}/ethercat-${IGH_VERSION}.tar.gz"
  partial="${tarball}.partial"
  source_dir="${build_root}/ethercat-${IGH_VERSION}"
  mkdir -p "${build_root}"
  if [[ ! -f "${tarball}" || "$(sha256_file "${tarball}")" != "${IGH_SOURCE_SHA256}" ]]; then
    rm -f "${tarball}" "${partial}"
    curl --fail --location --proto '=https' --tlsv1.2 --retry 5 --retry-delay 3 --connect-timeout 20 \
      --output "${partial}" "${PINNED_IGH_SOURCE_URL}"
    [[ "$(sha256_file "${partial}")" == "${IGH_SOURCE_SHA256}" ]] || { rm -f "${partial}"; fail "IgH source SHA-256 mismatch"; }
    mv -f "${partial}" "${tarball}"
  fi
  [[ "$(sha256_file "${tarball}")" == "${IGH_SOURCE_SHA256}" ]] || fail "cached IgH source SHA-256 mismatch"
  while IFS= read -r entry; do
    case "${entry}" in
      "ethercat-${IGH_VERSION}"|"ethercat-${IGH_VERSION}/"*) ;;
      *) fail "IgH archive contains an unexpected path: ${entry}" ;;
    esac
  done < <(tar -tzf "${tarball}")
  rm -rf "${source_dir}"
  tar -xzf "${tarball}" -C "${build_root}"
  [[ -x "${source_dir}/bootstrap" || -f "${source_dir}/bootstrap" ]] || fail "IgH archive is missing bootstrap"
  jobs="$(nproc)"
  [[ "${jobs}" =~ ^[1-9][0-9]*$ ]] || jobs=1
  (
    cd "${source_dir}"
    ./bootstrap
    ./configure \
      --prefix=/usr/local \
      --sysconfdir=/etc \
      --enable-kernel \
      "${CONFIGURE_DRIVERS[@]}" \
      --with-linux-dir="/lib/modules/${EXPECTED_KERNEL}/build" \
      --with-systemdsystemunitdir=/etc/systemd/system \
      --disable-initd
    make -j"${jobs}"
    make modules -j"${jobs}"
    if systemctl is-active --quiet ethercat.service; then
      systemctl stop ethercat.service
    fi
    make install
    # The installer runs with umask 077 so cached source and receipts stay
    # private. Autotools may create these public SDK directories under that
    # umask on a fresh host, though; keep the headers/pkg-config metadata
    # discoverable by the unprivileged RTCore build/install account.
    chmod 0755 \
      /usr/local/include \
      /usr/local/lib \
      /usr/local/lib/pkgconfig \
      /usr/local/lib/cmake \
      /usr/local/lib/cmake/ethercat
    make modules_install
  )
  ldconfig
  depmod -a "${EXPECTED_KERNEL}"

  install_file_from_renderer "${ETHERCAT_CONF_PATH}" 0644 render_ethercat_conf
  install_file_from_renderer "${NM_UNMANAGED_PATH}" 0644 render_nm_unmanaged
  install_file_from_renderer "${SERVICE_DROPIN_PATH}" 0644 render_service_dropin
  configure_ethercat_device_access
  local host_source host_destination host_mode host_service
  while read -r host_source host_destination host_mode; do
    install -D -m "${host_mode}" -o root -g root "${host_source}" "$(host_path "${host_destination}")"
  done < <(revpi_host_files)

  # Apply only the fieldbus-side NetworkManager state. The uplink is observed
  # above but is never renamed, reconfigured, disconnected, or reloaded here.
  nmcli device set "${ETHERCAT_INTERFACE}" managed no
  ip link set dev "${ETHERCAT_INTERFACE}" up
  systemctl daemon-reload
  systemctl enable ethercat.service
  # ethercat.service Wants= them through the drop-in, so its restart starts them.
  while read -r host_service; do
    systemctl enable "${host_service}.service"
  done < <(revpi_host_services)
  systemctl restart ethercat.service

  local attempt
  for attempt in $(seq 1 30); do
    if verify_runtime 1; then
      break
    fi
    sleep 1
  done
  if ! verify_runtime 0; then
    systemctl stop ethercat.service || true
    fail "strict post-install verification failed; EtherCAT master was stopped"
  fi

  mkdir -p "$(dirname "${RECEIPT_PATH}")"
  local receipt_tmp
  receipt_tmp="$(mktemp "${RECEIPT_PATH}.tmp.XXXXXX")"
  render_receipt > "${receipt_tmp}"
  chmod 0600 "${receipt_tmp}"
  chown root:root "${receipt_tmp}"
  mv -f "${receipt_tmp}" "${RECEIPT_PATH}"
  receipt_matches || fail "written foundation receipt does not match live artifacts"
}

validate_platform_and_contract

case "${ACTION}" in
  inspect)
    if receipt_matches; then
      printf 'ethercat_foundation_status=already_current profile=%s kernel=%s expected_slaves=%s\n' "${PROFILE}" "${EXPECTED_KERNEL}" "${EXPECTED_SLAVES}"
    else
      verify_runtime 0 || true
      printf 'ethercat_foundation_status=needs_install profile=%s kernel=%s expected_slaves=%s\n' "${PROFILE}" "${EXPECTED_KERNEL}" "${EXPECTED_SLAVES}"
    fi
    ;;
  verify)
    verify_runtime 0 || fail "foundation artifacts or runtime do not match the manifest"
    rtctl hostcheck || fail "read-only host qualification prerequisites failed"
    [[ -f "${RECEIPT_PATH}" ]] || fail "foundation receipt is missing"
    cmp -s "${RECEIPT_PATH}" <(render_receipt) || fail "foundation receipt does not match live content hashes"
    printf 'ethercat_foundation_status=verified profile=%s kernel=%s expected_slaves=%s\n' "${PROFILE}" "${EXPECTED_KERNEL}" "${EXPECTED_SLAVES}"
    ;;
  install)
    if receipt_matches; then
      if [[ "${TESTING}" != "1" ]]; then
        configure_ethercat_device_access
      fi
      printf 'ethercat_foundation_status=already_current profile=%s kernel=%s expected_slaves=%s\n' "${PROFILE}" "${EXPECTED_KERNEL}" "${EXPECTED_SLAVES}"
      exit 0
    fi
    [[ -z "${KERNEL_PIN_REFUSAL}" ]] || fail "${KERNEL_PIN_REFUSAL}"
    install_foundation
    printf 'ethercat_foundation_status=installed profile=%s kernel=%s expected_slaves=%s\n' "${PROFILE}" "${EXPECTED_KERNEL}" "${EXPECTED_SLAVES}"
    ;;
esac
