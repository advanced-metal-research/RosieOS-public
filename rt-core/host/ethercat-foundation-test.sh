#!/usr/bin/env bash

set -euo pipefail
export LC_ALL=C

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALLER="${SCRIPT_DIR}/ethercat-foundation.sh"
TMP="$(mktemp -d "${TMPDIR:-/tmp}/rosie-rt-core-foundation-test.XXXXXX")"
ROOT="${TMP}/root"
MOCK_BIN="${TMP}/bin"
CALL_LOG="${TMP}/calls.log"
trap 'rm -rf "${TMP}"' EXIT

fail() {
  printf 'ethercat-foundation-test error=%s\n' "$*" >&2
  exit 1
}

hash_file() {
  sha256sum "$1" | awk '{print $1}'
}

bash -n "${INSTALLER}"
grep -Fq '[[ "${NO_REAL_MOVEMENT}" == "1" ]]' "${INSTALLER}" || fail "movement lockout gate is missing"
if grep -Fq 'nmcli device set "${UPLINK_INTERFACE}"' "${INSTALLER}" || grep -Fq 'ip link set dev "${UPLINK_INTERFACE}"' "${INSTALLER}"; then
  fail "installer contains an uplink mutation"
fi

mkdir -p \
  "${ROOT}/etc/NetworkManager/conf.d" \
  "${ROOT}/etc/systemd/system/ethercat.service.d" \
  "${ROOT}/usr/local/include" \
  "${ROOT}/usr/local/lib/pkgconfig" \
  "${ROOT}/usr/local/bin" \
  "${ROOT}/usr/local/sbin" \
  "${ROOT}/lib/modules/6.12.56-revpi1-rpi-v8/ethercat/master" \
  "${ROOT}/lib/modules/6.12.56-revpi1-rpi-v8/ethercat/devices" \
  "${ROOT}/sys/class/net/eth0" \
  "${ROOT}/sys/class/net/eth1" \
  "${ROOT}/var/lib/rosie-rt-core/host-provision" \
  "${ROOT}/dev" \
  "${ROOT}/proc" \
  "${MOCK_BIN}"

printf 'ID=debian\nVERSION_ID="12"\n' > "${ROOT}/etc/os-release"
printf '02:00:00:00:00:02\n' > "${ROOT}/sys/class/net/eth0/address"
printf '1\n' > "${ROOT}/sys/class/net/eth0/carrier"
printf '02:00:00:00:00:03\n' > "${ROOT}/sys/class/net/eth1/address"
printf '1\n' > "${ROOT}/sys/class/net/eth1/carrier"
printf 'ec_master 1 0 - Live 0x0\nec_generic 1 0 - Live 0x0\n' > "${ROOT}/proc/modules"
printf 'master-module\n' > "${ROOT}/lib/modules/6.12.56-revpi1-rpi-v8/ethercat/master/ec_master.ko"
printf 'generic-module\n' > "${ROOT}/lib/modules/6.12.56-revpi1-rpi-v8/ethercat/devices/ec_generic.ko"
printf 'ecrt-header\n' > "${ROOT}/usr/local/include/ecrt.h"
printf 'static-library\n' > "${ROOT}/usr/local/lib/libethercat.a"
printf 'shared-library\n' > "${ROOT}/usr/local/lib/libethercat.so"
printf 'pkg-config\n' > "${ROOT}/usr/local/lib/pkgconfig/libethercat.pc"
printf 'ethercatctl\n' > "${ROOT}/usr/local/sbin/ethercatctl"
chmod 0755 "${ROOT}/usr/local/sbin/ethercatctl"
printf '[Unit]\nDescription=IgH EtherCAT Master\n' > "${ROOT}/etc/systemd/system/ethercat.service"
printf 'MASTER0_DEVICE="02:00:00:00:00:02"\nDEVICE_MODULES="generic"\n' > "${ROOT}/etc/ethercat.conf"
printf '[keyfile]\nunmanaged-devices=interface-name:eth0\n' > "${ROOT}/etc/NetworkManager/conf.d/10-rosie-rt-core-ethercat-unmanaged.conf"
printf '[Service]\nExecStart=\nExecStart=/usr/local/sbin/ethercatctl -c /etc/ethercat.conf start\nExecStop=\nExecStop=/usr/local/sbin/ethercatctl -c /etc/ethercat.conf stop\n' > "${ROOT}/etc/systemd/system/ethercat.service.d/10-rosie-rt-core-foundation.conf"
printf 'device\n' > "${ROOT}/dev/EtherCAT0"
# The RevPi host files, as installed from foundation/ beside the installer.
REVPI_HOST_DESTINATIONS=()
for service in robot-v4-cpu-performance robot-v4-ethercat-nic-tune robot-v4-irq-affinity; do
  cp "${SCRIPT_DIR}/foundation/${service}.service" "${ROOT}/etc/systemd/system/${service}.service"
  cp "${SCRIPT_DIR}/foundation/${service}.sh" "${ROOT}/usr/local/sbin/${service}.sh"
  chmod 0755 "${ROOT}/usr/local/sbin/${service}.sh"
  REVPI_HOST_DESTINATIONS+=("/etc/systemd/system/${service}.service" "/usr/local/sbin/${service}.sh")
done
cp "${SCRIPT_DIR}/foundation/ethercat.service.d/20-robot-v4-device-permissions.conf" "${ROOT}/etc/systemd/system/ethercat.service.d/"
REVPI_HOST_DESTINATIONS+=(/etc/systemd/system/ethercat.service.d/20-robot-v4-device-permissions.conf)

cat > "${ROOT}/usr/local/bin/ethercat" <<'EOF'
#!/usr/bin/env bash
printf 'ethercat %s\n' "$*" >> "${MOCK_CALL_LOG}"
case "${1:-}" in
  master)
    printf 'Master0\n  Phase: Idle\n  Slaves: 9\n'
    printf '  Main: %s (attached)\n      Link: UP\n' "${MOCK_MASTER_MAC:-02:00:00:00:00:02}"
    ;;
  slaves)
    index=0
    while [[ ${index} -lt ${MOCK_SLAVES:-9} ]]; do
      printf '%d  0:%d  PREOP  +  simulated-axis-%d\n' "${index}" "${index}" "${index}"
      index=$((index + 1))
    done
    ;;
  *) exit 2 ;;
esac
EOF
chmod 0755 "${ROOT}/usr/local/bin/ethercat"

cat > "${MOCK_BIN}/dispatcher" <<'EOF'
#!/usr/bin/env bash
set -eu
command_name="$(basename "$0")"
printf '%s %s\n' "${command_name}" "$*" >> "${MOCK_CALL_LOG}"
case "${command_name}" in
  uname)
    if [[ "${1:-}" == "-v" ]]; then printf '%s\n' "${MOCK_BUILD:-PREEMPT_RT}"; exit 0; fi
    [[ "${1:-}" == "-r" ]] || exit 2
    printf '%s\n' "${MOCK_KERNEL}"
    ;;
  dpkg)
    [[ "${1:-}" == "--print-architecture" ]] || exit 2
    printf 'arm64\n'
    ;;
  ethtool)
    [[ "${1:-}" == "-P" ]] || exit 2
    case "${2:-}" in
      eth0) printf 'Permanent address: 02:00:00:00:00:04\n' ;;
      eth1) printf 'Permanent address: 02:00:00:00:00:03\n' ;;
      *) exit 2 ;;
    esac
    ;;
  pkg-config)
    [[ "${1:-}" == "--modversion" && "${2:-}" == "libethercat" ]] || exit 2
    printf '%s\n' "${MOCK_IGH_VERSION:-1.6.9}"
    ;;
  modinfo)
    case "${1:-}" in
      -n)
        case "${2:-}" in
          ec_master) printf '%s/lib/modules/%s/ethercat/master/ec_master.ko\n' "${ROSIE_RT_FOUNDATION_TEST_ROOT}" "${MOCK_KERNEL}" ;;
          ec_generic|ec_macb) printf '%s/lib/modules/%s/ethercat/devices/%s.ko\n' "${ROSIE_RT_FOUNDATION_TEST_ROOT}" "${MOCK_KERNEL}" "$2" ;;
          *) exit 2 ;;
        esac
        ;;
      -F)
        [[ "${2:-}" == "vermagic" ]] || exit 2
        printf '%s SMP preempt mod_unload aarch64\n' "${MOCK_VERMAGIC:-${MOCK_KERNEL}}"
        ;;
      *) exit 2 ;;
    esac
    ;;
  systemctl)
    case "${1:-}" in
      is-active|is-enabled) exit 0 ;;
      *) exit 97 ;;
    esac
    ;;
  rtctl) exit 0 ;;
  nmcli)
    printf 'GENERAL.STATE:10 (unmanaged)\n'
    ;;
  apt-get|curl|wget|tar|make|install|ldconfig|depmod|modprobe|ip)
    exit 97
    ;;
  *) exit 2 ;;
esac
EOF
chmod 0755 "${MOCK_BIN}/dispatcher"
for command_name in rtctl uname dpkg ethtool pkg-config modinfo systemctl nmcli apt-get curl wget tar make install ldconfig depmod modprobe ip; do
  ln -s dispatcher "${MOCK_BIN}/${command_name}"
done

export NO_REAL_MOVEMENT=1
export ROSIE_RT_ETHERCAT_GROUP=picontrol
export ROSIE_RT_FOUNDATION_PROFILE=rosie-rt-core-revpi-igh-v1
export ROSIE_RT_FOUNDATION_OS_ID=debian
export ROSIE_RT_FOUNDATION_OS_VERSION=12
export ROSIE_RT_FOUNDATION_ARCH=arm64
export ROSIE_RT_FOUNDATION_KERNEL=6.12.56-revpi1-rpi-v8
export ROSIE_RT_FOUNDATION_IGH_VERSION=1.6.9
export ROSIE_RT_FOUNDATION_IGH_SOURCE_URL=https://gitlab.com/etherlab.org/ethercat/-/archive/1.6.9/ethercat-1.6.9.tar.gz
export ROSIE_RT_FOUNDATION_IGH_SOURCE_SHA256=2bc4d6b69ed980b896ecae9ea6603b327562dfacd3b8c1ed4e9e599a4cc4dfc3
export ROSIE_RT_FOUNDATION_ETHERCAT_INTERFACE=eth0
export ROSIE_RT_FOUNDATION_ETHERCAT_MAC=02:00:00:00:00:02
export ROSIE_RT_FOUNDATION_ETHERCAT_PERMANENT_MAC=02:00:00:00:00:04
export ROSIE_RT_FOUNDATION_UPLINK_INTERFACE=eth1
export ROSIE_RT_FOUNDATION_UPLINK_MAC=02:00:00:00:00:03
export ROSIE_RT_FOUNDATION_UPLINK_PERMANENT_MAC=02:00:00:00:00:03
export ROSIE_RT_FOUNDATION_DEVICE_MODULES=generic
export ROSIE_RT_FOUNDATION_EXPECTED_SLAVES=9
export ROSIE_RT_FOUNDATION_RECEIPT_PATH=/var/lib/rosie-rt-core/host-provision/ethercat.json
export ROSIE_RT_FOUNDATION_TESTING=1
export ROSIE_RT_FOUNDATION_TEST_ROOT="${ROOT}"
export MOCK_CALL_LOG="${CALL_LOG}"
export MOCK_KERNEL=6.12.56-revpi1-rpi-v8

render_contract() {
  printf '%s\n' \
    'schema=rosie-rt-core.ethercat-foundation.receipt.v1' \
    'profile=rosie-rt-core-revpi-igh-v1' \
    'os_id=debian' \
    'os_version=12' \
    'architecture=arm64' \
    'kernel_release=6.12.56-revpi1-rpi-v8' \
    'igh_version=1.6.9' \
    'igh_source_url=https://gitlab.com/etherlab.org/ethercat/-/archive/1.6.9/ethercat-1.6.9.tar.gz' \
    'igh_source_sha256=2bc4d6b69ed980b896ecae9ea6603b327562dfacd3b8c1ed4e9e599a4cc4dfc3' \
    'ethercat_interface=eth0' \
    'ethercat_mac=02:00:00:00:00:02' \
    'ethercat_permanent_mac=02:00:00:00:00:04' \
    'uplink_interface=eth1' \
    'uplink_mac=02:00:00:00:00:03' \
    'uplink_permanent_mac=02:00:00:00:00:03' \
    'device_modules=generic' \
    'expected_slaves=9' \
    'receipt_path=/var/lib/rosie-rt-core/host-provision/ethercat.json'
}

MASTER_MODULE="${ROOT}/lib/modules/6.12.56-revpi1-rpi-v8/ethercat/master/ec_master.ko"
GENERIC_MODULE="${ROOT}/lib/modules/6.12.56-revpi1-rpi-v8/ethercat/devices/ec_generic.ko"
RECEIPT="${ROOT}/var/lib/rosie-rt-core/host-provision/ethercat.json"
{
  render_contract
  printf '%s\n' \
    "contract_sha256=$(render_contract | sha256sum | awk '{print $1}')" \
    "ethercat_conf_sha256=$(hash_file "${ROOT}/etc/ethercat.conf")" \
    "networkmanager_conf_sha256=$(hash_file "${ROOT}/etc/NetworkManager/conf.d/10-rosie-rt-core-ethercat-unmanaged.conf")" \
    "service_unit_sha256=$(hash_file "${ROOT}/etc/systemd/system/ethercat.service")" \
    "service_dropin_sha256=$(hash_file "${ROOT}/etc/systemd/system/ethercat.service.d/10-rosie-rt-core-foundation.conf")" \
    "ecrt_header_sha256=$(hash_file "${ROOT}/usr/local/include/ecrt.h")" \
    "libethercat_static_sha256=$(hash_file "${ROOT}/usr/local/lib/libethercat.a")" \
    "libethercat_shared_sha256=$(hash_file "${ROOT}/usr/local/lib/libethercat.so")" \
    "libethercat_pc_sha256=$(hash_file "${ROOT}/usr/local/lib/pkgconfig/libethercat.pc")" \
    "ethercat_cli_sha256=$(hash_file "${ROOT}/usr/local/bin/ethercat")" \
    "ethercatctl_sha256=$(hash_file "${ROOT}/usr/local/sbin/ethercatctl")" \
    "ec_master_module_path=${MASTER_MODULE}" \
    "ec_master_module_sha256=$(hash_file "${MASTER_MODULE}")" \
    'ec_master_vermagic=6.12.56-revpi1-rpi-v8' \
    "ec_generic_module_path=${GENERIC_MODULE}" \
    "ec_generic_module_sha256=$(hash_file "${GENERIC_MODULE}")" \
    'ec_generic_vermagic=6.12.56-revpi1-rpi-v8'
  for destination in "${REVPI_HOST_DESTINATIONS[@]}"; do
    printf 'host_file_sha256:%s=%s\n' "${destination}" "$(hash_file "${ROOT}${destination}")"
  done
} > "${RECEIPT}"
chmod 0600 "${RECEIPT}"

before_receipt="$(hash_file "${RECEIPT}")"
before_tree="$(find "${ROOT}" -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}')"
: > "${CALL_LOG}"
output="$(PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" bash "${INSTALLER}" install)" || fail "matching receipt install action failed"
grep -Fq 'ethercat_foundation_status=already_current' <<<"${output}" || fail "install did not return already_current"
after_receipt="$(hash_file "${RECEIPT}")"
after_tree="$(find "${ROOT}" -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | awk '{print $1}')"
[[ "${before_receipt}" == "${after_receipt}" && "${before_tree}" == "${after_tree}" ]] || fail "already_current path changed foundation content"
if grep -Eq '^(apt-get|curl|wget|tar|make|install|ldconfig|depmod|modprobe|ip) |^systemctl (start|stop|restart|enable|disable|daemon-reload)' "${CALL_LOG}"; then
  fail "already_current path performed a package, build, network, module, or service mutation"
fi
grep -Fq 'pkg-config --modversion libethercat' "${CALL_LOG}" || fail "already_current path did not validate libethercat"
grep -Fq 'modinfo -F vermagic ec_master' "${CALL_LOG}" || fail "already_current path did not validate module vermagic"
grep -Fq 'ethercat slaves' "${CALL_LOG}" || fail "already_current path did not validate visible slaves"
grep -Fq 'systemctl is-enabled --quiet robot-v4-irq-affinity.service' "${CALL_LOG}" || fail "already_current path did not validate the RevPi host services"

# A drifted RevPi host file makes the receipt stale, and verify names it.
dropin="${ROOT}/etc/systemd/system/ethercat.service.d/20-robot-v4-device-permissions.conf"
cp "${dropin}" "${TMP}/dropin.orig"
printf '# local edit\n' >> "${dropin}"
output="$(PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" bash "${INSTALLER}" inspect 2>&1)" || fail "drifted host file inspection failed: ${output}"
[[ "${output}" == *'ethercat_foundation_status=needs_install'* ]] || fail "drifted host file still reads current: ${output}"
if PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" bash "${INSTALLER}" verify > "${TMP}/drift-host.out" 2>&1; then
  fail "drifted host file verified"
fi
grep -Fq '20-robot-v4-device-permissions.conf differs from' "${TMP}/drift-host.out" || fail "drifted host file not named"
cp "${TMP}/dropin.orig" "${dropin}"
if PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" ROSIE_RT_ETHERCAT_GROUP=ethercat bash "${INSTALLER}" inspect > "${TMP}/group.out" 2>&1; then
  fail "RevPi profile accepted a device group other than picontrol"
fi
grep -Fq 'set ROSIE_RT_ETHERCAT_GROUP=picontrol' "${TMP}/group.out" || fail "group refusal not explained"

: > "${CALL_LOG}"
export MOCK_KERNEL=6.12.57-revpi1-rpi-v8
if PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" bash "${INSTALLER}" install > "${TMP}/drift.out" 2>&1; then
  fail "kernel drift unexpectedly passed"
fi
grep -Fq 'kernel mismatch expected=6.12.56-revpi1-rpi-v8 observed=6.12.57-revpi1-rpi-v8' "${TMP}/drift.out" || fail "kernel drift did not fail with expected/observed evidence"
if grep -Eq '^(apt-get|curl|wget|tar|make|install|ldconfig|depmod|modprobe|ip) |^systemctl (start|stop|restart|enable|disable|daemon-reload)' "${CALL_LOG}"; then
  fail "kernel drift performed a mutation"
fi
[[ "$(hash_file "${RECEIPT}")" == "${before_receipt}" ]] || fail "kernel drift changed the receipt"

export MOCK_KERNEL=6.12.56-revpi1-rpi-v8
output="$(PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" env -u ROSIE_RT_FOUNDATION_PROFILE bash "${INSTALLER}" inspect 2>&1)" || fail 'RevPi autodetection failed'
[[ "${output}" == 'ethercat_foundation_status=needs_install profile=revpi-debian12 kernel=6.12.56-revpi1-rpi-v8 expected_slaves=9' ]] || fail "RevPi canonical profile differs: ${output}"

# An unpinned RevPi kernel warns on inspect and refuses a fresh install,
# before anything is downloaded or built; the kernel action refuses test mode.
mkdir -p "${ROOT}/lib/modules/6.12.91-revpi0-rpi-v8/build"
if PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" MOCK_KERNEL=6.12.91-revpi0-rpi-v8 ROSIE_RT_FOUNDATION_KERNEL=6.12.91-revpi0-rpi-v8 \
  bash "${INSTALLER}" inspect > "${TMP}/pin.out" 2>&1; then :; fi
grep -Fq 'running kernel 6.12.91-revpi0-rpi-v8 is not the pinned 6.12.56-revpi1-rpi-v8' "${TMP}/pin.out" || { cat "${TMP}/pin.out"; fail "unpinned kernel not reported"; }
: > "${CALL_LOG}"
if PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" bash "${INSTALLER}" kernel > "${TMP}/kernel.out" 2>&1; then
  fail "kernel action ran in test mode"
fi
grep -Fq 'test mode refuses a kernel install' "${TMP}/kernel.out" || { cat "${TMP}/kernel.out"; fail "kernel action refusal not explained"; }
if grep -Eq '^(apt-get|apt-mark|dpkg-reconfigure) ' "${CALL_LOG}"; then fail 'kernel refusal mutated host'; fi

# Pin selection is read-only, including when sourced by the userlib builder.
source "${INSTALLER}"
foundation_profile revpi-debian12 6.12.56-revpi1-rpi-v8
[[ "${PINNED_IGH_VERSION}" == 1.6.9 && "${HEADER_PACKAGE}" == linux-headers-6.12.56-revpi1-rpi-v8 && "${CONFIGURE_DRIVERS[*]}" == --enable-generic ]] || fail 'RevPi profile changed'
[[ "${PINNED_KERNEL}" == 6.12.56-revpi1-rpi-v8 && "${PINNED_KERNEL_VERSION}" == 6.12.56-revpi1-1+deb12+1 &&
  "${PINNED_PICONTROL_VERSION}" == 2.5.0-1+deb12+1 ]] || fail 'RevPi kernel pin changed'
foundation_profile rpi5-debian13 6.18.34+rpt-rpi-v8-rt
[[ "${PINNED_IGH_VERSION}" == 1.6.12 && "${HEADER_PACKAGE}" == linux-headers-rpi-v8-rt && "${CONFIGURE_DRIVERS[*]}" == '--enable-generic --enable-hrtimer' ]] || fail 'Pi profile differs'
[[ "${PINNED_IGH_SHA256}" == 6aa49381da3daf08131913c3fade8fd24b650836567459ef1d35991da1549310 ]] || fail 'Pi archive pin differs'

# Exercise the complete validator with mocked OS/kernel/modules, including the
# Pi build that has no /sys/kernel/realtime file. Inspection must never mutate.
export MOCK_KERNEL=6.18.34+rpt-rpi-v8-rt MOCK_IGH_VERSION=1.6.12 MOCK_SLAVES=1
export ROSIE_RT_FOUNDATION_PROFILE=rpi5-debian13
export ROSIE_RT_FOUNDATION_OS_VERSION=13 ROSIE_RT_FOUNDATION_KERNEL="${MOCK_KERNEL}"
export ROSIE_RT_FOUNDATION_IGH_VERSION=1.6.12
export ROSIE_RT_FOUNDATION_IGH_SOURCE_URL="${PINNED_IGH_SOURCE_URL}"
export ROSIE_RT_FOUNDATION_IGH_SOURCE_SHA256="${PINNED_IGH_SHA256}"
export ROSIE_RT_FOUNDATION_EXPECTED_SLAVES=1
printf 'ID=debian\nVERSION_ID="13"\n' > "${ROOT}/etc/os-release"
mkdir -p "${ROOT}/lib/modules/${MOCK_KERNEL}/ethercat/"{master,devices}
printf 'master-module\n' > "${ROOT}/lib/modules/${MOCK_KERNEL}/ethercat/master/ec_master.ko"
for driver in generic macb; do
  export ROSIE_RT_FOUNDATION_DEVICE_MODULES="${driver}"
  printf '%s-module\n' "${driver}" > "${ROOT}/lib/modules/${MOCK_KERNEL}/ethercat/devices/ec_${driver}.ko"
  printf 'ec_master 1 0 - Live 0x0\nec_%s 1 0 - Live 0x0\n' "${driver}" > "${ROOT}/proc/modules"
  printf 'MASTER0_DEVICE="02:00:00:00:00:02"\nDEVICE_MODULES="%s"\n' "${driver}" > "${ROOT}/etc/ethercat.conf"
  : > "${CALL_LOG}"
  output="$(PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" bash "${INSTALLER}" inspect 2>&1)" || fail "Pi ${driver} inspection failed: ${output}"
  [[ "${output}" == 'ethercat_foundation_status=needs_install profile=rpi5-debian13 kernel=6.18.34+rpt-rpi-v8-rt expected_slaves=1' ]] || fail "Pi runtime validation failed: ${output}"
  grep -Fq "modinfo -F vermagic ec_${driver}" "${CALL_LOG}" || fail 'selected module not checked'
  if [[ "${driver}" == generic ]]; then
    output="$(PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" env \
      -u ROSIE_RT_FOUNDATION_PROFILE -u ROSIE_RT_FOUNDATION_OS_VERSION \
      -u ROSIE_RT_FOUNDATION_KERNEL -u ROSIE_RT_FOUNDATION_IGH_VERSION \
      -u ROSIE_RT_FOUNDATION_IGH_SOURCE_URL -u ROSIE_RT_FOUNDATION_IGH_SOURCE_SHA256 \
      -u ROSIE_RT_FOUNDATION_DEVICE_MODULES bash "${INSTALLER}" inspect 2>&1)" || fail 'Pi autodetection/defaults failed'
    [[ "${output}" == 'ethercat_foundation_status=needs_install profile=rpi5-debian13 kernel=6.18.34+rpt-rpi-v8-rt expected_slaves=1' ]] || fail "Pi defaults differ: ${output}"
  fi
done

expect_refusal() {
  local reason="$1"; shift
  : > "${CALL_LOG}"
  if PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" "$@" bash "${INSTALLER}" verify > "${TMP}/refusal.out" 2>&1; then
    fail "accepted ${reason}"
  fi
  grep -Fq "${reason}" "${TMP}/refusal.out" || { cat "${TMP}/refusal.out"; fail "missing ${reason}"; }
  if grep -Eq '^(apt-get|curl|tar|make|install|ip) |^systemctl (start|stop|restart|enable)' "${CALL_LOG}"; then fail 'refusal mutated host'; fi
}
mv "${ROOT}/sys/class/net/eth0" "${ROOT}/sys/class/net/eth0-unclaimed"
output="$(PATH="${MOCK_BIN}:/usr/bin:/bin:/usr/sbin:/sbin" bash "${INSTALLER}" inspect 2>&1)" || fail "claimed macb inspection failed: ${output}"
[[ "${output}" == ethercat_foundation_status=needs_install* ]] || fail "claimed macb runtime failed: ${output}"
expect_refusal 'missing fieldbus netdev without matching macb master link' env MOCK_MASTER_MAC=02:00:00:00:00:05
mv "${ROOT}/sys/class/net/eth0-unclaimed" "${ROOT}/sys/class/net/eth0"
expect_refusal 'profile does not match' env ROSIE_RT_FOUNDATION_PROFILE=revpi-debian12
expect_refusal 'unsupported host' env MOCK_KERNEL=6.19.1+rpt-rpi-v8-rt
expect_refusal 'unsupported host' env MOCK_KERNEL=6.18.34+rpt-rpi-v8
expect_refusal 'PREEMPT_RT' env MOCK_BUILD=PREEMPT
expect_refusal 'SHA-256 does not match' env ROSIE_RT_FOUNDATION_IGH_SOURCE_SHA256="$(printf '%064d' 0)"
expect_refusal 'IgH version must be' env ROSIE_RT_FOUNDATION_IGH_VERSION=1.6.9
expect_refusal 'unsupported DEVICE_MODULES' env ROSIE_RT_FOUNDATION_DEVICE_MODULES=igb
for interface in wlan0 usb0; do
  expect_refusal 'Pi EtherCAT interface must be eth0' env ROSIE_RT_FOUNDATION_ETHERCAT_INTERFACE="${interface}"
done
expect_refusal 'vermagic mismatch' env MOCK_VERMAGIC=6.18.33+rpt-rpi-v8-rt
expect_refusal 'visible slave mismatch' env MOCK_SLAVES=2
printf 'ID=ubuntu\nVERSION_ID="13"\n' > "${ROOT}/etc/os-release"
expect_refusal 'unsupported host' env
printf 'ethercat-foundation-test status=passed contracts=syntax,safety,kernel-pin,revpi-receipt-no-churn,revpi-host-files,kernel-drift,pi-generic,pi-macb,host-refusals,pins,headers,vermagic,slave-count\n'
