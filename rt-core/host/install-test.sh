#!/usr/bin/env bash
# Exercise deployment against a confined host root; mocks prevent identity/service mutations.
set -euo pipefail
export LC_ALL=C
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export TMPDIR="${TMPDIR:-/tmp}"
mkdir -p "${component}/build"
scratch="$(mktemp -d "${component}/build/install-test.XXXXXX")"
runtime=""
trap 'rm -rf -- "${scratch}"; if [[ -n "${runtime}" ]]; then rm -rf -- "${runtime}"; fi' EXIT
runtime="$(mktemp -d "${TMPDIR:-/tmp}/rosie-rt-install-test.XXXXXX")"
fixture="${scratch}/component"
root="${runtime}"
mock="${scratch}/bin"
log="${scratch}/calls.log"
fail() { printf 'install-test error=%s\n' "$*" >&2; exit 1; }
expect_line() { grep -Fxq -- "$2" "$1" || fail "missing line $2 in $1"; }
expect_failure() {
  if "$@" > "${scratch}/failure.out" 2>&1; then
    fail "unexpected success: $*"
  fi
}
for script in install.sh generate-control-env.sh ethercat-foundation.sh install-test.sh; do
  bash -n "${component}/host/${script}"
done

mkdir -p "${fixture}/host" "${fixture}/tools" "${fixture}/build/deps/ethercat-1.6.9" "${root}" "${mock}" "${scratch}/accounts"
cp -a "${component}/config" "${fixture}/"
cp "${component}/host/"*.service "${component}/host/"*.timer "${component}/host/install.sh" "${component}/host/generate-control-env.sh" "${fixture}/host/"
# Normalize only fixture copies so Windows checkouts do not change the Linux script contract.
tr -d '\r' < "${component}/tools/package-runtime.sh" > "${fixture}/tools/package-runtime.sh"
tr -d '\r' < "${component}/host/run-core" > "${fixture}/host/run-core"
printf '#!/bin/sh\nexit 0\n' > "${fixture}/build/rosie-rt-core"
cp "${fixture}/build/rosie-rt-core" "${fixture}/build/rt-control"
[[ -x "${component}/build/rtctl" ]] || fail 'build/rtctl is missing; run make control first'
cp "${component}/build/rtctl" "${fixture}/build/rtctl"
cp "${component}/build/rt-natspublisher" "${fixture}/build/rt-natspublisher"
chmod +x "${fixture}/build/rosie-rt-core" "${fixture}/build/rt-control"
printf 'mock userspace library\n' > "${scratch}/libethercat.so.1"
printf 'mock license\n' > "${fixture}/build/deps/ethercat-1.6.9/COPYING"
cp "${fixture}/build/deps/ethercat-1.6.9/COPYING" "${fixture}/build/deps/ethercat-1.6.9/COPYING.LESSER"
# Unverified installation fixture: a 10:1 software ratio with the standard
# 131072 motor counts/rev and drive-side 0x6091 factor 1/1.
sed -e 's/"backend": "simulation"/"backend": "live"/' \
  -e 's/"wire_revs_per_axis_rev": 1/"wire_revs_per_axis_rev": 10/' \
  -e 's/"profile": "cia402_sim"/"profile": "a6ec"/' \
  "${component}/config/machines/simulation/simulation-program.json" > "${scratch}/machine.json"

cat > "${mock}/dispatch" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
command="${0##*/}"
printf '%s %s\n' "${command}" "$*" >> "${MOCK_LOG}"
case "${command}" in
  getent)
    [[ $# == 2 && ( "$1" == group || "$1" == passwd ) ]] || exit 2
    [[ -f "${MOCK_ACCOUNTS}/$1.$2" ]] || exit 2
    cat "${MOCK_ACCOUNTS}/$1.$2"
    ;;
  groupadd)
    [[ $# == 2 && "$1" == --system && ! -e "${MOCK_ACCOUNTS}/group.$2" ]] || exit 2
    printf '%s:x:123:\n' "$2" > "${MOCK_ACCOUNTS}/group.$2"
    ;;
  useradd)
    [[ $# == 11 && "$1" == --system && "$2" == --gid && "$4" == --groups && "$6" == --home-dir && "$7" == /nonexistent && "$8" == --no-create-home && "$9" == --shell && "${10}" == /usr/sbin/nologin ]] || exit 2
    user="${11}"
    [[ ! -e "${MOCK_ACCOUNTS}/passwd.${user}" && -e "${MOCK_ACCOUNTS}/group.$3" ]] || exit 2
    printf '%s:x:123:123::/nonexistent:/usr/sbin/nologin\n' "${user}" > "${MOCK_ACCOUNTS}/passwd.${user}"
    printf '%s\n' "$3" > "${MOCK_ACCOUNTS}/primary.${user}"
    printf '%s %s\n' "$3" "${5//,/ }" > "${MOCK_ACCOUNTS}/groups.${user}"
    ;;
  id)
    case "$1" in
      -gn) cat "${MOCK_ACCOUNTS}/primary.$2" ;;
      -Gn) cat "${MOCK_ACCOUNTS}/groups.$2" ;;
      *) exit 2 ;;
    esac
    ;;
  install)
    args=()
    while [[ $# -gt 0 ]]; do
      case "$1" in
        -o|-g) shift 2 ;;
        *) args+=("$1"); shift ;;
      esac
    done
    [[ "${args[-1]}" == "${ROSIE_RT_INSTALL_TEST_ROOT}/"* ]] || exit 2
    exec /usr/bin/install "${args[@]}"
    ;;
  systemctl)
    case "$*" in
      daemon-reload|'enable rosie-rt-core.service rosie-rt-control.service') ;;
      *) exit 2 ;;
    esac
    ;;
  udevadm)
    case "$*" in
      'control --reload-rules'|'trigger --action=change --sysname-match=EtherCAT0') ;;
      *) exit 2 ;;
    esac
    ;;
  ldd) printf 'libethercat.so.1 => %s (0x1)\n' "${MOCK_LIBRARY}" ;;
  rtctl)
    [[ "${MOCK_COMPILE_FAIL:-0}" == 0 ]] || exit 17
    exec "${MOCK_RTCTL}" "$@"
    ;;
  *) exit 2 ;;
esac
SH
chmod +x "${mock}/dispatch"
for command in getent groupadd useradd id install systemctl udevadm ldd rtctl; do
  ln -s dispatch "${mock}/${command}"
done
export PATH="${mock}:${PATH}" ROSIE_RT_INSTALL_TEST_ROOT="${root}"
export MOCK_LOG="${log}" MOCK_ACCOUNTS="${scratch}/accounts" MOCK_LIBRARY="${scratch}/libethercat.so.1"
export MOCK_RTCTL="${component}/build/rtctl"
unset ROSIE_RT_ETHERCAT_GROUP ECRT_LICENSE_DIR ROSIE_RT_NATS_URL ROSIE_RT_NATS_HOST ROSIE_RT_REMOTE_LISTEN \
  ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS ROSIE_RT_REMOTE_JOG_DRIFT_PPB ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS
installer="${fixture}/host/install.sh"
generator="${fixture}/host/generate-control-env.sh"
# Explicit revision 42 exercises the published binding; the dense axis map is
# not part of it (the adapter derives it from the description).
binding=("${scratch}/machine.json" bench-pair 42)
: > "${log}"
expect_failure bash "${installer}" --enable
expect_line "${scratch}/failure.out" 'install error=--enable requires machine.json, pair-id and pair-revision'
[[ ! -s "${log}" ]] || fail 'missing configuration mutated the host'
expect_failure env MOCK_COMPILE_FAIL=1 bash "${installer}" "${binding[@]}"
expect_line "${scratch}/failure.out" 'install error=configuration compilation failed; control.env was not replaced'
if grep -Eq '^(getent|groupadd|useradd|install|systemctl) ' "${log}"; then fail 'failed preflight mutated or inspected host identities'; fi
[[ ! -e "${root}/etc/rosie-rt-core/control.env" ]] || fail 'failed preflight published a binding'
: > "${log}"
# Pruning without a release store is a reported no-op; pinning needs a stored
# release; neither combines with installation. The release cases are in lifecycle-test.sh.
bash "${installer}" --prune > "${scratch}/prune.out"
expect_line "${scratch}/prune.out" 'prune status=done removed=0 kept=0 freed_kb=0 store=absent'
expect_failure bash "${installer}" --pin aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
expect_line "${scratch}/failure.out" 'install error=release store must be a directory'
expect_failure bash "${installer}" --pin not-a-revision
expect_line "${scratch}/failure.out" 'install error=--pin requires a 40-hex release revision'
expect_failure bash "${installer}" --prune --enable "${binding[@]}"
expect_line "${scratch}/failure.out" 'install error=--prune and --pin cannot be combined with installation options'
[[ ! -s "${log}" && ! -e "${root}/opt" ]] || fail 'prune without a store mutated the host'

bash "${installer}"
publisher_unit="${root}/etc/systemd/system/rosie-rt-natspublisher.service"
for line in DynamicUser=yes User=rosie-telemetry Group=rosie-rt-clients SupplementaryGroups= \
  AmbientCapabilities= CapabilityBoundingSet= NoNewPrivileges=yes \
  'EnvironmentFile=/etc/rosie-rt-core/natspublisher.env' 'Conflicts=robot-v4-rtcore-nats-bridge.service'; do
  expect_line "${publisher_unit}" "${line}"
done
[[ -x "${root}/opt/rosie/rt-core/build/rt-natspublisher" ]] || fail 'missing publisher binary'
[[ ! -e "${root}/etc/rosie-rt-core/natspublisher.env" ]] || fail 'invented NATS deployment'
ROSIE_RT_NATS_URL=nats://127.0.0.1:4222 ROSIE_RT_NATS_HOST=other-relay bash "${installer}"
publisher_env="${root}/etc/rosie-rt-core/natspublisher.env"
expect_line "${publisher_env}" 'ROSIE_RT_NATS_URL="nats://127.0.0.1:4222"'
expect_line "${publisher_env}" 'ROSIE_RT_NATS_HOST="other-relay"'
[[ "$(stat -c %a "${publisher_env}")" == 600 ]] || fail 'NATS deployment mode differs'
publisher_before="$(sha256sum "${publisher_env}")"
expect_failure env ROSIE_RT_NATS_URL=nats://127.0.0.1:4222 ROSIE_RT_NATS_HOST=realtime-core bash "${installer}"
expect_failure env ROSIE_RT_NATS_URL=nats://127.0.0.1:4222 bash "${installer}"
[[ "$(sha256sum "${publisher_env}")" == "${publisher_before}" ]] || fail 'invalid NATS deployment replaced configuration'
[[ ! -e "${root}/etc/rosie-rt-core/control.env" ]] || fail 'unconfigured installation invented a binding'
if grep -Eq '^systemctl (enable|start|restart)' "${log}"; then fail 'installation activated a service without --enable'; fi
bash "${installer}" "${binding[@]}"
[[ "$(grep -c '^groupadd ' "${log}")" == 4 ]] || fail 'groups were not created exactly once'
[[ "$(grep -c '^useradd ' "${log}")" == 2 ]] || fail 'users were not created exactly once'
for group in rosie-rt rosie-ctl rosie-rt-clients ethercat; do
  expect_line "${log}" "groupadd --system ${group}"
done
expect_line "${log}" 'useradd --system --gid rosie-rt --groups rosie-ctl,ethercat --home-dir /nonexistent --no-create-home --shell /usr/sbin/nologin rosie-rt'
expect_line "${log}" 'useradd --system --gid rosie-ctl --groups rosie-rt-clients --home-dir /nonexistent --no-create-home --shell /usr/sbin/nologin rosie-ctl'
if grep -Eq '^systemctl (enable|start|restart)' "${log}"; then fail 'configured installation enabled without --enable'; fi

environment="${root}/etc/rosie-rt-core/control.env"
expect_line "${environment}" 'ROSIE_RT_PAIR_ID=bench-pair'
expect_line "${environment}" 'ROSIE_RT_PAIR_REVISION=42'
[[ "$(stat -c %a "${environment}")" == 640 ]] || fail 'control.env mode is not 0640'
grep -Eq '^install -m 0640 -o root -g rosie-ctl .*control.env ' "${log}" || fail 'control.env ownership was not root:rosie-ctl'
# A fresh host gets the persistent Home-anchor directory (adapter-owned, native
# reader group, setgid 2750) once, and both units read it from a drop-in, never
# from control.env, whose EnvironmentFile= would override a cell's own drop-in.
anchors="${root}/var/lib/rosie-rt-anchors"
[[ -d "${anchors}" && ! -L "${anchors}" && "$(stat -c %a "${anchors}")" == 2750 ]] || fail 'anchor directory is not a 2750 directory'
[[ "$(grep -Fxc "install -d -m 2750 -o rosie-ctl -g rosie-rt ${anchors}" "${log}")" == 1 ]] || fail 'anchor directory was not created exactly once as rosie-ctl:rosie-rt'
for unit in rosie-rt-core rosie-rt-control; do
  dropin="${root}/etc/systemd/system/${unit}.service.d/20-rosie-rt-anchors.conf"
  expect_line "${dropin}" '[Service]'
  expect_line "${dropin}" 'Environment=ROSIE_RT_ANCHOR_DIR=/var/lib/rosie-rt-anchors'
  [[ "$(stat -c %a "${dropin}")" == 644 && "$(ls "${dropin%/*}")" == 20-rosie-rt-anchors.conf ]] || fail "anchor drop-in for ${unit} differs"
done
if grep -Fq ROSIE_RT_ANCHOR_DIR "${environment}"; then fail 'anchor directory written into control.env'; fi
# A re-run keeps an existing directory as it is: its contents, mode and owner
# may be the only Home evidence, so a non-2750 mode is reported, never repaired.
printf 'fixture anchor\n' > "${anchors}/axis-0.anchor"
chmod 0640 "${anchors}/axis-0.anchor"
chmod 2770 "${anchors}"
anchor_before="$(sha256sum "${anchors}/axis-0.anchor")"
dropins_before="$(cat "${root}/etc/systemd/system/"rosie-rt-{core,control}.service.d/*.conf | sha256sum)"
log_mark="$(wc -l < "${log}")"
bash "${installer}" "${binding[@]}" > "${scratch}/rerun.out"
expect_line "${scratch}/rerun.out" 'install component=anchors status=provisioned dir=/var/lib/rosie-rt-anchors directory=existing dropin=20-rosie-rt-anchors.conf'
[[ "$(stat -c %a "${anchors}")" == 2770 && "$(stat -c %a "${anchors}/axis-0.anchor")" == 640 && "$(sha256sum "${anchors}/axis-0.anchor")" == "${anchor_before}" ]] || fail 're-run changed the existing anchor directory'
[[ "$(cat "${root}/etc/systemd/system/"rosie-rt-{core,control}.service.d/*.conf | sha256sum)" == "${dropins_before}" ]] || fail 're-run changed the anchor drop-ins'
if tail -n "+$((log_mark + 1))" "${log}" | grep -Fq "${anchors}"; then fail 're-run touched the existing anchor directory'; fi
chmod 2750 "${anchors}"
rm -f -- "${anchors}/axis-0.anchor"
source "${environment}"
[[ "${ROSIE_RT_CONFIGURATION_SHA256}" =~ ^[0-9a-f]{64}$ ]] || fail 'invalid configuration digest'
compiled="${root}/etc/rosie-rt-core/compiled/${ROSIE_RT_CONFIGURATION_SHA256}"
[[ -s "${compiled}/axes.conf" && -s "${compiled}/argv.json" ]] || fail 'missing compiled configuration'
[[ "${ROSIE_RT_CORE_ARGS}" == *"--configuration-hash ${ROSIE_RT_CONFIGURATION_SHA256}"* && "${ROSIE_RT_CORE_ARGS}" == *"--axis-profiles ${compiled}/axes.conf"* ]] || fail 'native and adapter configuration identities differ'
[[ "$(stat -c %a "${compiled}")" == 2750 && "$(stat -c %a "${compiled}/axes.conf")" == 640 && "$(stat -c %a "${compiled}/argv.json")" == 640 ]] || fail 'compiled configuration permissions differ'
grep -Eq '^install -d -m 2750 -o root -g rosie-ctl .*/compiled$' "${log}" || fail 'adapter cannot traverse compiled root'
grep -Eq '^install -m 0640 -o root -g rosie-ctl .*resources.json ' "${log}" || fail 'compiled resources are not adapter-readable'
[[ -s "${compiled}/resources.json" && -s "${compiled}/configuration.identity" ]] || fail 'missing adapter identity/resources'
expected="$("${MOCK_RTCTL}" compile --config "${scratch}/machine.json" --profiles "${fixture}/config/drives" --out "${scratch}/expected")"
while IFS= read -r -d '' file; do
  relative="${file#"${expected}/"}"
  # control-env intentionally rewrites argv paths to the installation directory.
  [[ "${relative}" == argv.json ]] && continue
  cmp "${file}" "${compiled}/${relative}" || fail "incomplete installed snapshot: ${relative}"
  [[ "$(stat -c %a "${compiled}/${relative}")" == 640 ]] || fail "resource permissions: ${relative}"
done < <(find "${expected}" -type f -print0)
before="$(sha256sum "${environment}")"
bash "${generator}" "${binding[@]}"
[[ "$(sha256sum "${environment}")" == "${before}" ]] || fail 'repeated generation changed the binding'
[[ -z "$(find "${root}/etc/rosie-rt-core" -name '.staging.*' -o -name '.control.env.*')" ]] || fail 'generation left unpublished staging files'

core="${root}/etc/systemd/system/rosie-rt-core.service"
control="${root}/etc/systemd/system/rosie-rt-control.service"
for line in User=rosie-rt Group=rosie-rt-clients 'SupplementaryGroups=rosie-rt rosie-ctl ethercat' \
  'AmbientCapabilities=CAP_SYS_NICE CAP_IPC_LOCK' 'CapabilityBoundingSet=CAP_SYS_NICE CAP_IPC_LOCK' \
  RuntimeDirectory=rosie-rt-core RuntimeDirectoryMode=2770 RuntimeDirectoryPreserve=no; do
  expect_line "${core}" "${line}"
done
for line in User=rosie-ctl Group=rosie-rt-clients SupplementaryGroups=rosie-ctl AmbientCapabilities= CapabilityBoundingSet= \
  RuntimeDirectory=rosie-rt-core/public RuntimeDirectoryMode=2750 RuntimeDirectoryPreserve=no UMask=0007; do
  expect_line "${control}" "${line}"
done
for unit in "${core}" "${control}"; do
  expect_line "${unit}" 'EnvironmentFile=/etc/rosie-rt-core/control.env'
  expect_line "${unit}" 'NoNewPrivileges=yes'
  [[ "$(stat -c %a "${unit}")" == 644 ]] || fail 'unit is not mode 0644'
  if grep -Fq CAP_SYS_ADMIN "${unit}"; then fail 'administrative capability granted'; fi
done
expect_line "${core}" 'ExecStartPre=/usr/bin/install -d -m 2750 -g rosie-ctl /run/rosie-rt-core/native'
expect_line "${core}" 'ExecStartPre=/usr/bin/ln -s public/control.sock /run/rosie-rt-core/control.sock'
expect_line "${core}" 'ExecStartPre=/usr/bin/ln -s public/jog.sock /run/rosie-rt-core/jog.sock'
expect_line "${core}" 'ExecStart=/opt/rosie/rt-core/build/rosie-rt-core $ROSIE_RT_CORE_ARGS --socket-path /run/rosie-rt-core/native/ipc.sock --ipc-group rosie-ctl --adapter-user rosie-ctl'
grep -Fq -- '--core-socket /run/rosie-rt-core/native/ipc.sock --socket /run/rosie-rt-core/public/control.sock' "${control}" || fail 'adapter socket arguments differ'
for name in CONFIGURATION_SHA256 PAIR_ID PAIR_REVISION; do
  grep -Fq -- "\${ROSIE_RT_${name}}" "${control}" || fail 'adapter environment name mismatch'
done
# Home anchors: adapter-written, core-read, one directory both units name.
anchors="${root}/var/lib/rosie-rt-anchors"
expect_line "${log}" "install -d -m 2750 -o rosie-ctl -g rosie-rt ${anchors}"
[[ -d "${anchors}" && "$(stat -c %a "${anchors}")" == 2750 ]] || fail 'anchor directory is not setgid 2750'
for unit in "${core}" "${control}"; do
  expect_line "${unit}" 'Environment=ROSIE_RT_ANCHOR_DIR=/var/lib/rosie-rt-anchors'
done
expect_line "${control}" 'ExecStartPre=/usr/bin/test -w ${ROSIE_RT_ANCHOR_DIR}'
printf 'saved home\n' > "${anchors}/axis-0.anchor"
bash "${installer}" "${binding[@]}"
[[ "$(cat "${anchors}/axis-0.anchor")" == 'saved home' ]] || fail 'reinstall did not keep the saved anchors'

: > "${log}"
bash "${installer}" --enable "${binding[@]}"
expect_line "${log}" 'systemctl enable rosie-rt-core.service rosie-rt-control.service'
if grep -E '^systemctl (enable|start|restart).*rosie-rt-natspublisher' "${log}"; then fail 'optional publisher activated'; fi
[[ "$(grep -c '^systemctl enable ' "${log}")" == 1 ]] || fail 'enable count differs'
if grep -Eq '^systemctl (start|restart)|--now' "${log}"; then fail 'installation started hardware'; fi
if grep -Eq '^(useradd|groupadd) ' "${log}"; then fail 'idempotent install recreated accounts'; fi

: > "${log}"
expect_failure env MOCK_COMPILE_FAIL=1 bash "${installer}" --enable "${binding[@]}"
grep -Fq 'configuration compilation failed; control.env was not replaced' "${scratch}/failure.out" || fail 'compiler failure did not fail closed'
[[ "$(sha256sum "${environment}")" == "${before}" ]] || fail 'compiler failure replaced control.env'
if grep -Eq '^systemctl ' "${log}"; then fail 'compiler failure changed service state'; fi
expect_failure env MOCK_COMPILE_FAIL=1 bash "${generator}" "${binding[@]}"
expect_failure bash "${generator}" "${scratch}/machine.json" $'bad\nPAIR=wrong' 42
expect_failure bash "${generator}" "${scratch}/machine.json" bench-pair 0
expect_failure bash "${generator}" "${scratch}/machine.json" bench-pair 18446744073709551616
expect_failure bash "${generator}" "${scratch}/machine.json" bench-pair 42 J1,J1,J3,J4,J5,J6,J7,J8,J9
expect_failure bash "${generator}" "${scratch}/machine.json" bench-pair 42 J1,J2,J3,J4,J5,J6,J7,J8,missing
expect_failure bash "${generator}" "${component}/config/machines/simulation/simulation-program.json" bench-pair 42
[[ "$(sha256sum "${environment}")" == "${before}" ]] || fail 'rejected configuration replaced control.env'
expect_failure env ROSIE_RT_ETHERCAT_GROUP=$'bad\nUser=root' bash "${installer}"
expect_failure env ROSIE_RT_INSTALL_TEST_ROOT="${scratch}" bash "${installer}"
expect_failure env ROSIE_RT_INSTALL_TEST_ROOT="${scratch}" bash "${generator}" "${binding[@]}"

# --keep-remote-args keeps an installed listener line verbatim and refuses to
# invent one: the published binding above has ROSIE_RT_REMOTE_ARGS="".
expect_failure bash "${generator}" --keep-remote-args "${binding[@]}"
expect_line "${scratch}/failure.out" 'generate-control-env error=installed control.env has no remote listener to keep'
[[ "$(sha256sum "${environment}")" == "${before}" ]] || fail 'refused keep-remote-args replaced control.env'
kept='ROSIE_RT_REMOTE_ARGS="--remote-listen 127.0.0.1:8443 --remote-ca /etc/rosie-rt-core/pki/ca.pem --remote-cert /etc/rosie-rt-core/pki/server.pem --remote-key /etc/rosie-rt-core/pki/server-key.pem --remote-crl /etc/rosie-rt-core/pki/crl.pem --remote-jog-max-uncertainty-ns 100000000"'
sed -i "s|^ROSIE_RT_REMOTE_ARGS=.*|${kept}|" "${environment}"
mkdir -p "${root}/etc/rosie-rt-core/pki"
for file in ca.pem server.pem server-key.pem; do printf 'fixture\n' > "${root}/etc/rosie-rt-core/pki/${file}"; done
expect_failure bash "${generator}" --keep-remote-args "${binding[@]}"
expect_line "${scratch}/failure.out" 'generate-control-env error=missing installed remote PKI crl.pem'
printf 'fixture\n' > "${root}/etc/rosie-rt-core/pki/crl.pem"
sed -i 's|^ROSIE_RT_CONFIGURATION_SHA256=.*|ROSIE_RT_CONFIGURATION_SHA256=stale|' "${environment}"
bash "${generator}" --keep-remote-args "${binding[@]}"
expect_line "${environment}" "${kept}"
expect_line "${environment}" "ROSIE_RT_CONFIGURATION_SHA256=${ROSIE_RT_CONFIGURATION_SHA256}"
[[ "$(grep -c '^ROSIE_RT_REMOTE_ARGS=' "${environment}")" == 1 ]] || fail 'keep-remote-args duplicated the listener line'
expect_failure bash "${generator}" --keep-remote-args --remote-pki "${root}/etc/rosie-rt-core/pki" "${binding[@]}"

# --remote-pki carries the jog-clock policy only when all three values are
# given; without it the listener line is byte-identical to the pre-policy one
# (rt-control then refuses every remote jog) and the generator says so.
pki_source="${scratch}/remote-pki"
mkdir -p "${pki_source}"
for file in ca.pem ca-key.pem ca.cnf serial crlnumber server.pem server-key.pem crl.pem index.txt; do printf 'fixture\n' > "${pki_source}/${file}"; done
printf 'bench-pair\n' > "${pki_source}/pair-id"
listener='ROSIE_RT_REMOTE_ARGS="--remote-listen 127.0.0.1:8443 --remote-ca /etc/rosie-rt-core/pki/ca.pem --remote-cert /etc/rosie-rt-core/pki/server.pem --remote-key /etc/rosie-rt-core/pki/server-key.pem --remote-crl /etc/rosie-rt-core/pki/crl.pem'
bash "${generator}" --remote-pki "${pki_source}" "${binding[@]}" 2> "${scratch}/unqualified.err"
expect_line "${environment}" "${listener}\""
expect_line "${scratch}/unqualified.err" 'generate-control-env warning=remote listener has no jog-clock policy; remote jog stays unqualified (set ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS, ROSIE_RT_REMOTE_JOG_DRIFT_PPB, ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS)'
jog_policy=(ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS=100000000 ROSIE_RT_REMOTE_JOG_DRIFT_PPB=2000000 ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS=10000000000)
env "${jog_policy[@]}" bash "${generator}" --remote-pki "${pki_source}" "${binding[@]}" 2> "${scratch}/qualified.err"
expect_line "${environment}" "${listener} --remote-jog-max-uncertainty-ns 100000000 --remote-jog-drift-ppb 2000000 --remote-jog-calibration-max-age-ns 10000000000\""
[[ "$(grep -c '^ROSIE_RT_REMOTE_ARGS=' "${environment}")" == 1 ]] || fail 'remote-pki duplicated the listener line'
if grep -Fq warning "${scratch}/qualified.err"; then fail 'qualified listener still warned'; fi
# The uint64 ceiling is rt-control's own flag range and is accepted.
env "${jog_policy[@]}" ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS=18446744073709551615 bash "${generator}" --remote-pki "${pki_source}" "${binding[@]}"
expect_line "${environment}" "${listener} --remote-jog-max-uncertainty-ns 100000000 --remote-jog-drift-ppb 2000000 --remote-jog-calibration-max-age-ns 18446744073709551615\""
env "${jog_policy[@]}" bash "${generator}" --remote-pki "${pki_source}" "${binding[@]}"
before="$(sha256sum "${environment}")"
expect_failure env ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS=100000000 ROSIE_RT_REMOTE_JOG_DRIFT_PPB=2000000 bash "${generator}" --remote-pki "${pki_source}" "${binding[@]}"
expect_line "${scratch}/failure.out" 'generate-control-env error=remote jog-clock policy needs all of ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS, ROSIE_RT_REMOTE_JOG_DRIFT_PPB and ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS, or none'
for invalid in ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS=0 ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS=18446744073709551616 \
  ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS=1e8 ROSIE_RT_REMOTE_JOG_DRIFT_PPB=1000000000 ROSIE_RT_REMOTE_JOG_DRIFT_PPB=-1 \
  ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS=010000000000 'ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS=1 --remote-listen 0.0.0.0:1'; do
  expect_failure env "${jog_policy[@]}" "${invalid}" bash "${generator}" --remote-pki "${pki_source}" "${binding[@]}"
  expect_line "${scratch}/failure.out" "generate-control-env error=invalid ${invalid%%=*}: $(if [[ "${invalid}" == *DRIFT* ]]; then printf 'a positive integer below 1000000000'; else printf 'a positive integer up to 18446744073709551615'; fi)"
done
[[ "$(sha256sum "${environment}")" == "${before}" ]] || fail 'refused jog-clock policy replaced control.env'

# A host whose units already name an anchor directory in their own drop-in (the
# hand-made layout of a Rosie 1430 cell) keeps it: either unit's value leaves the directory,
# both units' drop-ins and the existing anchors exactly as they were.
handmade="$(mktemp -d "${TMPDIR}/rosie-rt-install-test.XXXXXX")"
cell_dropin="${handmade}/etc/systemd/system/rosie-rt-control.service.d/45-rosie_1420_v1-anchors.conf"
cell_anchors="${handmade}/var/lib/rosie-rt-core/8e3c9557/anchors"
mkdir -p "${cell_dropin%/*}" "${cell_anchors}"
printf '[Service]\nEnvironment=ROSIE_RT_ANCHOR_DIR=/var/lib/rosie-rt-core/8e3c9557/anchors\n' > "${cell_dropin}"
printf 'fixture anchor\n' > "${cell_anchors}/axis-0.anchor"
chmod 2770 "${cell_anchors}"
handmade_before="$(cat "${cell_dropin}" "${cell_anchors}/axis-0.anchor" | sha256sum)"
for pass in first repeat; do
  ROSIE_RT_INSTALL_TEST_ROOT="${handmade}" bash "${installer}" > "${scratch}/handmade.out" || { rm -rf -- "${handmade}"; fail "install over a cell anchor drop-in failed (${pass})"; }
  expect_line "${scratch}/handmade.out" 'install component=anchors status=kept reason=existing-configuration source="/etc/systemd/system/rosie-rt-control.service.d/45-rosie_1420_v1-anchors.conf" changed=none'
done
handmade_after="$(cat "${cell_dropin}" "${cell_anchors}/axis-0.anchor" | sha256sum)"
handmade_mode="$(stat -c %a "${cell_anchors}")"
handmade_created="$(find "${handmade}/var/lib" "${handmade}/etc/systemd/system" \( -name rosie-rt-anchors -o -name 20-rosie-rt-anchors.conf -o -name rosie-rt-core.service.d \) -print)"
rm -rf -- "${handmade}"
[[ "${handmade_after}" == "${handmade_before}" && "${handmade_mode}" == 2770 ]] || fail 'install changed a cell anchor drop-in or its directory'
[[ -z "${handmade_created}" ]] || fail "install added anchor provisioning beside a cell drop-in: ${handmade_created}"

# Exercise the foundation's device-only helper without its host/network provisioning path.
eval "$(sed -n '/^configure_ethercat_device_access() {/,/^}/p' "${component}/host/ethercat-foundation.sh")"
host_path() { printf '%s%s\n' "${root}" "$1"; }
ETHERCAT_DEVICE_GROUP=ethercat
: > "${log}"
configure_ethercat_device_access
rule="${root}/etc/udev/rules.d/99-rosie-rt-core-ethercat.rules"
expect_line "${rule}" 'KERNEL=="EtherCAT0", GROUP="ethercat", MODE="0660"'
expect_line "${log}" 'udevadm control --reload-rules'
expect_line "${log}" 'udevadm trigger --action=change --sysname-match=EtherCAT0'
[[ "$(stat -c %a "${rule}")" == 644 ]] || fail 'udev rule mode differs'
: > "${log}"
configure_ethercat_device_access
if grep -Eq '^(install|groupadd|udevadm) ' "${log}"; then fail 'matching device rule churned'; fi
ETHERCAT_DEVICE_GROUP=benchbus
configure_ethercat_device_access
expect_line "${rule}" 'KERNEL=="EtherCAT0", GROUP="benchbus", MODE="0660"'
# Existing incompatible account membership must be refused, never silently broadened.
expect_failure env ROSIE_RT_ETHERCAT_GROUP=benchbus bash "${installer}"
printf 'rosie-rt rosie-ctl benchbus\n' > "${MOCK_ACCOUNTS}/groups.rosie-rt"
ROSIE_RT_ETHERCAT_GROUP=benchbus bash "${installer}"
expect_line "${core}" 'SupplementaryGroups=rosie-rt rosie-ctl benchbus'

if command -v systemd-analyze >/dev/null; then
  # systemd 249 cannot verify --root: relocate only the binary prefix in verification copies.
  verification="${root}/verify"
  mkdir -p "${verification}"
  for unit in rosie-rt-core rosie-rt-control rosie-rt-natspublisher; do
    sed "s|/opt/rosie/rt-core|${root}/opt/rosie/rt-core|g" "${root}/etc/systemd/system/${unit}.service" > "${verification}/${unit}.service"
    if [[ -d "${root}/etc/systemd/system/${unit}.service.d" ]]; then cp -R "${root}/etc/systemd/system/${unit}.service.d" "${verification}/"; fi
  done
  printf '[Service]\nType=oneshot\nExecStart=/usr/bin/true\n' > "${verification}/ethercat.service"
  for target in sysinit basic shutdown; do
    printf '[Unit]\nDescription=mock dependency\n' > "${verification}/${target}.target"
  done
  SYSTEMD_UNIT_PATH="${verification}" systemd-analyze verify --man=no "${verification}/rosie-rt-core.service" "${verification}/rosie-rt-control.service" "${verification}/rosie-rt-natspublisher.service"
  printf 'systemd-analyze verify status=passed\n'
else
  printf 'systemd-analyze verify status=skipped reason=not-installed\n'
fi
printf 'install-test status=passed contracts=identities,idempotence,units,configuration,permissions,activation,compile-refusal,keep-remote-args,remote-jog-policy,anchors,udev,prune-refusals\n'
