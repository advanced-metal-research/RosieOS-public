#!/usr/bin/env bash
# Exercise real package inspection and confined installation; mocks refuse service activation.
set -euo pipefail
export LC_ALL=C
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${component}"
scratch="$(mktemp -d "${component}/build/package-test.XXXXXX")"
# The fake install root must keep POSIX file modes (the tamper case flips one),
# which the Windows-backed build directory cannot, so it lives under TMPDIR. Two
# installed package versions occupy about 52 MB there; on the shared 128 MB test
# tmpfs concurrent runs exhausted it, so wait for room instead of failing on
# "No space left on device". The bound is a test-harness policy, unverified.
package_root_needed_kb=65536
for attempt in $(seq 1 45); do
  available_kb="$(df --output=avail -k "${TMPDIR:?}" | tail -1 | tr -d ' ')"
  [[ "${available_kb}" -ge "${package_root_needed_kb}" ]] && break
  [[ "${attempt}" -eq 1 ]] && echo "package-test: waiting for ${package_root_needed_kb} KB free under ${TMPDIR} (have ${available_kb} KB)" >&2
  sleep 2
done
[[ "${available_kb}" -ge "${package_root_needed_kb}" ]] || { echo "package-test: TMPDIR filesystem has ${available_kb} KB free; need ${package_root_needed_kb} KB" >&2; exit 1; }
root="$(mktemp -d "${TMPDIR:?}/rosie-rt-install-test.XXXXXX")"
trap 'rm -rf -- "${scratch}" "${root}"' EXIT
fail() { printf 'package-test error=%s\n' "$*" >&2; exit 1; }
expect_status() {
  local expected="$1" observed=0; shift
  "$@" > "${scratch}/status.log" 2>&1 || observed=$?
  [[ "${observed}" == "${expected}" ]] || { cat "${scratch}/status.log"; fail "expected exit ${expected}, observed ${observed}: $*"; }
}
while IFS= read -r -d '' file; do bash -n "${file}"; done < <(find host tools -name '*.sh' -type f -print0)
bash -n host/run-core
# Refuse hidden roots and descendants before staging so ProtectHome stays enabled.
for invalid in /home /home/pi/.rosie/releases/robot-stack-next /root /root/releases /run/user /run/user/1000/releases; do
  expect_status 1 env ROSIE_RT_PACKAGE_SLOT_ROOT="${invalid}" ROSIE_RT_PACKAGE_CORE=build/rosie-rt-core-sim bash tools/package-runtime.sh "${scratch}/hidden"
  grep -F 'ProtectHome=yes' "${scratch}/status.log" >/dev/null || fail "missing ProtectHome=yes reason for ${invalid}"
  [[ ! -e "${scratch}/hidden" && ! -e "${scratch}/hidden.components" ]] || fail 'hidden slot created output'
done
if [[ ! -x build/rosie-rt-core-sim ]]; then make sim; fi
if [[ ! -x build/rt-control || ! -x build/rtctl || ! -x build/rt-natspublisher ]]; then make control; fi
if [[ ! -f build/deps/igh-prefix/lib/libethercat.so.1 ]]; then bash tools/build-igh-userlib.sh; fi
# Measured reproducibility input: an epoch of 1 second is fixed for both staging operations.
export SOURCE_DATE_EPOCH=1 ROSIE_RT_PACKAGE_CORE=build/rosie-rt-core-sim
bash tools/package-runtime.sh "${scratch}/first"
# Measured legacy pin: the default package must still identify the exact 1.6.9 archive.
# A cell whose deps select another reviewed version (the release gate exports
# ROSIE_RT_IGH_VERSION; a Pi 5 servo host runs 1.6.12) identifies that one's.
igh_expect="${ROSIE_RT_IGH_VERSION:-1.6.9}"
read -r _ igh_sha _ < <(bash tools/build-igh-userlib.sh --print-pin "${igh_expect}")
if [[ "${igh_expect}" == 1.6.9 ]]; then
  [[ "${igh_sha}" == 2bc4d6b69ed980b896ecae9ea6603b327562dfacd3b8c1ed4e9e599a4cc4dfc3 ]] || fail '1.6.9 archive pin changed'
fi
grep -F "\"igh_version\": \"${igh_expect}\"" "${scratch}/first/rt_package.json"
grep -F "\"igh_archive_sha256\": \"${igh_sha}\"" "${scratch}/first/rt_package.json"
grep -Fx '  "component_version": "0.1.0",' "${scratch}/first/rt_package.json"
[[ -x "${scratch}/first/bin/rt-natspublisher" ]] || fail 'publisher binary missing from package'
bash tools/package-runtime.sh "${scratch}/second"
cmp "${scratch}/first/rt_package.json" "${scratch}/second/rt_package.json"
cmp "${scratch}/first.components/SHA256SUMS" "${scratch}/second.components/SHA256SUMS"
(cd "${scratch}/first.components" && sha256sum -c SHA256SUMS)
expect_status 1 bash tools/package-runtime.sh "${scratch}/first"
mkdir "${scratch}/collision.components"
printf 'retained\n' > "${scratch}/collision.components/marker"
expect_status 1 bash tools/package-runtime.sh "${scratch}/collision"
[[ ! -e "${scratch}/collision" ]] || fail 'asset collision created install output'
grep -Fx retained "${scratch}/collision.components/marker"
for invalid in relative /slot/../escape '/slot with space' '/slot%h' /slot/; do
  expect_status 1 env ROSIE_RT_PACKAGE_SLOT_ROOT="${invalid}" bash tools/package-runtime.sh "${scratch}/invalid"
  [[ ! -e "${scratch}/invalid" && ! -e "${scratch}/invalid.components" ]] || fail 'invalid slot created output'
done
expect_status 1 env SOURCE_DATE_EPOCH=invalid bash tools/package-runtime.sh "${scratch}/invalid"
[[ ! -e "${scratch}/invalid" ]] || fail 'invalid epoch created output'
printf 'fixture producer invocation\n' > "${scratch}/producer.log"
ROSIE_RT_PACKAGE_SLOT_ROOT=/srv/rosie/releases/robot-stack-next ROSIE_RT_PACKAGE_BUILD_LOG="${scratch}/producer.log" bash tools/package-runtime.sh "${scratch}/slot"
cmp "${scratch}/producer.log" "${scratch}/slot.components/build.log"
for unit in rosie-rt-core rosie-rt-control rosie-rt-natspublisher; do
  grep -F 'ExecStart=/srv/rosie/releases/robot-stack-next/bin/' "${scratch}/slot/host/${unit}.service" >/dev/null
  grep -Fx 'WorkingDirectory=/srv/rosie/releases/robot-stack-next' "${scratch}/slot/host/${unit}.service" >/dev/null
done
grep -Fx 'ProtectHome=yes' "${scratch}/slot/host/rosie-rt-natspublisher.service"
grep -Fx 'ProtectHome=yes' "${scratch}/first/host/rosie-rt-natspublisher.service"
mkdir "${root}/unpacked"
tar -xzf "${scratch}/first.components/rosie-rt-core-0.1.0-linux-$(uname -m | sed 's/x86_64/amd64/;s/aarch64/arm64/').tar.gz" -C "${root}/unpacked"
expect_status 0 bash "${root}/unpacked/tools/verify-package.sh" "${root}/unpacked"
cmp "${scratch}/first/bin/rt-control" "${root}/unpacked/bin/rt-control"
cmp "${scratch}/first/lib/libethercat.so.1" "${root}/unpacked/lib/libethercat.so.1"
expect_status 0 bash "${scratch}/second/tools/verify-package.sh" "${scratch}/second"
grep -F '"backend": "simulation"' "${scratch}/status.log"
grep -F 'reproducible=false path=lib/libethercat.so.1 reason=' "${scratch}/status.log"
# Configuration compilation uses the relocated Go CLI and explicitly selected profile tree.
"${scratch}/second/bin/rtctl" compile --config "${scratch}/second/config/machines/simulation/simulation-program.json" --out "${scratch}/compiled" > "${scratch}/compile.log"
[[ "$(basename "$(cat "${scratch}/compile.log")")" =~ ^[0-9a-f]{64}$ ]] || fail 'missing compiled configuration identity'
cp -R "${scratch}/first" "${scratch}/tampered"
printf 'tampered\n' >> "${scratch}/tampered/config/machines/simulation/simulation-program.json"
expect_status 3 bash tools/verify-package.sh "${scratch}/tampered"
cp -R "${scratch}/first" "${scratch}/missing"
rm "${scratch}/missing/bin/rtctl"
expect_status 3 bash tools/verify-package.sh "${scratch}/missing"
cp -R "${scratch}/first" "${scratch}/extra"
printf 'untracked\n' > "${scratch}/extra/untracked"
expect_status 3 bash tools/verify-package.sh "${scratch}/extra"
cp -R "${scratch}/first" "${root}/mode"
build/rt-package install-modes "${root}/mode"
expect_status 0 bash tools/verify-package.sh "${root}/mode"
chmod 0600 "${root}/mode/bin/rtctl"
expect_status 3 bash tools/verify-package.sh "${root}/mode"
cp -R "${scratch}/first" "${scratch}/symlink"
rm "${scratch}/symlink/bin/rtctl"
ln -s "${scratch}/first/bin/rtctl" "${scratch}/symlink/bin/rtctl"
expect_status 3 bash tools/verify-package.sh "${scratch}/symlink"
cp -R "${scratch}/first" "${scratch}/arch"
sed -i 's/"architecture": "[^"]*"/"architecture": "incompatible-test-architecture"/' "${scratch}/arch/rt_package.json"
expect_status 2 bash tools/verify-package.sh "${scratch}/arch"

mkdir "${scratch}/mock"
cat > "${scratch}/mock/dispatch" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
command="${0##*/}"
printf '%s %s\n' "${command}" "$*" >> "${MOCK_LOG}"
case "${command}" in
  getent)
    case "$1" in
      group) printf '%s:x:123:\n' "$2" ;;
      passwd) printf '%s:x:123:123::/nonexistent:/usr/sbin/nologin\n' "$2" ;;
      *) exit 97 ;;
    esac ;;
  id)
    case "$1" in
      -gn) printf '%s\n' "$2" ;;
      -Gn) printf 'rosie-rt rosie-ctl rosie-rt-clients ethercat\n' ;;
      *) exit 97 ;;
    esac ;;
  install)
    [[ "${!#}" == "${ROSIE_RT_INSTALL_TEST_ROOT}/"* ]] || exit 97
    # An unprivileged test cannot assign service owners; the mode still applies.
    args=()
    while [[ $# -gt 0 ]]; do case "$1" in -o|-g) shift 2 ;; *) args+=("$1"); shift ;; esac; done
    exec /usr/bin/install "${args[@]}" ;;
  systemctl) [[ "$*" == daemon-reload ]] || exit 97 ;;
  *) exit 97 ;;
esac
SH
chmod +x "${scratch}/mock/dispatch"
for command in getent id groupadd useradd install systemctl; do ln -s dispatch "${scratch}/mock/${command}"; done
export ROSIE_RT_INSTALL_TEST_ROOT="${root}" MOCK_LOG="${scratch}/calls.log"
export PATH="${scratch}/mock:${PATH}"
: > "${MOCK_LOG}"
expect_status 1 bash host/install.sh --from-package "${scratch}/tampered"
[[ ! -s "${MOCK_LOG}" && ! -e "${root}/opt" ]] || fail 'rejected package mutated the mock host'
expect_status 1 bash host/install.sh --from-package "${scratch}/arch"
[[ ! -s "${MOCK_LOG}" && ! -e "${root}/opt" ]] || fail 'missing prerequisite mutated the mock host'
bash "${scratch}/first/host/install.sh" --from-package "${scratch}/first"
revision="$(build/rt-package git-sha "${scratch}/first")"
store="${root}/opt/rosie-rt-core"
[[ "$(readlink "${store}/current")" == "${revision}" ]] || fail 'current release differs'
expect_status 0 bash tools/verify-package.sh "${store}/${revision}"
for unit in rosie-rt-core rosie-rt-control rosie-rt-natspublisher; do
  grep -F 'ExecStart=/opt/rosie-rt-core/current/bin/' "${root}/etc/systemd/system/${unit}.service" >/dev/null
  grep -Fx 'WorkingDirectory=/opt/rosie-rt-core/current' "${root}/etc/systemd/system/${unit}.service" >/dev/null
done
grep -Fx 'User=rosie-rt' "${root}/etc/systemd/system/rosie-rt-core.service"
grep -Fx 'User=rosie-ctl' "${root}/etc/systemd/system/rosie-rt-control.service"
grep -Fx 'DynamicUser=yes' "${root}/etc/systemd/system/rosie-rt-natspublisher.service"
# The deploy path (--from-package) provisions the Home-anchor directory and both drop-ins too.
[[ "$(stat -c %a "${root}/var/lib/rosie-rt-anchors")" == 2750 ]] || fail 'package install did not provision the anchor directory'
for unit in rosie-rt-core rosie-rt-control; do
  grep -Fx 'Environment=ROSIE_RT_ANCHOR_DIR=/var/lib/rosie-rt-anchors' "${root}/etc/systemd/system/${unit}.service.d/20-rosie-rt-anchors.conf" >/dev/null
done
[[ -x "${store}/${revision}/bin/rt-natspublisher" ]] || fail 'installed publisher missing'
# Measured fixture identity: 40 hex digits emulate a second revision without mutating git.
next_revision=1111111111111111111111111111111111111111
sed -i "s/\"git_sha\": \"${revision}\"/\"git_sha\": \"${next_revision}\"/" "${scratch}/second/rt_package.json"
bash "${scratch}/second/host/install.sh" --from-package "${scratch}/second"
[[ "$(readlink "${store}/current")" == "${next_revision}" && "$(readlink "${store}/previous")" == "${revision}" ]] || fail 'upgrade selection differs'
bash "${store}/${next_revision}/host/install.sh" --rollback
[[ "$(readlink "${store}/current")" == "${revision}" && "$(readlink "${store}/previous")" == "${next_revision}" ]] || fail 'rollback selection differs'
if grep -E '^systemctl (start|enable|restart)' "${MOCK_LOG}"; then fail 'package operation activated services'; fi
printf 'package-test status=passed contracts=syntax,manifest-reproducibility,archive-reproducibility,archive-relocation,no-overwrite,slot-root,configuration-sha,tamper,missing,extra,mode,symlink,prerequisite,install,upgrade,rollback\n'
