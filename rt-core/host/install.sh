#!/usr/bin/env bash
# Install inactive services; activation requires an explicit flag and a compiled binding.
set -euo pipefail
export LC_ALL=C
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export PATH="${PATH}:${component}/bin:${component}/build"

fail() { printf 'install error=%s\n' "$*" >&2; exit 1; }

enable=0
remote_pki=""
from_package=""
rollback=0
# Unverified opt-in policy: --live requires verified hardware artifacts.
live=0
prune=0
pins=()
while [[ "${1:-}" == --* ]]; do
  case "$1" in
    --enable) enable=1; shift ;;
    --remote-pki) [[ $# -ge 2 ]] || fail '--remote-pki requires a directory'; remote_pki="$2"; shift 2 ;;
    --from-package) [[ $# -ge 2 ]] || fail '--from-package requires a directory'; from_package="$2"; shift 2 ;;
    --rollback) rollback=1; shift ;;
    --live) live=1; shift ;;
    --prune) prune=1; shift ;;
    --pin) [[ $# -ge 2 && "$2" =~ ^[0-9a-f]{40}$ ]] || fail '--pin requires a 40-hex release revision'; pins+=("$2"); shift 2 ;;
    *) fail 'unknown installation option' ;;
  esac
done
[[ ( ${prune} == 0 && ${#pins[@]} == 0 ) || ( -z "${from_package}" && ${enable} == 0 && ${rollback} == 0 && ${live} == 0 && $# == 0 && -z "${remote_pki}" ) ]] || fail '--prune and --pin cannot be combined with installation options'
[[ $# == 0 || $# == 3 ]] || fail 'usage: install.sh [--enable] [--remote-pki directory] [machine.json pair-id pair-revision]'
[[ ${enable} == 0 || $# == 3 ]] || fail '--enable requires machine.json, pair-id and pair-revision'
[[ -z "${remote_pki}" || $# == 3 ]] || fail '--remote-pki requires an explicit compiled pair binding'
[[ ${rollback} == 0 || ( -z "${from_package}" && ${enable} == 0 && $# == 0 && -z "${remote_pki}" ) ]] || fail '--rollback cannot be combined with installation options'
[[ ${live} == 0 || ( -n "${from_package}" && ${rollback} == 0 ) ]] || fail '--live requires --from-package'
# Package selection remains inactive until a separately compiled deployment binding is installed.
[[ -z "${from_package}" || ( $# == 0 && ${enable} == 0 && -z "${remote_pki}" ) ]] || fail '--from-package installs inactive artifacts only; provision the deployment binding separately'

root="${ROSIE_RT_INSTALL_TEST_ROOT:-}"
if [[ -n "${root}" ]]; then
  # The test seam confines installation to a dedicated directory on the test runtime filesystem.
  root="$(realpath -e -- "${root}")"
  [[ -n "${TMPDIR:-}" && "${root}" == "$(realpath -e -- "${TMPDIR}")/rosie-rt-install-test."* ]] || fail 'test root must be a dedicated TMPDIR/rosie-rt-install-test.* directory'
else
  [[ ${EUID} == 0 ]] || fail 'run installation as root'
fi
destination="${root}/opt/rosie/rt-core"
store="${root}/opt/rosie-rt-core"
inspector="${component}/bin/rt-package"
if [[ ! -x "${inspector}" ]]; then inspector="${component}/build/rt-package"; fi

# Atomic relative symlink replacement keeps each selected release inside its store.
select_release() {
  local name="$1" revision="$2" pending
  [[ "${revision}" =~ ^[0-9a-f]{40}$ && -d "${store}/${revision}" && ! -L "${store}/${revision}" ]] || fail 'invalid release target'
  pending="$(mktemp -d "${store}/.select.XXXXXX")"
  ln -s "${revision}" "${pending}/${name}"
  mv -Tf -- "${pending}/${name}" "${store}/${name}"
  rmdir "${pending}"
}

verify_release() {
  [[ "$1" =~ ^[0-9a-f]{40}$ && -d "${store}/$1" && ! -L "${store}/$1" ]] || fail 'invalid stored release'
  bash "${component}/tools/verify-package.sh" "${store}/$1" || fail 'stored package verification failed'
  [[ "$("${inspector}" git-sha "${store}/$1")" == "$1" ]] || fail 'stored release identity differs from directory'
}

# Pruning frees the store without weakening recovery: the releases selected by
# current and previous (the rollback target) and every revision listed in
# ${store}/pinned (one 40-hex revision per line, '#' comments allowed; known-good
# releases kept for a later --from-package reinstall) are kept. Only 40-hex
# release directories are candidates; symlinks, unknown names and installer
# staging are never removed, and nothing is followed out of the store. Each
# release is renamed to .prune.* inside the store before it is deleted, so an
# interrupted prune never leaves a partial release under a revision name; the
# next prune removes the leftovers. Units and /etc are never touched.
pin_releases() {
  local pinned="${store}/pinned" pending revision
  for revision in "$@"; do
    [[ -d "${store}/${revision}" && ! -L "${store}/${revision}" ]] || fail "cannot pin ${revision}: not a stored release"
  done
  pending="$(mktemp "${store}/.pinned.XXXXXX")"
  { [[ ! -f "${pinned}" ]] || cat -- "${pinned}"; } > "${pending}"
  for revision in "$@"; do
    if ! grep -Fxq -- "${revision}" "${pending}"; then printf '%s\n' "${revision}" >> "${pending}"; fi
    printf 'prune pinned=%s\n' "${revision}"
  done
  chmod 0644 "${pending}"
  mv -f -- "${pending}" "${pinned}"
}

prune_releases() {
  local -A keep=()
  local name target line entry size removed=0 kept=0 freed_kb=0 trash
  for name in current previous; do
    [[ -e "${store}/${name}" || -L "${store}/${name}" ]] || continue
    [[ -L "${store}/${name}" ]] || fail "${name} must be a release symlink; prune refused"
    target="$(readlink "${store}/${name}")"
    [[ "${target}" =~ ^[0-9a-f]{40}$ && -d "${store}/${target}" && ! -L "${store}/${target}" ]] || fail "${name} does not select a stored release; prune refused"
    keep["${target}"]+="${keep[${target}]:+,}${name}"
  done
  if [[ -e "${store}/pinned" || -L "${store}/pinned" ]]; then
    [[ -f "${store}/pinned" && ! -L "${store}/pinned" ]] || fail 'pinned must be a regular file; prune refused'
    while IFS= read -r line || [[ -n "${line}" ]]; do
      line="${line%%#*}"; line="${line//[[:space:]]/}"
      [[ -n "${line}" ]] || continue
      [[ "${line}" =~ ^[0-9a-f]{40}$ ]] || fail 'pinned lists a malformed revision; prune refused'
      keep["${line}"]+="${keep[${line}]:+,}pinned"
    done < "${store}/pinned"
  fi
  for entry in "${store}"/.prune.*; do
    [[ -d "${entry}" && ! -L "${entry}" ]] || continue
    rm -rf --one-file-system -- "${entry}"
    printf 'prune removed=%s reason=interrupted-prune\n' "${entry##*/}"
  done
  for entry in "${store}"/*; do
    name="${entry##*/}"
    [[ "${name}" =~ ^[0-9a-f]{40}$ && -d "${entry}" && ! -L "${entry}" ]] || continue
    size="$(du -sk -- "${entry}" | cut -f1)"
    if [[ -n "${keep[${name}]:-}" ]]; then
      printf 'prune kept=%s reason=%s size_kb=%s\n' "${name}" "${keep[${name}]}" "${size}"
      kept=$((kept + 1))
      continue
    fi
    trash="$(mktemp -d "${store}/.prune.XXXXXX")"
    mv -T -- "${entry}" "${trash}/release"
    rm -rf --one-file-system -- "${trash}"
    printf 'prune removed=%s size_kb=%s\n' "${name}" "${size}"
    removed=$((removed + 1)) freed_kb=$((freed_kb + size))
  done
  printf 'prune status=done removed=%s kept=%s freed_kb=%s\n' "${removed}" "${kept}" "${freed_kb}"
}

if [[ ${prune} == 1 || ${#pins[@]} -gt 0 ]]; then
  if [[ ! -e "${store}" && ! -L "${store}" && ${#pins[@]} == 0 ]]; then
    printf 'prune status=done removed=0 kept=0 freed_kb=0 store=absent\n'
    exit 0
  fi
  [[ -d "${store}" && ! -L "${store}" ]] || fail 'release store must be a directory'
  if [[ ${#pins[@]} -gt 0 ]]; then pin_releases "${pins[@]}"; fi
  if [[ ${prune} == 1 ]]; then prune_releases; fi
  exit 0
fi

if [[ ${rollback} == 1 ]]; then
  [[ ! -e "${store}/.rollback-consumed" ]] || fail 'no earlier release for rollback; previous rollback already consumed'
  [[ -L "${store}/current" && -L "${store}/previous" ]] || fail 'no previous release for rollback'
  current="$(readlink "${store}/current")"
  previous="$(readlink "${store}/previous")"
  verify_release "${previous}"
  verify_release "${current}"
  select_release current "${previous}"
  select_release previous "${current}"
  printf '%s\n' "${current}" > "${store}/.rollback-consumed"
  printf 'Rolled back current to %s. No units were started.\n' "${previous}"
  exit 0
fi

if [[ -n "${from_package}" ]]; then
  from_package="$(realpath -e -- "${from_package}")"
  bash "${component}/tools/verify-package.sh" "${from_package}" || fail 'package verification failed; installation refused'
  if [[ ${live} == 1 ]]; then
    [[ "$("${inspector}" backend "${from_package}")" == live ]] || fail '--live requires a live package; simulation package refused'
  fi
  revision="$("${inspector}" git-sha "${from_package}")"
  destination="${store}/${revision}"
  if [[ -e "${store}/current" || -L "${store}/current" ]]; then
    [[ -L "${store}/current" ]] || fail 'current must be a release symlink'
    current="$(readlink "${store}/current")"
    verify_release "${current}"
  else
    current=""
  fi
  if [[ -e "${destination}" || -L "${destination}" ]]; then
    verify_release "${revision}"
    cmp -s "${from_package}/rt_package.json" "${destination}/rt_package.json" || fail 'refusing to replace different artifacts for the same git identity'
  fi
fi
# unverified device-group default; the foundation uses the same deployment variable.
device_group="${ROSIE_RT_ETHERCAT_GROUP:-ethercat}"
[[ "${device_group}" =~ ^[a-z_][a-z0-9_-]*$ ]] || fail 'invalid ROSIE_RT_ETHERCAT_GROUP'

# Validate deployment telemetry before host mutation; absent values leave the
# optional observer unconfigured. The binary emits environment syntax, not shell.
publisher_environment=""
if [[ -n "${ROSIE_RT_NATS_URL:-}" || -n "${ROSIE_RT_NATS_HOST:-}" || -n "${ROSIE_RT_NATS_CADENCE:-}" ]]; then
  publisher_binary="${component}/build/rt-natspublisher"
  if [[ -n "${from_package}" ]]; then
    publisher_binary="${from_package}/bin/rt-natspublisher"
  elif [[ ! -x "${publisher_binary}" ]]; then
    publisher_binary="${component}/bin/rt-natspublisher"
  fi
  if [[ -x "${publisher_binary}" ]]; then
    publisher_environment="$("${publisher_binary}" --print-env)" || fail 'invalid NATS deployment configuration'
  else
    printf '%s\n' 'install component=rt-natspublisher status=unavailable reason=optional-binary-missing environment=not-written' >&2
  fi
fi

# Home anchors (docs/APPLICATION_API.md, ROSIE_RT_ANCHOR_DIR): the adapter
# writes them after Home and the core reads them for restore_anchor, so both
# units need the same persistent directory, owned by the adapter with the
# native-reader group and setgid 2750 (the only layout whose files the store
# makes 0640, readable by the core). Without it a fresh host has no anchor
# directory at all and the deploy tool's verify refuses it. The path is outside
# /run (deleted at every stop), the release store (pruned and rolled back) and
# the core's StateDirectory=rosie-rt-core: systemd chowns a StateDirectory to
# the core's user when its top level drifts, and a core-owned directory makes
# the adapter write 0600 files the core cannot read (a Rosie 1430 arm, 2026-10-06: every
# axis anchor_missing until the directory was rosie-ctl:rosie-rt 2750; DEVLOG).
# The installer's drop-ins carry the value, never control.env: EnvironmentFile=
# overrides Environment=, so a control.env value would silently replace a
# cell's own drop-in. A host whose units already get ROSIE_RT_ANCHOR_DIR from
# anything else (two Rosie 1430 cells were set up by hand at cutover) keeps it: no
# directory or drop-in is created or changed there. An existing directory is
# never chowned or chmodded; it may hold the only Home evidence.
anchor_dir=/var/lib/rosie-rt-anchors
anchor_dropin=20-rosie-rt-anchors.conf
anchor_units=(rosie-rt-core rosie-rt-control)
anchor_source=""
for unit in "${anchor_units[@]}"; do
  for dropin in "${root}/etc/systemd/system/${unit}.service.d/"*.conf; do
    [[ -e "${dropin}" && "${dropin##*/}" != "${anchor_dropin}" ]] || continue
    if grep -Fq ROSIE_RT_ANCHOR_DIR "${dropin}"; then anchor_source="${dropin#"${root}"}"; break 2; fi
  done
  # The test root has no service manager; its drop-in files are the whole evidence.
  if [[ -z "${root}" ]]; then
    shown="$(systemctl show "${unit}.service" -p Environment --value 2>/dev/null)" || shown=""
    # Every assignment counts: only this installer's own single value is not "existing".
    value="$(tr ' ' '\n' <<<"${shown}" | sed -n 's/^ROSIE_RT_ANCHOR_DIR=//p')"
    if [[ -n "${value}" && ( "${value}" != "${anchor_dir}" || ! -f "/etc/systemd/system/${unit}.service.d/${anchor_dropin}" ) ]]; then
      anchor_source="${unit}.service Environment ROSIE_RT_ANCHOR_DIR=${value//$'\n'/,}"; break
    fi
  fi
done
if [[ -z "${anchor_source}" ]] && grep -Eq '^ROSIE_RT_ANCHOR_DIR=' "${root}/etc/rosie-rt-core/control.env" 2>/dev/null; then
  anchor_source=/etc/rosie-rt-core/control.env
fi
if [[ -z "${anchor_source}" && ( -e "${root}${anchor_dir}" || -L "${root}${anchor_dir}" ) ]]; then
  [[ -d "${root}${anchor_dir}" && ! -L "${root}${anchor_dir}" ]] || fail "${anchor_dir} exists and is not a directory"
fi

# Reject unavailable configuration destinations before creating identities or artifacts.
anchor_targets=()
if [[ -z "${anchor_source}" ]]; then anchor_targets=("${root}${anchor_dir}"); fi
for target in "${root}/etc/rosie-rt-core" "${root}/etc/systemd/system" "${destination}" "${anchor_targets[@]}"; do
  parent="${target}"
  while [[ ! -e "${parent}" ]]; do parent="$(dirname "${parent}")"; done
  [[ -d "${parent}" && -w "${parent}" && -x "${parent}" ]] || fail "permission denied creating ${target}"
done
if [[ $# == 3 ]]; then
  [[ -f "$1" && -r "$1" ]] || fail 'missing or unreadable machine configuration'
  if [[ -n "${remote_pki}" ]]; then
    [[ -d "${remote_pki}" ]] || fail 'missing remote PKI directory'
  fi
  # Compile before host mutation; a refused compiler cannot leave accounts or configuration directories.
  preflight="$(mktemp -d "${TMPDIR:-/tmp}/rosie-rt-preflight.XXXXXX")"
  trap 'rm -rf -- "${preflight}"' EXIT
  rtctl compile --config "$1" --out "${preflight}" >/dev/null ||
    fail 'configuration compilation failed; control.env was not replaced'
  rm -rf -- "${preflight}"
  trap - EXIT
fi

# unverified identity policy: private native/config access is separate from public clients.
for group in rosie-rt rosie-ctl rosie-rt-clients "${device_group}"; do
  getent group "${group}" >/dev/null || groupadd --system "${group}"
done

ensure_user() {
  local user="$1" primary="$2" supplementary="$3" entry name password uid gid gecos home shell group
  if ! getent passwd "${user}" >/dev/null; then
    useradd --system --gid "${primary}" --groups "${supplementary}" \
      --home-dir /nonexistent --no-create-home --shell /usr/sbin/nologin "${user}"
  fi
  entry="$(getent passwd "${user}")"
  IFS=: read -r name password uid gid gecos home shell <<<"${entry}"
  [[ "${uid}" != 0 && "${home}" == /nonexistent && "${shell}" == /usr/sbin/nologin ]] || fail "unsafe existing service identity ${user}"
  [[ "$(id -gn "${user}")" == "${primary}" ]] || fail "unexpected primary group for ${user}"
  for group in ${supplementary//,/ }; do
    [[ " $(id -Gn "${user}") " == *" ${group} "* ]] || fail "missing ${group} membership for ${user}"
  done
}
ensure_user rosie-rt rosie-rt "rosie-ctl,${device_group}"
ensure_user rosie-ctl rosie-ctl rosie-rt-clients

if [[ $# == 3 ]]; then
  remote_options=()
  if [[ -n "${remote_pki}" ]]; then remote_options=(--remote-pki "${remote_pki}"); fi
  bash "${component}/host/generate-control-env.sh" "${remote_options[@]}" "$@"
fi
if [[ -n "${from_package}" ]]; then
  install -d -m 0755 "${store}" "${root}/etc/rosie-rt-core" "${root}/etc/systemd/system"
  if [[ ! -d "${destination}" ]]; then
    staging="$(mktemp -d "${store}/.package.XXXXXX")"
    trap 'rm -rf -- "${staging}"' EXIT
    cp -R --no-preserve=ownership "${from_package}/." "${staging}/"
    "${inspector}" install-modes "${staging}" || fail 'copied package modes could not be restored'
    bash "${component}/tools/verify-package.sh" "${staging}" || fail 'copied package verification failed'
    mv -T -- "${staging}" "${destination}"
    trap - EXIT
  fi
  unit_source="${destination}/host"
else
# Preserve source-tree installation for the existing compiler and its oracle tests.
library="$(ldd "${component}/build/rosie-rt-core" | awk '$1 == "libethercat.so.1" && $2 == "=>" && $3 ~ /^\// { print $3 }')"
[[ -f "${library}" ]] || fail 'cannot resolve linked IgH userspace library'
licenses="${ECRT_LICENSE_DIR:-${component}/build/deps/ethercat-1.6.9}"
install -d -m 0755 "${destination}/build" "${destination}/lib" "${destination}/licenses/igh"
install -m 0755 "${component}/build/rosie-rt-core" "${component}/build/rt-control" "${component}/build/rtctl" "${destination}/build/"
if [[ -x "${component}/build/rt-natspublisher" ]]; then
  install -m 0755 "${component}/build/rt-natspublisher" "${destination}/build/"
fi
install -m 0644 "${library}" "${destination}/lib/libethercat.so.1"
install -m 0644 "${licenses}/COPYING" "${licenses}/COPYING.LESSER" "${destination}/licenses/igh/"
# unverified installation policy: root owns executable code and public unit definitions.
install -d -m 0755 "${destination}" "${destination}/build" "${root}/etc/rosie-rt-core" "${root}/etc/systemd/system"
for directory in config host; do
  cp -a "${component}/${directory}" "${destination}/"
done
install -m 0755 "${component}/host/run-core" "${destination}/host/run-core"
unit_source="${component}/host"
fi
# Only the validated group token is substituted; no unit/shell syntax is accepted.
install -m 0644 <(sed "s/^SupplementaryGroups=rosie-rt rosie-ctl ethercat$/SupplementaryGroups=rosie-rt rosie-ctl ${device_group}/" \
  "${unit_source}/rosie-rt-core.service") "${root}/etc/systemd/system/rosie-rt-core.service"
install -m 0644 "${unit_source}/rosie-rt-control.service" "${root}/etc/systemd/system/rosie-rt-control.service"
install -m 0644 "${unit_source}/rosie-rt-natspublisher.service" "${root}/etc/systemd/system/rosie-rt-natspublisher.service"
install -m 0644 "${unit_source}/rosie-rt-crl.service" "${unit_source}/rosie-rt-crl.timer" "${root}/etc/systemd/system/"
if [[ -n "${publisher_environment}" ]]; then
  pending_publisher="$(mktemp "${root}/etc/rosie-rt-core/.natspublisher.env.XXXXXX")"
  trap 'rm -f -- "${pending_publisher}"' EXIT
  # unverified permission policy: systemd reads deployment values as root.
  install -m 0600 -o root -g root /dev/null "${pending_publisher}"
  printf '%s\n' "${publisher_environment}" > "${pending_publisher}"
  mv -f -- "${pending_publisher}" "${root}/etc/rosie-rt-core/natspublisher.env"
  trap - EXIT
  printf '%s\n' 'install component=rt-natspublisher warning="Two publishers on one subject are a fault. Stop the v4 bridge on the RevPi before enabling this Jetson publisher; Conflicts= cannot police another host."' >&2
fi
if [[ -n "${from_package}" ]]; then
  if [[ -n "${current}" && "${current}" != "${revision}" ]]; then
    select_release previous "${current}"
    rm -f -- "${store}/.rollback-consumed"
  fi
  select_release current "${revision}"
fi
if [[ -n "${anchor_source}" ]]; then
  printf 'install component=anchors status=kept reason=existing-configuration source="%s" changed=none\n' "${anchor_source}"
else
  anchor_state=created
  if [[ -d "${root}${anchor_dir}" ]]; then
    anchor_state=existing
    layout="$(stat -c '%U:%G %a' -- "${root}${anchor_dir}")"
    # The test root cannot own files as the service accounts; only live hosts compare owners.
    if [[ -z "${root}" && "${layout}" != 'rosie-ctl:rosie-rt 2750' ]]; then
      printf 'install component=anchors warning="%s is %s, not rosie-ctl:rosie-rt 2750; the installer never changes an existing anchor directory"\n' "${anchor_dir}" "${layout}" >&2
    fi
  else
    install -d -m 2750 -o rosie-ctl -g rosie-rt "${root}${anchor_dir}"
  fi
  for unit in "${anchor_units[@]}"; do
    [[ -d "${root}/etc/systemd/system/${unit}.service.d" ]] || install -d -m 0755 "${root}/etc/systemd/system/${unit}.service.d"
    install -m 0644 <(printf '%s\n' '# Installed by rt-core host/install.sh; a later-sorted drop-in overrides it.' \
      '[Service]' "Environment=ROSIE_RT_ANCHOR_DIR=${anchor_dir}") \
      "${root}/etc/systemd/system/${unit}.service.d/${anchor_dropin}"
  done
  printf 'install component=anchors status=provisioned dir=%s directory=%s dropin=%s\n' "${anchor_dir}" "${anchor_state}" "${anchor_dropin}"
fi
systemctl daemon-reload
if [[ ${enable} == 1 ]]; then
  systemctl enable rosie-rt-core.service rosie-rt-control.service
fi
printf '%s\n' 'Installed. No units were started.'
