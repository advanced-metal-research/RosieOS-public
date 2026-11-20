#!/usr/bin/env bash
# Publish a compiler-bound deployment atomically; failed compilation cannot replace control.env.
set -euo pipefail
export LC_ALL=C
# unverified staging policy: unpublished configuration is private to the installer.
umask 077
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH="${PATH}:${component}/bin:${component}/build"
fail() { printf 'generate-control-env error=%s\n' "$*" >&2; exit 1; }
remote_pki=""
keep_remote=0
if [[ "${1:-}" == --remote-pki ]]; then
  [[ $# -ge 2 ]] || fail '--remote-pki requires a directory'
  remote_pki="$(realpath -e -- "$2")"
  shift 2
elif [[ "${1:-}" == --keep-remote-args ]]; then
  # A host whose listener PKI is already installed keeps its listener line,
  # including reviewed jog-clock policy, instead of having it switched off.
  keep_remote=1
  shift
fi
[[ $# == 3 ]] || fail 'usage: generate-control-env.sh [--remote-pki directory | --keep-remote-args] machine.json pair-id pair-revision'
if [[ -n "${remote_pki}" ]]; then
  [[ "$(cat "${remote_pki}/pair-id")" == "$2" ]] || fail 'remote PKI pair does not match deployment'
  for file in ca.pem ca-key.pem ca.cnf pair-id serial crlnumber server.pem server-key.pem crl.pem; do
    [[ -r "${remote_pki}/${file}" && -s "${remote_pki}/${file}" ]] || fail "missing remote PKI ${file}"
  done
  [[ -r "${remote_pki}/index.txt" ]] || fail 'missing remote PKI index.txt'
  exec 9>"${remote_pki}/.lock"
  flock -n 9 || fail 'remote PKI operation already in progress'
  # unverified fail-closed listener default: loopback port 8443; explicitly override to expose a host interface.
  remote_listen="${ROSIE_RT_REMOTE_LISTEN:-127.0.0.1:8443}"
  [[ "${remote_listen}" =~ ^([A-Za-z0-9._-]+|\[[A-Fa-f0-9:]+\]):[0-9]+$ ]] || fail 'invalid ROSIE_RT_REMOTE_LISTEN'
  # rt-control admits remote jog only under a jog-clock policy; without one it
  # refuses every remote jog (cause="jog_clock_unqualified"), so a listener
  # provisioned here once could not jog until its control.env was edited by
  # hand. The policy comes from the environment, all three values or none: a
  # partial set would publish a listener rt-control silently leaves
  # unqualified. Bounds are rt-control's: the two ns values are positive
  # uint64 flags (zero disables qualification), drift is positive and below
  # 1e9 ppb (jog_clock.go). unverified policy: no value is chosen here; the
  # caller supplies the reviewed one (the Rosie 1430 cells run 100000000 /
  # 2000000 / 10000000000, not measured on this host).
  positive_u64() { [[ "$1" =~ ^[1-9][0-9]{0,19}$ ]] && { [[ ${#1} -lt 20 ]] || [[ "$1" < 18446744073709551616 ]]; }; }
  jog_given=0
  for name in ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS ROSIE_RT_REMOTE_JOG_DRIFT_PPB ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS; do
    if [[ -n "${!name:-}" ]]; then jog_given=$((jog_given + 1)); fi
  done
  remote_jog=""
  if [[ ${jog_given} == 3 ]]; then
    positive_u64 "${ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS}" || fail 'invalid ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS: a positive integer up to 18446744073709551615'
    [[ "${ROSIE_RT_REMOTE_JOG_DRIFT_PPB}" =~ ^[1-9][0-9]{0,8}$ ]] || fail 'invalid ROSIE_RT_REMOTE_JOG_DRIFT_PPB: a positive integer below 1000000000'
    positive_u64 "${ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS}" || fail 'invalid ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS: a positive integer up to 18446744073709551615'
    remote_jog=" --remote-jog-max-uncertainty-ns ${ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS} --remote-jog-drift-ppb ${ROSIE_RT_REMOTE_JOG_DRIFT_PPB} --remote-jog-calibration-max-age-ns ${ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS}"
  elif [[ ${jog_given} != 0 ]]; then
    fail 'remote jog-clock policy needs all of ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS, ROSIE_RT_REMOTE_JOG_DRIFT_PPB and ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS, or none'
  fi
fi
root="${ROSIE_RT_INSTALL_TEST_ROOT:-}"
if [[ -n "${root}" ]]; then
  root="$(realpath -e -- "${root}")"
  [[ -n "${TMPDIR:-}" && "${root}" == "$(realpath -e -- "${TMPDIR}")/rosie-rt-install-test."* ]] || fail 'test root must be a dedicated TMPDIR/rosie-rt-install-test.* directory'
else
  [[ ${EUID} == 0 ]] || fail 'run configuration generation as root'
fi
configuration="${root}/etc/rosie-rt-core"
kept_remote=""
if [[ ${keep_remote} == 1 ]]; then
  [[ -r "${configuration}/control.env" ]] || fail '--keep-remote-args requires an installed control.env'
  [[ "$(grep -c '^ROSIE_RT_REMOTE_ARGS=' "${configuration}/control.env")" == 1 ]] || fail 'installed control.env must carry exactly one ROSIE_RT_REMOTE_ARGS line'
  kept_remote="$(grep '^ROSIE_RT_REMOTE_ARGS=' "${configuration}/control.env")"
  [[ "${kept_remote}" == 'ROSIE_RT_REMOTE_ARGS="--remote-listen '* ]] || fail 'installed control.env has no remote listener to keep'
  for file in ca.pem server.pem server-key.pem crl.pem; do
    [[ -s "${configuration}/pki/${file}" ]] || fail "missing installed remote PKI ${file}"
  done
fi
# Both native and adapter read the compiled identity/resources. The native user
# is a member of rosie-ctl; application clients are not.
install -d -m 0755 -o root -g root "${configuration}"
install -d -m 2750 -o root -g rosie-ctl "${configuration}/compiled"
staging="$(mktemp -d "${configuration}/compiled/.staging.XXXXXX")"
pending=""
trap 'rm -rf -- "${staging}"; if [[ -n "${pending}" ]]; then rm -f -- "${pending}"; fi' EXIT

# The daemon and adapter consume the same validated deployment binding.
if ! rtctl control-env --config "$1" --pair-id "$2" --pair-revision "$3" \
    --out "${staging}" --installed-out "${configuration}/compiled" > "${staging}/control.env"; then
  fail 'configuration compilation failed; control.env was not replaced'
fi
if [[ -n "${remote_pki}" ]]; then
  # unverified credential policy: adapter material is group-readable; CA state and issued client keys are root-only.
  install -d -m 0750 -o root -g rosie-ctl "${configuration}/pki"
  for file in ca.pem server.pem server-key.pem crl.pem; do
    install -m 0640 -o root -g rosie-ctl "${remote_pki}/${file}" "${configuration}/pki/${file}"
  done
  for file in ca-key.pem ca.cnf pair-id index.txt serial crlnumber; do
    install -m 0600 -o root -g root "${remote_pki}/${file}" "${configuration}/pki/${file}"
  done
  for directory in clients newcerts; do
    install -d -m 0700 -o root -g root "${configuration}/pki/${directory}"
    for file in "${remote_pki}/${directory}/"*.pem; do
      [[ -e "${file}" ]] || continue
      install -m 0600 -o root -g root "${file}" "${configuration}/pki/${directory}/${file##*/}"
    done
  done
  # Without a policy the line stays exactly as before (installed hosts and their
  # tests compare it), and the operator is told the listener cannot jog.
  if [[ -z "${remote_jog}" ]]; then
    printf 'generate-control-env warning=remote listener has no jog-clock policy; remote jog stays unqualified (set ROSIE_RT_REMOTE_JOG_MAX_UNCERTAINTY_NS, ROSIE_RT_REMOTE_JOG_DRIFT_PPB, ROSIE_RT_REMOTE_JOG_CALIBRATION_MAX_AGE_NS)\n' >&2
  fi
  printf 'ROSIE_RT_REMOTE_ARGS="--remote-listen %s --remote-ca /etc/rosie-rt-core/pki/ca.pem --remote-cert /etc/rosie-rt-core/pki/server.pem --remote-key /etc/rosie-rt-core/pki/server-key.pem --remote-crl /etc/rosie-rt-core/pki/crl.pem%s"\n' "${remote_listen}" "${remote_jog}" >> "${staging}/control.env"
elif [[ -n "${kept_remote}" ]]; then
  printf '%s\n' "${kept_remote}" >> "${staging}/control.env"
else
  printf 'ROSIE_RT_REMOTE_ARGS=""\n' >> "${staging}/control.env"
fi
digest="$(sed -n 's/^ROSIE_RT_CONFIGURATION_SHA256=//p' "${staging}/control.env")"
[[ "${digest}" =~ ^[0-9a-f]{64}$ ]] || fail 'compiler did not return a SHA-256 configuration identity'
# Publish the complete compiler snapshot, including description blobs and the
# identities checked by rt-control. Publishing only axes.conf/argv.json leaves
# native robot deployments without the model resources required by the adapter.
install -d -m 2750 -o root -g rosie-ctl "${configuration}/compiled/${digest}"
while IFS= read -r -d '' file; do
  relative="${file#"${staging}/${digest}/"}"
  target="${configuration}/compiled/${digest}/${relative}"
  install -d -m 2750 -o root -g rosie-ctl "$(dirname "${target}")"
  install -m 0640 -o root -g rosie-ctl "${file}" "${target}"
done < <(find "${staging}/${digest}" -type f -print0)
pending="$(mktemp "${configuration}/.control.env.XXXXXX")"
# unverified permission policy: pairing metadata is never world-readable.
install -m 0640 -o root -g rosie-ctl "${staging}/control.env" "${pending}"
mv -f -- "${pending}" "${configuration}/control.env"
pending=""
printf 'configuration_sha256=%s\n' "${digest}"
