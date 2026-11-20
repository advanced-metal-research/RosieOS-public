#!/usr/bin/env bash
# A confined mock root exercises real OpenSSL signatures, modes and CRL enforcement.
set -euo pipefail
export LC_ALL=C
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
scratch="$(mktemp -d "${TMPDIR:?}/rosie-rt-pki-test.XXXXXX")"
install_root=""
trap 'rm -rf -- "${scratch}"; if [[ -n "${install_root}" ]]; then rm -rf -- "${install_root}"; fi' EXIT
export ROSIE_RT_PKI_TEST_ROOT="${scratch}"
pki="${scratch}/pki"
for script in remote-pki.sh remote-pki-test.sh install.sh generate-control-env.sh; do bash -n "${component}/host/${script}"; done
bash "${component}/host/remote-pki.sh" init "${pki}" bench-pair
bash "${component}/host/remote-pki.sh" issue-client "${pki}" pendant
for file in ca.pem server.pem server-key.pem crl.pem; do
  [[ -s "${pki}/${file}" && "$(stat -c %a "${pki}/${file}")" == 640 ]]
done
[[ "$(stat -c %a "${pki}/ca-key.pem")" == 600 ]]
[[ "$(stat -c %a "${pki}/clients/pendant-key.pem")" == 600 ]]
openssl x509 -in "${pki}/clients/pendant.pem" -noout -ext subjectAltName | grep -F 'URI:rosie-principal:pendant, URI:rosie-pair:bench-pair'
openssl verify -purpose sslclient -CAfile "${pki}/ca.pem" "${pki}/clients/pendant.pem"
openssl verify -crl_check -CRLfile "${pki}/crl.pem" -CAfile "${pki}/ca.pem" "${pki}/clients/pendant.pem"
serial="$(openssl x509 -in "${pki}/clients/pendant.pem" -noout -serial)"
bash "${component}/host/remote-pki.sh" revoke "${pki}" pendant
openssl crl -in "${pki}/crl.pem" -noout -text | grep -F "Serial Number: ${serial#serial=}"
if openssl verify -crl_check -CRLfile "${pki}/crl.pem" -CAfile "${pki}/ca.pem" "${pki}/clients/pendant.pem" >"${scratch}/revoked.log" 2>&1; then
  printf 'revoked principal accepted\n' >&2; exit 1
fi
grep -F 'certificate revoked' "${scratch}/revoked.log"
# crl re-signs the same list: a new number and lastUpdate, the revoked
# principal still on it, the CA database unchanged.
before_number="$(openssl crl -in "${pki}/crl.pem" -noout -crlnumber)"
before_index="$(sha256sum "${pki}/index.txt")"
sleep 1
bash "${component}/host/remote-pki.sh" crl "${pki}"
[[ "$(openssl crl -in "${pki}/crl.pem" -noout -crlnumber)" != "${before_number}" ]]
[[ "$(sha256sum "${pki}/index.txt")" == "${before_index}" ]]
[[ "$(stat -c %a "${pki}/crl.pem")" == 640 ]]
openssl crl -in "${pki}/crl.pem" -noout -text | grep -F "Serial Number: ${serial#serial=}"
openssl crl -in "${pki}/crl.pem" -CAfile "${pki}/ca.pem" -noout
if bash "${component}/host/remote-pki.sh" crl "${scratch}/missing" 2>/dev/null; then
  printf 'crl accepted an uninitialized directory\n' >&2; exit 1
fi
install_root="$(mktemp -d "${TMPDIR}/rosie-rt-install-test.XXXXXX")"
mkdir "${scratch}/bin"
cat > "${scratch}/bin/install" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' "$*" >> "${ROSIE_RT_PKI_TEST_ROOT}/install.log"
args=()
while [[ $# -gt 0 ]]; do
  case "$1" in -o|-g) shift 2 ;; *) args+=("$1"); shift ;; esac
done
[[ "${args[-1]}" == "${ROSIE_RT_INSTALL_TEST_ROOT}/"* ]] || exit 2
exec /usr/bin/install "${args[@]}"
SH
chmod +x "${scratch}/bin/install"
# Unverified installation fixture: a 10:1 software ratio with the standard
# 131072 motor counts/rev and drive-side 0x6091 factor 1/1.
sed -e 's/"backend": "simulation"/"backend": "live"/' \
  -e 's/"wire_revs_per_axis_rev": 1/"wire_revs_per_axis_rev": 10/' \
  -e 's/"profile": "cia402_sim"/"profile": "a6ec"/' \
  "${component}/config/machines/simulation/simulation-program.json" > "${scratch}/machine.json"
export ROSIE_RT_INSTALL_TEST_ROOT="${install_root}" PATH="${scratch}/bin:${PATH}" ROSIE_RT_REMOTE_LISTEN=127.0.0.1:8443
bash "${component}/host/generate-control-env.sh" --remote-pki "${pki}" "${scratch}/machine.json" bench-pair 1
for file in ca.pem server.pem server-key.pem crl.pem; do
  [[ "$(stat -c %a "${install_root}/etc/rosie-rt-core/pki/${file}")" == 640 ]]
done
grep -F -- '-m 0640 -o root -g rosie-ctl' "${scratch}/install.log" >/dev/null
grep -F 'ROSIE_RT_REMOTE_ARGS="--remote-listen 127.0.0.1:8443 --remote-ca /etc/rosie-rt-core/pki/ca.pem --remote-cert /etc/rosie-rt-core/pki/server.pem --remote-key /etc/rosie-rt-core/pki/server-key.pem --remote-crl /etc/rosie-rt-core/pki/crl.pem"' "${install_root}/etc/rosie-rt-core/control.env"
[[ "$(stat -c %a "${install_root}/etc/rosie-rt-core/pki/ca-key.pem")" == 600 ]]
[[ "$(stat -c %a "${install_root}/etc/rosie-rt-core/pki/clients/pendant-key.pem")" == 600 ]]
export ROSIE_RT_PKI_TEST_ROOT="${install_root}"
# The relocated CA must issue and revoke against its installed database, not its source directory.
bash "${component}/host/remote-pki.sh" issue-client "${install_root}/etc/rosie-rt-core/pki" replacement
bash "${component}/host/remote-pki.sh" revoke "${install_root}/etc/rosie-rt-core/pki" replacement
[[ ! -e "${pki}/clients/replacement.pem" ]]
printf 'remote-pki-test: PASS\n'
