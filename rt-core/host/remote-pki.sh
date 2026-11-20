#!/usr/bin/env bash
# Own a pair-specific CA and CRL; refusing overwrite prevents accidental identity replacement.
set -euo pipefail
export LC_ALL=C
# unverified provisioning policy: CA state and issued client keys remain administrator-only.
umask 077
fail() { printf 'remote-pki error=%s\n' "$*" >&2; exit 1; }
# crl re-signs the CRL with no change to it: rt-control refuses every client
# once the CRL's nextUpdate passes (remote_tls.go, "remote CRL not current"),
# and a CRL lasts default_crl_days, so rosie-rt-crl.timer runs it daily
# (two Rosie 1430 cells lapsed on 2026-10-19, a week after their last issue).
[[ $# == 3 || ( $# == 2 && "${1:-}" == crl ) ]] || fail 'usage: remote-pki.sh init|issue-client|revoke directory identity, or remote-pki.sh crl directory'
action="$1" directory="$2" identity="${3:-}"
[[ "${action}" == crl || ( "${identity}" =~ ^[A-Za-z0-9_.:-]+$ && "${identity}" != . && "${identity}" != .. ) ]] || fail 'invalid identity token'
test_root="${ROSIE_RT_PKI_TEST_ROOT:-}"
if [[ -n "${test_root}" ]]; then
  test_root="$(realpath -e -- "${test_root}")"
  component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  [[ -n "${TMPDIR:-}" && ( "${test_root}" == "$(realpath -e -- "${TMPDIR}")/rosie-rt-pki-test."* || "${test_root}" == "$(realpath -e -- "${TMPDIR}")/rosie-rt-install-test."* ) ]] || fail 'test root must be a dedicated PKI/install directory under TMPDIR'
  [[ "$(realpath -m -- "${directory}")" == "${test_root}/"* ]] || fail 'PKI directory escapes test root'
else
  [[ ${EUID} == 0 ]] || fail 'run provisioning as root'
  getent group rosie-ctl >/dev/null || fail 'rosie-ctl group must exist'
fi
publish() {
  local source="$1" target="$2" mode="$3"
  if [[ -n "${test_root}" ]]; then install -m "${mode}" "${source}" "${target}"
  else install -m "${mode}" -o root -g rosie-ctl "${source}" "${target}"; fi
}
if [[ "${action}" == init ]]; then
  [[ ! -e "${directory}" ]] || fail 'PKI directory already exists'
  mkdir -m 0750 -p -- "${directory}"
  directory="$(realpath -e -- "${directory}")"
  [[ "${directory}" != *$'\n'* && "${directory}" != *'"'* && "${directory}" != *'$'* && "${directory}" != *'\\'* ]] || fail 'unsafe PKI directory'
  [[ -n "${test_root}" ]] || chown root:rosie-ctl "${directory}"
  cd -- "${directory}"
  exec 9>"${directory}/.lock"
  flock -n 9 || fail 'PKI operation already in progress'
  mkdir -m 0700 "${directory}/newcerts" "${directory}/clients"
  printf '%s\n' "${identity}" > "${directory}/pair-id"
  : > "${directory}/index.txt"
  # unverified CA serial seed: monotonic OpenSSL database serials begin at hexadecimal 1000.
  printf '1000\n' > "${directory}/serial"
  printf '1000\n' > "${directory}/crlnumber"
  cat > "${directory}/ca.cnf" <<EOF
[ca]
default_ca = owned
[owned]
dir = .
database = \$dir/index.txt
new_certs_dir = \$dir/newcerts
certificate = \$dir/ca.pem
private_key = \$dir/ca-key.pem
serial = \$dir/serial
crlnumber = \$dir/crlnumber
default_md = sha256
default_days = 365
default_crl_days = 7
policy = identity
unique_subject = no
[identity]
commonName = supplied
EOF
  # unverified certificate lifetime policy: CA 3,650 days; leaves 365 days; CRL seven days.
  openssl req -new -x509 -newkey ec -pkeyopt ec_paramgen_curve:P-256 -nodes \
    -keyout "${directory}/ca-key.pem" -out "${directory}/ca.pem" -days 3650 \
    -subj "/CN=Rosie remote CA ${identity}" -addext 'basicConstraints=critical,CA:TRUE' \
    -addext 'keyUsage=critical,keyCertSign,cRLSign'
  openssl req -new -newkey ec -pkeyopt ec_paramgen_curve:P-256 -nodes \
    -keyout "${directory}/server-key.pending" -out "${directory}/server.csr" -subj "/CN=Rosie remote ${identity}"
  # unverified server-name default: loopback; deployment DNS/IP names come from ROSIE_RT_REMOTE_SERVER_SAN.
  server_san="${ROSIE_RT_REMOTE_SERVER_SAN:-DNS:localhost,IP:127.0.0.1,IP:::1}"
  [[ "${server_san}" =~ ^(DNS:[A-Za-z0-9._-]+|IP:[A-Fa-f0-9:.]+)(,(DNS:[A-Za-z0-9._-]+|IP:[A-Fa-f0-9:.]+))*$ ]] || fail 'invalid ROSIE_RT_REMOTE_SERVER_SAN'
  printf 'basicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature\nextendedKeyUsage=serverAuth\nsubjectAltName=%s\n' "${server_san}" > "${directory}/server.ext"
  openssl ca -batch -config "${directory}/ca.cnf" -notext -in "${directory}/server.csr" -out "${directory}/server.pending" -extfile "${directory}/server.ext"
  # unverified adapter-read policy: keys and certificates are root:rosie-ctl mode 0640; CA key is 0600.
  publish "${directory}/server-key.pending" "${directory}/server-key.pem" 0640
  publish "${directory}/server.pending" "${directory}/server.pem" 0640
  chmod 0600 "${directory}/ca-key.pem"
  chmod 0640 "${directory}/ca.pem"
  [[ -n "${test_root}" ]] || chown root:rosie-ctl "${directory}/ca.pem"
  rm -- "${directory}/server-key.pending" "${directory}/server.pending" "${directory}/server.csr" "${directory}/server.ext"
elif [[ "${action}" == issue-client || "${action}" == revoke || "${action}" == crl ]]; then
  directory="$(realpath -e -- "${directory}")"
  [[ -f "${directory}/ca.cnf" ]] || fail 'uninitialized PKI'
  cd -- "${directory}"
  exec 9>"${directory}/.lock"
  flock -n 9 || fail 'PKI operation already in progress'
  pair="$(cat "${directory}/pair-id")"
  [[ "${pair}" =~ ^[A-Za-z0-9_.:-]+$ ]] || fail 'invalid stored pair identity'
  if [[ "${action}" == crl ]]; then
    : # only the CRL below, re-signed with a new nextUpdate
  elif [[ "${action}" == issue-client ]]; then
    [[ ! -e "${directory}/clients/${identity}.pem" ]] || fail 'principal already issued'
    openssl req -new -newkey ec -pkeyopt ec_paramgen_curve:P-256 -nodes \
      -keyout "${directory}/clients/${identity}-key.pem" -out "${directory}/clients/${identity}.csr" -subj "/CN=${identity}"
    printf 'basicConstraints=critical,CA:FALSE\nkeyUsage=critical,digitalSignature\nextendedKeyUsage=clientAuth\nsubjectAltName=URI:rosie-principal:%s,URI:rosie-pair:%s\n' "${identity}" "${pair}" > "${directory}/clients/${identity}.ext"
    openssl ca -batch -config "${directory}/ca.cnf" -notext -in "${directory}/clients/${identity}.csr" -out "${directory}/clients/${identity}.pem" -extfile "${directory}/clients/${identity}.ext"
    rm -- "${directory}/clients/${identity}.csr" "${directory}/clients/${identity}.ext"
  else
    openssl ca -batch -config "${directory}/ca.cnf" -revoke "${directory}/clients/${identity}.pem"
  fi
else fail 'unknown subcommand'; fi
openssl ca -batch -config "${directory}/ca.cnf" -gencrl -out "${directory}/crl.pending"
publish "${directory}/crl.pending" "${directory}/crl.new" 0640
mv -f -- "${directory}/crl.new" "${directory}/crl.pem"
rm -- "${directory}/crl.pending"
