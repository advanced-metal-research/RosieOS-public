// Paired PKI migration keeps controller and OLP identities in one release.
package main

import (
	"strconv"
	"strings"
)

func releasePairPKIBackup(from, to string) string {
	return "/etc/rosie-rt-core/pki-history/" + from + "-to-" + to
}

func releasePairPKIServoPreflight(c robotV4RTCoreConfig, from string) string {
	stage := "/etc/rosie-rt-core/pki-staged/" + c.PairID
	backup := releasePairPKIBackup(from, c.PairID)
	return strings.Join([]string{
		"STAGED_PKI=" + shellQuote(stage),
		"PAIR_BACKUP=" + shellQuote(backup),
		`test "$(sudo -n sed -n 's/^ROSIE_RT_PAIR_ID=//p' /etc/rosie-rt-core/control.env)" = ` + shellQuote(from),
		`test "$(sudo -n sed -n 's/^ROSIE_RT_PAIR_REVISION=//p' /etc/rosie-rt-core/control.env)" = ` + strconv.FormatUint(c.PairRevision, 10),
		`test "$(sudo -n cat /etc/rosie-rt-core/pki/pair-id)" = ` + shellQuote(from),
		`test "$(sudo -n cat "$STAGED_PKI/pair-id")" = ` + shellQuote(c.PairID),
		`for f in ca.pem ca-key.pem ca.cnf pair-id serial crlnumber index.txt server.pem server-key.pem crl.pem; do sudo -n test -s "$STAGED_PKI/$f" || exit 1; done`,
		`sudo -n openssl x509 -in "$STAGED_PKI/ca.pem" -checkend 86400 -noout`,
		`sudo -n openssl x509 -in "$STAGED_PKI/server.pem" -checkend 86400 -noout`,
		`sudo -n openssl verify -CAfile "$STAGED_PKI/ca.pem" -CRLfile "$STAGED_PKI/crl.pem" -crl_check "$STAGED_PKI/server.pem"`,
		`sudo -n openssl x509 -in "$STAGED_PKI/server.pem" -noout -ext subjectAltName | grep -Fq 'IP Address:127.0.0.1'`,
		`server_pub=$(sudo -n openssl x509 -in "$STAGED_PKI/server.pem" -pubkey -noout | sha256sum | cut -d' ' -f1)`,
		`key_pub=$(sudo -n openssl pkey -in "$STAGED_PKI/server-key.pem" -pubout | sha256sum | cut -d' ' -f1)`,
		`test "$server_pub" = "$key_pub"`,
	}, "\n")
}

func releasePairPKIRelayPreflight(c robotV4RTCoreConfig, ssh string) string {
	remoteCA := "/etc/rosie-rt-core/pki-staged/" + c.PairID + "/ca.pem"
	return strings.Join([]string{
		`CLIENT_STAGE="$HOME/.rosie/pki-staged/` + c.PairID + `"`,
		`CA="$CLIENT_STAGE/ca.pem"`,
		`set -- "$CLIENT_STAGE"/clients/*.pem; test "$#" = 2 || { echo 'pair migration refused: expected one client certificate and key' >&2; exit 1; }`,
		`CERT=""; KEY=""; for f in "$@"; do case "$f" in *-key.pem) KEY="$f" ;; *) CERT="$f" ;; esac; done`,
		`for f in "$CA" "$CERT" "$KEY"; do test -s "$f" && test ! -L "$f" || exit 1; done`,
		`openssl x509 -in "$CERT" -checkend 86400 -noout`,
		`openssl verify -CAfile "$CA" "$CERT"`,
		`openssl x509 -in "$CERT" -noout -ext subjectAltName | grep -Fq ` + shellQuote("URI:rosie-pair:"+c.PairID),
		`client_pub=$(openssl x509 -in "$CERT" -pubkey -noout | sha256sum | cut -d' ' -f1)`,
		`key_pub=$(openssl pkey -in "$KEY" -pubout | sha256sum | cut -d' ' -f1)`,
		`test "$client_pub" = "$key_pub"`,
		`local_ca=$(sha256sum "$CA" | cut -d' ' -f1)`,
		`remote_ca=$(` + ssh + shellQuote("sudo -n sha256sum "+shellQuote(remoteCA)+" | cut -d' ' -f1") + `)`,
		`test "$local_ca" = "$remote_ca" || { echo 'pair migration refused: staged CA differs between hosts' >&2; exit 1; }`,
	}, "\n")
}

// releasePairPKIRetryScript retains a completed rollback before a new cutover.
// It refuses a partial or mismatched rollback instead of replacing its keys.
func releasePairPKIRetryScript(from, to string) string {
	return strings.Join([]string{
		`for entry in "$PAIR_BACKUP" "$PAIR_BACKUP.live-before-switch" "$PAIR_BACKUP.failed-active"; do`,
		`  sudo -n test ! -L "$entry" || { echo 'pair migration refused: PKI history contains a symlink' >&2; exit 1; }`,
		`done`,
		`if sudo -n test -e "$PAIR_BACKUP" || sudo -n test -e "$PAIR_BACKUP.live-before-switch" || sudo -n test -e "$PAIR_BACKUP.failed-active"; then`,
		`  sudo -n test -d "$PAIR_BACKUP" || { echo 'pair migration refused: incomplete previous PKI backup' >&2; exit 1; }`,
		`  test "$(sudo -n cat "$PAIR_BACKUP/pki/pair-id" 2>/dev/null)" = ` + shellQuote(from) + ` || { echo 'pair migration refused: previous PKI backup has the wrong pair' >&2; exit 1; }`,
		`  sudo -n cmp -s /etc/rosie-rt-core/control.env "$PAIR_BACKUP/control.env" || { echo 'pair migration refused: live control differs from the previous backup' >&2; exit 1; }`,
		`  if ! sudo -n diff -qr /etc/rosie-rt-core/pki "$PAIR_BACKUP/pki" >/dev/null; then`,
		`    for dir in /etc/rosie-rt-core/pki "$PAIR_BACKUP/pki"; do`,
		`      test -z "$(sudo -n find "$dir" -mindepth 2 \( -name crl.pem -o -name crlnumber -o -name crlnumber.old \) -print)" || { echo 'pair migration refused: nested CRL state' >&2; exit 1; }`,
		`      for f in crl.pem crlnumber; do sudo -n test -f "$dir/$f" && sudo -n test ! -L "$dir/$f" || { echo 'pair migration refused: missing or linked CRL state' >&2; exit 1; }; done`,
		`      sudo -n test ! -L "$dir/crlnumber.old" || { echo 'pair migration refused: linked CRL state' >&2; exit 1; }`,
		`      sudo -n openssl crl -in "$dir/crl.pem" -CAfile "$dir/ca.pem" -verify -noout >/dev/null 2>&1 || { echo 'pair migration refused: invalid CRL signature' >&2; exit 1; }`,
		`    done`,
		`    sudo -n diff -qr --exclude=crl.pem --exclude=crlnumber --exclude=crlnumber.old /etc/rosie-rt-core/pki "$PAIR_BACKUP/pki" >/dev/null || { echo 'pair migration refused: live PKI differs from the previous backup' >&2; exit 1; }`,
		`    sudo -n openssl verify -CAfile /etc/rosie-rt-core/pki/ca.pem -CRLfile /etc/rosie-rt-core/pki/crl.pem -crl_check /etc/rosie-rt-core/pki/server.pem >/dev/null 2>&1 || { echo 'pair migration refused: live CRL does not validate the server' >&2; exit 1; }`,
		`    old_crl=$(sudo -n openssl crl -in "$PAIR_BACKUP/pki/crl.pem" -noout -crlnumber | sed 's/^crlNumber=//')`,
		`    live_crl=$(sudo -n openssl crl -in /etc/rosie-rt-core/pki/crl.pem -noout -crlnumber | sed 's/^crlNumber=//')`,
		`    [[ "$old_crl" =~ ^0x[0-9A-Fa-f]{1,12}$ && "$live_crl" =~ ^0x[0-9A-Fa-f]{1,12}$ ]] && (( live_crl > old_crl )) || { echo 'pair migration refused: CRL number did not advance' >&2; exit 1; }`,
		`  fi`,
		`  if sudo -n test -e "$PAIR_BACKUP.live-before-switch"; then`,
		`    sudo -n test -d "$PAIR_BACKUP.live-before-switch" && test "$(sudo -n cat "$PAIR_BACKUP.live-before-switch/pair-id")" = ` + shellQuote(from) + ` || { echo 'pair migration refused: previous live PKI has the wrong pair' >&2; exit 1; }`,
		`  fi`,
		`  if sudo -n test -e "$PAIR_BACKUP.failed-active"; then`,
		`    sudo -n test -d "$PAIR_BACKUP.failed-active" && test "$(sudo -n cat "$PAIR_BACKUP.failed-active/pair-id")" = ` + shellQuote(to) + ` || { echo 'pair migration refused: failed PKI has the wrong pair' >&2; exit 1; }`,
		`  fi`,
		`  archive=$(sudo -n mktemp -d "$PAIR_BACKUP.retry.XXXXXXXX") || exit 1`,
		`  if sudo -n test -e "$PAIR_BACKUP.live-before-switch"; then sudo -n mv "$PAIR_BACKUP.live-before-switch" "$archive/live-before-switch"; fi`,
		`  if sudo -n test -e "$PAIR_BACKUP.failed-active"; then sudo -n mv "$PAIR_BACKUP.failed-active" "$archive/failed-active"; fi`,
		`  sudo -n mv "$PAIR_BACKUP" "$archive/backup"`,
		`  printf 'previous completed PKI rollback retained in %s\n' "$archive"`,
		`fi`,
	}, "\n")
}

func releasePairPKIBackupScript(from, to string) string {
	return strings.Join([]string{
		`sudo -n install -d -m 0700 -o root -g root /etc/rosie-rt-core/pki-history`,
		releasePairPKIRetryScript(from, to),
		`sudo -n install -d -m 0700 -o root -g root "$PAIR_BACKUP"`,
		`sudo -n flock -n /etc/rosie-rt-core/pki/.lock cp -a /etc/rosie-rt-core/pki "$PAIR_BACKUP/pki"`,
		`sudo -n install -m 0640 -o root -g rosie-ctl /etc/rosie-rt-core/control.env "$PAIR_BACKUP/control.env"`,
		`test "$(sudo -n cat "$PAIR_BACKUP/pki/pair-id")" = "$(sudo -n cat /etc/rosie-rt-core/pki/pair-id)"`,
	}, "\n")
}

func releasePairPKIControlEnvCommand(c robotV4RTCoreConfig) string {
	machine := "/opt/rosie-rt-core/current/" + strings.TrimPrefix(c.MachineConfig, "rt-core/")
	return strings.Join([]string{
		`sudo -n mv /etc/rosie-rt-core/pki "$PAIR_BACKUP.live-before-switch"`,
		`sudo -n env PATH=/opt/rosie-rt-core/current/bin:/usr/sbin:/usr/bin:/sbin:/bin ROSIE_RT_REMOTE_LISTEN=` + shellQuote(c.RemoteListen) + ` bash /opt/rosie-rt-core/current/host/generate-control-env.sh --remote-pki "$STAGED_PKI" ` + shellQuote(machine) + " " + shellQuote(c.PairID) + " " + strconv.FormatUint(c.PairRevision, 10),
		`test "$(sudo -n cat /etc/rosie-rt-core/pki/pair-id)" = ` + shellQuote(c.PairID),
	}, "\n")
}

func releasePairPKIRestoreScript(from, to string) string {
	backup := releasePairPKIBackup(from, to)
	return strings.Join([]string{
		"PAIR_BACKUP=" + shellQuote(backup),
		`test "$(sudo -n cat "$PAIR_BACKUP/pki/pair-id")" = ` + shellQuote(from) + ` || { echo 'rollback refused: previous pair PKI backup missing or mismatched' >&2; exit 1; }`,
		`if sudo -n test -e /etc/rosie-rt-core/pki; then sudo -n test ! -e "$PAIR_BACKUP.failed-active" && sudo -n mv /etc/rosie-rt-core/pki "$PAIR_BACKUP.failed-active"; fi`,
		`sudo -n cp -a "$PAIR_BACKUP/pki" /etc/rosie-rt-core/pki`,
		`test "$(sudo -n cat /etc/rosie-rt-core/pki/pair-id)" = ` + shellQuote(from),
	}, "\n")
}
