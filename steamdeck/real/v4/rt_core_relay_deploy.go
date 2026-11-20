package main

import (
	"fmt"
	"io"
	"os"
	"os/exec"
	"path"
	"strconv"
	"strings"
)

// rtCoreUnitAssertions requires installed, stopped services and an exclusive
// master selection. Enablement alone never authorises motion.
func rtCoreUnitAssertions(activate bool) string {
	enabled := "disabled"
	if activate {
		enabled = "enabled"
	}
	return strings.Join([]string{
		`for unit in robot-v4-rtcore.service robot-v4-rtcore-nats-bridge.service; do`,
		`  test "$(sudo -n systemctl is-enabled "$unit" || true)" = masked || { printf 'handover failed host=%s unit=%s expected=masked\n' "$(hostname)" "$unit" >&2; exit 1; }`,
		`  test "$(sudo -n systemctl show "$unit" -p ActiveState --value)" = inactive || { printf 'handover failed host=%s unit=%s expected=inactive\n' "$(hostname)" "$unit" >&2; exit 1; }`,
		`done`,
		`for unit in rosie-rt-core.service rosie-rt-control.service rosie-rt-natspublisher.service; do`,
		`  test "$(sudo -n systemctl show "$unit" -p LoadState --value)" = loaded`,
		`  test "$(sudo -n systemctl show "$unit" -p ActiveState --value)" = inactive`,
		`done`,
		`for unit in rosie-rt-core.service rosie-rt-control.service; do`,
		`  test "$(sudo -n systemctl is-enabled "$unit" || true)" = ` + enabled,
		`done`,
		`test "$(sudo -n systemctl is-enabled rosie-rt-natspublisher.service || true)" = disabled`,
		`sudo -n systemctl show rosie-rt-core.service -p Conflicts --value | tr ' ' '\n' | grep -Fx robot-v4-rtcore.service`,
		`sudo -n systemctl show rosie-rt-control.service -p Conflicts --value | tr ' ' '\n' | grep -Fx robot-v4-rtcore-nats-bridge.service`,
		`sudo -n systemctl show rosie-rt-natspublisher.service -p Conflicts --value | tr ' ' '\n' | grep -Fx robot-v4-rtcore-nats-bridge.service`,
	}, "\n")
}

// rtCoreClosureSummary exposes the bus handover and retained foundation so an
// operator cannot mistake package installation for a running controller.
func rtCoreClosureSummary(rtHost, relayHost string) string {
	return fmt.Sprintf("v4_closure: stop+disable+remove+mask robot-v4-rtcore.service robot-v4-rtcore-nats-bridge.service on %s; stop+disable+remove robot-v4-pair-owner.service on %s; stop+disable rosie-rt-core.service rosie-rt-control.service rosie-rt-natspublisher.service on %s; bus owner after this deploy: none until rosie-rt-core is activated; retained: ethercat.service.d/20-robot-v4-device-permissions.conf supplies /dev/EtherCAT0 group picontrol; robot-v4-cpu-performance.service robot-v4-ethercat-nic-tune.service robot-v4-irq-affinity.service stay installed", rtHost, relayHost, rtHost)
}

// rtCoreDisarmedProbe reads `rtctl status --json` on stdin and fails unless the
// controller is disarmed, has no enabled axis and holds no grant.
const rtCoreDisarmedProbe = `import json,sys
` + rtCoreBusyPy + `d=json.load(sys.stdin)
c=d.get('data',d)['core']
assert not rosie_busy(c), c
`

// rtCoreHandoverPreflight refuses to stop a running controller that is armed,
// enabled or granted: stopping it would drop torque under an operator's session.
// An inactive adapter has nothing to stop, so a first install proceeds.
func rtCoreHandoverPreflight() string {
	return strings.Join([]string{
		`if test "$(sudo -n systemctl show rosie-rt-control.service -p ActiveState --value 2>/dev/null || true)" = active; then`,
		`  preflight_status="$(sudo -n /opt/rosie-rt-core/current/bin/rtctl status --socket /run/rosie-rt-core/control.sock --json)" || { printf 'handover refused host=%s reason=running controller status unavailable\n' "$(hostname)" >&2; exit 1; }`,
		`  printf '%s\n' "$preflight_status" | python3 -c ` + shellQuote(rtCoreDisarmedProbe) + ` || { printf 'handover refused host=%s reason=controller armed, enabled or granted; disarm and release first\n' "$(hostname)" >&2; exit 1; }`,
		`fi`,
	}, "\n")
}

// rtCoreHealthRequired are the hostcheck ids deploy's health gate requires to
// pass ('deployment' too when present). The gate's assertion prints every
// check; the rest are not gate conditions: CPU isolation and nohz_full, and the
// /run/rosie-rt-core directories, which exist only while the units run and so
// always fail in a gate that runs with them stopped (a RevPi servo host reported all
// five as fail on every deploy from 6c6f97c0 to bc423a86).
func rtCoreHealthRequired(etherCATGroup string) []string {
	return []string{"rt_cpu", "preempt_rt", "igh_master", "master_nic", "ethercat_device", "group_rosie-rt", "group_rosie-ctl", "group_rosie-rt-clients",
		"group_" + etherCATGroup, "user_rosie-rt", "user_rosie-ctl", "memlock_rosie-rt-core"}
}

// rtCoreDeploymentHealth checks installation prerequisites without treating
// absent runtime sockets or unqualified host timing as running motion evidence.
func rtCoreDeploymentHealth(c robotV4RTCoreConfig) string {
	required := rtCoreHealthRequired(c.EtherCATDeviceGroup)
	for i, id := range required {
		required[i] = "'" + id + "'"
	}
	probe := `import json,sys
r=json.load(sys.stdin)
assert r['schema']=='rosie-rt-core.hostcheck.v1'
checks={c['id']:c['status'] for c in r['checks']}
required=[` + strings.Join(required, ",") + `]
assert all(checks.get(k)=='pass' for k in required), checks
assert 'deployment' not in checks or checks['deployment']=='pass', checks
`
	lines := []string{
		rtCoreUnitAssertions(c.Activate),
		`sudo -n test -s /etc/rosie-rt-core/control.env`,
		`sudo -n grep -Fx ` + shellQuote("ROSIE_RT_PAIR_ID="+c.PairID) + ` /etc/rosie-rt-core/control.env`,
		`sudo -n grep -Fx ` + shellQuote("ROSIE_RT_PAIR_REVISION="+strconv.FormatUint(c.PairRevision, 10)) + ` /etc/rosie-rt-core/control.env`,
		// The cell pin is the approval; a package that compiles anything else is refused.
		`sudo -n grep -Fx ` + shellQuote("ROSIE_RT_CONFIGURATION_SHA256="+c.ConfigurationSHA256) + ` /etc/rosie-rt-core/control.env || { printf 'deploy refused host=%s reason=compiled configuration differs from the cell pin %s\n' "$(hostname)" ` + shellQuote(c.ConfigurationSHA256) + ` >&2; exit 1; }`,
	}
	if c.RemotePKI == rtCoreRemotePKIInstalled {
		lines = append(lines,
			`sudo -n grep -Eq '^ROSIE_RT_REMOTE_ARGS="--remote-listen ' /etc/rosie-rt-core/control.env`,
			`for file in ca.pem server.pem server-key.pem crl.pem; do sudo -n test -s "/etc/rosie-rt-core/pki/$file"; done`)
	}
	return strings.Join(append(lines,
		`store_release="$(readlink -f /opt/rosie-rt-core/current)"`,
		`test "$store_release" = "/opt/rosie-rt-core/$(sudo -n /opt/rosie-rt-core/current/bin/rt-package git-sha "$store_release")"`,
		`sudo -n /opt/rosie-rt-core/current/bin/rt-package verify "$store_release"`,
		`hostcheck_status=0`,
		`hostcheck_json="$(sudo -n env ROSIE_RT_ETHERCAT_GROUP=`+shellQuote(c.EtherCATDeviceGroup)+` /opt/rosie-rt-core/current/bin/rtctl hostcheck --json)" || hostcheck_status=$?`,
		`case "$hostcheck_status" in 0|2|3) ;; *) exit 1 ;; esac`,
		`printf '%s\n' "$hostcheck_json" | python3 -c `+shellQuote(probe),
	), "\n")
}

// rtCoreRemoteArgsGuard refuses, before any unit stops, a deploy whose
// published ROSIE_RT_REMOTE_ARGS would lose the host's remote listener. A
// manifest without remote_pki "installed" publishes ROSIE_RT_REMOTE_ARGS="":
// on 2026-10-12 that silently replaced a RevPi servo host's hand-provisioned
// --remote-listen 127.0.0.1:8443 line, and olp install then failed its live
// Describe with the new controller already running. An installed-PKI cell
// keeps the host line, so it needs one --remote-listen line and matching
// installed pair identities; --keep-remote-args checks too late, after stops.
func rtCoreRemoteArgsGuard(c robotV4RTCoreConfig, controlEnv, pkiDir string) string {
	return rtCoreRemoteArgsGuardFor(c, controlEnv, pkiDir, "")
}

func rtCoreRemoteArgsGuardFor(c robotV4RTCoreConfig, controlEnv, pkiDir, fromPair string) string {
	env := shellQuote(controlEnv)
	pki := shellQuote(pkiDir + "/pair-id")
	lines := []string{
		`remote_lines=0; remote_line=""`,
		`if sudo -n test -e ` + env + `; then`,
		`  remote_lines=$(sudo -n grep -c '^ROSIE_RT_REMOTE_ARGS=' ` + env + ` || true)`,
		`  remote_line=$(sudo -n grep '^ROSIE_RT_REMOTE_ARGS=' ` + env + ` | head -n 1 || true)`,
		`fi`,
	}
	if c.RemotePKI == rtCoreRemotePKIInstalled {
		expectedPair := c.PairID
		pairLabel := "manifest pair"
		if fromPair != "" {
			expectedPair = fromPair
			pairLabel = "previous pair"
		}
		return strings.Join(append(lines,
			`remote_ok=0`,
			`case "$remote_line" in 'ROSIE_RT_REMOTE_ARGS="--remote-listen '*) if test "$remote_lines" = 1; then remote_ok=1; fi ;; esac`,
			`test "$remote_ok" = 1 || { printf 'deploy refused host=%s reason=rt_core.remote_pki is "installed" but control.env carries %s ROSIE_RT_REMOTE_ARGS line(s) and no single --remote-listen line to keep (%s); restore the listener first (realtime-core rollback restores the backed-up control.env); nothing was stopped\n' "$(hostname)" "$remote_lines" "$remote_line" >&2; exit 1; }`,
			`env_pair=$(sudo -n sed -n 's/^ROSIE_RT_PAIR_ID=//p' `+env+`)`,
			`pki_pair=$(sudo -n cat `+pki+` 2>/dev/null || true)`,
			`test "$env_pair" = `+shellQuote(expectedPair)+` && test "$pki_pair" = `+shellQuote(expectedPair)+` || { printf 'deploy refused host=%s reason=installed control.env pair (%s) or PKI pair (%s) differs from `+pairLabel+` (%s); nothing was stopped\n' "$(hostname)" "$env_pair" "$pki_pair" `+shellQuote(expectedPair)+` >&2; exit 1; }`,
		), "\n")
	}
	lines = append(lines,
		`case "$remote_line" in ''|'ROSIE_RT_REMOTE_ARGS=""') ;; *) printf 'deploy refused host=%s reason=control.env carries %s but the cell manifest has no rt_core.remote_pki "installed": this deploy would publish ROSIE_RT_REMOTE_ARGS="" and remove the remote listener; review the line and set remote_pki to "installed"; nothing was stopped\n' "$(hostname)" "$remote_line" >&2; exit 1 ;; esac`)
	if c.RemoteListen != "" {
		lines = append(lines, `printf 'warning: rt_core.remote_listen=%s names a remote endpoint but no listener PKI is installed or claimed: this deploy publishes ROSIE_RT_REMOTE_ARGS="" (no remote listener)\n' `+shellQuote(c.RemoteListen))
	}
	return strings.Join(lines, "\n")
}

// rtCoreControlEnvCommand publishes the compiled binding. An installed-PKI cell
// keeps its host listener line; otherwise the listener is derived from
// remote_listen with no PKI, which publishes it disabled.
func rtCoreControlEnvCommand(c robotV4RTCoreConfig) string {
	return rtCoreControlEnvCommandFor(c, "/opt/rosie-rt-core/current/"+strings.TrimPrefix(c.MachineConfig, "rt-core/"))
}

// rtCoreControlEnvCommandFor publishes the binding compiled from machinePath, an
// absolute path on the servo host (the package copy, or a configuration-only
// deploy's staged history file), with the installed release's own rtctl.
func rtCoreControlEnvCommandFor(c robotV4RTCoreConfig, machinePath string) string {
	machine := shellQuote(machinePath) + " " + shellQuote(c.PairID) + " " + strconv.FormatUint(c.PairRevision, 10)
	const env = `sudo -n env PATH=/opt/rosie-rt-core/current/bin:/usr/sbin:/usr/bin:/sbin:/bin `
	if c.RemotePKI == rtCoreRemotePKIInstalled {
		return env + `bash /opt/rosie-rt-core/current/host/generate-control-env.sh --keep-remote-args ` + machine
	}
	return env + `ROSIE_RT_REMOTE_LISTEN=` + shellQuote(c.RemoteListen) + ` bash /opt/rosie-rt-core/current/host/generate-control-env.sh ` + machine
}

// rtCoreStartReadySeconds bounds how long start waits for the started core to
// verify every axis's startup configuration. Measured 2026-10-06 on
// a RevPi servo host (six A6-EC): config_verified_mask=63 about 18 s after
// restart. Unverified on other cells.
const rtCoreStartReadySeconds = 60

// rtCoreStartBringupSample prints one line of the bus bring-up from the core's
// own metrics file (operational drives, per-axis OP and AL state, the
// master's AL states, startup resets; the poll's second is the time). Start
// prints it whenever it changes, so a start that does not converge leaves the drives' progress over
// time in the journal: on 2026-10-07 the first start of bc423a86 on one cell sat
// at 1/6 then 5/6 drives in OP (master AL 0xA) until the 60 s bound and only
// the operator's hand reading of the journal showed it.
const rtCoreStartBringupSample = `import json,sys
try:
    m=json.load(sys.stdin)
except Exception:
    print('metrics=unavailable'); sys.exit(0)
axes=m.get('axes') or []
print('op=%s/%d online=%s master_al=0x%x axes_op=%s axes_al=%s startup_ready=%s resets=%s' % (m.get('operational_slaves','?'), len(axes), m.get('online_slaves','?'), int(m.get('master_al_states') or 0), ''.join(str(int(a.get('slave_operational') or 0)) for a in axes), ','.join('0x%x' % int(a.get('slave_al_state') or 0) for a in axes), m.get('startup_ready','?'), m.get('startup_reset_count','?')))
`

// rtCoreStartCommand starts an installed-disarmed controller and proves it came
// up as the cell pin with every axis verified and nothing armed or granted. Any
// failure stops both units again, so a failed start leaves no bus owner. A
// failure ends with one `start stall kind=...` line: bringup (the controller
// never answered or did not verify every axis in time: the bus did not
// converge), identity (it runs another configuration or backend) or busy (it
// came up armed, enabled or granted). Only bringup is worth one more start.
func rtCoreStartCommand(c robotV4RTCoreConfig) (string, error) {
	if err := c.validate(); err != nil {
		return "", err
	}
	ready := `import json,sys
` + rtCoreBusyPy + `describe=json.load(open(sys.argv[1])); describe=describe.get('data',describe)
status=json.load(open(sys.argv[2])); core=status.get('data',status)['core']
if describe['configuration_sha256']!=sys.argv[3] or describe['backend']!='ethercat':
    sys.exit('kind=identity configuration=%s backend=%s' % (describe['configuration_sha256'], describe['backend']))
if not core['config_verified_mask']==(1<<len(describe['axes']))-1:
    sys.exit('kind=bringup config_verified_mask=%d' % core['config_verified_mask'])
if rosie_busy(core):
    sys.exit('kind=busy armed=%s axis_enable_mask=%s lease_valid=%s' % (core['armed'], core['axis_enable_mask'], core.get('lease_valid')))
print('rt-core started configuration=%s axes=%d config_verified_mask=%d armed=0 home_valid_mask=%d safety_fault_mask=%d execution_fault_reasons=%d' % (describe['configuration_sha256'], len(describe['axes']), core['config_verified_mask'], core['home_valid_mask'], core['safety_fault_mask'], core['execution_fault_reasons']))
`
	pin := shellQuote(c.ConfigurationSHA256)
	remoteCheck := `if sudo -n grep -Eq '^ROSIE_RT_REMOTE_ARGS="--remote-listen ' /etc/rosie-rt-core/control.env; then
  test "$(sudo -n cat /etc/rosie-rt-core/pki/pair-id 2>/dev/null)" = ` + shellQuote(c.PairID) + ` || refuse "installed PKI pair differs from control.env and the cell pin"
fi`
	if c.RemotePKI == rtCoreRemotePKIInstalled {
		remoteCheck = `sudo -n grep -Eq '^ROSIE_RT_REMOTE_ARGS="--remote-listen ' /etc/rosie-rt-core/control.env || refuse "installed pair PKI has no remote listener"
test "$(sudo -n cat /etc/rosie-rt-core/pki/pair-id 2>/dev/null)" = ` + shellQuote(c.PairID) + ` || refuse "installed PKI pair differs from control.env and the cell pin"`
	}
	return strings.Join([]string{
		"set -eu",
		"export NO_REAL_MOVEMENT=1",
		deployTimingStepFunc,
		`refuse() { printf 'start refused host=%s reason=%s\n' "$(hostname)" "$1" >&2; exit 1; }`,
		`for unit in rosie-rt-core.service rosie-rt-control.service; do`,
		`  test "$(sudo -n systemctl show "$unit" -p LoadState --value)" = loaded || refuse "$unit not installed"`,
		// A unit that failed (a startup check refused, e.g. a power-cycle key not yet
		// effective) has no process and holds no bus; it is the posture a retry starts from.
		`  state="$(sudo -n systemctl show "$unit" -p ActiveState --value)"`,
		`  case "$state" in inactive) ;; failed) printf 'start note=%s was failed; starting again\n' "$unit" ;; *) refuse "$unit already active; start expects the installed-disarmed posture" ;; esac`,
		`done`,
		`sudo -n grep -Fx ` + shellQuote("ROSIE_RT_CONFIGURATION_SHA256="+c.ConfigurationSHA256) + ` /etc/rosie-rt-core/control.env >/dev/null || refuse "control.env is not the cell pin"`,
		`sudo -n grep -Fx ` + shellQuote("ROSIE_RT_PAIR_ID="+c.PairID) + ` /etc/rosie-rt-core/control.env >/dev/null || refuse "control.env pair differs"`,
		`sudo -n grep -Fx ` + shellQuote("ROSIE_RT_PAIR_REVISION="+strconv.FormatUint(c.PairRevision, 10)) + ` /etc/rosie-rt-core/control.env >/dev/null || refuse "control.env pair revision differs"`,
		remoteCheck,
		`sudo -n /opt/rosie-rt-core/current/bin/rt-package verify "$(readlink -f /opt/rosie-rt-core/current)" >/dev/null || refuse "selected release fails verification"`,
		`scratch="$(mktemp -d)"`,
		`trap 'rm -rf -- "$scratch"' EXIT`,
		`step "phase start-to-verified"`,
		`sudo -n systemctl start rosie-rt-core.service rosie-rt-control.service`,
		`started=0 sample="" last_sample="" reason="kind=bringup control socket not answering"`,
		`for attempt in $(seq 1 ` + strconv.Itoa(rtCoreStartReadySeconds) + `); do`,
		`  sample="$(sudo -n cat /run/rosie-rt-core/native/metrics.json 2>/dev/null | python3 -c ` + shellQuote(rtCoreStartBringupSample) + ` 2>/dev/null || echo metrics=unreadable)"`,
		`  if test "$sample" != "$last_sample"; then printf 'start bring-up t=%ss %s\n' "$attempt" "$sample"; last_sample="$sample"; fi`,
		`  if sudo -n /opt/rosie-rt-core/current/bin/rtctl describe --socket /run/rosie-rt-core/control.sock --json >"$scratch/describe.json" 2>/dev/null &&`,
		`     sudo -n /opt/rosie-rt-core/current/bin/rtctl status --socket /run/rosie-rt-core/control.sock --json >"$scratch/status.json" 2>/dev/null; then`,
		`    if python3 -c ` + shellQuote(ready) + ` "$scratch/describe.json" "$scratch/status.json" ` + pin + ` 2>"$scratch/probe.err"; then started=1; break; fi`,
		`    reason="$(tail -n 1 "$scratch/probe.err")"`,
		`  fi`,
		`  sleep 1`,
		`done`,
		`if test "$started" != 1; then`,
		`  sudo -n systemctl stop rosie-rt-control.service rosie-rt-core.service || true`,
		`  printf 'start failed host=%s: not verified within ` + strconv.Itoa(rtCoreStartReadySeconds) + ` s; units stopped; bus owner: none\n' "$(hostname)" >&2`,
		`  case "$reason" in kind=*) ;; *) reason="kind=bringup $reason" ;; esac`,
		`  printf 'start stall %s; last bring-up sample: %s\n' "$reason" "$last_sample" >&2`,
		`  exit 1`,
		`fi`,
		`step "phase end verified at poll $attempt"`,
		`printf '%s\n' 'Started disarmed. Start does not enable at boot, arm, Home or move; those remain explicit operator actions.'`,
	}, "\n"), nil
}

// rtCoreRelayDeployCommand verifies the single artifact before extraction,
// delegates release selection to install.sh, and leaves both masters stopped.
func rtCoreRelayDeployCommand(relayPackage, rtTarget, digest, rosieHome string, c robotV4RTCoreConfig, rtHost, relayHost string) (string, error) {
	return rtCoreRelayDeployCommandPair(relayPackage, rtTarget, digest, rosieHome, c, rtHost, relayHost, "")
}

func rtCoreRelayDeployCommandPair(relayPackage, rtTarget, digest, rosieHome string, c robotV4RTCoreConfig, rtHost, relayHost, fromPair string) (string, error) {
	if err := c.validate(); err != nil {
		return "", err
	}
	normalized, err := normalizeRobotV4SHA256(digest)
	if err != nil {
		return "", err
	}
	digest = strings.TrimPrefix(normalized, "sha256:")
	if !strings.HasPrefix(rosieHome, "/") || path.Clean(rosieHome) != rosieHome || rosieHome == "/" {
		return "", fmt.Errorf("rt-core requires an absolute rosie_home")
	}
	if strings.ContainsAny(rosieHome+rtTarget, "\r\n") {
		return "", fmt.Errorf("rt-core paths and SSH targets must not contain line breaks")
	}
	archive := path.Join(rosieHome, "rt-core", "packages", "rosie-rt-core.tar.gz")
	staging := path.Join(rosieHome, "rt-core", "staging", digest)
	installEnv := "ROSIE_RT_ETHERCAT_GROUP=" + shellQuote(c.EtherCATDeviceGroup)
	if c.NATS.URL != "" {
		installEnv += " ROSIE_RT_NATS_URL=" + shellQuote(c.NATS.URL)
	}
	if c.NATS.Host != "" {
		installEnv += " ROSIE_RT_NATS_HOST=" + shellQuote(c.NATS.Host)
	}
	receive := "set -eu\nmkdir -p " + shellQuote(path.Dir(archive)) + "\ncat > " + shellQuote(archive+".tmp") +
		"\nprintf '%s  %s\\n' " + shellQuote(digest) + " " + shellQuote(archive+".tmp") + " | sha256sum -c -\nmv -f " + shellQuote(archive+".tmp") + " " + shellQuote(archive)
	lines := []string{
		"set -eu",
		"export NO_REAL_MOVEMENT=1 ROSIE_HOME=" + shellQuote(rosieHome),
		deployTimingStepFunc,
		`step "phase extract"`,
		"PACKAGE=" + shellQuote(staging),
		"printf '%s  %s\\n' " + shellQuote(digest) + " " + shellQuote(archive) + " | sha256sum -c -",
		`test ! -L "$PACKAGE"`,
		`if test -e "$PACKAGE"; then rm -rf -- "$PACKAGE"; fi`,
		`mkdir -p "$PACKAGE"`,
		"tar --extract --gzip --no-same-owner -f " + shellQuote(archive) + ` -C "$PACKAGE"`,
		`step "phase preflight"`,
		// /dev/EtherCAT0 permissions: on a RevPi the retained v4 foundation's
		// drop-in (group picontrol); on a host provisioned by
		// host/ethercat-foundation.sh (the Pi 5 mini bench) its udev rule, which
		// must grant the cell's ethercat_device_group.
		`if sudo -n test -s /etc/systemd/system/ethercat.service.d/20-robot-v4-device-permissions.conf; then`,
		`  sudo -n grep -F 'chgrp picontrol' /etc/systemd/system/ethercat.service.d/20-robot-v4-device-permissions.conf >/dev/null`,
		`elif ! grep -qxF ` + shellQuote(`KERNEL=="EtherCAT0", GROUP="`+c.EtherCATDeviceGroup+`", MODE="0660"`) + ` /etc/udev/rules.d/99-rosie-rt-core-ethercat.rules 2>/dev/null; then`,
		`  printf 'handover refused host=%s missing EtherCAT device permissions: neither the v4 picontrol drop-in nor the foundation udev rule for group %s\n' "$(hostname)" ` + shellQuote(c.EtherCATDeviceGroup) + ` >&2; exit 1`,
		`fi`,
		rtCoreHandoverPreflight(),
		rtCoreRemoteArgsGuardFor(c, "/etc/rosie-rt-core/control.env", "/etc/rosie-rt-core/pki", fromPair),
		// The running binding is kept before anything stops, in the history the
		// configuration-only deploy uses: tier-3 rollback restores it with the
		// previous release (README "Tier 3"); a failed backup stops the deploy.
		`if sudo -n test -s /etc/rosie-rt-core/control.env; then`,
		`  control_backup="` + rtCoreConfigHistoryDir + `/control-env/$(date -u +%Y%m%dT%H%M%SZ)-$(sudo -n sed -n 's/^ROSIE_RT_CONFIGURATION_SHA256=//p' /etc/rosie-rt-core/control.env | head -n 1).env"`,
		`  sudo -n install -d -m 0750 -o root -g root ` + rtCoreConfigHistoryDir + ` ` + rtCoreConfigHistoryDir + `/control-env`,
		`  sudo -n install -m 0640 -o root -g rosie-ctl /etc/rosie-rt-core/control.env "$control_backup"`,
		`  printf 'control.env backed up to %s\n' "$control_backup"`,
		`fi`,
	}
	if fromPair != "" {
		lines = append(lines, releasePairPKIServoPreflight(c, fromPair), releasePairPKIBackupScript(fromPair, c.PairID))
	}
	lines = append(lines,
		`step "phase stop"`,
		`for unit in rosie-rt-natspublisher.service rosie-rt-control.service rosie-rt-core.service; do`,
		`  if test "$(sudo -n systemctl show "$unit" -p LoadState --value)" != not-found; then sudo -n systemctl stop "$unit"; sudo -n systemctl disable "$unit"; fi`,
		`done`,
		`step "phase install"`,
		`cd "$PACKAGE"`,
		`sudo -n env `+installEnv+` bash host/install.sh --from-package "$PACKAGE" --live`,
		`step "phase control-env"`,
	)
	if fromPair != "" {
		lines = append(lines, releasePairPKIControlEnvCommand(c))
	} else {
		lines = append(lines, rtCoreControlEnvCommand(c))
	}
	lines = append(lines,
		`step "phase v4-closure"`,
		`for unit in robot-v4-rtcore-nats-bridge.service robot-v4-rtcore.service; do`,
		`  if test "$(sudo -n systemctl show "$unit" -p LoadState --value)" = not-found; then continue; fi`,
		`  if test "$(sudo -n systemctl is-enabled "$unit" || true)" = masked; then sudo -n systemctl stop "$unit"; else sudo -n systemctl disable --now "$unit"; fi || { printf 'handover failed host=%s unit=%s stop+disable failed\n' "$(hostname)" "$unit" >&2; exit 1; }`,
		`  test "$(sudo -n systemctl show "$unit" -p ActiveState --value)" = inactive || { printf 'handover failed host=%s unit=%s expected=inactive\n' "$(hostname)" "$unit" >&2; exit 1; }`,
		`done`,
		`sudo -n rm -f /etc/systemd/system/robot-v4-rtcore.service /etc/systemd/system/robot-v4-rtcore-nats-bridge.service /etc/default/robot-v4-rtcore-nats-bridge`,
		`sudo -n rm -rf /etc/systemd/system/robot-v4-rtcore.service.d /etc/systemd/system/robot-v4-rtcore-nats-bridge.service.d /run/systemd/system/robot-v4-rtcore.service.d /run/systemd/system/robot-v4-rtcore-nats-bridge.service.d`,
		`sudo -n systemctl mask robot-v4-rtcore-nats-bridge.service robot-v4-rtcore.service`,
		`sudo -n systemctl daemon-reload`,
		`sudo -n systemctl disable rosie-rt-core.service rosie-rt-control.service rosie-rt-natspublisher.service`,
	)
	if c.NATS.URL == "" {
		lines = append(lines, `sudo -n rm -f /etc/rosie-rt-core/natspublisher.env`)
	}
	if c.Activate {
		lines = append(lines, `sudo -n systemctl enable rosie-rt-core.service rosie-rt-control.service`)
	}
	// A fresh shell preserves errexit for every health assertion. Putting the
	// assertions directly in an if condition would suppress intermediate failures.

	lines = append(lines, `step "phase health"`, rtCoreHealthGate(rtCoreDeploymentHealth(c), rtTarget), `step "phase end"`,
		`if test -L /opt/rosie-rt-core/previous; then printf '%s\n' 'previous release remains selectable with host/install.sh --rollback'; fi`)
	ssh := "ssh -o BatchMode=yes " + sshAliveShell(sshConnectTimeout) + " -- " + shellQuote(rtTarget) + " "
	// The v4 pair owner is another legacy command producer on the relay host.
	// Remove it only after the servo install succeeds; leave no mask for tee.
	relayClosure := `if test "$(sudo -n systemctl show robot-v4-pair-owner.service -p LoadState --value)" != not-found; then
  sudo -n systemctl disable --now robot-v4-pair-owner.service || { printf 'closure failed host=%s unit=robot-v4-pair-owner.service\n' "$(hostname)" >&2; exit 1; }
fi
sudo -n rm -f /etc/systemd/system/robot-v4-pair-owner.service
sudo -n systemctl daemon-reload
test ! -e /etc/systemd/system/robot-v4-pair-owner.service
test ! -L /etc/systemd/system/robot-v4-pair-owner.service
test "$(sudo -n systemctl show robot-v4-pair-owner.service -p ActiveState --value)" = inactive
`
	relayPreflight := ""
	if fromPair != "" {
		relayPreflight = releasePairPKIRelayPreflight(c, ssh) + "\n"
	}
	return "set -eu\nexport NO_REAL_MOVEMENT=1\n" + deployTimingStepFunc + "\n" + rtCoreMaintenancePhaseDefault + ssh + shellQuote("true") + "\n" + relayPreflight +
		"step 'phase package-to-servo'\nmaint_phase 'copying the rt-core package to the servo host'\n" +
		"cat " + shellQuote(relayPackage) + " | " + ssh + shellQuote("bash -c "+shellQuote(receive)) + "\n" +
		"step 'phase servo-connect'\nmaint_phase 'installing on the servo host: stopping the controller, bus handover, health checks'\n" +
		ssh + shellQuote("bash -se") + " <<'ROSIE_RT_CORE_INSTALL'\n" + strings.Join(lines, "\n") + "\nROSIE_RT_CORE_INSTALL\n" +
		"step 'phase pair-owner-closure'\nmaint_phase 'closing the v4 pair owner on the relay'\n" + relayClosure + "step 'phase end'\n" +
		"printf '%s\\n' " + shellQuote("rt-core installed and disarmed; "+rtCoreClosureSummary(rtHost+" ("+rtTarget+")", relayHost)), nil
}

// rtCoreHealthGate keeps intermediate failures fatal and prints an explicit
// recovery path after the bus handover; it never attempts automatic rollback.
func rtCoreHealthGate(health, rtTarget string) string {
	// The retry names this run's own manifest, so the operator re-runs the
	// cell they deployed; "<cell>" only when none was selected.
	manifest := selectedRobotV4ManifestPath()
	if manifest == "" {
		manifest = "rt-core/config/cells/<cell>.json"
	}
	recovery := []string{
		"rt-core health failed on " + rtTarget,
		"v4 units robot-v4-rtcore.service robot-v4-rtcore-nats-bridge.service are masked; rosie units are stopped; bus owner: none; Jetson pair-owner remains unchanged.",
		"To restore v4, obtain the published robot-stack-v0.2.0 release tarball and its rtcore-v0.2.0, rtcore-bridge-v0.2.0 and pair-owner-v0.2.0 component assets (or the recorded v0.1.0 release set for the cells it was installed on).",
		"Verify the release checksums and exact retained cell receipts/configuration; keep motion inhibited and stop the native services before bus handover.",
		"Install the verified release binaries into the versioned slots named by those receipts using the retained release installation procedure; restore that release's units and configuration on each recorded host before unmasking its v4 units.",
		"The current checkout refuses v4 manifests with reason=backend_retired; do not rebuild or deploy the retired runtime from it. See rt-core/docs/MIGRATION.md (Repository defaults and deployed cutover).",
		"To continue rt-core instead, retain the rt-core manifests, correct the reported prerequisites, and run:",
		"go run . realtime-core deploy --manifest " + manifest,
		"Neither recovery path runs automatically.",
	}
	for i := range recovery {
		recovery[i] = shellQuote(recovery[i])
	}
	return "if bash -se <<'ROSIE_RT_CORE_HEALTH'\nset -eu\n" + health + "\nROSIE_RT_CORE_HEALTH\nthen :; else printf '%s\\n' " + strings.Join(recovery, " ") + " >&2; exit 1; fi"
}

// rtCoreRollbackCommand uses the installer's one-generation sentinel; it never
// starts a unit or constructs another release store.
func rtCoreRollbackCommand() string {
	return `set -eu
sudo -n systemctl stop rosie-rt-natspublisher.service rosie-rt-control.service rosie-rt-core.service
sudo -n systemctl disable rosie-rt-natspublisher.service rosie-rt-control.service rosie-rt-core.service
sudo -n test ! -e /opt/rosie-rt-core/.rollback-consumed
sudo -n test -L /opt/rosie-rt-core/previous
sudo -n bash /opt/rosie-rt-core/current/host/install.sh --rollback
sudo -n test -f /opt/rosie-rt-core/.rollback-consumed
sudo -n /opt/rosie-rt-core/current/bin/rt-package verify "$(readlink -f /opt/rosie-rt-core/current)"`
}

// rtCoreRelayRoute resolves the servo host and the relay that reaches it.
func rtCoreRelayRoute() (robotV4DaemonNodeInventoryNode, string, string, string, error) {
	node, ok := robotV4DaemonNodeInventoryRecord(robotV4RealtimeCoreNode)
	if !ok || !node.selectsRTCore() || node.RTCore == nil {
		return node, "", "", "", fmt.Errorf("rt-core deploy requires a selected manifest binding")
	}
	route, err := robotV4ManifestManagementConnection(node.Peer)
	if err != nil || route.ViaPeer == "" {
		return node, "", "", "", fmt.Errorf("rt-core deploy requires a manifest relay: %v", err)
	}
	relay, err := robotV4ManifestManagementConnection(route.ViaPeer)
	if err != nil {
		return node, "", "", "", err
	}
	return node, remoteTarget(route.SSHUser, route.Host), remoteTarget(relay.SSHUser, relay.Host), route.ViaPeer, nil
}

// rtCoreRelay is how deploy and start reach the manifest relay from the Mac:
// plain ssh to its LAN address, which needs the Mac's key unlocked and
// authorised there, or Tailscale SSH to its tailnet name by tailnet identity,
// the route every cell-driven command takes. The first live release on a Rosie
// 1430 cell (2026-10-07) failed at deploy's package upload with "Permission denied
// (publickey,password)" on plain ssh while Tailscale SSH to the same relay
// worked for every other step.
type rtCoreRelay struct {
	Transport string // "ssh" or "tailscale"
	Target    string // user@host, the plain ssh route
	Peer      string // the relay's tailnet name
}

func (r rtCoreRelay) userHost() (string, string) {
	if user, host, ok := strings.Cut(r.Target, "@"); ok {
		return user, host
	}
	return "", r.Target
}

func (r rtCoreRelay) String() string {
	if r.Transport == "tailscale" {
		user, _ := r.userHost()
		return remoteTarget(user, r.Peer) + " (tailscale)"
	}
	return r.Target
}

// rtCoreRelayExec runs one Tailscale SSH command; a seam for tests.
var rtCoreRelayExec = func(argv []string, stdin io.Reader, stdout, stderr io.Writer) error {
	cmd := exec.Command(argv[0], argv[1:]...)
	cmd.Stdin, cmd.Stdout, cmd.Stderr = stdin, stdout, stderr
	return cmd.Run()
}

func (r rtCoreRelay) tailscale(stdin io.Reader, stdout, stderr io.Writer, command string) error {
	user, host := r.userHost()
	fmt.Fprintf(stdout, "remote: tailscale ssh %s\n", remoteTarget(user, r.Peer))
	return rtCoreRelayExec(rtCoreConfigRemoteArgv("tailscale", user, r.Peer, host, command), stdin, stdout, stderr)
}

// run executes command on the relay, its output streamed to stdout/stderr.
func (r rtCoreRelay) run(stdout, stderr io.Writer, command string) error {
	if r.Transport != "tailscale" {
		return runRemoteSSH(stdout, stderr, r.Target, command)
	}
	return r.tailscale(nil, stdout, stderr, command)
}

// upload copies a local file to remotePath on the relay.
func (r rtCoreRelay) upload(stdout, stderr io.Writer, localPath, remotePath string) error {
	if r.Transport != "tailscale" {
		return syncRobotV4SSHPrebuiltFile(stdout, stderr, r.Target, localPath, remotePath)
	}
	file, err := os.Open(localPath)
	if err != nil {
		return err
	}
	defer file.Close()
	if err := r.tailscale(file, stdout, stderr, robotV4PrebuiltUploadCommand(remotePath)); err != nil {
		return fmt.Errorf("transfer prebuilt %s to %s:%s: %w", localPath, r, remoteShellPath(remotePath), err)
	}
	fmt.Fprintf(stdout, "tailscale deploy artifact transferred target=%s path=%s binary_sha256=%s target_builds=0\n", r, remoteShellPath(remotePath), fileSHA256OrUnknown(localPath))
	return nil
}

func rtCoreValidTransport(transport string) error {
	if transport != "tailscale" && transport != "ssh" {
		return fmt.Errorf("--transport must be tailscale or ssh, got %q", transport)
	}
	return nil
}

// rtCoreMaintenancePhaseDefault lets a relay script mark its phases whether or
// not rtCoreMaintenanceWrap put it under a maintenance notice.
const rtCoreMaintenancePhaseDefault = "command -v maint_phase > /dev/null 2>&1 || maint_phase(){ :; }\n"

// rtCoreMaintenanceCell names the selected cell by its manifest file.
func rtCoreMaintenanceCell() string {
	return strings.TrimSuffix(path.Base(selectedRobotV4ManifestPath()), ".json")
}

// runRTCoreStartWith is the explicit operator step after an installed-disarmed
// deploy. It runs on the servo host through the manifest relay, under the
// maintenance notice. The controller is stopped, so there is no idle wait.
func runRTCoreStartWith(dryRun bool, transport string, mf rtCoreMaintenanceFlags, stdout, stderr io.Writer) error {
	if err := rtCoreValidTransport(transport); err != nil {
		return err
	}
	node, target, viaTarget, viaPeer, err := rtCoreRelayRoute()
	if err != nil {
		return err
	}
	via := rtCoreRelay{Transport: transport, Target: viaTarget, Peer: viaPeer}
	script, err := rtCoreStartCommand(*node.RTCore)
	if err != nil {
		return err
	}
	mf.IdleWait = 0
	m := newRTCoreMaintenance("start", rtCoreMaintenanceCell(), mf, rtCoreMaintenanceStartEstimate)
	command := rtCoreMaintenanceWrap(m, target, "starting the controller and verifying every axis (stays disarmed)",
		"set -eu\nssh -o BatchMode=yes "+sshAliveShell(sshConnectTimeout)+" -- "+shellQuote(target)+" "+shellQuote("bash -se")+" <<'ROSIE_RT_CORE_START'\n"+script+"\nROSIE_RT_CORE_START\n")
	if dryRun {
		fmt.Fprintf(stdout, "rt-core start target=%s via=%s configuration=%s pair=%s revision=%d posture=started-disarmed\n", target, via, node.RTCore.ConfigurationSHA256, node.RTCore.PairID, node.RTCore.PairRevision)
		fmt.Fprintln(stdout, command)
		return nil
	}
	run, err := newDeployJournalRun(rtCoreMaintenanceCell(), "realtime-core start", stdout)
	if err != nil {
		return err
	}
	run.p.Step("start", fmt.Sprintf("target=%s via=%s configuration=%s", target, via, node.RTCore.ConfigurationSHA256))
	err = run.stream("start", via, deployTimingStepFunc+"\n"+command)
	run.finishStream(err)
	return err
}

// runRTCoreDeployWith installs the package under the maintenance notice. The
// notice goes up when the relay script starts (after the artifact is built
// and copied to the relay, which does not touch the cell), before the idle
// wait and the handover preflight; it is cleared when the script exits.
func runRTCoreDeployWith(dryRun bool, packagePath, transport string, mf rtCoreMaintenanceFlags, stdout, stderr io.Writer) error {
	return runRTCoreDeployWithExpected(dryRun, packagePath, "", transport, mf, stdout, stderr)
}

// runRTCoreDeployWithExpected binds a registered candidate digest to the
// package the existing deploy transaction transfers and verifies remotely.
func runRTCoreDeployWithExpected(dryRun bool, packagePath, expectedSHA256, transport string, mf rtCoreMaintenanceFlags, stdout, stderr io.Writer) error {
	return runRTCoreDeployWithExpectedPair(dryRun, packagePath, expectedSHA256, transport, "", mf, stdout, stderr)
}

func runRTCoreDeployWithExpectedPair(dryRun bool, packagePath, expectedSHA256, transport, fromPair string, mf rtCoreMaintenanceFlags, stdout, stderr io.Writer) error {
	if err := rtCoreValidTransport(transport); err != nil {
		return err
	}
	node, target, viaTarget, viaPeer, err := rtCoreRelayRoute()
	if err != nil {
		return err
	}
	if fromPair != "" && (fromPair == node.RTCore.PairID || node.RTCore.RemotePKI != rtCoreRemotePKIInstalled) {
		return fmt.Errorf("pair migration requires a distinct old pair and rt_core.remote_pki installed")
	}
	via := rtCoreRelay{Transport: transport, Target: viaTarget, Peer: viaPeer}
	m := newRTCoreMaintenance("deploy", rtCoreMaintenanceCell(), mf, rtCoreMaintenanceDeployEstimate)
	firstPhase := "preparing the rt-core deploy"
	var local *robotV4SSHPrebuiltArtifact
	if packagePath != "" {
		artifact, err := robotV4LocalRTCorePackage(packagePath, *node.RTCore, expectedSHA256)
		if err != nil {
			return err
		}
		local = &artifact
	}
	if dryRun {
		fmt.Fprintf(stdout, "rt-core deploy artifact=%s target=%s via=%s config=%s pair=%s revision=%d activate=%t remote_pki=%q target_builds=0 posture=installed-disarmed\n", node.RTCore.PackageArtifact, target, via, node.RTCore.MachineConfig, node.RTCore.PairID, node.RTCore.PairRevision, node.RTCore.Activate, node.RTCore.RemotePKI)
		fmt.Fprintln(stdout, rtCoreClosureSummary(node.Peer, viaPeer))
		relayPackage, digest := "/relay/UNBUILT-rosie-rt-core.tar.gz", strings.Repeat("0", 64)
		if local != nil {
			fmt.Fprintf(stdout, "rt-core package=%s sha256=%s source=%s\n", local.Path, local.SHA256, local.SourceSHA256)
			relayPackage, digest = "/relay/"+strings.TrimPrefix(local.SHA256, "sha256:")+".tar.gz", local.SHA256
		}
		// Unverified artifact identity without --package: dry-run renders placeholders without a build.
		script, err := rtCoreRelayDeployCommandPair(relayPackage, target, digest, node.RosieHome, *node.RTCore, node.Peer, viaPeer, fromPair)
		if err == nil {
			fmt.Fprintf(stdout, "maintenance notice %s on the relay: eta=%s idle_wait=%s keep=%t run=%s\n", rtCoreMaintenanceFile, m.ETA, m.IdleWait, m.Keep, m.RunID)
			fmt.Fprintln(stdout, rtCoreMaintenanceWrap(m, target, firstPhase, script))
		}
		return err
	}
	run, err := newDeployJournalRun(rtCoreMaintenanceCell(), "realtime-core deploy", stdout)
	if err != nil {
		return err
	}
	run.p.Step("deploy", fmt.Sprintf("target=%s via=%s configuration=%s", target, via, node.RTCore.ConfigurationSHA256))
	err = func() error {
		var artifact robotV4SSHPrebuiltArtifact
		out, errw := &deployTimingLineWriter{p: run.p}, &deployTimingLineWriter{p: run.p, stderr: true}
		defer out.Flush()
		defer errw.Flush()
		if local != nil {
			artifact = *local
		} else {
			run.begin("package")
			artifact, err = robotV4ResolveSSHPrebuiltArtifact("rt-core", out, errw)
			run.end("package", err)
			if err != nil {
				return err
			}
		}
		relayNode, ok := robotV4DaemonNodeInventoryRecord(robotV4MotionServerNode)
		if !ok || relayNode.Peer != viaPeer {
			return fmt.Errorf("rt-core relay must be the manifest motion-server host")
		}
		relayPath := path.Join(relayNode.RosieHome, "rt-core", "packages", strings.TrimPrefix(artifact.SHA256, "sha256:")+".tar.gz")
		run.begin("package to relay")
		err := via.upload(out, errw, artifact.Path, relayPath)
		run.end("package to relay", err)
		if err != nil {
			return err
		}
		command, err := rtCoreRelayDeployCommandPair(relayPath, target, artifact.SHA256, node.RosieHome, *node.RTCore, node.Peer, viaPeer, fromPair)
		if err != nil {
			return err
		}
		m.Release = firstNonEmpty(m.Release, artifact.SourceSHA256)
		return run.stream("deploy", via, deployTimingStepFunc+"\n"+rtCoreMaintenanceWrap(m, target, firstPhase, command))
	}()
	run.finishStream(err)
	return err
}
