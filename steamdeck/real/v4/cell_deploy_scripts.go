package main

import (
	"encoding/base64"
	"path"
	"slices"
	"strconv"
	"strings"
)

// Rendered relay scripts for the cell-driven deploy commands (realtime-core
// build/precheck/wait-idle/copy-dumps/verify/reboot/resume-after-reboot, olp
// install/rollback/status). Like `realtime-core config`, each script travels
// as the remote command argument `bash -c '<script>'`, never on stdin, so an
// inner ssh cannot swallow it; servo-host scripts go one hop further over a
// shared ControlMaster connection as a heredoc that the servo host saves to a
// file and runs with stdin from /dev/null. Remote progress lines are
// `@@step <ms> <phase> <detail>`; the Mac re-stamps them with elapsed times.
//
// Servo-host rules these scripts keep (CLAUDE.md): nothing heavy runs there
// (copies are nice 19 / ionice idle cat, throttled; nothing is decoded there);
// no Home, HM35, anchor restore or verified-delta command; no signal to
// rosie-rt-core other than through systemctl; a reboot only through
// `realtime-core reboot --owner-approved`.

// cellDeployOLPActivity matches OLP journal lines that mean an operator is
// using the cell (jog hold, arm, program start/play/stop, Move-to). Taken from
// a Rosie 1430 cell's 2026-10-07 wait-idle.sh.
const cellDeployOLPActivity = `jog hold: begin|arm=|dense-execution: (start|play|stop)|move`

// cellDeployRelayPrologue opens every relay script: strict-ish shell, progress,
// refusal, and the servo-host hop over one shared connection (servo: one
// command; servos: a script on stdin; servo_once: no shared connection, for
// polling a host that may be rebooting). Its EXIT trap replaces the one
// rtCoreMaintenanceWrap installs, so it clears the notice too.
func cellDeployRelayPrologue(ctx cellDeployContext) string {
	family := ""
	if _, host, ok := strings.Cut(ctx.ServoTarget, "@"); ok && strings.Contains(host, ":") {
		family = "-6 "
	}
	// set +e: under rtCoreMaintenanceWrap (set -eu) every check here still
	// reports its own failure instead of ending the script mid-step.
	return rtCoreConfigReplace(`set +e -uo pipefail
export LC_ALL=C NO_REAL_MOVEMENT=1
@STEP@
refuse(){ printf '@@refused %s\n' "$*"; exit 2; }
command -v maint_phase > /dev/null 2>&1 || maint_phase(){ :; }
SERVO=@SERVO@
CMP="$HOME/.ssh/rosie-cell-%C"
servo(){ ssh -n @FAMILY@-o BatchMode=yes @ALIVE@ -o ControlMaster=auto -o "ControlPath=$CMP" -o ControlPersist=60 -- "$SERVO" "$@"; }
servos(){ ssh @FAMILY@-o BatchMode=yes @ALIVE@ -o ControlMaster=auto -o "ControlPath=$CMP" -o ControlPersist=60 -- "$SERVO" 'f=$(mktemp) && cat > "$f" && bash "$f" < /dev/null; rc=$?; rm -f "$f"; exit $rc'; }
servo_once(){ ssh -n @FAMILY@-o BatchMode=yes @ALIVE_ONCE@ -o ControlPath=none -- "$SERVO" "$@"; }
trap 'rc=$?; trap - EXIT; if command -v maint_clear > /dev/null 2>&1; then maint_clear; fi; ssh -o "ControlPath=$CMP" -O exit -- "$SERVO" > /dev/null 2>&1; exit $rc' EXIT
`, "@STEP@", rtCoreConfigStepFunc, "@SERVO@", shellQuote(ctx.ServoTarget), "@FAMILY@", family,
		"@ALIVE@", sshAliveShell(sshConnectTimeout), "@ALIVE_ONCE@", sshAliveShell(5))
}

// cellDeployServoFacts runs on the servo host and prints one JSON object of
// read-only facts. With master=1 it also reads the EtherCAT master phase and
// slave list, and only when both rosie units are inactive (the IgH CLI must not
// share the master with a running core); each read is backgrounded with a
// timeout so a wedged master cannot hang the script.
func cellDeployServoFacts(master bool) string {
	withMaster := "0"
	if master {
		withMaster = "1"
	}
	return rtCoreConfigReplace(`set -uo pipefail
export LC_ALL=C
R=/opt/rosie-rt-core/current; SOCK=/run/rosie-rt-core/control.sock; T=@TELEMETRY@; MASTER=@MASTER@
t=$(mktemp -d) || exit 1
trap 'rm -rf -- "$t"' EXIT
@ANCHORS_NOW@
uptime -s > "$t/boot" 2>/dev/null
date -u -d "$(uptime -s)" +%s > "$t/boot_epoch" 2>/dev/null
date -u +%s > "$t/now_epoch"
readlink -f /opt/rosie-rt-core/current > "$t/current" 2>/dev/null
readlink -f /opt/rosie-rt-core/previous > "$t/previous" 2>/dev/null
sudo -n sed -n 's/^ROSIE_RT_CONFIGURATION_SHA256=//p' /etc/rosie-rt-core/control.env > "$t/pin" 2>/dev/null
sudo -n sha256sum "$R/rt_package.json" 2>/dev/null | awk '{print $1}' > "$t/package_manifest_sha256"
for u in rosie-rt-core.service rosie-rt-control.service; do
  printf '%s\t%s\t%s\t%s\n' "$u" "$(systemctl show "$u" -p ActiveState --value)" "$(systemctl is-enabled "$u" 2>/dev/null)" "$(systemctl show "$u" -p MainPID --value)" >> "$t/units"
done
if [ "$(systemctl show rosie-rt-control.service -p ActiveState --value)" = active ]; then
  sudo -n "$R/bin/rtctl" status --socket "$SOCK" --json > "$t/status.json" 2> "$t/status.err"
  sudo -n "$R/bin/rtctl" describe --socket "$SOCK" --json > "$t/describe.json" 2> "$t/describe.err"
  sudo -n cat /run/rosie-rt-core/native/metrics.json > "$t/native.json" 2> /dev/null || rm -f "$t/native.json"
fi
ANCHOR_DIR=$(systemctl show rosie-rt-control.service -p Environment --value | tr ' ' '\n' | sed -n 's/^ROSIE_RT_ANCHOR_DIR=//p' | head -n 1)
printf '%s' "$ANCHOR_DIR" > "$t/anchor_dir"
if [ -n "$ANCHOR_DIR" ]; then
  sudo -n stat -c '%U:%G %a' "$ANCHOR_DIR" > "$t/anchor_layout" 2>/dev/null
  anchors_now > "$t/anchor_files"
fi
sudo -n find "$T" -maxdepth 1 -type f -printf '%f %s\n' > "$t/dumps" 2>/dev/null || true
# A thread passes through D during ordinary disk/IPC I/O; one sample on
# 2026-10-11 caught rt-control in D seconds after its start and rolled back a
# good release. Report only threads that stay D/Z across 5 samples 0.4 s apart
# (2 s; window unverified): a stop wedged in an IgH call stays D for good.
dz(){ ps -eLo pid=,lwp=,stat=,comm= | awk '$3 ~ /^[DZ]/ && $4 ~ /rosie-rt|rt-control|commission|rt-cycle/ { print $1 ":" $2 " " $3 " " $4 }'; }
dz > "$t/dz"
for _ in 1 2 3 4; do
  [ -s "$t/dz" ] || break
  sleep 0.4
  dz > "$t/dz_now"
  awk 'NR==FNR { seen[$1] = 1; next } ($1 in seen)' "$t/dz_now" "$t/dz" > "$t/dz_kept"
  mv "$t/dz_kept" "$t/dz"
done
awk '{ print $2 " " $3 }' "$t/dz" > "$t/dstate"
if [ "$MASTER" = 1 ] && [ "$(systemctl show rosie-rt-core.service -p ActiveState --value)" = inactive ] && [ "$(systemctl show rosie-rt-control.service -p ActiveState --value)" = inactive ]; then
  printf '%s\n' "$(systemctl is-active ethercat.service 2>/dev/null)" > "$t/ethercat_unit"
  ( sudo -n timeout 5 ethercat master > "$t/master" 2>&1 & )
  ( sudo -n timeout 5 ethercat slaves > "$t/slaves" 2>&1 & )
  sleep 6
fi
python3 - "$t" <<'ROSIE_FACTS_PY'
import json, os, sys
t = sys.argv[1]
def rd(name):
    try:
        with open(os.path.join(t, name)) as f:
            return f.read()
    except OSError:
        return ""
def num(name):
    try:
        return int(rd(name).strip())
    except ValueError:
        return 0
out = {"boot": rd("boot").strip(), "boot_epoch": num("boot_epoch"), "now_epoch": num("now_epoch"),
       "current": rd("current").strip(), "previous": rd("previous").strip(), "control_env_pin": rd("pin").strip(),
       "package_manifest_sha256": rd("package_manifest_sha256").strip(),
       "units": {}, "errors": [], "dumps": [], "dstate": [l for l in rd("dstate").splitlines() if l]}
for line in rd("units").splitlines():
    f = line.split("\t")
    if len(f) == 4:
        out["units"][f[0]] = {"active": f[1], "enabled": f[2], "main_pid": f[3]}
def load(name):
    if not os.path.exists(os.path.join(t, name + ".json")):
        return None
    try:
        d = json.loads(rd(name + ".json"))
    except ValueError:
        out["errors"].append("rtctl %s failed: %s" % (name, rd(name + ".err").strip()[-300:]))
        return None
    return d.get("data", d) if isinstance(d, dict) else None
s = load("status")
if s is not None:
    c = s.get("core") or {}
    out["status"] = {k: int(c.get(k) or 0) for k in ("armed", "axis_enable_mask", "lease_valid", "config_verified_mask", "home_valid_mask", "safety_fault_mask", "execution_fault_reasons")}
    out["drive_readbacks"] = []
    native = load("native")
    if native is not None and native.get("daemon_incarnation") == c.get("daemon_incarnation") and native.get("configuration_sha256") == c.get("configuration_sha256"):
        for axis, state in enumerate(native.get("axes") or []):
            for setting in state.get("startup_drive_configs") or []:
                if not isinstance(setting, dict):
                    continue
                out["drive_readbacks"].append({
                    "axis": axis, "slave_position": int(state.get("slave_position", -1)),
                    "setting_key": setting.get("setting_key", ""), "type": setting.get("type", ""),
                    "index": int(setting.get("index", -1)), "subindex": int(setting.get("subindex", -1)),
                    "readback": int(setting.get("readback", 0)),
                    "readback_valid": bool(setting.get("readback_valid")), "verified": bool(setting.get("verified"))})
    alarms, detail = [], {}
    for group in (c.get("axes") or [], s.get("axes") or []):
        for i, a in enumerate(group):
            if (a.get("error_code") or 0) or (a.get("drive_alarm") or "") not in ("", "none"):
                alarms.append("axis %d error_code=%s drive_alarm=%s" % (i, a.get("error_code"), a.get("drive_alarm")))
                x = detail.setdefault(i, {"axis": i, "error_code": 0, "drive_alarm": "", "text": "", "aux_code": 0, "aux_valid": False})
                x["error_code"] = x["error_code"] or int(a.get("error_code") or 0)
                x["drive_alarm"] = x["drive_alarm"] or str(a.get("drive_alarm") or "")
                x["text"] = x["text"] or str(a.get("drive_alarm_text") or "")
                if a.get("drive_alarm_aux_valid"):
                    x["aux_code"], x["aux_valid"] = int(a.get("drive_alarm_aux_code") or 0), True
    out["status"]["alarms"] = sorted(set(alarms))
    out["status"]["drive_alarms"] = [detail[i] for i in sorted(detail)]
    out["status"]["recovery_faults"] = [{"bit": int(f.get("bit") or 0), "name": str(f.get("name") or ""), "axis_mask": int(f.get("axis_mask") or 0),
        "recovery": str(f.get("recovery") or ""), "rehome_axis_mask": int(f.get("rehome_axis_mask") or 0)}
        for f in ((c.get("recovery") or {}).get("faults") or [])]
    out["status"]["verified_reference_mask"] = sum(1 << i for i, a in enumerate(c.get("axes") or [])
        if a.get("position_state") == "trusted" and a.get("audit_status") == "verified" and a.get("coordinate_valid") == 1
        and a.get("coordinate_source_valid") == 1 and a.get("home_valid") is True)
d = load("describe")
if d is not None:
    out["describe"] = {k: str(d.get(k) or "") for k in ("configuration_sha256", "machine_sha256", "deployment_sha256", "capabilities_digest", "backend")}
    out["describe"]["axes"] = len(d.get("axes") or [])
    out["describe"]["protocol"] = "%s.%s" % (d.get("protocol_major"), d.get("protocol_minor"))
    out["describe"]["path_jog"] = bool(d.get("path_jog"))
out["anchor"] = {"dir": rd("anchor_dir").strip(), "layout": rd("anchor_layout").strip(), "files": [l for l in rd("anchor_files").splitlines() if l]}
for line in rd("dumps").splitlines():
    name, _, size = line.rpartition(" ")
    if name:
        out["dumps"].append({"name": name, "bytes": int(size or 0)})
if os.path.exists(os.path.join(t, "master")):
    m = rd("master")
    phase = ""
    for line in m.splitlines():
        if line.strip().startswith("Phase:"):
            phase = line.split(":", 1)[1].strip()
    out["ethercat"] = {"unit": rd("ethercat_unit").strip(), "phase": phase, "slaves": len([l for l in rd("slaves").splitlines() if l.strip()]), "master_raw": m[-600:]}
print(json.dumps(out))
ROSIE_FACTS_PY
`, "@TELEMETRY@", rtCoreConfigTelemetryDir, "@MASTER@", withMaster, "@ANCHORS_NOW@", rtCoreConfigAnchorsNow)
}

// cellDeployRelayFactsPy merges the servo facts with relay facts: the OLP's
// own session status, recent operator activity in its journal, and which stop
// dumps already have a copy in the dump directory whose sha256 is the one
// copy-dumps verified against the servo host and recorded in SHA256SUMS (a
// same-size file is not proof). recorded_sha256 lets the Mac count its own
// copy (copy-dumps --to mac) by the same hash.
const cellDeployRelayFactsPy = `import hashlib, json, os, sys
servo_path, status_path, activity_path, dump_dir, olp_unit, olp_dir = sys.argv[1:7]
facts = json.load(open(servo_path))
olp = {"unit": olp_unit, "deployment": olp_dir, "reachable": False}
try:
    o = json.load(open(status_path))
    s = o.get("session") or {}
    olp.update({"reachable": True, "available": o.get("available"), "session": s.get("state") or "", "held": bool(o.get("held"))})
except (OSError, ValueError):
    pass
try:
    olp["recent_activity"] = int(open(activity_path).read().strip() or 0)
except (OSError, ValueError):
    olp["recent_activity"] = -1
facts["olp"] = olp
sums = {}
try:
    for line in open(os.path.join(dump_dir, "SHA256SUMS")):
        h, _, n = line.rstrip("\n").partition("  ")
        if len(h) == 64 and n:
            sums[n] = h
except OSError:
    pass
def sha256(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()
for d in facts.get("dumps", []):
    p = os.path.join(dump_dir, d["name"])
    d["recorded_sha256"] = sums.get(d["name"], "")
    d["copied"] = bool(d["recorded_sha256"]) and os.path.isfile(p) and os.path.getsize(p) == d["bytes"] and sha256(p) == d["recorded_sha256"]
    if d["copied"]:
        d["where"] = "relay"
print("@@probe " + json.dumps(facts))
`

// cellDeployPrecheckScript is read-only on both hosts: servo facts, the OLP's
// session state and journal activity in the last quietSeconds, and the dump
// copies. The facts are also kept under scratch_dir/precheck for verify.
func cellDeployPrecheckScript(ctx cellDeployContext, quietSeconds int) string {
	return cellDeployRelayPrologue(ctx) + rtCoreConfigReplace(`S=@SCRATCH@/precheck; DUMPS=@DUMPS@; UNIT=@UNIT@; API=@API@
install -d -m 0700 "$S" || refuse "cannot create $S"
w=$(mktemp -d "$S/run.XXXXXX") || refuse "mktemp in $S failed"
wd=$(systemctl show "$UNIT" -p WorkingDirectory --value 2>/dev/null)
OLP_DIR=""; case "$wd" in */source/offline-programming/v1) OLP_DIR=${wd%/source/offline-programming/v1} ;; esac
step "precheck relay=$(hostname) olp=${OLP_DIR:-none}; reading the servo host (read-only)"
servos > "$w/servo.json" 2> "$w/servo.err" <<'ROSIE_SERVO_FACTS'
@FACTS@
ROSIE_SERVO_FACTS
python3 -c 'import json,sys; json.load(open(sys.argv[1]))' "$w/servo.json" 2> /dev/null || refuse "servo host facts unreadable: $(tail -n 3 "$w/servo.err" | tr '\n' ' ')"
curl -fsS -m 5 "$API/dense-execution/status" > "$w/olp-status.json" 2> /dev/null || : > "$w/olp-status.json"
sudo -n journalctl -u "$UNIT" --since "-@QUIET@ s" --no-pager -o cat 2> /dev/null | grep -cE @ACTIVITY@ > "$w/activity" || true
python3 - "$w/servo.json" "$w/olp-status.json" "$w/activity" "$DUMPS" "$UNIT" "$OLP_DIR" > "$w/probe.line" <<'ROSIE_MERGE_PY'
@MERGE@
ROSIE_MERGE_PY
cut -d' ' -f2- "$w/probe.line" > "$S/$(date -u +%Y%m%dT%H%M%SZ).json" && cut -d' ' -f2- "$w/probe.line" > "$S/latest.json"
cat "$w/probe.line"
rm -rf -- "$w"
`, "@SCRATCH@", shellQuote(ctx.Deploy.ScratchDir), "@DUMPS@", shellQuote(ctx.Deploy.DumpDir), "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit),
		"@API@", shellQuote("http://"+ctx.Deploy.OLP.Listen+"/api/offline-programming/v1"), "@QUIET@", strconv.Itoa(quietSeconds),
		"@ACTIVITY@", shellQuote(cellDeployOLPActivity), "@FACTS@", cellDeployServoFacts(false), "@MERGE@", cellDeployRelayFactsPy)
}

// cellDeployWaitIdleScript polls until the controller is disarmed with no
// enable or lease and the OLP journal shows no operator activity for
// quietSeconds; exit 0 idle, 3 not idle within limitSeconds. Read-only.
func cellDeployWaitIdleScript(ctx cellDeployContext, quietSeconds, limitSeconds, intervalSeconds int) string {
	return cellDeployRelayPrologue(ctx) + rtCoreConfigReplace(`UNIT=@UNIT@; QUIET=@QUIET@; LIMIT=@LIMIT@; EVERY=@EVERY@
start=$SECONDS
while :; do
  core=$(servo 'sudo -n /opt/rosie-rt-core/current/bin/rtctl status --socket /run/rosie-rt-core/control.sock --json' 2> /dev/null | python3 -c 'import json,sys
@BUSY@try:
    c = json.load(sys.stdin); c = c.get("data", c)["core"]
    print("%s %s %s %s" % ("busy" if rosie_busy(c) else "idle", c.get("armed"), c.get("axis_enable_mask"), int(bool(c.get("lease_valid")))))
except Exception:
    print("unknown")')
  act=$(sudo -n journalctl -u "$UNIT" --since "-$QUIET s" --no-pager -o cat 2> /dev/null | grep -cE @ACTIVITY@ || true)
  step "idle armed/enable/lease=${core:-unknown} olp_activity_last_${QUIET}s=${act:-?} waited=$((SECONDS - start))s"
  if [ "${core%% *}" = idle ] && [ "${act:-1}" = 0 ]; then printf '@@result %s\n' '{"status":"idle"}'; exit 0; fi
  if [ $((SECONDS - start)) -ge "$LIMIT" ]; then printf '@@result {"status":"busy","core":"%s","activity":"%s"}\n' "$core" "$act"; exit 3; fi
  sleep "$EVERY"
done
`, "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit), "@QUIET@", strconv.Itoa(quietSeconds), "@LIMIT@", strconv.Itoa(limitSeconds),
		"@EVERY@", strconv.Itoa(intervalSeconds), "@ACTIVITY@", shellQuote(cellDeployOLPActivity), "@BUSY@", rtCoreBusyPy)
}

// cellDeployThrottlePy copies stdin to stdout at no more than argv[1] bytes/s.
const cellDeployThrottlePy = `import sys, time
rate = int(sys.argv[1]); chunk = 256 * 1024; t0 = time.monotonic(); n = 0
out = sys.stdout.buffer
while True:
    b = sys.stdin.buffer.read(chunk)
    if not b:
        break
    out.write(b); n += len(b)
    ahead = n / rate - (time.monotonic() - t0)
    if ahead > 0:
        time.sleep(ahead)
`

// cellDeployCopyDumpsScript copies every telemetry dump off the servo host to
// the relay's dump_dir: `nice -n 19 ionice -c3 cat` on the servo host,
// throttled on the relay, then the servo host's sha256 (same priority) must
// equal the copy's, recorded in dump_dir/SHA256SUMS. A file already there is
// skipped only when its sha256 is the recorded one (a copy made by other
// means is copied again). It
// never decodes, deletes or moves anything on the servo host. An absent
// telemetry directory holds no dumps: rosie-rt-core creates it with its first
// Stop dump and a stop deletes it with /run/rosie-rt-core. On a Rosie
// 1430 cell (2026-10-07) a core started minutes earlier, and both units stopped after a
// failed start, each left copy-dumps refusing, and with it release. Only the
// servo host's explicit answer counts; an ssh or sudo failure still refuses.
func cellDeployCopyDumpsScript(ctx cellDeployContext) string {
	return cellDeployRelayPrologue(ctx) + rtCoreConfigReplace(`D=@DUMPS@; T=@TELEMETRY@; RATE=@RATE@
install -d -m 0750 "$D" || refuse "cannot create $D"
where=$(servo "sudo -n sh -c 'if [ -d $T ]; then echo present; else echo absent; fi'" 2> /dev/null)
case "$where" in
  present) listing=$(servo "sudo -n find $T -maxdepth 1 -type f -printf '%f %s\n'" 2> /dev/null) || refuse "cannot list $T on the servo host" ;;
  absent) listing=""; step "dumps none: $T does not exist on the servo host (created with the first Stop dump, removed by a stop)" ;;
  *) refuse "cannot list $T on the servo host" ;;
esac
step "dumps $(printf '%s' "$listing" | grep -c . || true) file(s) in $T; copying to $(hostname):$D at ${RATE} B/s (servo side nice 19, ionice idle)"
copied=0; present=0; out="["
while read -r f size; do
  [ -n "$f" ] || continue
  case "$f" in *[!A-Za-z0-9._-]*) refuse "unexpected dump file name '$f'" ;; esac
  have=0
  if [ -f "$D/$f" ] && [ "$(stat -c %s "$D/$f")" = "$size" ]; then
    sha=$(sha256sum < "$D/$f" | cut -d' ' -f1)
    grep -qxF "$sha  $f" "$D/SHA256SUMS" 2> /dev/null && have=1
  fi
  if [ "$have" = 1 ]; then
    present=$((present + 1))
    step "dumps already copied $f bytes=$size (sha256 recorded)"
  else
    t0=$SECONDS
    servo "sudo -n nice -n 19 ionice -c3 cat $T/$f" | python3 -c @THROTTLE@ "$RATE" > "$D/$f.part" || { rm -f "$D/$f.part"; refuse "copy of $f failed"; }
    remote=$(servo "sudo -n nice -n 19 ionice -c3 sha256sum $T/$f" | cut -d' ' -f1)
    sha=$(sha256sum < "$D/$f.part" | cut -d' ' -f1)
    [ -n "$remote" ] && [ "$remote" = "$sha" ] || { rm -f "$D/$f.part"; refuse "hash mismatch for $f (servo ${remote:-unreadable}, copy $sha); if the dump is still being written, re-run"; }
    mv -f "$D/$f.part" "$D/$f"
    printf '%s  %s\n' "$sha" "$f" >> "$D/SHA256SUMS"
    copied=$((copied + 1))
    step "dumps copied $f bytes=$(stat -c %s "$D/$f") sha256=${sha:0:16} in $((SECONDS - t0))s"
  fi
  out="$out{\"name\":\"$f\",\"bytes\":$(stat -c %s "$D/$f"),\"sha256\":\"$sha\"},"
done <<< "$listing"
out="${out%,}]"
printf '@@result {"status":"ok","dir":"%s","copied":%d,"present":%d,"files":%s}\n' "$D" "$copied" "$present" "$out"
`, "@DUMPS@", shellQuote(ctx.Deploy.DumpDir), "@TELEMETRY@", rtCoreConfigTelemetryDir,
		"@RATE@", strconv.FormatInt(ctx.Deploy.DumpCopyRate, 10), "@THROTTLE@", shellQuote(cellDeployThrottlePy))
}

// cellDeployVerifyServo runs on the servo host after a start: the startup SDO
// readbacks and fault/anchor/Home journal lines since rosie-rt-core last
// started, the control.env pin, the anchor files, and the distributed-clock
// lock (cell_deploy_dc_lock.go). Read-only.
//
// The DC part reads ESC 0x092C on each DC slave (DC_POS, empty: nothing is
// read) once rosie-rt-core has been active DC_SETTLE s, DC_ROUNDS times 1 s
// apart: one background subshell per round, detached from the session's
// output, reads the slaves in turn, each under `timeout`, so a wedged master
// leaves an empty sample (refused) rather than a hung verify. A register read is a datagram through the running
// master, not a mailbox transfer. The raw text goes back as it is; the Mac
// interprets it. Then it counts IgH's "Slave did not sync" kernel lines since
// the start.
const cellDeployVerifyServo = `set -uo pipefail
export LC_ALL=C
@ANCHORS_NOW@
DC_POS=@DC_POS@; DC_SETTLE=@DC_SETTLE@; DC_ROUNDS=@DC_ROUNDS@
since=$(date -d "$(systemctl show rosie-rt-core.service -p ActiveEnterTimestamp --value)" '+%Y-%m-%d %H:%M:%S' 2> /dev/null)
echo "since $since"
if [ -n "$since" ]; then
  j=$(sudo -n journalctl -u rosie-rt-core -u rosie-rt-control --since "$since" --no-pager -o cat 2> /dev/null)
  echo "readbacks_verified $(printf '%s\n' "$j" | grep -cE 'commanded=.*readback=.*verified=1' || true)"
  echo "readbacks_unverified $(printf '%s\n' "$j" | grep -cE 'commanded=.*readback=.*verified=0' || true)"
  printf '%s\n' "$j" | grep -E 'commanded=.*readback=.*verified=0' | head -n 20 | sed 's/^/unverified_line /'
  printf '%s\n' "$j" | grep -vE 'commanded=.*readback=.*verified=1' | grep -iE 'fault|anchor|home|error|warn|refus|fail' | head -n 40 | sed 's/^/journal_line /'
fi
ANCHOR_DIR=$(systemctl show rosie-rt-control.service -p Environment --value | tr ' ' '\n' | sed -n 's/^ROSIE_RT_ANCHOR_DIR=//p' | head -n 1)
[ -n "$ANCHOR_DIR" ] && anchors_now | sed 's/^/anchor_file /'
if [ -n "$DC_POS" ]; then
  if [ "$(systemctl show rosie-rt-core.service -p ActiveState --value)" != active ] || [ -z "$since" ]; then
    echo "dc_skipped rosie-rt-core is not active, so no reference clock is being distributed"
  else
    up=$(( $(date +%s) - $(date -d "$since" +%s) ))
    if [ "$up" -lt "$DC_SETTLE" ]; then
      echo "dc_settle_wait $((DC_SETTLE - up))"
      sleep $((DC_SETTLE - up))
      up=$DC_SETTLE
    fi
    echo "dc_core_up $up"
    dc=$(mktemp -d)
    for r in $(seq 1 "$DC_ROUNDS"); do
      ( for p in $DC_POS; do sudo -n timeout 3 ethercat reg_read -p"$p" -tuint32 0x092C > "$dc/$r-$p" 2>&1; done < /dev/null > /dev/null 2>&1 & )
      sleep 1
    done
    for r in $(seq 1 "$DC_ROUNDS"); do
      for p in $DC_POS; do echo "dc_sample $r $p $(head -c 160 "$dc/$r-$p" 2> /dev/null | tr '\n\t' '  ')"; done
    done
    rm -rf -- "$dc"
  fi
  if [ -n "$since" ] && k=$(sudo -n journalctl -k --since "$since" --no-pager -o cat 2> /dev/null); then
    k=$(printf '%s\n' "$k" | grep 'Slave did not sync')
    echo "dc_not_synced $(printf '%s' "$k" | grep -c . || true)"
    printf '%s\n' "$k" | grep . | head -n 6 | sed 's/^/dc_kernel_line /'
  else
    echo "dc_not_synced -1"
  fi
fi
`

// cellDeployVerifyServoScript renders cellDeployVerifyServo for ctx.
func cellDeployVerifyServoScript(ctx cellDeployContext) string {
	settle := 0
	if ctx.Deploy.DCLock != nil {
		settle = ctx.Deploy.DCLock.SettleS
	}
	return rtCoreConfigReplace(cellDeployVerifyServo, "@ANCHORS_NOW@", rtCoreConfigAnchorsNow, "@DC_POS@", shellQuote(cellDeployDCPositions(ctx)),
		"@DC_SETTLE@", strconv.Itoa(settle), "@DC_ROUNDS@", strconv.Itoa(cellDeployDCRounds))
}

// cellDeployVerifyScript waits (homeWaitSeconds at most) for home_valid_mask
// to settle, then reports the servo facts and the post-start journal lines.
func cellDeployVerifyScript(ctx cellDeployContext, homeWaitSeconds int) string {
	return cellDeployVerifyScriptFor(ctx, homeWaitSeconds, ctx.FullAxisMask())
}

// cellDeployVerifyScriptFor waits until home_valid_mask holds every bit of
// homeMask (restart: the Home that was valid before it stopped the cell).
func cellDeployVerifyScriptFor(ctx cellDeployContext, homeWaitSeconds int, homeMask uint64) string {
	return cellDeployRelayPrologue(ctx) + rtCoreConfigReplace(`S=@SCRATCH@/precheck; FULL=@FULL@; WAIT=@WAIT@
w=$(mktemp -d) || refuse "mktemp failed"
start=$SECONDS
step "phase home-wait"
while :; do
  m=$(servo 'sudo -n /opt/rosie-rt-core/current/bin/rtctl status --socket /run/rosie-rt-core/control.sock --json' 2> /dev/null | python3 -c 'import json,sys
try:
    c = json.load(sys.stdin); c = c.get("data", c)["core"]; print(c["home_valid_mask"])
except Exception:
    print("unknown")')
  case "$m" in ""|*[!0-9]*) have=0 ;; *) have=$(( (m & FULL) == FULL )) ;; esac
  if [ "$have" = 1 ] || [ $((SECONDS - start)) -ge "$WAIT" ]; then break; fi
  step "verify home_valid_mask=$m waiting for $FULL ($((SECONDS - start))s)"
  sleep 1
done
step "phase facts home_valid_mask=$m after $((SECONDS - start))s"
servos > "$w/servo.json" 2> "$w/servo.err" <<'ROSIE_SERVO_FACTS'
@FACTS@
ROSIE_SERVO_FACTS
servos > "$w/verify.txt" 2>&1 <<'ROSIE_SERVO_VERIFY'
@VERIFY@
ROSIE_SERVO_VERIFY
python3 - "$w/servo.json" "$w/verify.txt" "$S/latest.json" <<'ROSIE_VERIFY_PY' || refuse "servo host facts unreadable: $(tail -n 3 "$w/servo.err" | tr '\n' ' ')"
import json, sys
facts = json.load(open(sys.argv[1]))
v = {"readbacks_verified": 0, "readbacks_unverified": 0, "unverified_lines": [], "journal_lines": [], "anchor_files": [], "since": ""}
dc = {"skipped": "", "settle_wait_s": 0, "core_up_s": -1, "samples": [], "not_synced": -1, "kernel_lines": []}
def whole(text, default):
    try:
        return int(text.strip())
    except ValueError:
        return default
for line in open(sys.argv[2]).read().splitlines():
    key, _, rest = line.partition(" ")
    if key in ("readbacks_verified", "readbacks_unverified"):
        v[key] = int(rest or 0)
    elif key == "since":
        v["since"] = rest
    elif key in ("unverified_line", "journal_line", "anchor_file"):
        v[key + "s"].append(rest)
    elif key == "dc_skipped":
        dc["skipped"] = rest.strip() or "not read"
    elif key in ("dc_settle_wait", "dc_core_up", "dc_not_synced"):
        dc[{"dc_settle_wait": "settle_wait_s", "dc_core_up": "core_up_s", "dc_not_synced": "not_synced"}[key]] = whole(rest, -1)
    elif key == "dc_kernel_line":
        dc["kernel_lines"].append(rest)
    elif key == "dc_sample":
        r, _, rest = rest.partition(" ")
        p, _, raw = rest.partition(" ")
        dc["samples"].append({"round": whole(r, 0), "position": whole(p, -1), "raw": raw.strip()})
if dc["skipped"] or dc["samples"]:
    v["dc"] = dc
facts["verify"] = v
try:
    facts["precheck_anchor_files"] = json.load(open(sys.argv[3])).get("anchor", {}).get("files", [])
except (OSError, ValueError):
    facts["precheck_anchor_files"] = None
print("@@probe " + json.dumps(facts))
ROSIE_VERIFY_PY
step "phase end"
rm -rf -- "$w"
`, "@SCRATCH@", shellQuote(ctx.Deploy.ScratchDir), "@FULL@", strconv.FormatUint(homeMask, 10), "@WAIT@", strconv.Itoa(homeWaitSeconds),
		"@FACTS@", cellDeployServoFacts(false), "@VERIFY@", cellDeployVerifyServoScript(ctx))
}

// cellDeployFactsScript is a read-only relay session that prints the servo
// facts (with the EtherCAT master when the units are inactive) and the dump
// copies; resume-after-reboot and reboot evaluate it before acting.
func cellDeployFactsScript(ctx cellDeployContext, master bool) string {
	return cellDeployRelayPrologue(ctx) + rtCoreConfigReplace(`DUMPS=@DUMPS@; UNIT=@UNIT@
w=$(mktemp -d) || refuse "mktemp failed"
servos > "$w/servo.json" 2> "$w/servo.err" <<'ROSIE_SERVO_FACTS'
@FACTS@
ROSIE_SERVO_FACTS
python3 -c 'import json,sys; json.load(open(sys.argv[1]))' "$w/servo.json" 2> /dev/null || refuse "servo host facts unreadable: $(tail -n 3 "$w/servo.err" | tr '\n' ' ')"
: > "$w/none"
python3 - "$w/servo.json" "$w/none" "$w/none" "$DUMPS" "$UNIT" "" <<'ROSIE_MERGE_PY'
@MERGE@
ROSIE_MERGE_PY
rm -rf -- "$w"
`, "@DUMPS@", shellQuote(ctx.Deploy.DumpDir), "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit), "@FACTS@", cellDeployServoFacts(master), "@MERGE@", cellDeployRelayFactsPy)
}

// cellDeployRebootScript reboots the servo host. It is rendered only for
// `realtime-core reboot --owner-approved` after the Mac evaluated the facts,
// and re-checks on the servo host just before: no armed/enabled/leased
// controller and no stop dump without a copy (a reboot empties /run). It
// waits for a new boot time and prints the post-boot facts.
func cellDeployRebootScript(ctx cellDeployContext, reason string) string {
	return cellDeployRelayPrologue(ctx) + rtCoreConfigReplace(`D=@DUMPS@; T=@TELEMETRY@; REASON=@REASON@
trap '' HUP PIPE
before=$(servo_once 'date -u -d "$(uptime -s)" +%s') || refuse "servo host unreachable before the reboot"
st=$(servo_once 'if [ "$(systemctl show rosie-rt-control.service -p ActiveState --value)" = active ]; then sudo -n /opt/rosie-rt-core/current/bin/rtctl status --socket /run/rosie-rt-core/control.sock --json; fi' 2> /dev/null)
if [ -n "$st" ]; then
  printf '%s' "$st" | python3 -c 'import json,sys
@BUSY@c = json.load(sys.stdin); c = c.get("data", c)["core"]
sys.exit(1 if rosie_busy(c) else 0)' || refuse "controller armed, enabled or leased: disarm and release first"
fi
while read -r f size; do
  [ -n "$f" ] || continue
  [ -f "$D/$f" ] && [ "$(stat -c %s "$D/$f")" = "$size" ] || refuse "stop dump $f has no copy in $D; run realtime-core copy-dumps first (a reboot empties /run)"
done <<< "$(servo_once "sudo -n find $T -maxdepth 1 -type f -printf '%f %s\n'" 2> /dev/null)"
step "reboot OWNER-APPROVED reboot of $SERVO (reason: $REASON); boot epoch before $before"
servo_once 'sudo -n systemctl reboot' || true
now=""
for i in $(seq 1 120); do
  sleep 5
  now=$(servo_once 'date -u -d "$(uptime -s)" +%s' 2> /dev/null) || { [ $((i % 6)) = 0 ] && step "reboot waiting for the servo host $((i * 5))s"; continue; }
  if [ -n "$now" ] && [ "$now" != "$before" ]; then step "reboot servo host back after about $((i * 5))s (boot epoch $now)"; break; fi
done
[ -n "$now" ] && [ "$now" != "$before" ] || { printf '@@result {"status":"not_back","before":%s}\n' "$before"; exit 4; }
sleep 15
printf '@@result {"status":"rebooted","before":%s,"after":%s}\n' "$before" "$now"
`, "@DUMPS@", shellQuote(ctx.Deploy.DumpDir), "@TELEMETRY@", rtCoreConfigTelemetryDir, "@REASON@", shellQuote(reason), "@BUSY@", rtCoreBusyPy)
}

// cellDeployStartScript runs the existing `realtime-core start` servo script
// through the relay (rtCoreStartCommand: refuses unless both units are
// installed and stopped with the cell pin; stops both again on any failure).
func cellDeployStartScript(ctx cellDeployContext, start string) string {
	return cellDeployRelayPrologue(ctx) + "trap '' HUP PIPE\nmaint_phase 'starting the controller after the servo host reboot (stays disarmed)'\nstep \"start servo host $SERVO\"\nservos <<'ROSIE_RT_CORE_START'\n" + start + "\nROSIE_RT_CORE_START\nrc=$?\nstep \"start exit $rc\"\nexit $rc\n"
}

// --- build -----------------------------------------------------------------

// cellDeployPreviousShell sets P to the deployment the OLP unit runs (its
// WorkingDirectory is <deployment>/source/offline-programming/v1) unless an
// explicit previous tree was given.
func cellDeployPreviousShell(ctx cellDeployContext, previous string) string {
	return rtCoreConfigReplace(`P=@PREV@
if [ -z "$P" ]; then
  wd=$(systemctl show @UNIT@ -p WorkingDirectory --value 2> /dev/null)
  case "$wd" in */source/offline-programming/v1) P=${wd%/source/offline-programming/v1} ;; esac
fi
`, "@PREV@", shellQuote(previous), "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit))
}

// cellDeployBuildProbe is read-only: which tree the OLP runs (the build's
// git base and planner env donor), its commit, and the target tree's state.
func cellDeployBuildProbe(ctx cellDeployContext, commit, previous string) string {
	return "set -uo pipefail\nexport LC_ALL=C\n" + cellDeployPreviousShell(ctx, previous) + rtCoreConfigReplace(`D=@D@; IGH=@IGH@
export PATH=@PATH@:$PATH
python3 - "$P" "$D" "$IGH" @SHA@ <<'ROSIE_BUILD_PROBE_PY'
import json, os, shutil, subprocess, sys
p, d, igh, sha = sys.argv[1:5]
def git(repo, *args):
    try:
        return subprocess.run(["git", "-C", repo] + list(args), capture_output=True, text=True, timeout=30).stdout.strip()
    except Exception:
        return ""
def ver(*cmd):
    try:
        return subprocess.run(list(cmd), capture_output=True, text=True, timeout=30).stdout.strip().splitlines()[0]
    except Exception:
        return ""
out = {"previous": p, "previous_head": git(p + "/source", "rev-parse", "HEAD") if p else "",
       "deployment": d, "deployment_exists": os.path.isdir(d), "deployment_complete": os.path.exists(d + "/BUILD_COMPLETE"),
       "deployment_head": git(d + "/source", "rev-parse", "HEAD") if os.path.isdir(d + "/source") else "",
       "igh_pkgconfig": os.path.isfile(igh + "/igh-prefix/lib/pkgconfig/libethercat.pc"),
       "tools": {"go": ver("go", "version"), "node": ver("node", "--version"), "pixi": ver("pixi", "--version"), "pkg-config": ver("pkg-config", "--version"), "make": ver("make", "--version")}}
try:
    st = os.statvfs(os.path.dirname(d))
    out["free_mib"] = st.f_bavail * st.f_frsize // (1024 * 1024)
except OSError:
    out["free_mib"] = -1
import glob, hashlib
packages = sorted(glob.glob(d + "/runtime-package.components/rosie-rt-core-*.tar.gz"))
if out["deployment_complete"] and len(packages) == 1:
    h = hashlib.sha256()
    with open(packages[0], "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    out["package"], out["package_sha256"] = packages[0], h.hexdigest()
print("@@probe " + json.dumps(out))
ROSIE_BUILD_PROBE_PY
`, "@D@", shellQuote(ctx.DeploymentDir(commit)), "@IGH@", shellQuote(ctx.Deploy.IgHDepsDir), "@PATH@", shellQuote(strings.Join(ctx.Deploy.Path, ":")), "@SHA@", shellQuote(commit))
}

// cellDeployUIStub is the API-only page the relay's OLP serves; the operator
// runs the UI on their own machine (8e3c9557's ui-stub, byte for byte).
const cellDeployUIStub = `<!DOCTYPE html><meta charset="utf-8"><title>Offline Programming V1</title><p>API only. The UI runs on the operator machine (Vite :5189).</p>` + "\n"

type cellDeployBuildPlan struct {
	Commit            string
	Ref               string // bundle ref the commit is reachable from
	Previous          string // "" = the OLP unit's tree
	BundlePath        string // on the relay
	BundleSHA256      string
	InstallPlannerEnv bool
	AllowDrift        bool
	// Locked: the build runs under the cell lock (cell_lock.go) at normal
	// priority with every core and refuses unless the lock is held; unset
	// (--low-priority), at low CPU and I/O priority with build_jobs and no lock.
	Locked bool
	// Clean deletes the persistent tree first (release_build.go).
	Clean bool
	// FetchPackage: the package is for a full deploy, so it is built from
	// the commit, never copied from the previous tree.
	FetchPackage bool
}

// cellDeployGateTSH is the make SHELL for gate steps: it runs each recipe line
// with bash, unchanged, and appends "<start_ns> <end_ns> <rc> <line>" to
// $ROSIE_TSH_LOG.
const cellDeployGateTSH = `#!/bin/bash
t0=$(date +%s%N)
/bin/bash "$@"
rc=$?
if [ -n "${ROSIE_TSH_LOG:-}" ]; then printf '%s %s %s %s\n' "$t0" "$(date +%s%N)" "$rc" "$(printf '%s' "${@: -1}" | tr '\n\t' '  ' | cut -c1-200)" >> "$ROSIE_TSH_LOG"; fi
exit $rc`

// --- OLP -------------------------------------------------------------------

// cellDeployOLPUnit renders the relay's OLP systemd unit for one deployment
// tree. Every value comes from the cell manifest; the layout under the tree
// (bin/, motion-bin/, ui-stub/, private/, olp-state/, source/) is the build's.
func cellDeployOLPUnit(ctx cellDeployContext, deployment, release string) string {
	o := ctx.Deploy.OLP
	wants := "network-online.target"
	if o.ControlTunnelUnit != "" {
		wants += " " + o.ControlTunnelUnit
	}
	wants += " " + o.PlannerUnit
	d := deployment
	discovery := []string{}
	if len(o.DiscoveryInterfaces) > 0 {
		discovery = append(discovery, "Environment=OFFLINE_PROGRAMMING_DISCOVERY_INTERFACES="+strings.Join(o.DiscoveryInterfaces, ","))
	}
	if o.DiscoveryTailnet != nil {
		discovery = append(discovery, "Environment=OFFLINE_PROGRAMMING_DISCOVERY_TAILNET_PORT="+strconv.Itoa(o.DiscoveryTailnet.ListenPort))
	}
	if o.TestPrograms {
		discovery = append(discovery, "Environment=OFFLINE_PROGRAMMING_TEST_PROGRAMS=1")
	}
	if origin := ctx.Deploy.CameraOrigin(); origin != "" {
		discovery = append(discovery, "Environment=OFFLINE_PROGRAMMING_CELL_CAMERA_ORIGIN="+origin)
	}
	if c := ctx.Deploy.Camera; c != nil && c.Video != nil {
		discovery = append(discovery, "Environment=OFFLINE_PROGRAMMING_CELL_VIDEO_ORIGIN="+c.VideoOrigin())
	}
	if extra := ctx.Deploy.ExtraCamerasEnv(); extra != "" {
		discovery = append(discovery, "Environment=OFFLINE_PROGRAMMING_CELL_EXTRA_CAMERAS="+extra)
	}
	discovery = append(discovery, cellDeployProgramStoreEnv(ctx)...)
	return strings.Join(slices.Concat([]string{
		"[Unit]",
		"Description=" + strings.Replace(o.Description, cellDeployReleaseToken, release, 1),
		"After=" + wants,
		"Wants=" + wants,
		"[Service]",
		"User=" + ctx.RelayUser,
		"WorkingDirectory=" + d + "/source/offline-programming/v1",
		"Environment=HOME=" + ctx.Deploy.Home,
		"Environment=PATH=" + strings.Join(ctx.Deploy.Path, ":"),
		"Environment=OFFLINE_PROGRAMMING_CELLS=" + d + "/private/olp-cells.json",
		"Environment=OFFLINE_PROGRAMMING_EXECUTION_BACKEND=rt_core",
		"Environment=OFFLINE_PROGRAMMING_JOG_BACKEND=rt_core",
		"Environment=OFFLINE_PROGRAMMING_EXECUTION_STATE_DIR=" + d + "/olp-state",
		"Environment=OFFLINE_PROGRAMMING_WELD_PLANNER_MOTION_ORIGIN=" + o.PlannerOrigin,
		"Environment=SEAM_WORKER_PYTHON=" + d + "/source/weld_planner/v1/.pixi/envs/" + o.PlannerPixiEnv + "/bin/python",
		// The build's checked CadQuery env, run directly: the worker's
		// fallback, `pixi run` with the relay's pixi, cannot use the lock.
		"Environment=CADQUERY_TOPOLOGY_PYTHON=" + cellDeployCadQueryPython(d),
		"Environment=OLP_CARTESIAN_RESOLVER=" + d + "/motion-bin/robot-v4-cartesiand",
		"Environment=OLP_CARTESIAN_RESOLVER_REPO=" + d + "/source",
	}, discovery, []string{
		"ExecStart=" + d + "/bin/offline-programming serve --listen " + o.Listen + " --dist " + d + "/ui-stub --enable-local-simulator=false",
		"Restart=on-failure",
		"RestartSec=3",
		"UMask=0077",
		"[Install]",
		"WantedBy=multi-user.target",
	}), "\n") + "\n"
}

// cellDeployOLPIdlePy refuses while an operator uses the OLP: a program
// playing or paused, a jog hold, or a controller armed, enabled or leased.
const cellDeployOLPIdlePy = `import json, sys
` + rtCoreBusyPy + `try:
    o = json.load(open(sys.argv[1]))
except (OSError, ValueError):
    print("olp status unavailable (server down): nothing to interrupt")
    sys.exit(0)
s = o.get("session") or {}
c = ((o.get("rt_core") or {}).get("status") or {}).get("core") or {}
busy = []
if s.get("state") in ("playing", "paused"):
    busy.append("program " + s["state"])
if o.get("held"):
    busy.append("jog held")
if c and rosie_busy(c):
    busy.append("controller armed=%s enable=%s lease=%s" % (c.get("armed"), c.get("axis_enable_mask"), c.get("lease_valid")))
print("olp %s session=%s armed=%s lease=%s" % ("BUSY " + ", ".join(busy) if busy else "idle", s.get("state") or "none", c.get("armed"), c.get("lease_valid")))
sys.exit(1 if busy else 0)
`

// cellDeployOLPCheckShell defines, for install and rollback: olp_check (health
// within 60 s, the unit's environment, /targets listing the cell) and
// olp_select (a Describe-only target select and the OLP's own status). The OLP
// refuses target selection while a maintenance notice is up, so olp_select
// first clears this run's notice; with --keep-maintenance, or inside an outer
// cell lock whose notice stays up, it skips the select.
var cellDeployOLPCheckShell = strings.Replace(cellDeployOLPCheckShellTemplate, "@JOG_STATE_PY@", cellDeployOLPJogStatePy, 1)

const cellDeployOLPCheckShellTemplate = `olp_check(){ # $1 deployment the unit must run; returns non-zero with OLP_FAIL set
  local ok=0 i pid
  for i in $(seq 1 60); do
    if curl -fsS -m 2 -o /dev/null "$API/health" 2> /dev/null; then ok=1; break; fi
    [ $((i % 5)) = 0 ] && step "olp waiting for health ${i}s"
    sleep 1
  done
  [ "$ok" = 1 ] || { OLP_FAIL="health not ok within 60s"; return 1; }
  pid=$(systemctl show -p MainPID --value "$UNIT")
  sudo -n cat "/proc/$pid/environ" 2> /dev/null | tr '\0' '\n' | grep -E '^(OLP_CARTESIAN_RESOLVER|OFFLINE_PROGRAMMING_CELLS)=' > "$W/env.txt"
  grep -qx "OLP_CARTESIAN_RESOLVER=$1/motion-bin/robot-v4-cartesiand" "$W/env.txt" && grep -qx "OFFLINE_PROGRAMMING_CELLS=$1/private/olp-cells.json" "$W/env.txt" \
    || { OLP_FAIL="running OLP (pid $pid) does not use $1 ($(tr '\n' ' ' < "$W/env.txt"))"; return 1; }
  curl -fsS -m 5 "$API/targets" > "$1/targets-post.json" || { OLP_FAIL="/targets unavailable"; return 1; }
  python3 -c 'import json,sys
views = [v for v in json.load(open(sys.argv[1])).get("targets", []) if v.get("cell_id") == sys.argv[2]]
assert views, "cell not listed"
print("olp lists %s backend=%s reason=%s" % (sys.argv[2], views[0].get("backend", ""), views[0].get("reason", "") or "none"))' "$1/targets-post.json" "$CELL" || { OLP_FAIL="/targets does not list $CELL"; return 1; }
  return 0
}
olp_select(){ # $1 deployment; returns non-zero with OLP_FAIL set
  if [ "${MAINT_KEEP:-0}" = 1 ]; then
    step "olp select skipped: the maintenance notice stays up (--keep-maintenance) and the OLP refuses selection under it; select the cell after it clears"
    return 0
  fi
  if [ -n "${MAINT_LOCK_INHERIT:-}" ]; then
    step "olp select skipped: the notice of the cell lock $MAINT_LOCK_INHERIT stays up until that run ends and the OLP refuses selection under it (realtime-core release selects after the lock)"
    return 0
  fi
  if command -v maint_clear > /dev/null 2>&1; then maint_clear; fi
  olp_select_now "$1"
}
olp_select_now(){ # $1 deployment: Describe-only select, then the OLP's own view once the controller description reached it
  local code
  code=$(curl -sS -m 30 -X POST -H 'Content-Type: application/json' -d "{\"cell_id\":\"$CELL\",\"model_id\":\"$MODEL\"}" -o "$1/select-post.json" -w '%{http_code}' "$API/targets/select")
  [ "$code" = 200 ] || { OLP_FAIL="target select (Describe only) http $code: $(head -c 300 "$1/select-post.json")"; return 1; }
  step "olp select $CELL/$MODEL http 200 (Describe only; no lease, no motion)"
  python3 - "$API" "$1/jog-state-post.json" <<'ROSIE_JOG_STATE_PY'
@JOG_STATE_PY@
ROSIE_JOG_STATE_PY
  curl -fsS -m 5 "$API/dense-execution/status" 2> /dev/null | python3 -c 'import json,sys
o = json.load(sys.stdin); s = o.get("session") or {}; c = ((o.get("rt_core") or {}).get("status") or {}).get("core") or {}
print("olp status", {"available": o.get("available"), "session": s.get("state"), "armed": c.get("armed"), "enable": c.get("axis_enable_mask"), "lease": c.get("lease_valid"), "home": c.get("home_valid_mask"), "faults": (c.get("safety_fault_mask"), c.get("execution_fault_reasons"))})' || true
  # The OLP's own startup line about a notice is not a refusal of this install.
  sudo -n journalctl -u "$UNIT" --since "-60 s" --no-pager -o cat 2> /dev/null | grep -iE "refus|error|mismatch" | grep -v "maintenance: notice" | head -n 10 | sed 's/^/  journal | /'
  return 0
}
`

// cellDeployOLPJogStatePy reads the OLP's jog state once the controller's
// Describe reached it: right after a select fraction_updates reads false for a
// few seconds (DEVLOG 2026-10-07 16:50), which is no finding. It waits up to
// 15 s for the Describe and for fraction_updates to agree with its path_jog.
const cellDeployOLPJogStatePy = `import json, sys, time, urllib.request
api, out = sys.argv[1], sys.argv[2]
def get(p):
    try:
        with urllib.request.urlopen(api + p, timeout=3) as r:
            return json.load(r)
    except Exception:
        return None
t0, d, s = time.monotonic(), {}, {}
while time.monotonic() - t0 < 15:
    d = ((get("/dense-execution/status") or {}).get("rt_core") or {}).get("description") or {}
    if d.get("configuration_sha256"):
        s = get("/dense-execution/jog/state") or {}
        if bool(s.get("fraction_updates")) == bool(d.get("path_jog")):
            json.dump(s, open(out, "w"))
            print("olp jog state fraction_updates=%s (Describe path_jog=%s; %.1fs after select)" % (s.get("fraction_updates"), bool(d.get("path_jog")), time.monotonic() - t0))
            sys.exit(0)
    time.sleep(0.5)
if d.get("configuration_sha256"):
    print("olp jog state fraction_updates=%s but Describe path_jog=%s after 15 s: check the OLP" % (s.get("fraction_updates"), bool(d.get("path_jog"))))
else:
    print("olp jog state not read: the controller Describe did not reach the OLP within 15 s (later: GET %s/dense-execution/jog/state)" % api)`

type cellDeployOLPPlan struct {
	Deployment         string
	Previous           string // "" = the unit's current tree
	ExpectCapabilities string
	PairMigration      bool
}

// cellDeployOLPInstallScript repoints the relay's OLP unit at a built tree:
// refuse unless the tree is complete, its build recorded a passing CadQuery
// check (cell_deploy_cadquery.go), planner/tunnel are active and the OLP is
// idle; read the live Describe through the cell's execution route with the
// current tree's client credentials; copy the PKI and OLP state; write the
// catalogue pinned to that Describe; back up the unit; install the rendered
// unit, restart, and check health, environment (the CadQuery python
// included), /targets and select. A failed
// check reinstalls the backup (exit 5). It touches no controller.
func cellDeployOLPInstallScript(ctx cellDeployContext, plan cellDeployOLPPlan) string {
	release := strings.TrimPrefix(path.Base(plan.Deployment), ctx.Deploy.DeploymentPrefix)
	unit := cellDeployOLPUnit(ctx, plan.Deployment, release)
	clientPKISelection := ""
	if plan.PairMigration {
		clientPKISelection = `[ "$BIND_PAIR" != "$PAIR" ] || refuse "pair migration requested but the OLP already names the manifest pair"
CLIENT_STAGE=@CLIENT_STAGE@
set -- "$CLIENT_STAGE"/clients/*.pem
[ "$#" = 2 ] || refuse "staged client PKI needs exactly one certificate and one key"
CA="$CLIENT_STAGE/ca.pem"; CERT=""; KEY=""
for f in "$@"; do case "$f" in *-key.pem) KEY="$f" ;; *) CERT="$f" ;; esac; done
for f in "$CA" "$CERT" "$KEY"; do [ -s "$f" ] && [ ! -L "$f" ] || refuse "staged client PKI file missing or a symlink: $f"; done
openssl x509 -in "$CERT" -checkend 86400 -noout || refuse "staged client certificate expires soon"
openssl verify -CAfile "$CA" "$CERT" || refuse "staged client certificate has another CA"
openssl x509 -in "$CERT" -noout -ext subjectAltName | grep -Fq @PAIR_SAN@ || refuse "staged client certificate has another pair"
cert_pub=$(openssl x509 -in "$CERT" -pubkey -noout | sha256sum | cut -d' ' -f1)
key_pub=$(openssl pkey -in "$KEY" -pubout | sha256sum | cut -d' ' -f1)
[ "$cert_pub" = "$key_pub" ] || refuse "staged client key does not match certificate"
`
	} else {
		clientPKISelection = `[ "$BIND_PAIR" = "$PAIR" ] || refuse "OLP credential pair differs from the manifest pair; use an explicit paired migration"
`
	}
	return cellDeployRelayPrologue(ctx) + "trap '' HUP PIPE\n" + cellDeployPreviousShell(ctx, plan.Previous) + rtCoreConfigReplace(`CELL=@CELL@; D=@D@; UNIT=@UNIT@; UNIT_PATH=/etc/systemd/system/@UNIT_NAME@
API=@API@; EXEC=@EXEC@; PIN=@PIN@; AXES=@AXES@; MODEL=@MODEL@; ENV=@ENV@; PREFIX=@PREFIX@; CAPS=@CAPS@; PAIR=@PAIR@; REV=@REV@
PLANNER=@PLANNER@; TUNNEL=@TUNNEL@; PORIGIN=@PORIGIN@; PSTORE=@PSTORE@; CQPY=@CQPY@
W="$D/olp-install"
`+cellDeployOLPCheckShell+cellDeployPlannerCheckShell+cellDeployCadQueryCheckShell+`[ -n "$P" ] || refuse "no previous tree: $UNIT has no <deployment>/source/offline-programming/v1 WorkingDirectory; pass --previous <dir holding private/olp-cells.json, its client PKI and olp-state/>"
[ "$P" != "$D" ] || refuse "$D is already the installed tree"
for f in BUILD_COMPLETE bin/offline-programming motion-bin/robot-v4-cartesiand ui-stub/index.html "source/weld_planner/v1/.pixi/envs/$ENV/bin/python"; do
  [ -e "$D/$f" ] || refuse "$D/$f missing: run realtime-core build first"
done
`+cellDeployCadQueryInstallRefusalShell+`[ ! -e "$D/olp-state" ] || refuse "$D/olp-state exists: this tree was installed before (olp rollback leaves it); move it aside by hand to reinstall"
[ -f "$P/private/olp-cells.json" ] || refuse "$P/private/olp-cells.json missing"
[ -f "$UNIT_PATH" ] || refuse "$UNIT_PATH missing: the first OLP unit for a cell is provisioned by hand"
for u in $TUNNEL $PLANNER; do [ "$(systemctl is-active "$u")" = active ] || refuse "$u is not active"; done
install -d -m 0700 "$W" || refuse "cannot create $W"
maint_phase "installing OLP $(basename "$D")"
step "olp install $D (previous $P) relay=$(hostname)"
curl -fsS -m 5 "$API/dense-execution/status" > "$W/status-pre.json" 2> /dev/null || : > "$W/status-pre.json"
python3 -c @IDLE_PY@ "$W/status-pre.json" || refuse "the OLP is in use: finish, Stop, disarm and release first"
python3 - "$P/private/olp-cells.json" "$CELL" "$EXEC" > "$W/pki" <<'ROSIE_PKI_PY' || refuse "previous catalogue has no single execution entry for $CELL (or, failing that, for $EXEC) with ca_file, certificate_file and key_file"
import json, sys
everything = json.load(open(sys.argv[1])).get("cells", [])
cells = [c for c in everything if c.get("cell_id") == sys.argv[2]]
if not cells:
    cells = [c for c in everything if (c.get("execution") or {}).get("address") == sys.argv[3]]
assert len(cells) == 1
x = cells[0]["execution"]
print(x["ca_file"]); print(x["certificate_file"]); print(x["key_file"]); print(x["binding"]["pair_id"])
ROSIE_PKI_PY
{ read -r CA; read -r CERT; read -r KEY; read -r BIND_PAIR; } < "$W/pki"
@CLIENT_PKI_SELECTION@
curl -fsS -m 10 --cacert "$CA" --cert "$CERT" --key "$KEY" "$EXEC/v1/describe" > "$D/describe-live.json" || refuse "live Describe through $EXEC failed"
python3 - "$D/describe-live.json" "$PIN" "$AXES" "$CAPS" <<'ROSIE_DESCRIBE_PY' || refuse "live Describe does not match the cell (above)"
import json, sys
e = json.load(open(sys.argv[1]))
assert e.get("schema") == "rosie.rt-control.response.v1", e.get("schema")
d = e["data"]
assert d["configuration_sha256"] == sys.argv[2], "configuration %s, cell pin %s" % (d["configuration_sha256"], sys.argv[2])
assert d["backend"] == "ethercat" and len(d["axes"]) == int(sys.argv[3]), (d["backend"], len(d["axes"]))
if sys.argv[4]:
    assert d["capabilities_digest"] == sys.argv[4], "capabilities %s, expected %s" % (d["capabilities_digest"], sys.argv[4])
print("live Describe configuration=%s machine=%s deployment=%s protocol=%s.%s capabilities=%s path_jog=%s" % (d["configuration_sha256"], d.get("machine_sha256"),
      d.get("deployment_sha256"), d.get("protocol_major"), d.get("protocol_minor"), d["capabilities_digest"], bool(d.get("path_jog"))))
ROSIE_DESCRIBE_PY

# ---- stage the tree: client PKI, OLP state, catalogue pinned to the live Describe ----
# A refusal before the unit swap removes what was staged, so a re-run starts clean.
unstage(){ rm -rf -- "$D/private" "$D/olp-state"; refuse "$@"; }
install -d -m 0700 "$D/private" || unstage "cannot create $D/private"
for f in "$CA" "$CERT" "$KEY"; do cp -p "$f" "$D/private/$(basename "$f")" || unstage "copy $f"; done
if [ -d "$P/olp-state" ]; then cp -a "$P/olp-state" "$D/olp-state" || unstage "copy olp-state"; else install -d -m 0700 "$D/olp-state"; fi
python3 - "$P" "$D" "$CELL" "$EXEC" "$MODEL" "$PAIR" "$REV" "$CA" "$CERT" "$KEY" <<'ROSIE_CATALOGUE_PY' || unstage "catalogue update refused (above)"
import json, os, pathlib, sys
p, d, cell, address, model, pair, revision, ca, cert, key = sys.argv[1:11]
envelope = json.load(open(os.path.join(d, "describe-live.json")))["data"]
catalogue = json.load(open(os.path.join(p, "private/olp-cells.json")))
matches = [c for c in catalogue.get("cells", []) if c.get("cell_id") == cell]
# A bench whose servo host was last released under another model binding (a
# bench ran a Rosie 1430 H-frame binding until 2026-10-31) has no entry for this cell.
# The single entry that reaches the same controller address carries the pair's PKI;
# adopt it under this cell and model instead of refusing, and say so.
if not matches:
    matches = [c for c in catalogue.get("cells", []) if (c.get("execution") or {}).get("address") == address]
    assert len(matches) == 1, "catalogue has no entry for %s and %d for %s" % (cell, len(matches), address)
    print("CATALOGUE_ENTRY_ADOPTED from=%s models=%s to=%s models=%s address=%s" % (matches[0].get("cell_id"), matches[0].get("models"), cell, [model], address))
    matches[0]["cell_id"], matches[0]["models"] = cell, [model]
assert len(matches) == 1, "catalogue has %d entries for %s" % (len(matches), cell)
c = matches[0]
assert c.get("models") == [model], "catalogue models %s, machine robot %s" % (c.get("models"), model)
x = c["execution"]
assert x.get("address") == address, "catalogue address %s, cell remote_listen %s" % (x.get("address"), address)
previous = dict(configuration=x["binding"]["configuration_sha256"], capabilities=x.get("capabilities_digest"))
for field, source in (("ca_file", ca), ("certificate_file", cert), ("key_file", key)):
    x[field] = os.path.join(d, "private", os.path.basename(source))
    assert pathlib.Path(source).is_file(), source
    assert pathlib.Path(x[field]).is_file(), x[field]
x["binding"]["pair_id"] = pair
x["binding"]["revision"] = int(revision)
x["binding"]["configuration_sha256"] = envelope["configuration_sha256"]
x["binding"]["machine_sha256"] = envelope["machine_sha256"]
x["deployment_sha256"] = envelope["deployment_sha256"]
x["capabilities_digest"] = envelope["capabilities_digest"]
target = os.path.join(d, "private/olp-cells.json")
with open(target + ".tmp", "w") as f:
    f.write(json.dumps(catalogue, indent=2) + "\n")
os.chmod(target + ".tmp", 0o600)
os.replace(target + ".tmp", target)
print("LIVE_CATALOGUE_IDENTITY_VERIFIED configuration=%s machine=%s deployment=%s capabilities=%s (previous %s)" % (
    envelope["configuration_sha256"], envelope["machine_sha256"], envelope["deployment_sha256"], envelope["capabilities_digest"], previous))
ROSIE_CATALOGUE_PY
pb=$(basename "$P"); BACKUP="$D/@UNIT_NAME@.bak-${pb#"$PREFIX"}"
`+cellDeployProgramStorePrepShell+`
# ---- planner (cell_deploy.olp.planner): swapped and checked before the OLP ----
@PLANNER_INSTALL@
cp -p "$UNIT_PATH" "$BACKUP" || unstage "cannot back up $UNIT_PATH"
printf '%s' @UNIT_B64@ | base64 -d > "$D/@UNIT_NAME@.new" || unstage "cannot write the new unit"
step "olp unit backup $BACKUP; new unit $D/@UNIT_NAME@.new"

# ---- swap; from here a failed check reinstalls the backup ----
restore(){
  step "olp RESTORE: $1"
  planner_reinstate
  if ! { sudo -n install -m 644 "$BACKUP" "$UNIT_PATH" && sudo -n systemctl daemon-reload && sudo -n systemctl restart "$UNIT"; }; then
    sudo -n systemctl stop "$UNIT" || true
    printf '@@result {"status":"failed","reason":"OLP restore failed; unit stopped","backup":"%s"}\n' "$BACKUP"
    exit 6
  fi
  printf '@@result {"status":"restored","reason":"%s","backup":"%s"}\n' "$(printf '%s' "$1" | tr '"\\' "''")" "$BACKUP"
  exit 5
}
sudo -n install -m 644 "$D/@UNIT_NAME@.new" "$UNIT_PATH" || unstage "cannot install $UNIT_PATH (unchanged)"
sudo -n systemctl daemon-reload || restore "daemon-reload failed"
maint_phase "restarting the OLP server on $(basename "$D")"
sudo -n systemctl restart "$UNIT" || restore "restart failed"
step "olp restarted $UNIT on $D"
OLP_FAIL=""
olp_check "$D" || restore "$OLP_FAIL"
cadquery_check "$CQPY" || restore "$OLP_FAIL"
`+cellDeployProgramStoreCheckShell+`olp_select "$D" || restore "$OLP_FAIL"
`+cellDeployOLPRetireShell(ctx)+`printf '@@result {"status":"ok","deployment":"%s","previous":"%s","backup":"%s"}\n' "$D" "$P" "$BACKUP"
`, "@CELL@", shellQuote(ctx.CellID), "@D@", shellQuote(plan.Deployment), "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit), "@UNIT_NAME@", ctx.Deploy.OLP.Unit,
		"@API@", shellQuote("http://"+ctx.Deploy.OLP.Listen+"/api/offline-programming/v1"), "@EXEC@", shellQuote(ctx.ExecutionAddress()),
		"@PIN@", shellQuote(ctx.RTCore.ConfigurationSHA256), "@AXES@", strconv.Itoa(ctx.MachineAxes), "@MODEL@", shellQuote(ctx.RobotModel),
		"@PAIR@", shellQuote(ctx.RTCore.PairID), "@REV@", strconv.FormatUint(ctx.RTCore.PairRevision, 10),
		"@CLIENT_PKI_SELECTION@", rtCoreConfigReplace(clientPKISelection, "@CLIENT_STAGE@", shellQuote(path.Join(ctx.Deploy.Home, ".rosie", "pki-staged", ctx.RTCore.PairID)), "@PAIR_SAN@", shellQuote("URI:rosie-pair:"+ctx.RTCore.PairID)),
		"@ENV@", shellQuote(ctx.Deploy.OLP.PlannerPixiEnv), "@PREFIX@", shellQuote(ctx.Deploy.DeploymentPrefix), "@CAPS@", shellQuote(plan.ExpectCapabilities),
		"@PLANNER@", shellQuote(ctx.Deploy.OLP.PlannerUnit), "@TUNNEL@", shellQuote(ctx.Deploy.OLP.ControlTunnelUnit),
		"@PORIGIN@", shellQuote(strings.TrimSuffix(ctx.Deploy.OLP.PlannerOrigin, "/")), "@PLANNER_INSTALL@", cellDeployPlannerInstallShell(ctx, plan.Deployment, release),
		"@IDLE_PY@", shellQuote(cellDeployOLPIdlePy), "@UNIT_B64@", shellQuote(base64.StdEncoding.EncodeToString([]byte(unit))),
		"@PSTORE@", shellQuote(cellDeployProgramStoreDir(ctx)), "@CQPY@", shellQuote(cellDeployCadQueryPython(plan.Deployment)))
}

// cellDeployOLPRollbackScript reinstalls the unit backup that `olp install`
// saved in the current tree (exactly one <unit>.bak-*), after the same idle
// check, and runs the same checks against the tree it names. The current unit
// is kept as <unit>.rolled-back-<UTC>. OLP state written since the install
// stays in the current tree.
func cellDeployOLPRollbackScript(ctx cellDeployContext, to string) string {
	return cellDeployRelayPrologue(ctx) + "trap '' HUP PIPE\n" + rtCoreConfigReplace(`CELL=@CELL@; UNIT=@UNIT@; UNIT_PATH=/etc/systemd/system/@UNIT_NAME@; TO=@TO@
API=@API@; MODEL=@MODEL@; PLANNER=@PLANNER@; PORIGIN=@PORIGIN@; PMARKER=@PMARKER@; PSTORE=@PSTORE@
`+cellDeployOLPCheckShell+cellDeployPlannerCheckShell+`wd=$(systemctl show "$UNIT" -p WorkingDirectory --value 2> /dev/null)
case "$wd" in */source/offline-programming/v1) CUR=${wd%/source/offline-programming/v1} ;; *) refuse "$UNIT WorkingDirectory '$wd' is not <deployment>/source/offline-programming/v1" ;; esac
shopt -s nullglob
set -- "$CUR"/@UNIT_NAME@.bak-*
shopt -u nullglob
[ "$#" = 1 ] && [ -f "$1" ] || refuse "$CUR holds $# unit backups (@UNIT_NAME@.bak-*); expected exactly one from olp install"
BACKUP=$1
twd=$(sed -n 's/^WorkingDirectory=//p' "$BACKUP" | head -n 1)
case "$twd" in */source/offline-programming/v1) T=${twd%/source/offline-programming/v1} ;; *) refuse "backup $BACKUP has no deployment WorkingDirectory" ;; esac
[ -z "$TO" ] || [ "$TO" = "$T" ] || refuse "the backup returns to $T, not --to $TO"
[ -x "$T/bin/offline-programming" ] && [ -f "$T/private/olp-cells.json" ] || refuse "$T is not a usable OLP tree"
W="$CUR/olp-install"; install -d -m 0700 "$W" || refuse "cannot create $W"
curl -fsS -m 5 "$API/dense-execution/status" > "$W/status-rollback.json" 2> /dev/null || : > "$W/status-rollback.json"
python3 -c @IDLE_PY@ "$W/status-rollback.json" || refuse "the OLP is in use: finish, Stop, disarm and release first"
`+cellDeployPlannerRollbackShell+`KEPT="$CUR/@UNIT_NAME@.rolled-back-$(date -u +%Y%m%dT%H%M%SZ)"
cp -p "$UNIT_PATH" "$KEPT" || refuse "cannot keep $UNIT_PATH"
step "olp rollback $CUR -> $T (unit $BACKUP; current kept as $KEPT)"
sudo -n install -m 644 "$BACKUP" "$UNIT_PATH" && sudo -n systemctl daemon-reload && sudo -n systemctl restart "$UNIT" || { printf '@@result {"status":"failed","reason":"install or restart of %s failed"}\n' "$BACKUP"; exit 1; }
OLP_FAIL=""
olp_check "$T" && olp_select "$T" || { printf '@@result {"status":"failed","reason":"%s"}\n' "$(printf '%s' "$OLP_FAIL" | tr '"\\' "''")"; exit 1; }
`+cellDeployProgramStoreReportShell+`printf '@@result {"status":"ok","deployment":"%s","from":"%s","kept":"%s"}\n' "$T" "$CUR" "$KEPT"
`, "@CELL@", shellQuote(ctx.CellID), "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit), "@UNIT_NAME@", ctx.Deploy.OLP.Unit, "@TO@", shellQuote(to),
		"@API@", shellQuote("http://"+ctx.Deploy.OLP.Listen+"/api/offline-programming/v1"), "@MODEL@", shellQuote(ctx.RobotModel),
		"@PLANNER@", shellQuote(ctx.Deploy.OLP.PlannerUnit), "@PORIGIN@", shellQuote(strings.TrimSuffix(ctx.Deploy.OLP.PlannerOrigin, "/")),
		"@PMARKER@", shellQuote(cellDeployPlannerManagedMarker),
		"@IDLE_PY@", shellQuote(cellDeployOLPIdlePy), "@PSTORE@", shellQuote(cellDeployProgramStoreDir(ctx)))
}

// cellDeployOLPStatusScript is read-only: the unit, its tree, the catalogue
// pins, the planner/tunnel units, the CadQuery env, health and /targets.
func cellDeployOLPStatusScript(ctx cellDeployContext) string {
	return "set -uo pipefail\nexport LC_ALL=C\n" + rtCoreConfigReplace(`UNIT=@UNIT@; API=@API@; CELL=@CELL@
for u in "$UNIT" @PLANNER@ @TUNNEL@; do [ -n "$u" ] && echo "unit $u $(systemctl is-active "$u" 2> /dev/null) $(systemctl is-enabled "$u" 2> /dev/null)"; done
pu=$(systemctl cat @PLANNER@ 2> /dev/null)
echo "planner_unit $(printf '%s\n' "$pu" | grep -qxF @PMARKER@ && echo managed || echo hand-made)"
echo "planner_tree $(printf '%s\n' "$pu" | sed -n 's/^WorkingDirectory=//p' | head -n 1)"
echo "planner_exec $(printf '%s\n' "$pu" | sed -n 's/^ExecStart=//p' | head -n 1)"
echo "planner_revision $(printf '%s\n' "$pu" | sed -n 's/^Environment=AMR_WELD_PLANNER_SOURCE_REVISION=//p' | head -n 1)"
echo "planner_health $(curl -sS -m 30 @PORIGIN@/api/motion/health 2> /dev/null || echo unavailable)"
wd=$(systemctl show "$UNIT" -p WorkingDirectory --value 2> /dev/null); echo "working_directory $wd"
cells=$(systemctl show "$UNIT" -p Environment --value 2> /dev/null | tr ' ' '\n' | sed -n 's/^OFFLINE_PROGRAMMING_CELLS=//p' | head -n 1); echo "catalogue $cells"
[ -f "$cells" ] && python3 -c 'import json,sys
for c in json.load(open(sys.argv[1])).get("cells", []):
    x = c.get("execution") or {}; b = x.get("binding") or {}
    print("cell %s models=%s address=%s configuration=%s machine=%s deployment=%s capabilities=%s" % (c.get("cell_id"), c.get("models"), x.get("address"), b.get("configuration_sha256"), b.get("machine_sha256"), x.get("deployment_sha256"), x.get("capabilities_digest")))' "$cells"
d=${wd%/source/offline-programming/v1}
[ -f "$d/build.json" ] && echo "build $(cat "$d/build.json")"
`+cellDeployCadQueryStatusShell+`ls -1 "$d" 2> /dev/null | grep -E '\.bak-|\.rolled-back-' | sed 's/^/unit_backup /'
echo "health $(curl -sS -m 3 -o /dev/null -w '%{http_code}' "$API/health" 2> /dev/null)"
curl -fsS -m 5 "$API/targets" 2> /dev/null | python3 -c 'import json,sys
for v in json.load(sys.stdin).get("targets", []):
    print("target %s backend=%s reason=%s" % (v.get("cell_id"), v.get("backend"), v.get("reason") or "none"))' || echo "targets unavailable"
PSTORE=@PSTORE@
`+cellDeployProgramStoreReportShell+cellDeployOLPReplacedStatusShell(ctx), "@UNIT@", shellQuote(ctx.Deploy.OLP.Unit), "@API@", shellQuote("http://"+ctx.Deploy.OLP.Listen+"/api/offline-programming/v1"), "@CELL@", shellQuote(ctx.CellID),
		"@PLANNER@", shellQuote(ctx.Deploy.OLP.PlannerUnit), "@TUNNEL@", shellQuote(ctx.Deploy.OLP.ControlTunnelUnit),
		"@PORIGIN@", shellQuote(strings.TrimSuffix(ctx.Deploy.OLP.PlannerOrigin, "/")), "@PMARKER@", shellQuote(cellDeployPlannerManagedMarker),
		"@PSTORE@", shellQuote(cellDeployProgramStoreDir(ctx)))
}
