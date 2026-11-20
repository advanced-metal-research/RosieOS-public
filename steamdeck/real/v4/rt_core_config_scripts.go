package main

import (
	"bytes"
	"compress/gzip"
	"encoding/base64"
	"strconv"
	"strings"
)

// Rendered scripts for `realtime-core config`. The Mac reaches only the relay;
// every script travels as the remote command argument
// `bash -c '<script>'`, never on stdin, so an inner ssh cannot swallow it. Inner
// servo-host hops share one ControlMaster connection: `ssh -n` for single
// commands, and a heredoc for scripts that the servo host first saves to a file
// and then runs with stdin from /dev/null. Remote progress lines are
// `@@step <ms> <phase> <detail>`; the Mac re-stamps them.

// rtCoreConfigPayload is the machine (or bridge) JSON as it travels inside a
// script: gzip, then base64, decoded with `base64 -d | gunzip`. The apply
// session is one argv string and Linux refuses any single argument over 128
// KiB (MAX_ARG_STRLEN, E2BIG). One arm's machine JSON is ~35 KB, ~47 KB as plain
// base64 and carried twice (relay copy and servo-host copy), three times with a
// bridge: plain base64 put the apply session at 132 KB on 2026-10-12. Gzipped
// it is a few KB per copy. Every copy is checked against its sha256 after
// decoding, so a bad decode is refused, never staged.
func rtCoreConfigPayload(b []byte) string {
	var z bytes.Buffer
	w, _ := gzip.NewWriterLevel(&z, gzip.BestCompression)
	w.Write(b)
	w.Close()
	return base64.StdEncoding.EncodeToString(z.Bytes())
}

// rtCoreConfigCompiledFiles are the compiled outputs the Mac compares.
// tuning.json (per-joint tuning keys) is absent in releases before 2026-10-07.
var rtCoreConfigCompiledFiles = []string{"axes.conf", "coordinate.identity.json", "deployment.identity", "machine.identity", "argv.json", "tuning.json"}

func rtCoreConfigReplace(template string, pairs ...string) string {
	return strings.NewReplacer(pairs...).Replace(template)
}

// rtCoreConfigAnchorsNow prints one sorted line per anchor file: name,
// owner:group, mode and sha256. Probe and apply share it byte-for-byte, so the
// apply's "anchors unchanged" check compares like with like.
const rtCoreConfigAnchorsNow = `anchors_now(){ sudo -n bash -c 'cd "$1" && for f in axis-*.anchor; do [ -f "$f" ] || continue; printf "%s %s %s\n" "$f" "$(stat -c "%U:%G %a" "$f")" "$(sha256sum < "$f" | cut -d" " -f1)"; done' _ "$ANCHOR_DIR" | sort; }`

const rtCoreConfigStepFunc = `T0=$(date +%s%N)
step(){ printf '@@step %d %s\n' $(( ($(date +%s%N) - T0) / 1000000 )) "$*"; }`

// rtCoreConfigCollectPy turns one `rtctl compile` run into the compile JSON.
const rtCoreConfigCollectPy = `import json, os, sys
out_txt, err_txt, rc = sys.argv[1], sys.argv[2], int(sys.argv[3])
def rd(p):
    try:
        with open(p) as f:
            return f.read()
    except OSError:
        return ""
res = {"rc": rc, "error": "", "configuration": "", "files": {}}
lines = [l.strip() for l in rd(out_txt).splitlines() if l.strip()]
if rc != 0 or not lines:
    res["error"] = rd(err_txt).strip()[-2000:] or "rtctl compile produced no output"
else:
    directory = lines[-1]
    res["configuration"] = os.path.basename(directory)
    for name in @FILES@:
        res["files"][name] = rd(os.path.join(directory, name))
print(json.dumps(res))`

func rtCoreConfigFilesPy() string {
	quoted := make([]string, len(rtCoreConfigCompiledFiles))
	for i, f := range rtCoreConfigCompiledFiles {
		quoted[i] = strconv.Quote(f)
	}
	return "[" + strings.Join(quoted, ", ") + "]"
}

// rtCoreConfigRevPiProbe reads servo-host facts and prints one JSON object. It
// writes nothing outside its own temporary directory and the lock probe.
func rtCoreConfigRevPiProbe() string {
	return rtCoreConfigReplace(`set -uo pipefail
export LC_ALL=C
R=/opt/rosie-rt-core/current; SOCK=/run/rosie-rt-core/control.sock
t=$(mktemp -d) || exit 1
trap 'rm -rf -- "$t"' EXIT
@ANCHORS_NOW@
readlink -f "$R" > "$t/release"
sha256sum "$R/rt_package.json" 2> "$t/pkg.err" | cut -d' ' -f1 > "$t/pkg"
sudo -n cat /etc/rosie-rt-core/control.env > "$t/control.env" 2> "$t/control.env.err"
sudo -n "$R/bin/rtctl" describe --socket "$SOCK" --json > "$t/describe.json" 2> "$t/describe.err"
sudo -n "$R/bin/rtctl" status --socket "$SOCK" --json > "$t/status.json" 2> "$t/status.err"
for u in rosie-rt-core.service rosie-rt-control.service; do
  printf '%s\t%s\t%s\t%s\n' "$u" "$(systemctl show "$u" -p ActiveState --value)" "$(systemctl is-enabled "$u" 2>/dev/null)" "$(systemctl show "$u" -p NRestarts --value)" >> "$t/units"
done
ANCHOR_DIR=$(systemctl show rosie-rt-control.service -p Environment --value | tr ' ' '\n' | sed -n 's/^ROSIE_RT_ANCHOR_DIR=//p' | head -n 1)
printf '%s' "$ANCHOR_DIR" > "$t/anchor_dir"
if [ -n "$ANCHOR_DIR" ]; then
  sudo -n stat -c '%U:%G %a' "$ANCHOR_DIR" > "$t/anchor_layout" 2>/dev/null
  anchors_now > "$t/anchor_files"
fi
sudo -n find @TELEMETRY@ -maxdepth 1 -type f -name '*-stop.*' -printf '%f %s\n' > "$t/dumps" 2>/dev/null || true
locked=0
if [ -e @LOCK@ ]; then flock -n @LOCK@ true || locked=1; fi
printf '%s' "$locked" > "$t/locked"
cfg=$(sed -n 's/^ROSIE_RT_CONFIGURATION_SHA256=//p' "$t/control.env" | head -n 1)
mkdir "$t/cur"
if printf '%s' "$cfg" | grep -Eqx '[0-9a-f]{64}'; then
  for f in @FILES_SH@; do
    sudo -n cat "/etc/rosie-rt-core/compiled/$cfg/$f" > "$t/cur/$f" 2>/dev/null || rm -f "$t/cur/$f"
  done
fi
python3 - "$t" "$cfg" <<'ROSIE_PROBE_PY'
import json, os, sys
t, cfg = sys.argv[1], sys.argv[2]
def rd(name):
    try:
        with open(os.path.join(t, name)) as f:
            return f.read()
    except OSError:
        return ""
out = {"errors": [], "units": {}, "stop_dumps": [], "locked": rd("locked").strip() == "1"}
out["release"] = os.path.basename(rd("release").strip())
out["package_manifest_sha256"] = rd("pkg").strip()
if not out["package_manifest_sha256"]:
    out["errors"].append("rt_package.json unreadable: " + rd("pkg.err").strip())
env = rd("control.env")
if not env:
    out["errors"].append("control.env unreadable: " + rd("control.env.err").strip())
kv, remote = {}, 0
for line in env.splitlines():
    k, sep, v = line.partition("=")
    if sep:
        kv[k] = v
    if line.startswith("ROSIE_RT_REMOTE_ARGS="):
        remote += 1
out["control_env"] = {"configuration": kv.get("ROSIE_RT_CONFIGURATION_SHA256", ""), "pair_id": kv.get("ROSIE_RT_PAIR_ID", ""),
                      "pair_revision": kv.get("ROSIE_RT_PAIR_REVISION", ""), "remote_args_lines": remote}
def load(name):
    try:
        d = json.loads(rd(name + ".json"))
    except ValueError:
        out["errors"].append("rtctl %s failed: %s" % (name, rd(name + ".err").strip()[-300:]))
        return {}
    return d.get("data", d) if isinstance(d, dict) else {}
d = load("describe")
out["describe"] = {k: str(d.get(k) or "") for k in ("configuration_sha256", "machine_sha256", "deployment_sha256", "capabilities_digest", "backend")}
out["describe"]["axes"] = len(d.get("axes") or [])
s = load("status")
c = s.get("core") if isinstance(s.get("core"), dict) else {}
status = {}
for k in ("armed", "axis_enable_mask", "lease_valid", "config_verified_mask", "home_valid_mask", "safety_fault_mask", "execution_fault_reasons"):
    v = c.get(k)
    if v is None:
        if s:
            out["errors"].append("status core lacks " + k)
        v = 0
    status[k] = int(v)
alarms = set()
for axes in (c.get("axes") or [], s.get("axes") or []):
    for i, a in enumerate(axes):
        code, alarm = a.get("error_code") or 0, a.get("drive_alarm") or ""
        if code or alarm not in ("", "none"):
            alarms.add("axis %d error_code=%s drive_alarm=%s %s" % (i, code, alarm, a.get("drive_alarm_text") or ""))
status["alarms"] = sorted(alarms)
out["status"] = status
for line in rd("units").splitlines():
    f = line.split("\t")
    if len(f) == 4:
        out["units"][f[0]] = {"active": f[1], "enabled": f[2], "n_restarts": f[3]}
out["anchor"] = {"dir": rd("anchor_dir").strip(), "layout": rd("anchor_layout").strip(), "files": [l for l in rd("anchor_files").splitlines() if l]}
out["stop_dumps"], out["stop_dump_bytes"] = [], {}
for l in rd("dumps").splitlines():
    name, _, size = l.rpartition(" ")
    if name:
        out["stop_dumps"].append(name)
        out["stop_dump_bytes"][name] = int(size or 0)
out["current"] = {"configuration": cfg, "files": {}}
for name in @FILES@:
    p = os.path.join(t, "cur", name)
    if os.path.exists(p):
        out["current"]["files"][name] = rd(os.path.join("cur", name))
print(json.dumps(out))
ROSIE_PROBE_PY
`, "@ANCHORS_NOW@", rtCoreConfigAnchorsNow, "@TELEMETRY@", rtCoreConfigTelemetryDir, "@LOCK@", rtCoreConfigLockPath,
		"@FILES_SH@", strings.Join(rtCoreConfigCompiledFiles, " "), "@FILES@", rtCoreConfigFilesPy())
}

// rtCoreConfigRevPiCompile compiles the machine bytes with the installed rtctl
// at the lowest CPU priority while the controller keeps running. It is used
// only when the relay's package copy is not the installed build.
func rtCoreConfigRevPiCompile(src rtCoreConfigSource) string {
	return rtCoreConfigReplace(`set -uo pipefail
export LC_ALL=C
t=$(mktemp -d) || exit 1
trap 'rm -rf -- "$t"' EXIT
printf '%s' @B64@ | base64 -d | gunzip > "$t/machine.json"
if [ "$(sha256sum "$t/machine.json" | cut -d' ' -f1)" != @BYTES_SHA@ ]; then
  printf '{"rc": 1, "error": "machine bytes changed in transit", "configuration": "", "files": {}}\n'; exit 0
fi
nice -n 19 /opt/rosie-rt-core/current/bin/rtctl compile --config "$t/machine.json" --out "$t/out" > "$t/out.txt" 2> "$t/err.txt"
rc=$?
python3 - "$t/out.txt" "$t/err.txt" "$rc" <<'ROSIE_COLLECT_PY'
@COLLECT@
ROSIE_COLLECT_PY
`, "@B64@", shellQuote(rtCoreConfigPayload(src.MachineBytes)), "@BYTES_SHA@", shellQuote(src.BytesSHA256),
		"@COLLECT@", strings.ReplaceAll(rtCoreConfigCollectPy, "@FILES@", rtCoreConfigFilesPy()))
}

// rtCoreConfigRelayTransport opens every relay script: progress and the
// servo-host hop helpers over one shared connection. -6 only for an IPv6
// literal (a Rosie 1430 cell's link-local route); it would refuse an IPv4 servo host.
func rtCoreConfigRelayTransport(revpiTarget string) string {
	family := ""
	if _, host, ok := strings.Cut(revpiTarget, "@"); ok && strings.Contains(host, ":") {
		family = "-6 "
	}
	return rtCoreConfigReplace(`set -uo pipefail
export LC_ALL=C NO_REAL_MOVEMENT=1
@STEP@
REVPI=@REVPI@
CMP="$HOME/.ssh/rosie-cfg-%C"
rssh(){ ssh -n @FAMILY@-o BatchMode=yes @ALIVE@ -o ControlMaster=auto -o "ControlPath=$CMP" -o ControlPersist=60 -- "$REVPI" "$@"; }
rsshs(){ ssh @FAMILY@-o BatchMode=yes @ALIVE@ -o ControlMaster=auto -o "ControlPath=$CMP" -o ControlPersist=60 -- "$REVPI" 'f=$(mktemp) && cat > "$f" && bash "$f" < /dev/null; rc=$?; rm -f "$f"; exit $rc'; }
trap 'ssh -o "ControlPath=$CMP" -O exit -- "$REVPI" > /dev/null 2>&1 || true' EXIT
`, "@STEP@", rtCoreConfigStepFunc, "@REVPI@", shellQuote(revpiTarget), "@FAMILY@", family, "@ALIVE@", sshAliveShell(5))
}

// rtCoreConfigMaintenanceScript is a relay session that only handles the
// maintenance notice (rt_core_maintenance.go): body runs with the notice
// helpers defined and rsshs reaching the servo host.
func rtCoreConfigMaintenanceScript(revpiTarget string, m rtCoreMaintenance, body string) string {
	return rtCoreConfigRelayTransport(revpiTarget) + rtCoreMaintenanceShell(m) + body
}

// rtCoreConfigRelayPrologue opens the relay script: progress, the servo-host
// hop helpers and the per-run working directory (not /tmp: noexec on one relay).
func rtCoreConfigRelayPrologue(src rtCoreConfigSource, revpiTarget, runID string, olpUnit string) string {
	return rtCoreConfigRelayTransport(revpiTarget) + rtCoreConfigReplace(`CELL_ID=@CELL_ID@; CELL_MANIFEST=@CELL_MANIFEST@; MACHINE=@MACHINE@
W="$HOME/.rosie/rt-core/config-deploy/"@RUN@
install -d -m 0700 "$W" || { printf '@@refused %s\n' "cannot create $W"; exit 2; }
printf '%s' @B64@ | base64 -d | gunzip > "$W/machine.json"
if [ "$(sha256sum "$W/machine.json" | cut -d' ' -f1)" != @BYTES_SHA@ ]; then printf '@@refused %s\n' 'machine bytes changed in transit'; exit 2; fi
OLP_UNIT=@OLP_UNIT@; OLP_ERR=""; OLP_SRC=""; OLP_DIR=""; OLP_CELLS=""; OLP_LISTEN=""; OLP_PKG_SHA=""
DUMP_DIR=@DUMP_DIR@
if [ -z "$OLP_UNIT" ]; then
  OLP_ERR="no OLP unit is recorded for $CELL_ID"
else
  olp_wd=$(systemctl show "$OLP_UNIT" -p WorkingDirectory --value 2>/dev/null)
  case "$olp_wd" in
    */offline-programming/v1) OLP_SRC=${olp_wd%/offline-programming/v1}; OLP_DIR=$(dirname "$OLP_SRC") ;;
    *) OLP_ERR="$OLP_UNIT WorkingDirectory '$olp_wd' is not <deployment>/source/offline-programming/v1" ;;
  esac
  OLP_CELLS=$(systemctl show "$OLP_UNIT" -p Environment --value 2>/dev/null | tr ' ' '\n' | sed -n 's/^OFFLINE_PROGRAMMING_CELLS=//p' | head -n 1)
  OLP_LISTEN=$(systemctl show "$OLP_UNIT" -p ExecStart --value 2>/dev/null | grep -o -- '--listen[= ][^ ;]*' | head -n 1 | sed 's/^--listen[= ]//')
  if [ -z "$OLP_ERR" ] && [ ! -f "$OLP_SRC/$CELL_MANIFEST" ]; then OLP_ERR="$OLP_SRC/$CELL_MANIFEST missing"; fi
  if [ -z "$OLP_ERR" ] && [ ! -f "$OLP_SRC/$MACHINE" ]; then OLP_ERR="$OLP_SRC/$MACHINE missing"; fi
  if [ -z "$OLP_ERR" ] && [ ! -f "$OLP_CELLS" ]; then OLP_ERR="$OLP_UNIT OFFLINE_PROGRAMMING_CELLS '$OLP_CELLS' is not a file"; fi
  if [ -n "$OLP_DIR" ] && [ -f "$OLP_DIR/runtime-package/rt_package.json" ]; then
    OLP_PKG_SHA=$(sha256sum "$OLP_DIR/runtime-package/rt_package.json" | cut -d' ' -f1)
  fi
fi
OLP_LISTEN=${OLP_LISTEN:-127.0.0.1:8794}
`, "@RUN@", shellQuote(runID),
		"@CELL_ID@", shellQuote(src.CellID), "@CELL_MANIFEST@", shellQuote(src.CellManifest), "@MACHINE@", shellQuote(src.MachinePath),
		"@B64@", shellQuote(rtCoreConfigPayload(src.MachineBytes)), "@BYTES_SHA@", shellQuote(src.BytesSHA256),
		"@OLP_UNIT@", shellQuote(olpUnit), "@DUMP_DIR@", shellQuote(src.DumpDir))
}

// rtCoreConfigProbeCommand is session 1 and the whole of --dry-run: read-only on
// both hosts. It precompiles with the installed build's rtctl (the relay's
// package copy when its rt_package.json is the servo host's, else the servo
// host's own rtctl at nice 19) and prints one @@probe JSON line.
func rtCoreConfigProbeCommand(src rtCoreConfigSource, revpiTarget, runID, olpUnit string) string {
	return rtCoreConfigRelayPrologue(src, revpiTarget, runID, olpUnit) + rtCoreConfigReplace(`step "probe relay=$(hostname) olp=${OLP_DIR:-none} ${OLP_ERR}"
rsshs > "$W/revpi.json" 2> "$W/revpi.err" <<'ROSIE_RT_CONFIG_PROBE'
@REVPI_PROBE@
ROSIE_RT_CONFIG_PROBE
rc=$?
if [ "$rc" != 0 ] || ! python3 -c 'import json,sys; json.load(open(sys.argv[1]))' "$W/revpi.json" 2> /dev/null; then
  printf '@@refused %s\n' "servo host probe failed (ssh exit $rc): $(tail -n 3 "$W/revpi.err" | tr '\n' ' ')"; exit 2
fi
REVPI_PKG=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("package_manifest_sha256", ""))' "$W/revpi.json")
step "probe servo host read release=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("release", ""))' "$W/revpi.json") package=${REVPI_PKG:0:12}"
if [ -n "$OLP_PKG_SHA" ] && [ "$OLP_PKG_SHA" = "$REVPI_PKG" ]; then
  WHERE="relay copy of the installed package ($OLP_DIR/runtime-package, rt_package.json identical)"
  rm -rf -- "$W/new"
  "$OLP_DIR/runtime-package/bin/rtctl" compile --config "$W/machine.json" --out "$W/new" > "$W/compile.out" 2> "$W/compile.err" < /dev/null
  crc=$?
  python3 - "$W/compile.out" "$W/compile.err" "$crc" > "$W/compile.json" <<'ROSIE_COLLECT_PY'
@COLLECT@
ROSIE_COLLECT_PY
else
  WHERE="servo host installed rtctl at nice 19 (relay package copy missing or not the installed build)"
  rsshs > "$W/compile.json" 2> "$W/compile.err" <<'ROSIE_RT_CONFIG_COMPILE'
@REVPI_COMPILE@
ROSIE_RT_CONFIG_COMPILE
fi
step "probe precompiled with $WHERE"
python3 - "$W" "$WHERE" "$OLP_UNIT" "$OLP_DIR" "$OLP_SRC" "$OLP_CELLS" "$OLP_LISTEN" "$OLP_PKG_SHA" "$OLP_ERR" "$CELL_MANIFEST" "$MACHINE" "$DUMP_DIR" <<'ROSIE_MERGE_PY'
import hashlib, json, os, sys
w, where, unit, olp_dir, olp_src, cells, listen, pkg, err, cell_manifest, machine, dump_dir = sys.argv[1:13]
def rd(p):
    try:
        with open(p) as f:
            return f.read()
    except OSError:
        return ""
probe = json.load(open(os.path.join(w, "revpi.json")))
try:
    compiled = json.loads(rd(os.path.join(w, "compile.json")))
except ValueError:
    compiled = {"rc": 1, "error": rd(os.path.join(w, "compile.err"))[-2000:] or "precompile produced no result", "configuration": "", "files": {}}
compiled["where"] = where
probe["compile"] = compiled
pin = ""
if not err:
    try:
        for node in json.loads(rd(os.path.join(olp_src, cell_manifest))).get("nodes", []):
            rt = node.get("rt_core") or {}
            if rt.get("machine_config") == machine:
                pin = rt.get("configuration_sha256", "")
    except ValueError as e:
        err = "OLP cell manifest unreadable: %s" % e
probe["olp"] = {"unit": unit, "dir": olp_dir, "source": olp_src, "catalogue": cells, "listen": listen, "package_manifest_sha256": pkg, "cell_pin": pin, "error": err}
# Stop dumps with a copy in the cell's dump_dir whose sha256 is the one
# copy-dumps verified against the servo host (dump_dir/SHA256SUMS).
sums, copied = {}, []
if dump_dir:
    for line in rd(os.path.join(dump_dir, "SHA256SUMS")).splitlines():
        h, _, n = line.partition("  ")
        if len(h) == 64 and n:
            sums[n] = h
for name in probe.get("stop_dumps") or []:
    p = os.path.join(dump_dir, name) if dump_dir else ""
    if name in sums and p and os.path.isfile(p) and os.path.getsize(p) == (probe.get("stop_dump_bytes") or {}).get(name):
        h = hashlib.sha256()
        with open(p, "rb") as f:
            for b in iter(lambda: f.read(1 << 20), b""):
                h.update(b)
        if h.hexdigest() == sums[name]:
            copied.append(name)
probe["stop_dumps_copied"] = copied
print("@@probe " + json.dumps(probe))
ROSIE_MERGE_PY
`, "@REVPI_PROBE@", rtCoreConfigRevPiProbe(), "@REVPI_COMPILE@", rtCoreConfigRevPiCompile(src),
		"@COLLECT@", strings.ReplaceAll(rtCoreConfigCollectPy, "@FILES@", rtCoreConfigFilesPy()))
}

// rtCoreConfigRevPiApply is session 2 on the servo host. Order: lock, re-check
// every probe fact, stage and back up, re-check disarmed, stop both units,
// publish with the installed rtctl through generate-control-env.sh
// --keep-remote-args, start, then wait for the configuration, Home, no restart,
// unchanged anchors and enablement. Any failure after the stop restores the
// backed-up control.env and proves the old configuration; if that also fails it
// stops both units. It ignores SIGHUP/SIGPIPE so a dropped connection cannot
// leave it between stop and start. It never installs a release, runs a host
// check, changes enablement, touches the EtherCAT master, or sends any Home or
// anchor command.
func rtCoreConfigRevPiApply(c robotV4RTCoreConfig, src rtCoreConfigSource, plan rtCoreConfigPlan, release, packageSHA string, opts rtCoreConfigOptions) string {
	staged := rtCoreConfigHistoryDir + "/machines/" + plan.New + ".json"
	bridgeStaged := rtCoreConfigHistoryDir + "/machines/" + plan.New + ".bridge.json"
	bridgeB64 := ""
	if len(plan.Bridge) > 0 {
		bridgeB64 = rtCoreConfigPayload(plan.Bridge)
	}
	return rtCoreConfigReplace(`set -uo pipefail
trap '' PIPE HUP
export NO_REAL_MOVEMENT=1 LC_ALL=C
NEW=@NEW@; OLD=@OLD@; RELEASE=@RELEASE@; PKGSHA=@PKGSHA@; BYTES_SHA=@BYTES_SHA@
PAIR=@PAIR@; REV=@REV@; HOME_MASK=@HOME_MASK@; NEW_HOME_MASK=@NEW_HOME_MASK@; READY_S=@READY_S@; HOME_S=@HOME_S@
LABEL=@LABEL@; GIT_HEAD=@GIT_HEAD@; EN_BEFORE=@EN_BEFORE@; COPIED=@COPIED@
CORE_RESTARTS=@CORE_RESTARTS@; CONTROL_RESTARTS=@CONTROL_RESTARTS@
ANCHOR_DIR=@ANCHOR_DIR@
ANCHORS_BEFORE=@ANCHORS@
MACHINE_B64=@B64@
BRIDGE_B64=@BRIDGE_B64@; BRIDGE_SHA=@BRIDGE_SHA@; BRIDGE_STAGED=@BRIDGE_STAGED@
R=/opt/rosie-rt-core/current; RTCTL="$R/bin/rtctl"; SOCK=/run/rosie-rt-core/control.sock
H=@HISTORY@; ENV=/etc/rosie-rt-core/control.env; LOCK=@LOCK@; TELEMETRY=@TELEMETRY@
STAGED=@STAGED@
UNITS="rosie-rt-core.service rosie-rt-control.service"
@STEP@
refuse(){ step "revpi refused: $*"; printf '@@refused %s\n' "$*"; exit 2; }
result(){ python3 -c 'import json, sys; print("@@result " + json.dumps(dict(a.split("=", 1) for a in sys.argv[1:])))' "$@"; }
scratch=$(mktemp -d) || refuse "mktemp failed"
trap 'rm -rf -- "$scratch"' EXIT
@ANCHORS_NOW@
en_now(){ printf '%s/%s' "$(systemctl is-enabled rosie-rt-core.service 2>/dev/null)" "$(systemctl is-enabled rosie-rt-control.service 2>/dev/null)"; }
# Only rosie-rt-core restarts rewrite startup SDOs. rosie-rt-control gives up on
# the native socket after 6 s and systemd restarts it once while a nine-axis core
# is still reading back its startup values (~25 s, nine-motor bench 2026-11-01); that
# restart never touches the bus, so it is not counted here.
restart_marker(){ for u in rosie-rt-core.service; do printf '%s:%s:%s ' "$u" "$(systemctl show "$u" -p InvocationID --value)" "$(systemctl show "$u" -p NRestarts --value)"; done; }
units_inactive(){ for u in $UNITS; do s=$(systemctl show "$u" -p ActiveState --value); if [ "$s" != active ]; then printf '%s is %s' "$u" "$s"; return 0; fi; done; return 1; }
units_down(){ for u in $UNITS; do s=$(systemctl show "$u" -p ActiveState --value); case "$s" in active|activating|reloading) ;; *) printf '%s is %s' "$u" "$s"; return 0 ;; esac; done; return 1; }
journal(){
  sudo -n test -e "$H/journal.jsonl" || sudo -n install -m 0640 -o root -g root /dev/null "$H/journal.jsonl"
  python3 -c 'import json, sys, time; e, d, rel, old, new, label, head, sha = sys.argv[1:9]; print(json.dumps({"time": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "event": e, "release": rel, "old": old, "new": new, "label": label, "git_head": head, "bytes_sha256": sha, "detail": d}))' \
    "$1" "${2:-}" "$RELEASE" "$OLD" "$NEW" "$LABEL" "$GIT_HEAD" "$BYTES_SHA" | sudo -n tee -a "$H/journal.jsonl" > /dev/null
}
check_state(){ # $1 configuration, $2 required home_valid_mask ("" = any); prints the blockers
  sudo -n "$RTCTL" describe --socket "$SOCK" --json > "$scratch/d.json" 2> /dev/null || { echo "describe unavailable"; return 1; }
  sudo -n "$RTCTL" status --socket "$SOCK" --json > "$scratch/s.json" 2> /dev/null || { echo "status unavailable"; return 1; }
  python3 - "$scratch/d.json" "$scratch/s.json" "$1" "$2" <<'ROSIE_CHECK_PY'
import json, sys
@BUSY_PY@d = json.load(open(sys.argv[1])); d = d.get("data", d)
s = json.load(open(sys.argv[2])); s = s.get("data", s); c = s.get("core") or {}
p = []
axes = len(d.get("axes") or [])
full = (1 << axes) - 1
if d.get("configuration_sha256") != sys.argv[3]:
    p.append("configuration=" + str(d.get("configuration_sha256"))[:12])
if d.get("backend") != "ethercat":
    p.append("backend=" + str(d.get("backend")))
if rosie_busy(c):
    p.append("armed/enabled/leased")
cv = int(c.get("config_verified_mask", 0))
if axes == 0 or cv != full:
    p.append("config_verified_mask=%d/%d" % (cv, full))
if int(c.get("safety_fault_mask", 1)) or int(c.get("execution_fault_reasons", 1)):
    p.append("faults safety=%s execution=%s" % (c.get("safety_fault_mask"), c.get("execution_fault_reasons")))
for group in (c.get("axes") or [], s.get("axes") or []):
    for i, a in enumerate(group):
        if (a.get("error_code") or 0) or (a.get("drive_alarm") or "") not in ("", "none"):
            p.append("axis %d error_code=%s drive_alarm=%s" % (i, a.get("error_code"), a.get("drive_alarm")))
hv = int(c.get("home_valid_mask", 0))
if sys.argv[4] != "" and hv != int(sys.argv[4]):
    p.append("home_valid_mask=%d/%s" % (hv, sys.argv[4]))
print("; ".join(sorted(set(p))) or "configuration=%s config_verified_mask=%d home_valid_mask=%d" % (str(d.get("configuration_sha256"))[:12], cv, hv))
sys.exit(1 if p else 0)
ROSIE_CHECK_PY
}
WAIT_FAIL=""
wait_until(){ # $1 configuration, $2 home mask or "", $3 seconds, $4 label; polls every 1 s, reports every 2 s
  local start=$SECONDS last=-2 elapsed msg="not checked" down
  while :; do
    elapsed=$((SECONDS - start))
    if [ "$(restart_marker)" != "$MARKER" ]; then WAIT_FAIL="unexpected unit restart during $4 ($(restart_marker)); each restart rewrites every startup SDO"; return 1; fi
    if down=$(units_down); then WAIT_FAIL="$down during $4"; return 1; fi
    if msg=$(check_state "$1" "$2"); then step "revpi $4 ok after ${elapsed}s: $msg"; return 0; fi
    if [ "$elapsed" -ge "$3" ]; then break; fi
    if [ $((elapsed - last)) -ge 2 ]; then step "revpi $4 waiting ${elapsed}s: $msg"; last=$elapsed; fi
    sleep 1
  done
  WAIT_FAIL="$4 not reached within ${3}s: $msg"; return 1
}

# ---- lock: one configuration deploy at a time ----
[ -e "$LOCK" ] || : > "$LOCK" 2> /dev/null
exec 9< "$LOCK" || refuse "cannot open $LOCK"
flock -n 9 || refuse "another configuration deploy holds $LOCK"
step "revpi lock held"

# ---- preflight: every probe fact again; the probe may be stale ----
test "$(readlink -f "$R")" = "/opt/rosie-rt-core/$RELEASE" || refuse "installed release changed since the probe (now $(readlink -f "$R"))"
test "$(sha256sum "$R/rt_package.json" | cut -d' ' -f1)" = "$PKGSHA" || refuse "installed package manifest changed since the probe"
sudo -n grep -qFx "ROSIE_RT_CONFIGURATION_SHA256=$OLD" "$ENV" || refuse "control.env is no longer $OLD"
sudo -n grep -qFx "ROSIE_RT_PAIR_ID=$PAIR" "$ENV" || refuse "control.env pair id is not $PAIR"
sudo -n grep -qFx "ROSIE_RT_PAIR_REVISION=$REV" "$ENV" || refuse "control.env pair revision is not $REV"
if down=$(units_inactive); then refuse "$down; config-only restarts a running controller"; fi
test "$(en_now)" = "$EN_BEFORE" || refuse "unit enablement changed since the probe ($(en_now), was $EN_BEFORE)"
test "$(systemctl show rosie-rt-core.service -p NRestarts --value)" = "$CORE_RESTARTS" || refuse "rosie-rt-core restarted since the probe"
test "$(systemctl show rosie-rt-control.service -p NRestarts --value)" = "$CONTROL_RESTARTS" || refuse "rosie-rt-control restarted since the probe"
msg=$(check_state "$OLD" "$HOME_MASK") || refuse "preflight: $msg (config deploy never clears faults, releases a lease or disarms)"
test "$(sudo -n stat -c '%U:%G %a' "$ANCHOR_DIR")" = @LAYOUT@ || refuse "anchor dir $ANCHOR_DIR is not @LAYOUT_TEXT@; this tool does not repair it"
test "$(anchors_now)" = "$ANCHORS_BEFORE" || refuse "anchor files changed since the probe"
for f in $(sudo -n find "$TELEMETRY" -maxdepth 1 -type f -name '*-stop.*' 2> /dev/null | sed 's|.*/||'); do
  case " $COPIED " in *" $f "*) ;; *) refuse "stop dump $f in $TELEMETRY has no verified copy and would be deleted by the restart; run realtime-core copy-dumps first" ;; esac
done
step "revpi preflight ok (release, package, control.env, pair, units, enablement, restarts, disarmed, no lease, no faults, config verified, home $HOME_MASK, anchors, every stop dump copied)"

# ---- stage the machine bytes and back up control.env ----
sudo -n install -d -m 0750 -o root -g root "$H" "$H/machines" "$H/control-env" || refuse "cannot create $H"
printf '%s' "$MACHINE_B64" | base64 -d | gunzip > "$scratch/machine.json"
test "$(sha256sum "$scratch/machine.json" | cut -d' ' -f1)" = "$BYTES_SHA" || refuse "machine bytes changed in transit"
sudo -n install -m 0640 -o root -g root "$scratch/machine.json" "$STAGED.tmp" && sudo -n mv -f "$STAGED.tmp" "$STAGED" || refuse "cannot stage $STAGED"
test "$(sudo -n sha256sum "$STAGED" | cut -d' ' -f1)" = "$BYTES_SHA" || refuse "staged machine bytes differ"
backup="$H/control-env/$(date -u +%Y%m%dT%H%M%SZ)-$OLD.env"
sudo -n install -m 0640 -o root -g rosie-ctl "$ENV" "$backup" || refuse "cannot back up control.env"
test "$(sudo -n sha256sum "$backup" | cut -d' ' -f1)" = "$(sudo -n sha256sum "$ENV" | cut -d' ' -f1)" || refuse "control.env backup differs"
remote_before=$(sudo -n grep '^ROSIE_RT_REMOTE_ARGS=' "$ENV")
journal apply-begin "staged=$STAGED backup=$backup"
step "revpi staged $STAGED; backup $backup"

failed_rollback(){
  step "revpi ROLLBACK FAILED: $1"
  sudo -n systemctl stop rosie-rt-control.service rosie-rt-core.service
  journal rollback-failed "$1"
  result status=rollback_failed reason="$1" backup="$backup" staged="$STAGED"
  printf '%s\n' "ROLLBACK FAILED on $(hostname): $1" \
    "rosie-rt-control and rosie-rt-core are STOPPED; bus owner: none." \
    "control.env should hold the backup $backup ($OLD); check: sudo -n grep CONFIGURATION_SHA256 $ENV" \
    "Read the cause first: sudo -n journalctl -u rosie-rt-core -u rosie-rt-control --since '$since' (servo host clock is UTC)." \
    "Then start the old configuration explicitly: sudo -n systemctl start rosie-rt-core.service rosie-rt-control.service" \
    "and verify Describe configuration $OLD, config_verified_mask full, home_valid_mask $HOME_MASK, disarmed." \
    "Fix a fault at its cause; never re-home or restore anchors to recover." >&2
  exit 4
}
rollback(){
  step "revpi rollback: $1"
  journal rollback-begin "$1"
  sudo -n systemctl stop rosie-rt-control.service rosie-rt-core.service
  p=$(sudo -n mktemp "$(dirname "$ENV")/.control.env.XXXXXX") || failed_rollback "mktemp for the restore failed ($1)"
  sudo -n install -m 0640 -o root -g rosie-ctl "$backup" "$p" && sudo -n mv -f "$p" "$ENV" || failed_rollback "could not restore control.env from $backup ($1)"
  step "revpi restored control.env from $backup"
  sudo -n systemctl start rosie-rt-core.service rosie-rt-control.service || failed_rollback "start of the old configuration failed ($1)"
  MARKER=$(restart_marker)
  if wait_until "$OLD" "" "$READY_S" "rollback start" && wait_until "$OLD" "$HOME_MASK" "$HOME_S" "rollback home"; then
    journal rollback-ok "$1"
    result status=rolled_back reason="$1" configuration="$OLD" backup="$backup" staged="$STAGED"
    exit 3
  fi
  failed_rollback "old configuration $OLD did not verify: $WAIT_FAIL ($1)"
}

# ---- stop, publish with the installed compiler, start, verify ----
since=$(date -u '+%Y-%m-%d %H:%M:%S')
msg=$(check_state "$OLD" "$HOME_MASK") || refuse "state changed just before the stop: $msg"
journal stop
sudo -n systemctl stop rosie-rt-control.service rosie-rt-core.service || rollback "systemctl stop failed"
step "revpi stopped rosie-rt-control rosie-rt-core"
if [ -n "$BRIDGE_B64" ]; then
  # Trial keys stored unchanged: start the bridge (those keys RAM-only one raw
  # step away) so the new configuration's stored write differs from drive RAM.
  printf '%s' "$BRIDGE_B64" | base64 -d | gunzip > "$scratch/bridge.json"
  test "$(sha256sum "$scratch/bridge.json" | cut -d' ' -f1)" = "$BRIDGE_SHA" || rollback "bridge machine bytes changed in transit"
  sudo -n install -m 0640 -o root -g root "$scratch/bridge.json" "$BRIDGE_STAGED.tmp" && sudo -n mv -f "$BRIDGE_STAGED.tmp" "$BRIDGE_STAGED" || rollback "cannot stage $BRIDGE_STAGED"
  out=$(@BRIDGE_CONTROL_ENV@ 2>&1)
  rc=$?
  [ "$rc" -eq 0 ] || rollback "bridge: generate-control-env.sh failed (exit $rc): $(printf '%s' "$out" | tail -n 3 | tr '\n' ' ')"
  bridge=$(printf '%s\n' "$out" | tail -n 1); bridge=${bridge#configuration_sha256=}
  printf '%s' "$bridge" | grep -qxE '[0-9a-f]{64}' || rollback "bridge: installed rtctl published no configuration ($bridge)"
  [ "$bridge" != "$NEW" ] && [ "$bridge" != "$OLD" ] || rollback "bridge configuration equals the old or new one ($bridge)"
  sudo -n grep -qFx "ROSIE_RT_CONFIGURATION_SHA256=$bridge" "$ENV" || rollback "control.env does not carry the bridge $bridge"
  test "$(sudo -n grep '^ROSIE_RT_REMOTE_ARGS=' "$ENV")" = "$remote_before" || rollback "ROSIE_RT_REMOTE_ARGS changed by the bridge"
  journal bridge-published "$bridge"
  step "revpi bridge published configuration=$bridge: @PROMOTED@"
  sudo -n systemctl start rosie-rt-core.service rosie-rt-control.service || rollback "systemctl start of the bridge failed"
  MARKER=$(restart_marker)
  wait_until "$bridge" "" "$READY_S" "bridge start" || rollback "$WAIT_FAIL"
  wait_until "$bridge" "$NEW_HOME_MASK" "$HOME_S" "bridge home" || rollback "$WAIT_FAIL"
  journal bridge-ok "$bridge"
  sudo -n systemctl stop rosie-rt-control.service rosie-rt-core.service || rollback "systemctl stop after the bridge failed"
  step "revpi bridge verified and stopped; the new configuration now stores the promoted keys"
fi
out=$(@CONTROL_ENV@ 2>&1)
rc=$?
[ "$rc" -eq 0 ] || rollback "generate-control-env.sh failed (exit $rc): $(printf '%s' "$out" | tail -n 3 | tr '\n' ' ')"
got=$(printf '%s\n' "$out" | tail -n 1)
test "$got" = "configuration_sha256=$NEW" || rollback "installed rtctl published $got, the probe planned $NEW"
sudo -n grep -qFx "ROSIE_RT_CONFIGURATION_SHA256=$NEW" "$ENV" || rollback "control.env does not carry $NEW"
test "$(sudo -n grep '^ROSIE_RT_REMOTE_ARGS=' "$ENV")" = "$remote_before" || rollback "ROSIE_RT_REMOTE_ARGS changed"
journal published "$got"
step "revpi published configuration=$NEW (generate-control-env.sh --keep-remote-args $STAGED)"
sudo -n systemctl start rosie-rt-core.service rosie-rt-control.service || rollback "systemctl start failed"
MARKER=$(restart_marker)
step "revpi started; waiting up to ${READY_S}s for every axis to verify $NEW"
wait_until "$NEW" "" "$READY_S" "start" || rollback "$WAIT_FAIL"
wait_until "$NEW" "$NEW_HOME_MASK" "$HOME_S" "home" || rollback "$WAIT_FAIL"
test "$(anchors_now)" = "$ANCHORS_BEFORE" || rollback "anchor files changed during the restart"
test "$(en_now)" = "$EN_BEFORE" || rollback "unit enablement changed ($(en_now), was $EN_BEFORE)"
journal apply-ok
step "revpi verified $NEW: config verified, home_valid_mask $NEW_HOME_MASK (was $HOME_MASK; an accepted coordinate change invalidates the difference until the operator homes those axes at their home pose), disarmed, no faults, no restarts, anchors and enablement unchanged"
python3 - "$scratch/d.json" "$backup" "$STAGED" <<'ROSIE_RESULT_PY'
import json, sys
d = json.load(open(sys.argv[1])); d = d.get("data", d)
print("@@result " + json.dumps({"status": "ok", "configuration": d.get("configuration_sha256", ""), "machine": d.get("machine_sha256", ""),
      "deployment": d.get("deployment_sha256", ""), "capabilities": d.get("capabilities_digest", ""), "backup": sys.argv[2], "staged": sys.argv[3]}))
ROSIE_RESULT_PY
exit 0
`,
		"@NEW@", shellQuote(plan.New), "@OLD@", shellQuote(plan.Old), "@RELEASE@", shellQuote(release), "@PKGSHA@", shellQuote(packageSHA),
		"@BYTES_SHA@", shellQuote(src.BytesSHA256), "@PAIR@", shellQuote(c.PairID), "@REV@", strconv.FormatUint(c.PairRevision, 10),
		"@HOME_MASK@", strconv.FormatUint(uint64(plan.HomeMask), 10), "@NEW_HOME_MASK@", strconv.FormatUint(uint64(plan.NewHomeMask), 10),
		"@READY_S@", strconv.Itoa(int(opts.ReadyTimeout.Seconds())), "@HOME_S@", strconv.Itoa(int(opts.HomeTimeout.Seconds())),
		"@LABEL@", shellQuote(plan.Label), "@GIT_HEAD@", shellQuote(src.GitHead), "@EN_BEFORE@", shellQuote(plan.EnablementBefore), "@COPIED@", shellQuote(strings.Join(plan.CopiedDumps, " ")),
		"@CORE_RESTARTS@", shellQuote(plan.CoreRestarts), "@CONTROL_RESTARTS@", shellQuote(plan.ControlRestarts),
		"@ANCHOR_DIR@", shellQuote(plan.AnchorDir), "@ANCHORS@", shellQuote(plan.Anchors),
		"@B64@", shellQuote(rtCoreConfigPayload(src.MachineBytes)),
		"@HISTORY@", rtCoreConfigHistoryDir, "@LOCK@", rtCoreConfigLockPath, "@TELEMETRY@", rtCoreConfigTelemetryDir,
		"@STAGED@", shellQuote(staged), "@STEP@", rtCoreConfigStepFunc, "@ANCHORS_NOW@", rtCoreConfigAnchorsNow,
		"@LAYOUT@", shellQuote(rtCoreConfigAnchorLayout), "@LAYOUT_TEXT@", rtCoreConfigAnchorLayout,
		"@CONTROL_ENV@", rtCoreControlEnvCommandFor(c, staged), "@BUSY_PY@", rtCoreBusyPy,
		"@BRIDGE_B64@", shellQuote(bridgeB64), "@BRIDGE_SHA@", shellQuote(plan.BridgeSHA256), "@BRIDGE_STAGED@", shellQuote(bridgeStaged),
		"@BRIDGE_CONTROL_ENV@", rtCoreControlEnvCommandFor(c, bridgeStaged),
		"@PROMOTED@", shellQuote(rtCorePromotionText(plan.Promotions)+" written RAM-only; the new configuration stores them"))
}

// rtCoreConfigOLPUpdate runs on the relay after the robot verified the new
// configuration. It edits the OLP deployment's own source tree (machine JSON,
// the one cell-manifest pin) and catalogue, restarts the OLP and waits for it
// to list the cell. On failure it restores those files and restarts the OLP,
// which then refuses the cell on its pin check; the robot stays on NEW.
const rtCoreConfigOLPUpdate = `olp_restore(){
  step "olp restore: $1"
  cp -p "$hist/machine.json" "$OLP_SRC/$MACHINE" && cp -p "$hist/cell.json" "$OLP_SRC/$CELL_MANIFEST" && cp -p "$hist/olp-cells.json" "$OLP_CELLS" || step "olp restore copy FAILED; restore by hand from $hist"
  sudo -n systemctl restart "$OLP_UNIT" || step "olp restart after restore failed"
  printf '@@olp restored %s (backup %s)\n' "$1" "$hist"
  exit 5
}
res=$(grep '^@@result ' "$W/apply.out" | tail -n 1 | cut -d' ' -f2-)
ts=$(date -u +%Y%m%dT%H%M%SZ)
hist="$OLP_DIR/config-history/$ts-$OLD"
install -d -m 0700 "$hist" || { printf '@@olp failed cannot create %s; OLP untouched\n' "$hist"; exit 5; }
cp -p "$OLP_SRC/$MACHINE" "$hist/machine.json" && cp -p "$OLP_SRC/$CELL_MANIFEST" "$hist/cell.json" && cp -p "$OLP_CELLS" "$hist/olp-cells.json" || { printf '@@olp failed backup to %s; OLP untouched\n' "$hist"; exit 5; }
step "olp backed up machine, cell manifest and catalogue to $hist"
python3 - "$OLP_SRC/$MACHINE" "$W/machine.json" "$OLP_SRC/$CELL_MANIFEST" "$OLP_CELLS" "$CELL_ID" "$OLD" "$NEW" "$res" <<'ROSIE_OLP_PY' || olp_restore "file update refused"
import json, os, shutil, sys
machine, staged, cell, cells, cell_id, old, new, result = sys.argv[1:9]
res = json.loads(result)
text = open(cell).read()
if text.count(old) != 1:
    sys.exit("OLP cell manifest holds %d occurrences of %s, need exactly 1" % (text.count(old), old))
catalogue = json.load(open(cells))
matches = [x for x in catalogue.get("cells", []) if x.get("cell_id") == cell_id]
if len(matches) != 1:
    sys.exit("OLP catalogue has %d entries for %s" % (len(matches), cell_id))
execution = matches[0].setdefault("execution", {})
binding = execution.setdefault("binding", {})
if binding.get("configuration_sha256") not in (None, "", old):
    sys.exit("OLP catalogue binding pins %s, not %s" % (binding.get("configuration_sha256"), old))
binding["configuration_sha256"] = new
if "machine_sha256" in binding:
    binding["machine_sha256"] = res["machine"]
if "deployment_sha256" in execution:
    execution["deployment_sha256"] = res["deployment"]
if "capabilities_digest" in execution:
    execution["capabilities_digest"] = res["capabilities"]
def replace(path, write):
    tmp = path + ".rosie-config.tmp"
    write(tmp)
    shutil.copymode(path, tmp)
    os.replace(tmp, path)
replace(machine, lambda tmp: shutil.copyfile(staged, tmp))
replace(cell, lambda tmp: open(tmp, "w").write(text.replace(old, new)))
replace(cells, lambda tmp: open(tmp, "w").write(json.dumps(catalogue, indent=2) + "\n"))
ROSIE_OLP_PY
step "olp updated machine JSON, cell pin and catalogue binding to $NEW"
sudo -n systemctl restart "$OLP_UNIT" || olp_restore "restart failed"
ok=0
for i in $(seq 1 60); do
  if curl -fsS -m 2 "http://$OLP_LISTEN/api/offline-programming/v1/health" > /dev/null 2>&1; then ok=1; break; fi
  if [ $((i % 2)) -eq 0 ]; then step "olp waiting for health ${i}s"; fi
  sleep 1
done
[ "$ok" = 1 ] || olp_restore "health not ok within 60s"
curl -fsS -m 5 "http://$OLP_LISTEN/api/offline-programming/v1/targets" > "$W/targets.json" || olp_restore "/targets unavailable"
python3 - "$W/targets.json" "$CELL_ID" <<'ROSIE_TARGETS_PY' || olp_restore "/targets does not list $CELL_ID"
import json, sys
views = [v for v in json.load(open(sys.argv[1])).get("targets", []) if v.get("cell_id") == sys.argv[2]]
if not views:
    sys.exit(1)
print("olp lists %s backend=%s reason=%s" % (sys.argv[2], views[0].get("backend", ""), views[0].get("reason", "") or "none"))
ROSIE_TARGETS_PY
printf '@@olp ok %s (backup %s)\n' "$OLP_UNIT restarted, health ok, /targets lists $CELL_ID" "$hist"
`

// rtCoreConfigApplyCommand is session 2 on the relay: re-check the OLP
// deployment the probe saw, run the servo-host apply, then (only after the
// robot verified, and only with --olp restart) update the OLP.
func rtCoreConfigApplyCommand(c robotV4RTCoreConfig, src rtCoreConfigSource, p rtCoreConfigProbe, plan rtCoreConfigPlan, revpiTarget, runID, olpUnit string, opts rtCoreConfigOptions) string {
	script := rtCoreConfigRelayPrologue(src, revpiTarget, runID, olpUnit) + "trap '' PIPE HUP\nOLD=" + shellQuote(plan.Old) + "; NEW=" + shellQuote(plan.New) + "\n"
	if opts.notice.RunID != "" {
		// The notice is cleared when this script ends, however it ends: the
		// relay finishes or rolls back on its own if the Mac's connection drops.
		script += rtCoreMaintenanceShell(opts.notice) +
			`trap 'maint_clear; ssh -o "ControlPath=$CMP" -O exit -- "$REVPI" > /dev/null 2>&1 || true' EXIT` + "\n"
	} else {
		script += rtCoreMaintenancePhaseDefault
	}
	if opts.OLP == "restart" {
		script += `if [ -n "$OLP_ERR" ] || [ "$OLP_SRC" != ` + shellQuote(p.OLP.Source) + ` ] || [ "$OLP_CELLS" != ` + shellQuote(p.OLP.Catalogue) + ` ]; then
  printf '@@refused %s\n' "OLP deployment changed since the probe (${OLP_SRC:-none} ${OLP_ERR}); robot untouched"; exit 2
fi
`
	}
	script += `maint_phase "restarting the controller with configuration ${NEW:0:12}; verifying every axis and Home"
step "apply relay=$(hostname); servo host apply begins"
rsshs <<'ROSIE_RT_CONFIG_APPLY' | tee -p "$W/apply.out"
` + rtCoreConfigRevPiApply(c, src, plan, p.Release, p.PackageManifestSHA256, opts) + `ROSIE_RT_CONFIG_APPLY
rc=${PIPESTATUS[0]}
if [ "$rc" != 0 ]; then exit "$rc"; fi
`
	if opts.OLP == "restart" {
		script += "maint_phase \"controller verified on ${NEW:0:12}; updating and restarting the OLP server\"\n" + rtCoreConfigOLPUpdate
	} else {
		script += "printf '@@olp skipped %s\\n' '--olp skip: OLP tree, catalogue and unit untouched'\n"
	}
	return script + "exit 0\n"
}

// rtCoreConfigRemoteArgv reaches the relay. Tailscale SSH authenticates by
// tailnet identity, so it works while the Mac's key is passphrase-locked; it
// passes the options after the destination to the real ssh. Both carry the
// keepalives (sshAliveArgs): every relay session, the lock holder, uploads
// and fetches end within ~15 s of their path dying instead of hanging.
func rtCoreConfigRemoteArgv(transport, relayUser, relayPeer, relayHost, script string) []string {
	remote := "bash -c " + shellQuote(script)
	if transport == "tailscale" {
		return append(append([]string{"tailscale", "ssh", remoteTarget(relayUser, relayPeer)}, sshRelayAliveArgs(sshConnectTimeout)...), remote)
	}
	return append(append([]string{"ssh", "-o", "BatchMode=yes"}, sshRelayAliveArgs(sshConnectTimeout)...), "--", remoteTarget(relayUser, relayHost), remote)
}
