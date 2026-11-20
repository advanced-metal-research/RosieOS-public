#!/usr/bin/env bash
# Exercise installed releases and public recovery in isolation; no command may activate host hardware.
set -euo pipefail
export LC_ALL=C
component="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${component}"
fail() { printf 'lifecycle-test error=%s\n' "$*" >&2; exit 1; }
mkdir -p build "${TMPDIR:?}"
scratch="$(mktemp -d "${component}/build/lifecycle-test.XXXXXX")"
root="" runtime="" core_pid="" adapter_pid=""
cleanup() {
  local status=$?
  trap - EXIT
  if [[ -n "${adapter_pid}" ]]; then kill "${adapter_pid}" 2>/dev/null || :; wait "${adapter_pid}" 2>/dev/null || :; fi
  if [[ -n "${core_pid}" ]]; then kill "${core_pid}" 2>/dev/null || :; wait "${core_pid}" 2>/dev/null || :; fi
  if [[ ${status} != 0 ]]; then
    for log in status core adapter; do [[ ! -f "${scratch}/${log}.log" ]] || tail -30 "${scratch}/${log}.log" >&2; done
  fi
  [[ -z "${root}" ]] || { chmod u+w "${root}/etc" 2>/dev/null || :; rm -rf -- "${root}"; }
  [[ -z "${runtime}" ]] || rm -rf -- "${runtime}"
  rm -rf -- "${scratch}"
  exit "${status}"
}
trap cleanup EXIT
# Unverified shared-tmpfs admission policy, in KB; package trees stay on the build filesystem.
needed_kb=65536
for attempt in $(seq 1 45); do
  available_kb="$(df --output=avail -k "${TMPDIR}" | tail -1 | tr -d ' ')"
  [[ "${available_kb}" -ge "${needed_kb}" ]] && break
  [[ "${attempt}" -ne 1 ]] || printf 'lifecycle-test: waiting for %s KB free under %s\n' "${needed_kb}" "${TMPDIR}" >&2
  sleep 2
done
[[ "${available_kb}" -ge "${needed_kb}" ]] || fail "TMPDIR needs ${needed_kb} KB free; observed ${available_kb} KB"
root="$(mktemp -d "${TMPDIR}/rosie-rt-install-test.XXXXXX")"
runtime="$(mktemp -d "${TMPDIR}/lc.XXXXXX")"
mkdir -p "${scratch}/opt" "${root}/etc" "${root}/accounts" "${runtime}/native" "${runtime}/public"
ln -s "${scratch}/opt" "${root}/opt"
for script in host/install.sh host/lifecycle-test.sh; do bash -n "${script}"; done
[[ -x build/rt-control && -x build/rtctl ]] || make control
[[ -x build/rosie-rt-core-sim && -f build/obj/sim/main.o ]] || make sim
[[ -f build/deps/igh-prefix/lib/libethercat.so.1 ]] || bash tools/build-igh-userlib.sh

# The helper hashes names, types, modes, symlink targets and bytes, including the
# build-backed opt tree. JSON assertions use the public documents, never native IPC.
cat > "${scratch}/check.go" <<'GO'
package main
import (
 "context"
 "crypto/sha256"
 "encoding/json"
 "fmt"
 "io"
 "net"
 "net/http"
 "os"
 "path/filepath"
 "strings"
 "syscall"
 "time"
)
func must(err error) { if err != nil { panic(err) } }
func read(path string) any { b,e:=os.ReadFile(path); must(e); var v any; must(json.Unmarshal(b,&v)); return v }
func field(v any, path string) any { for _,p:=range strings.Split(path,".") { v=v.(map[string]any)[p] }; return v }
func require(ok bool, message string) { if !ok { panic(message) } }
func main() {
 switch os.Args[1] {
 case "digest":
  h:=sha256.New()
  var walk func(string,string)
  walk=func(path,rel string) {
   st,e:=os.Lstat(path); must(e)
   fmt.Fprintf(h,"%q %v\n",rel,st.Mode())
   stat:=st.Sys().(*syscall.Stat_t); fmt.Fprintf(h,"uid=%d gid=%d inode=%d\n",stat.Uid,stat.Gid,stat.Ino)
   if st.Mode()&os.ModeSymlink!=0 { target,e:=os.Readlink(path); must(e); fmt.Fprintf(h,"link %q\n",target); st,e=os.Stat(path); must(e) }
   if st.IsDir() { entries,e:=os.ReadDir(path); must(e); for _,entry:=range entries { walk(filepath.Join(path,entry.Name()),filepath.Join(rel,entry.Name())) } } else if st.Mode().IsRegular() { f,e:=os.Open(path); must(e); _,e=io.Copy(h,f); must(e); must(f.Close()) }
  }
  walk(os.Args[2],"."); fmt.Printf("%x\n",h.Sum(nil))
 case "field": fmt.Println(field(read(os.Args[2]),os.Args[3]))
 case "argv": for _,arg:=range read(os.Args[2]).([]any) { fmt.Printf("%s%c",arg.(string),0) }
 case "machine":
  v:=read(os.Args[2]).(map[string]any)
  v["backend"]="live"
  // Unverified installation fixture: 10:1 software ratio, 131072 motor counts/rev, drive factor 1/1.
  for _,axis:=range v["axes"].([]any) { a:=axis.(map[string]any); a["profile"]="a6ec"; a["wire_counts_per_rev"]=131072; a["wire_revs_per_axis_rev"]=10 }
  if os.Args[4]=="invalid" { v["cycle_ns"]=0 }
  b,e:=json.MarshalIndent(v,"","  "); must(e); must(os.WriteFile(os.Args[3],append(b,'\n'),0600))
 case "status":
  v:=read(os.Args[2])
  require(field(v,"core.backend")=="simulation","unexpected runtime backend")
  require(field(v,"core.armed")==float64(0)&&field(v,"core.axis_enable_mask")==float64(0),"startup enabled motion")
  require(len(field(v,"core.axes").([]any))==9,"status axis count")
 case "incarnations":
  // Unverified public HTTP observation deadline, in seconds; no native IPC is used.
  client:=http.Client{Timeout:5*time.Second,Transport:&http.Transport{DialContext:func(ctx context.Context,_,_ string)(net.Conn,error){ return (&net.Dialer{}).DialContext(ctx,"unix",os.Args[2]) }}}
  response,e:=client.Get("http://localhost/v1/status"); must(e); defer response.Body.Close()
  require(response.StatusCode==200,"public status HTTP code")
  var envelope any; must(json.NewDecoder(response.Body).Decode(&envelope)); v:=field(envelope,"data")
  for _,key:=range []string{"daemon_incarnation","adapter_incarnation"} { s,ok:=field(v,key).(string); require(ok&&len(s)==32,"missing process incarnation: "+key) }
  must(json.NewEncoder(os.Stdout).Encode(v))
 case "hostcheck":
  v:=read(os.Args[2]); require(field(v,"exit_code")==float64(2),"hostcheck exit code")
  found:=map[string]bool{}
  for _,row:=range field(v,"checks").([]any) { r:=row.(map[string]any); if r["status"]=="not_applicable" { found[r["id"].(string)]=true } }
  for _,id:=range []string{"timing_qualification","timer_slack","timer_resolution"} { require(found[id],"dishonest WSL timing row: "+id) }
 default: panic("unknown assertion")
 }
}
GO
go build -o "${scratch}/check" "${scratch}/check.go"
check="${scratch}/check"

# Build two actual simulation executables with distinct recorded source identities.
# The packaging tool currently requires git: this confined shim supplies only
# identity queries from the copied source's SOURCE_IDENTITY, without editing git.
fixture="${scratch}/source"
mkdir -p "${fixture}/build" "${scratch}/source-git" "${scratch}/mock"
cp Makefile go.mod go.sum "${fixture}/"
for directory in src include engine tools host config ipcclient sdk adapters internal; do cp -R "${directory}" "${fixture}/"; done
# Preserve the repository's sibling resource/module boundary: go.mod replaces
# rosieos/robotdesc with ../robot_description/go, and packaging reads robots there.
mkdir -p "${scratch}/robot_description"
cp -R ../robot_description/robots ../robot_description/go "${scratch}/robot_description/"
mkdir -p "${fixture}/clients/cpp"
cp -R clients/cpp/include "${fixture}/clients/cpp/"
cp build/rtctl build/rt-control "${fixture}/build/"
ln -s "${component}/build/deps" "${fixture}/build/deps"
# The simulation object does not depend on the source identity: copied after the
# sources, a current one leaves each fixture build compiling only src/version.cpp
# and linking, instead of two serial main.cpp compiles (~95 s each on the relay).
if make --no-print-directory -q build/obj/sim/main.o 2>/dev/null; then
  mkdir -p "${fixture}/build/obj/sim"
  cp build/obj/sim/main.o "${fixture}/build/obj/sim/main.o"
  touch "${fixture}/build/obj/sim/main.o"
fi
cat > "${scratch}/source-git/git" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
case "$*" in
  'rev-parse --git-dir') printf '.\n' ;;
  'rev-parse HEAD'|'describe --always --dirty') cat "${LIFECYCLE_SOURCE}/SOURCE_IDENTITY" ;;
  'status --porcelain --untracked-files=normal') ;;
  *) printf 'unexpected fixture git query: %s\n' "$*" >&2; exit 97 ;;
esac
SH
chmod +x "${scratch}/source-git/git"
# Measured fixture identity width: package revision directories contain 40 hex digits.
revision_a=aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
revision_b=bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
for revision in "${revision_a}" "${revision_b}"; do
  printf '%s\n' "${revision}" > "${fixture}/SOURCE_IDENTITY"
  (
    export PATH="${scratch}/source-git:${PATH}" LIFECYCLE_SOURCE="${fixture}" GOFLAGS="${GOFLAGS:-} -buildvcs=false"
    make -C "${fixture}" sim
    ROSIE_RT_PACKAGE_CORE=build/rosie-rt-core-sim bash "${fixture}/tools/package-runtime.sh" "${scratch}/${revision}"
  ) > "${scratch}/package-${revision}.log" 2>&1 || { tail -30 "${scratch}/package-${revision}.log"; fail 'fixture package build'; }
  [[ "$("${check}" field "${scratch}/${revision}/rt_package.json" backend)" == simulation ]] || fail 'package backend'
  [[ "$("${scratch}/${revision}/bin/rosie-rt-core" --version)" == "${revision}" ]] || fail 'package source identity'
done
package_a="${scratch}/${revision_a}" package_b="${scratch}/${revision_b}"
cp "${fixture}/build/rosie-rt-core-sim" "${fixture}/build/rosie-rt-core"

cat > "${scratch}/mock/dispatch" <<'SH'
#!/usr/bin/env bash
set -euo pipefail
command="${0##*/}"
printf '%s %s\n' "${command}" "$*" >> "${MOCK_LOG}"
case "${command}" in
  getent) [[ $# == 2 && ( "$1" == group || "$1" == passwd ) ]] || exit 97; [[ -f "${MOCK_ACCOUNTS}/$1.$2" ]] || exit 2; cat "${MOCK_ACCOUNTS}/$1.$2" ;;
  groupadd) [[ $# == 2 && "$1" == --system ]] || exit 97; printf '%s:x:123:\n' "$2" > "${MOCK_ACCOUNTS}/group.$2" ;;
  useradd)
    [[ $# == 11 && "$1" == --system && "$2" == --gid && "$4" == --groups && "$6" == --home-dir && "$7" == /nonexistent && "$8" == --no-create-home && "$9" == --shell && "${10}" == /usr/sbin/nologin ]] || exit 97
    printf '%s:x:123:123::/nonexistent:/usr/sbin/nologin\n' "${11}" > "${MOCK_ACCOUNTS}/passwd.${11}"
    printf '%s\n' "$3" > "${MOCK_ACCOUNTS}/primary.${11}"
    printf '%s %s\n' "$3" "${5//,/ }" > "${MOCK_ACCOUNTS}/groups.${11}" ;;
  id) case "$1" in -gn) cat "${MOCK_ACCOUNTS}/primary.$2";; -Gn) cat "${MOCK_ACCOUNTS}/groups.$2";; *) exit 97;; esac ;;
  install)
    args=()
    while [[ $# -gt 0 ]]; do case "$1" in -o|-g) shift 2;; *) args+=("$1"); shift;; esac; done
    for arg in "${args[@]}"; do
      [[ "${arg}" != /* || "${arg}" == "${MOCK_SCRATCH}/"* || "${arg}" == "${ROSIE_RT_INSTALL_TEST_ROOT}/"* || "${arg}" == /dev/fd/* ]] || exit 97
    done
    [[ "${args[-1]}" == "${ROSIE_RT_INSTALL_TEST_ROOT}/"* ]] || exit 97
    exec /usr/bin/install "${args[@]}" ;;
  chown) [[ "${!#}" == "${ROSIE_RT_INSTALL_TEST_ROOT}/"* ]] || exit 97 ;;
  systemctl)
    case "$*" in
      daemon-reload) ;;
      'enable rosie-rt-core.service rosie-rt-control.service') [[ "${MOCK_ALLOW_ENABLE:-0}" == 1 ]] || exit 97 ;;
      *) exit 97 ;;
    esac ;;
  rtctl) [[ "${MOCK_COMPILE_FAIL:-0}" == 0 ]] || { echo 'compiler refused fixture' >&2; exit 17; }; exec "${MOCK_RTCTL}" "$@" ;;
  ldd) printf 'libethercat.so.1 => %s (0x1)\n' "${MOCK_LIBRARY}" ;;
  *) exit 97 ;;
esac
SH
chmod +x "${scratch}/mock/dispatch"
for command in getent groupadd useradd id install chown systemctl rtctl ldd; do ln -s dispatch "${scratch}/mock/${command}"; done
export PATH="${scratch}/mock:${PATH}" ROSIE_RT_INSTALL_TEST_ROOT="${root}"
export MOCK_LOG="${scratch}/calls.log" MOCK_ACCOUNTS="${root}/accounts" MOCK_SCRATCH="${scratch}"
export MOCK_LIBRARY="${fixture}/build/deps/igh-prefix/lib/libethercat.so.1"
export MOCK_RTCTL="${component}/build/rtctl"
unset ROSIE_RT_ETHERCAT_GROUP ECRT_LICENSE_DIR ROSIE_RT_HOSTCHECK_ROOT
: > "${MOCK_LOG}"
expect_status() {
  local expected="$1" observed=0; shift
  "$@" > "${scratch}/status.log" 2>&1 || observed=$?
  [[ "${observed}" == "${expected}" ]] || fail "expected exit ${expected}; observed ${observed}: $*"
}
no_activation() {
  if grep -Eq '^systemctl (enable|start|restart)|^systemctl .*--now' "${MOCK_LOG}"; then fail 'unexpected service activation'; fi
}
unchanged_failure() {
  local message="$1" before; shift
  before="$("${check}" digest "${root}")"
  expect_status 1 "$@"
  grep -Fq -- "${message}" "${scratch}/status.log" || fail "missing refusal: ${message}"
  [[ "$("${check}" digest "${root}")" == "${before}" ]] || fail "refusal mutated fake root: ${message}"
  no_activation
}
# Measured adapter binding: positive uint64 revision 42; the dense axis map is the description's.
"${check}" machine config/machines/simulation/simulation-program.json "${scratch}/machine.json" valid
"${check}" machine config/machines/simulation/simulation-program.json "${scratch}/invalid.json" invalid
binding=("${scratch}/machine.json" lifecycle-pair 42)
installer="${fixture}/host/install.sh"
unchanged_failure 'missing or unreadable machine configuration' bash "${installer}" --enable "${scratch}/missing.json" lifecycle-pair 42
expect_status 1 build/rtctl validate --config "${scratch}/invalid.json" --out "${scratch}/validate" --backend simulation --binary build/rosie-rt-core-sim
grep -Fq cycle_ns "${scratch}/status.log" || fail 'invalid configuration diagnostic'
unchanged_failure 'configuration compilation failed' bash "${installer}" --enable "${scratch}/invalid.json" lifecycle-pair 42
unchanged_failure 'configuration compilation failed; control.env was not replaced' env MOCK_COMPILE_FAIL=1 bash "${installer}" --enable "${binding[@]}"
unchanged_failure 'missing remote PKI directory' bash "${installer}" --remote-pki "${scratch}/absent-pki" "${binding[@]}"
cp -R "${package_a}" "${scratch}/tampered"
printf 'tampered\n' >> "${scratch}/tampered/config/machines/simulation/simulation-program.json"
unchanged_failure 'package verification failed; installation refused' bash "${package_a}/host/install.sh" --from-package "${scratch}/tampered"
unchanged_failure '--live requires a live package; simulation package refused' bash "${package_a}/host/install.sh" --live --from-package "${package_a}"
chmod 0555 "${root}/etc"
[[ ! -w "${root}/etc" ]] || fail 'permission scenario requires an unprivileged test user'
unchanged_failure "permission denied creating ${root}/etc/rosie-rt-core" bash "${package_a}/host/install.sh" --from-package "${package_a}"
chmod 0755 "${root}/etc"

expect_status 0 bash "${package_a}/host/install.sh" --from-package "${package_a}"
no_activation
store="${root}/opt/rosie-rt-core"
[[ "$(readlink "${store}/current")" == "${revision_a}" ]] || fail 'fresh current selection'
[[ "$(find "${root}/accounts" -name 'group.*' | wc -l)" == 4 && "$(find "${root}/accounts" -name 'passwd.*' | wc -l)" == 2 ]] || fail 'service identities'
grep -Fxq rosie-rt "${root}/accounts/primary.rosie-rt" || fail 'daemon primary group'
grep -Fxq 'rosie-rt rosie-ctl ethercat' "${root}/accounts/groups.rosie-rt" || fail 'daemon supplementary groups'
grep -Fxq 'rosie-ctl rosie-rt-clients' "${root}/accounts/groups.rosie-ctl" || fail 'adapter supplementary groups'
[[ ! -e "${root}/etc/rosie-rt-core/control.env" ]] || fail 'inactive package invented a machine binding'
for unit in rosie-rt-core rosie-rt-control; do
  file="${root}/etc/systemd/system/${unit}.service"
  cmp "${store}/current/host/${unit}.service" "${file}"
  grep -Fq 'ExecStart=/opt/rosie-rt-core/current/bin/' "${file}" || fail 'unit executable'
  grep -Fxq 'WorkingDirectory=/opt/rosie-rt-core/current' "${file}" || fail 'unit working directory'
  grep -Fxq 'EnvironmentFile=/etc/rosie-rt-core/control.env' "${file}" || fail 'unit binding'
  [[ "$(stat -c %a "${file}")" == 644 ]] || fail 'unit permissions'
done
expect_status 0 bash "${package_a}/host/install.sh" --from-package "${package_a}"
[[ "$(grep -c '^groupadd ' "${MOCK_LOG}")" == 4 && "$(grep -c '^useradd ' "${MOCK_LOG}")" == 2 ]] || fail 'idempotent identity creation'
no_activation
expect_status 0 bash "${package_b}/host/install.sh" --from-package "${package_b}"
[[ "$(readlink "${store}/current")" == "${revision_b}" && -d "${store}/${revision_a}" ]] || fail 'upgrade selection or retention'
no_activation
expect_status 0 bash "${store}/current/host/install.sh" --rollback
[[ "$(readlink "${store}/current")" == "${revision_a}" && -d "${store}/${revision_b}" ]] || fail 'rollback selection or retention'
no_activation
unchanged_failure 'no earlier release for rollback' bash "${store}/current/host/install.sh" --rollback
for revision in "${revision_a}" "${revision_b}"; do expect_status 0 bash "${store}/${revision}/tools/verify-package.sh" "${store}/${revision}"; done

# Prune keeps current (a), previous (b) and pinned (d); it removes the unselected
# release c and an interrupted prune's leftovers, never a symlink, its target
# outside the store, or an unknown name. Pruning is idempotent and inert.
revision_c=cccccccccccccccccccccccccccccccccccccccc revision_d=dddddddddddddddddddddddddddddddddddddddd
revision_e=eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee revision_f=ffffffffffffffffffffffffffffffffffffffff
pruner="${store}/current/host/install.sh"
cp -R "${store}/${revision_a}" "${store}/${revision_c}"
cp -R "${store}/${revision_a}" "${store}/${revision_d}"
mkdir -p "${scratch}/outside" "${store}/.prune.interrupted/release"
printf 'outside\n' > "${scratch}/outside/marker"
ln -s "${scratch}/outside" "${store}/${revision_e}"
printf 'operator note\n' > "${store}/notes"
unchanged_failure "cannot pin ${revision_f}: not a stored release" bash "${pruner}" --pin "${revision_f}"
unchanged_failure '--prune and --pin cannot be combined with installation options' bash "${pruner}" --prune --rollback
expect_status 0 bash "${pruner}" --pin "${revision_d}"
expect_status 0 bash "${pruner}" --pin "${revision_d}"
pin_contents="$(cat "${store}/pinned")"
pin_mode="$(stat -c %a "${store}/pinned")"
[[ "${pin_contents}" == "${revision_d}" ]] || fail "pin list contents=${pin_contents@Q}"
if [[ "${pin_mode}" != 644 ]]; then
  [[ "$(stat -f -c %T "${store}")" == v9fs ]] || fail "pin list mode=${pin_mode}"
  grep -Fq 'chmod 0644 "${pending}"' "${pruner}" || fail 'pin list chmod contract missing'
fi
expect_status 0 bash "${pruner}" --prune
for line in "prune kept=${revision_a} reason=current" "prune kept=${revision_b} reason=previous" "prune kept=${revision_d} reason=pinned" \
  "prune removed=${revision_c}" 'prune removed=.prune.interrupted reason=interrupted-prune' 'prune status=done removed=1 kept=3'; do
  grep -Fq -- "${line}" "${scratch}/status.log" || fail "prune report: ${line}"
done
[[ ! -e "${store}/${revision_c}" && ! -e "${store}/.prune.interrupted" && -z "$(find "${store}" -maxdepth 1 -name '.prune.*' -print -quit)" ]] || fail 'prune removal'
[[ -d "${store}/${revision_a}" && -d "${store}/${revision_b}" && -d "${store}/${revision_d}" && -f "${store}/notes" ]] || fail 'prune retention'
[[ -L "${store}/${revision_e}" && -f "${scratch}/outside/marker" ]] || fail 'prune followed a symlink'
[[ "$(readlink "${store}/current")" == "${revision_a}" && "$(readlink "${store}/previous")" == "${revision_b}" ]] || fail 'prune changed the selection'
before="$("${check}" digest "${root}")"
expect_status 0 bash "${pruner}" --prune
grep -Fq 'prune status=done removed=0 kept=3' "${scratch}/status.log" || fail 'repeated prune'
[[ "$("${check}" digest "${root}")" == "${before}" ]] || fail 'repeated prune mutated the store'
no_activation
rm -rf -- "${store}/${revision_d}" "${store}/${revision_e}" "${store}/notes" "${store}/pinned"

# Generate the real hardware binding separately, then use a separately compiled
# simulation binding for isolated startup. Installed units are never started.
expect_status 0 bash "${fixture}/host/generate-control-env.sh" "${binding[@]}"
environment="${root}/etc/rosie-rt-core/control.env"
[[ "$(stat -c %a "${environment}")" == 640 ]] || fail 'control.env permissions'
unchanged_failure 'configuration compilation failed; control.env was not replaced' env MOCK_COMPILE_FAIL=1 bash "${installer}" --enable "${binding[@]}"
unchanged_failure 'configuration compilation failed; control.env was not replaced' env MOCK_COMPILE_FAIL=1 bash "${fixture}/host/generate-control-env.sh" "${binding[@]}"
binary="${store}/current/bin/rosie-rt-core"
adapter="${store}/current/bin/rt-control"
rtctl="${store}/current/bin/rtctl"
compiled="$("${rtctl}" compile --config "${store}/current/config/machines/simulation/simulation-program.json" --out "${scratch}/compiled")"
hash="${compiled##*/}"
mapfile -d '' -t core_args < <("${check}" argv "${compiled}/argv.json")
core_args+=(--socket-path "${runtime}/native/ipc.sock")
adapter_args=(--core-socket "${runtime}/native/ipc.sock" --socket "${runtime}/public/control.sock" --backend simulation --pair-id lifecycle-pair --pair-revision 42)
printf 'ROSIE_RT_PAIR_ID=lifecycle-pair\nROSIE_RT_PAIR_REVISION=42\nROSIE_RT_CONFIGURATION_SHA256=%s\nROSIE_RT_CORE_ARGS="%s"\n' \
  "${hash}" "${core_args[*]}" > "${scratch}/simulation.env"
install -m 0640 "${scratch}/simulation.env" "${environment}"
assert_no_sockets() { [[ -z "$(find "${runtime}" -type s -print -quit)" ]] || fail 'failed startup left a socket'; }
expect_status 2 "${binary}" "${core_args[@]}" --configuration-hash wrong
assert_no_sockets
expect_status 2 "${binary}" "${core_args[@]}" --axis-profiles "${scratch}/missing-profile.conf"
grep -Fq 'invalid --axis-profiles file' "${scratch}/status.log" || fail 'missing-profile diagnostic'
assert_no_sockets
"${binary}" "${core_args[@]}" > "${scratch}/core.log" 2>&1 & core_pid=$!
# Unverified harness deadlines in seconds; successful public responses, never elapsed sleeps, establish readiness.
ready_budget=20
wait_native() {
  local deadline=$((SECONDS + ready_budget))
  until [[ -S "${runtime}/native/ipc.sock" ]]; do
    kill -0 "${core_pid}" 2>/dev/null || fail 'simulation daemon exited'
    [[ ${SECONDS} -lt ${deadline} ]] || fail 'native startup deadline'
    sleep 0.02
  done
}
wait_native
# Measured SHA-256 width: the alternate 64-hex digest must be refused by the real adapter.
wrong_hash=0000000000000000000000000000000000000000000000000000000000000000
sed "s/^ROSIE_RT_CONFIGURATION_SHA256=.*/ROSIE_RT_CONFIGURATION_SHA256=${wrong_hash}/" "${scratch}/simulation.env" > "${environment}"
source "${environment}"
expect_status 1 "${adapter}" "${adapter_args[@]}" --configuration-sha256 "${ROSIE_RT_CONFIGURATION_SHA256}"
grep -Fq 'configuration/backend identity mismatch' "${scratch}/status.log" || fail 'mismatched binding diagnostic'
[[ -z "$(find "${runtime}/public" -type s -print -quit)" ]] || fail 'rejected adapter left a socket'
expect_status 2 "${adapter}" "${adapter_args[@]}" --configuration-sha256 "${hash}" --remote-listen 127.0.0.1:0 \
  --remote-ca "${scratch}/absent-pki/ca.pem" --remote-cert "${scratch}/absent-pki/server.pem" \
  --remote-key "${scratch}/absent-pki/server-key.pem" --remote-crl "${scratch}/absent-pki/crl.pem"
grep -Fq 'no such file or directory' "${scratch}/status.log" || fail 'missing PKI startup diagnostic'
[[ -z "$(find "${runtime}/public" -type s -print -quit)" ]] || fail 'PKI refusal left a socket'
install -m 0640 "${scratch}/simulation.env" "${environment}"
source "${environment}"
start_adapter() {
  "${adapter}" "${adapter_args[@]}" --configuration-sha256 "${ROSIE_RT_CONFIGURATION_SHA256}" > "${scratch}/adapter.log" 2>&1 & adapter_pid=$!
  local deadline=$((SECONDS + ready_budget))
  until "${rtctl}" status --socket "${runtime}/public/control.sock" --json --timeout 1s > "${scratch}/public-status.json" 2> "${scratch}/poll.log"; do
    kill -0 "${adapter_pid}" 2>/dev/null || fail 'adapter exited'
    [[ ${SECONDS} -lt ${deadline} ]] || fail 'public startup deadline'
    sleep 0.02
  done
}
start_adapter
"${check}" status "${scratch}/public-status.json"
"${check}" incarnations "${runtime}/public/control.sock" > "${scratch}/incarnations.json"
before="$("${check}" digest "${root}")"
socket_before="$("${check}" digest "${runtime}/public")"
expect_status 3 "${adapter}" "${adapter_args[@]}" --configuration-sha256 "${hash}"
grep -Fq 'owned' "${scratch}/status.log" || fail 'occupied socket diagnostic'
[[ "$("${check}" digest "${root}")" == "${before}" && "$("${check}" digest "${runtime}/public")" == "${socket_before}" ]] || fail 'competing adapter changed installed tree or live socket'
expect_status 0 "${rtctl}" status --socket "${runtime}/public/control.sock" --json
"${check}" status "${scratch}/status.log"
expect_status 2 "${rtctl}" hostcheck --json
"${check}" hostcheck "${scratch}/status.log"
expect_status 0 "${rtctl}" describe --socket "${runtime}/public/control.sock" --json
grep -Fq "${hash}" "${scratch}/status.log" || fail 'public description configuration identity'
# Package identity is currently exposed by the installed executable, not the public description.
[[ "$("${binary}" --version)" == "$("${check}" field "${store}/current/rt_package.json" build_inputs.daemon_version)" ]] || fail 'installed package identity'
expect_status 0 "${rtctl}" acquire --socket "${runtime}/public/control.sock" --json --pair-id lifecycle-pair --pair-revision 42 --configuration-sha256 "${hash}"
session="$("${check}" field "${scratch}/status.log" data.session)"
generation="$("${check}" field "${scratch}/status.log" data.generation)"
expect_status 0 "${rtctl}" stop --socket "${runtime}/public/control.sock" --json --session "${session}" --generation "${generation}"
expect_status 0 "${rtctl}" status --socket "${runtime}/public/control.sock" --json
"${check}" status "${scratch}/status.log"
no_activation
old_adapter="$("${check}" field "${scratch}/incarnations.json" adapter_incarnation)"
old_daemon="$("${check}" field "${scratch}/incarnations.json" daemon_incarnation)"
kill -KILL "${adapter_pid}"
observed=0
wait "${adapter_pid}" 2>/dev/null || observed=$?
adapter_pid=""
[[ "${observed}" == 137 && -S "${runtime}/public/control.sock" && -S "${runtime}/public/jog.sock" ]] || fail 'crash fixture did not retain owned sockets'
start_adapter
"${check}" incarnations "${runtime}/public/control.sock" > "${scratch}/restarted.json"
[[ "$("${check}" field "${scratch}/restarted.json" daemon_incarnation)" == "${old_daemon}" && "$("${check}" field "${scratch}/restarted.json" adapter_incarnation)" != "${old_adapter}" ]] || fail 'restart incarnation fencing'
kill "${adapter_pid}"; wait "${adapter_pid}"; adapter_pid=""
kill "${core_pid}"; wait "${core_pid}"; core_pid=""
assert_no_sockets
printf 'lifecycle-test status=passed contracts=packages,identities,units,upgrade,rollback-once,prune,atomic-refusals,permissions,compiler,pki,hash,profile,socket-ownership,stale-socket-recovery,public-recovery public-package-identity=unavailable rtctl-incarnations=unavailable\n'
