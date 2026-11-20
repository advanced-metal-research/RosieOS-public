package main

import (
	"strconv"
)

// Part import on the relay. The OLP's CadQuery worker
// (offline-programming/v1/internal/cad) runs cadquery/v1's scripts on the
// cadquery/v1 pixi env. Until 2026-11-04 no command provisioned that env: no
// deployment tree had source/cadquery/v1/.pixi, the worker fell back to
// `pixi run` with the relay's own pixi, and cadquery/v1/pixi.toml did not
// list linux-aarch64, so on the mini bench's Jetson every Import a part
// answered 502 worker_failed ("unsupported-platform ... The workspace does
// not support 'linux-aarch64'") while every check the deploy runs passed. The
// OLPs of two Rosie 1430 cells run on Jetsons too. Now every build provisions the env
// for its tree and proves it on the part the repo ships, the OLP unit names
// the env's python (CADQUERY_TOPOLOGY_PYTHON, so the worker never runs a
// relay pixi: a host's pixi 0.67.2 re-solves a format v7 lock from the
// network, README "Build"), and olp install refuses a tree without that proof.

// cellDeployCadQueryPixiEnv is cadquery/v1's only pixi environment.
const cellDeployCadQueryPixiEnv = "default"

// cellDeployCadQueryEnvMinFreeMiB refuses an env install that would fill the
// relay: the linux-64 env installed is 2.5 GiB and its package cache 0.5 GiB
// (pixi 0.81.0 --locked into an empty cache, WSL, 2026-11-04, 644 s); the
// linux-aarch64 env, 308 packages against linux-64's 275, is unmeasured.
const cellDeployCadQueryEnvMinFreeMiB = 6144

// cellDeployCadQueryPython is the python the OLP unit of a deployment tree
// imports parts with: the tree's own env link, so the unit names its tree.
func cellDeployCadQueryPython(deployment string) string {
	return deployment + "/source/cadquery/v1/.pixi/envs/" + cellDeployCadQueryPixiEnv + "/bin/python"
}

// cellDeployPinnedPixiShell defines pixi_pinned: PIXI is cellDeployPixiVersion
// under the cell's cache dir, fetched and sha256-checked when missing or not
// the pinned bytes. Both pixi envs the build installs use it (the managed
// planner's and CadQuery's), so a host's own pixi never reads a lock.
func cellDeployPinnedPixiShell() string {
	aarch64, x86 := cellDeployPixiPins["aarch64"], cellDeployPixiPins["x86_64"]
	return rtCoreConfigReplace(`pixi_pinned(){ # sets PIXI to the pinned pixi under $C
  local purl psha tool
  for tool in curl sha256sum; do command -v "$tool" > /dev/null || fail "$tool not found on the relay (fetching the pinned pixi needs it)"; done
  case "$(uname -m)" in
    aarch64) purl=@PIXI_URL_AARCH64@; psha=@PIXI_SHA_AARCH64@ ;;
    x86_64) purl=@PIXI_URL_X86_64@; psha=@PIXI_SHA_X86_64@ ;;
    *) fail "no pinned pixi for $(uname -m)" ;;
  esac
  PIXI="$C/pixi-"@PIXI_VERSION@"/pixi"
  if [ ! -x "$PIXI" ] || [ "$(sha256sum < "$PIXI" | cut -d' ' -f1)" != "$psha" ]; then
    install -d -m 0700 "${PIXI%/pixi}" && curl -fsSL --retry 3 -m 600 -o "$PIXI.part" "$purl" || fail "download of pixi "@PIXI_VERSION@" from $purl"
    [ "$(sha256sum < "$PIXI.part" | cut -d' ' -f1)" = "$psha" ] || { rm -f -- "$PIXI.part"; fail "pixi from $purl is not sha256 $psha"; }
    chmod 0755 "$PIXI.part" && mv -f -- "$PIXI.part" "$PIXI" || fail "cannot install $PIXI"
  fi
}
`, "@PIXI_VERSION@", shellQuote(cellDeployPixiVersion),
		"@PIXI_URL_AARCH64@", shellQuote(aarch64[0]), "@PIXI_SHA_AARCH64@", shellQuote(aarch64[1]),
		"@PIXI_URL_X86_64@", shellQuote(x86[0]), "@PIXI_SHA_X86_64@", shellQuote(x86[1]))
}

// cellDeployCadQueryEnvShell provisions the deployment's
// source/cadquery/v1/.pixi, for every cell (every cell_deploy has an OLP)
// and without a flag: an OLP that cannot import a part is a broken OLP, and
// the env changes only with cadquery/v1's pixi.toml or pixi.lock. Key =
// sha256(pixi.toml, pixi.lock, env, pinned pixi version), the managed planner
// env's key. When the previous tree's record holds the same key and its env
// has a python, the deployment links that env (resolved, so the link names
// the tree that installed it); otherwise the pinned pixi installs it --locked
// into the deployment (pixi checks every locked package's sha256; a build
// that stopped half way resumes). Then, every build (a JetPack update can
// break a reused env), the env's python, run as the OLP runs it (directly,
// in cadquery/v1, no pixi activation), must import cadquery and extract the
// seam topology of the repo's dog_box.step (128 edges, 10 recommended seams,
// linux-64, 2026-11-04). That run also warms the env: a first CadQuery pixi
// run has been seen to exceed the OLP's 90 s import deadline. Its record,
// $D/cadquery-env.json, written only when both pass, is what olp install
// requires and what the next build reuses by.
func cellDeployCadQueryEnvShell() string {
	return rtCoreConfigReplace(`# ---- CadQuery environment (part import): every build ----
step "phase cadquery-env"
CQ="$D/source/cadquery/v1"; CQ_ENV=@CQ_ENV@; CQ_PIXI=@PIXI_VERSION@
[ -f "$CQ/pixi.toml" ] && [ -f "$CQ/pixi.lock" ] || fail "no cadquery/v1/pixi.toml and pixi.lock in $D/source"
CQ_KEY=$( { cat "$CQ/pixi.toml"; printf '\0'; cat "$CQ/pixi.lock"; printf '\0%s\0%s' "$CQ_ENV" "$CQ_PIXI"; } | sha256sum | cut -c1-16)
CQ_FROM=""
if [ -n "$P" ] && [ -e "$P/source/cadquery/v1/.pixi" ]; then
  cqprev=$(python3 -c 'import json,sys; r=json.load(open(sys.argv[1])); print(r.get("key", "") if r.get("status") == "ok" else "")' "$P/cadquery-env.json" 2> /dev/null)
  cqdir=$(readlink -f "$P/source/cadquery/v1/.pixi")
  if [ "$cqprev" = "$CQ_KEY" ] && [ -x "$cqdir/envs/$CQ_ENV/bin/python" ]; then CQ_FROM=$cqdir; fi
fi
if [ -n "$CQ_FROM" ]; then
  if [ -e "$CQ/.pixi" ] && [ ! -L "$CQ/.pixi" ]; then rm -rf -- "$CQ/.pixi" || fail "cannot remove the unfinished $CQ/.pixi"; fi
  ln -sfn "$CQ_FROM" "$CQ/.pixi" || fail "cadquery env link"
  CQ_HOW="reused $CQ_FROM"
  step "build cadquery env $CQ_ENV-$CQ_KEY reused from $CQ_FROM (pixi.toml and pixi.lock unchanged since $(basename "$P"))"
else
  if [ -L "$CQ/.pixi" ]; then rm -f -- "$CQ/.pixi" || fail "cannot remove the link $CQ/.pixi"; fi
  free=$(df -Pm "$D" | awk 'NR == 2 {print $4}')
  [ "${free:-0}" -ge @MIN_FREE@ ] || fail "only ${free:-?} MiB free under $D; the CadQuery env needs @MIN_FREE@"
  pixi_pinned
  cqwhy="no previous tree"; [ -z "$P" ] || cqwhy="$(basename "$P") holds no passing env with this key"
  step "build cadquery env $CQ_ENV-$CQ_KEY: installing with $("$PIXI" --version) into $CQ/.pixi (${free} MiB free; $cqwhy)"
  cd "$CQ" || fail "no cadquery/v1"
  run cadquery-env "$PIXI" install --manifest-path "$CQ/pixi.toml" -e "$CQ_ENV" --locked
  CQ_HOW="installed by pixi $CQ_PIXI"
fi
CQPY="$CQ/.pixi/envs/$CQ_ENV/bin/python"
[ -x "$CQPY" ] || fail "cadquery env $CQ_ENV has no python ($CQPY)"
cq_ms(){ local n; n=$(date +%s%N); echo $(( n / 1000000 )); }
t=$(cq_ms)
cqver=$(cd "$CQ" && prio "$CQPY" -B -c 'import cadquery, OCP; print(cadquery.__version__)' 2> "$S/cadquery-import.err") \
  || fail "cadquery does not import on $CQPY ($(tail -n 3 "$S/cadquery-import.err" | tr '\n' ' '))"
cq_import=$(( $(cq_ms) - t )); t=$(cq_ms)
(cd "$CQ" && prio "$CQPY" -B python/extract_weld_edges.py --step step/dog_box.step --sample-count 8) > "$S/cadquery-dog-box.json" 2> "$S/cadquery-dog-box.err" \
  || fail "the cadquery env cannot extract the topology of cadquery/v1/step/dog_box.step ($(tail -n 3 "$S/cadquery-dog-box.err" | tr '\n' ' '))"
cq_topology=$(( $(cq_ms) - t ))
cq_args=("$S/cadquery-dog-box.json" "$D/cadquery-env.json" "$CQPY" "$(readlink -f "$CQ/.pixi")" "$CQ_ENV" "$CQ_KEY" "$CQ_PIXI" "$CQ_HOW" "$cqver" "$cq_import" "$cq_topology")
python3 - "${cq_args[@]}" <<'ROSIE_CADQUERY_PY' || fail "the cadquery env's dog_box.step topology is not usable (above; $S/cadquery-dog-box.json)"
import json, os, sys, time
topology, out, python, env_dir, env, key, pixi, how, version, import_ms, topology_ms = sys.argv[1:12]
t = json.load(open(topology))
edges, seams = len(t.get("edges") or []), t.get("recommended_seams")
seams = len(seams) if isinstance(seams, list) else seams
if not edges:
    raise SystemExit("dog_box.step gave no edges")
record = {"status": "ok", "python": python, "env_dir": env_dir, "env": env, "key": key, "pixi": pixi, "provisioned": how,
          "cadquery": version, "import_ms": int(import_ms), "dog_box_edges": edges, "dog_box_recommended_seams": seams,
          "dog_box_topology_ms": int(topology_ms), "checked_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
with open(out + ".tmp", "w") as f:
    f.write(json.dumps(record) + "\n")
os.replace(out + ".tmp", out)
print(json.dumps(record))
ROSIE_CADQUERY_PY
step "build cadquery $cqver imports in ${cq_import} ms and reads dog_box.step in ${cq_topology} ms on $CQPY ($CQ_HOW)"
step "phase end"
`, "@CQ_ENV@", shellQuote(cellDeployCadQueryPixiEnv), "@PIXI_VERSION@", shellQuote(cellDeployPixiVersion),
		"@MIN_FREE@", strconv.Itoa(cellDeployCadQueryEnvMinFreeMiB))
}

// cellDeployCadQueryInstallRefusalShell refuses, before anything is staged,
// a tree whose build did not record a passing CadQuery check for the python
// its unit will name ($CQPY): installed, its OLP would answer every Import a
// part with worker_failed. A tree built before the build provisioned the env
// has no record and is refused too; build a new one.
const cellDeployCadQueryInstallRefusalShell = `python3 -c 'import json, os, sys
r = json.load(open(sys.argv[1]))
sys.exit(0 if r.get("status") == "ok" and r.get("python") == sys.argv[2] and os.access(sys.argv[2], os.X_OK) and r.get("dog_box_edges") else 1)' "$D/cadquery-env.json" "$CQPY" 2> /dev/null \
  || refuse "$D/cadquery-env.json is missing or records no passing CadQuery check for $CQPY: Import a part would fail on this tree (realtime-core build provisions and checks the env)"
`

// cellDeployCadQueryCheckShell defines, for olp install after the restart,
// cadquery_check PYTHON: the running OLP's CADQUERY_TOPOLOGY_PYTHON is PYTHON
// and executable. Existence only: the build proved that python on the part.
const cellDeployCadQueryCheckShell = `cadquery_check(){ # $1 python the running OLP must import parts with; sets OLP_FAIL
  local pid py
  pid=$(systemctl show -p MainPID --value "$UNIT")
  py=$(sudo -n cat "/proc/$pid/environ" 2> /dev/null | tr '\0' '\n' | sed -n 's/^CADQUERY_TOPOLOGY_PYTHON=//p' | head -n 1)
  [ -n "$py" ] && [ "$py" = "$1" ] || { OLP_FAIL="running OLP (pid $pid) imports parts with ${py:-pixi run (no CADQUERY_TOPOLOGY_PYTHON)}, not $1"; return 1; }
  [ -x "$py" ] || { OLP_FAIL="the running OLP's CadQuery python $py is not executable"; return 1; }
  step "olp imports parts with $py"
}
`

// cellDeployCadQueryStatusShell is olp status's report (needs UNIT and d, the
// unit's tree): cadquery_env ok|MISSING and the build's recorded check.
const cellDeployCadQueryStatusShell = `cqpy=$(systemctl show "$UNIT" -p Environment --value 2> /dev/null | tr ' ' '\n' | sed -n 's/^CADQUERY_TOPOLOGY_PYTHON=//p' | head -n 1)
if [ -z "$cqpy" ]; then
  echo "cadquery_env MISSING: $UNIT names no CADQUERY_TOPOLOGY_PYTHON (a tree built before the build provisioned the env); Import a part runs pixi run in $d/source/cadquery/v1"
elif [ ! -x "$cqpy" ]; then
  echo "cadquery_env MISSING: $cqpy is not executable; Import a part fails"
else
  echo "cadquery_env ok $cqpy -> $(readlink -f "$cqpy")"
fi
echo "cadquery_check $(cat "$d/cadquery-env.json" 2> /dev/null || echo "none recorded in $d")"
`
