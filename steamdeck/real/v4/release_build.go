package main

import (
	"encoding/json"
	"path"
	"strconv"
	"strings"
)

// The build host's persistent tree. `realtime-core build` used to clone the
// commit into every deployment tree and the test gate into a second clone,
// so every build recompiled rt-core and the resolver and every gate
// recompiled every test (DEVLOG 2026-10-07 17:50: ~100 s, ~67 s and the
// gate's 686 s make test per tree). Now one tree per cell,
// <scratch_dir>/tree, is fetched from the bundle and checked out at exactly
// the commit, verified by rev-parse and an empty `git status --porcelain`
// before and after, so nothing but the commit can be built from it (a tree
// edited by hand is refused; --clean deletes it). make's own dependencies
// decide what to rebuild there; the deployment tree receives copies of the
// artifacts and its own clean clone, as before.

// TreeDir is the cell's persistent build tree on the relay.
func (ctx cellDeployContext) TreeDir() string { return path.Join(ctx.Deploy.ScratchDir, "tree") }

// CacheDir holds the cell's caches that outlive trees (ccache, the tree lock).
func (ctx cellDeployContext) CacheDir() string { return path.Join(ctx.Deploy.ScratchDir, "cache") }

// cellDeployCachesShell names the relay user's persistent caches for the
// build and the gate alike: Go's build cache (GOCACHE, the user's default
// unless set) and the Cartesian resolver's content-addressed object cache,
// the motion-server Makefile's own default ~/.rosie/cache/motion-server/v1
// (keyed by compiler, flags and preprocessed source, so every tree and the
// gate's motion_test share it safely). The build used a per-cell copy and the
// gate the default, so one never warmed the other.
const cellDeployCachesShell = `export GOCACHE="${GOCACHE:-$(go env GOCACHE 2> /dev/null || echo "$HOME/.cache/go-build")}"
export OBJECT_CACHE_DIR="${OBJECT_CACHE_DIR:-$HOME/.rosie/cache/motion-server/v1}"
CACHES="GOCACHE=$GOCACHE ($(du -sh "$GOCACHE" 2> /dev/null | cut -f1)) resolver objects=$OBJECT_CACHE_DIR ($(ls "$OBJECT_CACHE_DIR/objects" 2> /dev/null | wc -l | tr -d ' '))"
`

// cellDeployTreeSyncShell defines tree_sync: bring $T to exactly $SHA from
// $BUNDLE ($REF), cloning it first from the previous deployment's source (or
// the bundle) when it does not exist. Needs T, C, BUNDLE, REF, SHA, P, CLEAN
// and step/refuse/fail.
const cellDeployTreeSyncShell = `tree_sync(){
  local fresh=0 changes
  if [ "$CLEAN" = 1 ] && [ -e "$T" ]; then
    step "build --clean: deleting the persistent tree $T (content-addressed caches stay)"
    rm -rf -- "$T" || refuse "cannot delete $T"
  fi
  if [ ! -d "$T/.git" ]; then
    if [ -n "$P" ] && [ -d "$P/source/.git" ]; then
      git clone -q --no-checkout "$P/source" "$T" || fail "clone of $P/source into $T"
    else
      git clone -q --no-checkout "$BUNDLE" "$T" || fail "clone from the bundle into $T (a first build needs a full bundle)"
    fi
    fresh=1
  fi
  if [ "$fresh" = 0 ]; then
    changes=$(git -C "$T" status --porcelain 2>&1) || refuse "git status in $T failed: $changes"
    [ -z "$changes" ] || refuse "the persistent tree $T has changes no commit carries ($(printf '%s' "$changes" | head -n 5 | tr '\n' ' ')): someone worked in it by hand. Look at them, then pass --clean"
  fi
  git -C "$T" fetch -q "$BUNDLE" "$REF" || fail "fetch $REF from the bundle into $T"
  git -C "$T" -c advice.detachedHead=false checkout -q --force --detach "$SHA" || fail "checkout $SHA in $T"
  [ "$(git -C "$T" rev-parse HEAD)" = "$SHA" ] || refuse "$T is at $(git -C "$T" rev-parse HEAD), not $SHA"
  changes=$(git -C "$T" status --porcelain)
  [ -z "$changes" ] || refuse "$T has changes after checking out $SHA ($(printf '%s' "$changes" | head -n 5 | tr '\n' ' ')): an ignore rule is missing; nothing was built"
  step "build tree $T at $SHA, clean ($([ "$fresh" = 1 ] && echo new || echo reused))"
}
`

// cellDeployIncompleteOutputsShell removes what an earlier build of $D wrote
// when that build did not reach BUILD_COMPLETE (the caller has refused a
// complete $D). Without it a rebuild packages into a non-empty runtime-package
// ("destination must be empty") or copies the previous package inside the
// stale one: on 2026-10-11 a planner-env failure after packaging left
// runtime-package in rosieos-<robot>-7d6c0414 and the next release of that
// commit failed in 3 s, with no repo command to clear it. $D/source stays:
// it is checked clean at its commit, and pixi resumes a half-installed env.
const cellDeployIncompleteOutputsShell = `cleared=""
for o in runtime-package runtime-package.components compiled compiled.path bin ui-stub motion-bin artifact-sha256.txt build.json cadquery-env.json; do
  [ -e "$D/$o" ] || [ -L "$D/$o" ] || continue
  rm -rf -- "${D:?}/$o" || refuse "cannot remove $D/$o left by an earlier build that did not complete"
  cleared="$cleared $o"
done
[ -z "$cleared" ] || step "build removed outputs of an earlier build of $D that did not complete:$cleared"
`

// cellDeployCompilerShell puts C and C++ through ccache when the host has it
// (Ubuntu's /usr/lib/ccache masquerade directory, else CXX/CC wrappers). It
// caches separate compile steps only: a rule that compiles and links in one
// invocation (every rt-core binary and test in 2026-10) is not cacheable.
const cellDeployCompilerShell = `if [ -d /usr/lib/ccache ]; then
  export PATH="/usr/lib/ccache:$PATH" CCACHE_DIR="$C/ccache" CCACHE_BASEDIR="$T"; COMPILER="ccache ($(ccache --version 2> /dev/null | head -n 1))"
elif command -v ccache > /dev/null 2>&1; then
  export CXX="ccache g++" CC="ccache gcc" CCACHE_DIR="$C/ccache" CCACHE_BASEDIR="$T"; COMPILER="ccache wrapper"
else
  COMPILER="no ccache on this host"
fi
`

func flagShell(b bool) string {
	if b {
		return "1"
	}
	return "0"
}

// The resolver's FK request contains the Cartesian arm joints, not every
// configured motor (a six-joint arm may have external axes). Read the validated
// binding produced by this build's pinned compile. The current resolver accepts
// exactly six joints; a different binding must fail rather than get padded or
// truncated into a probe for a robot it does not describe. A binding with no
// Cartesian joint (a bench of loose motors, no arm) has nothing to resolve and
// the OLP never asks: exit 3 skips the smoke, said so in the build log.
const cellDeployResolverSmokeShell = `python3 - "$D/compiled/$PIN/robot.json" "$MODEL" > "$S/resolver-smoke-request.json" <<'ROSIE_RESOLVER_SMOKE_PY'
import json, sys
with open(sys.argv[1]) as f:
    robot = json.load(f)
if robot["model_id"] != sys.argv[2]:
    raise SystemExit("compiled robot model does not match resolver model")
joints = [axis for axis in robot["axes"] if axis["cartesian_participates"] is True]
if not joints:
    raise SystemExit(3)
if len(joints) != 6:
    raise SystemExit(f"resolver FK requires six Cartesian joints, compiled binding has {len(joints)}")
print(json.dumps({"model": sys.argv[2], "frame": "base", "twist": [0] * 6,
    "lower": [-9] * len(joints), "upper": [9] * len(joints), "fraction": 0.5,
    "input_age_ns": 0, "operation": "fk", "pose": [0.1] * len(joints)}))
ROSIE_RESOLVER_SMOKE_PY
rc=$?
if [ "$rc" = 3 ]; then
  step "build resolver fk skipped: $MODEL has no Cartesian joints"
else
[ "$rc" = 0 ] || fail "resolver fk smoke request for $MODEL"
fk=$("$D/motion-bin/robot-v4-cartesiand" --resolve-only "$D/source" "$MODEL" < "$S/resolver-smoke-request.json" 2> "$S/resolver-smoke.err")
printf '%s' "$fk" | python3 -c 'import json,sys; r=json.load(sys.stdin); assert r["accepted"] and len(r["transform"]) > 0, r' 2> /dev/null || fail "resolver fk smoke for $MODEL: ${fk:0:300} $(tail -n 3 "$S/resolver-smoke.err")"
step "build resolver answers fk for $MODEL"
fi
`

// cellDeployBuildScript builds one deployment tree from the persistent
// tree at exactly the commit: rt-core live/control/sim, the cell compile (must
// equal the cell pin), package-runtime and verify-package, the OLP server,
// the UI stub, the Cartesian resolver (with an fk smoke), the planner
// environment and the CadQuery environment (with a part topology check,
// cell_deploy_cadquery.go). The rt-core package and compiled outputs are copied from the
// previous deployment when nothing under rt-core/ or robot_description/
// changed since its commit (never with --fetch-package: the package that
// would be installed is always built from the commit); the resolver
// likewise when motion-server/v1 is unchanged. It installs nothing and
// contacts no controller; a dropped connection leaves it running, logged in
// scratch_dir/build-<short>/build.log. A deployment tree without
// BUILD_COMPLETE is never installed, so a rebuild first removes the outputs
// an earlier failed build left in it (cellDeployIncompleteOutputsShell).
func cellDeployBuildScript(ctx cellDeployContext, plan cellDeployBuildPlan) string {
	meta, _ := json.Marshal(map[string]any{"commit": plan.Commit, "cell": ctx.CellID, "manifest": ctx.Manifest, "configuration_sha256": ctx.RTCore.ConfigurationSHA256, "machine_config": ctx.RTCore.MachineConfig})
	prio, check := cellLockWorkShell(ctx.CellID, plan.Locked)
	jobs := strconv.Itoa(ctx.Deploy.BuildJobs)
	if plan.Locked {
		jobs = "$(nproc)"
	}
	return "set -uo pipefail\nexport LC_ALL=C NO_REAL_MOVEMENT=1\n" + rtCoreConfigStepFunc + "\n" + cellDeployPreviousShell(ctx, plan.Previous) + rtCoreConfigReplace(`refuse(){ printf '@@refused %s\n' "$*"; exit 2; }
fail(){ step "build FAILED: $*"; printf '@@result {"status":"failed","deployment":"%s","reason":"%s"}\n' "$D" "$(printf '%s' "$*" | tr '"\\' "''")"; exit 1; }
trap '' HUP PIPE
SHA=@SHA@; REF=@REF@; PIN=@PIN@; D=@D@; S=@S@; T=@T@; C=@C@; BUNDLE=@BUNDLE@; BUNDLE_SHA=@BUNDLE_SHA@
MACHINE=@MACHINE@; MODEL=@MODEL@; IGH=@IGH@; ENV=@ENV@; OWNER=@OWNER@
INSTALL_ENV=@INSTALL_ENV@; ALLOW_DRIFT=@ALLOW_DRIFT@; CLEAN=@CLEAN@; FETCH=@FETCH@
JOBS=@JOBS@
export PATH=@PATH@:$PATH
export TMPDIR="$S/tmp"
@PRIO@
install -d -m 0700 "$S" "$S/tmp" "$C" || refuse "cannot create $S and $C"
exec 9> "$C/tree.lock"
flock -n 9 || refuse "another build or test gate of this cell uses $T ($C/tree.lock)"
@CHECK@exec > >(tee -p -a "$S/build.log") 2>&1
`+cellDeployCompilerShell+cellDeployCachesShell+cellDeployTreeSyncShell+`step "build begin commit=$SHA deployment=$D previous=${P:-none} relay=$(hostname) jobs=$JOBS compiler: $COMPILER; caches: $CACHES"
test "$(sha256sum < "$BUNDLE" | cut -d' ' -f1)" = "$BUNDLE_SHA" || refuse "bundle $BUNDLE does not have sha256 $BUNDLE_SHA"
[ ! -e "$D/BUILD_COMPLETE" ] || refuse "$D is already built (BUILD_COMPLETE)"
if [ ! -d "$D" ]; then sudo -n install -d -m 0755 -o "$OWNER" -g "$OWNER" "$D" || refuse "cannot create $D"; fi
[ -O "$D" ] || refuse "$D is not owned by $OWNER"
`+cellDeployIncompleteOutputsShell+`PH=""; [ -n "$P" ] && PH=$(git -C "$P/source" rev-parse HEAD 2> /dev/null)
unchanged(){ [ -n "$PH" ] && git -C "$T" cat-file -e "$PH^{commit}" 2> /dev/null && [ -z "$(git -C "$T" diff --name-only "$PH" "$SHA" -- "$@")" ]; }

step "phase source"
tree_sync
if [ ! -d "$D/source/.git" ]; then
  git clone -q --no-checkout "$T" "$D/source" && git -C "$D/source" -c advice.detachedHead=false checkout -q --detach "$SHA" || fail "clone of $T into $D/source"
fi
test "$(git -C "$D/source" rev-parse HEAD)" = "$SHA" || refuse "$D/source is at $(git -C "$D/source" rev-parse HEAD), not $SHA"
test -z "$(git -C "$D/source" status --porcelain)" || refuse "$D/source has local changes"
step "build source $D/source at $SHA, clean"

@PLANNER_EARLY@
export PKG_CONFIG_PATH="$IGH/igh-prefix/lib/pkgconfig"
igh_version=$(pkg-config --modversion libethercat 2> /dev/null) || refuse "no libethercat.pc under $IGH/igh-prefix/lib/pkgconfig"
# rt-core's Makefile pins 1.6.9 unless told otherwise; the cell's deps decide
# (a Pi 5 servo host runs 1.6.12, the RevPis 1.6.9).
export ROSIE_RT_IGH_VERSION="$igh_version"
export ECRT_LICENSE_DIR="$IGH/ethercat-$igh_version"
[ -f "$ECRT_LICENSE_DIR/COPYING" ] && [ -f "$ECRT_LICENSE_DIR/COPYING.LESSER" ] || refuse "IgH notices missing in $ECRT_LICENSE_DIR"
step "build IgH libethercat $igh_version from $IGH"

run(){ # name, command...: log per step, elapsed time
  local name=$1 t=$SECONDS rc; shift
  step "build $name: $*"
  prio "$@" > "$S/$name.log" 2>&1; rc=$?
  if [ "$rc" != 0 ]; then tail -n 25 "$S/$name.log" | sed 's/^/  | /'; fail "$name exit $rc (log $S/$name.log)"; fi
  step "build $name ok in $((SECONDS - t))s"
}
`+cellDeployPinnedPixiShell()+`
step "phase artifacts"
# ---- rt-core ----
RTCTL="$T/rt-core/build/rtctl"
if [ "$FETCH" != 1 ] && [ -f "$P/BUILD_COMPLETE" ] && [ -x "$P/runtime-package/bin/rtctl" ] && unchanged rt-core robot_description; then
  step "build rt-core reused: rt-core and robot_description unchanged since ${PH:0:12} ($P)"
  cp -a "$P/runtime-package" "$D/runtime-package" && cp -a "$P/runtime-package.components" "$D/runtime-package.components" || fail "copy of $P's rt-core package"
  RTCTL="$D/runtime-package/bin/rtctl"
else
  cd "$T/rt-core" || fail "no rt-core"
  run rt-core make -j"$JOBS" live control sim
  run package bash tools/package-runtime.sh "$D/runtime-package"
  run verify-package bash tools/verify-package.sh "$D/runtime-package"
fi
cd "$T/rt-core" || fail "no rt-core"
step "build compile $MACHINE with $RTCTL"
prio "$RTCTL" compile --config "$MACHINE" --out "$D/compiled" > "$D/compiled.path" 2> "$S/compile.log" || { tail -n 20 "$S/compile.log"; fail "rtctl compile"; }
got=$(basename "$(tail -n 1 "$D/compiled.path")")
[ "$got" = "$PIN" ] || fail "compiled configuration $got is not the cell pin $PIN"
step "build compiled configuration $got = cell pin"
if [ -n "$P" ] && [ -d "$P/compiled/$PIN" ]; then
  if diff -r -x argv.json "$D/compiled/$PIN" "$P/compiled/$PIN" > "$S/compiled.diff" 2>&1; then
    step "build compiled outputs byte-identical to $P (argv.json excluded)"
  elif [ "$ALLOW_DRIFT" = 1 ]; then
    step "build WARNING compiled outputs for $PIN differ from $P (allowed): $(head -c 300 "$S/compiled.diff" | tr '\n' ' ')"
  else
    fail "compiled outputs for the unchanged pin $PIN differ from $P ($S/compiled.diff): the compiler's semantics changed under the same digest; review, then pass --allow-compiled-drift"
  fi
fi
set -- "$D"/runtime-package.components/rosie-rt-core-*.tar.gz
[ "$#" = 1 ] && [ -f "$1" ] || fail "expected one rosie-rt-core-*.tar.gz in $D/runtime-package.components"
PKG=$1; PKG_SHA=$(sha256sum < "$PKG" | cut -d' ' -f1)
step "build package $PKG sha256=$PKG_SHA"

# ---- OLP server, UI stub, resolver ----
cd "$T/offline-programming/v1" || fail "no offline-programming/v1"
run olp go build -trimpath -o "$D/bin/offline-programming" .
# The camera's WebRTC sender, installed by camera install when the cell's
# camera has video (cell-camera/v1/video).
cd "$T/cell-camera/v1/video" || fail "no cell-camera/v1/video"
run video go build -trimpath -o "$D/bin/cell-video" .
install -d -m 0755 "$D/ui-stub" && printf '%s' @UI_STUB@ > "$D/ui-stub/index.html" || fail "ui stub"
if [ -x "$P/motion-bin/robot-v4-cartesiand" ] && unchanged motion-server/v1; then
  install -d -m 0755 "$D/motion-bin" && cp -p "$P/motion-bin/robot-v4-cartesiand" "$D/motion-bin/" || fail "copy of $P's resolver"
  step "build resolver reused: motion-server/v1 unchanged since ${PH:0:12}"
else
  cd "$T/motion-server/v1" || fail "no motion-server/v1"
  run resolver make -j"$JOBS" all BIN_DIR="$D/motion-bin" OBJECT_CACHE_DIR="$OBJECT_CACHE_DIR"
fi
`+cellDeployResolverSmokeShell+`@PLANNER_ENV@`+cellDeployCadQueryEnvShell()+`changes=$(git -C "$T" status --porcelain)
[ -z "$changes" ] || fail "the build left changes in $T ($(printf '%s' "$changes" | head -n 5 | tr '\n' ' ')): an ignore rule is missing"
sha256sum "$D/runtime-package/bin/"* "$D/bin/offline-programming" "$D/motion-bin/robot-v4-cartesiand" > "$D/artifact-sha256.txt"
sha256sum "$D/bin/cell-video" >> "$D/artifact-sha256.txt"
sed 's/^/  /' "$D/artifact-sha256.txt"
printf '%s\n' @META@ > "$D/build.json"
touch "$D/BUILD_COMPLETE"
step "build complete $D"
printf '@@result {"status":"ok","deployment":"%s","commit":"%s","configuration":"%s","igh":"%s","package":"%s","package_sha256":"%s"}\n' "$D" "$SHA" "$PIN" "$igh_version" "$PKG" "$PKG_SHA"
`, "@SHA@", shellQuote(plan.Commit), "@REF@", shellQuote(plan.Ref), "@PIN@", shellQuote(ctx.RTCore.ConfigurationSHA256),
		"@D@", shellQuote(ctx.DeploymentDir(plan.Commit)), "@S@", shellQuote(path.Join(ctx.Deploy.ScratchDir, "build-"+cellDeployShort(plan.Commit))),
		"@T@", shellQuote(ctx.TreeDir()), "@C@", shellQuote(ctx.CacheDir()),
		"@BUNDLE@", shellQuote(plan.BundlePath), "@BUNDLE_SHA@", shellQuote(plan.BundleSHA256), "@MACHINE@", shellQuote(ctx.MachineInRTCore()),
		"@MODEL@", shellQuote(ctx.RobotModel), "@IGH@", shellQuote(ctx.Deploy.IgHDepsDir), "@JOBS@", jobs,
		"@ENV@", shellQuote(ctx.Deploy.OLP.PlannerPixiEnv), "@OWNER@", shellQuote(ctx.RelayUser), "@INSTALL_ENV@", flagShell(plan.InstallPlannerEnv),
		"@ALLOW_DRIFT@", flagShell(plan.AllowDrift), "@CLEAN@", flagShell(plan.Clean), "@FETCH@", flagShell(plan.FetchPackage),
		"@PATH@", shellQuote(strings.Join(ctx.Deploy.Path, ":")), "@UI_STUB@", shellQuote(cellDeployUIStub),
		"@PRIO@", prio, "@CHECK@", check, "@META@", shellQuote(string(meta)),
		"@PLANNER_EARLY@", cellDeployPlannerEnvEarlyShell(ctx), "@PLANNER_ENV@", cellDeployPlannerEnvShell(ctx))
}

// cellDeployGateScript runs the selected suites in the persistent tree,
// brought back to the built commit from the deployment's own clean clone
// when a later build moved it (and refused if anyone edited it), on lanes,
// logging each
// suite and printing one @@result with every suite's exit, duration, failing
// test names and slowest parts for the Mac to classify. A suite that leaves
// files behind is reported and the tree reset (it was verified clean before).
func cellDeployGateScript(ctx cellDeployContext, commit string, sel gateSelection, locked bool) string {
	prio, check := cellLockWorkShell(ctx.CellID, locked)
	jobs := strconv.Itoa(ctx.Deploy.BuildJobs)
	lanes := "1"
	if locked {
		// The lanes run at once, so they share the cores: each lane's make
		// gets nproc/lanes jobs. Every lane at -j$(nproc) ran 3 x 6 compilers
		// on the 6-core, 7.6 GB Jetsons beside the live OLP, planner and
		// camera: on 2026-10-20 cc1plus was OOM-killed in one relay's gate and
		// another relay stopped answering ssh for ~6 min (load 27, 6.1 GB used),
		// which failed both releases.
		lanes = "$(( ($(nproc) + 1) / 2 ))"
		jobs = "$(( $(nproc) / (($(nproc) + 1) / 2) ))"
	}
	return "set -uo pipefail\nexport LC_ALL=C NO_REAL_MOVEMENT=1\n" + rtCoreConfigStepFunc + "\n" + rtCoreConfigReplace(`refuse(){ printf '@@refused %s\n' "$*"; exit 2; }
fail(){ printf '@@refused %s\n' "$*"; exit 1; }
trap '' HUP PIPE
SHA=@SHA@; D=@D@; S=@S@; T=@T@; C=@C@; IGH=@IGH@; LEVEL=@LEVEL@
JOBS=@JOBS@; LANES=@LANES@; TREE=$T
export JOBS S D TREE
L="$S/gate-logs"
export PATH=@PATH@:$PATH
export TMPDIR="$S/tmp" GOFLAGS=-mod=mod
`+cellDeployCachesShell+`@PRIO@
[ -e "$D/BUILD_COMPLETE" ] || refuse "$D is not built; run realtime-core build first"
install -d -m 0700 "$S" "$S/tmp" "$C" || refuse "cannot create $S"
exec 9> "$C/tree.lock"
flock -n 9 || refuse "another build or test gate of this cell uses $T ($C/tree.lock)"
@CHECK@rm -rf -- "$L" && install -d -m 0700 "$L" || refuse "cannot create $L"
`+cellDeployTreeSyncShell+`BUNDLE="$D/source"; REF=HEAD; P=""; CLEAN=0
tree_sync
if [ ! -d "$T/rt-core/build/deps" ]; then mkdir -p "$T/rt-core/build" && cp -a "$IGH" "$T/rt-core/build/deps" || refuse "copy IgH deps"; fi
# The build's IgH version, not the Makefile's 1.6.9 default (release build above).
ROSIE_RT_IGH_VERSION=$(PKG_CONFIG_PATH="$IGH/igh-prefix/lib/pkgconfig" pkg-config --modversion libethercat 2> /dev/null) || refuse "no libethercat.pc under $IGH/igh-prefix/lib/pkgconfig"
export ROSIE_RT_IGH_VERSION
cat > "$S/tsh" <<'ROSIE_GATE_TSH' && chmod 0755 "$S/tsh" || refuse "cannot write $S/tsh"
@TSH@
ROSIE_GATE_TSH
step "gate level $LEVEL in $T at $SHA: $(printf '%s' @NAMES@) on $LANES lane(s), make -j$JOBS; caches: $CACHES"
gate(){ # name, directory, command
  local name=$1 dir=$2 cmd=$3 t=$SECONDS rc
  step "gate $name ($dir): $cmd"
  : > "$L/$name.recipes"
  ( cd "$T/$dir" && export ROSIE_TSH_LOG="$L/$name.recipes" && prio bash -c "$cmd" ) > "$L/$name.log" 2>&1; rc=$?
  python3 - "$L/$name.log" "$name" "$rc" "$((SECONDS - t))" "$L/$name.recipes" > "$L/$name.result.json" <<'ROSIE_GATE_PY'
@GATE_PY@
ROSIE_GATE_PY
  step "gate $name exit $rc in $((SECONDS - t))s"
}
`+gateSchedulerShell+`gate_run_all <<'ROSIE_SUITES'
@TABLE@ROSIE_SUITES
changes=$(git -C "$T" status --porcelain)
if [ -n "$changes" ]; then
  step "gate WARNING the suites left changes in $T ($(printf '%s' "$changes" | head -n 5 | tr '\n' ' ')); resetting it to $SHA"
  git -C "$T" checkout -q --force -- . && git -C "$T" clean -fdq
fi
printf '@@result {"status":"done","level":"%s","steps":[%s]}\n' "$LEVEL" "$(cat "$L"/*.result.json 2> /dev/null | paste -sd, -)"
`, "@SHA@", shellQuote(commit), "@D@", shellQuote(ctx.DeploymentDir(commit)), "@S@", shellQuote(path.Join(ctx.Deploy.ScratchDir, "build-"+cellDeployShort(commit))),
		"@T@", shellQuote(ctx.TreeDir()), "@C@", shellQuote(ctx.CacheDir()), "@IGH@", shellQuote(ctx.Deploy.IgHDepsDir), "@LEVEL@", shellQuote(sel.Level),
		"@JOBS@", jobs, "@LANES@", lanes, "@PATH@", shellQuote(strings.Join(ctx.Deploy.Path, ":")), "@PRIO@", prio, "@CHECK@", check,
		"@TSH@", cellDeployGateTSH, "@GATE_PY@", cellDeployGatePy, "@NAMES@", shellQuote(strings.Join(gateNamesOf(sel), " ")), "@TABLE@", gateSuiteTable(sel))
}

func gateNamesOf(sel gateSelection) []string {
	var out []string
	for _, s := range sel.Suites {
		out = append(out, s.Name)
	}
	if len(out) == 0 {
		return []string{"(none)"}
	}
	return out
}

// cellDeployGatePy reads one suite's log: failing test names (Go "--- FAIL:"
// and TAP "not ok" lines) and its slowest parts (make recipe lines through the
// SHELL wrapper, Go packages, node tests) for the Mac to classify and time.
const cellDeployGatePy = `import json, re, sys
log, name, rc, secs, recipes = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), sys.argv[5]
fails, suites, tap = [], [], None
for line in open(log, errors="replace"):
    m = re.match(r"^\s*--- FAIL: (\S+)", line) or re.match(r"^not ok \d+ - (.+?)\s*$", line)
    if m and m.group(1) not in fails:
        fails.append(m.group(1))
    m = re.match(r"^(?:ok|FAIL)\s+(\S+)\s+([0-9.]+)s\s*$", line)
    if m:
        suites.append({"name": "go " + m.group(1), "seconds": float(m.group(2))})
    m = re.match(r"^(?:not )?ok \d+ - (.+?)\s*$", line)
    if m:
        tap = m.group(1)
    m = re.match(r"^  duration_ms: ([0-9.]+)", line)
    if m and tap:
        suites.append({"name": "node " + tap[:100], "seconds": float(m.group(1)) / 1000})
        tap = None
try:
    for line in open(recipes, errors="replace"):
        p = line.rstrip("\n").split(" ", 3)
        if len(p) == 4 and p[0].isdigit() and p[1].isdigit():
            suites.append({"name": " ".join(p[3].split())[:120], "seconds": (int(p[1]) - int(p[0])) / 1e9})
except OSError:
    pass
suites = [dict(s, seconds=round(s["seconds"], 1)) for s in sorted(suites, key=lambda s: -s["seconds"]) if s["seconds"] >= 1][:15]
print(json.dumps({"step": name, "rc": rc, "seconds": secs, "failures": fails[:50], "log": log, "suites": suites}))
`
