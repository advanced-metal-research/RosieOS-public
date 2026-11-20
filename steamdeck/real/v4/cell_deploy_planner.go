package main

import (
	"encoding/base64"
	"fmt"
	"net/url"
	"path"
	"strconv"
	"strings"
)

// The weld planner on the relay. Until 2026-10-11 rosie-<robot>-planner.service
// was a unit written by hand at the 2026-10-06 cutover: CPU-only (motion-cpu)
// on the relay, running the 8e3c9557 tree and its env, with its dense store
// inside that deployment. No command updated it, so every OLP release after
// 8e3c9557 ran behind a planner five days old: the HOME-from-measured-start
// planning of 5a8f5bd6 reached the cell only in the OLP, and the old planner
// refused it. A cell_deploy.olp.planner block hands the unit to the deploy
// commands: the build installs the pixi env from the lock with a pinned pixi
// into a content-addressed directory outside every deployment (reused by
// every later tree with the same pixi.toml, pixi.lock and env), proves the
// env loads the planner server and, for a CUDA cell, that torch runs a kernel
// on the GPU; olp install renders the unit for the tree it installs, keeps
// the replaced unit for olp rollback (and a hand-made one in the state dir
// for good), restarts the planner and refuses, restoring the previous unit,
// unless /api/motion/health answers from that tree's directory with that
// tree's commit and the cuda value the manifest names. Without the block the
// planner unit stays hand-provisioned and only has to be active.

type cellDeployOLPPlanner struct {
	// Description is the unit's, holding {release} once.
	Description string `json:"description"`
	// StateDir holds what outlives deployments: envs/<env>-<key> (the pixi
	// envs), dense/ (the planner's dense store, which programs Load from by
	// digest), cache/ (XDG and Warp kernel caches) and hand-made/ (the
	// hand-provisioned unit a first managed install replaced).
	StateDir string `json:"state_dir"`
	// CUDA is what /api/motion/health must report: true for a GPU cell.
	CUDA *bool `json:"cuda"`
	// Threads bounds OpenMP and Numba (OMP_NUM_THREADS, NUMBA_NUM_THREADS);
	// OpenBLAS stays single-threaded. The relay also runs the OLP and the
	// control tunnel: 2 of the relay's 6 cores since the 2026-10-06 cutover
	// (unmeasured).
	Threads int `json:"threads"`
}

// cellDeployPixiVersion is the pixi the build installs planner envs with. It
// is pinned because the lock's format is: weld_planner/v1/pixi.lock is
// format v7, which pixi 0.67.2 (the relay's ~/.rosie/bin/pixi in 2026-10)
// rewrites as v6 while re-solving every environment, and 0.81.0 reads as is
// (adding motion-jetson left the other six environments byte-identical).
const cellDeployPixiVersion = "0.81.0"

// cellDeployPixiPins are the static musl builds of cellDeployPixiVersion by
// `uname -m`, with the digests GitHub publishes for the release assets.
var cellDeployPixiPins = map[string][2]string{
	"aarch64": {"https://github.com/prefix-dev/pixi/releases/download/v0.81.0/pixi-aarch64-unknown-linux-musl", "7fcd20f7c18339f13552a6767ab146ce6b439ce29db46cc03ece005ca9696bb1"},
	"x86_64":  {"https://github.com/prefix-dev/pixi/releases/download/v0.81.0/pixi-x86_64-unknown-linux-musl", "248510f5f75460a2ae4ec62d761e15c0191dc35db3b1da6f82d9f17c6017051d"},
}

// cellDeployPlannerEnvMinFreeMiB refuses an env build that would fill the
// relay: motion-jetson installed is 4.2 GiB (linux/arm64 container,
// 2026-10-11), its wheels 0.4 GiB, and the conda package cache grows by
// about as much again on a first install (unmeasured on the relay).
const cellDeployPlannerEnvMinFreeMiB = 12288

// cellDeployPlannerManagedMarker is the rendered unit's first line. olp
// install keeps a unit without it in <state_dir>/hand-made/ before replacing
// it: a hand-made unit is the only record of how the cell ran before.
const cellDeployPlannerManagedMarker = "# Rendered by realtime-core olp install from cell_deploy.olp.planner; change the cell manifest, not this file."

// cellDeployPlannerRevisionToken is replaced on the relay by the deployment
// source's full commit (the planner's AMR_WELD_PLANNER_SOURCE_REVISION).
const cellDeployPlannerRevisionToken = "@@SOURCE_REVISION@@"

func (p cellDeployOLPPlanner) validate(c cellDeployConfig) error {
	if strings.Count(p.Description, cellDeployReleaseToken) != 1 || strings.ContainsAny(p.Description, "\r\n%\\") {
		return fmt.Errorf("cell_deploy olp.planner.description must be one line holding %s exactly once, without %% or backslashes", cellDeployReleaseToken)
	}
	if err := cellDeployCleanAbs("olp.planner.state_dir", p.StateDir); err != nil {
		return err
	}
	if p.StateDir == c.DeploymentsDir || strings.HasPrefix(p.StateDir+"/", c.DeploymentsDir+"/") || strings.HasPrefix(c.DeploymentsDir+"/", p.StateDir+"/") {
		return fmt.Errorf("cell_deploy olp.planner.state_dir must be outside deployments_dir: the dense store and envs outlive every deployment")
	}
	if p.CUDA == nil {
		return fmt.Errorf("cell_deploy olp.planner.cuda is required: true when the planner must run on the relay's GPU, false for CPU")
	}
	if p.Threads < 1 || p.Threads > 64 {
		return fmt.Errorf("cell_deploy olp.planner.threads must be 1..64, got %d", p.Threads)
	}
	if _, err := cellDeployPlannerPort(c.OLP.PlannerOrigin); err != nil {
		return err
	}
	return nil
}

// cellDeployPlannerPort is the port of a planner origin the unit can bind:
// http on 127.0.0.1 (the server binds loopback only).
func cellDeployPlannerPort(origin string) (int, error) {
	u, err := url.Parse(origin)
	if err == nil && u.Scheme == "http" && u.Hostname() == "127.0.0.1" {
		if n, perr := strconv.Atoi(u.Port()); perr == nil && n > 0 && n < 65536 {
			return n, nil
		}
	}
	return 0, fmt.Errorf("cell_deploy olp.planner_origin must be http://127.0.0.1:<port> when olp.planner is managed (the unit binds loopback), got %q", origin)
}

func (p cellDeployOLPPlanner) DenseDir() string { return path.Join(p.StateDir, "dense") }

// cellDeployPlannerUnit renders the planner unit for one deployment tree.
// The python is the tree's own env link, so the unit names the tree it runs;
// the source revision is filled in on the relay from the tree's checkout.
func cellDeployPlannerUnit(ctx cellDeployContext, deployment, release string) string {
	o := ctx.Deploy.OLP
	p := *o.Planner
	port, _ := cellDeployPlannerPort(o.PlannerOrigin)
	w := deployment + "/source/weld_planner/v1"
	threads := strconv.Itoa(p.Threads)
	return strings.Join([]string{
		cellDeployPlannerManagedMarker,
		"[Unit]",
		"Description=" + strings.Replace(p.Description, cellDeployReleaseToken, release, 1),
		"After=network.target",
		"[Service]",
		"User=" + ctx.RelayUser,
		"WorkingDirectory=" + w,
		"Environment=HOME=" + ctx.Deploy.Home,
		"Environment=PYTHONDONTWRITEBYTECODE=1",
		"Environment=PYTHONPATH=" + w + "/python",
		"Environment=AMR_WELD_PLANNER_SOURCE_REVISION=" + cellDeployPlannerRevisionToken,
		"Environment=OMP_NUM_THREADS=" + threads,
		"Environment=OPENBLAS_NUM_THREADS=1",
		"Environment=NUMBA_NUM_THREADS=" + threads,
		// pixi's activation sets this for `pixi run`; the unit runs the
		// env's python directly (weld_planner/v1/pixi.toml, TF32 policy).
		"Environment=TORCH_ALLOW_TF32_CUBLAS_OVERRIDE=0",
		"Environment=XDG_CACHE_HOME=" + p.StateDir + "/cache/xdg",
		"Environment=WARP_CACHE_PATH=" + p.StateDir + "/cache/warp",
		"ExecStart=" + w + "/.pixi/envs/" + o.PlannerPixiEnv + "/bin/python -B -m weld_motion_planner.server.motion_planner_server --host 127.0.0.1 --port " + strconv.Itoa(port) + " --dense-store-dir " + p.DenseDir(),
		"Restart=on-failure",
		"RestartSec=3",
		"UMask=0077",
		"[Install]",
		"WantedBy=multi-user.target",
	}, "\n") + "\n"
}

// cellDeployPlannerEnvEarlyShell runs in the build before anything is
// built: the legacy reuse decision (planner unit by hand), or for a managed
// planner nothing but the announcement; its env is built after the OLP.
func cellDeployPlannerEnvEarlyShell(ctx cellDeployContext) string {
	if ctx.Deploy.OLP.Planner == nil {
		return `# ---- planner environment: reuse the previous tree's when weld_planner is unchanged ----
ENV_LINK=""; envsha=""
if [ -n "$P" ] && [ -e "$P/source/weld_planner/v1/.pixi" ]; then
  envdir=$(readlink -f "$P/source/weld_planner/v1/.pixi")
  envsha=$(git -C "${envdir%/weld_planner/v1/.pixi}" rev-parse HEAD 2> /dev/null)
  if [ -n "$envsha" ] && [ -x "$envdir/envs/$ENV/bin/python" ] && git -C "$D/source" cat-file -e "$envsha^{commit}" 2> /dev/null \
     && [ -z "$(git -C "$D/source" diff --name-only "$envsha" "$SHA" -- weld_planner)" ]; then
    ENV_LINK=$envdir
  fi
fi
if [ -z "$ENV_LINK" ] && [ "$INSTALL_ENV" != 1 ]; then
  refuse "no reusable planner env: weld_planner changed since ${envsha:-<none>} or the previous tree has no $ENV env; re-run with --install-planner-env (pixi install -e $ENV)"
fi
`
	}
	return rtCoreConfigReplace(`# ---- planner environment: managed (cell_deploy.olp.planner), built after the OLP ----
ENV_LINK=""; PSTATE=@PSTATE@; PCUDA=@PCUDA@; PIXI_VERSION=@PIXI_VERSION@
for tool in curl python3 sha256sum; do command -v "$tool" > /dev/null || refuse "$tool not found on the relay (the planner env build needs it)"; done
step "build planner env $ENV: managed under $PSTATE (pixi $PIXI_VERSION, cuda expected $PCUDA)"
`, "@PSTATE@", shellQuote(ctx.Deploy.OLP.Planner.StateDir), "@PCUDA@", flagShell(*ctx.Deploy.OLP.Planner.CUDA), "@PIXI_VERSION@", shellQuote(cellDeployPixiVersion))
}

// cellDeployPlannerEnvShell links the deployment's weld_planner/v1/.pixi to
// its env. Legacy: the previous tree's, or pixi install in the tree.
// Managed: <state_dir>/envs/<env>-<key>, key = sha256(pixi.toml, pixi.lock,
// env, pixi version), installed --locked by the pinned pixi when no finished
// one exists (a build that stopped half way leaves no READY and is redone).
// pixi checks every locked package's sha256; the lock holds no direct-URL
// wheel, which pixi would not check. Then, every build (a JetPack or driver
// update can break a reused env): the device probe
// (weld_motion_planner.utils.device, its report is $D/planner-env.json,
// which olp install requires) must find a GPU torch can run a kernel on when
// the manifest says cuda, and the tree's planner server must load on the env.
func cellDeployPlannerEnvShell(ctx cellDeployContext) string {
	if ctx.Deploy.OLP.Planner == nil {
		return `if [ -n "$ENV_LINK" ]; then
  ln -sfn "$ENV_LINK" "$D/source/weld_planner/v1/.pixi" || fail "planner env link"
  step "build planner env reused from $ENV_LINK (weld_planner unchanged since $envsha)"
else
  cd "$D/source/weld_planner/v1" || fail "no weld_planner/v1"
  run planner-env pixi install -e "$ENV"
fi
[ -x "$D/source/weld_planner/v1/.pixi/envs/$ENV/bin/python" ] || fail "planner env $ENV has no python"
`
	}
	return rtCoreConfigReplace(`WP="$T/weld_planner/v1"
KEY=$( { cat "$WP/pixi.toml"; printf '\0'; cat "$WP/pixi.lock"; printf '\0%s\0%s' "$ENV" "$PIXI_VERSION"; } | sha256sum | cut -c1-16)
ER="$PSTATE/envs/$ENV-$KEY"
step "phase planner-env"
install -d -m 0700 "$PSTATE" "$PSTATE/envs" "$PSTATE/dense" "$PSTATE/cache" || fail "cannot create $PSTATE"
if [ -e "$ER/READY" ]; then
  step "build planner env $ENV-$KEY reused ($ER, built $(cat "$ER/READY"))"
else
  free=$(df -Pm "$PSTATE" | awk 'NR == 2 {print $4}')
  [ "${free:-0}" -ge @MIN_FREE@ ] || fail "only ${free:-?} MiB free under $PSTATE; the planner env needs @MIN_FREE@"
  if [ -e "$ER" ]; then step "build planner env: removing the unfinished $ER"; rm -rf -- "$ER" || fail "cannot remove $ER"; fi
  pixi_pinned
  step "build planner env $ENV-$KEY: installing with $("$PIXI" --version) into $ER (${free} MiB free)"
  install -d -m 0700 "$ER" && cp -p "$WP/pixi.toml" "$WP/pixi.lock" "$ER/" || fail "cannot stage $ER"
  run planner-env "$PIXI" install --manifest-path "$ER/pixi.toml" -e "$ENV" --locked
fi
[ -x "$ER/.pixi/envs/$ENV/bin/python" ] || fail "planner env $ER has no $ENV python"
[ -e "$ER/READY" ] || date -u +%Y-%m-%dT%H:%M:%SZ > "$ER/READY" || fail "cannot mark $ER ready"
ENV_LINK="$ER/.pixi"
ln -sfn "$ENV_LINK" "$D/source/weld_planner/v1/.pixi" || fail "planner env link"
step "build planner env $ENV-$KEY linked into $D"
PY="$D/source/weld_planner/v1/.pixi/envs/$ENV/bin/python"
[ -x "$PY" ] || fail "planner env $ENV has no python"
pyenv(){ (cd "$D/source/weld_planner/v1" && PYTHONPATH="$D/source/weld_planner/v1/python" XDG_CACHE_HOME="$PSTATE/cache/xdg" WARP_CACHE_PATH="$PSTATE/cache/warp" \
  TORCH_ALLOW_TF32_CUBLAS_OVERRIDE=0 prio "$PY" -B "$@"); }
probe=$(pyenv -m weld_motion_planner.utils.device 2> "$S/planner-device.err"); rc=$?
printf '%s\n' "$probe" > "$D/planner-env.json"
step "build planner device probe exit $rc: $(printf '%s' "$probe" | tr -s ' \n' ' ' | cut -c1-400)"
[ "$rc" = 0 ] || fail "the planner env's python failed its device probe (exit $rc; $(tail -n 3 "$S/planner-device.err" | tr '\n' ' '))"
dev=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("device"))' "$D/planner-env.json" 2> /dev/null)
if [ "$PCUDA" = 1 ] && [ "$dev" != cuda ]; then
  fail "the manifest expects a CUDA planner and $ENV's torch cannot run a kernel on this relay's GPU (device $dev; $D/planner-env.json)"
fi
pyenv -c 'import sys, tempfile; from weld_motion_planner.server.motion_planner_server import make_app; make_app(tempfile.mkdtemp(dir=sys.argv[1]))' "$S/tmp" 2> "$S/planner-load.err" \
  || fail "the planner server does not load from $D on $ENV ($(tail -n 3 "$S/planner-load.err" | tr '\n' ' '))"
step "build planner server loads from $D on $ENV (device $dev)"
`, "@MIN_FREE@", strconv.Itoa(cellDeployPlannerEnvMinFreeMiB))
}

// cellDeployPlannerCheckShell defines, for olp install, rollback and status:
// planner_idle (no plan in flight: /api/motion/progress has no stage) and
// planner_check PYTHON WORKDIR CUDA REVISION: health within 180 s, from the
// unit's main process whose executable is PYTHON resolved and whose working
// directory is WORKDIR resolved (the env is shared by every tree with the
// same lock, so only the directory names the tree), health's cuda equal to
// CUDA (1, 0, or "" for either) and its source_revision equal to REVISION
// ("" for any: a planner from before 2026-10-11 does not report one). The
// first health request imports Torch and, on a GPU, initialises CUDA:
// unmeasured on the relay, hence the wide wait.
const cellDeployPlannerCheckShell = `planner_idle(){
  local s
  s=$(curl -fsS -m 5 "$PORIGIN/api/motion/progress" 2> /dev/null) || return 0
  printf '%s' "$s" | python3 -c 'import json,sys; sys.exit(0 if json.load(sys.stdin).get("stage") is None else 1)'
}
planner_check(){ # $1 python, $2 working directory, $3 expected cuda (1, 0 or ""), $4 expected commit or ""; sets PLANNER_FAIL
  local i h="" pid exe cwd want wantd
  for i in $(seq 1 180); do
    h=$(curl -fsS -m 30 "$PORIGIN/api/motion/health" 2> /dev/null) && break
    h=""
    [ $((i % 15)) = 0 ] && step "planner waiting for health ${i}s"
    sleep 1
  done
  [ -n "$h" ] || { PLANNER_FAIL="planner health not ok within 180s ($PORIGIN/api/motion/health)"; return 1; }
  step "planner health answered after ${i}s"
  pid=$(systemctl show -p MainPID --value "$PLANNER")
  exe=$(sudo -n readlink -f "/proc/$pid/exe" 2> /dev/null)
  cwd=$(sudo -n readlink -f "/proc/$pid/cwd" 2> /dev/null)
  want=$(readlink -f "$1"); wantd=$(readlink -f "$2")
  [ -n "$exe" ] && [ "$exe" = "$want" ] || { PLANNER_FAIL="running planner (pid $pid) is ${exe:-unreadable}, not $want"; return 1; }
  [ -n "$cwd" ] && [ "$cwd" = "$wantd" ] || { PLANNER_FAIL="running planner (pid $pid) runs in ${cwd:-unreadable}, not $wantd"; return 1; }
  PLANNER_HEALTH=$h
  printf '%s' "$h" | python3 -c '
import json, sys
h, want, rev = json.load(sys.stdin), sys.argv[1], sys.argv[2]
cuda = h.get("cuda") is True
print("planner health cuda=%s device=%s source_revision=%s (expected %s, %s)" % (h.get("cuda"), h.get("device"), h.get("source_revision"),
      {"1": "cuda=true", "0": "cuda=false"}.get(want, "either"), rev or "any revision"))
sys.exit(1 if (want in ("0", "1") and cuda != (want == "1")) or (cuda and not h.get("device")) or (rev and h.get("source_revision") != rev) else 0)' "$3" "$4" \
    || { PLANNER_FAIL="planner health $h does not match cuda=$([ "$3" = 1 ] && echo true || { [ "$3" = 0 ] && echo false || echo either; })${4:+ and source_revision=$4}"; return 1; }
  return 0
}
planner_unit_value(){ # $1 unit file, $2 key: ExecStart's program, WorkingDirectory, or the source revision
  case "$2" in
    exec) sed -n 's/^ExecStart=\([^ ]*\).*/\1/p' "$1" | head -n 1 ;;
    workdir) sed -n 's/^WorkingDirectory=//p' "$1" | head -n 1 ;;
    revision) sed -n 's/^Environment=AMR_WELD_PLANNER_SOURCE_REVISION=//p' "$1" | head -n 1 ;;
  esac
}
`

// cellDeployPlannerInstallShell swaps the planner unit for the tree being
// installed, before the OLP unit: back it up beside the OLP's backup, keep
// what the old dense store holds (programs Load by digest), install, restart
// and check; a failed check reinstates the backup and exits 5 before the OLP
// is touched. planner_reinstate is what the OLP's restore calls too.
func cellDeployPlannerInstallShell(ctx cellDeployContext, deployment, release string) string {
	if ctx.Deploy.OLP.Planner == nil {
		return "planner_reinstate(){ :; }\n"
	}
	p := ctx.Deploy.OLP.Planner
	unit := cellDeployPlannerUnit(ctx, deployment, release)
	return rtCoreConfigReplace(`step "phase planner"
PUNIT_PATH=/etc/systemd/system/@PLANNER_NAME@; PSTATE=@PSTATE@; PDENSE=@PDENSE@; PCUDA=@PCUDA@; PSWAPPED=0
PBACKUP="$D/@PLANNER_NAME@.bak-${pb#"$PREFIX"}"
planner_reinstate(){
  [ "$PSWAPPED" = 1 ] || return 0
  step "planner RESTORE: reinstalling $PBACKUP"
  sudo -n install -m 644 "$PBACKUP" "$PUNIT_PATH" && sudo -n systemctl daemon-reload && sudo -n systemctl restart "$PLANNER" || step "planner restore of $PBACKUP FAILED: reinstall it by hand"
}
planner_restore(){
  step "planner check failed: $1"
  planner_reinstate
  printf '@@result {"status":"restored","reason":"%s","backup":"%s"}\n' "$(printf '%s' "$1" | tr '"\\' "''")" "$PBACKUP"
  exit 5
}
[ -f "$PUNIT_PATH" ] || unstage "$PUNIT_PATH missing: the first planner unit for a cell is provisioned by hand"
python3 -c 'import json,sys; d=json.load(open(sys.argv[1])).get("device"); sys.exit(0 if d in ("cpu", "cuda") and (sys.argv[2] != "1" or d == "cuda") else 1)' "$D/planner-env.json" "$PCUDA" 2> /dev/null \
  || unstage "$D/planner-env.json is missing or says the env cannot run on the GPU: the build's planner device probe did not pass (realtime-core build)"
planner_idle || unstage "the planner is planning ($PORIGIN/api/motion/progress names a stage): wait for the plan to finish"
PREV_REV=$(planner_unit_value "$PUNIT_PATH" revision)
step "planner replacing $(planner_unit_value "$PUNIT_PATH" workdir) (revision ${PREV_REV:-none})"
if ! grep -qxF @MARKER@ "$PUNIT_PATH"; then
  install -d -m 0700 "$PSTATE/hand-made" && cp -p "$PUNIT_PATH" "$PSTATE/hand-made/@PLANNER_NAME@.$(date -u +%Y%m%dT%H%M%SZ)" || unstage "cannot keep the hand-made $PUNIT_PATH in $PSTATE/hand-made"
  step "planner unit $PUNIT_PATH was made by hand: kept in $PSTATE/hand-made"
fi
cp -p "$PUNIT_PATH" "$PBACKUP" || unstage "cannot back up $PUNIT_PATH"
REV=$(git -C "$D/source" rev-parse HEAD)
printf '%s' @PLANNER_UNIT_B64@ | base64 -d | sed "s/@REV_TOKEN@/$REV/" > "$D/@PLANNER_NAME@.new" || unstage "cannot write the new planner unit"
grep -qxE 'Environment=AMR_WELD_PLANNER_SOURCE_REVISION=[0-9a-f]{40}' "$D/@PLANNER_NAME@.new" || unstage "the new planner unit has no source revision"
install -d -m 0700 "$PDENSE" || unstage "cannot create $PDENSE"
old=$(sed -n 's/^ExecStart=.*--dense-store-dir \([^ ]*\).*/\1/p' "$PUNIT_PATH" | head -n 1)
if [ -n "$old" ] && [ "$old" != "$PDENSE" ] && [ -d "$old" ]; then
  cp -an "$old/." "$PDENSE/" || unstage "cannot copy the dense store $old into $PDENSE"
  step "planner dense store $PDENSE now also holds the $(ls -A "$old" | wc -l) entries of $old (left in place)"
fi
step "planner unit backup $PBACKUP; new unit $D/@PLANNER_NAME@.new"
maint_phase "restarting the weld planner on $(basename "$D")"
PSWAPPED=1
sudo -n install -m 644 "$D/@PLANNER_NAME@.new" "$PUNIT_PATH" || planner_restore "cannot install $PUNIT_PATH"
sudo -n systemctl daemon-reload || planner_restore "daemon-reload failed"
sudo -n systemctl restart "$PLANNER" || planner_restore "planner restart failed"
PLANNER_FAIL=""
planner_check "$D/source/weld_planner/v1/.pixi/envs/$ENV/bin/python" "$D/source/weld_planner/v1" "$PCUDA" "$REV" || planner_restore "$PLANNER_FAIL"
step "planner runs $D at $REV on its $ENV env"
step "phase olp"
`, "@PLANNER_NAME@", ctx.Deploy.OLP.PlannerUnit, "@PSTATE@", shellQuote(p.StateDir), "@PDENSE@", shellQuote(p.DenseDir()), "@PCUDA@", flagShell(*p.CUDA),
		"@MARKER@", shellQuote(cellDeployPlannerManagedMarker),
		"@REV_TOKEN@", cellDeployPlannerRevisionToken, "@PLANNER_UNIT_B64@", shellQuote(base64.StdEncoding.EncodeToString([]byte(unit))))
}

// cellDeployPlannerRollbackShell reinstalls the planner backup an install
// saved beside the OLP backup in the current tree, if there is one (an
// install without a managed planner saved none), and checks health from the
// python and directory that backup runs, with whatever cuda it reports, and
// its commit when the backup is a rendered unit (a hand-made unit's planner
// predates source_revision in health).
const cellDeployPlannerRollbackShell = `PUNIT_PATH=/etc/systemd/system/$PLANNER
shopt -s nullglob
set -- "$CUR/$PLANNER".bak-*
shopt -u nullglob
if [ "$#" = 1 ] && [ -f "$1" ]; then
  PBACKUP=$1
  step "phase planner"
  planner_idle || refuse "the planner is planning ($PORIGIN/api/motion/progress names a stage): wait for the plan to finish"
  cp -p "$PUNIT_PATH" "$CUR/$PLANNER.rolled-back-$(date -u +%Y%m%dT%H%M%SZ)" || refuse "cannot keep $PUNIT_PATH"
  step "planner rollback: reinstalling $PBACKUP ($(planner_unit_value "$PBACKUP" workdir))"
  sudo -n install -m 644 "$PBACKUP" "$PUNIT_PATH" && sudo -n systemctl daemon-reload && sudo -n systemctl restart "$PLANNER" \
    || { printf '@@result {"status":"failed","reason":"install or restart of %s failed"}\n' "$PBACKUP"; exit 1; }
  prev=""; grep -qxF "$PMARKER" "$PBACKUP" && prev=$(planner_unit_value "$PBACKUP" revision)
  PLANNER_FAIL=""
  planner_check "$(planner_unit_value "$PBACKUP" exec)" "$(planner_unit_value "$PBACKUP" workdir)" "" "$prev" \
    || { printf '@@result {"status":"failed","reason":"%s"}\n' "$(printf '%s' "$PLANNER_FAIL" | tr '"\\' "''")"; exit 1; }
  step "phase olp"
elif [ "$#" -gt 1 ]; then
  refuse "$CUR holds $# planner unit backups ($PLANNER.bak-*); expected at most one from olp install"
else
  step "planner unit left as it is: $CUR holds no planner backup (that install did not manage the planner)"
fi
`
