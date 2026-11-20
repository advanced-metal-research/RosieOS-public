package main

import (
	"bytes"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"time"
)

// Steam Deck v5 pendant build and install (tier 4), driven by the cell
// manifest's top-level "pendant_deploy" block:
//
//	"pendant_deploy": {
//	  "builder": {"kind": "container", "image": "rosie-v5-build:22.04", "platform": "linux/amd64"},
//	  "decks": [{"name": "<deck>", "transport": "tailscale", "ssh_user": "deck", "host": "<deck>",
//	             "app_dir": "/home/deck/devkit-game/RosiePendantV5", "steam_app_id": 2312498980}]
//	}
//
// builder.kind "ssh" names a native linux/amd64 Ubuntu 22.04 builder
// (ssh_target, work_dir) instead of the Mac's emulated container. Nothing is
// hard-coded per Deck; a cell without the block cannot use these commands.
//
// The v5 component's own scripts do the work; this command adds the cell
// manifest's hosts, the run journal, transport, upload and hash checks.
// `pendant build` packages one commit (never the working tree): a git bundle
// into a persistent work tree, where steamdeck/real/v5/container/build.sh runs
// make all, make check and package.sh; the Go OLP server is cross-compiled
// natively on this machine and handed to package.sh. `pendant install`,
// `rollback` and `status` run steamdeck/real/v5/deck-install.sh on the Deck:
// it replaces the package-owned files of the app directory and keeps every
// site file (pendant.env, olp-cells.json, private/, credentials, sim/), since
// the package's SHA256SUMS names what it owns. While a pendant runs on the
// Deck it refuses while the robot behind that pendant's OLP is armed,
// enabled, leased or running a program (closing the pendant sends it Stop);
// with none running it checks no robot. A failed check restores the
// previous package (exit 5).

type pendantDeployConfig struct {
	Builder pendantBuilder `json:"builder"`
	Decks   []pendantDeck  `json:"decks"`
}

type pendantBuilder struct {
	Kind      string `json:"kind"`
	Image     string `json:"image,omitempty"`
	Platform  string `json:"platform,omitempty"`
	SSHTarget string `json:"ssh_target,omitempty"`
	WorkDir   string `json:"work_dir,omitempty"`
}

type pendantDeck struct {
	Name       string         `json:"name"`
	Transport  string         `json:"transport"`
	SSHUser    string         `json:"ssh_user"`
	Host       string         `json:"host"`
	AppDir     string         `json:"app_dir"`
	SteamAppID uint32         `json:"steam_app_id,omitempty"`
	Tunnel     *pendantTunnel `json:"tunnel,omitempty"`
	// Launcher is the site script in app_dir that the steam_app_id shortcut
	// runs instead of run.sh: it exports ROSIE_PENDANT_OLP_ORIGIN and execs
	// run.sh (for example a Deck's run-<relay>.sh). Empty: the shortcut runs run.sh
	// and pendant.env alone names the origin.
	Launcher string `json:"launcher,omitempty"`
	// AppOwner is the cell id of the manifest that installs the app in
	// app_dir, when this cell's shortcut runs another cell's install (one app
	// per Deck, one launcher per cell: a Deck's launcher for one arm runs the
	// app another arm's manifest installs). This cell's install then only writes
	// its cell into its launcher; it never swaps the package or stops the
	// app, so two manifests do not take turns replacing one app directory.
	AppOwner string `json:"app_owner,omitempty"`
}

// shared: another cell's manifest installs this Deck's app.
func (d pendantDeck) shared(cell string) bool {
	return d.AppOwner != "" && d.AppOwner != cell
}

var (
	pendantImage     = regexp.MustCompile(`^[a-z0-9][a-z0-9._/-]*(:[A-Za-z0-9._-]+)?$`)
	pendantSSHTarget = regexp.MustCompile(`^[A-Za-z0-9._-]+@[A-Za-z0-9._:-]+$`)
	pendantHost      = regexp.MustCompile(`^[A-Za-z0-9._:-]+$`)
	pendantLauncher  = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]*\.sh$`)
)

// Docker volumes that persist the container builder's work tree and caches
// between builds (incremental make, node_modules, the npm cache).
const (
	pendantWorkVolume  = "rosie-v5-work"
	pendantCacheVolume = "rosie-v5-cache"
	pendantNPMVolume   = "rosie-v5-npm"
)

// pendantStripNotes removes "_"-prefixed keys from objects at any depth.
func pendantStripNotes(v any) any {
	switch t := v.(type) {
	case map[string]any:
		for k, e := range t {
			if strings.HasPrefix(k, "_") {
				delete(t, k)
				continue
			}
			t[k] = pendantStripNotes(e)
		}
	case []any:
		for i, e := range t {
			t[i] = pendantStripNotes(e)
		}
	}
	return v
}

func decodePendantDeployConfig(manifest []byte) (pendantDeployConfig, error) {
	var root map[string]json.RawMessage
	if err := json.Unmarshal(manifest, &root); err != nil {
		return pendantDeployConfig{}, fmt.Errorf("cell manifest is not a JSON object: %w", err)
	}
	raw, ok := root["pendant_deploy"]
	if !ok {
		return pendantDeployConfig{}, fmt.Errorf("cell manifest has no pendant_deploy block; add one (docs/DEPLOY_AND_TEST_WORKFLOW.md, \"Steam Deck pendant\")")
	}
	var generic any
	if err := json.Unmarshal(raw, &generic); err != nil {
		return pendantDeployConfig{}, err
	}
	stripped, _ := json.Marshal(pendantStripNotes(generic))
	var c pendantDeployConfig
	d := json.NewDecoder(bytes.NewReader(stripped))
	d.DisallowUnknownFields()
	if err := d.Decode(&c); err != nil {
		return pendantDeployConfig{}, fmt.Errorf("decode pendant_deploy: %w", err)
	}
	return c, c.validate()
}

// sharedOnly: every Deck runs an app another manifest installs, so this
// block needs no builder.
func (c pendantDeployConfig) sharedOnly() bool {
	for _, d := range c.Decks {
		if d.AppOwner == "" {
			return false
		}
	}
	return len(c.Decks) > 0
}

func (c pendantDeployConfig) validate() error {
	b := c.Builder
	switch b.Kind {
	case "":
		if !c.sharedOnly() {
			return fmt.Errorf("pendant_deploy.builder.kind must be container or ssh (only a block whose Decks all name app_owner may omit the builder)")
		}
	case "container":
		if !pendantImage.MatchString(b.Image) || (b.Platform != "linux/amd64" && b.Platform != "linux/arm64") {
			return fmt.Errorf("pendant_deploy.builder: a container builder needs image (name:tag) and platform linux/amd64 (the Deck is x86_64)")
		}
		if b.SSHTarget != "" || b.WorkDir != "" {
			return fmt.Errorf("pendant_deploy.builder: ssh_target and work_dir are for kind ssh")
		}
	case "ssh":
		if !pendantSSHTarget.MatchString(b.SSHTarget) {
			return fmt.Errorf("pendant_deploy.builder.ssh_target must be user@host of a native linux/amd64 Ubuntu 22.04 builder, got %q", b.SSHTarget)
		}
		if err := cellDeployCleanAbs("pendant_deploy.builder.work_dir", b.WorkDir); err != nil {
			return err
		}
	default:
		return fmt.Errorf("pendant_deploy.builder.kind must be container or ssh, got %q", b.Kind)
	}
	if len(c.Decks) == 0 {
		return fmt.Errorf("pendant_deploy.decks is empty")
	}
	seen := map[string]bool{}
	for i, d := range c.Decks {
		if !cellDeployToken.MatchString(d.Name) || seen[d.Name] {
			return fmt.Errorf("pendant_deploy.decks[%d].name must be a unique token, got %q", i, d.Name)
		}
		seen[d.Name] = true
		if d.Transport != "tailscale" && d.Transport != "ssh" {
			return fmt.Errorf("pendant_deploy.decks[%d].transport must be tailscale or ssh", i)
		}
		if !cellDeployToken.MatchString(d.SSHUser) || !pendantHost.MatchString(d.Host) {
			return fmt.Errorf("pendant_deploy.decks[%d] needs ssh_user and host", i)
		}
		if err := cellDeployCleanAbs(fmt.Sprintf("pendant_deploy.decks[%d].app_dir", i), d.AppDir); err != nil {
			return err
		}
		if d.Launcher != "" && (!pendantLauncher.MatchString(d.Launcher) || d.Launcher == "run.sh") {
			return fmt.Errorf("pendant_deploy.decks[%d].launcher must be the file name of a site script in app_dir other than run.sh (name.sh), got %q", i, d.Launcher)
		}
		if d.AppOwner != "" {
			if !cellDeployToken.MatchString(d.AppOwner) {
				return fmt.Errorf("pendant_deploy.decks[%d].app_owner must be the cell id of the manifest that installs app_dir, got %q", i, d.AppOwner)
			}
			// Every launcher of a shared app reads its pendant.env: a cell
			// written there would be every shortcut's.
			if d.Launcher == "" {
				return fmt.Errorf("pendant_deploy.decks[%d] names app_owner but no launcher: a shared app's pendant.env is read by every shortcut, so this cell's id must go into its own launcher", i)
			}
		}
		if d.Tunnel != nil {
			if err := d.Tunnel.validate(fmt.Sprintf("pendant_deploy.decks[%d].tunnel", i)); err != nil {
				return err
			}
		}
	}
	return nil
}

func (c pendantDeployConfig) deck(name string) (pendantDeck, error) {
	var names []string
	for _, d := range c.Decks {
		if d.Name == name {
			return d, nil
		}
		names = append(names, d.Name)
	}
	return pendantDeck{}, fmt.Errorf("--host %q is not a Deck of this cell's pendant_deploy (have: %s)", name, strings.Join(names, ", "))
}

// GameID is the 64-bit Steam game id of a non-Steam shortcut: the 32-bit
// shortcut app id shifted up, with the shortcut type bit (README v5: app
// 2729270736 launches as steam://rungameid/11722128553083404288).
func (d pendantDeck) GameID() uint64 {
	return uint64(d.SteamAppID)<<32 | 0x02000000
}

func (d pendantDeck) argv(script string) []string {
	return rtCoreConfigRemoteArgv(d.Transport, d.SSHUser, d.Host, d.Host, script)
}

// pendantSetup loads the cell context (cell_deploy, for the run journal and
// the cell lock) and the pendant_deploy block.
func pendantSetup(name string, args []string, stderr io.Writer, define func(*flag.FlagSet), common *cellDeployCommonFlags, allowed ...string) (cellDeployContext, pendantDeployConfig, func(), error) {
	ctx, restore, err := cellDeploySetup(name, args, stderr, define, common, true, allowed...)
	if err != nil {
		return ctx, pendantDeployConfig{}, nil, err
	}
	raw, err := os.ReadFile(resolveRobotV4ManifestPath(selectedRobotV4ManifestPath()))
	if err != nil {
		restore()
		return ctx, pendantDeployConfig{}, nil, err
	}
	c, err := decodePendantDeployConfig(raw)
	if err != nil {
		restore()
		return ctx, pendantDeployConfig{}, nil, fmt.Errorf("%s: %w", ctx.Manifest, err)
	}
	return ctx, c, restore, nil
}

const pendantUsage = `usage: pendant <build|install|rollback|status|tunnel> --manifest cell.json [flags]

Tier 4: the Steam Deck v5 pendant. Builder and Decks come from the cell
manifest's pendant_deploy block. Touches no robot; while a pendant runs on
the Deck, install refuses while the robot behind its OLP is armed, enabled,
leased or running.

  build     Package one commit: --commit <sha> [--with-tesseract-env] [--clean]
            In the builder's persistent work tree (container on this machine,
            or a native linux/amd64 builder); the Go OLP server is
            cross-compiled natively here. Output:
            ~/.rosie/pendant-packages/RosiePendantV5-<commit>.tar
  install   --host <deck> [--commit <sha> | --package <tar>] [--no-relaunch]
            Replace the package-owned files, keep pendant.env, olp-cells.json,
            private/ and other site files; verify SHA256SUMS and libraries;
            relaunch the Steam shortcut if the app was running and check the
            pendant's OLP health;
            a failed check restores the previous package (exit 5).
            A Deck naming app_owner runs another cell's install: only this
            cell is written into its launcher ([--commit] must be the app's).
  rollback  --host <deck> [--no-relaunch]   Reinstall the package install kept.
  status    --host <deck>                   Read-only; with the Deck's tunnel.
  tunnel    install|status|logs|restart|stop|rollback --host <deck>: the Deck's
            self-healing ssh tunnel to the cell's OLP (systemd --user);
            go run . pendant tunnel --help
All take --dry-run (prints what would run, contacts nothing) and --transport
for the cell's relay (the cell lock); a Deck's own transport is in the manifest.
`

func runPendant(args []string, stdout, stderr io.Writer) error {
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, pendantUsage)
		return nil
	}
	switch args[0] {
	case "build":
		return runPendantBuild(args[1:], stdout, stderr)
	case "install":
		return runPendantInstall(args[1:], stdout, stderr, false)
	case "rollback":
		return runPendantInstall(args[1:], stdout, stderr, true)
	case "status":
		return runPendantStatus(args[1:], stdout, stderr)
	case "tunnel":
		return runPendantTunnel(args[1:], stdout, stderr)
	}
	return fmt.Errorf("unknown pendant command %q", args[0])
}

// --- build -------------------------------------------------------------------

type pendantBuildPlan struct {
	Commit, Ref      string
	Staging          string // this machine: in/ and out/
	WithEnv, Clean   bool
	Builder          pendantBuilder
	PackageDir       string // this machine: finished packages
	CrossCompiledOLP bool
}

func (p pendantBuildPlan) flags() []string {
	var out []string
	if p.WithEnv {
		out = append(out, "--with-tesseract-env")
	}
	if p.Clean {
		out = append(out, "--clean")
	}
	return out
}

// pendantContainerArgv is the docker command of a container build: the
// staging in/ read-only, out/ for the tarball, persistent work and caches.
func pendantContainerArgv(p pendantBuildPlan) []string {
	argv := []string{"docker", "run", "--rm", "--platform", p.Builder.Platform,
		"-v", filepath.Join(p.Staging, "in") + ":/in:ro", "-v", filepath.Join(p.Staging, "out") + ":/out",
		"-v", pendantWorkVolume + ":/work", "-v", pendantCacheVolume + ":/root/.cache", "-v", pendantNPMVolume + ":/root/.npm",
		p.Builder.Image, "bash", "/in/build.sh", p.Commit, p.Ref, "/in", "/out", "/work"}
	return append(argv, p.flags()...)
}

// pendantSSHBuildScript runs build.sh on a native builder from the staged
// files the Mac uploaded to <work_dir>/in.
func pendantSSHBuildScript(p pendantBuildPlan) string {
	w := p.Builder.WorkDir
	cmd := []string{"bash", shellQuote(path.Join(w, "in", "build.sh")), shellQuote(p.Commit), shellQuote(p.Ref), shellQuote(path.Join(w, "in")), shellQuote(path.Join(w, "out")), shellQuote(path.Join(w, "work"))}
	cmd = append(cmd, p.flags()...)
	return "set -euo pipefail\n" + strings.Join(cmd, " ") + "\n"
}

// pendantCrossCompile builds the linux/amd64 OLP server of commit natively:
// the OLP module and its replace targets from `git archive`, never the
// working tree.
var pendantCrossCompile = func(repo, commit, out string, p *rtCoreProgress) error {
	dir, err := os.MkdirTemp("", "rosie-pendant-olp-*")
	if err != nil {
		return err
	}
	defer os.RemoveAll(dir)
	archive := exec.Command("git", "-C", repo, "archive", "--format=tar", commit, "offline-programming/v1", "rt-core", "robot_description/go")
	untar := exec.Command("tar", "-x", "-C", dir)
	pipe, err := archive.StdoutPipe()
	if err != nil {
		return err
	}
	untar.Stdin = pipe
	if err := untar.Start(); err != nil {
		return err
	}
	if err := archive.Run(); err != nil {
		return fmt.Errorf("git archive %s: %w", cellDeployShort(commit), err)
	}
	if err := untar.Wait(); err != nil {
		return err
	}
	t := time.Now()
	build := exec.Command("go", "build", "-trimpath", "-o", out, ".")
	build.Dir = filepath.Join(dir, "offline-programming", "v1")
	build.Env = append(os.Environ(), "GOOS=linux", "GOARCH=amd64", "CGO_ENABLED=0")
	if output, err := build.CombinedOutput(); err != nil {
		return fmt.Errorf("cross-compile the OLP server: %v: %s", err, strings.TrimSpace(string(output)))
	}
	p.Step("olp", fmt.Sprintf("OLP server cross-compiled natively for linux/amd64 in %.1fs", time.Since(t).Seconds()))
	return nil
}

// pendantRunLocal runs a local command (docker) and streams its output
// through the progress printer, so build.sh's @@step and @@result lines
// count as a session's.
var pendantRunLocal = func(argv []string, p *rtCoreProgress) (int, error) {
	return cellDeployExec(argv, "", nil, p)
}

// pendantEnsureImage builds the container image from steamdeck/real/v5/container
// when the tag is missing or was built from another Dockerfile: the image
// carries the Dockerfile's sha256 as a label, so a package added there (for
// example libopenh264-dev, 2026-10-19) reaches every builder's next build.
var pendantEnsureImage = func(repo string, b pendantBuilder, p *rtCoreProgress) error {
	contextDir := filepath.Join(repo, "steamdeck", "real", "v5", "container")
	sum, err := cellDeployFileSHA256(filepath.Join(contextDir, "Dockerfile"))
	if err != nil {
		return err
	}
	have, err := exec.Command("docker", "image", "inspect", "--format", `{{index .Config.Labels "`+pendantImageLabel+`"}}`, b.Image).Output()
	switch {
	case err == nil && strings.TrimSpace(string(have)) == sum:
		return nil
	case err == nil:
		p.Step("image", "rebuilding "+b.Image+": steamdeck/real/v5/container/Dockerfile changed since it was built")
	default:
		p.Step("image", "building "+b.Image+" from steamdeck/real/v5/container (first build on this machine)")
	}
	code, err := pendantRunLocal([]string{"docker", "build", "--platform", b.Platform, "--label", pendantImageLabel + "=" + sum, "-t", b.Image, contextDir}, p)
	if err != nil || code != 0 {
		return fmt.Errorf("docker build %s failed (exit %d, %v)", b.Image, code, err)
	}
	return nil
}

const pendantImageLabel = "rosie.container.sha256"

func pendantPackageDir() (string, error) {
	home, err := os.UserHomeDir()
	if err != nil {
		return "", err
	}
	return filepath.Join(home, ".rosie", "pendant-packages"), nil
}

func pendantPackagePath(dir, commit string) string {
	return filepath.Join(dir, "RosiePendantV5-"+commit+".tar")
}

func runPendantBuild(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var commit string
	var withEnv, clean bool
	ctx, cfg, restore, err := pendantSetup("pendant build", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&commit, "commit", "HEAD", "commit to package (must be on a local ref; uncommitted work never ships)")
		fs.BoolVar(&withEnv, "with-tesseract-env", false, "also stage tesseract/v1's pixi environment (~1 GB): a Deck's first install and whenever its pixi.lock changes")
		fs.BoolVar(&clean, "clean", false, "delete the builder's work tree first (a from-scratch build)")
	}, &common, "--commit", "--with-tesseract-env", "--clean")
	if err != nil {
		return err
	}
	defer restore()
	if cfg.sharedOnly() {
		return fmt.Errorf("pendant build: this cell's Decks run the app %s's manifest installs (pendant_deploy.decks[].app_owner); build and install with that manifest, then pendant install here writes this cell into its launcher", cfg.Decks[0].AppOwner)
	}
	repo, err := repoRoot()
	if err != nil {
		return err
	}
	full, ref, err := cellDeployResolveCommit(repo, commit)
	if err != nil {
		return err
	}
	pkgDir, err := pendantPackageDir()
	if err != nil {
		return err
	}
	home, _ := os.UserHomeDir()
	plan := pendantBuildPlan{Commit: full, Ref: ref, Staging: filepath.Join(home, ".rosie", "pendant-build"), WithEnv: withEnv, Clean: clean,
		Builder: cfg.Builder, PackageDir: pkgDir, CrossCompiledOLP: true}
	if common.DryRun {
		fmt.Fprintf(stdout, "pendant build cell=%s commit=%s ref=%s builder=%s output=%s (dry run: nothing built, nothing contacted)\n", ctx.CellID, full, ref, cfg.Builder.Kind, pendantPackagePath(pkgDir, full))
		fmt.Fprintf(stdout, "# 1. git bundle create %s/in/src.bundle %s [^<last built commit>]; copy steamdeck/real/v5/container/build.sh\n", plan.Staging, ref)
		fmt.Fprintf(stdout, "# 2. natively: git archive %s offline-programming/v1 rt-core robot_description/go | GOOS=linux GOARCH=amd64 CGO_ENABLED=0 go build -trimpath -o %s/in/offline-programming-linux\n", cellDeployShort(full), plan.Staging)
		if cfg.Builder.Kind == "container" {
			fmt.Fprintf(stdout, "# 3. docker image inspect %s (label rosie.container.sha256 = the Dockerfile's) || docker build --platform %s -t %s steamdeck/real/v5/container\n# 4. %s\n", cfg.Builder.Image, cfg.Builder.Platform, cfg.Builder.Image, strings.Join(pendantContainerArgv(plan), " "))
		} else {
			fmt.Fprintf(stdout, "# 3. upload in/ to %s:%s/in\n# 4. on %s:\n%s# 5. fetch %s/out/RosiePendantV5-%s.tar\n", cfg.Builder.SSHTarget, cfg.Builder.WorkDir, cfg.Builder.SSHTarget, pendantSSHBuildScript(plan), cfg.Builder.WorkDir, full)
		}
		return nil
	}
	run, err := newCellDeployRun(ctx, "pendant build", common.Transport, stdout)
	if err != nil {
		return err
	}
	fail := func(e error) error {
		run.finish("failed", deployExitCode(e), e.Error())
		return e
	}
	in, out := filepath.Join(plan.Staging, "in"), filepath.Join(plan.Staging, "out")
	for _, d := range []string{in, out, pkgDir} {
		if err := os.MkdirAll(d, 0o700); err != nil {
			return fail(err)
		}
	}
	base := ""
	if last, err := os.ReadFile(filepath.Join(in, "last-commit")); err == nil && !clean {
		base = strings.TrimSpace(string(last))
	}
	kind, err := cellDeployBundle(repo, ref, base, filepath.Join(in, "src.bundle"))
	if err != nil {
		return fail(err)
	}
	run.p.Step("bundle", kind+" bundle of "+ref)
	script, err := os.ReadFile(filepath.Join(repo, "steamdeck", "real", "v5", "container", "build.sh"))
	if err != nil {
		return fail(err)
	}
	if err := os.WriteFile(filepath.Join(in, "build.sh"), script, 0o755); err != nil {
		return fail(err)
	}
	if err := pendantCrossCompile(repo, full, filepath.Join(in, "offline-programming-linux"), run.p); err != nil {
		return fail(err)
	}
	var code int
	switch cfg.Builder.Kind {
	case "container":
		if err := pendantEnsureImage(repo, cfg.Builder, run.p); err != nil {
			return fail(err)
		}
		argv := pendantContainerArgv(plan)
		run.p.Record("container command", strings.Join(argv, " "))
		code, err = pendantRunLocal(argv, run.p)
	case "ssh":
		code, err = pendantSSHBuild(run, plan)
	}
	if err != nil || code != 0 {
		return fail(cellDeployExit(firstNonZero(code, cellDeployExitFailed), "pendant build failed (exit %d, %v): see %s", code, err, run.logPath))
	}
	var result struct {
		Package string `json:"package"`
		SHA256  string `json:"sha256"`
	}
	if err := json.Unmarshal(run.p.result, &result); err != nil || !rtCoreHexSHA256.MatchString(result.SHA256) {
		return fail(cellDeployExit(cellDeployExitFailed, "pendant build printed no package result"))
	}
	src := filepath.Join(out, result.Package)
	if cfg.Builder.Kind == "ssh" {
		src = filepath.Join(out, "RosiePendantV5-"+full+".tar")
	}
	if got, err := cellDeployFileSHA256(src); err != nil || got != result.SHA256 {
		return fail(cellDeployExit(cellDeployExitFailed, "package %s sha256 %s, the builder reported %s", src, got, result.SHA256))
	}
	dst := pendantPackagePath(pkgDir, full)
	if err := os.Rename(src, dst); err != nil {
		return fail(err)
	}
	_ = os.WriteFile(dst+".sha256", []byte(result.SHA256+"  "+filepath.Base(dst)+"\n"), 0o600)
	_ = os.WriteFile(filepath.Join(in, "last-commit"), []byte(full+"\n"), 0o600)
	run.p.Step("done", fmt.Sprintf("%s sha256=%s; install: go run . pendant install --manifest %s --host <deck> --commit %s", dst, result.SHA256, ctx.Manifest, cellDeployShort(full)))
	run.finish("ok", 0, dst)
	return nil
}

// deployExitCode is the exit code a command error carries (1 for any other
// error, 0 for none).
func deployExitCode(err error) int {
	var coded interface{ RealtimeCoreExitCode() int }
	if errors.As(err, &coded) {
		return coded.RealtimeCoreExitCode()
	}
	if err != nil {
		return 1
	}
	return 0
}

func firstNonZero(values ...int) int {
	for _, v := range values {
		if v != 0 {
			return v
		}
	}
	return 0
}

// pendantSSHBuild uploads in/ to a native builder, runs build.sh there and
// fetches the tarball into out/.
func pendantSSHBuild(run *cellDeployRun, p pendantBuildPlan) (int, error) {
	user, host, _ := strings.Cut(p.Builder.SSHTarget, "@")
	argv := func(script string) []string { return rtCoreConfigRemoteArgv("ssh", user, host, host, script) }
	for _, name := range []string{"src.bundle", "build.sh", "offline-programming-linux"} {
		dst := path.Join(p.Builder.WorkDir, "in", name)
		code, err := cellDeployUpload(argv("set -eu\ninstall -d -m 0700 "+shellQuote(path.Dir(dst))+"\ncat > "+shellQuote(dst+".part")+"\nmv -f "+shellQuote(dst+".part")+" "+shellQuote(dst)+"\n"), filepath.Join(p.Staging, "in", name), run.p)
		if err != nil || code != 0 {
			return firstNonZero(code, 1), fmt.Errorf("upload %s: %v", name, err)
		}
	}
	code, err := rtCoreConfigRunRemote(argv(pendantSSHBuildScript(p)), run.p)
	if err != nil || code != 0 {
		return code, err
	}
	f, err := os.Create(filepath.Join(p.Staging, "out", "RosiePendantV5-"+p.Commit+".tar"))
	if err != nil {
		return 1, err
	}
	defer f.Close()
	return cellDeployFetch(argv("cat "+shellQuote(path.Join(p.Builder.WorkDir, "out", "RosiePendantV5-"+p.Commit+".tar"))), f, run.p)
}

// --- install, rollback, status --------------------------------------------------

// pendantDeckScriptPath is the Deck half of install, rollback and status. The
// v5 component owns it beside package.sh, whose layout and SHA256SUMS it
// installs: the refusal while the robot is in use, the staged SHA256SUMS and
// library checks, the swap of package-owned files only, the relaunch and
// health check, and the restore on a failed check. It is read from this
// checkout, like the command itself.
const pendantDeckScriptPath = "steamdeck/real/v5/deck-install.sh"

// pendantStartWait bounds each wait of a pendant stop or start, in seconds:
// run.sh waits up to 30 s for its OLP and the server's shutdown takes up to
// 25 s (steamdeck/real/v5/run.sh, both unverified bounds).
var pendantStartWait = 30

// pendantDeckRun is what deck-install.sh does on one Deck.
type pendantDeckRun struct {
	// install, rollback or status; cell on a Deck whose app another cell's
	// manifest installs: write this cell into its launcher, nothing else
	// (Commit, when set, must be the installed app's).
	Action    string
	Relaunch  bool
	TarSHA256 string // install: of $APP.incoming/package.tar
	Commit    string // install
	// Cell: the manifest's cell id. Install writes it into the launcher the
	// shortcut runs (or pendant.env) as ROSIE_PENDANT_EXPECT_CELL, which
	// run.sh passes to the app as --expect-cell, and refuses a launcher that
	// names another cell; status shows it.
	Cell string
	// Origin: the Deck's tunnel origin when its manifest entry has a tunnel
	// block. Install writes it beside the cell as ROSIE_PENDANT_OLP_ORIGIN
	// when neither pendant.env nor the launcher names one, so the pendant
	// uses the cell's OLP through the tunnel instead of a Deck-local server,
	// and refuses a file that names another origin.
	Origin string
}

// tunnelOrigin is the pendant's OLP origin through the Deck's tunnel; empty
// without a tunnel block.
func (d pendantDeck) tunnelOrigin() string {
	if d.Tunnel == nil {
		return ""
	}
	return fmt.Sprintf("http://127.0.0.1:%d", d.Tunnel.OLPPort)
}

// pendantDeckVars are the variables deck-install.sh reads, one per line.
func pendantDeckVars(d pendantDeck, r pendantDeckRun) string {
	relaunch := "0"
	if r.Relaunch {
		relaunch = "1"
	}
	vars := []string{"ACTION=" + r.Action, "APP=" + shellQuote(d.AppDir), "GAMEID=" + strconv.FormatUint(d.GameID(), 10),
		"WAIT=" + strconv.Itoa(pendantStartWait), "RELAUNCH=" + relaunch, "TAR_SHA=" + shellQuote(r.TarSHA256),
		"COMMIT=" + shellQuote(r.Commit), "LAUNCHER=" + shellQuote(d.Launcher), "EXPECT_CELL=" + shellQuote(r.Cell), "OLP_ORIGIN=" + shellQuote(r.Origin),
		"APP_OWNER=" + shellQuote(d.AppOwner), "IDLE_PY=" + shellQuote(cellDeployOLPIdlePy)}
	return strings.Join(vars, "\n") + "\n"
}

// pendantDeckScript is deck-install.sh after the variables it reads.
func pendantDeckScript(d pendantDeck, r pendantDeckRun) (string, error) {
	repo, err := repoRoot()
	if err != nil {
		return "", err
	}
	body, err := os.ReadFile(filepath.Join(repo, pendantDeckScriptPath))
	if err != nil {
		return "", err
	}
	return pendantDeckVars(d, r) + string(body), nil
}

// pendantDeckDryRun prints the Deck session a dry run would open.
func pendantDeckDryRun(stdout io.Writer, d pendantDeck, r pendantDeckRun) {
	fmt.Fprintf(stdout, "# Deck session: %s, stdin: these variables, then %s\n%s", strings.Join(pendantDeckArgv(d), " "), pendantDeckScriptPath, pendantDeckVars(d, r))
}

// pendantDeckArgv runs a script the Deck reads on stdin.
func pendantDeckArgv(d pendantDeck) []string {
	return d.argv("bash -s")
}

// pendantDeckOnline refuses a tailscale Deck that Tailscale lists as offline,
// before any session opens. A sleeping Deck otherwise cost a 75 s TCP timeout
// on port 22 and then an empty reason (a Steam Deck, 2026-10-08 00:40 PDT).
// Nil when Tailscale cannot answer or does not list the host: ssh decides.
var pendantDeckOnline = func(d pendantDeck) error {
	if d.Transport != "tailscale" {
		return nil
	}
	out, err := exec.Command("tailscale", "status", "--json").Output()
	if err != nil {
		return nil
	}
	return pendantTailscaleOnline(out, d, time.Now())
}

// pendantTailscaleOnline reads `tailscale status --json` for the Deck's host.
func pendantTailscaleOnline(status []byte, d pendantDeck, now time.Time) error {
	var s struct {
		Peer map[string]struct {
			HostName string
			DNSName  string
			Online   bool
			LastSeen time.Time
		}
	}
	if json.Unmarshal(status, &s) != nil {
		return nil
	}
	for _, p := range s.Peer {
		dns := strings.TrimSuffix(p.DNSName, ".")
		if !strings.EqualFold(p.HostName, d.Host) && !strings.EqualFold(dns, d.Host) && !strings.EqualFold(strings.SplitN(dns, ".", 2)[0], d.Host) {
			continue
		}
		if p.Online {
			return nil
		}
		seen := "never"
		if !p.LastSeen.IsZero() {
			seen = fmt.Sprintf("%s, %s ago", p.LastSeen.UTC().Format(time.RFC3339), now.Sub(p.LastSeen).Round(time.Minute))
		}
		return cellDeployExit(cellDeployExitRefused, "deck %s (%s) is offline in Tailscale (last seen %s): asleep, powered off or off the network. Wake it, check `tailscale status` lists it online, then rerun; nothing was changed", d.Name, d.Host, seen)
	}
	return nil
}

// pendantRunDeck sends script to the Deck on stdin, never in argv: the text
// names the app's binary, and in a command line it would make the script's
// own pgrep -f and pkill -f patterns match (and stop) the installer.
var pendantRunDeck = func(d pendantDeck, script string, p *rtCoreProgress) (int, error) {
	f, err := os.CreateTemp("", "rosie-pendant-deck-*.sh")
	if err != nil {
		return -1, err
	}
	defer os.Remove(f.Name())
	if _, err := f.WriteString(script); err != nil {
		f.Close()
		return -1, err
	}
	if err := f.Close(); err != nil {
		return -1, err
	}
	return cellDeployExec(pendantDeckArgv(d), f.Name(), nil, p)
}

func runPendantInstall(args []string, stdout, stderr io.Writer, rollback bool) error {
	var common cellDeployCommonFlags
	var host, commit, pkg string
	var noRelaunch bool
	name := "pendant install"
	if rollback {
		name = "pendant rollback"
	}
	allowed := []string{"--host", "--no-relaunch"}
	ctx, cfg, restore, err := pendantSetup(name, args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&host, "host", "", "Deck name from the manifest's pendant_deploy.decks (required)")
		fs.BoolVar(&noRelaunch, "no-relaunch", false, "do not start the Steam shortcut afterwards (the checks then stop at hashes and libraries)")
		if !rollback {
			fs.StringVar(&commit, "commit", "", "install ~/.rosie/pendant-packages/RosiePendantV5-<commit>.tar (from pendant build)")
			fs.StringVar(&pkg, "package", "", "install this package tarball (its SOURCE_COMMIT is taken from the file name)")
		}
	}, &common, append(allowed, "--commit", "--package")...)
	if err != nil {
		return err
	}
	defer restore()
	deck, err := cfg.deck(host)
	if err != nil {
		return err
	}
	if deck.shared(ctx.CellID) {
		return pendantInstallCell(ctx, common, stdout, name, deck, rollback, commit, pkg)
	}
	if rollback {
		r := pendantDeckRun{Action: "rollback", Relaunch: !noRelaunch}
		if common.DryRun {
			fmt.Fprintf(stdout, "pendant rollback cell=%s deck=%s app=%s (dry run: nothing contacted)\n", ctx.CellID, deck.Name, deck.AppDir)
			pendantDeckDryRun(stdout, deck, r)
			return nil
		}
		script, err := pendantDeckScript(deck, r)
		if err != nil {
			return err
		}
		return pendantSession(ctx, common, stdout, name, deck, script, nil)
	}
	full, tar, err := pendantResolvePackage(commit, pkg, common.DryRun)
	if err != nil {
		return err
	}
	sha := "<package-sha256>"
	if !common.DryRun {
		if sha, err = cellDeployFileSHA256(tar); err != nil {
			return err
		}
	}
	r := pendantDeckRun{Action: "install", Relaunch: !noRelaunch, TarSHA256: sha, Commit: full, Cell: ctx.CellID, Origin: deck.tunnelOrigin()}
	upload := pendantUploadScript(deck)
	if common.DryRun {
		fmt.Fprintf(stdout, "pendant install cell=%s deck=%s app=%s package=%s commit=%s (dry run: nothing contacted)\n# 1. upload (%s < %s)\n%s\n# 2.\n",
			ctx.CellID, deck.Name, deck.AppDir, tar, full, strings.Join(deck.argv("<upload>"), " "), tar, upload)
		pendantDeckDryRun(stdout, deck, r)
		return nil
	}
	script, err := pendantDeckScript(deck, r)
	if err != nil {
		return err
	}
	return pendantSession(ctx, common, stdout, name, deck, script, func(run *cellDeployRun) error {
		code, err := cellDeployUpload(deck.argv(upload), tar, run.p)
		if err != nil || code != 0 {
			return cellDeployExit(cellDeployExitFailed, "upload to %s failed (exit %d, %v)", deck.Name, code, err)
		}
		return nil
	})
}

// pendantInstallCell is install on a Deck whose app another cell's manifest
// installs (decks[].app_owner): it writes this cell into this cell's launcher
// and touches nothing else. No upload, no swap, no stop: the owner's install
// replaces the app both shortcuts run and relaunches the one that was open,
// and a second package swap from this manifest would close that pendant and
// could put the app at another commit. --commit, when given, must be the
// installed app's, so a stale app is reported instead of written over.
func pendantInstallCell(ctx cellDeployContext, common cellDeployCommonFlags, stdout io.Writer, name string, deck pendantDeck, rollback bool, commit, pkg string) error {
	if rollback {
		return cellDeployExit(cellDeployExitRefused, "%s: %s's app in %s is installed by %s's manifest; roll back with that manifest (this cell's %s keeps its cell)", name, deck.Name, deck.AppDir, deck.AppOwner, deck.Launcher)
	}
	if pkg != "" {
		return fmt.Errorf("%s: %s's app is installed by %s's manifest: install the package with that manifest; here give --commit (optional) to check the installed app is at it", name, deck.Name, deck.AppOwner)
	}
	full := ""
	if commit != "" {
		repo, err := repoRoot()
		if err != nil {
			return err
		}
		if full, _, err = cellDeployResolveCommit(repo, commit); err != nil {
			return err
		}
	}
	r := pendantDeckRun{Action: "cell", Commit: full, Cell: ctx.CellID, Origin: deck.tunnelOrigin()}
	if common.DryRun {
		fmt.Fprintf(stdout, "pendant install cell=%s deck=%s app=%s (installed by %s's manifest) launcher=%s: writes this cell into the launcher only (dry run: nothing contacted)\n",
			ctx.CellID, deck.Name, deck.AppDir, deck.AppOwner, deck.Launcher)
		pendantDeckDryRun(stdout, deck, r)
		return nil
	}
	script, err := pendantDeckScript(deck, r)
	if err != nil {
		return err
	}
	return pendantSession(ctx, common, stdout, name, deck, script, nil)
}

func pendantUploadScript(d pendantDeck) string {
	return rtCoreConfigReplace(`set -eu
I=@APP@.incoming
mkdir -p "$I"
cat > "$I/package.tar.part"
mv -f "$I/package.tar.part" "$I/package.tar"
printf '@@step 0 upload stored %s sha256=%s\n' "$I/package.tar" "$(sha256sum < "$I/package.tar" | cut -d' ' -f1)"
`, "@APP@", shellQuote(d.AppDir))
}

var pendantPackageName = regexp.MustCompile(`RosiePendantV5-([0-9a-f]{40})\.tar$`)

// pendantResolvePackage finds the tarball: --commit names a pendant build
// output; --package must carry its commit in the name (pendant build's).
func pendantResolvePackage(commit, pkg string, dryRun bool) (string, string, error) {
	if (commit == "") == (pkg == "") {
		return "", "", fmt.Errorf("pendant install needs exactly one of --commit or --package")
	}
	if pkg != "" {
		m := pendantPackageName.FindStringSubmatch(pkg)
		if m == nil {
			return "", "", fmt.Errorf("--package must be a pendant build output RosiePendantV5-<40-hex commit>.tar")
		}
		return m[1], pkg, nil
	}
	repo, err := repoRoot()
	if err != nil {
		return "", "", err
	}
	full, _, err := cellDeployResolveCommit(repo, commit)
	if err != nil {
		return "", "", err
	}
	dir, err := pendantPackageDir()
	if err != nil {
		return "", "", err
	}
	tar := pendantPackagePath(dir, full)
	if _, err := os.Stat(tar); err != nil && !dryRun {
		return "", "", fmt.Errorf("no package for %s at %s: run go run . pendant build --manifest <cell> --commit %s first", cellDeployShort(full), tar, cellDeployShort(full))
	}
	return full, tar, nil
}

// pendantSession runs one Deck script under the run journal; before is the
// optional upload.
func pendantSession(ctx cellDeployContext, common cellDeployCommonFlags, stdout io.Writer, name string, deck pendantDeck, script string, before func(*cellDeployRun) error) error {
	run, err := newCellDeployRun(ctx, name, common.Transport, stdout)
	if err != nil {
		return err
	}
	run.p.Step("pendant", fmt.Sprintf("deck %s (%s@%s, %s) app %s", deck.Name, deck.SSHUser, deck.Host, deck.Transport, deck.AppDir))
	if err := pendantDeckOnline(deck); err != nil {
		run.finish("refused", deployExitCode(err), err.Error())
		return err
	}
	if before != nil {
		if err := before(run); err != nil {
			run.finish("failed", deployExitCode(err), err.Error())
			return err
		}
	}
	run.p.Record(name+" (Deck script)", script)
	run.p.mu.Lock()
	run.p.result, run.p.refused = nil, nil
	run.p.mu.Unlock()
	code, err := pendantRunDeck(deck, script, run.p)
	if err == nil && code == cellDeployExitRestored {
		run.finish("restored", code, string(run.p.result))
		return cellDeployExit(code, "%s failed a check and the previous package was restored: %s", name, string(run.p.result))
	}
	if e := sessionOutcome(name, code, run.p.result, run.p.refused, err); e != nil {
		run.finish("failed", code, e.Error())
		return e
	}
	run.p.Step("done", string(run.p.result))
	run.finish("ok", 0, string(run.p.result))
	return nil
}

func runPendantStatus(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var host string
	ctx, cfg, restore, err := pendantSetup("pendant status", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&host, "host", "", "Deck name from the manifest's pendant_deploy.decks (required)")
	}, &common, "--host")
	if err != nil {
		return err
	}
	defer restore()
	deck, err := cfg.deck(host)
	if err != nil {
		return err
	}
	r := pendantDeckRun{Action: "status", Cell: ctx.CellID, Origin: deck.tunnelOrigin()}
	// A Deck with a tunnel block reports it from the same session: the tunnel
	// probe runs after deck-install.sh's status.
	var tunnel *pendantTunnelPlan
	if deck.Tunnel != nil {
		plan, err := pendantTunnelResolve(ctx, deck)
		if err != nil {
			return err
		}
		tunnel = &plan
	}
	if common.DryRun {
		fmt.Fprintf(stdout, "pendant status cell=%s deck=%s (read-only; dry run: nothing contacted)\n", ctx.CellID, deck.Name)
		pendantDeckDryRun(stdout, deck, r)
		if tunnel != nil {
			fmt.Fprintf(stdout, "# then the tunnel probe of %s (pendant tunnel status): %s\n", tunnel.Unit, tunnel.describe())
		}
		return nil
	}
	if err := pendantDeckOnline(deck); err != nil {
		return err
	}
	script, err := pendantDeckScript(deck, r)
	if err != nil {
		return err
	}
	if tunnel != nil {
		script += pendantTunnelScript(*tunnel, pendantTunnelOpts{Action: "status", Wait: time.Second, Lines: 1})
	}
	p := newRTCoreProgress(stdout, nil)
	code, err := pendantRunDeck(deck, script, p)
	if e := sessionOutcome("pendant status", code, nil, nil, err); e != nil || tunnel == nil {
		return e
	}
	if problems := pendantTunnelReport(p, *tunnel, p.probe); len(problems) > 0 {
		return cellDeployExit(cellDeployExitFailed, "pendant on %s: tunnel unhealthy:\n  - %s", deck.Name, strings.Join(problems, "\n  - "))
	}
	return nil
}
