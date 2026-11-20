package main

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"sort"
	"strings"
)

// Drift guards (owner rule 2026-10-11): robot-specific settings live in the
// repository, and a cell never runs a configuration git does not hold.
//
// On 2026-10-10 a `realtime-core config` on one arm applied speed feedforward off
// (C01.13/C01.14 = 0/0) from an uncommitted machine JSON as configuration
// 66c7300b. The repository still pinned 1/500, so the next release would have
// written the old values back into every drive's EEPROM (A6-EC C13.10 stores
// startup writes) and `release --plan` said nothing about it. Two guards:
//
//   - release and config refuse to apply from a working tree with uncommitted
//     changes under rt-core/config, robot_description or the cell manifest;
//   - when the running configuration matches no commit on the branch, release
//     and config say so loudly, compare the running compiled axes.conf with the
//     commit's, and refuse a change to any startup (drive) value unless the
//     operator names the running pin with --accept-config-drift.

// cellDeployConfigInputs are the repository paths whose edits reach a robot's
// configuration: machine configs, drive profiles and tuning catalogs, cell
// manifests (rt-core/config) and robot descriptions.
var cellDeployConfigInputs = []string{"rt-core/config", "robot_description"}

// cellDeployUncommittedConfig lists `git status --porcelain` entries (tracked
// edits and untracked files) under the configuration inputs and the cell
// manifest.
func cellDeployUncommittedConfig(repo, manifest string) ([]string, error) {
	args := append([]string{"status", "--porcelain", "--untracked-files=all", "--"}, cellDeployConfigInputs...)
	if manifest != "" {
		args = append(args, manifest)
	}
	out, err := robotV4GitOutput(repo, args...)
	if err != nil {
		return nil, err
	}
	var lines []string
	for _, line := range strings.Split(out, "\n") {
		if strings.TrimSpace(line) != "" {
			lines = append(lines, strings.TrimSpace(line))
		}
	}
	return lines, nil
}

// cellDeployUncommittedMessage is the shared refusal text.
func cellDeployUncommittedMessage(dirty []string) string {
	shown := dirty
	if len(shown) > 8 {
		shown = append(append([]string(nil), shown[:8]...), fmt.Sprintf("... %d more", len(dirty)-8))
	}
	return fmt.Sprintf("uncommitted changes under rt-core/config, robot_description or the cell manifest (%s): robot-specific settings live in the repository and a cell never runs a configuration git does not hold; commit them (machine JSON plus every pin) or stash them, then run again", strings.Join(shown, "; "))
}

// configPinCommits lists the commits reachable from commit whose change to a
// cell manifest added or removed pin: empty means no committed manifest on
// this branch ever carried the running configuration. Measured 0.2 s on this
// repository (2026-10-11), so it runs on every release.
func configPinCommits(repo, commit, pin string) ([]string, error) {
	out, err := robotV4GitOutput(repo, "log", "--format=%h", "-S"+pin, commit, "--", "rt-core/config/cells")
	if err != nil {
		return nil, err
	}
	return strings.Fields(out), nil
}

// configTuningDrift lists every startup value (startup_sdos) and stored/RAM
// choice (startup_trial) that differs between the running compiled axes.conf
// and the commit's, per axis. Old is the running value, New the commit's.
func configTuningDrift(liveAxes, commitAxes string) []rtCoreStartupChange {
	d := rtCoreConfigCompare(rtCoreCompiledFiles{Files: map[string]string{"axes.conf": liveAxes}}, rtCoreCompiledFiles{Files: map[string]string{"axes.conf": commitAxes}})
	changes := d.StartupChanges
	_, live := rtCoreParseAxesConf(liveAxes)
	_, next := rtCoreParseAxesConf(commitAxes)
	for i := 0; i < len(live) || i < len(next); i++ {
		var o, n string
		if i < len(live) {
			o = live[i]["startup_trial"]
		}
		if i < len(next) {
			n = next[i]["startup_trial"]
		}
		if o != n {
			changes = append(changes, rtCoreStartupChange{Axis: i, Key: "startup_trial", Old: o, New: n})
		}
	}
	sort.SliceStable(changes, func(a, b int) bool {
		if changes[a].Axis != changes[b].Axis {
			return changes[a].Axis < changes[b].Axis
		}
		return changes[a].Key < changes[b].Key
	})
	return changes
}

// configDrift is what a release or config knows about the running
// configuration's provenance.
type configDrift struct {
	Live, Commit string   // running pin, the pin about to be applied
	KnownIn      []string // commits whose cell manifest change carried Live
	Unknown      bool     // Live differs from Commit and no commit carried it
	Changes      []rtCoreStartupChange
	// Unproven says why the startup values could not be compared.
	Unproven string
	Note     string
}

func (d configDrift) lines(accept string) []string {
	if !d.Unknown {
		return nil
	}
	lines := []string{fmt.Sprintf("WARNING the cell runs configuration %.12s, which no commit on this branch pins (git log -S over rt-core/config/cells): it was applied from an uncommitted or unpushed machine config; this run replaces it with %.12s", d.Live, d.Commit)}
	if d.Note != "" {
		lines = append(lines, d.Note)
	}
	switch {
	case d.Unproven != "":
		lines = append(lines, "cannot compare the running drive values with the commit's: "+d.Unproven)
	case len(d.Changes) == 0:
		lines = append(lines, "running and committed startup (drive) values are identical on every axis: no drive value changes")
	default:
		lines = append(lines, fmt.Sprintf("%d drive value(s) the cell runs would change (running -> commit):", len(d.Changes)))
		for _, ch := range d.Changes {
			lines = append(lines, fmt.Sprintf("  axis %d %s: %s -> %s", ch.Axis, ch.Key, rtCoreShort(ch.Old), rtCoreShort(ch.New)))
		}
	}
	if accept != "" && accept == d.Live {
		lines = append(lines, "--accept-config-drift "+d.Live[:12]+": the operator accepts replacing the running configuration")
	}
	return lines
}

// refusal is nil unless the running configuration is unknown to git and the
// commit would change (or cannot be shown not to change) a drive value it
// carries. accept must name the running pin exactly.
func (d configDrift) refusal(accept string) error {
	if !d.Unknown || accept == d.Live {
		return nil
	}
	if accept != "" {
		return fmt.Errorf("--accept-config-drift %s does not name the running configuration %s", accept, d.Live)
	}
	if d.Unproven == "" && len(d.Changes) == 0 {
		return nil
	}
	what := fmt.Sprintf("it would change %d drive value(s) listed above", len(d.Changes))
	if d.Unproven != "" {
		what = "the running drive values cannot be compared (" + d.Unproven + ")"
	}
	return fmt.Errorf("refused: the cell runs configuration %s, which no commit pins, and %s. Bring the running values into the repository (machine JSON and every pin, from the relay's OLP deployment tree or %s on the servo host), or, if the owner wants them replaced, re-run with --accept-config-drift %s",
		d.Live, what, rtCoreConfigHistoryDir+"/machines/"+d.Live+".json", d.Live)
}

// releaseCompileAt compiles a commit's machine config with its own rtctl on
// this Mac. The detached worktree is removed after the compile.
func releaseCompileAt(repo, commit, machine string) (rtCoreCompiledFiles, string, error) {
	tmp, err := os.MkdirTemp("", "rosie-config-drift-")
	if err != nil {
		return rtCoreCompiledFiles{}, "", err
	}
	defer os.RemoveAll(tmp)
	tree := filepath.Join(tmp, "tree")
	if out, err := exec.Command("git", "-C", repo, "worktree", "add", "--detach", "--quiet", tree, commit).CombinedOutput(); err != nil {
		return rtCoreCompiledFiles{}, "", fmt.Errorf("git worktree add: %v: %s", err, strings.TrimSpace(string(out)))
	}
	defer exec.Command("git", "-C", repo, "worktree", "remove", "--force", tree).Run()
	goBin, err := exec.LookPath("go")
	if err != nil {
		goBin = filepath.Join(runtime.GOROOT(), "bin", "go")
	}
	build := exec.Command(goBin, "build", "-o", filepath.Join(tmp, "rtctl"), "./tools/rtctl/cmd/rtctl")
	build.Dir = filepath.Join(tree, "rt-core")
	if out, err := build.CombinedOutput(); err != nil {
		return rtCoreCompiledFiles{}, "", fmt.Errorf("go build rtctl: %v: %s", err, rtCoreShort(strings.TrimSpace(string(out))))
	}
	compile := exec.Command(filepath.Join(tmp, "rtctl"), "compile", "--config", filepath.Join(tree, filepath.FromSlash(machine)), "--out", filepath.Join(tmp, "out"))
	compile.Dir = filepath.Join(tree, "rt-core")
	out, err := compile.Output()
	if err != nil {
		return rtCoreCompiledFiles{}, "", fmt.Errorf("rtctl compile: %v", err)
	}
	fields := strings.Fields(string(out))
	if len(fields) == 0 {
		return rtCoreCompiledFiles{}, "", fmt.Errorf("rtctl compile printed no output directory")
	}
	dir := fields[len(fields)-1]
	files := rtCoreCompiledFiles{Files: make(map[string]string)}
	for _, name := range []string{"axes.conf", "argv.json"} {
		content, err := os.ReadFile(filepath.Join(dir, name))
		if err != nil {
			return rtCoreCompiledFiles{}, "", err
		}
		files.Files[name] = string(content)
	}
	return files, filepath.Base(dir), nil
}

// releaseCompileAxesAt keeps the drift check's existing axes.conf contract.
func releaseCompileAxesAt(repo, commit, machine string) (string, string, error) {
	files, pin, err := releaseCompileAt(repo, commit, machine)
	return pin, files.Files["axes.conf"], err
}

// releaseConfigDriftFor judges the running configuration against the
// commit's: known to git (or unchanged) -> nothing to say; unknown -> the
// per-key startup diff from the running axes.conf (probe) and the commit's
// compile on this Mac.
func releaseConfigDriftFor(repo, commit, machine, liveAxes string, in releaseInstalled, target releaseTarget) configDrift {
	d := configDrift{Live: in.Configuration, Commit: target.Configuration}
	if !rtCoreHexSHA256.MatchString(d.Live) || d.Live == d.Commit {
		return d
	}
	known, err := releasePinCommits(repo, commit, d.Live)
	if err == nil && len(known) > 0 {
		d.KnownIn = known
		return d
	}
	d.Unknown = true
	if err != nil {
		d.Note = "pin history unreadable: " + err.Error()
	}
	if strings.TrimSpace(liveAxes) == "" {
		d.Unproven = "the running compiled axes.conf (/etc/rosie-rt-core/compiled/" + d.Live + "/axes.conf) was not readable on the servo host"
		return d
	}
	pin, axes, err := releaseCompileAxes(repo, commit, machine)
	if err != nil {
		d.Unproven = fmt.Sprintf("compiling %s at %s on this Mac failed: %v", machine, cellDeployShort(commit), err)
		return d
	}
	if pin != target.Configuration {
		d.Note = fmt.Sprintf("note: this Mac compiles %s at %s to %.12s, not its pin %.12s; the startup values below come from that compile", machine, cellDeployShort(commit), pin, target.Configuration)
	}
	d.Changes = configTuningDrift(liveAxes, axes)
	return d
}

// Seams for tests: the pin history, the commit's local compile and the
// working-tree status.
var (
	releasePinCommits       = configPinCommits
	releaseCompileAxes      = releaseCompileAxesAt
	releaseCompileTarget    = releaseCompileAt
	rtCoreConfigPinCommits  = configPinCommits
	rtCoreConfigUncommitted = cellDeployUncommittedConfig
	rtCoreConfigSourceFor   = rtCoreConfigLocalSource
)
