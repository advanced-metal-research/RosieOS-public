package main

import (
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path"
	"path/filepath"
	"sort"
	"strings"
)

// What each deployed artifact is built from, read from the build definitions
// instead of a hand-kept prefix list, so the release tier follows the build:
//
//   - the servo host's package (rt-core/tools/package-runtime.sh): the
//     native core (Makefile `live`: src, include, engine and the generated
//     C++ headers under clients/cpp/include), the Go binaries it installs
//     (rt-control, rtctl, rt-package, rt-natspublisher) with every package
//     they import (`go list -deps`, linux), the host scripts and units it
//     copies, verify-package.sh, the IgH userlib build, config/ and the
//     robot descriptions;
//   - the relay's OLP server: the Go packages it imports from rt-core.
//
// The old rule treated every rt-core source as the package's, so a change to
// an off-cell tool (tools/tuningreport, tools/cidriver) chose the full tier.
// `go list` reads this checkout, so the import graph is used only when the
// checkout's rt-core, robot_description and offline-programming/v1 equal the
// commit; otherwise the conservative prefix rule stands.

// releaseShippedMains are the Go binaries package-runtime.sh installs.
var releaseShippedMains = []string{"./cmd/rt-control", "./tools/rtctl/cmd/rtctl", "./tools/packageinfo", "./tools/natspublisher"}

// releaseServoFiles are the package's non-Go inputs (prefixes end in "/").
var releaseServoFiles = []string{"rt-core/src/", "rt-core/include/", "rt-core/engine/", "rt-core/clients/cpp/include/", "rt-core/protocol/",
	"rt-core/Makefile", "rt-core/go.mod", "rt-core/go.sum", "rt-core/config/",
	"rt-core/host/install.sh", "rt-core/host/generate-control-env.sh", "rt-core/host/remote-pki.sh", "rt-core/host/ethercat-foundation.sh",
	"rt-core/host/run-core", "rt-core/host/rosie-rt-core.service", "rt-core/host/rosie-rt-control.service", "rt-core/host/rosie-rt-natspublisher.service",
	"rt-core/host/rosie-rt-crl.service", "rt-core/host/rosie-rt-crl.timer",
	"rt-core/tools/package-runtime.sh", "rt-core/tools/verify-package.sh", "rt-core/tools/build-igh-userlib.sh",
	"robot_description/robots/"}

// releaseShipped is the import graph of the shipped Go binaries, as
// repository-relative package directories.
type releaseShipped struct {
	ServoGo map[string]bool
	OLPGo   map[string]bool
}

func (s *releaseShipped) servo(p string) bool {
	for _, f := range releaseServoFiles {
		if strings.HasSuffix(f, "/") && strings.HasPrefix(p, f) || p == f {
			return true
		}
	}
	return strings.HasSuffix(p, ".go") && s.ServoGo[path.Dir(p)]
}

func (s *releaseShipped) olp(p string) bool {
	return strings.HasSuffix(p, ".go") && s.OLPGo[path.Dir(p)]
}

// releaseGoDeps lists the repository package directories the mains in dir
// import, for linux (the cells' OS).
var releaseGoDeps = func(repo, dir string, mains []string) (map[string]bool, error) {
	cmd := exec.Command("go", append([]string{"list", "-deps", "-f", "{{if not .Standard}}{{.Dir}}{{end}}"}, mains...)...)
	cmd.Dir = filepath.Join(repo, dir)
	cmd.Env = append(os.Environ(), "GOOS=linux", "GOFLAGS=-mod=mod", "GOWORK=off")
	out, err := cmd.Output()
	if err != nil {
		return nil, fmt.Errorf("go list in %s: %v", dir, err)
	}
	root, _ := filepath.EvalSymlinks(repo)
	deps := map[string]bool{}
	for _, line := range strings.Split(strings.TrimSpace(string(out)), "\n") {
		line, _ = filepath.EvalSymlinks(strings.TrimSpace(line))
		if rel, err := filepath.Rel(root, line); err == nil && !strings.HasPrefix(rel, "..") {
			deps[filepath.ToSlash(rel)] = true
		}
	}
	if len(deps) == 0 {
		return nil, fmt.Errorf("go list in %s found no repository packages", dir)
	}
	return deps, nil
}

// releaseShippedAt reads the import graph when this checkout's trees equal
// the commit's; nil (the conservative prefix rule) otherwise, with why.
func releaseShippedAt(repo, commit string) (*releaseShipped, string) {
	diff, err := robotV4GitOutput(repo, "diff", "--name-only", commit, "--", "rt-core", "robot_description", "offline-programming/v1")
	if err != nil || strings.TrimSpace(diff) != "" {
		return nil, fmt.Sprintf("this checkout's rt-core, robot_description or offline-programming/v1 differ from %s: every rt-core source counts as the servo package's", cellDeployShort(commit))
	}
	servo, err := releaseGoDeps(repo, "rt-core", releaseShippedMains)
	if err != nil {
		return nil, err.Error() + ": every rt-core source counts as the servo package's"
	}
	olp, err := releaseGoDeps(repo, "offline-programming/v1", []string{"."})
	if err != nil {
		return nil, err.Error() + ": every rt-core source counts as the servo package's"
	}
	return &releaseShipped{ServoGo: servo, OLPGo: olp}, ""
}

// releaseDeploymentSurface is quality/deployment-surface.mjs over the range
// (cell manifests' declared runtime paths, units and build commands; command
// and service entry points): advisory lines for the plan. Empty without node.
var releaseDeploymentSurface = func(repo, from, to string) []string {
	if _, err := exec.LookPath("node"); err != nil || !cellDeployHexCommit.MatchString(from) {
		return nil
	}
	cmd := exec.Command("node", "--input-type=module", "-e",
		`const m = await import(process.argv[1]); console.log(JSON.stringify(m.report(process.argv[2], process.argv[3])));`,
		filepath.Join(repo, "quality", "deployment-surface.mjs"), from, to)
	cmd.Dir = repo
	out, err := cmd.Output()
	if err != nil {
		return nil
	}
	var r struct {
		Changes []struct {
			Path    string   `json:"path"`
			Added   []string `json:"added"`
			Removed []string `json:"removed"`
		} `json:"changes"`
		Entrypoints struct {
			Added   []string `json:"added"`
			Removed []string `json:"removed"`
		} `json:"entrypoints"`
	}
	if json.Unmarshal(out, &r) != nil {
		return nil
	}
	var lines []string
	for _, c := range r.Changes {
		lines = append(lines, fmt.Sprintf("%s: %d declared deployment surface entr(y/ies) added, %d removed", c.Path, len(c.Added), len(c.Removed)))
	}
	for _, e := range r.Entrypoints.Added {
		lines = append(lines, "new command/service file "+e)
	}
	for _, e := range r.Entrypoints.Removed {
		lines = append(lines, "removed command/service file "+e)
	}
	sort.Strings(lines)
	return lines
}
