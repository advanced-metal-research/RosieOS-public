package main

import (
	"archive/tar"
	"bytes"
	"compress/gzip"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path"
	"path/filepath"
	"regexp"
	"strings"
)

func robotV4RTCoreMeshDeployTarget() (robotV4MeshDeployTarget, error) {
	node, ok := robotV4DaemonNodeInventoryRecord(robotV4RealtimeCoreNode)
	if !ok || !node.selectsRTCore() || node.RTCore == nil {
		return robotV4MeshDeployTarget{}, fmt.Errorf("rt-core is not selected")
	}
	return robotV4MeshDeployTarget{
		Peer: node.Peer, Role: node.Role, ArtifactName: node.RTCore.PackageArtifact, ArtifactKind: "rt-core",
		BuildInputPaths:     []string{"steamdeck/real/v4/tools/build-rt-core-package.sh", node.RTCore.MachineConfig},
		BuildCommand:        `bash steamdeck/real/v4/tools/build-rt-core-package.sh`,
		RequiredRuntimePath: path.Join(node.RosieHome, "rt-core", "packages", "rosie-rt-core.tar.gz"),
		InstallStrategy:     "rt-core-package-install-only",
		HealthCheck:         "rt-core-deployment", HealthExpectedField: "posture", HealthExpectedValue: "installed-disarmed",
		ActiveHashReportField: "rt_core_package_sha256",
	}, nil
}

// robotV4RTCoreSourceInputs includes code and packaging inputs only, preventing
// generated binaries or unrelated repository material from entering the build.
func robotV4RTCoreSourceInputs(repo string) ([]string, error) {
	inputs := []string{"rt-core/Makefile", "rt-core/go.mod", "rt-core/go.sum",
		// Unverified provisioning: copy these notices from the pinned IgH source
		// tree before bundling; no other dependency build output is shipped.
		"rt-core/build/deps/ethercat-1.6.9/COPYING", "rt-core/build/deps/ethercat-1.6.9/COPYING.LESSER"}
	for _, directory := range []string{"src", "include", "engine", "cmd", "ipcclient", "adapters", "internal", "sdk", "protocol", "tools", "host", "config/drives", "config"} {
		err := filepath.WalkDir(filepath.Join(repo, "rt-core", directory), func(p string, d os.DirEntry, err error) error {
			if err != nil {
				return err
			}
			if d.IsDir() {
				if d.Name() == "__pycache__" || d.Name() == "build" {
					return filepath.SkipDir
				}
				return nil
			}
			if d.Type()&os.ModeSymlink != 0 {
				return fmt.Errorf("rt-core source link refused: %s", p)
			}
			switch filepath.Ext(p) {
			case ".go", ".cpp", ".hpp", ".h", ".sh", ".json", ".service":
			default:
				if d.Name() != "run-core" {
					return nil
				}
			}
			rel, err := filepath.Rel(repo, p)
			if err != nil {
				return err
			}
			inputs = append(inputs, filepath.ToSlash(rel))
			return nil
		})
		if err != nil {
			return nil, err
		}
	}
	return robotV4UniqueSourceInputPaths(inputs), nil
}

// validateRTCoreArchive rejects path escapes, links and ambiguous identities
// before the authenticated tarball is sent to the servo host for verification.
func validateRTCoreArchive(raw []byte, machineConfig ...string) error {
	_, err := rtCoreArchiveGitSHA(raw, machineConfig...)
	return err
}

// rtCoreArchiveGitSHA validates the archive like validateRTCoreArchive and
// returns the source commit recorded in its rt_package.json.
func rtCoreArchiveGitSHA(raw []byte, machineConfig ...string) (string, error) {
	gz, err := gzip.NewReader(bytes.NewReader(raw))
	if err != nil {
		return "", fmt.Errorf("rt-core package is not gzip: %w", err)
	}
	defer gz.Close()
	tr := tar.NewReader(gz)
	seen := map[string]bool{}
	regular := map[string]bool{}
	var manifest struct {
		Backend string `json:"backend"`
		Inputs  struct {
			GitSHA string `json:"git_sha"`
		} `json:"build_inputs"`
	}
	for {
		h, err := tr.Next()
		if err == io.EOF {
			break
		}
		if err != nil {
			return "", err
		}
		name := strings.TrimPrefix(h.Name, "./")
		if h.Typeflag == tar.TypeDir {
			name = strings.TrimSuffix(name, "/")
			if name == "" || name == "." {
				continue
			}
		} else if h.Typeflag != tar.TypeReg {
			return "", fmt.Errorf("rt-core archive contains nonregular entry %q", h.Name)
		}
		if path.Clean(name) != name || strings.HasPrefix(name, "/") || name == ".." || strings.HasPrefix(name, "../") || strings.Contains(name, "\\") || seen[name] {
			return "", fmt.Errorf("rt-core archive contains unsafe or duplicate path %q", h.Name)
		}
		seen[name] = true
		regular[name] = h.Typeflag == tar.TypeReg
		if name == "rt_package.json" {
			if err := json.NewDecoder(tr).Decode(&manifest); err != nil {
				return "", err
			}
		}
	}
	if _, err := io.Copy(io.Discard, gz); err != nil {
		return "", fmt.Errorf("rt-core gzip integrity: %w", err)
	}
	if manifest.Backend != "live" || !regexp.MustCompile(`^[0-9a-f]{40}$`).MatchString(manifest.Inputs.GitSHA) {
		return "", fmt.Errorf("rt-core archive requires a live package with a full git identity")
	}
	for _, required := range []string{"rt_package.json", "bin/rosie-rt-core", "bin/rt-control", "bin/rtctl", "bin/rt-package", "bin/rt-natspublisher", "host/install.sh", "host/generate-control-env.sh"} {
		if !regular[required] {
			return "", fmt.Errorf("rt-core package missing %s", required)
		}
	}
	for _, config := range machineConfig {
		if !regular[strings.TrimPrefix(config, "rt-core/")] {
			return "", fmt.Errorf("rt-core package missing selected machine %s", config)
		}
	}
	return manifest.Inputs.GitSHA, nil
}

// robotV4LocalRTCorePackage admits an operator-supplied live package, for a
// cell whose manifest build peer cannot be reached, only when it was built from
// this checkout's exact clean HEAD; any other source would deploy unreviewed code
// or configuration under the current cell pins.
func robotV4LocalRTCorePackage(packagePath string, c robotV4RTCoreConfig, expectedSHA256 ...string) (robotV4SSHPrebuiltArtifact, error) {
	if len(expectedSHA256) > 1 {
		return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("rt-core package accepts at most one expected sha256")
	}
	raw, err := os.ReadFile(packagePath)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	gitSHA, err := rtCoreArchiveGitSHA(raw, c.MachineConfig)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	repo, err := repoRoot()
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	head, err := robotV4GitOutput(repo, "rev-parse", "HEAD")
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	if strings.TrimSpace(head) != gitSHA {
		return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("rt-core package was built from %s, not this checkout's HEAD %s", gitSHA, strings.TrimSpace(head))
	}
	dirty, err := robotV4GitOutput(repo, "status", "--porcelain", "--", "rt-core", "robot_description")
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	if strings.TrimSpace(dirty) != "" {
		return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("rt-core package refused: rt-core or robot_description has uncommitted changes")
	}
	abs, err := filepath.Abs(packagePath)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	artifact := robotV4SSHPrebuiltArtifact{Kind: "rt-core", Name: c.PackageArtifact, Path: abs, SHA256: sha256String(raw), SourceSHA256: "git:" + gitSHA}
	if len(expectedSHA256) == 1 && expectedSHA256[0] != "" && artifact.SHA256 != expectedSHA256[0] {
		return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("rt-core package sha256 is %s, want registered candidate %s", artifact.SHA256, expectedSHA256[0])
	}
	return artifact, nil
}
