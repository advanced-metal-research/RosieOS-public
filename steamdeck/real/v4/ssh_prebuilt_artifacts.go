package main

import (
	"archive/tar"
	"bytes"
	"compress/gzip"
	"crypto/sha256"
	"encoding/base64"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"os"
	"os/exec"
	pathpkg "path"
	"path/filepath"
	"strings"
	"time"

	robotv4build "rosieos/steamdeck/real/v4/internal/build"
)

type robotV4SSHPrebuiltArtifact struct {
	Kind         string
	Name         string
	Path         string
	SHA256       string
	SourceSHA256 string
	BuildPeer    string
	CacheHit     bool
}

func robotV4ReadSSHLocalPrebuiltArtifact(path string, kind ...string) ([]byte, bool, error) {
	binary, err := os.ReadFile(path)
	if os.IsNotExist(err) {
		return nil, false, nil
	}
	if err != nil {
		return nil, false, fmt.Errorf("read local SSH artifact cache %s: %w", path, err)
	}
	if len(kind) > 0 && kind[0] == "rt-core" {
		if err := validateRTCoreArchive(binary); err != nil {
			return nil, false, err
		}
	} else if len(binary) < 4 || !bytes.Equal(binary[:4], []byte{0x7f, 'E', 'L', 'F'}) {
		return nil, false, fmt.Errorf("local SSH artifact cache %s is not an ELF binary", path)
	}
	return binary, true, nil
}

// robotV4ResolveSSHPrebuiltArtifact builds only on the manifest-declared
// build peer and materializes a verified binary locally for the direct SSH
// installer. Runtime peers are install-only under remote-cache-required.
func robotV4ResolveSSHPrebuiltArtifact(kind string, stdout, stderr io.Writer) (robotV4SSHPrebuiltArtifact, error) {
	target, err := robotV4MeshDeployTargetForKind(kind)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	repo, err := repoRoot()
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	sourceBundleRaw, sourceHash, buildInputPaths, err := robotV4ComponentSourceBundle(repo, target)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	artifact := robotV4DeployManifestArtifact{
		ArtifactName:       target.ArtifactName,
		ArtifactKind:       target.ArtifactKind,
		TargetHost:         target.Peer,
		TargetPeer:         target.Peer,
		TargetRole:         target.Role,
		SourceTreeHash:     sourceHash,
		SourceBundleSHA256: sourceHash,
		BuildCommand:       target.BuildCommand,
	}
	buildPeer, preferred, err := robotV4SSHBuildPeerForArtifact(artifact)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	localDir := filepath.Join(repo, ".run", "robot-v4", "ssh-artifacts", strings.TrimPrefix(sourceHash, "sha256:"))
	if target.ArtifactKind == "rt-core" {
		localDir = filepath.Join(repo, "rt-core", "build", "ssh-artifacts", strings.TrimPrefix(sourceHash, "sha256:"))
	}
	localPath := filepath.Join(localDir, target.ArtifactName)
	if binary, cacheHit, err := robotV4ReadSSHLocalPrebuiltArtifact(localPath, target.ArtifactKind); err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	} else if cacheHit {
		binarySHA256 := sha256String(binary)
		fmt.Fprintf(
			stdout,
			"ssh deploy artifact local-cache hit artifact=%s build_peer=%s source_sha256=%s binary_sha256=%s path=%s\n",
			target.ArtifactName,
			robotV4BuildPeerServicePeer(buildPeer),
			sourceHash,
			binarySHA256,
			localPath,
		)
		resolved := robotV4SSHPrebuiltArtifact{
			Kind:         target.ArtifactKind,
			Name:         target.ArtifactName,
			Path:         localPath,
			SHA256:       binarySHA256,
			SourceSHA256: sourceHash,
			BuildPeer:    robotV4BuildPeerServicePeer(buildPeer),
			CacheHit:     true,
		}
		return robotV4FinalizeSSHPrebuiltArtifact(
			target,
			buildPeer,
			preferred,
			sourceBundleRaw,
			buildInputPaths,
			binary,
			resolved,
			stdout,
		)
	}
	fmt.Fprintf(
		stdout,
		"ssh deploy artifact local-cache miss artifact=%s build_peer=%s source_sha256=%s path=%s\n",
		target.ArtifactName,
		robotV4BuildPeerServicePeer(buildPeer),
		sourceHash,
		localPath,
	)
	if err := robotV4EnsureBuildPeerSysroot(buildPeer, stdout); err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	candidate := robotV4BuildPeerRequestCandidate{Inventory: buildPeer, Policy: preferred}
	buildCommand := strings.TrimSpace(robotV4BuildPeerArtifactCommand(artifact, candidate))
	if buildCommand == "" {
		return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("build peer %s has no command for %s", robotV4BuildPeerServicePeer(buildPeer), target.ArtifactName)
	}
	buildCommand = robotV4BuildPeerArtifactCommandWithPrerequisites(
		buildPeer,
		target.ArtifactName,
		buildCommand,
	)
	sourceArchive, err := robotV4SourceBundleTarGZ(sourceBundleRaw)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	recipeIdentity := strings.Join([]string{
		buildCommand,
		strings.TrimSpace(buildPeer.BuildService.Toolchain),
		strings.TrimSpace(buildPeer.TargetTriple),
		robotV4BuildPeerTargetABI(buildPeer),
	}, "\n")
	recipeHash := fmt.Sprintf("%x", sha256.Sum256([]byte(recipeIdentity)))
	identity := strings.TrimPrefix(sourceHash, "sha256:") + "-" + recipeHash[:16]
	cacheRoot := `$HOME/.rosie/build-cache/robot-v4-ssh-artifacts/` + target.ArtifactName
	script := robotv4build.SSHBuildPeerCacheScript(cacheRoot, identity, target.ArtifactName, buildCommand)
	args, err := robotV4BuildPeerRemoteCommandArgs(buildPeer, script)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	fmt.Fprintf(stdout, "ssh deploy artifact resolve artifact=%s build_peer=%s source_sha256=%s identity=%s\n", target.ArtifactName, robotV4BuildPeerServicePeer(buildPeer), sourceHash, identity)
	raw, err := runCommandWithStdinCapture("", base64.StdEncoding.EncodeToString(sourceArchive), stderr, args[0], args[1:]...)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("resolve %s on build peer %s: %w", target.ArtifactName, robotV4BuildPeerServicePeer(buildPeer), err)
	}
	binary, err := decodeWhitespaceBase64(raw)
	if err != nil {
		return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("decode %s from build peer: %w", target.ArtifactName, err)
	}
	if target.ArtifactKind == "rt-core" {
		if err := validateRTCoreArchive(binary); err != nil {
			return robotV4SSHPrebuiltArtifact{}, err
		}
	} else if len(binary) < 4 || !bytes.Equal(binary[:4], []byte{0x7f, 'E', 'L', 'F'}) {
		return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("build peer returned non-ELF artifact %s", target.ArtifactName)
	}
	observedHash := sha256String(binary)
	if err := os.MkdirAll(localDir, 0o755); err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	cacheHit := false
	if existing, readErr := os.ReadFile(localPath); readErr == nil && sha256String(existing) == observedHash {
		cacheHit = true
	} else {
		tempPath := localPath + fmt.Sprintf(".tmp-%d", time.Now().UnixNano())
		if err := os.WriteFile(tempPath, binary, 0o755); err != nil {
			return robotV4SSHPrebuiltArtifact{}, err
		}
		if err := os.Rename(tempPath, localPath); err != nil {
			_ = os.Remove(tempPath)
			return robotV4SSHPrebuiltArtifact{}, err
		}
		fmt.Fprintf(stdout, "ssh deploy artifact ready artifact=%s binary_sha256=%s target_builds=0\n", target.ArtifactName, observedHash)
	}
	resolved := robotV4SSHPrebuiltArtifact{
		Kind:         kind,
		Name:         target.ArtifactName,
		Path:         localPath,
		SHA256:       observedHash,
		SourceSHA256: sourceHash,
		BuildPeer:    robotV4BuildPeerServicePeer(buildPeer),
		CacheHit:     cacheHit,
	}
	return robotV4FinalizeSSHPrebuiltArtifact(
		target,
		buildPeer,
		preferred,
		sourceBundleRaw,
		buildInputPaths,
		binary,
		resolved,
		stdout,
	)
}

// robotV4BuildPeerArtifactCommandWithPrerequisites makes a cold manifest
// Cargo cross-build self-bootstrapping. Rustup targets are toolchain-specific,
// so a build host checkout can have the pinned compiler and GCC/sysroot ready
// while still lacking the target standard library. The check lives inside the
// immutable artifact-cache build command: cache hits do no prerequisite work,
// and concurrent cold requests remain serialized by the existing cache lock.
func robotV4BuildPeerArtifactCommandWithPrerequisites(
	buildPeer robotV4BuildPeerInventory,
	artifactName string,
	buildCommand string,
) string {
	buildCommand = strings.TrimSpace(buildCommand)
	targetTriple := strings.TrimSpace(buildPeer.TargetTriple)
	if buildCommand == "" || targetTriple == "" ||
		!strings.Contains(buildCommand, "cargo") ||
		!strings.Contains(buildCommand, "--target "+targetTriple) {
		return buildCommand
	}
	target := posixShellQuote(targetTriple)
	artifact := posixShellQuote(strings.TrimSpace(artifactName))
	prerequisite := strings.Join([]string{
		`RUSTUP_BIN="$HOME/.rosie/rust-1.95.0/cargo/bin/rustup"`,
		`if ! test -x "$RUSTUP_BIN"; then RUSTUP_BIN="$(command -v rustup || true)"; fi`,
		`test -n "$RUSTUP_BIN" && test -x "$RUSTUP_BIN" || { echo "robot-v4 build prerequisite failed artifact=` + artifact + ` rustup=missing" >&2; exit 1; }`,
		`if "$RUSTUP_BIN" target list --installed | grep -Fqx ` + target + `; then echo "robot-v4 build prerequisite ready artifact=` + artifact + ` rust_target=` + target + `" >&2; else echo "robot-v4 build prerequisite installing artifact=` + artifact + ` rust_target=` + target + `" >&2; "$RUSTUP_BIN" target add ` + target + ` >&2; fi`,
	}, "; ")
	return prerequisite + "; " + buildCommand
}

func robotV4FinalizeSSHPrebuiltArtifact(
	target robotV4MeshDeployTarget,
	buildPeer robotV4BuildPeerInventory,
	preferred robotV4BuildCachePolicyPeer,
	sourceBundleRaw []byte,
	buildInputPaths []string,
	binary []byte,
	artifact robotV4SSHPrebuiltArtifact,
	stdout io.Writer,
) (robotV4SSHPrebuiltArtifact, error) {
	if target.ArtifactKind == "rt-core" {
		node, ok := robotV4DaemonNodeInventoryRecord(robotV4RealtimeCoreNode)
		if !ok || node.RTCore == nil {
			return robotV4SSHPrebuiltArtifact{}, fmt.Errorf("rt-core package has no deployment binding")
		}
		if err := validateRTCoreArchive(binary, node.RTCore.MachineConfig); err != nil {
			return robotV4SSHPrebuiltArtifact{}, err
		}
	}
	if err := robotV4PublishSSHPrebuiltBuildCacheRecord(
		target,
		buildPeer,
		preferred,
		sourceBundleRaw,
		buildInputPaths,
		binary,
		artifact,
		stdout,
	); err != nil {
		return robotV4SSHPrebuiltArtifact{}, err
	}
	return artifact, nil
}

func robotV4PublishSSHPrebuiltBuildCacheRecord(
	target robotV4MeshDeployTarget,
	buildPeer robotV4BuildPeerInventory,
	preferred robotV4BuildCachePolicyPeer,
	sourceBundleRaw []byte,
	buildInputPaths []string,
	binary []byte,
	artifact robotV4SSHPrebuiltArtifact,
	stdout io.Writer,
) error {
	buildPeerDaemonURL := strings.TrimRight(strings.TrimSpace(buildPeer.DaemonIndexURL), "/")
	if buildPeerDaemonURL == "" {
		return fmt.Errorf("SSH build peer %s has no daemon index URL", artifact.BuildPeer)
	}
	orchestrator, ok := robotV4DaemonNodeInventoryRecord(robotV4MotionServerNode)
	if !ok || strings.TrimSpace(orchestrator.DaemonIndexURL) == "" {
		return fmt.Errorf("selected manifest motion-server has no orchestrator daemon URL")
	}
	orchestratorURL := strings.TrimRight(strings.TrimSpace(orchestrator.DaemonIndexURL), "/")
	client := &http.Client{Timeout: 5 * time.Minute}
	sourceUpload, err := postDaemonBlob(
		client,
		buildPeerDaemonURL,
		sourceBundleRaw,
		artifact.SourceSHA256,
	)
	if err != nil {
		return fmt.Errorf("publish SSH source bundle to build-peer daemon: %w", err)
	}
	binaryUpload, err := postDaemonBlob(
		client,
		buildPeerDaemonURL,
		binary,
		artifact.SHA256,
	)
	if err != nil {
		return fmt.Errorf("publish SSH binary to build-peer daemon: %w", err)
	}

	targetOS, targetArch := robotV4DeployTargetPlatform(target.Peer)
	targetTriple := robotV4DeployTargetTriple(target.Peer)
	targetABI := robotV4DeployTargetABI(target.Peer)
	if strings.TrimSpace(preferred.PlatformOS) != "" {
		targetOS = strings.ToLower(strings.TrimSpace(preferred.PlatformOS))
	}
	if strings.TrimSpace(preferred.PlatformArch) != "" {
		targetArch = robotV4DeployPlatformToken(preferred.PlatformArch)
	}
	if strings.TrimSpace(preferred.TargetTriple) != "" {
		targetTriple = strings.ToLower(strings.TrimSpace(preferred.TargetTriple))
	}
	if strings.TrimSpace(preferred.TargetABI) != "" {
		targetABI = robotV4TargetABIToken(preferred.TargetABI)
	}
	if targetABI == "" {
		targetABI = robotV4TargetABIToken(firstNonEmpty(
			buildPeer.TargetABI,
			buildPeer.BuildService.TargetABI,
		))
	}
	cache := robotV4DaemonDeployBuildCacheRecord{
		PeerNode:           artifact.BuildPeer,
		Component:          target.ArtifactName,
		TargetHost:         robotV4DeployCanonicalPeerName(target.Peer),
		BuildProfile:       "release",
		Toolchain:          firstNonEmpty(buildPeer.BuildService.Toolchain, buildPeer.BuildService.HostRuntime, "build-server"),
		PlatformOS:         targetOS,
		PlatformArch:       targetArch,
		TargetTriple:       targetTriple,
		TargetABI:          targetABI,
		SourceTreeHash:     artifact.SourceSHA256,
		SourceBundleSHA256: artifact.SourceSHA256,
		SourceBundleURI:    sourceUpload.URI,
		BinarySHA256:       artifact.SHA256,
		BinaryBlobURI:      binaryUpload.URI,
		BuildInputPaths:    append([]string(nil), buildInputPaths...),
		Status:             "ready",
		UpdatedAtUnix:      time.Now().Unix(),
	}
	published, err := robotV4PublishBuildPeerCacheRecord(
		deployOptions{DaemonURL: orchestratorURL},
		cache,
		buildPeerDaemonURL,
	)
	if err != nil {
		return fmt.Errorf("publish SSH exact-source build cache: %w", err)
	}
	fmt.Fprintf(
		stdout,
		"ssh deploy build-cache artifact=%s target=%s build_peer=%s orchestrator=%s published=%t source_sha256=%s binary_sha256=%s\n",
		target.ArtifactName,
		cache.TargetHost,
		cache.PeerNode,
		orchestratorURL,
		published,
		cache.SourceBundleSHA256,
		cache.BinarySHA256,
	)
	return nil
}

func robotV4SSHBuildPeerForArtifact(artifact robotV4DeployManifestArtifact) (robotV4BuildPeerInventory, robotV4BuildCachePolicyPeer, error) {
	policy, ok := robotV4BuildCachePolicyForArtifact(artifact)
	if !ok || !strings.EqualFold(strings.TrimSpace(policy.Mode), "remote-cache-required") || len(policy.PreferredBuildPeers) == 0 {
		return robotV4BuildPeerInventory{}, robotV4BuildCachePolicyPeer{}, fmt.Errorf("artifact %s target %s is not remote-cache-required with a preferred build peer", artifact.ArtifactName, artifact.TargetHost)
	}
	manifest, ok := loadRobotV4DaemonNodeInventoryManifest()
	if !ok {
		return robotV4BuildPeerInventory{}, robotV4BuildCachePolicyPeer{}, fmt.Errorf("selected Robot v4 manifest is unavailable")
	}
	return robotV4PickBuildPeer(policy.PreferredBuildPeers, manifest.BuildPeers, artifact, selectedRobotV4BuildServer())
}

// robotV4PickBuildPeer returns the build peer that builds artifact: the
// operator's buildServer when one is set, else the first of the manifest's
// preferred peers, in its order, that the manifest lists and that builds the
// artifact. It refuses when none qualifies, naming the operator's choice.
func robotV4PickBuildPeer(preferredPeers []robotV4BuildCachePolicyPeer, buildPeers []robotV4BuildPeerInventory, artifact robotV4DeployManifestArtifact, buildServer string) (robotV4BuildPeerInventory, robotV4BuildCachePolicyPeer, error) {
	for _, preferred := range preferredPeers {
		if buildServer != "" && !strings.EqualFold(strings.TrimSpace(preferred.Peer), buildServer) {
			continue
		}
		buildPeer, found := robotV4BuildPeerInventoryForPolicyPeer(buildPeers, preferred)
		if !found || !robotV4BuildServiceSupportsArtifact(buildPeer.BuildService, artifact) {
			continue
		}
		return buildPeer, preferred, nil
	}
	if buildServer == "" {
		return robotV4BuildPeerInventory{}, robotV4BuildCachePolicyPeer{}, fmt.Errorf("no preferred build peer in the cell manifest builds %s for %s (set %s to choose one)", artifact.ArtifactName, artifact.TargetHost, robotV4BuildServerEnv)
	}
	return robotV4BuildPeerInventory{}, robotV4BuildCachePolicyPeer{}, fmt.Errorf("configured build server %q is not a compatible preferred build peer for %s on %s", buildServer, artifact.ArtifactName, artifact.TargetHost)
}

func robotV4SourceBundleTarGZ(raw []byte) ([]byte, error) {
	var bundle robotV4SourceBundle
	if err := json.Unmarshal(raw, &bundle); err != nil {
		return nil, fmt.Errorf("parse component source bundle: %w", err)
	}
	if bundle.SchemaVersion != "robot_v4_component_source_bundle_v1" || len(bundle.Files) == 0 {
		return nil, fmt.Errorf("invalid component source bundle schema=%q files=%d", bundle.SchemaVersion, len(bundle.Files))
	}
	var out bytes.Buffer
	gz := gzip.NewWriter(&out)
	tw := tar.NewWriter(gz)
	for _, file := range bundle.Files {
		clean, err := robotV4CleanComponentBuildInputPath(file.Path)
		if err != nil {
			return nil, err
		}
		payload, err := base64.StdEncoding.DecodeString(file.PayloadBase64)
		if err != nil {
			return nil, fmt.Errorf("decode component source %s: %w", clean, err)
		}
		if sha256String(payload) != file.SHA256 {
			return nil, fmt.Errorf("component source %s hash mismatch", clean)
		}
		header := &tar.Header{Name: clean, Mode: 0o644, Size: int64(len(payload))}
		if err := tw.WriteHeader(header); err != nil {
			return nil, err
		}
		if _, err := tw.Write(payload); err != nil {
			return nil, err
		}
	}
	if err := tw.Close(); err != nil {
		return nil, err
	}
	if err := gz.Close(); err != nil {
		return nil, err
	}
	return out.Bytes(), nil
}

func syncRobotV4SSHPrebuiltFile(stdout, stderr io.Writer, sshTarget, localPath, remotePath string) error {
	file, err := os.Open(localPath)
	if err != nil {
		return err
	}
	defer file.Close()
	cmd := exec.Command("ssh", append(sshAliveArgs(sshConnectTimeout), sshTarget, robotV4PrebuiltUploadCommand(remotePath))...)
	cmd.Stdin = file
	cmd.Stdout = stdout
	cmd.Stderr = stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("transfer prebuilt %s to %s:%s: %w", localPath, sshTarget, remoteShellPath(remotePath), err)
	}
	fmt.Fprintf(stdout, "ssh deploy artifact transferred target=%s path=%s binary_sha256=%s target_builds=0\n", sshTarget, remoteShellPath(remotePath), fileSHA256OrUnknown(localPath))
	return nil
}

// robotV4PrebuiltUploadCommand writes stdin to remotePath through a temporary
// file in the same directory, so a broken transfer never leaves a partial
// artifact under the final name.
func robotV4PrebuiltUploadCommand(remotePath string) string {
	quoted := remoteShellPath(remotePath)
	return "set -eu; mkdir -p " + remoteShellPath(pathpkg.Dir(remotePath)) + "; tmp=" + quoted + ".tmp-$$; cat > \"$tmp\"; chmod 0755 \"$tmp\"; mv \"$tmp\" " + quoted
}

func fileSHA256OrUnknown(path string) string {
	raw, err := os.ReadFile(path)
	if err != nil {
		return "unknown"
	}
	return sha256String(raw)
}
