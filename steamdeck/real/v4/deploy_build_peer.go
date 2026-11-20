package main

import (
	"fmt"
	"strings"
)

const robotV4DaemonV1BuildPeerTargetKindPrefix = "daemon-v1-build-peer:"

func robotV4DaemonV1BuildPeerMeshDeployTarget(kind string) (robotV4MeshDeployTarget, bool, error) {
	if !strings.HasPrefix(kind, robotV4DaemonV1BuildPeerTargetKindPrefix) {
		return robotV4MeshDeployTarget{}, false, nil
	}
	peerName := strings.TrimPrefix(kind, robotV4DaemonV1BuildPeerTargetKindPrefix)
	buildPeer, ok := robotV4BuildPeerInventoryRecordForAlias(peerName)
	if !ok {
		return robotV4MeshDeployTarget{}, true, fmt.Errorf("manifest build peer %q is not configured", peerName)
	}
	runtimePath, err := robotV4BuildPeerDaemonRuntimePath(buildPeer, "rosie-daemon-v1")
	if err != nil {
		return robotV4MeshDeployTarget{}, true, err
	}
	target, err := robotV4DaemonV1MeshDeployTarget(
		robotV4BuildPeerServicePeer(buildPeer),
		robotV4BuildPeerServiceRole(buildPeer),
		runtimePath,
	)
	return target, true, err
}

func robotV4BuildPeerInventoryRecordForAlias(alias string) (robotV4BuildPeerInventory, bool) {
	manifest, ok := loadRobotV4DaemonNodeInventoryManifest()
	if !ok {
		return robotV4BuildPeerInventory{}, false
	}
	preferred := robotV4DeployCanonicalPeerName(alias)
	if preferred == "" {
		return robotV4BuildPeerInventory{}, false
	}
	var matches []robotV4BuildPeerInventory
	for _, buildPeer := range manifest.BuildPeers {
		if robotV4BuildPeerNameMatches(buildPeer, preferred) {
			matches = append(matches, buildPeer)
		}
	}
	if len(matches) == 0 {
		return robotV4BuildPeerInventory{}, false
	}
	daemonArtifact := robotV4DeployManifestArtifact{ArtifactName: "rosie-daemon-v1", ArtifactKind: "daemon-v1"}
	for _, buildPeer := range matches {
		if robotV4BuildServiceSupportsArtifact(buildPeer.BuildService, daemonArtifact) &&
			(strings.TrimSpace(buildPeer.DaemonService.Unit) != "" ||
				len(buildPeer.DaemonService.DaemonEnvironment) > 0 ||
				len(buildPeer.DaemonStartupArgs) > 0 ||
				len(buildPeer.RuntimePaths) > 0 ||
				strings.TrimSpace(buildPeer.DaemonRuntimePath) != "") {
			return buildPeer, true
		}
	}
	for _, buildPeer := range matches {
		if robotV4BuildServiceSupportsArtifact(buildPeer.BuildService, daemonArtifact) {
			return buildPeer, true
		}
	}
	return matches[0], true
}

func robotV4BuildPeerServicePeer(buildPeer robotV4BuildPeerInventory) string {
	return firstNonEmpty(
		robotV4DeployCanonicalPeerName(buildPeer.Peer),
		robotV4DeployCanonicalPeerName(buildPeer.Host),
	)
}

func robotV4BuildPeerServiceRole(buildPeer robotV4BuildPeerInventory) string {
	return firstNonEmpty(
		strings.TrimSpace(buildPeer.Role),
		strings.TrimSpace(buildPeer.BuildService.Role),
		"build-server",
	)
}

func robotV4BuildPeerDaemonRuntimePath(buildPeer robotV4BuildPeerInventory, artifactName string) (string, error) {
	artifactName = robotV4BootstrapPathSegment(artifactName, "rosie-daemon-v1")
	if strings.TrimSpace(buildPeer.DaemonRuntimePath) != "" {
		return strings.TrimSpace(buildPeer.DaemonRuntimePath), nil
	}
	for _, runtimePath := range buildPeer.RuntimePaths {
		runtimePath = strings.TrimSpace(runtimePath)
		if runtimePath == "" {
			continue
		}
		if strings.HasSuffix(strings.TrimRight(runtimePath, "/\\"), "/"+artifactName) ||
			strings.HasSuffix(strings.TrimRight(runtimePath, "/\\"), "\\"+artifactName) ||
			strings.HasSuffix(strings.TrimRight(runtimePath, "/\\"), "/"+artifactName+".exe") ||
			strings.HasSuffix(strings.TrimRight(runtimePath, "/\\"), "\\"+artifactName+".exe") {
			return runtimePath, nil
		}
	}
	if rosieHome := strings.TrimRight(strings.TrimSpace(buildPeer.RosieHome), "/\\"); rosieHome != "" {
		separator := "/"
		if strings.Contains(rosieHome, "\\") {
			separator = "\\"
		}
		return rosieHome + separator + "bin" + separator + artifactName, nil
	}
	return "", fmt.Errorf("manifest build peer %s missing daemon_runtime_path for %s", robotV4BuildPeerServicePeer(buildPeer), artifactName)
}

func robotV4BuildPeerCommandForCandidate(buildCommand string, candidate robotV4BuildPeerRequestCandidate) string {
	buildCommand = strings.TrimSpace(buildCommand)
	if buildCommand == "" {
		return buildCommand
	}
	if sysrootPrefix := robotV4TargetSysrootBuildCommandPrefix(candidate.Inventory.BuildService.TargetSysroot); sysrootPrefix != "" {
		buildCommand = sysrootPrefix + " " + buildCommand
	}
	prefix := robotV4BuildPeerToolEnvPrefix(candidate.Inventory.BuildService)
	if prefix == "" {
		return buildCommand
	}
	return prefix + " && " + buildCommand
}

// robotV4BuildPeerArtifactCommand resolves the build command a build peer
// should run for an artifact: a manifest artifact_build_commands override
// (used verbatim, keyed by artifact name or kind) or the artifact's own
// build command wrapped with the peer's sysroot/toolchain env.
func robotV4BuildPeerArtifactCommand(artifact robotV4DeployManifestArtifact, candidate robotV4BuildPeerRequestCandidate) string {
	overrides := candidate.Inventory.BuildService.ArtifactBuildCommands
	for _, key := range []string{artifact.ArtifactName, artifact.ArtifactKind} {
		if command := strings.TrimSpace(overrides[strings.TrimSpace(key)]); command != "" {
			return command
		}
	}
	return robotV4BuildPeerCommandForCandidate(artifact.BuildCommand, candidate)
}

func robotV4BuildPeerToolEnvPrefix(service robotV4BuildServiceInventory) string {
	var parts []string
	var exports []string
	if cxx := strings.TrimSpace(service.CXX); cxx != "" {
		exports = append(exports, "CXX="+posixShellQuote(cxx))
	}
	if cc := strings.TrimSpace(service.CC); cc != "" {
		exports = append(exports, "CC="+posixShellQuote(cc))
	}
	if len(exports) > 0 {
		parts = append(parts, "export "+strings.Join(exports, " "))
	}
	for _, tool := range service.RequiredTools {
		tool = strings.TrimSpace(tool)
		if tool == "" {
			continue
		}
		parts = append(parts, "command -v "+posixShellQuote(tool)+">/dev/null")
	}
	if len(parts) == 0 {
		return ""
	}
	return strings.Join(parts, " && ")
}
