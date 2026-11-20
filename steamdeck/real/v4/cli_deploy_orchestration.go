package main

import (
	"encoding/json"

	"fmt"

	"net/http"

	pathpkg "path"
	"path/filepath"

	"strings"
)

func robotV4DeployCanonicalPeerName(name string) string {
	normalized := strings.ToLower(strings.TrimSpace(name))
	if node, ok := robotV4DaemonNodeInventoryRecordForAlias(normalized); ok {
		return strings.ToLower(strings.TrimSpace(firstNonEmpty(node.Peer, node.Host)))
	}
	return normalized
}

func robotV4DeployCanonicalNodeSelector(name string) string {
	normalized := strings.ToLower(strings.TrimSpace(name))
	if node, ok := robotV4DaemonNodeInventoryRecordForAlias(normalized); ok {
		return strings.ToLower(strings.TrimSpace(node.Node))
	}
	return normalized
}

func fetchRobotV4DaemonHealth(client *http.Client, daemonURL string) (robotV4DaemonHealth, error) {
	raw, err := getDaemonJSON(client, strings.TrimRight(daemonURL, "/")+"/health")
	if err != nil {
		return robotV4DaemonHealth{}, err
	}
	var health robotV4DaemonHealth
	if err := json.Unmarshal(raw, &health); err != nil {
		return robotV4DaemonHealth{}, fmt.Errorf("parse daemon /health response: %w: %s", err, raw)
	}
	if !health.OK || health.PID <= 0 {
		return robotV4DaemonHealth{}, fmt.Errorf("daemon /health returned unhealthy response: %s", raw)
	}
	return health, nil
}

func robotV4MeshDeployTargetForKind(kind string) (robotV4MeshDeployTarget, error) {
	if kind == "rt-core" {
		return robotV4RTCoreMeshDeployTarget()
	}
	if kind == "realtime-core" || kind == "realtime-core-nats-bridge" || kind == "pair-owner" {
		return robotV4MeshDeployTarget{}, &retiredRuntimeError{Selection: kind}
	}
	if target, ok, err := robotV4DaemonV1BuildPeerMeshDeployTarget(kind); ok || err != nil {
		return target, err
	}
	return robotV4MeshDeployTarget{}, fmt.Errorf("deploy target kind %q is not recognized", kind)
}

func robotV4TargetSysrootBuildCommandPrefix(sysroot robotV4BuildCacheTargetSysroot) string {
	includeDir := strings.TrimSpace(sysroot.BuildPeerIncludeDir)
	libraryDir := strings.TrimSpace(sysroot.BuildPeerLibraryDir)
	if includeDir == "" && strings.TrimSpace(sysroot.BuildPeerRoot) != "" {
		includeDir = pathpkg.Join(strings.TrimSpace(sysroot.BuildPeerRoot), "include")
	}
	if libraryDir == "" && strings.TrimSpace(sysroot.BuildPeerRoot) != "" {
		libraryDir = pathpkg.Join(strings.TrimSpace(sysroot.BuildPeerRoot), "lib")
	}
	if includeDir == "" || libraryDir == "" {
		return ""
	}
	requiredFiles := robotV4TargetSysrootRequiredFiles(sysroot, includeDir, libraryDir)
	checks := make([]string, 0, len(requiredFiles))
	for _, file := range requiredFiles {
		file = strings.TrimSpace(file)
		if file == "" {
			continue
		}
		checks = append(checks, "test -r "+posixShellQuote(file))
	}
	cflags := strings.TrimSpace(sysroot.CFlags)
	if cflags == "" {
		cflags = "-I" + includeDir
	}
	libs := strings.TrimSpace(sysroot.Libs)
	if libs == "" {
		libs = "-L" + libraryDir
		if runtimeLibraryDir := strings.TrimSpace(sysroot.RuntimeLibraryDir); runtimeLibraryDir != "" {
			libs += " -Wl,-rpath," + runtimeLibraryDir
		}
		libs += " -lethercat"
	}
	env := "ETHERCAT_CFLAGS=" + posixShellQuote(cflags) + " ETHERCAT_LIBS=" + posixShellQuote(libs)
	if len(checks) == 0 {
		return env
	}
	return strings.Join(checks, " && ") + " && " + env
}

func robotV4TargetSysrootRequiredFiles(sysroot robotV4BuildCacheTargetSysroot, includeDir, libraryDir string) []string {
	if len(sysroot.BuildPeerRequiredFiles) > 0 {
		return append([]string(nil), sysroot.BuildPeerRequiredFiles...)
	}
	files := make([]string, 0, 2)
	for _, file := range sysroot.IncludeFiles {
		if mapped := robotV4TargetSysrootBuildPeerPath(file, includeDir, "/usr/local/include/"); mapped != "" {
			files = append(files, mapped)
		}
	}
	for _, file := range sysroot.LibraryFiles {
		if mapped := robotV4TargetSysrootBuildPeerPath(file, libraryDir, "/usr/local/lib/"); mapped != "" {
			files = append(files, mapped)
		}
	}
	if len(files) == 0 {
		files = append(files, pathpkg.Join(includeDir, "ecrt.h"), pathpkg.Join(libraryDir, "libethercat.so"))
	}
	return files
}

func robotV4TargetSysrootBuildPeerPath(file, buildPeerDir, sourceDir string) string {
	file = strings.TrimSpace(file)
	buildPeerDir = strings.TrimSpace(buildPeerDir)
	if file == "" || buildPeerDir == "" {
		return ""
	}
	if strings.HasPrefix(file, sourceDir) {
		return pathpkg.Join(buildPeerDir, strings.TrimPrefix(file, sourceDir))
	}
	if strings.HasPrefix(file, "/") {
		return file
	}
	return pathpkg.Join(buildPeerDir, file)
}

func robotV4DaemonV1MeshDeployTarget(peer, role, runtimePath string) (robotV4MeshDeployTarget, error) {
	daemonEnvironment, err := robotV4DaemonV1MeshDeployEnvironment(peer, role)
	if err != nil {
		return robotV4MeshDeployTarget{}, err
	}
	runtimeConfigJSON := ""
	runtimeConfigHash := ""
	runtimeConfigPath := ""
	if manifest, ok := loadRobotV4DaemonNodeInventoryManifest(); ok {
		if node, ok := robotV4DaemonNodeInventoryRecordByAlias(firstNonEmpty(peer, role)); ok {
			config, err := compileRobotV4DaemonRuntimeConfig(manifest, node)
			if err != nil {
				return robotV4MeshDeployTarget{}, err
			}
			hash, raw, err := robotV4DaemonRuntimeConfigHash(config)
			if err != nil {
				return robotV4MeshDeployTarget{}, err
			}
			runtimeConfigJSON = string(raw)
			runtimeConfigHash = hash
			runtimeConfigPath = robotV4DaemonRuntimeConfigPathForNode(node)
			daemonEnvironment = append([]string{
				"DAEMON_V1_METADATA_JSON_FILE=" + config.MetadataFile,
				"DAEMON_V1_CONFIG_PATH=" + runtimeConfigPath,
				"DAEMON_V1_CONFIG_HASH=" + runtimeConfigHash,
			}, daemonEnvironment...)
			daemonEnvironment = robotV4MergeDaemonEnvironmentValues(daemonEnvironment)
		}
	}
	return robotV4MeshDeployTarget{
		Peer:                    peer,
		Role:                    role,
		ArtifactName:            "rosie-daemon-v1",
		ArtifactKind:            "daemon-v1",
		BuildCommand:            robotV4DaemonV1BuildCommand(peer, runtimePath, daemonEnvironment),
		DaemonEnvironment:       daemonEnvironment,
		DaemonRuntimeConfigJSON: runtimeConfigJSON,
		DaemonRuntimeConfigHash: runtimeConfigHash,
		DaemonRuntimeConfigPath: runtimeConfigPath,
		RequiredRuntimePath:     runtimePath,
		InstallStrategy:         "service-restart",
		HealthCheck:             fmt.Sprintf("daemon-v1://%s/%s/rosie-daemon-v1", peer, role),
		HealthExpectedField:     "status",
		HealthExpectedValue:     "running",
		ActiveHashReportField:   "daemon_v1_sha256",
	}, nil
}

func robotV4DaemonV1MeshDeployEnvironment(peer, role string) ([]string, error) {
	if node, ok := robotV4DaemonNodeInventoryRecordByAlias(firstNonEmpty(peer, role)); ok {
		daemonEnvironment, err := robotV4DaemonEnvironmentFromManifest(node)
		if err != nil {
			return nil, err
		}
		derivedEnvironment := []string{"DAEMON_V1_ENABLE_PEER_BUILDS=" + robotV4DaemonRuntimePeerBuildsEnvValue(node)}
		command, err := robotV4DaemonSelfRestartCommandFromManifest(node)
		if err != nil {
			return nil, err
		}
		if command != "" {
			derivedEnvironment = append(derivedEnvironment, "DAEMON_V1_SELF_RESTART_COMMAND="+command)
		}
		daemonEnvironment = append(derivedEnvironment, daemonEnvironment...)
		return robotV4MergeDaemonEnvironmentValues(daemonEnvironment), nil
	}
	if buildPeer, ok := robotV4BuildPeerInventoryRecordForAlias(firstNonEmpty(peer, role)); ok {
		daemonEnvironment, err := robotV4DaemonEnvironmentValuesFromManifestMap(
			firstNonEmpty(buildPeer.Peer, buildPeer.Host),
			"daemon_service.daemon_environment",
			buildPeer.DaemonService.DaemonEnvironment,
		)
		if err != nil {
			return nil, err
		}
		daemonEnvironment = append([]string{"DAEMON_V1_ENABLE_PEER_BUILDS=1"}, daemonEnvironment...)
		return robotV4MergeDaemonEnvironmentValues(daemonEnvironment), nil
	}
	return nil, nil
}

func robotV4DaemonRuntimePeerBuildsEnvValue(node robotV4DaemonNodeInventoryNode) string {
	if node.BuildCachePolicy.RuntimePeerBuild != nil {
		if *node.BuildCachePolicy.RuntimePeerBuild {
			return "1"
		}
		return "0"
	}
	if strings.EqualFold(strings.TrimSpace(node.BuildCachePolicy.Mode), "remote-cache-required") {
		return "0"
	}
	return "1"
}

func robotV4DaemonV1BuildCommand(_ string, _ string, _ []string) string {
	// Building must be side-effect free. The daemon apply path records the
	// content-addressed output in its persistent cache before it installs the
	// binary and schedules exactly one service restart. Restarting from here
	// can terminate the HTTP handler before that cache record is durable.
	return `cd daemon/v1 && CARGO="${CARGO:-cargo}" && if [ -x "$HOME/.rosie/rust-1.95.0/cargo/bin/cargo" ]; then export RUSTUP_HOME="$HOME/.rosie/rust-1.95.0/rustup" CARGO_HOME="$HOME/.rosie/rust-1.95.0/cargo"; CARGO="$CARGO_HOME/bin/cargo"; fi && DAEMON_CARGO_TARGET_DIR="${DAEMON_CARGO_TARGET_DIR:-$HOME/.rosie/build-cache/daemon-v1-target}" && mkdir -p "$DAEMON_CARGO_TARGET_DIR" && export CARGO_TARGET_DIR="$DAEMON_CARGO_TARGET_DIR" && CARGO_BUILD_JOBS="${CARGO_BUILD_JOBS:-1}" && "$CARGO" build --release -j "$CARGO_BUILD_JOBS" && TARGET_DIR="${CARGO_TARGET_DIR:-target}" && cp "$TARGET_DIR/release/rosie-daemon-v1" "$DAEMON_V1_BUILD_OUTPUT"`
}

func robotV4DaemonV1MetadataPath(runtimePath string) string {
	const daemonBinSuffix = "/daemon/v1/bin/rosie-daemon-v1"
	runtimePath = filepath.ToSlash(filepath.Clean(runtimePath))
	if strings.HasSuffix(runtimePath, daemonBinSuffix) {
		return strings.TrimSuffix(runtimePath, daemonBinSuffix) + "/daemon/v1/orchestrator/daemon-v1.metadata.json"
	}
	return "$HOME/.config/rosie-daemon-v1/daemon-v1.metadata.json"
}
