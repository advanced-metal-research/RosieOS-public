package main

// Build-peer sysroot auto-provisioning for `robot v4 deploy`.
//
// A manifest build peer may declare build_service.target_sysroot with
// sync_paths. When the deploy needs that build peer and the sysroot's
// required files are missing on it, the deploy provisions the sysroot
// automatically by running rsync ON the build peer, pulling from the
// manifest source peer. Everything (hosts, users, paths, rsync filters,
// post-sync commands) comes from the selected manifest; nothing here is
// specific to a robot cell.

import (
	"context"
	"fmt"
	"io"
	"os"
	"os/exec"
	"runtime"
	"strings"
	"time"
)

const robotV4SysrootOKSentinel = "robot-v4-sysroot-ok"
const robotV4SysrootProvisionedSentinel = "robot-v4-sysroot-provisioned"

// robotV4BuildServiceSysrootProvisionable reports whether the build peer
// declares enough manifest data for automatic sysroot provisioning.
func robotV4BuildServiceSysrootProvisionable(sysroot robotV4BuildCacheTargetSysroot) bool {
	return strings.TrimSpace(sysroot.BuildPeerRoot) != "" &&
		strings.TrimSpace(sysroot.SourcePeer) != "" &&
		len(sysroot.SyncPaths) > 0
}

// robotV4EnsureBuildPeerSysroot verifies the build peer's target sysroot and
// provisions it from the manifest source peer when required files are
// missing. It is a no-op for build peers without sync_paths.
func robotV4EnsureBuildPeerSysroot(buildPeer robotV4BuildPeerInventory, stdout io.Writer) error {
	sysroot := buildPeer.BuildService.TargetSysroot
	if !robotV4BuildServiceSysrootProvisionable(sysroot) {
		return nil
	}
	sourceRoute, err := robotV4SysrootSourceRouteForPeer(sysroot.SourcePeer)
	if err != nil {
		return err
	}
	script := robotV4SysrootEnsureScript(sysroot, sourceRoute)
	args, err := robotV4BuildPeerRemoteCommandArgs(buildPeer, script)
	if err != nil {
		return err
	}
	buildPeerName := robotV4BuildPeerServicePeer(buildPeer)
	fmt.Fprintf(stdout, "mesh deploy sysroot ensure build_peer=%s root=%s source_peer=%s source_host=%s source_route=%s\n",
		buildPeerName, sysroot.BuildPeerRoot, sysroot.SourcePeer, sourceRoute.Host,
		robotV4SysrootSourceRouteSummary(sourceRoute))
	output, err := robotV4RunLocalCommand(20*time.Minute, args)
	trimmed := strings.TrimSpace(output)
	if err != nil {
		return fmt.Errorf("sysroot ensure on build peer %s failed: %w; output: %s", buildPeerName, err, tailLines(trimmed, 6))
	}
	switch {
	case strings.Contains(trimmed, robotV4SysrootProvisionedSentinel):
		fmt.Fprintf(stdout, "mesh deploy sysroot provisioned build_peer=%s root=%s\n", buildPeerName, sysroot.BuildPeerRoot)
	case strings.Contains(trimmed, robotV4SysrootOKSentinel):
		fmt.Fprintf(stdout, "mesh deploy sysroot verified build_peer=%s root=%s\n", buildPeerName, sysroot.BuildPeerRoot)
	default:
		return fmt.Errorf("sysroot ensure on build peer %s did not report a sentinel; output: %s", buildPeerName, tailLines(trimmed, 6))
	}
	return nil
}

type robotV4SysrootSourceRoute struct {
	Host      string
	User      string
	RelayHost string
	RelayUser string
}

// robotV4SysrootSourceRouteForPeer resolves the manifest node the sysroot is
// copied from and, for an isolated child such as a RevPi, its manifest parent
// relay. The build peer reaches the child through a relay-side rsync/tar stream
// instead of assuming the child-only LAN is directly routable from the build host.
func robotV4SysrootSourceRouteForPeer(sourcePeer string) (robotV4SysrootSourceRoute, error) {
	node, ok := robotV4DaemonNodeInventoryRecordByAlias(sourcePeer)
	if !ok {
		return robotV4SysrootSourceRoute{}, fmt.Errorf("sysroot source_peer %q is not a node in the selected manifest", sourcePeer)
	}
	route := robotV4SysrootSourceRoute{
		Host: firstNonEmpty(strings.TrimSpace(node.SSHHost), strings.TrimSpace(node.Host), strings.TrimSpace(node.Peer)),
		User: strings.TrimSpace(node.RemoteUser),
	}
	if route.Host == "" {
		return robotV4SysrootSourceRoute{}, fmt.Errorf("sysroot source_peer %q has no host/ssh_host in the selected manifest", sourcePeer)
	}
	parentAlias := firstNonEmpty(strings.TrimSpace(node.ParentNode), strings.TrimSpace(node.IPv6Fallback.ParentNode))
	if parentAlias == "" {
		return route, nil
	}
	// The relay-side rsync executes in the parent network namespace, so prefer
	// ssh_host there. This preserves scoped IPv6 addresses and avoids requiring
	// child peer DNS on either the build host or relay.
	route.Host = firstNonEmpty(strings.TrimSpace(node.SSHHost), strings.TrimSpace(node.Peer), strings.TrimSpace(node.Host))
	parent, ok := robotV4DaemonNodeInventoryRecordByAlias(parentAlias)
	if !ok {
		return robotV4SysrootSourceRoute{}, fmt.Errorf("sysroot source_peer %q parent_node %q is not a node in the selected manifest", sourcePeer, parentAlias)
	}
	route.RelayHost = firstNonEmpty(strings.TrimSpace(parent.SSHHost), strings.TrimSpace(parent.Host), strings.TrimSpace(parent.Peer))
	route.RelayUser = strings.TrimSpace(parent.RemoteUser)
	if route.RelayHost == "" {
		return robotV4SysrootSourceRoute{}, fmt.Errorf("sysroot source_peer %q parent_node %q has no host/ssh_host in the selected manifest", sourcePeer, parentAlias)
	}
	return route, nil
}

func robotV4SysrootSourceRouteSummary(route robotV4SysrootSourceRoute) string {
	if strings.TrimSpace(route.RelayHost) == "" {
		return "direct"
	}
	return "via:" + robotV4SSHUserHost(route.RelayUser, route.RelayHost)
}

func robotV4SSHUserHost(user, host string) string {
	host = strings.TrimSpace(host)
	if strings.TrimSpace(user) == "" {
		return host
	}
	return strings.TrimSpace(user) + "@" + host
}

// robotV4SysrootEnsureScript builds the POSIX shell script that runs on the
// build peer: check required files, provision via rsync pulls from the
// source peer when missing, then re-check and report a sentinel.
func robotV4SysrootEnsureScript(sysroot robotV4BuildCacheTargetSysroot, route robotV4SysrootSourceRoute) string {
	root := strings.TrimRight(strings.TrimSpace(sysroot.BuildPeerRoot), "/")
	includeDir := strings.TrimSpace(sysroot.BuildPeerIncludeDir)
	libraryDir := strings.TrimSpace(sysroot.BuildPeerLibraryDir)
	required := robotV4TargetSysrootRequiredFiles(sysroot, includeDir, libraryDir)
	checks := make([]string, 0, len(required))
	for _, file := range required {
		file = strings.TrimSpace(file)
		if file == "" {
			continue
		}
		checks = append(checks, "test -r "+posixShellQuote(file))
	}
	checkExpr := strings.Join(checks, " && ")
	if checkExpr == "" {
		checkExpr = "test -d " + posixShellQuote(root)
	}
	sourceHost := strings.TrimSpace(route.Host)
	if strings.Contains(sourceHost, ":") && !strings.HasPrefix(sourceHost, "[") {
		// rsync's host:path syntax requires IPv6 literals to be bracketed so
		// the address colons are not mistaken for the remote-path separator.
		sourceHost = "[" + sourceHost + "]"
	}
	source := robotV4SSHUserHost(route.User, sourceHost)
	sshTransport := "ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new"
	relay := robotV4SSHUserHost(route.RelayUser, route.RelayHost)
	var lines []string
	lines = append(lines, "set -e")
	lines = append(lines, fmt.Sprintf("if %s; then echo %s; exit 0; fi", checkExpr, robotV4SysrootOKSentinel))
	lines = append(lines, "command -v rsync >/dev/null")
	lines = append(lines, "mkdir -p "+posixShellQuote(root))
	for _, sync := range sysroot.SyncPaths {
		sourcePath := strings.TrimSpace(sync.SourcePath)
		buildPeerPath := strings.Trim(strings.TrimSpace(sync.BuildPeerPath), "/")
		if sourcePath == "" || buildPeerPath == "" {
			continue
		}
		destination := root + "/" + buildPeerPath
		lines = append(lines, "mkdir -p "+posixShellQuote(destination))
		var filters []string
		for _, filter := range sync.Filters {
			filter = strings.TrimSpace(filter)
			if filter == "" {
				continue
			}
			filters = append(filters, posixShellQuote(filter))
		}
		if relay != "" {
			// The parent owns the child-network SSH key and name resolution.
			// Stage only this filtered path on the relay, stream it to the build
			// peer, and remove the staging directory via the relay-side trap.
			relayRsync := append([]string{"rsync", "-a"}, filters...)
			relayRsync = append(relayRsync, posixShellQuote(source+":"+sourcePath), `"$tmp/"`)
			relayScript := strings.Join([]string{
				"set -e",
				`tmp=$(mktemp -d /tmp/robot-v4-sysroot-relay.XXXXXX)`,
				`trap 'rm -rf "$tmp"' EXIT`,
				strings.Join(relayRsync, " "),
				`tar -czf - -C "$tmp" .`,
			}, "; ")
			lines = append(lines,
				sshTransport+" "+posixShellQuote(relay)+" "+posixShellQuote(relayScript)+
					" | tar -xzf - -C "+posixShellQuote(destination))
			continue
		}
		rsyncArgs := []string{"rsync", "-a", "-e", posixShellQuote(sshTransport)}
		rsyncArgs = append(rsyncArgs, filters...)
		rsyncArgs = append(rsyncArgs, posixShellQuote(source+":"+sourcePath), posixShellQuote(destination+"/"))
		lines = append(lines, strings.Join(rsyncArgs, " "))
	}
	for _, command := range sysroot.PostSyncCommands {
		command = strings.TrimSpace(command)
		if command == "" {
			continue
		}
		lines = append(lines, "(cd "+posixShellQuote(root)+" && "+command+")")
	}
	lines = append(lines, checkExpr)
	lines = append(lines, "echo "+robotV4SysrootProvisionedSentinel)
	return strings.Join(lines, "\n")
}

type robotV4LocalBuildRuntime struct {
	GOOS            string
	Hostname        string
	WSLDistribution string
	User            string
}

func robotV4CurrentLocalBuildRuntime() robotV4LocalBuildRuntime {
	hostname, _ := os.Hostname()
	return robotV4LocalBuildRuntime{
		GOOS:            runtime.GOOS,
		Hostname:        hostname,
		WSLDistribution: strings.TrimSpace(os.Getenv("WSL_DISTRO_NAME")),
		User:            firstNonEmpty(os.Getenv("USER"), os.Getenv("LOGNAME")),
	}
}

func robotV4LocalBuildRuntimeHostToken(value string) string {
	value = strings.TrimSpace(value)
	if at := strings.LastIndex(value, "@"); at >= 0 {
		value = value[at+1:]
	}
	value = strings.Trim(value, "[]")
	if colon := strings.Index(value, ":"); colon >= 0 {
		value = value[:colon]
	}
	if dot := strings.Index(value, "."); dot >= 0 {
		value = value[:dot]
	}
	return strings.ToLower(strings.TrimSpace(value))
}

func robotV4BuildPeerRunsLocally(buildPeer robotV4BuildPeerInventory, local robotV4LocalBuildRuntime) bool {
	hostRuntime := strings.ToLower(firstNonEmpty(
		strings.TrimSpace(buildPeer.BuildService.HostRuntime),
		strings.TrimSpace(buildPeer.DaemonService.Unit),
	))
	if local.GOOS != "linux" || !strings.Contains(hostRuntime, "wsl") {
		return false
	}
	distribution := strings.TrimSpace(buildPeer.WSLDistribution)
	user := strings.TrimSpace(buildPeer.WSLUser)
	if distribution == "" || user == "" ||
		!strings.EqualFold(distribution, strings.TrimSpace(local.WSLDistribution)) ||
		!strings.EqualFold(user, strings.TrimSpace(local.User)) {
		return false
	}
	hostname := robotV4LocalBuildRuntimeHostToken(local.Hostname)
	if hostname == "" {
		return false
	}
	for _, candidate := range []string{buildPeer.Peer, buildPeer.Host, buildPeer.SSHHost} {
		if robotV4LocalBuildRuntimeHostToken(candidate) == hostname {
			return true
		}
	}
	return false
}

// robotV4BuildPeerRemoteCommandArgs wraps a shell script for execution on
// the manifest build peer. A process already running inside the peer's exact
// WSL distribution and Linux identity executes directly; callers outside that
// runtime retain the manifest's SSH-to-Windows-to-WSL management route.
func robotV4BuildPeerRemoteCommandArgs(buildPeer robotV4BuildPeerInventory, script string) ([]string, error) {
	return robotV4BuildPeerCommandArgsForRuntime(buildPeer, script, robotV4CurrentLocalBuildRuntime())
}

func robotV4BuildPeerCommandArgsForRuntime(buildPeer robotV4BuildPeerInventory, script string, local robotV4LocalBuildRuntime) ([]string, error) {
	sshHost := firstNonEmpty(strings.TrimSpace(buildPeer.SSHHost), strings.TrimSpace(buildPeer.Host), strings.TrimSpace(buildPeer.Peer))
	if sshHost == "" {
		return nil, fmt.Errorf("build peer %s has no ssh_host/host in the selected manifest", robotV4BuildPeerServicePeer(buildPeer))
	}
	hostRuntime := strings.ToLower(firstNonEmpty(
		strings.TrimSpace(buildPeer.BuildService.HostRuntime),
		strings.TrimSpace(buildPeer.DaemonService.Unit),
	))
	if strings.Contains(hostRuntime, "wsl") {
		if user := strings.TrimSpace(buildPeer.SSHUser); user != "" && !robotV4ManifestUserPattern.MatchString(user) {
			return nil, fmt.Errorf("build peer %s has unsafe Windows ssh_user %q", robotV4BuildPeerServicePeer(buildPeer), user)
		}
		if distribution := strings.TrimSpace(buildPeer.WSLDistribution); distribution != "" && !robotV4ManifestTokenPattern.MatchString(distribution) {
			return nil, fmt.Errorf("build peer %s has unsafe WSL distribution %q", robotV4BuildPeerServicePeer(buildPeer), distribution)
		}
		if user := strings.TrimSpace(buildPeer.WSLUser); user != "" && !robotV4ManifestUserPattern.MatchString(user) {
			return nil, fmt.Errorf("build peer %s has unsafe WSL user %q", robotV4BuildPeerServicePeer(buildPeer), user)
		}
		if directory := strings.TrimSpace(buildPeer.WSLWorkingDirectory); directory != "" &&
			(!robotV4ManifestWorkingDirectoryValid(directory) || !robotV4ManifestWSLWorkingDirectoryPattern.MatchString(directory)) {
			return nil, fmt.Errorf("build peer %s has unsafe WSL working directory %q", robotV4BuildPeerServicePeer(buildPeer), directory)
		}
		if robotV4BuildPeerRunsLocally(buildPeer, local) {
			return []string{"bash", "-c", script}, nil
		}
		target := sshHost
		if user := strings.TrimSpace(buildPeer.SSHUser); user != "" {
			target = user + "@" + sshHost
		}
		return []string{
			"ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", target,
			robotV4WSLRemoteCommandForPeer(buildPeer, script),
		}, nil
	}
	target := sshHost
	if user := strings.TrimSpace(buildPeer.RemoteUser); user != "" {
		target = user + "@" + sshHost
	}
	return []string{"ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", target, script}, nil
}

// robotV4WSLRemoteCommandForPeer pins Windows-hosted builds to the manifest's
// WSL distribution, Linux user, and working directory. Empty fields preserve
// the legacy default-distribution behavior for older manifests.
func robotV4WSLRemoteCommandForPeer(buildPeer robotV4BuildPeerInventory, script string) string {
	escaped := strings.ReplaceAll(script, `"`, `\"`)
	escaped = strings.ReplaceAll(escaped, "\n", "; ")
	args := []string{`C:\Windows\System32\wsl.exe`}
	if distribution := strings.TrimSpace(buildPeer.WSLDistribution); distribution != "" {
		args = append(args, "--distribution", distribution)
	}
	if user := strings.TrimSpace(buildPeer.WSLUser); user != "" {
		args = append(args, "--user", user)
	}
	if directory := strings.TrimSpace(buildPeer.WSLWorkingDirectory); directory != "" {
		args = append(args, "--cd", directory)
	}
	args = append(args, "--exec", "bash", "-c", `"`+escaped+`"`)
	return strings.Join(args, " ")
}

func robotV4RunLocalCommand(timeout time.Duration, args []string) (string, error) {
	if len(args) == 0 {
		return "", fmt.Errorf("empty command")
	}
	output, err := localCommandOutputWithStderr(timeout, args[0], args[1:]...)
	return output, err
}

func localCommandOutputWithStderr(timeout time.Duration, name string, args ...string) (string, error) {
	ctx, cancel := context.WithTimeout(context.Background(), timeout)
	defer cancel()
	cmd := exec.CommandContext(ctx, name, args...)
	output, err := cmd.CombinedOutput()
	return string(output), err
}

func tailLines(text string, count int) string {
	lines := strings.Split(strings.TrimSpace(text), "\n")
	if len(lines) > count {
		lines = lines[len(lines)-count:]
	}
	return strings.Join(lines, " | ")
}
