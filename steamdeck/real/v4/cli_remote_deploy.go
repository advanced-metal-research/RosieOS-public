package main

import (
	"fmt"
	"io"
	"net"

	"os/exec"

	"strings"
)

func realtimeCoreRelaySSHAuthPreflightCommand(rtTarget string) string {
	return "ssh -o BatchMode=yes -o ConnectTimeout=5 " + shellQuote(rtTarget) + " true"
}

const robotV4RemoteSSHConnectTimeoutSeconds = 10

func robotV4RemoteSSHArgs(sshTarget, command string) []string {
	return append(sshAliveArgs(robotV4RemoteSSHConnectTimeoutSeconds), sshTarget, command)
}

func runRemoteSSH(stdout, stderr io.Writer, sshTarget, command string) error {
	if err := validateRobotV4SSHTarget("remote SSH target", sshTarget); err != nil {
		return err
	}
	fmt.Fprintf(stdout, "remote: ssh %s connect_timeout_seconds=%d\n", sshTarget, robotV4RemoteSSHConnectTimeoutSeconds)
	cmd := exec.Command("ssh", robotV4RemoteSSHArgs(sshTarget, command)...)
	cmd.Stdout = stdout
	cmd.Stderr = stderr
	return cmd.Run()
}

func validateRobotV4SSHTarget(label, target string) error {
	target = strings.TrimSpace(target)
	if target == "" || strings.HasPrefix(target, "-") {
		return fmt.Errorf("%s must contain a concrete peer/host, got %q", label, target)
	}
	host := target
	if at := strings.LastIndexByte(host, '@'); at >= 0 {
		if strings.TrimSpace(host[:at]) == "" {
			return fmt.Errorf("%s has an empty remote user in %q", label, target)
		}
		host = host[at+1:]
	}
	return validateRobotV4ConcreteSSHHost(label, host)
}

func validateRobotV4ConcreteSSHHost(label, host string) error {
	host = strings.TrimSpace(host)
	semantic := robotV4ReservedSemanticIdentityTokens
	if manifest, ok := loadRobotV4DaemonNodeInventoryManifest(); ok {
		semantic = robotV4ManifestSemanticIdentityTokens(manifest)
	}
	if host == "" || robotV4ManifestIdentityIsSemantic(semantic, host) {
		return fmt.Errorf("%s must use a concrete peer/host, not semantic role %q", label, host)
	}
	if strings.Contains(host, "://") || strings.ContainsAny(host, "@/?#=, \t\r\n") {
		return fmt.Errorf("%s must be a plain concrete peer/host without URL, userinfo, or path syntax, got %q", label, host)
	}
	if _, _, err := net.SplitHostPort(host); err == nil || strings.Count(host, ":") == 1 {
		return fmt.Errorf("%s must not include a port; pass a plain concrete peer/host, got %q", label, host)
	}
	if strings.Contains(host, ":") {
		candidate := strings.Trim(host, "[]")
		if zone := strings.LastIndex(candidate, "%"); zone >= 0 {
			candidate = candidate[:zone]
		}
		if net.ParseIP(candidate) == nil {
			return fmt.Errorf("%s contains an invalid IPv6 peer/host %q", label, host)
		}
	}
	return nil
}

func remoteTarget(user, host string) string {
	user = strings.TrimSpace(user)
	host = strings.TrimSpace(host)
	if user == "" || strings.Contains(host, "@") {
		return host
	}
	return user + "@" + host
}

func remoteShellPath(path string) string {
	path = strings.TrimSpace(path)
	if path == "" {
		return "''"
	}
	if strings.HasPrefix(path, "~/") {
		return "$HOME/" + shellQuote(path[2:])
	}
	return shellQuote(path)
}
