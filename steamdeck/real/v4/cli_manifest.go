package main

import (
	"encoding/json"

	"fmt"

	"net"
	"net/url"
	"os"
	"os/exec"

	pathpkg "path"

	"regexp"

	"strings"
)

const robotV4DeploymentManifestSchemaV1 = "robot_v4_deployment_manifest_v1"

type robotV4ManifestDocument struct {
	SchemaVersion string                               `json:"schema_version"`
	Name          string                               `json:"name"`
	Connections   map[string]robotV4ManifestConnection `json:"connections"`
	MotionPolicy  robotV4ManifestMotionPolicy          `json:"motion_policy"`
	Telemetry     robotV4ManifestTelemetry             `json:"telemetry"`
}

type robotV4ManifestConnection struct {
	Peer             string                    `json:"-"`
	Transport        string                    `json:"transport"`
	Network          string                    `json:"network"`
	Host             string                    `json:"host"`
	SSHUser          string                    `json:"ssh_user"`
	WorkingDirectory string                    `json:"working_directory,omitempty"`
	ViaPeer          string                    `json:"via_peer,omitempty"`
	WSL              robotV4ManifestWSLRuntime `json:"wsl,omitempty"`
}

type robotV4ManifestWSLRuntime struct {
	Distribution     string `json:"distribution"`
	User             string `json:"user"`
	WorkingDirectory string `json:"working_directory"`
}

type robotV4ManifestMotionPolicy struct {
	AutomationMode        string `json:"automation_mode"`
	Fixture               string `json:"fixture"`
	HumanAuthorization    string `json:"human_authorization"`
	AutomatedFinalPosture string `json:"automated_final_posture"`
	LiveTransitionPeer    string `json:"live_transition_peer"`
}

type robotV4ManifestTelemetry struct {
	Network             string `json:"network"`
	LegionLeafAuthority string `json:"legion_leaf_authority"`
}

func robotV4GitOutput(root string, args ...string) (string, error) {
	commandArgs := append([]string{"-C", root}, args...)
	output, err := exec.Command("git", commandArgs...).CombinedOutput()
	if err != nil {
		return "", fmt.Errorf("git %s: %w: %s", strings.Join(commandArgs, " "), err, strings.TrimSpace(string(output)))
	}
	return strings.TrimSpace(string(output)), nil
}

func robotV4LoadAndValidateManifestDocument(path string) (robotV4ManifestDocument, robotV4DaemonNodeInventoryManifest, string, error) {
	path = resolveRobotV4ManifestPath(strings.TrimSpace(path))
	if path == "" {
		return robotV4ManifestDocument{}, robotV4DaemonNodeInventoryManifest{}, "", fmt.Errorf("manifest command requires --manifest")
	}
	document, err := robotV4ReadManifestDocument(path)
	if err != nil {
		return robotV4ManifestDocument{}, robotV4DaemonNodeInventoryManifest{}, "", err
	}
	restore := withRobotV4ManifestPath(path)
	inventory, ok := loadRobotV4DaemonNodeInventoryManifest()
	restore()
	if !ok {
		return robotV4ManifestDocument{}, robotV4DaemonNodeInventoryManifest{}, "", fmt.Errorf("manifest legacy deployment compatibility validation failed: %s", path)
	}
	if err := robotV4ValidateManifestDocument(document, inventory); err != nil {
		return robotV4ManifestDocument{}, robotV4DaemonNodeInventoryManifest{}, "", fmt.Errorf("manifest %s: %w", path, err)
	}
	return document, inventory, path, nil
}

func robotV4ReadManifestDocument(path string) (robotV4ManifestDocument, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return robotV4ManifestDocument{}, fmt.Errorf("read manifest %s: %w", path, err)
	}
	var root map[string]json.RawMessage
	if err := json.Unmarshal(raw, &root); err != nil {
		return robotV4ManifestDocument{}, fmt.Errorf("decode manifest %s: %w", path, err)
	}
	required := []string{"schema_version", "name", "connections", "motion_policy"}
	for _, field := range required {
		if len(root[field]) == 0 {
			return robotV4ManifestDocument{}, fmt.Errorf("manifest %s is missing %s", path, field)
		}
	}
	var document robotV4ManifestDocument
	if err := json.Unmarshal(root["schema_version"], &document.SchemaVersion); err != nil {
		return robotV4ManifestDocument{}, fmt.Errorf("decode manifest schema_version: %w", err)
	}
	if err := json.Unmarshal(root["name"], &document.Name); err != nil {
		return robotV4ManifestDocument{}, fmt.Errorf("decode manifest name: %w", err)
	}
	for rawField, destination := range map[string]any{
		"connections":   &document.Connections,
		"motion_policy": &document.MotionPolicy,
		"telemetry":     &document.Telemetry,
	} {
		if rawField == "telemetry" && len(root[rawField]) == 0 {
			continue
		}
		decoder := json.NewDecoder(strings.NewReader(string(root[rawField])))
		decoder.DisallowUnknownFields()
		if err := decoder.Decode(destination); err != nil {
			return robotV4ManifestDocument{}, fmt.Errorf("decode manifest %s: %w", rawField, err)
		}
	}
	return document, nil
}

func robotV4ValidateManifestDocument(document robotV4ManifestDocument, inventory robotV4DaemonNodeInventoryManifest) error {
	document.SchemaVersion = strings.TrimSpace(document.SchemaVersion)
	document.Name = strings.TrimSpace(document.Name)
	if document.SchemaVersion != robotV4DeploymentManifestSchemaV1 {
		return fmt.Errorf("schema_version must be %q", robotV4DeploymentManifestSchemaV1)
	}
	if document.Name == "" || document.Name != strings.TrimSpace(inventory.Name) {
		return fmt.Errorf("companion name %q does not match deployment cell %q", document.Name, inventory.Name)
	}
	if len(document.Connections) == 0 {
		return fmt.Errorf("connections must not be empty")
	}
	byPeer := map[string]robotV4ManifestConnection{}
	for peer, connection := range document.Connections {
		connection.Peer = strings.TrimSpace(peer)
		connection = robotV4NormalizeManifestConnection(connection)
		if err := robotV4ValidateManifestConnection(connection); err != nil {
			return fmt.Errorf("connections[%q]: %w", peer, err)
		}
		peerKey := strings.ToLower(connection.Peer)
		if _, exists := byPeer[peerKey]; exists {
			return fmt.Errorf("duplicate connection peer %q", connection.Peer)
		}
		byPeer[peerKey] = connection
	}
	for _, connection := range byPeer {
		if connection.Transport == "ssh-via" {
			if _, ok := byPeer[strings.ToLower(connection.ViaPeer)]; !ok {
				return fmt.Errorf("connection %q references unknown via_peer %q", connection.Peer, connection.ViaPeer)
			}
		}
	}
	for peer := range byPeer {
		seen := map[string]bool{}
		for current := peer; current != ""; {
			if seen[current] {
				return fmt.Errorf("connection relay cycle includes peer %q", current)
			}
			seen[current] = true
			connection := byPeer[current]
			current = strings.ToLower(strings.TrimSpace(connection.ViaPeer))
		}
	}
	for _, node := range inventory.Nodes {
		connection, ok := byPeer[strings.ToLower(strings.TrimSpace(node.Peer))]
		if !ok {
			return fmt.Errorf("node peer %q has no manifest connection", node.Peer)
		}
		if !strings.EqualFold(connection.SSHUser, strings.TrimSpace(node.RemoteUser)) {
			return fmt.Errorf("connection peer %q ssh_user %q disagrees with node remote_user %q", node.Peer, connection.SSHUser, node.RemoteUser)
		}
		if !robotV4ManifestConnectionMatchesNodeHost(connection.Host, node) {
			return fmt.Errorf("connection peer %q host %q is absent from the node's concrete host/alias inventory", node.Peer, connection.Host)
		}
	}
	for _, buildPeer := range inventory.BuildPeers {
		connection, ok := byPeer[strings.ToLower(strings.TrimSpace(buildPeer.Peer))]
		if !ok {
			return fmt.Errorf("build peer %q has no manifest connection", buildPeer.Peer)
		}
		buildHost := firstNonEmpty(strings.TrimSpace(buildPeer.SSHHost), strings.TrimSpace(buildPeer.Host), strings.TrimSpace(buildPeer.Peer))
		if !strings.EqualFold(connection.Host, buildHost) || connection.SSHUser != strings.TrimSpace(buildPeer.SSHUser) {
			return fmt.Errorf("connection peer %q route %s@%s disagrees with build peer route %s@%s", buildPeer.Peer, connection.SSHUser, connection.Host, buildPeer.SSHUser, buildHost)
		}
		if connection.Transport == "ssh-windows-wsl" &&
			(connection.WSL.Distribution != strings.TrimSpace(buildPeer.WSLDistribution) ||
				connection.WSL.User != strings.TrimSpace(buildPeer.WSLUser) ||
				connection.WSL.WorkingDirectory != strings.TrimSpace(buildPeer.WSLWorkingDirectory)) {
			return fmt.Errorf("connection peer %q WSL runtime disagrees with the build peer runtime", buildPeer.Peer)
		}
	}
	if err := robotV4ValidateManifestMotionPolicy(document.MotionPolicy, byPeer); err != nil {
		return err
	}
	if document.Telemetry == (robotV4ManifestTelemetry{}) && robotV4ManifestOnlyRTCore(inventory) {
		return nil
	}
	return robotV4ValidateManifestTelemetry(document.Telemetry)
}

// robotV4ManifestOnlyRTCore permits absent legacy telemetry only when every
// node explicitly uses rt-core, preventing a mixed deployment from losing NATS.
func robotV4ManifestOnlyRTCore(inventory robotV4DaemonNodeInventoryManifest) bool {
	if len(inventory.Nodes) == 0 {
		return false
	}
	for _, node := range inventory.Nodes {
		if node.Runtime != "rt-core" || node.RTCore == nil || node.RTCore.NATS != (robotV4RTCoreNATS{}) {
			return false
		}
	}
	return true
}

func robotV4ManifestConnectionMatchesNodeHost(host string, node robotV4DaemonNodeInventoryNode) bool {
	for _, candidate := range append([]string{node.Peer, node.Host, node.SSHHost}, node.Aliases...) {
		if strings.EqualFold(strings.TrimSpace(host), strings.TrimSpace(candidate)) {
			return true
		}
	}
	return false
}

func robotV4NormalizeManifestConnection(connection robotV4ManifestConnection) robotV4ManifestConnection {
	connection.Peer = strings.TrimSpace(connection.Peer)
	connection.Transport = strings.ToLower(strings.TrimSpace(connection.Transport))
	connection.Network = strings.ToLower(strings.TrimSpace(connection.Network))
	connection.Host = strings.TrimSpace(connection.Host)
	connection.SSHUser = strings.TrimSpace(connection.SSHUser)
	connection.WorkingDirectory = strings.TrimSpace(connection.WorkingDirectory)
	connection.ViaPeer = strings.TrimSpace(connection.ViaPeer)
	connection.WSL.Distribution = strings.TrimSpace(connection.WSL.Distribution)
	connection.WSL.User = strings.TrimSpace(connection.WSL.User)
	connection.WSL.WorkingDirectory = strings.TrimSpace(connection.WSL.WorkingDirectory)
	return connection
}

var robotV4ManifestTokenPattern = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9._-]*$`)
var robotV4ManifestUserPattern = regexp.MustCompile(`^[A-Za-z_][A-Za-z0-9._-]*$`)
var robotV4ManifestHostnamePattern = regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9.-]*$`)
var robotV4ManifestWSLWorkingDirectoryPattern = regexp.MustCompile(`^/[A-Za-z0-9._/-]+$`)

func robotV4ValidateManifestConnection(connection robotV4ManifestConnection) error {
	if !robotV4ManifestTokenPattern.MatchString(connection.Peer) {
		return fmt.Errorf("peer %q is not a concrete token", connection.Peer)
	}
	if !robotV4ManifestUserPattern.MatchString(connection.SSHUser) {
		return fmt.Errorf("ssh_user %q is invalid", connection.SSHUser)
	}
	if !robotV4ManifestHostValid(connection.Host) {
		return fmt.Errorf("host %q is invalid or transport-decorated", connection.Host)
	}
	switch connection.Transport {
	case "ssh":
		if !robotV4ManifestWorkingDirectoryValid(connection.WorkingDirectory) {
			return fmt.Errorf("working_directory %q must be a safe absolute POSIX path", connection.WorkingDirectory)
		}
		if connection.Network != "tailnet-v1" && connection.Network != "lan" {
			return fmt.Errorf("ssh network must be lan or tailnet-v1")
		}
		if connection.ViaPeer != "" || connection.WSL != (robotV4ManifestWSLRuntime{}) {
			return fmt.Errorf("ssh connection cannot declare via_peer or wsl")
		}
	case "ssh-via":
		if !robotV4ManifestWorkingDirectoryValid(connection.WorkingDirectory) {
			return fmt.Errorf("working_directory %q must be a safe absolute POSIX path", connection.WorkingDirectory)
		}
		if connection.Network != "cell-link" || connection.ViaPeer == "" {
			return fmt.Errorf("ssh-via requires network=cell-link and via_peer")
		}
		if connection.WSL != (robotV4ManifestWSLRuntime{}) {
			return fmt.Errorf("ssh-via connection cannot declare wsl")
		}
	case "ssh-windows-wsl":
		if connection.Network != "lan" && connection.Network != "tailnet-v1" {
			return fmt.Errorf("ssh-windows-wsl network must be lan or tailnet-v1")
		}
		if connection.ViaPeer != "" {
			return fmt.Errorf("ssh-windows-wsl cannot declare via_peer")
		}
		if connection.WorkingDirectory != "" {
			return fmt.Errorf("ssh-windows-wsl owns working_directory inside wsl; do not duplicate it on the connection")
		}
		if !robotV4ManifestTokenPattern.MatchString(connection.WSL.Distribution) ||
			!robotV4ManifestUserPattern.MatchString(connection.WSL.User) ||
			!robotV4ManifestWorkingDirectoryValid(connection.WSL.WorkingDirectory) ||
			!robotV4ManifestWSLWorkingDirectoryPattern.MatchString(connection.WSL.WorkingDirectory) {
			return fmt.Errorf("ssh-windows-wsl requires a safe distribution, user, and absolute working_directory")
		}
	default:
		return fmt.Errorf("transport %q must be ssh, ssh-via, or ssh-windows-wsl", connection.Transport)
	}
	return nil
}

func robotV4ManifestHostValid(host string) bool {
	if host == "" || strings.ContainsAny(host, "@/\\\t\r\n ") || strings.Contains(host, "://") {
		return false
	}
	ipHost := host
	if zoneIndex := strings.LastIndex(ipHost, "%"); zoneIndex >= 0 {
		if zoneIndex == len(ipHost)-1 || !robotV4ManifestTokenPattern.MatchString(ipHost[zoneIndex+1:]) {
			return false
		}
		ipHost = ipHost[:zoneIndex]
	}
	return net.ParseIP(strings.Trim(ipHost, "[]")) != nil || robotV4ManifestHostnamePattern.MatchString(host)
}

func robotV4ManifestWorkingDirectoryValid(value string) bool {
	if value == "" || !pathpkg.IsAbs(value) || strings.ContainsAny(value, "\x00\r\n\t") {
		return false
	}
	for _, segment := range strings.Split(value, "/") {
		if segment == ".." {
			return false
		}
	}
	return true
}

func robotV4ValidateManifestMotionPolicy(policy robotV4ManifestMotionPolicy, connections map[string]robotV4ManifestConnection) error {
	policy.AutomationMode = strings.ToLower(strings.TrimSpace(policy.AutomationMode))
	policy.Fixture = strings.ToLower(strings.TrimSpace(policy.Fixture))
	policy.HumanAuthorization = strings.ToLower(strings.TrimSpace(policy.HumanAuthorization))
	policy.AutomatedFinalPosture = strings.ToLower(strings.TrimSpace(policy.AutomatedFinalPosture))
	policy.LiveTransitionPeer = strings.TrimSpace(policy.LiveTransitionPeer)
	switch policy.AutomationMode {
	case "bounded-live":
		if policy.Fixture != "bench-fixtured" ||
			policy.HumanAuthorization != "none" ||
			policy.AutomatedFinalPosture != "live-disarmed" {
			return fmt.Errorf("bounded-live policy must be bench-fixtured, human_authorization=none, and final live-disarmed")
		}
		if !strings.EqualFold(policy.LiveTransitionPeer, "controller") {
			if _, ok := connections[strings.ToLower(policy.LiveTransitionPeer)]; !ok ||
				!strings.HasPrefix(strings.ToLower(policy.LiveTransitionPeer), "steamdeck") {
				return fmt.Errorf("bounded-live live_transition_peer must be controller or a declared Steam Deck")
			}
		}
	case "simulation-only":
		if policy.Fixture != "production-cell" ||
			policy.HumanAuthorization != "steamdeck-simulation-toggle-off" ||
			policy.AutomatedFinalPosture != "simulation-disarmed" {
			return fmt.Errorf("simulation-only policy must be production-cell, Steam Deck toggle authorized, and final simulation-disarmed")
		}
		if _, ok := connections[strings.ToLower(policy.LiveTransitionPeer)]; !ok {
			return fmt.Errorf("simulation-only live_transition_peer %q has no connection", policy.LiveTransitionPeer)
		}
		if !strings.HasPrefix(strings.ToLower(policy.LiveTransitionPeer), "steamdeck") {
			return fmt.Errorf("simulation-only live_transition_peer must be the Steam Deck")
		}
	case "hardware-dark":
		if policy.Fixture != "runtime-only" ||
			policy.HumanAuthorization != "none" ||
			policy.AutomatedFinalPosture != "simulation-disarmed" ||
			!strings.EqualFold(policy.LiveTransitionPeer, "none") {
			return fmt.Errorf("hardware-dark policy must be runtime-only, human_authorization=none, final simulation-disarmed, and have no live transition peer")
		}
	default:
		return fmt.Errorf("motion_policy.automation_mode must be bounded-live, simulation-only, or hardware-dark")
	}
	return nil
}

func robotV4ValidateManifestTelemetry(telemetry robotV4ManifestTelemetry) error {
	if strings.ToLower(strings.TrimSpace(telemetry.Network)) != "tailnet-v1" {
		return fmt.Errorf("telemetry.network must be tailnet-v1")
	}
	parsed, err := url.Parse(strings.TrimSpace(telemetry.LegionLeafAuthority))
	if err != nil ||
		parsed.Scheme != "nats-leaf" ||
		parsed.User != nil ||
		parsed.Hostname() == "" ||
		parsed.Port() == "" ||
		parsed.Path != "" ||
		parsed.RawQuery != "" ||
		parsed.Fragment != "" {
		return fmt.Errorf("telemetry.legion_leaf_authority must be nats-leaf://<concrete-host>:<port>")
	}
	if !robotV4ManifestHostValid(parsed.Hostname()) {
		return fmt.Errorf("telemetry authority host %q is invalid", parsed.Hostname())
	}
	return nil
}

func robotV4ResolveManifestConnection(document robotV4ManifestDocument, inventory robotV4DaemonNodeInventoryManifest, target string) (robotV4ManifestConnection, error) {
	target = strings.ToLower(strings.TrimSpace(target))
	if target == "" {
		return robotV4ManifestConnection{}, fmt.Errorf("manifest connect requires --target")
	}
	peerMatches := map[string]bool{}
	for peer := range document.Connections {
		if target == strings.ToLower(strings.TrimSpace(peer)) {
			peerMatches[strings.ToLower(strings.TrimSpace(peer))] = true
		}
	}
	for _, node := range inventory.Nodes {
		for _, value := range append([]string{node.Node, node.Peer, node.Role, node.HostRole, node.Host, node.SSHHost}, node.Aliases...) {
			if target == strings.ToLower(strings.TrimSpace(value)) {
				peerMatches[strings.ToLower(strings.TrimSpace(node.Peer))] = true
			}
		}
	}
	for _, buildPeer := range inventory.BuildPeers {
		for _, value := range []string{buildPeer.Peer, buildPeer.Role, buildPeer.HostRole, buildPeer.Host, buildPeer.SSHHost} {
			if target == strings.ToLower(strings.TrimSpace(value)) {
				peerMatches[strings.ToLower(strings.TrimSpace(buildPeer.Peer))] = true
			}
		}
	}
	if len(peerMatches) != 1 {
		return robotV4ManifestConnection{}, fmt.Errorf("manifest target %q resolves to %d concrete peers", target, len(peerMatches))
	}
	for peer, connection := range document.Connections {
		if peerMatches[strings.ToLower(strings.TrimSpace(peer))] {
			connection.Peer = peer
			return robotV4NormalizeManifestConnection(connection), nil
		}
	}
	return robotV4ManifestConnection{}, fmt.Errorf("manifest target %q has no connection", target)
}

// robotV4ManifestManagementConnection is the single resolver for management
// routes used by deploy, cache, and operator tooling. The compatibility
// inventory may supply aliases and roles, but the returned host, SSH user, and
// relay peer always come from the manifest's connections map.
func robotV4ManifestManagementConnection(target string) (robotV4ManifestConnection, error) {
	manifestPath := selectedRobotV4ManifestPath()
	if strings.TrimSpace(manifestPath) == "" {
		return robotV4ManifestConnection{}, fmt.Errorf("management connection %q requires a selected Robot v4 manifest", target)
	}
	document, inventory, resolvedPath, err := robotV4LoadAndValidateManifestDocument(manifestPath)
	if err != nil {
		return robotV4ManifestConnection{}, err
	}
	connection, err := robotV4ResolveManifestConnection(document, inventory, target)
	if err != nil {
		return robotV4ManifestConnection{}, fmt.Errorf("resolve management connection %q from %s: %w", target, resolvedPath, err)
	}
	return connection, nil
}
