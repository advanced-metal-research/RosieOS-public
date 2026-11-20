package main

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"net"
	pathpkg "path"
	"sort"
	"strconv"
	"strings"
)

const robotV4DaemonRuntimeConfigPath = "~/.config/rosie-daemon-v1/runtime-config.json"
const robotV4DaemonMetadataFileName = "daemon-v1.metadata.json"
const robotV4DaemonDefaultSelfRestartDelaySeconds = 12

type robotV4DaemonRuntimeConfig struct {
	SchemaVersion             string                               `json:"schema_version"`
	Node                      string                               `json:"node"`
	Role                      string                               `json:"role"`
	StartupArgs               []string                             `json:"startup_args"`
	MetadataFile              string                               `json:"metadata_file"`
	AdvertisedAddresses       []string                             `json:"advertised_addresses"`
	BlobFetchURLs             []string                             `json:"blob_fetch_urls"`
	PeerBuildsEnabled         bool                                 `json:"peer_builds_enabled"`
	ProcessSupervisionEnabled bool                                 `json:"process_supervision_enabled"`
	SelfRestartCommand        string                               `json:"self_restart_command"`
	SelfRestartDelaySeconds   int                                  `json:"self_restart_delay_seconds"`
	DeployAllowedRoots        []string                             `json:"deploy_allowed_roots"`
	RobotCommandAdapters      []robotV4DaemonRuntimeCommandAdapter `json:"robot_command_adapters"`
}

type robotV4DaemonRuntimeCommandAdapter struct {
	Name        string   `json:"name"`
	Service     string   `json:"service"`
	AdapterRef  string   `json:"adapter_ref"`
	AdapterKind string   `json:"adapter_kind"`
	TargetNodes []string `json:"target_nodes"`
	TargetPeer  string   `json:"target_peer"`
	Commands    []string `json:"commands"`
	Packet      string   `json:"packet"`
	UDPAddr     string   `json:"udp_addr"`
	SenderID    string   `json:"sender_id"`
}

func compileRobotV4DaemonRuntimeConfig(manifest robotV4DaemonNodeInventoryManifest, node robotV4DaemonNodeInventoryNode) (robotV4DaemonRuntimeConfig, error) {
	startupArgs, err := robotV4DaemonStartupArgsFromManifest(node)
	if err != nil {
		return robotV4DaemonRuntimeConfig{}, err
	}
	selfRestartCommand, err := robotV4DaemonSelfRestartCommandFromManifest(node)
	if err != nil {
		return robotV4DaemonRuntimeConfig{}, err
	}
	environment, err := robotV4DaemonEnvironmentFromManifest(node)
	if err != nil {
		return robotV4DaemonRuntimeConfig{}, err
	}
	return robotV4DaemonRuntimeConfig{
		SchemaVersion:             "robot_v4_daemon_runtime_config_v1",
		Node:                      firstNonEmpty(node.Peer, node.Host),
		Role:                      strings.TrimSpace(node.Role),
		StartupArgs:               startupArgs,
		MetadataFile:              robotV4DaemonRuntimeMetadataPathForNode(node),
		AdvertisedAddresses:       robotV4SortedUnique(node.AdvertisedAddrs),
		BlobFetchURLs:             robotV4DaemonRuntimeBlobFetchURLs(manifest, node),
		PeerBuildsEnabled:         robotV4DaemonRuntimePeerBuildsEnvValue(node) == "1",
		ProcessSupervisionEnabled: robotV4DaemonRuntimeProcessSupervisionEnabled(node),
		SelfRestartCommand:        selfRestartCommand,
		SelfRestartDelaySeconds:   robotV4DaemonRuntimeRestartDelaySeconds(environment),
		DeployAllowedRoots:        robotV4DaemonRuntimeDeployAllowedRootsForNode(environment, node),
		RobotCommandAdapters:      robotV4DaemonRuntimeCommandAdapters(environment, node),
	}, nil
}

func robotV4DaemonSelfRestartCommandFromManifest(node robotV4DaemonNodeInventoryNode) (string, error) {
	command := strings.TrimSpace(node.DaemonSelfRestart)
	if strings.ContainsAny(command, "\r\n") {
		return "", fmt.Errorf("deployment manifest node %q daemon_self_restart_command must be a single-line command", firstNonEmpty(node.Node, node.Peer))
	}
	if command != "" {
		return command, nil
	}
	units, err := robotV4DaemonServiceUnitsFromManifest(node)
	if err != nil {
		return "", err
	}
	return robotV4DaemonRestartCommandFromServiceUnits(units), nil
}

func robotV4DaemonServiceUnitsFromManifest(node robotV4DaemonNodeInventoryNode) ([]string, error) {
	label := firstNonEmpty(node.Node, node.Peer)
	candidates := append([]string{node.DaemonService.Unit}, node.DaemonService.FallbackUnits...)
	units := make([]string, 0, len(candidates))
	seen := map[string]bool{}
	for _, raw := range candidates {
		unit := strings.TrimSpace(raw)
		if unit == "" {
			continue
		}
		if strings.ContainsAny(unit, " \t\r\n'\";|&$`\\") {
			return nil, fmt.Errorf("deployment manifest node %q daemon_service unit %q must be a plain systemd unit name", label, raw)
		}
		if !strings.HasSuffix(unit, ".service") {
			return nil, fmt.Errorf("deployment manifest node %q daemon_service unit %q must end with .service", label, raw)
		}
		if seen[unit] {
			continue
		}
		seen[unit] = true
		units = append(units, unit)
	}
	return units, nil
}

func robotV4DaemonRestartCommandFromServiceUnits(units []string) string {
	if len(units) == 0 {
		return ""
	}
	commands := make([]string, 0, len(units))
	for _, unit := range units {
		unit = strings.TrimSpace(unit)
		if unit == "" {
			continue
		}
		commands = append(commands, "systemctl --user restart "+unit)
	}
	return strings.Join(commands, " || ")
}

func canonicalRobotV4DaemonRuntimeConfigJSON(config robotV4DaemonRuntimeConfig) ([]byte, error) {
	raw, err := json.Marshal(config)
	if err != nil {
		return nil, fmt.Errorf("marshal daemon runtime config: %w", err)
	}
	return raw, nil
}

func robotV4DaemonRuntimeConfigHash(config robotV4DaemonRuntimeConfig) (string, []byte, error) {
	raw, err := canonicalRobotV4DaemonRuntimeConfigJSON(config)
	if err != nil {
		return "", nil, err
	}
	sum := sha256.Sum256(raw)
	return "sha256:" + hex.EncodeToString(sum[:]), raw, nil
}

func robotV4SortedUnique(values []string) []string {
	seen := make(map[string]bool, len(values))
	out := make([]string, 0, len(values))
	for _, value := range values {
		value = strings.TrimSpace(value)
		if value == "" || seen[value] {
			continue
		}
		seen[value] = true
		out = append(out, value)
	}
	sort.Strings(out)
	return out
}

func robotV4DaemonRuntimeProcessSupervisionEnabled(node robotV4DaemonNodeInventoryNode) bool {
	if node.DaemonProcessSupervision == nil {
		return true
	}
	return *node.DaemonProcessSupervision
}

func robotV4DaemonRuntimeRestartDelaySeconds(environment []string) int {
	if value := robotV4DaemonRuntimeEnvironmentValue(environment, "DAEMON_V1_SELF_RESTART_DELAY_SECS"); value != "" {
		if seconds, err := strconv.Atoi(value); err == nil && seconds >= 0 {
			return seconds
		}
	}
	return robotV4DaemonDefaultSelfRestartDelaySeconds
}

func robotV4DaemonRuntimeDeployAllowedRoots(environment []string) []string {
	raw := robotV4DaemonRuntimeEnvironmentValue(environment, "DAEMON_V1_DEPLOY_ALLOWED_ROOTS")
	if raw == "" {
		return []string{}
	}
	return robotV4SortedUnique(strings.Split(raw, ":"))
}

// robotV4DaemonRuntimeDeployAllowedRootsForNode mirrors the bootstrap script's
// derivation: an explicit DAEMON_V1_DEPLOY_ALLOWED_ROOTS in the manifest wins,
// otherwise the roots are derived from the node's runtime_paths + rosie_home so
// mesh apply can always install the artifacts the manifest tells the node to
// run. Keeping this identical to bootstrap-daemon-v1-peers.mjs keeps the
// compiled runtime-config.json hash in sync with the daemon env the peer runs.
func robotV4DaemonRuntimeDeployAllowedRootsForNode(environment []string, node robotV4DaemonNodeInventoryNode) []string {
	if explicit := robotV4DaemonRuntimeDeployAllowedRoots(environment); len(explicit) > 0 {
		return explicit
	}
	roots := []string{"/tmp/rosie-daemon-v1-deploy"}
	rosieHome := strings.TrimRight(strings.TrimSpace(node.RosieHome), "/")
	if rosieHome != "" {
		roots = append(roots, rosieHome+"/bin", rosieHome+"/daemon-v1-deploy")
	}
	for _, runtimePath := range node.RuntimePaths {
		dir := pathpkg.Dir(strings.TrimSpace(runtimePath))
		if dir != "" && dir != "." && dir != "/" {
			roots = append(roots, dir)
		}
	}
	return robotV4SortedUnique(roots)
}

func robotV4DaemonRuntimeEnvironmentValue(environment []string, key string) string {
	for _, value := range environment {
		envKey, envValue, ok := strings.Cut(value, "=")
		if ok && strings.TrimSpace(envKey) == key {
			return strings.TrimSpace(envValue)
		}
	}
	return ""
}

func robotV4DaemonRuntimeCommandAdapters(environment []string, node robotV4DaemonNodeInventoryNode) []robotV4DaemonRuntimeCommandAdapter {
	adapters := make([]robotV4DaemonRuntimeCommandAdapter, 0, len(node.ColdPathCommands))
	targetNodes := robotV4DaemonRuntimeCSV(environment, "ROBOT_V4_RTCORE_COMMAND_TARGET_NODES")
	senderID := robotV4DaemonRuntimeEnvironmentValue(environment, "ROBOT_V4_RTCORE_COMMAND_SENDER_ID")
	for _, command := range node.ColdPathCommands {
		if strings.TrimSpace(command.AdapterKind) == "" && strings.TrimSpace(command.AdapterTarget.UDPAddr) == "" {
			continue
		}
		adapter := robotV4DaemonRuntimeCommandAdapter{
			Name:        strings.TrimSpace(command.Name),
			Service:     strings.TrimSpace(command.Service),
			AdapterRef:  strings.TrimSpace(command.AdapterRef),
			AdapterKind: strings.TrimSpace(command.AdapterKind),
			TargetNodes: robotV4SortedUnique(append(targetNodes, strings.TrimSpace(command.TargetNode))),
			TargetPeer:  strings.TrimSpace(command.TargetPeer),
			Commands:    robotV4SortedUnique(command.Commands),
			Packet:      strings.TrimSpace(command.AdapterTarget.Packet),
			UDPAddr:     firstNonEmpty(strings.TrimSpace(command.AdapterTarget.UDPAddr), robotV4DaemonRuntimeEnvironmentValue(environment, "ROBOT_V4_RTCORE_COMMAND_UDP_ADDR")),
			SenderID:    senderID,
		}
		adapters = append(adapters, adapter)
	}
	sort.Slice(adapters, func(i, j int) bool {
		return adapters[i].Name < adapters[j].Name
	})
	return adapters
}

func robotV4DaemonRuntimeCSV(environment []string, key string) []string {
	value := robotV4DaemonRuntimeEnvironmentValue(environment, key)
	if value == "" {
		return []string{}
	}
	return strings.Split(value, ",")
}

func robotV4DaemonRuntimeBlobFetchURLs(manifest robotV4DaemonNodeInventoryManifest, node robotV4DaemonNodeInventoryNode) []string {
	parentURL := strings.TrimSpace(node.IPv6Fallback.ParentDaemonURLFromTarget)
	if parentURL == "" {
		return []string{}
	}
	parentName := firstNonEmpty(node.IPv6Fallback.ParentNode, node.ParentNode)
	parent, ok := robotV4ManifestNodeByDirectIdentity(manifest, parentName)
	if !ok {
		return []string{}
	}
	names := robotV4DaemonConcreteIdentityAliases(parent)
	if parentPeer := strings.TrimSpace(node.IPv6Fallback.ParentPeer); parentPeer != "" &&
		!robotV4DaemonSemanticIdentityAlias(parent, parentPeer) &&
		robotV4DaemonConcreteIdentityCandidate([]string{parent.Peer, parent.Host, parent.SSHHost}, parentPeer) {
		names = robotV4SortedUnique(append(names, parentPeer))
	}
	urls := make([]string, 0, len(names))
	for _, name := range names {
		if strings.ContainsAny(name, "=,\t\r\n ") {
			continue
		}
		urls = append(urls, name+"="+parentURL)
	}
	sort.Strings(urls)
	return urls
}

// robotV4DaemonConcreteIdentityAliases returns only values that may identify a
// physical daemon peer or host. Logical selectors and roles are deliberately
// excluded so they cannot silently stand in for a peer ID at protocol
// boundaries.
func robotV4DaemonConcreteIdentityAliases(node robotV4DaemonNodeInventoryNode) []string {
	explicit := []string{node.Peer, node.Host, node.SSHHost}
	candidates := append(append([]string{}, explicit...), node.Aliases...)
	out := make([]string, 0, len(candidates))
	for _, candidate := range candidates {
		candidate = strings.TrimSpace(candidate)
		if candidate == "" || robotV4DaemonSemanticIdentityAlias(node, candidate) ||
			!robotV4DaemonConcreteIdentityCandidate(explicit, candidate) {
			continue
		}
		out = append(out, candidate)
	}
	return robotV4SortedUnique(out)
}

// robotV4DaemonConcreteIdentityCandidate accepts an explicitly declared peer
// ID/host, or a network-qualified alias. Generic selectors such as "revpi"
// and dotted daemon labels such as "robot-v4.motion-server.cell-a" are not
// transport identities and must never become URL authorities, SSH targets, or
// gossip addresses.
func robotV4DaemonConcreteIdentityCandidate(explicit []string, candidate string) bool {
	candidate = strings.TrimSpace(candidate)
	if candidate == "" || strings.ContainsAny(candidate, "=,/\\\t\r\n ") || strings.Contains(candidate, "://") {
		return false
	}
	for _, value := range explicit {
		if strings.TrimSpace(value) != "" && strings.EqualFold(strings.TrimSpace(value), candidate) {
			return true
		}
	}

	normalized := strings.Trim(strings.TrimSpace(candidate), "[]")
	if zoneIndex := strings.LastIndex(normalized, "%"); zoneIndex >= 0 {
		normalized = normalized[:zoneIndex]
	}
	if net.ParseIP(normalized) != nil {
		return true
	}
	lower := strings.ToLower(candidate)
	if strings.HasPrefix(lower, "robot-v4.") || !strings.Contains(lower, ".") {
		return false
	}
	for _, label := range strings.Split(lower, ".") {
		if label == "" || strings.Trim(label, "abcdefghijklmnopqrstuvwxyz0123456789-") != "" ||
			strings.HasPrefix(label, "-") || strings.HasSuffix(label, "-") {
			return false
		}
	}
	return true
}

func robotV4DaemonSemanticIdentityAlias(node robotV4DaemonNodeInventoryNode, candidate string) bool {
	candidate = strings.TrimSpace(candidate)
	if candidate == "" {
		return false
	}
	if robotV4ManifestIdentityIsSemantic(nil, candidate) {
		return true
	}
	for _, semantic := range []string{node.Role, node.HostRole} {
		semantic = strings.TrimSpace(semantic)
		if semantic != "" && strings.EqualFold(candidate, semantic) {
			return true
		}
	}
	nodeSelector := strings.TrimSpace(node.Node)
	if nodeSelector != "" && strings.EqualFold(candidate, nodeSelector) &&
		!strings.EqualFold(candidate, strings.TrimSpace(node.Peer)) &&
		!strings.EqualFold(candidate, strings.TrimSpace(node.Host)) {
		return true
	}
	return false
}

func robotV4ManifestNodeByDirectIdentity(manifest robotV4DaemonNodeInventoryManifest, alias string) (robotV4DaemonNodeInventoryNode, bool) {
	alias = strings.TrimSpace(strings.ToLower(alias))
	if alias == "" {
		return robotV4DaemonNodeInventoryNode{}, false
	}
	for _, node := range manifest.Nodes {
		candidates := []string{node.Node, node.Peer, node.Role, node.Host}
		for _, candidate := range candidates {
			if strings.TrimSpace(strings.ToLower(candidate)) == alias {
				return node, true
			}
		}
	}
	return robotV4DaemonNodeInventoryNode{}, false
}

func robotV4DaemonRuntimeMetadataPathForNode(node robotV4DaemonNodeInventoryNode) string {
	if home := robotV4DaemonRuntimeHomeForNode(node); home != "" {
		return pathpkg.Join(home, ".config/rosie-daemon-v1", robotV4DaemonMetadataFileName)
	}
	return robotV4DaemonV1MetadataPath(node.DaemonRuntimePath)
}

func robotV4DaemonRuntimeConfigPathForNode(node robotV4DaemonNodeInventoryNode) string {
	if home := robotV4DaemonRuntimeHomeForNode(node); home != "" {
		return pathpkg.Join(home, ".config/rosie-daemon-v1/runtime-config.json")
	}
	return robotV4DaemonRuntimeConfigPath
}

func robotV4DaemonRuntimeHomeForNode(node robotV4DaemonNodeInventoryNode) string {
	if home := robotV4DaemonRuntimeHomeFromRosieHome(node.RosieHome); home != "" {
		return home
	}
	return robotV4DaemonRuntimeHomeFromRuntimePath(node.DaemonRuntimePath)
}

func robotV4DaemonRuntimeHomeFromRosieHome(rosieHome string) string {
	rosieHome = strings.TrimSpace(rosieHome)
	if rosieHome == "" || !strings.HasPrefix(rosieHome, "/") {
		return ""
	}
	rosieHome = strings.TrimRight(pathpkg.Clean(rosieHome), "/")
	if pathpkg.Base(rosieHome) == ".rosie" {
		return pathpkg.Dir(rosieHome)
	}
	return pathpkg.Dir(rosieHome)
}

func robotV4DaemonRuntimeHomeFromRuntimePath(runtimePath string) string {
	runtimePath = strings.TrimSpace(runtimePath)
	if runtimePath == "" || !strings.HasPrefix(runtimePath, "/") {
		return ""
	}
	runtimePath = strings.TrimRight(pathpkg.Clean(runtimePath), "/")
	if pathpkg.Base(runtimePath) != "rosie-daemon-v1" {
		return ""
	}
	binDir := pathpkg.Dir(runtimePath)
	rosieHome := pathpkg.Dir(binDir)
	if pathpkg.Base(rosieHome) == ".rosie" {
		return pathpkg.Dir(rosieHome)
	}
	return ""
}
