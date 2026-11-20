package main

import (
	"fmt"
	"net"
	"net/url"
	"path"
	"regexp"
	"strconv"
	"strings"
)

// robotV4ValidateShippedRuntimeSelection requires an approved native binding
// before deployment can acquire resources or re-enable a retired producer.
func robotV4ValidateShippedRuntimeSelection(manifest robotV4DaemonNodeInventoryManifest) error {
	for _, node := range manifest.Nodes {
		if node.Role == robotV4RealtimeCoreNode && !node.selectsRTCore() {
			return &retiredRuntimeError{Selection: node.Peer}
		}
	}
	return nil
}

type robotV4RTCoreConfig struct {
	PackageArtifact     string            `json:"package_artifact"`
	MachineConfig       string            `json:"machine_config"`
	ConfigurationSHA256 string            `json:"configuration_sha256"`
	PairID              string            `json:"pair_id"`
	PairRevision        uint64            `json:"pair_revision"`
	EtherCATDeviceGroup string            `json:"ethercat_device_group"`
	RemoteListen        string            `json:"remote_listen"`
	RemotePKI           string            `json:"remote_pki,omitempty"`
	NATS                robotV4RTCoreNATS `json:"nats"`
	Activate            bool              `json:"activate"`
}

// rtCoreRemotePKIInstalled names a host whose listener PKI and
// ROSIE_RT_REMOTE_ARGS are already provisioned in /etc/rosie-rt-core. Deploy
// keeps that listener line verbatim instead of deriving it from remote_listen,
// which then names the client-facing endpoint (for example a relay tunnel).
// Without it a deploy publishes ROSIE_RT_REMOTE_ARGS="" and the listener is off.
const rtCoreRemotePKIInstalled = "installed"

type robotV4RTCoreNATS struct {
	URL  string `json:"url,omitempty"`
	Host string `json:"host,omitempty"`
}

func (node robotV4DaemonNodeInventoryNode) selectsRTCore() bool {
	return node.Runtime == "rt-core"
}

func robotV4ManifestSelectsRTCore(manifest robotV4DaemonNodeInventoryManifest) bool {
	for _, node := range manifest.Nodes {
		if node.selectsRTCore() {
			return true
		}
	}
	return false
}

// validateRTCore binds an install-only package to the servo host and rejects
// legacy command producers, ambiguous identities and escaping package paths.
func (node robotV4DaemonNodeInventoryNode) validateRTCore(manifest robotV4DaemonNodeInventoryManifest) error {
	if node.Runtime != "" && node.Runtime != "v4" && node.Runtime != "rt-core" {
		return fmt.Errorf("unsupported runtime %q", node.Runtime)
	}
	if !node.selectsRTCore() {
		if node.RTCore != nil {
			return fmt.Errorf("rt_core requires runtime rt-core")
		}
		return nil
	}
	if node.Role != robotV4RealtimeCoreNode || node.RTCore == nil {
		return fmt.Errorf("rt-core requires the realtime-core servo-authority role and rt_core binding")
	}
	c := node.RTCore
	if err := c.validate(); err != nil {
		return err
	}
	// The dense axis map is derived by the adapter from the described axes of
	// the compiled configuration; the binding no longer restates robot_axes.
	if node.BuildCachePolicy.RuntimePeerBuild == nil || *node.BuildCachePolicy.RuntimePeerBuild ||
		node.BuildCachePolicy.Mode != "remote-cache-required" || !node.BuildCachePolicy.InstallOnlyAfterHashVerified {
		return fmt.Errorf("rt-core servo host must be hash-verified and install-only")
	}
	for _, name := range []string{"robot-v4-rtcore", "robot-v4-rtcore-nats-bridge"} {
		if _, exists := node.ProcessIntents[name]; exists {
			return fmt.Errorf("rt-core closure contains retired intent %s", name)
		}
	}
	for _, p := range node.RuntimePaths {
		if strings.HasPrefix(path.Base(p), "robot-v4-rtcore") {
			return fmt.Errorf("rt-core closure contains retired runtime path %s", p)
		}
	}
	for key, value := range node.DaemonEnvironment {
		if strings.HasPrefix(key, "ROBOT_V4_RTCORE_COMMAND_") || strings.Contains(value, "/run/robot-v4-rtcore") {
			return fmt.Errorf("rt-core closure contains retired daemon environment %s", key)
		}
	}
	if len(node.ColdPathCommands) != 0 {
		return fmt.Errorf("rt-core install authorises no daemon cold-path commands")
	}
	for _, binary := range []string{"rosie-rt-core", "rt-control", "rtctl", "rt-natspublisher"} {
		if !containsString(node.RuntimePaths, "/opt/rosie-rt-core/current/bin/"+binary) {
			return fmt.Errorf("rt-core runtime_paths missing %s", binary)
		}
	}
	semantic := robotV4ManifestSemanticIdentityTokens(manifest)
	for field, host := range map[string]string{"pair_id": c.PairID, "nats.host": c.NATS.Host} {
		if host != "" && robotV4ManifestIdentityIsSemantic(semantic, host) {
			return fmt.Errorf("rt_core %s must be concrete", field)
		}
	}
	for _, address := range []string{c.NATS.URL, "tcp://" + c.RemoteListen} {
		u, err := url.Parse(address)
		if err == nil && robotV4ManifestIdentityIsSemantic(semantic, u.Hostname()) {
			return fmt.Errorf("rt_core address must use a concrete host")
		}
	}
	return nil
}

func (c robotV4RTCoreConfig) validate() error {
	if c.PackageArtifact != "rosie-rt-core" {
		return fmt.Errorf("rt_core package_artifact must be rosie-rt-core")
	}
	if !strings.HasPrefix(c.MachineConfig, "rt-core/config/") || path.Clean(c.MachineConfig) != c.MachineConfig ||
		path.Ext(c.MachineConfig) != ".json" || strings.ContainsAny(c.MachineConfig, "\\\r\n") {
		return fmt.Errorf("rt_core machine_config must be a repository rt-core/config JSON path")
	}
	if !regexp.MustCompile(`^[0-9a-f]{64}$`).MatchString(c.ConfigurationSHA256) {
		return fmt.Errorf("rt_core configuration_sha256 must pin the approved compiled configuration (lowercase 64-hex)")
	}
	if c.RemotePKI != "" && c.RemotePKI != rtCoreRemotePKIInstalled {
		return fmt.Errorf("rt_core remote_pki must be %q or absent", rtCoreRemotePKIInstalled)
	}
	token := regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]*$`)
	if !token.MatchString(c.PairID) || c.PairRevision == 0 {
		return fmt.Errorf("rt_core requires a concrete pair_id and a positive pair_revision")
	}
	if !regexp.MustCompile(`^[a-z_][a-z0-9_-]*$`).MatchString(c.EtherCATDeviceGroup) {
		return fmt.Errorf("rt_core requires a valid ethercat_device_group")
	}
	host, port, err := net.SplitHostPort(c.RemoteListen)
	n, portErr := strconv.ParseUint(port, 10, 16)
	if err != nil || host == "" || portErr != nil || n == 0 || strings.ContainsAny(host, " \t\r\n") {
		return fmt.Errorf("rt_core remote_listen requires host:port")
	}
	if (c.NATS.URL == "") != (c.NATS.Host == "") {
		return fmt.Errorf("rt_core nats requires both the broker URL and concrete publisher host, or neither")
	}
	if c.NATS.URL != "" {
		u, err := url.Parse(c.NATS.URL)
		if err != nil || u.Scheme != "nats" || u.Hostname() == "" || u.User != nil || u.Path != "" || u.RawQuery != "" || u.Fragment != "" {
			return fmt.Errorf("rt_core nats.url requires a credential-free nats URL")
		}
	}
	if c.NATS.Host != "" && (!robotV4PlainConcreteIdentitySyntax(c.NATS.Host) || strings.ContainsAny(c.NATS.Host, " \t\r\n")) {
		return fmt.Errorf("rt_core nats.host must be concrete")
	}
	return nil
}
