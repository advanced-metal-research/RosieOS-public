package main

import (
	"encoding/json"
	"fmt"

	"net"

	"net/url"
	"os"

	"path/filepath"
	"strconv"
	"strings"
	"time"
)

type deployOptions struct {
	Host            string
	BuildServer     string
	DaemonURL       string
	ManifestPath    string
	TargetHost      string
	TargetUser      string
	ViaHost         string
	ViaUser         string
	RemoteRepo      string
	BinaryPath      string
	BinarySHA256    string
	BuildID         string
	TestID          string
	SystemState     string
	Transport       string
	Fast            bool
	ShortLog        bool
	ReadySimulation bool
	Tour            bool
	NoVideo         bool
	TourTag         string
	SteamDeckTarget string
	PR              int
	Finish          string
	TourProofOutput string
	// AutomaticTourFingerprint is an internal manifest-wrapper marker. An
	// explicit/manual --tour leaves it empty and retains the historical path.
	AutomaticTourFingerprint string
	// AutomaticTourPublicationRetryFingerprint selects the receipt-backed,
	// no-motion delivery recovery path for an already completed deployment.
	AutomaticTourPublicationRetryFingerprint string
	// AutomaticTourLease carries the exact pre-mutation lease owner through the
	// SSH deployment and into the physical post-deploy tour.
	AutomaticTourLease *robotV4TourDeploymentLease
	// AutomaticTourRuntime carries the manifest-owned evidence sidecar and
	// launch ordering state through the fast deploy and proof collector.
	AutomaticTourRuntime *robotV4AutomaticTourRuntime
	// BridgeRun binds the one run-owned live-enable transition used by the
	// exact Rosie live bench, whether its Steam client is WSL-hosted or a
	// physical manifest SteamDeck. Ordinary posture switches leave it nil.
	BridgeRun             *robotV4BridgeRunBinding
	BridgeRunPlanSHA256   string
	BridgeRunLimitsSHA256 string
	// AutomaticTourPreProofError records a failure before the proof waiter can
	// run so the normal safe-gate and failure-publication path can finish it.
	AutomaticTourPreProofError error
	NoWait                     bool
	BatchPublish               bool
	SSHBackup                  bool
	WaitTimeout                time.Duration
	DryRun                     bool
}

type robotV4MeshDeployTarget struct {
	Peer                    string
	Role                    string
	ArtifactName            string
	ArtifactKind            string
	BuildInputPaths         []string
	BuildCommand            string
	DaemonEnvironment       []string
	DaemonRuntimeConfigJSON string
	DaemonRuntimeConfigHash string
	DaemonRuntimeConfigPath string
	RequiredRuntimePath     string
	InstallStrategy         string
	HealthCheck             string
	HealthExpectedField     string
	HealthExpectedValue     string
	ActiveHashReportField   string
}

type robotV4DeployManifestArtifact struct {
	ArtifactName                         string         `json:"artifact_name"`
	ArtifactKind                         string         `json:"artifact_kind"`
	TargetNode                           string         `json:"target_node,omitempty"`
	TargetHost                           string         `json:"target_host"`
	TargetPeer                           string         `json:"target_peer,omitempty"`
	TargetRole                           string         `json:"target_role,omitempty"`
	SourceTreeHash                       string         `json:"source_tree_hash"`
	BuildInputPaths                      []string       `json:"build_input_paths"`
	BuildRecipeSHA256                    string         `json:"build_recipe_sha256,omitempty"`
	SourceBundleSHA256                   string         `json:"source_bundle_sha256"`
	SourceBundleURI                      string         `json:"source_bundle_uri"`
	BuildCommand                         string         `json:"build_command"`
	BuildFromSource                      bool           `json:"build_from_source,omitempty"`
	DaemonEnvironment                    []string       `json:"daemon_environment,omitempty"`
	DaemonRuntimeConfigJSON              string         `json:"daemon_runtime_config_json,omitempty"`
	DaemonRuntimeConfigHash              string         `json:"daemon_runtime_config_hash,omitempty"`
	DaemonRuntimeConfigPath              string         `json:"daemon_runtime_config_path,omitempty"`
	BinarySHA256                         string         `json:"binary_sha256"`
	BlobURI                              string         `json:"blob_uri"`
	SyncTransport                        string         `json:"sync_transport"`
	WholeRepoSync                        bool           `json:"whole_repo_sync"`
	GitRequired                          bool           `json:"git_required"`
	SSHRequired                          bool           `json:"ssh_required"`
	RequiredRuntimePath                  string         `json:"required_runtime_path"`
	InstallStrategy                      string         `json:"install_strategy"`
	HealthCheck                          map[string]any `json:"health_check"`
	ActiveHashReportField                string         `json:"active_hash_report_field"`
	ExpectedProducerManifestID           string         `json:"-"`
	ExpectedProducerBuildSessionID       string         `json:"-"`
	ExpectedProducerSourceIdentitySHA256 string         `json:"-"`
}

type robotV4SourceBundle struct {
	SchemaVersion     string                    `json:"schema_version"`
	TargetHost        string                    `json:"target_host"`
	BuildRecipeSHA256 string                    `json:"build_recipe_sha256"`
	Files             []robotV4SourceBundleFile `json:"files"`
}

type robotV4SourceBundleFile struct {
	Path          string `json:"path"`
	SHA256        string `json:"sha256"`
	PayloadBase64 string `json:"payload_base64"`
}

type daemonBlobUploadResponse struct {
	OK       bool   `json:"ok"`
	SHA256   string `json:"sha256"`
	URI      string `json:"uri"`
	CacheHit bool   `json:"cache_hit,omitempty"`
}

type robotV4DaemonDeployBuildCacheRecord struct {
	ID                           string `json:"id"`
	PeerNode                     string `json:"peer_node"`
	Component                    string `json:"component"`
	TargetHost                   string `json:"target_host"`
	BuildProfile                 string `json:"build_profile"`
	Toolchain                    string `json:"toolchain"`
	PlatformOS                   string `json:"platform_os"`
	PlatformArch                 string `json:"platform_arch"`
	TargetTriple                 string `json:"target_triple"`
	TargetABI                    string `json:"target_abi"`
	SourceTreeHash               string `json:"source_tree_hash"`
	SourceBundleSHA256           string `json:"source_bundle_sha256"`
	SourceBundleURI              string `json:"source_bundle_uri"`
	BinarySHA256                 string `json:"binary_sha256"`
	BinaryBlobURI                string `json:"binary_blob_uri"`
	ProducerManifestID           string `json:"producer_manifest_id,omitempty"`
	ProducerBuildSessionID       string `json:"producer_build_session_id,omitempty"`
	ProducerSourceIdentitySHA256 string `json:"producer_source_identity_sha256,omitempty"`
	// BuildInputPaths must survive the round trip from the build peer's
	// record back to the orchestrator daemon, whose deploy policy rejects
	// build-cache records without component-scoped build inputs.
	BuildInputPaths []string `json:"build_input_paths"`
	GitRequired     bool     `json:"git_required"`
	SSHRequired     bool     `json:"ssh_required"`
	WholeRepoSync   bool     `json:"whole_repo_sync"`
	Status          string   `json:"status"`
	UpdatedAtUnix   int64    `json:"updated_at_unix"`
}

type robotV4DaemonHealth struct {
	OK            bool   `json:"ok"`
	Node          string `json:"node"`
	PID           int    `json:"pid"`
	StartedAtUnix int64  `json:"started_at_unix"`
}

var (
	robotV4RealtimeCoreRemoteSSH = runRemoteSSH
	robotV4ManifestPathOverride  string
)

type robotV4DaemonNodeInventoryManifest struct {
	Name string `json:"name"`
	// LeaderControllerRoles is the explicit manifest allowlist shared by the
	// Motion Server and RTCore's final command bridge. Omitting it preserves
	// the legacy Steam Deck-only path and does not enable Offline Programming.
	LeaderControllerRoles []string `json:"leader_controller_roles"`
	// RobotCommandSubject is the single per-robot-cell NATS command subject
	// (e.g. robot/v4/robot.cell-a.command). Every robot command (arm,
	// disarm, hm35, go_home, position, io) rides this one subject with the
	// command kind and target robot in the payload.
	RobotCommandSubject string                           `json:"robot_command_subject"`
	IndexURL            string                           `json:"index_url"`
	PeerStaleAfterS     int64                            `json:"peer_stale_after_s"`
	BuildPeers          []robotV4BuildPeerInventory      `json:"build_peers"`
	Nodes               []robotV4DaemonNodeInventoryNode `json:"nodes"`
	// RobotAxes is the manifest's single source of truth for the robot cell's
	// live servo axes: how many there are, their URDF joint names, and which
	// participate in Cartesian TCP motion. Runtime components (rtcore, the
	// Cartesian daemon, the NATS bridge, and the Steam Deck) read this instead
	// of hardcoding an axis count. It is required; omission fails closed.
	RobotAxes          robotV4RobotAxesConfig          `json:"robot_axes"`
	RuntimeConnections robotV4RuntimeConnectionsConfig `json:"runtime_connections"`
	DeployChecks       robotV4DeployChecksConfig       `json:"deploy_checks"`
}

type robotV4DeployChecksConfig struct {
	UIPort80           robotV4DeployUIHTTPCheck             `json:"ui_port80"`
	ServoUIPort80      robotV4DeployUIHTTPCheck             `json:"servo_ui_port80"`
	ServoUIPort443     robotV4DeployUIHTTPCheck             `json:"servo_ui_port443"`
	ServoReadyDisarmed robotV4DeployServoReadyDisarmedCheck `json:"servo_ready_disarmed"`
}

type robotV4DeployUIHTTPCheck struct {
	URL                          string                   `json:"url"`
	ExpectedStatus               int                      `json:"expected_status"`
	RequiredText                 []string                 `json:"required_text"`
	TimeoutMS                    int                      `json:"timeout_ms"`
	RealtimeCoreWebSocketPath    string                   `json:"realtime_core_websocket_path"`
	RequireRealtimeCoreAxisCount bool                     `json:"require_realtime_core_axis_count"`
	RuntimeDefaults              robotV4UIRuntimeDefaults `json:"runtime_defaults"`
}

// robotV4UIRuntimeDefaults is the manifest-owned visual-tour startup contract
// for every browser URL emitted by the Robot v4 deploy tooling. It does not
// disable the normal runtime-gated operator controls after TOUR is stopped.
// Existing query parameters remain explicit per-client overrides.
type robotV4UIRuntimeDefaults struct {
	TourMode              string `json:"tour_mode"`
	CalibrationTourMode   string `json:"calibration_tour_mode"`
	Tour                  string `json:"tour"`
	TourExecutionPolicy   string `json:"tour_execution_policy"`
	PostTourControlPolicy string `json:"post_tour_control_policy"`
}

func (defaults robotV4UIRuntimeDefaults) validate(context string) error {
	context = strings.TrimSpace(context)
	if context == "" {
		context = "Robot v4 UI runtime defaults"
	}
	defaults.Tour = strings.TrimSpace(defaults.Tour)
	defaults.TourMode = strings.TrimSpace(defaults.TourMode)
	defaults.CalibrationTourMode = strings.TrimSpace(defaults.CalibrationTourMode)
	defaults.TourExecutionPolicy = strings.TrimSpace(defaults.TourExecutionPolicy)
	defaults.PostTourControlPolicy = strings.TrimSpace(defaults.PostTourControlPolicy)
	if defaults.Tour != "one-shot" && defaults.Tour != "off" {
		return fmt.Errorf("%s tour must be one-shot or off, got %q", context, defaults.Tour)
	}
	if defaults.TourMode != "fast-test" && defaults.TourMode != "slow-user" {
		return fmt.Errorf("%s tour_mode must be fast-test or slow-user, got %q", context, defaults.TourMode)
	}
	if defaults.CalibrationTourMode != "fast-test" && defaults.CalibrationTourMode != "slow-user" {
		return fmt.Errorf("%s calibration_tour_mode must be fast-test or slow-user, got %q", context, defaults.CalibrationTourMode)
	}
	if defaults.TourExecutionPolicy != "simulation-visual-no-send" {
		return fmt.Errorf("%s tour_execution_policy must be simulation-visual-no-send, got %q", context, defaults.TourExecutionPolicy)
	}
	if defaults.PostTourControlPolicy != "runtime-mode-gated" {
		return fmt.Errorf("%s post_tour_control_policy must be runtime-mode-gated, got %q", context, defaults.PostTourControlPolicy)
	}
	return nil
}

func (defaults robotV4UIRuntimeDefaults) clientURL(rawURL string) (string, error) {
	if err := defaults.validate("Robot v4 UI client URL defaults"); err != nil {
		return "", err
	}
	parsed, err := url.Parse(strings.TrimSpace(rawURL))
	if err != nil || (parsed.Scheme != "http" && parsed.Scheme != "https") || strings.TrimSpace(parsed.Host) == "" {
		return "", fmt.Errorf("Robot v4 UI client URL is invalid: %q", strings.TrimSpace(rawURL))
	}
	// The web client natively defaults to tour-off. Keep the emitted
	// URL query-free so callers do not have to supply or strip policy state;
	// when a caller does provide an explicit query override, preserve it byte
	// for byte through URL parsing and host substitution.
	return parsed.String(), nil
}

type robotV4DeployServoReadyDisarmedCheck struct {
	URL                     string `json:"url"`
	TimeoutMS               int    `json:"timeout_ms"`
	RequireCSPMode          bool   `json:"require_csp_mode"`
	RequireZeroMasks        bool   `json:"require_zero_masks"`
	RequireLiveMotionUnlock bool   `json:"require_live_motion_unlock"`
	MaxPositionDeltaCounts  int    `json:"max_position_delta_counts"`
}

// robotV4RobotAxesConfig describes the robot cell's live servo axes. It is the
// manifest source of truth that removes the compile-time axis-count hardcode
// from the runtime components.
type robotV4RobotAxesConfig struct {
	// Axes are ordered by their live servo index (index 0 is the first live
	// axis). Every entry's Index must equal its position in this slice.
	Axes []robotV4RobotAxis `json:"axes"`
}

type robotV4RobotAxis struct {
	Index int    `json:"index"`
	Name  string `json:"name"`
	// URDFJoint is the joint name in the cell's URDF this axis drives.
	URDFJoint string `json:"urdf_joint"`
	// Kind is "arm" for the 6-DOF manipulator joints that the Cartesian
	// solver owns, or "external" for positioner/fixture axes (e.g. the
	// H-frame yaw J9 and positioner tables J7/J8).
	Kind string `json:"kind"`
	// These reject-only sentinels make stale manifests fail closed instead of
	// silently restoring a second model-configuration authority. Exact gear
	// ratios live only in the selected Jetson's persisted URDF revision.
	DeprecatedGearRatio       json.RawMessage `json:"gear_ratio,omitempty"`
	DeprecatedGearNumerator   json.RawMessage `json:"gear_numerator,omitempty"`
	DeprecatedGearDenominator json.RawMessage `json:"gear_denominator,omitempty"`
	// Sign is physical cell wiring, not a model configuration value.
	Sign float64 `json:"sign"`
	// CartesianParticipates is true only for axes the Cartesian TCP jog is
	// allowed to move. External axes set this false so TCP mode never drives
	// them; they move only in joint mode.
	CartesianParticipates bool `json:"cartesian_participates"`
}

type robotV4RuntimeConnectionsConfig struct {
	MotionServerUDPHost   string `json:"motion_server_udp_host"`
	MotionServerUDPPort   int    `json:"motion_server_udp_port"`
	MotionServerNATSURL   string `json:"motion_server_nats_url"`
	RealtimeCoreUDPTarget string `json:"realtime_core_udp_target"`
}

func (c robotV4RuntimeConnectionsConfig) normalized() robotV4RuntimeConnectionsConfig {
	c.MotionServerUDPHost = strings.TrimSpace(c.MotionServerUDPHost)
	c.MotionServerNATSURL = strings.TrimSpace(c.MotionServerNATSURL)
	c.RealtimeCoreUDPTarget = robotV4CanonicalUDPHostPort(c.RealtimeCoreUDPTarget)
	return c
}

// robotV4CanonicalUDPHostPort accepts the legacy unbracketed scoped-IPv6
// targets already present in deployed manifests and emits the unambiguous
// host:port form required by socket parsers and the motion-server CLI.
func robotV4CanonicalUDPHostPort(target string) string {
	target = strings.TrimSpace(target)
	if target == "" {
		return ""
	}
	if host, port, err := net.SplitHostPort(target); err == nil {
		return net.JoinHostPort(strings.Trim(strings.TrimSpace(host), "[]"), strings.TrimSpace(port))
	}
	colon := strings.LastIndex(target, ":")
	if colon <= 0 {
		return target
	}
	host := strings.TrimSpace(target[:colon])
	port := strings.TrimSpace(target[colon+1:])
	if strings.Contains(host, ":") && !strings.HasPrefix(host, "[") {
		return net.JoinHostPort(host, port)
	}
	return target
}

func (c robotV4RuntimeConnectionsConfig) validate(context string) error {
	c = c.normalized()
	if c.MotionServerUDPHost == "" {
		return fmt.Errorf("%s requires manifest runtime_connections.motion_server_udp_host", context)
	}
	if strings.Contains(c.MotionServerUDPHost, "daemon-v1://") {
		return fmt.Errorf("%s runtime_connections.motion_server_udp_host must be a direct tailnet/LAN host, got daemon ref %q", context, c.MotionServerUDPHost)
	}
	if robotV4ManifestIdentityIsSemantic(nil, c.MotionServerUDPHost) {
		return fmt.Errorf("%s runtime_connections.motion_server_udp_host must be a concrete peer/host, not role %q", context, c.MotionServerUDPHost)
	}
	if c.MotionServerUDPPort <= 0 || c.MotionServerUDPPort > 65535 {
		return fmt.Errorf("%s requires manifest runtime_connections.motion_server_udp_port in 1..65535", context)
	}
	if c.MotionServerNATSURL == "" {
		return fmt.Errorf("%s requires manifest runtime_connections.motion_server_nats_url", context)
	}
	if strings.Contains(c.MotionServerNATSURL, "daemon-v1://") || !strings.HasPrefix(c.MotionServerNATSURL, "nats://") {
		return fmt.Errorf("%s runtime_connections.motion_server_nats_url must be a direct nats:// URL, got %q", context, c.MotionServerNATSURL)
	}
	natsURL, err := url.Parse(c.MotionServerNATSURL)
	if err != nil || strings.TrimSpace(natsURL.Hostname()) == "" || robotV4ManifestIdentityIsSemantic(nil, natsURL.Hostname()) {
		return fmt.Errorf("%s runtime_connections.motion_server_nats_url must use a concrete peer/host, got %q", context, c.MotionServerNATSURL)
	}
	if c.RealtimeCoreUDPTarget == "" {
		return fmt.Errorf("%s requires manifest runtime_connections.realtime_core_udp_target", context)
	}
	if strings.Contains(c.RealtimeCoreUDPTarget, "daemon-v1://") {
		return fmt.Errorf("%s runtime_connections.realtime_core_udp_target must be a direct UDP host:port target, got daemon ref %q", context, c.RealtimeCoreUDPTarget)
	}
	if !strings.Contains(c.RealtimeCoreUDPTarget, ":") {
		return fmt.Errorf("%s runtime_connections.realtime_core_udp_target must be host:port, got %q", context, c.RealtimeCoreUDPTarget)
	}
	if robotV4ManifestIdentityIsSemantic(nil, c.realtimeCoreUDPHost()) {
		return fmt.Errorf("%s runtime_connections.realtime_core_udp_target must use a concrete peer/host, not role %q", context, c.RealtimeCoreUDPTarget)
	}
	rtcorePort := c.realtimeCoreUDPPort()
	if parsed, err := strconv.Atoi(rtcorePort); err != nil || parsed <= 0 || parsed > 65535 {
		return fmt.Errorf("%s runtime_connections.realtime_core_udp_target must end in a valid UDP port, got %q", context, c.RealtimeCoreUDPTarget)
	}
	return nil
}

func (c robotV4RuntimeConnectionsConfig) validateForManifest(context string, manifest robotV4DaemonNodeInventoryManifest) error {
	if err := c.validate(context); err != nil {
		return err
	}
	motion, err := robotV4ManifestUniqueRoleNode(manifest, robotV4MotionServerNode)
	if err != nil {
		return fmt.Errorf("%s: %w", context, err)
	}
	rtcore, err := robotV4ManifestUniqueRoleNode(manifest, robotV4RealtimeCoreNode)
	if err != nil {
		return fmt.Errorf("%s: %w", context, err)
	}
	parsedNATS, _ := url.Parse(c.MotionServerNATSURL)
	for field, host := range map[string]string{
		"motion_server_udp_host": c.MotionServerUDPHost,
		"motion_server_nats_url": parsedNATS.Hostname(),
	} {
		if !robotV4ManifestHostMatchesNode(manifest, motion, host) {
			return fmt.Errorf("%s runtime_connections.%s %q does not identify selected peer %s", context, field, host, robotV4DaemonIdentityLabel(motion, "runtime-connections"))
		}
	}
	if host := c.realtimeCoreUDPHost(); !robotV4ManifestHostMatchesNode(manifest, rtcore, host) {
		return fmt.Errorf("%s runtime_connections.realtime_core_udp_target host %q does not identify selected peer %s", context, host, robotV4DaemonIdentityLabel(rtcore, "runtime-connections"))
	}
	return nil
}

func (c robotV4RuntimeConnectionsConfig) realtimeCoreUDPPort() string {
	c = c.normalized()
	colon := strings.LastIndex(c.RealtimeCoreUDPTarget, ":")
	if colon < 0 {
		return ""
	}
	return strings.TrimSpace(c.RealtimeCoreUDPTarget[colon+1:])
}

func (c robotV4RuntimeConnectionsConfig) realtimeCoreUDPHost() string {
	c = c.normalized()
	if host, _, err := net.SplitHostPort(c.RealtimeCoreUDPTarget); err == nil {
		return strings.Trim(strings.TrimSpace(host), "[]")
	}
	colon := strings.LastIndex(c.RealtimeCoreUDPTarget, ":")
	if colon < 0 {
		return ""
	}
	return strings.Trim(strings.TrimSpace(c.RealtimeCoreUDPTarget[:colon]), "[]")
}

const (

// robotV4MaxLiveAxes is the compile-time wire/buffer ceiling shared with
// the runtime components (GRADIENT_MAX_AXES). Manifests may declare any
// live axis count from 1 up to this maximum.

)

type robotV4DaemonNodeInventoryNode struct {
	Runtime                  string                                 `json:"runtime,omitempty"`
	RTCore                   *robotV4RTCoreConfig                   `json:"rt_core,omitempty"`
	Node                     string                                 `json:"node"`
	Peer                     string                                 `json:"peer"`
	Role                     string                                 `json:"role"`
	HostRole                 string                                 `json:"host_role"`
	Host                     string                                 `json:"host"`
	SSHHost                  string                                 `json:"ssh_host"`
	ParentNode               string                                 `json:"parent_node"`
	DaemonLabel              string                                 `json:"daemon_label"`
	Aliases                  []string                               `json:"aliases"`
	LegacyAliases            []string                               `json:"legacy_aliases"`
	StaleMarkers             []string                               `json:"stale_markers"`
	PlatformOS               string                                 `json:"platform_os"`
	PlatformArch             string                                 `json:"platform_arch"`
	TargetTriple             string                                 `json:"target_triple"`
	TargetABI                string                                 `json:"target_abi"`
	RemoteUser               string                                 `json:"remote_user"`
	RosieHome                string                                 `json:"rosie_home"`
	DaemonIndexURL           string                                 `json:"daemon_index_url"`
	DaemonRuntimePath        string                                 `json:"daemon_runtime_path"`
	DaemonSelfRestart        string                                 `json:"daemon_self_restart_command"`
	DaemonStartupArgs        []string                               `json:"daemon_startup_args"`
	DaemonEnvironment        map[string]string                      `json:"daemon_environment"`
	DaemonService            robotV4DaemonServiceInventory          `json:"daemon_service"`
	DaemonProcessSupervision *bool                                  `json:"daemon_process_supervision"`
	IPv6Fallback             robotV4DaemonIPv6Fallback              `json:"ipv6_fallback"`
	AdvertisedAddrs          []string                               `json:"advertised_addresses"`
	RuntimePaths             []string                               `json:"runtime_paths"`
	ProcessIntents           map[string]robotV4ProcessIntentProcess `json:"process_intents"`
	ColdPathCommands         []robotV4ColdPathCommand               `json:"cold_path_commands"`
	ColdPathSubjects         []string                               `json:"cold_path_subjects"`
	WakeMAC                  string                                 `json:"wake_mac"`
	WakeAddresses            []string                               `json:"wake_broadcast_addresses"`
	BuildCachePolicy         robotV4BuildCachePolicy                `json:"build_cache_policy"`
	RevPiSetup               robotV4RevPiSetupConfig                `json:"revpi_setup"`
}

type robotV4RevPiSetupConfig struct {
	ViaNode               string                          `json:"via_node"`
	Interface             string                          `json:"interface"`
	ConnectionName        string                          `json:"connection_name"`
	JumpIPv4CIDR          string                          `json:"jump_ipv4_cidr"`
	JumpIPv4Method        string                          `json:"jump_ipv4_method"`
	CredentialPeer        string                          `json:"credential_peer"`
	ExpectedHostKeySHA256 string                          `json:"expected_host_key_sha256"`
	EtherCATFoundation    robotV4EtherCATFoundationConfig `json:"ethercat_foundation"`
}

type robotV4EtherCATFoundationConfig struct {
	Profile              string `json:"profile"`
	OSID                 string `json:"os_id"`
	OSVersion            string `json:"os_version"`
	Architecture         string `json:"architecture"`
	KernelRelease        string `json:"kernel_release"`
	IGHVersion           string `json:"igh_version"`
	IGHSourceURL         string `json:"igh_source_url"`
	IGHSourceSHA256      string `json:"igh_source_sha256"`
	EtherCATInterface    string `json:"ethercat_interface"`
	EtherCATMAC          string `json:"ethercat_mac"`
	EtherCATPermanentMAC string `json:"ethercat_permanent_mac"`
	UplinkInterface      string `json:"uplink_interface"`
	UplinkMAC            string `json:"uplink_mac"`
	UplinkPermanentMAC   string `json:"uplink_permanent_mac"`
	DeviceModules        string `json:"device_modules"`
	ExpectedSlaves       int    `json:"expected_slaves"`
	ReceiptPath          string `json:"receipt_path"`
}

type robotV4DaemonServiceInventory struct {
	Unit              string            `json:"unit"`
	FallbackUnits     []string          `json:"fallback_units"`
	DaemonEnvironment map[string]string `json:"daemon_environment"`
}

type robotV4DaemonIPv6Fallback struct {
	ParentNode                string `json:"parent_node"`
	ParentPeer                string `json:"parent_peer"`
	ParentDaemonURLFromTarget string `json:"parent_daemon_url_from_target"`
	ParentNATSURLFromTarget   string `json:"parent_nats_url_from_target"`
}

type robotV4ColdPathCommand struct {
	Name          string                       `json:"name"`
	Service       string                       `json:"service"`
	TargetNode    string                       `json:"target_node"`
	TargetPeer    string                       `json:"target_peer"`
	Commands      []string                     `json:"commands"`
	AdapterRef    string                       `json:"adapter_ref"`
	AdapterKind   string                       `json:"adapter_kind"`
	AdapterTarget robotV4ColdPathCommandTarget `json:"adapter_target"`
}

type robotV4ColdPathCommandTarget struct {
	Packet  string `json:"packet"`
	UDPAddr string `json:"udp_addr"`
}

type robotV4BuildCachePolicy struct {
	Mode                         string                         `json:"mode"`
	RuntimePeerBuild             *bool                          `json:"runtime_peer_build"`
	PreferredBuildPeers          []robotV4BuildCachePolicyPeer  `json:"preferred_build_peers"`
	RequiredRecordFields         []string                       `json:"required_record_fields"`
	TargetSysroot                robotV4BuildCacheTargetSysroot `json:"target_sysroot"`
	InstallOnlyAfterHashVerified bool                           `json:"install_only_after_hash_verified"`
}

type robotV4BuildCacheTargetSysroot struct {
	SourcePeer             string   `json:"source_peer"`
	Usage                  string   `json:"usage"`
	BuildPeerRoot          string   `json:"build_peer_root"`
	BuildPeerIncludeDir    string   `json:"build_peer_include_dir"`
	BuildPeerLibraryDir    string   `json:"build_peer_library_dir"`
	RuntimeLibraryDir      string   `json:"runtime_library_dir"`
	CFlags                 string   `json:"cflags"`
	Libs                   string   `json:"libs"`
	IncludeFiles           []string `json:"include_files"`
	LibraryFiles           []string `json:"library_files"`
	BuildPeerRequiredFiles []string `json:"build_peer_required_files"`
	// SyncPaths lets the deploy auto-provision the sysroot on the build
	// peer: each entry is rsynced from the source peer (source_peer node in
	// this manifest) into build_peer_root when the required files are
	// missing. Filters are raw rsync --include/--exclude arguments.
	SyncPaths []robotV4SysrootSyncPath `json:"sync_paths"`
	// PostSyncCommands run on the build peer (cwd build_peer_root) after a
	// successful sync, e.g. to create merged-usr symlinks.
	PostSyncCommands []string `json:"post_sync_commands"`
	VerifiedAt       string   `json:"verified_at"`
}

type robotV4SysrootSyncPath struct {
	// SourcePath on the source peer (trailing slash rsyncs directory
	// contents), e.g. "/usr/include/".
	SourcePath string `json:"source_path"`
	// BuildPeerPath relative to build_peer_root, e.g. "usr/include/".
	BuildPeerPath string `json:"build_peer_path"`
	// Filters are rsync filter arguments applied in order, e.g.
	// ["--include=libc*", "--exclude=*"].
	Filters []string `json:"filters"`
}

type robotV4BuildCachePolicyPeer struct {
	Peer         string   `json:"peer"`
	Role         string   `json:"role"`
	PlatformOS   string   `json:"platform_os"`
	PlatformArch string   `json:"platform_arch"`
	TargetTriple string   `json:"target_triple"`
	TargetABI    string   `json:"target_abi"`
	Services     []string `json:"services"`
}

type robotV4BuildPeerInventory struct {
	Peer                string                        `json:"peer"`
	Role                string                        `json:"role"`
	HostRole            string                        `json:"host_role"`
	Host                string                        `json:"host"`
	SSHHost             string                        `json:"ssh_host"`
	RemoteUser          string                        `json:"remote_user"`
	SSHUser             string                        `json:"ssh_user"`
	WSLUser             string                        `json:"wsl_user"`
	WSLDistribution     string                        `json:"wsl_distribution"`
	WSLWorkingDirectory string                        `json:"wsl_working_directory"`
	RosieHome           string                        `json:"rosie_home"`
	IndexURL            string                        `json:"index_url"`
	DaemonIndexURL      string                        `json:"daemon_index_url"`
	DaemonRuntimePath   string                        `json:"daemon_runtime_path"`
	DaemonMetadataPath  string                        `json:"daemon_metadata_path"`
	DaemonStartupArgs   []string                      `json:"daemon_startup_args"`
	DaemonService       robotV4DaemonServiceInventory `json:"daemon_service"`
	RuntimePaths        []string                      `json:"runtime_paths"`
	PlatformOS          string                        `json:"platform_os"`
	PlatformArch        string                        `json:"platform_arch"`
	TargetTriple        string                        `json:"target_triple"`
	TargetABI           string                        `json:"target_abi"`
	BuildService        robotV4BuildServiceInventory  `json:"build_service"`
}

type robotV4BuildServiceInventory struct {
	Role        string `json:"role"`
	HostRuntime string `json:"host_runtime"`
	Toolchain   string `json:"toolchain"`
	CXX         string `json:"cxx"`
	CC          string `json:"cc"`
	// ArtifactBuildCommands overrides the artifact's own build command when
	// this build peer builds it, keyed by artifact name or kind. This is how
	// an artifact whose default build command is target-side (e.g. the
	// daemon's cargo-build-and-self-restart command) gets a build-only
	// cross-compile recipe on the build server.
	ArtifactBuildCommands map[string]string              `json:"artifact_build_commands"`
	RequiredTools         []string                       `json:"required_tools"`
	Services              []string                       `json:"services"`
	RequestEndpoint       string                         `json:"request_endpoint"`
	CacheProbeEndpoint    string                         `json:"cache_probe_endpoint"`
	BlobUploadEndpoint    string                         `json:"blob_upload_endpoint"`
	BlobProbeEndpoint     string                         `json:"blob_probe_endpoint"`
	BlobTicketEndpoint    string                         `json:"blob_ticket_endpoint"`
	RecordEndpoint        string                         `json:"record_endpoint"`
	StateField            string                         `json:"state_field"`
	ArtifactScope         []string                       `json:"artifact_scope"`
	TargetABI             string                         `json:"target_abi"`
	TargetSysroot         robotV4BuildCacheTargetSysroot `json:"target_sysroot"`
	CompatibilityStatus   string                         `json:"compatibility_status"`
	CompatibilityNote     string                         `json:"compatibility_note"`
	ActivationRequired    bool                           `json:"activation_required"`
	ActivationNote        string                         `json:"activation_note"`
}

type robotV4ManifestSelection struct {
	path     string
	loaded   bool
	manifest robotV4DaemonNodeInventoryManifest
	err      error
}

var robotV4CurrentManifestSelection *robotV4ManifestSelection

// robotV4LoadSelectedManifest snapshots a command's selection, including load
// failures, so repeated intent lookups cannot change runtime ownership.
func robotV4LoadSelectedManifest() (robotV4DaemonNodeInventoryManifest, error) {
	path := resolveRobotV4ManifestPath(selectedRobotV4ManifestPath())
	if path == "" {
		return robotV4DaemonNodeInventoryManifest{}, fmt.Errorf("no deployment manifest selected")
	}
	selection := robotV4CurrentManifestSelection
	if selection == nil || selection.path != path {
		selection = &robotV4ManifestSelection{path: path}
		robotV4CurrentManifestSelection = selection
	}
	if selection.loaded {
		return selection.manifest, selection.err
	}
	selection.loaded = true
	selection.manifest, selection.err = robotV4ReadInventoryManifest(path)
	return selection.manifest, selection.err
}

func robotV4ReadInventoryManifest(path string) (robotV4DaemonNodeInventoryManifest, error) {
	path = resolveRobotV4ManifestPath(path)
	raw, err := os.ReadFile(path)
	if err != nil {
		return robotV4DaemonNodeInventoryManifest{}, fmt.Errorf("load deployment manifest %s: %w", path, err)
	}
	var parsed robotV4DaemonNodeInventoryManifest
	if err := json.Unmarshal(raw, &parsed); err != nil {
		return robotV4DaemonNodeInventoryManifest{}, fmt.Errorf("decode deployment manifest %s: %w", path, err)
	}
	parsed.Name = strings.TrimSpace(parsed.Name)
	parsed.RobotCommandSubject = strings.TrimSpace(parsed.RobotCommandSubject)
	parsed.IndexURL = strings.TrimSpace(parsed.IndexURL)
	if err := robotV4ValidateManifestIdentityInvariant(parsed); err != nil {
		return robotV4DaemonNodeInventoryManifest{}, err
	}
	if len(parsed.Nodes) == 0 {
		return parsed, fmt.Errorf("deployment manifest %s has no nodes", path)
	}
	return parsed, nil
}

func loadRobotV4DaemonNodeInventoryManifest() (robotV4DaemonNodeInventoryManifest, bool) {
	manifest, err := robotV4LoadSelectedManifest()
	return manifest, err == nil
}

func selectedRobotV4ManifestPath() string {
	return firstNonEmpty(
		robotV4ManifestPathOverride,
		os.Getenv("ROBOT_V4_MANIFEST"),
		os.Getenv("ROBOT_V4_DEPLOYMENT_MANIFEST"),
	)
}

func withRobotV4ManifestPath(path string) func() {
	previous := robotV4ManifestPathOverride
	previousSelection := robotV4CurrentManifestSelection
	robotV4ManifestPathOverride = strings.TrimSpace(path)
	robotV4CurrentManifestSelection = &robotV4ManifestSelection{path: selectedRobotV4ManifestPath()}
	return func() {
		robotV4ManifestPathOverride = previous
		robotV4CurrentManifestSelection = previousSelection
	}
}

// robotV4CellManifestDir locates the single repository cell directory from either
// CLI copy and stops at a repository boundary, preventing stale parent selection.
func robotV4CellManifestDir(start string) string {
	start, err := filepath.Abs(start)
	if err != nil {
		return filepath.Join(start, "rt-core", "config", "cells")
	}
	for candidate := start; ; candidate = filepath.Dir(candidate) {
		dir := filepath.Join(candidate, "rt-core", "config", "cells")
		if info, err := os.Stat(dir); err == nil && info.IsDir() {
			return dir
		}
		if _, err := os.Stat(filepath.Join(candidate, ".git")); err == nil || filepath.Dir(candidate) == candidate {
			return dir
		}
	}
}

func resolveRobotV4ManifestPath(path string) string {
	path = strings.TrimSpace(path)
	if path == "" || filepath.IsAbs(path) {
		return path
	}
	if path == "rt-core/config/cells" || strings.HasPrefix(path, "rt-core/config/cells/") {
		if wd, err := os.Getwd(); err == nil {
			return filepath.Join(robotV4CellManifestDir(wd), strings.TrimPrefix(strings.TrimPrefix(path, "rt-core/config/cells"), "/"))
		}
	}
	if abs, err := filepath.Abs(path); err == nil && pathExists(abs) {
		return abs
	}
	if root, err := projectRoot(); err == nil {
		candidate := filepath.Join(root, filepath.FromSlash(path))
		if pathExists(candidate) {
			return candidate
		}
	}
	if repo, err := repoRoot(); err == nil {
		candidate := filepath.Join(repo, filepath.FromSlash(path))
		if pathExists(candidate) {
			return candidate
		}
	}
	return path
}

func robotV4DaemonNodeInventoryRecord(nodeName string) (robotV4DaemonNodeInventoryNode, bool) {
	nodeName = strings.ToLower(strings.TrimSpace(nodeName))
	if nodeName == "" {
		return robotV4DaemonNodeInventoryNode{}, false
	}
	manifest, ok := loadRobotV4DaemonNodeInventoryManifest()
	if !ok {
		return robotV4DaemonNodeInventoryNode{}, false
	}
	for _, node := range manifest.Nodes {
		if strings.EqualFold(strings.TrimSpace(node.Node), nodeName) {
			return node, true
		}
	}
	return robotV4DaemonNodeInventoryNode{}, false
}

func robotV4DaemonNodeInventoryRecordByAlias(alias string) (robotV4DaemonNodeInventoryNode, bool) {
	if node, ok := robotV4DaemonNodeInventoryRecord(alias); ok {
		return node, true
	}
	return robotV4DaemonNodeInventoryRecordForAlias(alias)
}

func robotV4DaemonNodeInventoryRecordForAlias(alias string) (robotV4DaemonNodeInventoryNode, bool) {
	alias = strings.ToLower(strings.TrimSpace(alias))
	if alias == "" {
		return robotV4DaemonNodeInventoryNode{}, false
	}
	manifest, ok := loadRobotV4DaemonNodeInventoryManifest()
	if !ok {
		return robotV4DaemonNodeInventoryNode{}, false
	}
	for _, node := range manifest.Nodes {
		for _, candidate := range robotV4DaemonNodeInventoryAliases(node) {
			if strings.EqualFold(strings.TrimSpace(candidate), alias) {
				return node, true
			}
		}
	}
	return robotV4DaemonNodeInventoryNode{}, false
}

func robotV4DaemonNodeInventoryAliases(node robotV4DaemonNodeInventoryNode) []string {
	aliases := []string{node.Node, node.Peer, node.Role, node.HostRole, node.Host, node.SSHHost, node.DaemonLabel}
	aliases = append(aliases, node.Aliases...)
	return nonEmptyStrings(aliases)
}

// robotV4DaemonIdentityLabel keeps the four identity dimensions visible and
// ordered consistently. Peer and host are concrete identities; role and
// service describe what that peer is doing. Callers must not substitute a
// semantic role (for example "realtime-core") for the peer or host fields.
func robotV4DaemonIdentityLabel(node robotV4DaemonNodeInventoryNode, service string) string {
	peer := firstNonEmpty(strings.TrimSpace(node.Peer), strings.TrimSpace(node.Host), "unknown-peer")
	host := firstNonEmpty(strings.TrimSpace(node.Host), strings.TrimSpace(node.SSHHost), "unknown-host")
	role := firstNonEmpty(strings.TrimSpace(node.Role), strings.TrimSpace(node.Node), "unknown-role")
	service = firstNonEmpty(strings.TrimSpace(service), "unknown-service")
	return fmt.Sprintf("%s(%s)/%s/%s", peer, host, role, service)
}

func robotV4ValidateManifestIdentityInvariant(manifest robotV4DaemonNodeInventoryManifest) error {
	seenControllerRoles := make(map[string]struct{}, len(manifest.LeaderControllerRoles))
	for index, rawRole := range manifest.LeaderControllerRoles {
		role := strings.TrimSpace(rawRole)
		if !robotV4ValidLeaderControllerRole(role) {
			return fmt.Errorf(
				"manifest leader_controller_roles[%d] %q is not a canonical controller role",
				index,
				rawRole,
			)
		}
		if _, duplicate := seenControllerRoles[role]; duplicate {
			return fmt.Errorf(
				"manifest leader_controller_roles contains duplicate role %q",
				role,
			)
		}
		seenControllerRoles[role] = struct{}{}
		manifest.LeaderControllerRoles[index] = role
	}
	semanticTokens := robotV4ManifestSemanticIdentityTokens(manifest)
	if name := strings.TrimSpace(manifest.Name); name != "" && robotV4ManifestIdentityIsSemantic(semanticTokens, name) {
		return fmt.Errorf("manifest name %q is a semantic role/placeholder, not a concrete robot peer", name)
	}
	seenPeers := map[string]bool{}
	for index, node := range manifest.Nodes {
		if err := node.validateRTCore(manifest); err != nil {
			return fmt.Errorf("manifest nodes[%d]: %w", index, err)
		}
		peer := strings.TrimSpace(node.Peer)
		if peer == "" {
			return fmt.Errorf("manifest nodes[%d] is missing its concrete peer ID", index)
		}
		if !robotV4PlainConcreteIdentitySyntax(peer) {
			return fmt.Errorf("manifest identity %s uses transport-decorated/non-concrete peer ID %q", robotV4DaemonIdentityLabel(node, "manifest"), peer)
		}
		if robotV4ManifestIdentityIsSemantic(semanticTokens, peer) {
			return fmt.Errorf("manifest identity %s uses semantic role/node %q as its peer ID", robotV4DaemonIdentityLabel(node, "manifest"), peer)
		}
		peerKey := strings.ToLower(peer)
		if seenPeers[peerKey] {
			return fmt.Errorf("manifest concrete peer ID %q is duplicated", peer)
		}
		seenPeers[peerKey] = true
		for field, value := range map[string]string{"host": node.Host, "ssh_host": node.SSHHost} {
			value = strings.TrimSpace(value)
			if value != "" && !robotV4PlainConcreteIdentitySyntax(value) {
				return fmt.Errorf("manifest identity %s uses transport-decorated/non-concrete %s %q", robotV4DaemonIdentityLabel(node, "manifest"), field, value)
			}
			if value != "" && robotV4ManifestIdentityIsSemantic(semanticTokens, value) {
				return fmt.Errorf("manifest identity %s uses semantic role/node %q as %s", robotV4DaemonIdentityLabel(node, "manifest"), value, field)
			}
		}
		for _, address := range node.AdvertisedAddrs {
			address = strings.TrimSpace(address)
			if address != "" && !robotV4PlainConcreteIdentitySyntax(address) {
				return fmt.Errorf("manifest identity %s uses transport-decorated/non-concrete advertised address %q", robotV4DaemonIdentityLabel(node, "manifest"), address)
			}
			if address != "" && robotV4ManifestIdentityIsSemantic(semanticTokens, address) {
				return fmt.Errorf("manifest identity %s uses semantic role/node %q as advertised address", robotV4DaemonIdentityLabel(node, "manifest"), address)
			}
		}
	}
	for index, peer := range manifest.BuildPeers {
		concretePeer := strings.TrimSpace(peer.Peer)
		if concretePeer == "" {
			return fmt.Errorf("manifest build_peers[%d] is missing its concrete peer ID", index)
		}
		for field, value := range map[string]string{"peer": peer.Peer, "host": peer.Host, "ssh_host": peer.SSHHost} {
			value = strings.TrimSpace(value)
			if value == "" {
				continue
			}
			if !robotV4PlainConcreteIdentitySyntax(value) {
				return fmt.Errorf("manifest build peer %q uses transport-decorated/non-concrete %s %q", concretePeer, field, value)
			}
			if robotV4ManifestIdentityIsSemantic(semanticTokens, value) {
				return fmt.Errorf("manifest build peer %q uses semantic role %q as %s", concretePeer, value, field)
			}
		}
	}
	for _, node := range manifest.Nodes {
		for _, preferred := range node.BuildCachePolicy.PreferredBuildPeers {
			if !robotV4ManifestContainsConcretePeer(manifest, preferred.Peer) {
				return fmt.Errorf("manifest identity %s build cache uses non-concrete preferred build peer %q", robotV4DaemonIdentityLabel(node, "build-cache"), preferred.Peer)
			}
		}
		if sourcePeer := strings.TrimSpace(node.BuildCachePolicy.TargetSysroot.SourcePeer); sourcePeer != "" && !robotV4ManifestContainsConcretePeer(manifest, sourcePeer) {
			return fmt.Errorf("manifest identity %s build cache sysroot uses non-concrete source_peer %q", robotV4DaemonIdentityLabel(node, "build-cache"), sourcePeer)
		}
	}
	for _, buildPeer := range manifest.BuildPeers {
		if sourcePeer := strings.TrimSpace(buildPeer.BuildService.TargetSysroot.SourcePeer); sourcePeer != "" && !robotV4ManifestContainsConcretePeer(manifest, sourcePeer) {
			return fmt.Errorf("manifest build peer %q sysroot uses non-concrete source_peer %q", buildPeer.Peer, sourcePeer)
		}
	}
	for _, node := range manifest.Nodes {
		environment, err := robotV4DaemonEnvironmentFromManifest(node)
		if err != nil {
			return err
		}
		for _, target := range robotV4DaemonRuntimeCSV(environment, "ROBOT_V4_RTCORE_COMMAND_TARGET_NODES") {
			if _, ok := robotV4ManifestNodeByConcreteIdentity(manifest, target); !ok {
				return fmt.Errorf("manifest identity %s publishes non-concrete robot command target %q", robotV4DaemonIdentityLabel(node, "robot-command-adapter"), strings.TrimSpace(target))
			}
		}
		for _, command := range node.ColdPathCommands {
			targetNode, nodeOK := robotV4ManifestNodeByConcreteIdentity(manifest, command.TargetNode)
			targetPeer, peerOK := robotV4ManifestNodeByConcreteIdentity(manifest, command.TargetPeer)
			if strings.TrimSpace(command.TargetNode) != "" && !nodeOK {
				return fmt.Errorf("manifest identity %s cold-path command %q uses non-concrete target_node %q", robotV4DaemonIdentityLabel(node, command.Service), command.Name, command.TargetNode)
			}
			if strings.TrimSpace(command.TargetPeer) != "" && !peerOK {
				return fmt.Errorf("manifest identity %s cold-path command %q uses non-concrete target_peer %q", robotV4DaemonIdentityLabel(node, command.Service), command.Name, command.TargetPeer)
			}
			if nodeOK && peerOK && !strings.EqualFold(targetNode.Peer, targetPeer.Peer) {
				return fmt.Errorf("manifest identity %s cold-path command %q target_node and target_peer resolve to different peers", robotV4DaemonIdentityLabel(node, command.Service), command.Name)
			}
		}
	}
	connections := manifest.RuntimeConnections.normalized()
	if connections.MotionServerUDPHost != "" || connections.MotionServerNATSURL != "" || connections.RealtimeCoreUDPTarget != "" || connections.MotionServerUDPPort != 0 {
		if err := connections.validateForManifest("deployment manifest", manifest); err != nil {
			return err
		}
	}
	if name := strings.TrimSpace(manifest.Name); name != "" {
		motion, err := robotV4ManifestUniqueRoleNode(manifest, robotV4MotionServerNode)
		if err == nil && !robotV4ManifestHostMatchesNode(manifest, motion, name) {
			return fmt.Errorf("manifest name %q does not identify selected motion-server peer %s", name, robotV4DaemonIdentityLabel(motion, "manifest"))
		}
		if subject := strings.TrimSpace(manifest.RobotCommandSubject); subject != "" && subject != "robot/v4/robot."+name+".command" {
			return fmt.Errorf("manifest robot_command_subject %q must use concrete manifest peer %q", subject, name)
		}
	}
	if err := robotV4ValidateManifestWireURLAuthorities(manifest, semanticTokens); err != nil {
		return err
	}
	return robotV4ValidateManifestDaemonServiceRefPeers(manifest)
}

func robotV4ValidLeaderControllerRole(role string) bool {
	if len(role) == 0 || len(role) > 31 || role[0] < 'a' || role[0] > 'z' {
		return false
	}
	for _, char := range role {
		if (char < 'a' || char > 'z') &&
			(char < '0' || char > '9') &&
			char != '-' {
			return false
		}
	}
	return true
}

func robotV4ValidateManifestWireURLAuthorities(manifest robotV4DaemonNodeInventoryManifest, semanticTokens map[string]bool) error {
	raw, err := json.Marshal(manifest)
	if err != nil {
		return fmt.Errorf("marshal manifest for URL identity validation: %w", err)
	}
	text := string(raw)
	for offset := 0; ; {
		marker := strings.Index(text[offset:], "://")
		if marker < 0 {
			break
		}
		authorityStart := offset + marker + len("://")
		authorityEnd := authorityStart
		for authorityEnd < len(text) && !strings.ContainsRune("/\\?#,;\"'() \t\r\n", rune(text[authorityEnd])) {
			authorityEnd++
		}
		authority := text[authorityStart:authorityEnd]
		parsedAuthority, parseErr := url.Parse("identity://" + authority)
		if parseErr != nil || parsedAuthority.User != nil || strings.TrimSpace(parsedAuthority.Hostname()) == "" {
			return fmt.Errorf("manifest URL authority %q is transport-decorated/malformed; expected a concrete peer(host)", authority)
		}
		host := strings.TrimSpace(parsedAuthority.Hostname())
		if robotV4ManifestIdentityIsSemantic(semanticTokens, host) {
			return fmt.Errorf("manifest URL authority %q uses semantic role/placeholder %q instead of a concrete peer(host)", authority, host)
		}
		offset = authorityEnd
		if offset >= len(text) {
			break
		}
	}
	for _, prefix := range []string{"robot/v4/rtcore.", "robot/v4/motion-server.", "robot/v4/robot."} {
		lower := strings.ToLower(text)
		for offset := 0; ; {
			index := strings.Index(lower[offset:], prefix)
			if index < 0 {
				break
			}
			start := offset + index + len(prefix)
			end := start
			for end < len(text) && !strings.ContainsRune("./\\?#,;\"'() \t\r\n", rune(text[end])) {
				end++
			}
			host := text[start:end]
			if !robotV4PlainConcreteIdentitySyntax(host) {
				return fmt.Errorf("manifest subject uses transport-decorated/non-concrete identity %q in its peer(host) slot", host)
			}
			if robotV4ManifestIdentityIsSemantic(semanticTokens, host) {
				return fmt.Errorf("manifest subject uses semantic role/placeholder %q in its peer(host) slot", host)
			}
			offset = end
			if offset >= len(text) {
				break
			}
		}
	}
	return nil
}

// robotV4ManifestSemanticIdentityTokens builds one manifest-wide namespace of
// logical selectors. A transport identity is invalid when it equals any role,
// even when that role belongs to a different node or build peer.
func robotV4ManifestSemanticIdentityTokens(manifest robotV4DaemonNodeInventoryManifest) map[string]bool {
	tokens := map[string]bool{}
	for token := range robotV4ReservedSemanticIdentityTokens {
		tokens[token] = true
	}
	for _, node := range manifest.Nodes {
		for _, value := range []string{node.Role, node.HostRole} {
			if value = strings.ToLower(strings.TrimSpace(value)); value != "" {
				tokens[value] = true
			}
		}
		nodeSelector := strings.TrimSpace(node.Node)
		if nodeSelector != "" &&
			!strings.EqualFold(nodeSelector, strings.TrimSpace(node.Peer)) &&
			!strings.EqualFold(nodeSelector, strings.TrimSpace(node.Host)) &&
			!strings.EqualFold(nodeSelector, strings.TrimSpace(node.SSHHost)) {
			tokens[strings.ToLower(nodeSelector)] = true
		}
	}
	for _, peer := range manifest.BuildPeers {
		for _, value := range []string{peer.Role, peer.HostRole, peer.BuildService.Role} {
			if value = strings.ToLower(strings.TrimSpace(value)); value != "" {
				tokens[value] = true
			}
		}
	}
	return tokens
}

func robotV4ManifestIdentityIsSemantic(tokens map[string]bool, value string) bool {
	value = robotV4TransportIdentityToken(value)
	return tokens[strings.ToLower(strings.TrimSpace(value))] || robotV4ReservedSemanticIdentityTokens[strings.ToLower(strings.TrimSpace(value))]
}

// These identities are semantic even when a malformed manifest omits or
// corrupts its role fields. Keeping the reserved set independent of manifest
// data prevents circular validation (for example peer=motion-server with an
// empty role) from promoting a role into a peer/host slot.
var robotV4ReservedSemanticIdentityTokens = map[string]bool{
	"manifest":                        true,
	"service":                         true,
	"motion-server":                   true,
	"realtime-core":                   true,
	"rt-core":                         true,
	"rt-control":                      true,
	"steamdeck-teach-pendent":         true,
	"build-server":                    true,
	"motion-authority":                true,
	"servo-authority":                 true,
	"human-client-ui":                 true,
	"os-arch-build-server":            true,
	"builder-dashboard-artifact-node": true,
}

func robotV4TransportIdentityToken(value string) string {
	value = strings.TrimSpace(value)
	lower := strings.ToLower(value)
	for _, prefix := range []string{"local:", "iroh-gossip:"} {
		if strings.HasPrefix(lower, prefix) {
			value = value[len(prefix):]
			break
		}
	}
	if parsed, err := url.Parse(value); err == nil && parsed.Hostname() != "" {
		value = parsed.Hostname()
	}
	if separator := strings.LastIndex(value, "@"); separator >= 0 {
		value = value[separator+1:]
	}
	if host, _, err := net.SplitHostPort(value); err == nil {
		value = host
	} else if strings.Count(value, ":") == 1 {
		value = strings.SplitN(value, ":", 2)[0]
	}
	value = strings.ToLower(strings.Trim(strings.TrimSpace(value), "[]\"'"))
	value = strings.TrimRight(value, ".")
	return strings.TrimSuffix(value, ".local")
}

func robotV4PlainConcreteIdentitySyntax(value string) bool {
	value = strings.TrimSpace(value)
	if value == "" || strings.HasPrefix(value, "-") || strings.Contains(value, "://") ||
		strings.ContainsAny(value, "@/?#=,\\\"' \t\r\n") {
		return false
	}
	lower := strings.ToLower(value)
	if strings.HasPrefix(lower, "local:") || strings.HasPrefix(lower, "iroh-gossip:") {
		return false
	}
	if _, _, err := net.SplitHostPort(value); err == nil || strings.Count(value, ":") == 1 {
		return false
	}
	if strings.Contains(value, ":") {
		candidate := strings.Trim(value, "[]")
		if zone := strings.LastIndex(candidate, "%"); zone >= 0 {
			candidate = candidate[:zone]
		}
		return net.ParseIP(candidate) != nil
	}
	return true
}

func robotV4ManifestUniqueRoleNode(manifest robotV4DaemonNodeInventoryManifest, role string) (robotV4DaemonNodeInventoryNode, error) {
	role = strings.TrimSpace(role)
	var selected robotV4DaemonNodeInventoryNode
	found := false
	for _, node := range manifest.Nodes {
		if !strings.EqualFold(strings.TrimSpace(node.Role), role) {
			continue
		}
		if found {
			return robotV4DaemonNodeInventoryNode{}, fmt.Errorf("deployment manifest role %q is assigned to multiple peers", role)
		}
		selected = node
		found = true
	}
	if !found {
		return robotV4DaemonNodeInventoryNode{}, fmt.Errorf("deployment manifest is missing unique role %q", role)
	}
	return selected, nil
}

func robotV4ManifestHostMatchesNode(manifest robotV4DaemonNodeInventoryManifest, node robotV4DaemonNodeInventoryNode, host string) bool {
	host = strings.TrimSpace(host)
	if host == "" || robotV4ManifestIdentityIsSemantic(robotV4ManifestSemanticIdentityTokens(manifest), host) {
		return false
	}
	explicit := []string{node.Peer, node.Host, node.SSHHost}
	candidates := append(robotV4DaemonConcreteIdentityAliases(node), node.AdvertisedAddrs...)
	wanted := robotV4ComparableHostIdentity(host)
	for _, candidate := range candidates {
		candidate = strings.TrimSpace(candidate)
		if candidate == "" || robotV4ManifestIdentityIsSemantic(robotV4ManifestSemanticIdentityTokens(manifest), candidate) ||
			!robotV4DaemonConcreteIdentityCandidate(explicit, candidate) {
			continue
		}
		if strings.EqualFold(wanted, robotV4ComparableHostIdentity(candidate)) {
			return true
		}
	}
	return false
}

func robotV4ComparableHostIdentity(value string) string {
	value = strings.ToLower(robotV4TransportIdentityToken(value))
	if zone := strings.LastIndex(value, "%"); zone >= 0 {
		value = value[:zone]
	}
	return strings.Trim(value, "[]")
}

func robotV4ValidateManifestDaemonServiceRefPeers(manifest robotV4DaemonNodeInventoryManifest) error {
	raw, err := json.Marshal(manifest)
	if err != nil {
		return fmt.Errorf("marshal manifest for daemon-v1 identity validation: %w", err)
	}
	text := string(raw)
	for offset := 0; ; {
		index := strings.Index(text[offset:], "daemon-v1://")
		if index < 0 {
			return nil
		}
		start := offset + index
		end := start
		for end < len(text) && (isRobotV4DaemonV1RefChar(text[end]) || text[end] == ':') {
			end++
		}
		ref := text[start:end]
		parts := strings.Split(strings.TrimPrefix(ref, "daemon-v1://"), "/")
		if len(parts) != 3 || strings.TrimSpace(parts[0]) == "" || strings.TrimSpace(parts[1]) == "" || strings.TrimSpace(parts[2]) == "" {
			return fmt.Errorf("manifest contains malformed daemon-v1 service reference %q", ref)
		}
		peer := strings.TrimSpace(parts[0])
		if !robotV4ManifestContainsConcretePeer(manifest, peer) {
			return fmt.Errorf("manifest daemon-v1 service reference %q uses unknown/non-concrete peer %q", ref, peer)
		}
		offset = end
	}
}

func robotV4ManifestContainsConcretePeer(manifest robotV4DaemonNodeInventoryManifest, peer string) bool {
	for _, node := range manifest.Nodes {
		for _, candidate := range robotV4DaemonConcreteIdentityAliases(node) {
			if strings.EqualFold(strings.TrimSpace(candidate), strings.TrimSpace(peer)) {
				return true
			}
		}
	}
	for _, buildPeer := range manifest.BuildPeers {
		for _, candidate := range []string{buildPeer.Peer, buildPeer.Host, buildPeer.SSHHost} {
			if strings.TrimSpace(candidate) != "" && strings.EqualFold(strings.TrimSpace(candidate), strings.TrimSpace(peer)) {
				return true
			}
		}
	}
	return false
}

func robotV4ManifestNodeByConcreteIdentity(manifest robotV4DaemonNodeInventoryManifest, identity string) (robotV4DaemonNodeInventoryNode, bool) {
	identity = strings.TrimSpace(identity)
	if identity == "" {
		return robotV4DaemonNodeInventoryNode{}, false
	}
	for _, node := range manifest.Nodes {
		for _, concrete := range robotV4DaemonConcreteIdentityAliases(node) {
			if strings.EqualFold(identity, strings.TrimSpace(concrete)) {
				return node, true
			}
		}
	}
	return robotV4DaemonNodeInventoryNode{}, false
}
