package main

import (
	"bytes"

	"encoding/base64"

	"encoding/json"

	"flag"
	"fmt"
	"io"

	"net/http"
	"net/url"

	"os"
	"os/exec"

	"regexp"
	"sort"

	"strings"
)

func containsString(items []string, needle string) bool {
	for _, item := range items {
		if item == needle {
			return true
		}
	}
	return false
}

func postDaemonBlob(client *http.Client, daemonURL string, raw []byte, sha256Value string) (daemonBlobUploadResponse, error) {
	return postDaemonBlobWithPaths(client, daemonURL, raw, sha256Value, "/api/artifacts/blob", "/api/artifacts/blob")
}

func postDaemonBlobWithPaths(client *http.Client, daemonURL string, raw []byte, sha256Value, uploadPath, probePath string) (daemonBlobUploadResponse, error) {
	cacheHit, err := daemonBlobCacheContains(client, daemonURL, probePath, sha256Value)
	if err != nil {
		return daemonBlobUploadResponse{}, err
	}
	if cacheHit {
		return daemonBlobUploadResponse{
			OK:       true,
			SHA256:   sha256Value,
			URI:      "iroh-blobs://" + sha256Value,
			CacheHit: true,
		}, nil
	}
	body, err := json.Marshal(map[string]string{
		"sha256":         sha256Value,
		"payload_base64": base64.StdEncoding.EncodeToString(raw),
	})
	if err != nil {
		return daemonBlobUploadResponse{}, fmt.Errorf("marshal daemon blob upload: %w", err)
	}
	response, err := postDaemonJSON(client, strings.TrimRight(daemonURL, "/")+uploadPath, body)
	if err != nil {
		return daemonBlobUploadResponse{}, err
	}
	var uploaded daemonBlobUploadResponse
	if err := json.Unmarshal(response, &uploaded); err != nil {
		return daemonBlobUploadResponse{}, fmt.Errorf("parse daemon blob upload response: %w: %s", err, response)
	}
	if !uploaded.OK || uploaded.SHA256 != sha256Value || uploaded.URI == "" {
		return daemonBlobUploadResponse{}, fmt.Errorf("daemon blob upload returned unexpected response: %s", response)
	}
	return uploaded, nil
}

func daemonBlobCacheContains(client *http.Client, daemonURL, probePath, sha256Value string) (bool, error) {
	endpoint := strings.TrimRight(strings.TrimSpace(daemonURL), "/") + probePath +
		"?sha256=" + url.QueryEscape(strings.TrimSpace(sha256Value))
	if err := robotV4ValidateOutboundDaemonHTTPURL(endpoint); err != nil {
		return false, err
	}
	request, err := http.NewRequest(http.MethodHead, endpoint, nil)
	if err != nil {
		return false, fmt.Errorf("prepare daemon blob cache probe: %w", err)
	}
	response, err := client.Do(request)
	if err != nil {
		return false, fmt.Errorf("HEAD %s: %w", endpoint, err)
	}
	defer response.Body.Close()
	switch response.StatusCode {
	case http.StatusOK:
		return true, nil
	case http.StatusNotFound, http.StatusMethodNotAllowed, http.StatusNotImplemented:
		// A 404 is a cache miss. 405/501 are older daemons which predate the
		// probe endpoint; fall back to the idempotent upload for compatibility.
		return false, nil
	default:
		raw, _ := io.ReadAll(io.LimitReader(response.Body, 4096))
		return false, fmt.Errorf("HEAD %s: HTTP %s: %s", endpoint, response.Status, strings.TrimSpace(string(raw)))
	}
}

func getDaemonJSON(client *http.Client, url string) ([]byte, error) {
	if err := robotV4ValidateOutboundDaemonHTTPURL(url); err != nil {
		return nil, err
	}
	response, err := client.Get(url)
	if err != nil {
		return nil, fmt.Errorf("GET %s: %w", url, err)
	}
	defer response.Body.Close()
	raw, err := io.ReadAll(response.Body)
	if err != nil {
		return nil, fmt.Errorf("read GET %s response: %w", url, err)
	}
	if response.StatusCode < 200 || response.StatusCode >= 300 {
		return nil, fmt.Errorf("GET %s: HTTP %s: %s", url, response.Status, strings.TrimSpace(string(raw)))
	}
	return raw, nil
}

func postDaemonJSON(client *http.Client, url string, body []byte) ([]byte, error) {
	if err := robotV4ValidateOutboundDaemonHTTPURL(url); err != nil {
		return nil, err
	}
	request, err := http.NewRequest(http.MethodPost, url, bytes.NewReader(body))
	if err != nil {
		return nil, fmt.Errorf("prepare POST %s: %w", url, err)
	}
	request.Header.Set("Content-Type", "application/json")
	if err := robotV4AttachDaemonWriteAuthorization(request); err != nil {
		return nil, err
	}
	authorization := request.Header.Get("Authorization")
	response, err := robotV4DaemonMutationClient(client).Do(request)
	if err != nil {
		return nil, fmt.Errorf("POST %s: %s", url, robotV4RedactDaemonWriteAuthorization(err.Error(), authorization))
	}
	defer response.Body.Close()
	raw, err := io.ReadAll(response.Body)
	if err != nil {
		return nil, fmt.Errorf("read POST %s response: %w", url, err)
	}
	if response.StatusCode < 200 || response.StatusCode >= 300 {
		responseText := robotV4RedactDaemonWriteAuthorization(strings.TrimSpace(string(raw)), authorization)
		return nil, fmt.Errorf("POST %s: HTTP %s: %s", url, response.Status, responseText)
	}
	return raw, nil
}

func robotV4ValidateOutboundDaemonHTTPURL(rawURL string) error {
	parsed, err := url.Parse(strings.TrimSpace(rawURL))
	if err != nil || (parsed.Scheme != "http" && parsed.Scheme != "https") || strings.TrimSpace(parsed.Hostname()) == "" {
		return fmt.Errorf("daemon request URL must be an absolute http(s) URL with a concrete peer(host), got %q", rawURL)
	}
	if parsed.User != nil {
		return fmt.Errorf("daemon request URL must not put userinfo in the peer(host) authority, got %q", rawURL)
	}
	if robotV4ManifestIdentityIsSemantic(nil, parsed.Hostname()) {
		return fmt.Errorf("daemon request URL %q uses semantic role/placeholder %q instead of a concrete peer(host)", rawURL, parsed.Hostname())
	}
	return nil
}

func runRealtimeCore(args []string, stdout, stderr io.Writer) error {
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, `usage: rosie steamdeck real v4 realtime-core <command> [flags]

Commands:
  deploy  Install the cell's native package installed-disarmed (no bus owner)
          --package rosie-rt-core.tar.gz (from realtime-core build --fetch-package)
          [--expect-package-sha256 HASH (registered candidate digest)]
          [--manifest cell.json] [--dry-run] [--transport tailscale|ssh]
          [--maintenance-eta 45m] [--idle-wait 2m] [--keep-maintenance]
          [--legacy-build-peer: build on the manifest's build peer with
          tools/build-rt-core-package.sh instead of --package]
  start   Start an installed-disarmed controller and verify it is the cell pin,
          every axis verified, disarmed [--manifest cell.json] [--dry-run]
          [--transport tailscale|ssh] [--maintenance-eta 5m] [--keep-maintenance]
  config  Configuration-only deploy: compile the cell's machine JSON with the
          installed release's rtctl, restart only rosie-rt-core/rosie-rt-control,
          verify or roll back [--manifest cell.json] [--dry-run]
          [--accept-configuration sha256] [--olp restart|skip]
          [--ready-timeout 60s] [--home-timeout 45s] [--transport tailscale|ssh]
          [--maintenance-eta 2m] [--idle-wait 2m] [--keep-maintenance]
  maintenance
          Set, clear or show the update notice on the cell's OLP host
          set|clear|show [--manifest cell.json] [--reason TEXT] [--eta 30m]
  timings Deploy history from ~/.rosie/deploy-journal: per run total, machine
          time and phases; then the slowest phases and test suites
          [--last 20] [--cell id] [--command text] [--top 12]

deploy, start and config put up the maintenance notice on the relay's OLP
before they wait for the cell to go idle (every OLP browser and Steam Deck
shows a blocking "Update in progress" overlay; robot actions are refused,
Stop/Disarm/Disconnect still work) and clear it when they end, failures
included, unless --keep-maintenance.

`+cellDeployRealtimeCoreUsage+`
Examples:
  go run . realtime-core deploy --manifest rt-core/config/cells/<cell>.json --dry-run
  go run . realtime-core start --manifest rt-core/config/cells/<cell>.json
  go run . realtime-core config --manifest rt-core/config/cells/<cell>.json --dry-run
  go run . realtime-core build --manifest rt-core/config/cells/<cell>.json --test
  go run . olp install --manifest rt-core/config/cells/<cell>.json --deployment <commit>
`)
		return nil
	}
	if containsString(cellDeployRealtimeCoreCommands, args[0]) {
		return runRealtimeCoreCell(args[0], args[1:], stdout, stderr)
	}
	switch args[0] {
	case "deploy":
		return runRealtimeCoreDeploy(args[1:], stdout, stderr)
	case "start":
		return runRealtimeCoreStart(args[1:], stdout, stderr)
	case "config":
		return runRealtimeCoreConfig(args[1:], stdout, stderr)
	case "maintenance":
		return runRealtimeCoreMaintenance(args[1:], stdout, stderr)
	case "timings":
		return runRealtimeCoreTimings(args[1:], stdout, stderr)
	default:
		return fmt.Errorf("unknown realtime-core command %q", args[0])
	}
}

type realtimeCoreDeployOptions struct {
	Host              string
	ManifestPath      string
	AxisEnv           []string
	BridgeEnv         []string
	RemoteUser        string
	RosieHome         string
	DaemonIndexURL    string
	MeshIndexURL      string
	DaemonNode        string
	DaemonStartupArgs []string
	DaemonEnvironment []string
	DaemonStateURL    string
	NATSURL           string
	NATSHost          string
	RTCoreUDPPort     string
	ViaHost           string
	ViaUser           string
	RemoteRepo        string
	RTCorePrebuilt    string
	BridgePrebuilt    string
	DaemonPrebuilt    string
	ReadySimulation   bool
	DryRun            bool
}

func runRealtimeCoreDeploy(args []string, stdout, stderr io.Writer) error {
	fs := flag.NewFlagSet("realtime-core deploy", flag.ContinueOnError)
	fs.SetOutput(stderr)
	manifestPath := fs.String("manifest", "", "approved native cell config")
	dryRun := fs.Bool("dry-run", false, "print the verified deploy plan")
	packagePath := fs.String("package", "", "live rosie-rt-core package built on the cell's relay from this clean checkout's HEAD (realtime-core build --fetch-package)")
	expectedPackageSHA256 := fs.String("expect-package-sha256", "", "exact package digest from a verified release candidate")
	pairMigrationFrom := fs.String("pair-migration-from", "", "old pair ID for an explicit stopped controller and OLP credential migration")
	legacy := fs.Bool("legacy-build-peer", false, "build the package on the manifest's build peer with steamdeck/real/v4/tools/build-rt-core-package.sh (the path before the relay-native build; kept while installed cells may need it)")
	transport := fs.String("transport", "tailscale", "how the Mac reaches the relay: tailscale or ssh")
	var maintenance rtCoreMaintenanceFlags
	maintenance.register(fs, true)
	if err := rejectRetiredRealtimeCoreFlags(args, "--manifest", "--dry-run", "--package", "--expect-package-sha256", "--pair-migration-from", "--legacy-build-peer", "--transport", "--maintenance-eta", "--idle-wait", "--keep-maintenance"); err != nil {
		return err
	}
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() != 0 {
		return &retiredRuntimeError{Selection: "realtime-core deploy arguments"}
	}
	if err := maintenance.validate(); err != nil {
		return err
	}
	if err := rtCoreDeployPackageSource(*packagePath, *legacy, *dryRun); err != nil {
		return err
	}
	if *expectedPackageSHA256 != "" {
		if *packagePath == "" || !rtCoreHexSHA256.MatchString(*expectedPackageSHA256) {
			return fmt.Errorf("--expect-package-sha256 needs --package and a lowercase 64-hex digest")
		}
	}
	restore, err := selectRealtimeCoreManifest(*manifestPath)
	if err != nil {
		return err
	}
	defer restore()
	if *pairMigrationFrom != "" && !regexp.MustCompile(`^[A-Za-z0-9][A-Za-z0-9_.-]*$`).MatchString(*pairMigrationFrom) {
		return fmt.Errorf("--pair-migration-from must name a valid installed pair")
	}
	if *pairMigrationFrom != "" && !*dryRun && os.Getenv(cellLockEnv) == "" {
		return fmt.Errorf("pair migration is only available inside a locked realtime-core release")
	}
	return runRTCoreDeployWithExpectedPair(*dryRun, *packagePath, *expectedPackageSHA256, *transport, *pairMigrationFrom, maintenance, stdout, stderr)
}

func runRealtimeCoreStart(args []string, stdout, stderr io.Writer) error {
	fs := flag.NewFlagSet("realtime-core start", flag.ContinueOnError)
	fs.SetOutput(stderr)
	manifestPath := fs.String("manifest", "", "approved native cell config")
	dryRun := fs.Bool("dry-run", false, "print the start script without contacting the cell")
	transport := fs.String("transport", "tailscale", "how the Mac reaches the relay: tailscale or ssh")
	var maintenance rtCoreMaintenanceFlags
	maintenance.register(fs, false)
	if err := rejectRetiredRealtimeCoreFlags(args, "--manifest", "--dry-run", "--transport", "--maintenance-eta", "--keep-maintenance"); err != nil {
		return err
	}
	if err := fs.Parse(args); err != nil {
		return err
	}
	if fs.NArg() != 0 {
		return &retiredRuntimeError{Selection: "realtime-core start arguments"}
	}
	if err := maintenance.validate(); err != nil {
		return err
	}
	restore, err := selectRealtimeCoreManifest(*manifestPath)
	if err != nil {
		return err
	}
	defer restore()
	return runRTCoreStartWith(*dryRun, *transport, maintenance, stdout, stderr)
}

// rtCoreDeployPackageSource: the package deploy installs is the one the cell's
// relay built (realtime-core build --fetch-package, which release runs); the
// manifest build peer with tools/build-rt-core-package.sh is the explicit
// legacy path. A dry run may render without either.
func rtCoreDeployPackageSource(packagePath string, legacy, dryRun bool) error {
	switch {
	case packagePath != "" && legacy:
		return fmt.Errorf("--package and --legacy-build-peer exclude each other: the package is either the relay's build or the build peer's")
	case packagePath == "" && !legacy && !dryRun:
		return fmt.Errorf("realtime-core deploy needs --package: build the cell's package on its relay with go run . realtime-core build --manifest <cell> --commit <sha> --test --fetch-package (it prints the deploy command; realtime-core release runs both). The manifest build peer path (tools/build-rt-core-package.sh) is --legacy-build-peer")
	}
	return nil
}

// rejectRetiredRealtimeCoreFlags refuses any flag outside the native set, so a
// retired v4 selection (--ready-sim, --tour, ...) cannot slip through.
func rejectRetiredRealtimeCoreFlags(args []string, allowed ...string) error {
	for _, arg := range args {
		if !strings.HasPrefix(arg, "--") {
			continue
		}
		name, _, _ := strings.Cut(arg, "=")
		if name == "--help" {
			continue
		}
		if !containsString(allowed, name) {
			return &retiredRuntimeError{Selection: arg}
		}
	}
	return nil
}

func selectRealtimeCoreManifest(manifestPath string) (func(), error) {
	restore := withRobotV4ManifestPath(firstNonEmpty(manifestPath, selectedRobotV4ManifestPath()))
	manifest, err := robotV4LoadSelectedManifest()
	if err == nil {
		err = robotV4ValidateShippedRuntimeSelection(manifest)
	}
	if err == nil && !robotV4ManifestSelectsRTCore(manifest) {
		err = &retiredRuntimeError{Selection: "realtime-core"}
	}
	if err != nil {
		restore()
		return nil, err
	}
	return restore, nil
}

func robotV4DaemonStartupArgsFromManifest(node robotV4DaemonNodeInventoryNode) ([]string, error) {
	args := make([]string, 0, len(node.DaemonStartupArgs))
	for _, raw := range node.DaemonStartupArgs {
		arg := strings.TrimSpace(raw)
		if arg == "" {
			continue
		}
		if strings.ContainsAny(arg, " \t\r\n") {
			return nil, fmt.Errorf("deployment manifest node %q daemon_startup_args contains whitespace in %q; split flags and values into separate entries", firstNonEmpty(node.Node, node.Peer), raw)
		}
		args = append(args, arg)
	}
	return args, nil
}

func robotV4DaemonEnvironmentFromManifest(node robotV4DaemonNodeInventoryNode) ([]string, error) {
	label := firstNonEmpty(node.Node, node.Peer)
	serviceEnvironment, err := robotV4DaemonEnvironmentValuesFromManifestMap(label, "daemon_service.daemon_environment", node.DaemonService.DaemonEnvironment)
	if err != nil {
		return nil, err
	}
	nodeEnvironment, err := robotV4DaemonEnvironmentValuesFromManifestMap(label, "daemon_environment", node.DaemonEnvironment)
	if err != nil {
		return nil, err
	}
	return robotV4MergeDaemonEnvironmentValues(append(serviceEnvironment, nodeEnvironment...)), nil
}

func robotV4DaemonEnvironmentValuesFromManifestMap(label, field string, environment map[string]string) ([]string, error) {
	if len(environment) == 0 {
		return nil, nil
	}
	keys := make([]string, 0, len(environment))
	valuesByKey := make(map[string]string, len(environment))
	keyPattern := regexp.MustCompile(`^[A-Za-z_][A-Za-z0-9_]*$`)
	for rawKey, value := range environment {
		key := strings.TrimSpace(rawKey)
		if key == "" || !keyPattern.MatchString(key) {
			return nil, fmt.Errorf("deployment manifest node %q %s contains invalid key %q", label, field, rawKey)
		}
		valuesByKey[key] = value
		keys = append(keys, key)
	}
	sort.Strings(keys)
	values := make([]string, 0, len(keys))
	for _, key := range keys {
		value := valuesByKey[key]
		if strings.ContainsAny(value, "\r\n") {
			return nil, fmt.Errorf("deployment manifest node %q %s %s must be a single-line value", label, field, key)
		}
		values = append(values, key+"="+value)
	}
	return values, nil
}

func robotV4MergeDaemonEnvironmentValues(values []string) []string {
	order := make([]string, 0, len(values))
	byKey := make(map[string]string, len(values))
	for _, raw := range values {
		value := strings.TrimSpace(raw)
		if value == "" {
			continue
		}
		key, _, ok := strings.Cut(value, "=")
		if !ok || key == "" {
			continue
		}
		if _, exists := byKey[key]; !exists {
			order = append(order, key)
		}
		byKey[key] = value
	}
	merged := make([]string, 0, len(order))
	for _, key := range order {
		merged = append(merged, byKey[key])
	}
	return merged
}

func preflightRealtimeCoreTargetAuth(stdout, stderr io.Writer, viaTarget, rtTarget string) error {
	if err := robotV4RealtimeCoreRemoteSSH(stdout, stderr, viaTarget, realtimeCoreRelaySSHAuthPreflightCommand(rtTarget)); err != nil {
		return fmt.Errorf("cannot authenticate to %s through motion-server relay before relay source sync or remote build; routing is reachable but RevPi SSH credentials/key are missing; RevPi SSH login is usually user pi with the device/sticker password, or pi/raspberry on default images; newer RevPi images can lock login for 10 minutes after repeated failures: %w", rtTarget, err)
	}
	return nil
}

type motionServerDeployOptions struct {
	Host           string
	ManifestPath   string
	AxisEnv        []string
	RemoteUser     string
	RemoteRepo     string
	DaemonStateURL string
	Prebuilt       string
	DryRun         bool
}

func nonEmptyStrings(values []string) []string {
	out := make([]string, 0, len(values))
	for _, value := range values {
		if trimmed := strings.TrimSpace(value); trimmed != "" {
			out = append(out, trimmed)
		}
	}
	return out
}

func syncMotionServerSources(root string, stdout, stderr io.Writer, sshTarget, remoteRepo string) error {
	if err := runRemoteSSH(stdout, stderr, sshTarget, "mkdir -p "+remoteShellPath(remoteRepo)); err != nil {
		return err
	}
	tar := exec.Command("tar", motionServerSourceArchiveArgs(root)...)
	ssh := exec.Command("ssh", sshTarget, "tar -C "+remoteShellPath(remoteRepo)+" -xzf -")
	reader, writer := io.Pipe()
	tar.Stdout = writer
	tar.Stderr = stderr
	ssh.Stdin = reader
	ssh.Stdout = stdout
	ssh.Stderr = stderr

	fmt.Fprintf(stdout, "motion-server: syncing motion-server/v1 plus daemon/v1 metadata to %s:%s\n", sshTarget, remoteRepo)
	if err := ssh.Start(); err != nil {
		_ = reader.Close()
		_ = writer.Close()
		return err
	}
	if err := tar.Start(); err != nil {
		_ = reader.Close()
		_ = writer.Close()
		_ = ssh.Wait()
		return err
	}
	tarErr := tar.Wait()
	_ = writer.Close()
	sshErr := ssh.Wait()
	_ = reader.Close()
	if tarErr != nil {
		return tarErr
	}
	return sshErr
}

func motionServerSourceArchiveArgs(root string) []string {
	return []string{
		"-C", root,
		"--no-xattrs",
		"-czf", "-",
		"steamdeck/real/v4/orchestrator",
		"motion-server/v1",
		"robot_description/robots/rosie_1420_v1/robot.urdf",
	}
}
