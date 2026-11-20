package main

import (
	"crypto/sha256"
	"encoding/base64"

	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"

	"net/http"
	"net/url"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"time"

	robotv4build "rosieos/steamdeck/real/v4/internal/build"
)

func robotV4BuildCachePolicyForArtifact(artifact robotV4DeployManifestArtifact) (robotV4BuildCachePolicy, bool) {
	node, ok := robotV4DaemonNodeInventoryRecord(robotV4DeployCanonicalNodeSelector(artifact.TargetHost))
	if !ok {
		return robotV4BuildCachePolicy{}, false
	}
	policy := node.BuildCachePolicy
	if strings.TrimSpace(policy.Mode) == "" &&
		policy.RuntimePeerBuild == nil &&
		len(policy.PreferredBuildPeers) == 0 &&
		len(policy.RequiredRecordFields) == 0 &&
		!policy.InstallOnlyAfterHashVerified {
		return robotV4BuildCachePolicy{}, false
	}
	return policy, true
}

type robotV4BuildPeerRequestCandidate struct {
	Inventory robotV4BuildPeerInventory
	Policy    robotV4BuildCachePolicyPeer
	DaemonURL string
}

func robotV4PublishBuildPeerCacheRecord(opts deployOptions, cache robotV4DaemonDeployBuildCacheRecord, buildPeerDaemonURL string) (bool, error) {
	daemonURL := strings.TrimRight(strings.TrimSpace(opts.DaemonURL), "/")
	if daemonURL == "" {
		return false, nil
	}
	if robotV4BuildPeerCacheRecordAlreadyOnOrchestrator(daemonURL, buildPeerDaemonURL, cache) {
		return false, nil
	}
	client := &http.Client{Timeout: 5 * time.Minute}
	if err := robotV4HydrateBuildPeerCacheBlobFromNode(client, daemonURL, cache.BinarySHA256, cache.BinaryBlobURI, cache.PeerNode); err != nil {
		// Self-updating an older orchestrator can require the very transport fix
		// contained in the replacement binary. Keep daemon/Iroh hydration first,
		// then use the CLI only as a content-verified compatibility relay so the
		// upgrade does not deadlock on the old daemon's transfer limits.
		if relayErr := robotV4RelayBuildPeerCacheBlob(client, buildPeerDaemonURL, daemonURL, cache.BinarySHA256); relayErr != nil {
			return false, fmt.Errorf("hydrate build-peer binary cache on orchestrator daemon: %w; compatibility relay failed: %v", err, relayErr)
		}
	}
	raw, err := json.Marshal(cache)
	if err != nil {
		return false, fmt.Errorf("marshal build-peer cache record: %w", err)
	}
	endpoint := daemonURL + "/api/deploy/build-cache?node=" + url.QueryEscape(cache.PeerNode)
	if _, err := postDaemonJSON(client, endpoint, raw); err != nil {
		return false, fmt.Errorf("publish build-peer cache record to orchestrator daemon: %w", err)
	}
	return true, nil
}

func robotV4BuildPeerCacheRecordAlreadyOnOrchestrator(daemonURL, buildPeerDaemonURL string, cache robotV4DaemonDeployBuildCacheRecord) bool {
	daemonURL = strings.TrimRight(strings.TrimSpace(daemonURL), "/")
	buildPeerDaemonURL = strings.TrimRight(strings.TrimSpace(buildPeerDaemonURL), "/")
	if daemonURL == "" || buildPeerDaemonURL == "" || daemonURL != buildPeerDaemonURL {
		return false
	}
	health, err := fetchRobotV4DaemonHealth(&http.Client{Timeout: 2 * time.Second}, daemonURL)
	if err != nil {
		return false
	}
	return robotV4DeployObservedIdentityMatchesPeer(health.Node, cache.PeerNode)
}

func robotV4HydrateBuildPeerCacheBlobFromNode(client *http.Client, daemonURL, sha256, uri, sourceNode string) error {
	sha256 = strings.TrimSpace(sha256)
	uri = strings.TrimSpace(uri)
	if sha256 == "" || uri == "" {
		return nil
	}
	request := map[string]string{
		"sha256": sha256,
		"uri":    uri,
	}
	if sourceNode = strings.TrimSpace(sourceNode); sourceNode != "" {
		// The cache producer is the authoritative first fallback when a native
		// Iroh ticket cannot connect. Supplying it avoids probing every gossiped
		// node before reaching the build peer that just proved the blob exists.
		request["source_node"] = sourceNode
	}
	body, err := json.Marshal(request)
	if err != nil {
		return err
	}
	if _, err := postDaemonJSON(client, daemonURL+"/api/artifacts/fetch", body); err != nil {
		return err
	}
	return nil
}

func robotV4RelayBuildPeerCacheBlob(client *http.Client, sourceDaemonURL, destinationDaemonURL, sha256Value string) error {
	const maxBlobBytes = int64(512 * 1024 * 1024)
	if err := robotV4ValidateOutboundDaemonHTTPURL(destinationDaemonURL); err != nil {
		return err
	}
	normalized, err := normalizeRobotV4SHA256(sha256Value)
	if err != nil {
		return err
	}
	endpoint := strings.TrimRight(strings.TrimSpace(sourceDaemonURL), "/") +
		"/api/artifacts/blob?sha256=" + url.QueryEscape(normalized)
	if err := robotV4ValidateOutboundDaemonHTTPURL(endpoint); err != nil {
		return err
	}
	response, err := client.Get(endpoint)
	if err != nil {
		return fmt.Errorf("GET build-peer blob %s: %w", endpoint, err)
	}
	defer response.Body.Close()
	if response.StatusCode < 200 || response.StatusCode >= 300 {
		raw, _ := io.ReadAll(io.LimitReader(response.Body, 4096))
		return fmt.Errorf("GET build-peer blob %s: HTTP %s: %s", endpoint, response.Status, strings.TrimSpace(string(raw)))
	}
	if response.ContentLength > maxBlobBytes {
		return fmt.Errorf("GET build-peer blob %s: response too large %d > %d", endpoint, response.ContentLength, maxBlobBytes)
	}
	raw, err := io.ReadAll(io.LimitReader(response.Body, maxBlobBytes+1))
	if err != nil {
		return fmt.Errorf("read build-peer blob %s: %w", endpoint, err)
	}
	if int64(len(raw)) > maxBlobBytes {
		return fmt.Errorf("GET build-peer blob %s: response too large %d > %d", endpoint, len(raw), maxBlobBytes)
	}
	if observed := sha256String(raw); observed != normalized {
		return fmt.Errorf("GET build-peer blob %s: sha256 mismatch observed=%s expected=%s", endpoint, observed, normalized)
	}
	if _, err := postDaemonBlob(client, destinationDaemonURL, raw, normalized); err != nil {
		return fmt.Errorf("upload verified build-peer blob to orchestrator: %w", err)
	}
	return nil
}

func robotV4BuildServiceSupportsArtifact(service robotV4BuildServiceInventory, artifact robotV4DeployManifestArtifact) bool {
	status := strings.ToLower(strings.TrimSpace(service.CompatibilityStatus))
	if status != "" && status != "ready" && status != "compatible" {
		// A sysroot-blocked build peer stays eligible when the manifest
		// declares sync_paths: the deploy auto-provisions the sysroot from
		// the source peer before requesting the build.
		if !(strings.Contains(status, "sysroot") && robotV4BuildServiceSysrootProvisionable(service.TargetSysroot)) {
			return false
		}
	}
	if len(service.ArtifactScope) == 0 {
		return true
	}
	for _, item := range service.ArtifactScope {
		switch strings.TrimSpace(item) {
		case artifact.ArtifactName, artifact.ArtifactKind:
			return true
		}
	}
	return false
}

func robotV4BuildPeerInventoryForPolicyPeer(buildPeers []robotV4BuildPeerInventory, preferred robotV4BuildCachePolicyPeer) (robotV4BuildPeerInventory, bool) {
	preferredName := robotV4DeployCanonicalPeerName(preferred.Peer)
	for _, buildPeer := range buildPeers {
		if !robotV4BuildPeerNameMatches(buildPeer, preferredName) {
			continue
		}
		if !robotV4BuildPeerPlatformMatches(buildPeer, preferred) {
			continue
		}
		return buildPeer, true
	}
	return robotV4BuildPeerInventory{}, false
}

func robotV4BuildPeerNameMatches(buildPeer robotV4BuildPeerInventory, preferredName string) bool {
	for _, candidate := range []string{buildPeer.Peer, buildPeer.Role, buildPeer.Host, buildPeer.HostRole} {
		if robotV4DeployCanonicalPeerName(candidate) == preferredName {
			return true
		}
	}
	return false
}

func robotV4BuildPeerPlatformMatches(buildPeer robotV4BuildPeerInventory, preferred robotV4BuildCachePolicyPeer) bool {
	wantOS := strings.ToLower(strings.TrimSpace(preferred.PlatformOS))
	gotOS := strings.ToLower(strings.TrimSpace(buildPeer.PlatformOS))
	if wantOS != "" && gotOS != "" && wantOS != gotOS {
		return false
	}
	wantArch := robotV4DeployPlatformToken(preferred.PlatformArch)
	gotArch := robotV4DeployPlatformToken(buildPeer.PlatformArch)
	if wantArch != "" && gotArch != "" && wantArch != gotArch {
		return false
	}
	wantTriple := strings.ToLower(strings.TrimSpace(preferred.TargetTriple))
	gotTriple := strings.ToLower(strings.TrimSpace(buildPeer.TargetTriple))
	if wantTriple != "" && gotTriple != "" && wantTriple != gotTriple {
		return false
	}
	wantABI := robotV4TargetABIToken(preferred.TargetABI)
	if wantABI == "" {
		return true
	}
	gotABI := robotV4BuildPeerTargetABI(buildPeer)
	return gotABI != "" && wantABI == gotABI
}

func robotV4BuildPeerTargetABI(buildPeer robotV4BuildPeerInventory) string {
	return robotV4TargetABIToken(firstNonEmpty(buildPeer.TargetABI, buildPeer.BuildService.TargetABI))
}

func robotV4DeployTargetPlatform(targetHost string) (string, string) {
	node, _ := robotV4DaemonNodeInventoryRecord(robotV4DeployCanonicalNodeSelector(targetHost))
	if node.PlatformOS != "" || node.PlatformArch != "" {
		return strings.ToLower(strings.TrimSpace(node.PlatformOS)), robotV4DeployPlatformToken(node.PlatformArch)
	}
	return "", ""
}

func robotV4DeployTargetTriple(targetHost string) string {
	node, _ := robotV4DaemonNodeInventoryRecord(robotV4DeployCanonicalNodeSelector(targetHost))
	return strings.ToLower(strings.TrimSpace(node.TargetTriple))
}

func robotV4DeployTargetABI(targetHost string) string {
	node, _ := robotV4DaemonNodeInventoryRecord(robotV4DeployCanonicalNodeSelector(targetHost))
	return robotV4TargetABIToken(node.TargetABI)
}

func robotV4TargetABIToken(value string) string {
	return strings.ToLower(strings.TrimSpace(value))
}

func robotV4DeployPlatformToken(value string) string {
	switch strings.ToLower(strings.TrimSpace(value)) {
	case "arm64":
		return "aarch64"
	case "amd64":
		return "x86_64"
	default:
		return strings.ToLower(strings.TrimSpace(value))
	}
}

func robotV4ComponentSourceBundle(repo string, target robotV4MeshDeployTarget) ([]byte, string, []string, error) {
	if target.ArtifactKind == "realtime-core" || target.ArtifactKind == "realtime-core-nats-bridge" || target.ArtifactKind == "pair-owner" {
		return nil, "", nil, &retiredRuntimeError{Selection: target.ArtifactKind}
	}
	buildInputPaths := append([]string(nil), target.BuildInputPaths...)
	var rtIdentity []byte
	if target.ArtifactKind == "rt-core" {
		inputs, err := robotV4RTCoreSourceInputs(repo)
		if err != nil {
			return nil, "", nil, err
		}
		for _, required := range target.BuildInputPaths {
			if !pathExists(filepath.Join(repo, filepath.FromSlash(required))) {
				return nil, "", nil, fmt.Errorf("rt-core required source missing: %s", required)
			}
		}
		buildInputPaths = robotV4UniqueSourceInputPaths(append(buildInputPaths, inputs...))
		for _, required := range inputs {
			if strings.HasPrefix(required, "rt-core/build/deps/") {
				info, err := os.Lstat(filepath.Join(repo, filepath.FromSlash(required)))
				if err != nil || !info.Mode().IsRegular() || info.Size() == 0 {
					return nil, "", nil, fmt.Errorf("rt-core requires provisioned IgH source notice %s; copy it from the pinned ethercat-1.6.9 source tree before bundling", required)
				}
			}
		}
		statusArgs := append([]string{"status", "--porcelain", "--untracked-files=normal", "--"}, buildInputPaths...)
		status, err := robotV4GitOutput(repo, statusArgs...)
		if err != nil {
			return nil, "", nil, err
		}
		if status != "" {
			return nil, "", nil, fmt.Errorf("rt-core package requires committed build inputs")
		}
		sha, err := robotV4GitOutput(repo, "rev-parse", "HEAD")
		if err != nil {
			return nil, "", nil, err
		}
		epoch, err := robotV4GitOutput(repo, "show", "-s", "--format=%ct", sha)
		if err != nil {
			return nil, "", nil, err
		}
		rtIdentity = []byte(sha + " " + epoch + "\n")
	}
	if len(buildInputPaths) == 0 && target.ArtifactKind == "daemon-v1" {
		daemonInputs, err := robotv4build.DaemonV1SourceInputPaths(repo)
		if err != nil {
			return nil, "", nil, err
		}
		buildInputPaths = daemonInputs
	}
	var files []robotV4SourceBundleFile
	var included []string
	if rtIdentity != nil {
		files = append(files, robotV4SourceBundleFile{Path: "rt-core/SOURCE_IDENTITY", SHA256: sha256String(rtIdentity), PayloadBase64: base64.StdEncoding.EncodeToString(rtIdentity)})
		included = append(included, "rt-core/SOURCE_IDENTITY")
	}
	for _, rel := range buildInputPaths {
		cleanRel, err := robotV4CleanComponentBuildInputPath(rel)
		if err != nil {
			return nil, "", nil, err
		}
		abs := filepath.Join(repo, filepath.FromSlash(cleanRel))
		if !pathExists(abs) {
			continue
		}
		raw, err := os.ReadFile(abs)
		if err != nil {
			return nil, "", nil, fmt.Errorf("read deploy source input %s: %w", cleanRel, err)
		}
		files = append(files, robotV4SourceBundleFile{
			Path:          cleanRel,
			SHA256:        sha256String(raw),
			PayloadBase64: base64.StdEncoding.EncodeToString(raw),
		})
		included = append(included, cleanRel)
	}
	if len(files) == 0 {
		return nil, "", nil, fmt.Errorf("deploy target %s has no existing component-scoped build inputs", target.Peer)
	}
	sort.Slice(files, func(i, j int) bool { return files[i].Path < files[j].Path })
	sort.Strings(included)
	bundle := robotV4SourceBundle{
		SchemaVersion: "robot_v4_component_source_bundle_v1",
		// Component source and its recipe are portable across peers on the
		// same platform. The signed artifact manifest retains target_host;
		// leaving the inner bundle target empty lets one relay reuse a cache
		// built for another relay from identical source instead of rebuilding it.
		TargetHost:        "",
		BuildRecipeSHA256: sha256String([]byte(target.BuildCommand)),
		Files:             files,
	}
	raw, err := json.Marshal(bundle)
	if err != nil {
		return nil, "", nil, fmt.Errorf("marshal source bundle: %w", err)
	}
	return raw, sha256String(raw), included, nil
}

func robotV4UniqueSourceInputPaths(paths []string) []string {
	out := make([]string, 0, len(paths))
	seen := make(map[string]struct{}, len(paths))
	for _, path := range paths {
		clean := filepath.ToSlash(filepath.Clean(path))
		if clean == "" || clean == "." {
			continue
		}
		if _, ok := seen[clean]; ok {
			continue
		}
		seen[clean] = struct{}{}
		out = append(out, clean)
	}
	return out
}

func robotV4CleanComponentBuildInputPath(rel string) (string, error) {
	cleanRel := filepath.ToSlash(filepath.Clean(filepath.FromSlash(strings.TrimSpace(rel))))
	if cleanRel == "." || cleanRel == "/" || cleanRel == "" {
		return "", fmt.Errorf("deploy source input %q would sync the whole checkout", rel)
	}
	// What the 1420 description IS, and nothing else: the files its
	// robot_description_manifest.json registers (docs/ROBOT-DESCRIPTION-CONTRACT.md).
	// The cell hashes exactly these and refuses every plan when one is missing,
	// so the SRDF and the JSON documents ship with the URDF and the
	// meshes. The CAD export and the bundle tools beside them are provenance
	// and stay out.
	rosie1420RuntimeAsset := cleanRel == "robot_description/robots/rosie_1420_v1/robot.urdf" ||
		cleanRel == "robot_description/robots/rosie_1420_v1/robot.srdf" ||
		cleanRel == "robot_description/robots/rosie_1420_v1/robot_description_manifest.json" ||
		cleanRel == "robot_description/robots/rosie_1420_v1/rtcore_definition.json" ||
		cleanRel == "robot_description/robots/rosie_1420_v1/config.json" ||
		cleanRel == "robot_description/robots/rosie_1420_v1/spheres.json" ||
		(strings.HasPrefix(cleanRel, "robot_description/robots/rosie_1420_v1/meshes/") &&
			strings.EqualFold(filepath.Ext(cleanRel), ".dae"))
	if cleanRel != "steamdeck/real/v4/go.mod" && cleanRel != "steamdeck/real/v4/go.sum" &&
		cleanRel != "steamdeck/real/v4/tools/build-rt-core-package.sh" &&
		!strings.HasPrefix(cleanRel, "steamdeck/real/v4/orchestrator/") &&
		!strings.HasPrefix(cleanRel, "rt-core/") &&
		!strings.HasPrefix(cleanRel, "daemon/v1/") &&
		!strings.HasPrefix(cleanRel, "motion-server/v1/") &&
		!strings.HasPrefix(cleanRel, "robot_description/robots/rosie_1400_v3/") &&
		!rosie1420RuntimeAsset {
		return "", fmt.Errorf("deploy source input %q is outside daemon/v1, motion-server/v1, rt-core, the rt-core package script, or the reviewed Robot v4 URDF sources", rel)
	}
	for _, segment := range strings.Split(cleanRel, "/") {
		switch segment {
		case ".git", ".run", "artifacts", "bin", "dist", "node_modules", "target":
			return "", fmt.Errorf("deploy source input %q includes generated payload segment %q", rel, segment)
		case "build":
			if strings.HasPrefix(cleanRel, "rt-core/") && cleanRel != "rt-core/build/deps/ethercat-1.6.9/COPYING" && cleanRel != "rt-core/build/deps/ethercat-1.6.9/COPYING.LESSER" {
				return "", fmt.Errorf("rt-core build output is not a source input")
			}
		}
	}
	return cleanRel, nil
}

func normalizeRobotV4SHA256(value string) (string, error) {
	value = strings.ToLower(strings.TrimSpace(value))
	hexValue := strings.TrimPrefix(value, "sha256:")
	if len(hexValue) != 64 {
		return "", fmt.Errorf("expected sha256:<64-hex> or 64-hex, got %q", value)
	}
	for _, ch := range hexValue {
		if !((ch >= '0' && ch <= '9') || (ch >= 'a' && ch <= 'f')) {
			return "", fmt.Errorf("expected sha256:<64-hex> or 64-hex, got %q", value)
		}
	}
	return "sha256:" + hexValue, nil
}

func sha256String(raw []byte) string {
	sum := sha256.Sum256(raw)
	return "sha256:" + hex.EncodeToString(sum[:])
}

func robotV4BootstrapPathSegment(value, fallback string) string {
	value = strings.TrimSpace(value)
	var b strings.Builder
	for _, r := range value {
		switch {
		case r >= 'a' && r <= 'z':
			b.WriteRune(r)
		case r >= 'A' && r <= 'Z':
			b.WriteRune(r)
		case r >= '0' && r <= '9':
			b.WriteRune(r)
		case r == '-' || r == '_' || r == '.':
			b.WriteRune(r)
		}
	}
	if b.Len() == 0 {
		return fallback
	}
	return b.String()
}
