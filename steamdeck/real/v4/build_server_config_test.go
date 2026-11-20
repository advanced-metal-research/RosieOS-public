package main

import (
	"strings"
	"testing"
)

func TestRobotV4ConfiguredBuildServerIsEmptyWithoutAChoice(t *testing.T) {
	t.Setenv(robotV4BuildServerEnv, "")
	t.Setenv("ROBOT_V4_STEAMDECK_BUILD_HOST", "")
	if got := robotV4ConfiguredBuildServer(""); got != "" {
		t.Fatalf("build server with no choice = %q, want empty (the manifest decides)", got)
	}
}

func TestRobotV4ConfiguredBuildServerPrecedence(t *testing.T) {
	t.Setenv(robotV4BuildServerEnv, "env-builder")
	t.Setenv("ROBOT_V4_STEAMDECK_BUILD_HOST", "legacy-builder")
	if got := robotV4ConfiguredBuildServer(""); got != "env-builder" {
		t.Fatalf("environment build server = %q, want env-builder", got)
	}
	if got := robotV4ConfiguredBuildServer("explicit-builder"); got != "explicit-builder" {
		t.Fatalf("explicit build server = %q, want explicit-builder", got)
	}
}

// TestRobotV4PickBuildPeerFollowsTheManifest keeps the build host out of
// code: with no operator choice the manifest's preferred peers decide, in
// order, skipping one the manifest does not list or that cannot build the
// artifact; an operator choice still wins, and an unusable one refuses.
func TestRobotV4PickBuildPeerFollowsTheManifest(t *testing.T) {
	artifact := robotV4DeployManifestArtifact{ArtifactName: "rosie-rt-core", TargetHost: "example-servo"}
	ready := func(name string, scope ...string) robotV4BuildPeerInventory {
		return robotV4BuildPeerInventory{Peer: name, BuildService: robotV4BuildServiceInventory{CompatibilityStatus: "ready", ArtifactScope: scope}}
	}
	peers := []robotV4BuildPeerInventory{ready("build-main", "rosie-daemon-v1"), ready("build-standby", "rosie-rt-core"), ready("build-spare", "rosie-rt-core")}
	preferred := []robotV4BuildCachePolicyPeer{{Peer: "build-missing"}, {Peer: "build-main"}, {Peer: "build-standby"}, {Peer: "build-spare"}}

	got, policy, err := robotV4PickBuildPeer(preferred, peers, artifact, "")
	if err != nil || got.Peer != "build-standby" || policy.Peer != "build-standby" {
		t.Fatalf("no choice: got %q (%q), %v; want build-standby, the first listed peer that builds rosie-rt-core", got.Peer, policy.Peer, err)
	}
	if got, _, err = robotV4PickBuildPeer(preferred, peers, artifact, "build-spare"); err != nil || got.Peer != "build-spare" {
		t.Fatalf("operator choice: got %q, %v; want build-spare", got.Peer, err)
	}
	if _, _, err = robotV4PickBuildPeer(preferred, peers, artifact, "build-main"); err == nil || !strings.Contains(err.Error(), `"build-main"`) {
		t.Fatalf("a choice that cannot build the artifact must refuse by name: %v", err)
	}
	if _, _, err = robotV4PickBuildPeer(preferred[:2], peers, artifact, ""); err == nil || !strings.Contains(err.Error(), robotV4BuildServerEnv) {
		t.Fatalf("no qualifying peer must refuse and name the override: %v", err)
	}
}
