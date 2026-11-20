package main

import (
	"reflect"
	"strings"
	"testing"
)

func TestRobotV4DaemonRuntimeConfigCanonicalManifestDefaults(t *testing.T) {
	parent := robotV4DaemonNodeInventoryNode{
		Node:    "parent-node",
		Peer:    "parent-peer",
		Role:    "parent-role",
		Host:    "parent-host",
		Aliases: []string{"parent-peer.tail.example.ts.net", "robot-v4.parent"},
	}
	child := robotV4DaemonNodeInventoryNode{
		Node: "child-node",
		Peer: "child-peer",
		Role: "child-role",
		IPv6Fallback: robotV4DaemonIPv6Fallback{
			ParentNode:                parent.Node,
			ParentPeer:                parent.Peer,
			ParentDaemonURLFromTarget: "http://[fe80::1%25eth0]:8787",
		},
	}
	manifest := robotV4DaemonNodeInventoryManifest{Nodes: []robotV4DaemonNodeInventoryNode{parent, child}}

	config, err := compileRobotV4DaemonRuntimeConfig(manifest, child)
	if err != nil {
		t.Fatal(err)
	}
	if config.SelfRestartDelaySeconds != robotV4DaemonDefaultSelfRestartDelaySeconds {
		t.Fatalf("self restart delay seconds = %d, want daemon runtime default %d", config.SelfRestartDelaySeconds, robotV4DaemonDefaultSelfRestartDelaySeconds)
	}
	wantBlobFetchURLs := []string{
		"parent-host=http://[fe80::1%25eth0]:8787",
		"parent-peer.tail.example.ts.net=http://[fe80::1%25eth0]:8787",
		"parent-peer=http://[fe80::1%25eth0]:8787",
	}
	if !reflect.DeepEqual(config.BlobFetchURLs, wantBlobFetchURLs) {
		t.Fatalf("blob fetch URLs = %#v, want %#v", config.BlobFetchURLs, wantBlobFetchURLs)
	}
	for _, forbidden := range []string{"parent-node", "parent-role", "robot-v4.parent"} {
		if strings.Contains(strings.Join(config.BlobFetchURLs, "\n"), forbidden+"=") {
			t.Fatalf("semantic alias %q leaked into blob fetch identities: %#v", forbidden, config.BlobFetchURLs)
		}
	}
	_, raw, err := robotV4DaemonRuntimeConfigHash(config)
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(string(raw), "null") {
		t.Fatalf("runtime config JSON contains null instead of canonical empty arrays: %s", raw)
	}
}

func TestRobotV4DaemonRuntimeConfigPreservesExplicitZeroSelfRestartDelay(t *testing.T) {
	node := robotV4DaemonNodeInventoryNode{
		Node: "example-relay",
		Peer: "example-relay",
		DaemonEnvironment: map[string]string{
			"DAEMON_V1_SELF_RESTART_DELAY_SECS": "0",
		},
	}
	manifest := robotV4DaemonNodeInventoryManifest{Nodes: []robotV4DaemonNodeInventoryNode{node}}

	config, err := compileRobotV4DaemonRuntimeConfig(manifest, node)
	if err != nil {
		t.Fatal(err)
	}
	if config.SelfRestartDelaySeconds != 0 {
		t.Fatalf("self restart delay seconds = %d, want explicit override 0", config.SelfRestartDelaySeconds)
	}
}

func TestRobotV4DaemonRuntimeConfigPreservesSelfRestartCommandWithSpaces(t *testing.T) {
	command := "systemctl --user restart com.gradient.rosie-daemon-v1.service || systemctl --user restart rosie-daemon-v1.service"
	node := robotV4DaemonNodeInventoryNode{
		Node:              "realtime-core",
		Peer:              "revpi-test",
		Role:              "realtime-core",
		DaemonSelfRestart: "  " + command + "  ",
	}
	manifest := robotV4DaemonNodeInventoryManifest{Nodes: []robotV4DaemonNodeInventoryNode{node}}

	config, err := compileRobotV4DaemonRuntimeConfig(manifest, node)
	if err != nil {
		t.Fatal(err)
	}
	if config.SelfRestartCommand != command {
		t.Fatalf("self restart command = %q, want %q", config.SelfRestartCommand, command)
	}
	hash, raw, err := robotV4DaemonRuntimeConfigHash(config)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(raw), `"self_restart_command":"`+command+`"`) {
		t.Fatalf("runtime config JSON does not preserve restart command with spaces: %s", raw)
	}

	node.DaemonSelfRestart = "systemctl --user restart rosie-daemon-v1.service"
	changedConfig, err := compileRobotV4DaemonRuntimeConfig(manifest, node)
	if err != nil {
		t.Fatal(err)
	}
	changedHash, _, err := robotV4DaemonRuntimeConfigHash(changedConfig)
	if err != nil {
		t.Fatal(err)
	}
	if changedHash == hash {
		t.Fatalf("self restart command change did not change runtime config hash %s", hash)
	}
}

func TestRobotV4DaemonRuntimeConfigDerivesSelfRestartCommandFromManifestServiceUnits(t *testing.T) {
	node := robotV4DaemonNodeInventoryNode{
		Node: "motion-server",
		Peer: "other-relay",
		Role: "motion-server",
		DaemonService: robotV4DaemonServiceInventory{
			Unit:          "com.gradient.rosie-daemon-v1.service",
			FallbackUnits: []string{"rosie-daemon-v1.service"},
		},
	}
	manifest := robotV4DaemonNodeInventoryManifest{Nodes: []robotV4DaemonNodeInventoryNode{node}}

	config, err := compileRobotV4DaemonRuntimeConfig(manifest, node)
	if err != nil {
		t.Fatal(err)
	}
	want := "systemctl --user restart com.gradient.rosie-daemon-v1.service || systemctl --user restart rosie-daemon-v1.service"
	if config.SelfRestartCommand != want {
		t.Fatalf("self restart command = %q, want manifest-derived %q", config.SelfRestartCommand, want)
	}
}

func TestRobotV4DaemonRuntimeConfigRejectsMultilineSelfRestartCommand(t *testing.T) {
	node := robotV4DaemonNodeInventoryNode{
		Node:              "realtime-core",
		Peer:              "revpi-test",
		Role:              "realtime-core",
		DaemonSelfRestart: "systemctl --user restart rosie-daemon-v1.service\nsystemctl --user status rosie-daemon-v1.service",
	}
	manifest := robotV4DaemonNodeInventoryManifest{Nodes: []robotV4DaemonNodeInventoryNode{node}}

	_, err := compileRobotV4DaemonRuntimeConfig(manifest, node)
	if err == nil {
		t.Fatal("compileRobotV4DaemonRuntimeConfig accepted a multiline self restart command")
	}
	if !strings.Contains(err.Error(), "daemon_self_restart_command must be a single-line command") {
		t.Fatalf("multiline self restart error = %v", err)
	}
}

func TestRobotV4DaemonRuntimeConfigUsesManifestHomePaths(t *testing.T) {
	node := robotV4DaemonNodeInventoryNode{
		Node:              "realtime-core",
		Peer:              "revpi-test",
		Role:              "realtime-core",
		RosieHome:         "/home/pi/.rosie",
		DaemonRuntimePath: "/home/pi/.rosie/bin/rosie-daemon-v1",
	}
	manifest := robotV4DaemonNodeInventoryManifest{Nodes: []robotV4DaemonNodeInventoryNode{node}}

	config, err := compileRobotV4DaemonRuntimeConfig(manifest, node)
	if err != nil {
		t.Fatal(err)
	}
	if config.MetadataFile != "/home/pi/.config/rosie-daemon-v1/daemon-v1.metadata.json" {
		t.Fatalf("metadata file = %q", config.MetadataFile)
	}
	if got := robotV4DaemonRuntimeConfigPathForNode(node); got != "/home/pi/.config/rosie-daemon-v1/runtime-config.json" {
		t.Fatalf("runtime config path = %q", got)
	}
}
