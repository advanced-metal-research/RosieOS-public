package motion_server

import (
	"os"

	"path/filepath"
	"strings"
	"testing"
)

const motionServerStatusStream = "ROBOT_V4_MOTION_SERVER"
const realtimeCoreStatusStream = "ROBOT_V4_RTCORE"

type daemonStateSnapshot struct {
	Nodes           []daemonStateNodeSnapshot      `json:"nodes"`
	DeployManifests []daemonDeployManifestSnapshot `json:"deploy_manifests"`
	ProcessIntents  []daemonProcessIntentSnapshot  `json:"process_intents"`
}

type daemonStateNodeSnapshot struct {
	Node             string                          `json:"node"`
	ServiceEndpoints []daemonServiceEndpointSnapshot `json:"service_endpoints"`
}

type daemonServiceEndpointSnapshot struct {
	Name     string `json:"name"`
	Protocol string `json:"protocol"`
	URL      string `json:"url"`
	Status   string `json:"status"`
}

type daemonDeployManifestSnapshot struct {
	BuildSessionID string   `json:"build_session_id"`
	TestSessionID  string   `json:"test_session_id"`
	TargetHosts    []string `json:"target_hosts"`
	ArtifactNames  []string `json:"artifact_names"`
	Status         string   `json:"status"`
}

type daemonProcessIntentSnapshot struct {
	ID         string `json:"id"`
	TargetNode string `json:"target_node"`
	Status     string `json:"status"`
	Process    struct {
		Name string   `json:"name"`
		Env  []string `json:"env"`
	} `json:"process"`
}

func findRepoRoot(t *testing.T) string {
	t.Helper()
	if root := strings.TrimSpace(os.Getenv("ROBOT_V4_REPO_ROOT")); root != "" {
		if isRobotV4RepoRoot(root) {
			return root
		}
		t.Fatalf("ROBOT_V4_REPO_ROOT=%q is not a robot v4 repo root", root)
	}
	wd, err := os.Getwd()
	if err != nil {
		t.Fatal(err)
	}
	for {
		if isRobotV4RepoRoot(wd) {
			return wd
		}
		parent := filepath.Dir(wd)
		if parent == wd {
			t.Fatalf("could not find repo root from %s", wd)
		}
		wd = parent
	}
}

func isRobotV4RepoRoot(root string) bool {
	return fileExists(filepath.Join(root, "steamdeck", "real", "v4", "go.mod")) &&
		fileExists(filepath.Join(root, "rt-core", "config", "cells")) &&
		(fileExists(filepath.Join(root, "rosie.sh")) ||
			fileExists(filepath.Join(root, "daemon", "v1", "Cargo.toml")))
}

func fileExists(path string) bool {
	_, err := os.Stat(path)
	return err == nil
}
