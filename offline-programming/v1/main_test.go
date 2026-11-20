package main

import (
	"bytes"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
)

func TestRunHelpAndUnknownCommand(t *testing.T) {
	var stdout, stderr bytes.Buffer
	if code := Run([]string{"help"}, &stdout, &stderr); code != 0 {
		t.Fatalf("help exit=%d stderr=%q", code, stderr.String())
	}
	if !strings.Contains(stdout.String(), "offline-programming") || !strings.Contains(stdout.String(), "serve") {
		t.Fatalf("help missing lifecycle: %q", stdout.String())
	}
	stdout.Reset()
	stderr.Reset()
	if code := Run([]string{"launch-real-robot"}, &stdout, &stderr); code != 2 {
		t.Fatalf("unknown command exit=%d want 2", code)
	}
}

func TestRunTestFilterDispatchesOnlyRequestedSuite(t *testing.T) {
	previous := runExternalCommand
	t.Cleanup(func() { runExternalCommand = previous })
	var calls [][]string
	runExternalCommand = func(_ string, _ io.Writer, _ io.Writer, name string, args ...string) error {
		calls = append(calls, append([]string{name}, args...))
		return nil
	}
	var stdout, stderr bytes.Buffer
	if code := Run([]string{"test", "--filter", "go"}, &stdout, &stderr); code != 0 {
		t.Fatalf("go test dispatch exit=%d stderr=%q", code, stderr.String())
	}
	if len(calls) != 1 || strings.Join(calls[0], " ") != "go test ./..." {
		t.Fatalf("calls=%#v", calls)
	}

	calls = nil
	stdout.Reset()
	stderr.Reset()
	if code := Run([]string{"test", "--filter", "live"}, &stdout, &stderr); code != 0 {
		t.Fatalf("live test dispatch exit=%d stderr=%q", code, stderr.String())
	}
	if len(calls) != 1 ||
		strings.Join(calls[0], " ") !=
			"go test ./internal/simulator -run ^TestPCNativeBackendLiveCanonicalPlanner$ -count=1 -v -offline-live-planner" {
		t.Fatalf("live calls=%#v", calls)
	}
}

func TestBinaryNameUsesPlatformSuffix(t *testing.T) {
	got := binaryName()
	if runtime.GOOS == "windows" && filepath.Ext(got) != ".exe" {
		t.Fatalf("windows binary=%q", got)
	}
	if runtime.GOOS != "windows" && filepath.Ext(got) == ".exe" {
		t.Fatalf("non-windows binary=%q", got)
	}
}

func TestLocalSimulatorListenRequiresExplicitLoopback(t *testing.T) {
	for _, value := range []string{
		"127.0.0.1:8794",
		"localhost:8794",
		"[::1]:8794",
	} {
		if _, ok := loopbackListenHost(value); !ok {
			t.Fatalf("loopback listen rejected: %q", value)
		}
	}
	for _, value := range []string{
		"0.0.0.0:8794",
		":8794",
		"192.0.2.10:8794",
		"bad-address",
	} {
		if _, ok := loopbackListenHost(value); ok {
			t.Fatalf("non-loopback listen accepted: %q", value)
		}
	}
}

func TestRepositoryRootRequiresMotionServerAtRepoRoot(t *testing.T) {
	root := t.TempDir()
	product := filepath.Join(root, "offline-programming", "v1")
	for _, relative := range []string{
		filepath.Join("offline-programming", "v1"),
		filepath.Join("cadquery", "v1"),
		filepath.Join("tesseract", "v1"),
		filepath.Join("robot", "v4", "motion-server"),
	} {
		if err := os.MkdirAll(filepath.Join(root, relative), 0o755); err != nil {
			t.Fatal(err)
		}
	}
	if _, err := repositoryRoot(product); err == nil {
		t.Fatal("expected missing motion-server/v1 to fail even when robot/v4/motion-server exists")
	} else if !strings.Contains(err.Error(), "motion-server/v1") {
		t.Fatalf("error %q should name motion-server/v1", err)
	}
	if err := os.MkdirAll(filepath.Join(root, "motion-server", "v1"), 0o755); err != nil {
		t.Fatal(err)
	}
	got, err := repositoryRoot(product)
	if err != nil {
		t.Fatal(err)
	}
	if got != filepath.Clean(root) {
		t.Fatalf("repo=%q want %q", got, filepath.Clean(root))
	}
}
