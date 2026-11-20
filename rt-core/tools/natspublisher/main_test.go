//go:build linux

package main

import (
	"context"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"

	"rosieos/rt-core/tools/natspublisher/publisher"
)

func TestPermanentExitCodesMatchUnit(t *testing.T) {
	unit, err := os.ReadFile("../../host/rosie-rt-natspublisher.service")
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(unit), "Restart=on-failure\n") || !strings.Contains(string(unit), "RestartPreventExitStatus=2 10 11 12 13 14 15\n") {
		t.Fatal("permanent failures can crash-loop")
	}
	for _, tc := range []struct {
		err  error
		code int
	}{
		{errConfiguration, 2}, {publisher.ErrContract, 10}, {publisher.ErrAuthRequired, 11},
		{publisher.ErrTLSRequired, 12}, {publisher.ErrJetStreamUnavailable, 13},
		{publisher.ErrStreamConfig, 14}, {publisher.ErrPublisherActive, 15},
		{errors.New("ordinary failure"), 1},
	} {
		if got := exitCode(fmt.Errorf("wrapped: %w", tc.err)); got != tc.code {
			t.Fatalf("%v: exit=%d want=%d", tc.err, got, tc.code)
		}
	}
	if exitCode(nil) != 0 {
		t.Fatal("success is not zero")
	}
}

func TestCadenceEnvironmentAndFlag(t *testing.T) {
	for _, tc := range []struct {
		env  string
		args []string
		want string
	}{
		{"", nil, "500ms"}, {"750ms", nil, "750ms"}, {"750ms", []string{"--cadence", "250ms"}, "250ms"},
	} {
		t.Setenv("ROSIE_RT_NATS_CADENCE", tc.env)
		args := append([]string{"--print-env", "--nats-url", "nats://127.0.0.1:4222", "--host", "other-relay"}, tc.args...)
		out, err := exec.Command("../../build/rt-natspublisher", args...).CombinedOutput()
		if err != nil || !strings.Contains(string(out), "ROSIE_RT_NATS_CADENCE=\""+tc.want+"\"\n") {
			t.Fatalf("cadence environment: %s %v", out, err)
		}
	}
	for _, cadence := range []string{"0ms", "-1ms", "1us", "5001ms", "bad\nHOST=other"} {
		err := run(context.Background(), []string{"--print-env", "--host", "other-relay", "--nats-url", "nats://127.0.0.1:4222", "--cadence", cadence})
		if !errors.Is(err, errConfiguration) {
			t.Fatalf("invalid cadence: %q %v", cadence, err)
		}
	}
}

// TestOptionalPackageWithoutPublisher stages the real package with its optional
// binary absent in an isolated fixture, preventing an unconditional install or
// manifest requirement without disturbing binaries used by other package tests.
func TestOptionalPackageWithoutPublisher(t *testing.T) {
	root, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	scratch, err := os.MkdirTemp(filepath.Join(root, "build"), "optional-publisher-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(scratch) })
	fixture := filepath.Join(scratch, "component")
	copyFile := func(relative string) {
		t.Helper()
		target := filepath.Join(fixture, relative)
		if err := os.MkdirAll(filepath.Dir(target), 0755); err != nil {
			t.Fatal(err)
		}
		in, err := os.Open(filepath.Join(root, relative))
		if err != nil {
			t.Fatal(err)
		}
		defer in.Close()
		info, err := in.Stat()
		if err != nil {
			t.Fatal(err)
		}
		out, err := os.OpenFile(target, os.O_CREATE|os.O_WRONLY, info.Mode().Perm())
		if err != nil {
			t.Fatal(err)
		}
		defer out.Close()
		if _, err := io.Copy(out, in); err != nil {
			t.Fatal(err)
		}
	}
	for _, relative := range []string{"go.mod", "go.sum", "tools/package-runtime.sh", "tools/build-igh-userlib.sh", "tools/verify-package.sh", "build/rosie-rt-core-sim", "build/rt-control", "build/rtctl"} {
		copyFile(relative)
	}
	for _, directory := range []string{"tools/packageinfo", "tools/rtctl", "ipcclient", "sdk", "adapters", "internal", "host", "config"} {
		if err := os.CopyFS(filepath.Join(fixture, directory), os.DirFS(filepath.Join(root, directory))); err != nil {
			t.Fatal(err)
		}
	}
	// The robot descriptions sit beside the component, where the package script
	// and the component's go.mod replace both look for them.
	if err := os.CopyFS(filepath.Join(scratch, "robot_description"), os.DirFS(filepath.Join(filepath.Dir(root), "robot_description"))); err != nil {
		t.Fatal(err)
	}
	if err := os.Symlink(filepath.Join(root, "build/deps"), filepath.Join(fixture, "build/deps")); err != nil {
		t.Fatal(err)
	}
	// Read the canonical worktree's identity through its existing WSL git helper.
	gitHelper := strings.ReplaceAll(filepath.Join(root, "tools/packageinfo/git.sh"), "'", "'\\''")
	if err := os.WriteFile(filepath.Join(fixture, "tools/packageinfo/git.sh"), []byte("#!/bin/bash\nexec bash '"+gitHelper+"' \"$@\"\n"), 0755); err != nil {
		t.Fatal(err)
	}
	destination := filepath.Join(scratch, "package")
	command := exec.Command("bash", "tools/package-runtime.sh", destination)
	command.Dir = fixture
	command.Env = append(os.Environ(), "ROSIE_RT_PACKAGE_CORE=build/rosie-rt-core-sim")
	if out, err := command.CombinedOutput(); err != nil {
		t.Fatalf("optional package: %v\n%s", err, out)
	}
	if _, err := os.Stat(filepath.Join(destination, "bin/rt-natspublisher")); !os.IsNotExist(err) {
		t.Fatalf("optional binary unexpectedly present: %v", err)
	}
	unit, err := os.ReadFile(filepath.Join(destination, "host/rosie-rt-natspublisher.service"))
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(unit), "ConditionFileIsExecutable=/opt/rosie-rt-core/current/bin/rt-natspublisher\n") {
		t.Fatal("optional unit is not installed with relocated binary condition")
	}
	// Verify with the packaged inspector: the source tree's build/rt-package may
	// predate the manifest fields the fresh package writes.
	command = exec.Command("bash", filepath.Join(destination, "tools/verify-package.sh"), destination)
	command.Dir = root
	if out, err := command.CombinedOutput(); err != nil {
		t.Fatalf("optional package verification: %v\n%s", err, out)
	}
}
