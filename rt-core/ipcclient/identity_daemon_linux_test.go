//go:build linux

package ipcclient_test

import (
	"context"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"syscall"
	"testing"
	"time"

	ipc "rosieos/rt-core/ipcclient"
	"rosieos/rt-core/tools/rtctl"
)

// Unverified process watchdog; successful assertions require actual exit/status.
const identityWatchdog = 8 * time.Second

type identityFixture struct {
	binary, directory, socket string
	compiled                  rtctl.Compiled
	args                      []string
}

func identityCompile(t *testing.T, edit func(map[string]any)) identityFixture {
	t.Helper()
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "identity-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	data, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation-home.json"))
	if err != nil {
		t.Fatal(err)
	}
	var machine map[string]any
	if err := json.Unmarshal(data, &machine); err != nil {
		t.Fatal(err)
	}
	if edit != nil {
		edit(machine)
	}
	data, err = json.Marshal(machine)
	if err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(dir, "machine.json")
	if err := os.WriteFile(path, data, 0600); err != nil {
		t.Fatal(err)
	}
	compiled, err := simfixture.CompileRaw(data, filepath.Join(root, "config/drives"), rtctl.Compile)
	if err != nil {
		t.Fatal(err)
	}
	directory, args, err := simfixture.Compile(root, path, dir)
	if err != nil {
		t.Fatal(err)
	}
	if os.Getenv("TMPDIR") == "" {
		t.Fatal("TMPDIR is required for Unix sockets")
	}
	sockets, err := os.MkdirTemp("", "id-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(sockets) })
	binary := simfixture.Locate(root, "rosie-rt-core-sim")
	return identityFixture{binary: binary, directory: directory, socket: filepath.Join(sockets, "s"), compiled: compiled, args: args}
}

func (f identityFixture) validate(t *testing.T, args ...string) (string, error) {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), identityWatchdog)
	defer cancel()
	argv := append(append([]string{}, f.args...), "--validate-config")
	out, err := exec.CommandContext(ctx, f.binary, append(argv, args...)...).CombinedOutput()
	if ctx.Err() != nil {
		t.Fatalf("startup validation watchdog: %s", out)
	}
	return string(out), err
}

func TestDaemonRefusesMismatchedMachineDigest(t *testing.T) {
	for _, tc := range []struct {
		name, old, new, field string
		args                  []string
	}{
		{name: "limit", old: "limits=-0.5|0.5", new: "limits=-0.4|0.5", field: "axis.0.limits"},
		{name: "gearing", args: []string{"--gear-ratio", "2"}, field: "wire.revs_per_axis_rev"},
		{name: "cycle", args: []string{"--cycle-ns", "2000000"}, field: "cycle_ns"},
		{name: "digest", args: []string{"--machine-sha256", strings.Repeat("0", 64)}, field: "machine_sha256"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			f := identityCompile(t, nil)
			if tc.old != "" {
				changed := strings.Replace(f.compiled.AxisProfiles, tc.old, tc.new, 1)
				if changed == f.compiled.AxisProfiles {
					t.Fatal("fixture field absent")
				}
				if err := os.WriteFile(filepath.Join(f.directory, "axes.conf"), []byte(changed), 0600); err != nil {
					t.Fatal(err)
				}
			}
			out, err := f.validate(t, tc.args...)
			exit, ok := err.(*exec.ExitError)
			message := "machine identity mismatch: " + tc.field
			if tc.name == "gearing" {
				// Coordinate admission identifies the wire-scale change first.
				message = "coordinate identity mismatch axis=0 field=" + tc.field
			}
			if !ok || exit.ExitCode() != 2 || !strings.Contains(out, message) {
				t.Fatalf("want exit 2 and field %s: %v %s", tc.field, err, out)
			}
		})
	}
}

func (f identityFixture) start(t *testing.T, simulationTest bool) *ipc.Client {
	t.Helper()
	log, err := os.Create(filepath.Join(f.directory, "daemon.log"))
	if err != nil {
		t.Fatal(err)
	}
	command := exec.Command(f.binary, append(append([]string{}, f.args...), "--socket-path", f.socket)...)
	command.Stdout, command.Stderr = log, log
	if err := simfixture.IsolateCommand(command); err != nil {
		log.Close()
		t.Fatal(err)
	}
	if err := command.Start(); err != nil {
		log.Close()
		t.Fatal(err)
	}
	done := make(chan error, 1)
	go func() { done <- command.Wait() }()
	var client *ipc.Client
	t.Cleanup(func() {
		if client != nil {
			_ = client.Close()
		}
		_ = command.Process.Signal(syscall.SIGTERM)
		select {
		case <-done:
		case <-time.After(identityWatchdog):
			_ = command.Process.Kill()
			<-done
			t.Error("shutdown watchdog")
		}
		_ = log.Close()
		if t.Failed() {
			data, _ := os.ReadFile(log.Name())
			t.Log(string(data))
		}
	})
	ctx, cancel := context.WithTimeout(context.Background(), identityWatchdog)
	defer cancel()
	ticker := time.NewTicker(time.Millisecond)
	defer ticker.Stop()
	for {
		data, _ := os.ReadFile(filepath.Join(filepath.Dir(f.socket), "metrics.json"))
		var status ipc.Status
		if json.Unmarshal(data, &status) == nil && status.ConfigVerifiedMask == 1 {
			if status.HomeValidMask != 0 {
				t.Fatal("restart restored Home without evidence")
			}
			break
		}
		select {
		case err := <-done:
			done <- err
			t.Fatalf("daemon exited: %v", err)
		case <-ctx.Done():
			t.Fatal("configuration verification watchdog")
		case <-ticker.C:
		}
	}
	client, err = ipc.Dial(ctx, ipc.Options{SocketPath: f.socket, ConfigurationSHA256: f.compiled.SHA256,
		MachineSHA256: f.compiled.MachineSHA256, Backend: "simulation", SimulationTest: simulationTest})
	if err != nil {
		t.Fatal(err)
	}
	return client
}

func TestDaemonAcceptsDeploymentChangeWithSameMachineDigest(t *testing.T) {
	baseline := identityCompile(t, nil)
	changed := identityCompile(t, func(machine map[string]any) {
		machine["runtime"].(map[string]any)["cpu"] = 3
		machine["socket_path"] = "/run/relocated.sock"
		machine["host_name"] = "relocated-host"
	})
	if baseline.compiled.MachineSHA256 != changed.compiled.MachineSHA256 || baseline.compiled.DeploymentSHA256 == changed.compiled.DeploymentSHA256 {
		t.Fatal("deployment partition failed")
	}
	for index, f := range []identityFixture{baseline, changed} {
		t.Run([]string{"before_restart", "after_deployment_change"}[index], func(t *testing.T) {
			out, err := f.validate(t, "--socket-path", f.socket)
			if err != nil {
				t.Fatalf("deployment rejected: %v %s", err, out)
			}
			client := f.start(t, true)
			description, err := client.Describe()
			if err != nil {
				t.Fatal(err)
			}
			if description.MachineSHA256 != baseline.compiled.MachineSHA256 || description.DeploymentSHA256 != f.compiled.DeploymentSHA256 {
				t.Fatal("published wrong identities")
			}
			if index == 0 {
				ctx, cancel := context.WithTimeout(context.Background(), identityWatchdog)
				defer cancel()
				if _, err := client.Enable(ctx, 1); err != nil {
					t.Fatal(err)
				}
				if _, err := client.Home(ctx, 1); err != nil {
					t.Fatal(err)
				}
				if _, err := client.Wait(ctx, func(s ipc.Status) bool { return s.HomeValidMask == 1 }); err != nil {
					t.Fatal(err)
				}
			}
		})
	}
}

func TestDescribeReportsBothDigests(t *testing.T) {
	f := identityCompile(t, nil)
	c := f.start(t, true)
	description, err := c.Describe()
	if err != nil {
		t.Fatal(err)
	}
	if description.ConfigurationSHA256 != f.compiled.SHA256 || description.MachineSHA256 != f.compiled.MachineSHA256 || description.DeploymentSHA256 != f.compiled.DeploymentSHA256 {
		t.Fatalf("wrong description identities: %+v", description)
	}
}

func TestDaemonComputesMachineDigestForLegacyInvocation(t *testing.T) {
	f := identityCompile(t, nil)
	f.args = append(append([]string{}, f.compiled.Argv...), "--axis-profiles", filepath.Join(f.directory, "axes.conf"), "--configuration-hash", f.compiled.SHA256)
	out, err := f.validate(t)
	if err != nil || !strings.Contains(out, "machine_sha256="+f.compiled.MachineSHA256) {
		t.Fatalf("legacy invocation skipped computation: %v %s", err, out)
	}
}
