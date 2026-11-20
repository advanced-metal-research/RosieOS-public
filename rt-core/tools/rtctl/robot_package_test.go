package rtctl_test

import (
	"context"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/tools/rtctl"
)

// robotPackageRuntimeDir keeps native sockets on a short POSIX path. The
// telemetry reader socket adds a suffix that can exceed AF_UNIX's path limit
// when the release gate sets TMPDIR inside its deployment scratch directory.
func robotPackageRuntimeDir(t *testing.T) string {
	t.Helper()
	dir, err := os.MkdirTemp("/tmp", "rp-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := os.RemoveAll(dir); err != nil {
			t.Error(err)
		}
	})
	return dir
}

// TestRobotPackagedConfigurations exercises the real sim package, both migrated
// cells and a native handshake; a checkout-only compiler cannot satisfy it.
func TestRobotPackagedConfigurations(t *testing.T) {
	component, err := filepath.Abs("../..")
	if err != nil {
		t.Fatal(err)
	}
	for _, file := range []string{"build/rosie-rt-core-sim", "build/rtctl", "build/rt-control", "build/deps/igh-prefix/lib/libethercat.so.1"} {
		if _, err := os.Stat(filepath.Join(component, file)); err != nil {
			t.Skipf("requires make control sim and tools/build-igh-userlib.sh: %v", err)
		}
	}
	scratch, err := os.MkdirTemp(filepath.Join(component, "build"), "robot-package-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(scratch) })
	root := filepath.Join(scratch, "relocated")
	cmd := exec.Command("bash", "tools/package-runtime.sh", root)
	cmd.Dir = component
	cmd.Env = append(os.Environ(), "ROSIE_RT_PACKAGE_CORE=build/rosie-rt-core-sim")
	if output, err := cmd.CombinedOutput(); err != nil {
		t.Fatalf("package: %v\n%s", err, output)
	}
	manifestData, err := os.ReadFile(filepath.Join(root, "rt_package.json"))
	if err != nil {
		t.Fatal(err)
	}
	var m struct {
		RobotDescriptions []struct {
			ModelID                string `json:"model_id"`
			Path                   string `json:"path"`
			RobotDescriptionSHA256 string `json:"robot_description_sha256"`
		} `json:"robot_descriptions"`
	}
	if err := json.Unmarshal(manifestData, &m); err != nil {
		t.Fatal(err)
	}
	descriptions, err := filepath.Glob(filepath.Join(component, "..", "robot_description", "robots", "*", "robot_description_manifest.json"))
	if err != nil {
		t.Fatal(err)
	}
	if len(descriptions) == 0 || len(m.RobotDescriptions) != len(descriptions) {
		t.Fatalf("expected every description, including the unused ones (%d in the repository): %+v", len(descriptions), m.RobotDescriptions)
	}
	verifyPackage := func(want bool) {
		t.Helper()
		cmd := exec.Command("bash", filepath.Join(root, "tools/verify-package.sh"), root)
		if output, err := cmd.CombinedOutput(); (err == nil) != want {
			t.Fatalf("verify want success=%t: %v\n%s", want, err, output)
		}
	}
	verifyPackage(true)
	if _, err := os.Stat(filepath.Join(root, "config/machines/bench-ninemotor-a6ec.json")); !os.IsNotExist(err) {
		t.Fatalf("retired machine shipped in runtime package: %v", err)
	}
	var jetson rtctl.Compiled
	var daemonArgv []string
	for _, cell := range []string{"bench/rosie1420-six-motors.json", "bench/nine-motors-1to1.json"} {
		file := filepath.Join(root, "config/machines", cell)
		data, err := os.ReadFile(file)
		if err != nil {
			t.Fatal(err)
		}
		compiled, err := rtctl.Compile(data, filepath.Join(root, "config/drives"))
		if err != nil {
			t.Fatalf("packaged %s: %v", cell, err)
		}
		source, err := rtctl.Compile(data, filepath.Join(component, "config/drives"))
		if err != nil || !reflect.DeepEqual(compiled, source) {
			t.Fatalf("relocation changed %s compilation: %v", cell, err)
		}
		cmd := exec.Command(filepath.Join(root, "bin/rtctl"), "compile", "--config", file, "--out", filepath.Join(scratch, "compiled"))
		cmd.Dir = root
		output, err := cmd.Output()
		if err != nil {
			t.Fatalf("packaged CLI %s: %v", cell, err)
		}
		if filepath.Base(strings.TrimSpace(string(output))) != compiled.SHA256 {
			t.Fatal("packaged CLI identity differs")
		}
		if cell == "bench/rosie1420-six-motors.json" {
			jetson = compiled
			data, err := os.ReadFile(filepath.Join(strings.TrimSpace(string(output)), "argv.json"))
			if err != nil {
				t.Fatal(err)
			}
			if err := json.Unmarshal(data, &daemonArgv); err != nil {
				t.Fatal(err)
			}
		}
	}
	t.Run("daemon", func(t *testing.T) {
		socket := filepath.Join(robotPackageRuntimeDir(t), "ipc.sock")
		// Unverified test admission budget: 10000 ms; readiness is an observed
		// six-axis native status with the independently compiled identity.
		ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
		defer cancel()
		args := append(daemonArgv, "--socket-path", socket, "--telemetry-directory", filepath.Join(scratch, "telemetry"))
		cmd := exec.CommandContext(ctx, filepath.Join(root, "bin/rosie-rt-core"), args...)
		cmd.Dir = root
		log, err := os.Create(filepath.Join(scratch, "daemon.log"))
		if err != nil {
			t.Fatal(err)
		}
		defer log.Close()
		cmd.Stdout, cmd.Stderr = log, log
		if err := cmd.Start(); err != nil {
			t.Fatal(err)
		}
		defer func() {
			cmd.Process.Kill()
			cmd.Wait()
			if t.Failed() {
				data, _ := os.ReadFile(log.Name())
				t.Log(string(data))
			}
		}()
		// Unverified polling cadence: 5 ms; elapsed time never establishes readiness.
		ticker := time.NewTicker(5 * time.Millisecond)
		defer ticker.Stop()
		var client *ipcclient.Client
		for {
			client, err = ipcclient.Dial(ctx, ipcclient.Options{SocketPath: socket, ConfigurationSHA256: jetson.SHA256, MachineSHA256: jetson.MachineSHA256, Backend: "simulation"})
			if err == nil {
				break
			}
			select {
			case <-ctx.Done():
				t.Fatalf("packaged daemon handshake: %v", err)
			case <-ticker.C:
			}
		}
		defer client.Close()
		if _, err := client.Wait(ctx, func(status ipcclient.Status) bool {
			return len(status.Axes) == 6 && status.ConfigurationSHA256 == jetson.SHA256 && status.TimeNS > 0
		}); err != nil {
			t.Fatal(err)
		}
	})
	verifyPackage(true)
	resource := filepath.Join(root, m.RobotDescriptions[0].Path, "rtcore_definition.json")
	if err := os.WriteFile(resource, []byte("tampered"), 0644); err != nil {
		t.Fatal(err)
	}
	verifyPackage(false)
	if err := os.Remove(resource); err != nil {
		t.Fatal(err)
	}
	verifyPackage(false)
}
