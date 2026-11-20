package rtctl

import (
	"bytes"
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strconv"
	"testing"
)

func TestRunLauncherHelper(t *testing.T) {
	if os.Getenv("RTCTL_RUN_HELPER") != "1" {
		return
	}
	for i, arg := range os.Args {
		if arg == "--" {
			os.Exit(RunCLI(os.Args[i+1:], os.Getenv("RTCTL_RUN_ROOT"), os.Stdout, os.Stderr))
		}
	}
	os.Exit(98)
}

// TestRunReplacesLauncher verifies PID identity and exact native arguments;
// a launcher subprocess would leave systemd supervising the wrong process.
func TestRunReplacesLauncher(t *testing.T) {
	for _, tc := range []struct {
		backend  string
		packaged bool
	}{{"simulation", false}, {"ipc-only", false}, {"live", false}, {"live", true}} {
		t.Run(tc.backend+strconv.FormatBool(tc.packaged), func(t *testing.T) {
			backend := tc.backend
			root, machine := commandFixture(t)
			f := fresh()
			if backend == "live" {
				f.put("machine.backend", `"live"`)
				// Unverified physical fixture: explicit 1000-count encoder and 1/1 gearing.
				f.put("machine.axes.0.motor_encoder_counts_per_rev", "1000")
				f.put("machine.axes.0.gear_ratio", `{"numerator":1,"denominator":1}`)
				f.put("machine.axes.0.position_tracking_mode", `"continuous"`)
				writeTestFile(t, machine, canonical(f.get("machine")), 0o644)
			}
			suffix := map[string]string{"simulation": "-sim", "ipc-only": "-ipc", "live": ""}[backend]
			binary := filepath.Join(root, "build", "rosie-rt-core"+suffix)
			if tc.packaged {
				writeTestFile(t, filepath.Join(root, "bin", "rtctl"), "", 0o755)
				binary = filepath.Join(root, "bin", "rosie-rt-core")
			}
			daemonStub(t, binary)
			capture, pidPath := filepath.Join(root, "capture.json"), filepath.Join(root, "pid")
			t.Setenv("RTCTL_RUN_HELPER", "1")
			t.Setenv("RTCTL_RUN_ROOT", root)
			t.Setenv("RTCTL_TEST_CAPTURE", capture)
			t.Setenv("RTCTL_TEST_PID_CAPTURE", pidPath)
			t.Setenv("RTCTL_TEST_STATUS", "23")
			exe, err := os.Executable()
			if err != nil {
				t.Fatal(err)
			}
			out, socket := filepath.Join(root, "output space"), filepath.Join(root, "native.sock")
			cmd := exec.Command(exe, "-test.run=^TestRunLauncherHelper$", "--", "run", "--config", machine, "--backend", backend, "--out", out, "--socket", socket)
			output, err := cmd.CombinedOutput()
			var exited *exec.ExitError
			if !errors.As(err, &exited) || exited.ExitCode() != 23 {
				t.Fatalf("daemon exit: %v %s", err, output)
			}
			pid, err := os.ReadFile(pidPath)
			if err != nil || string(pid) != strconv.Itoa(cmd.Process.Pid) {
				t.Fatalf("launcher was not replaced: pid %s want %d: %v", pid, cmd.Process.Pid, err)
			}
			data, err := os.ReadFile(capture)
			if err != nil {
				t.Fatal(err)
			}
			var got []string
			if err := json.Unmarshal(data, &got); err != nil {
				t.Fatal(err)
			}
			compiled, err := f.compile()
			if err != nil {
				t.Fatal(err)
			}
			want := append(compiled.Argv, "--machine-sha256", compiled.MachineSHA256, "--deployment-sha256", compiled.DeploymentSHA256,
				"--axis-profiles", filepath.Join(out, compiled.SHA256, "axes.conf"), "--configuration-hash", compiled.SHA256)
			published, err := os.ReadFile(filepath.Join(out, compiled.SHA256, "argv.json"))
			if err != nil {
				t.Fatal(err)
			}
			var saved []string
			if err := json.Unmarshal(published, &saved); err != nil || !reflect.DeepEqual(saved, want) {
				t.Fatalf("published argv: %v %s", err, published)
			}
			want = append(want, "--socket-path", socket)
			if !reflect.DeepEqual(got, want) {
				t.Fatalf("daemon argv: %q want %q", got, want)
			}
		})
	}
}

func TestRunRequiresExactLiveBackendAndRefusesPackagedFallback(t *testing.T) {
	root, machine := commandFixture(t)
	f := fresh()
	f.remove("machine.backend")
	f.put("machine.Backend", `"live"`)
	writeTestFile(t, machine, canonical(f.get("machine")), 0o644)
	out := filepath.Join(root, "unpublished")
	var output bytes.Buffer
	if code := RunCLI([]string{"run", "--config", machine, "--backend", "live", "--out", out}, root, &output, &output); code != 1 {
		t.Fatalf("case-insensitive live admission: %d %s", code, output.String())
	}
	writeTestFile(t, filepath.Join(root, "bin", "rtctl"), "", 0o755)
	for _, backend := range []string{"simulation", "ipc-only"} {
		if code := RunCLI([]string{"run", "--config", machine, "--backend", backend, "--out", out}, root, &output, &output); code != 2 {
			t.Fatalf("packaged fallback accepted: %d %s", code, output.String())
		}
	}
	if _, err := os.Stat(out); !os.IsNotExist(err) {
		t.Fatalf("refused backend published output: %v", err)
	}
}

func TestRunRefusesUnsafeConfigurationBeforePublication(t *testing.T) {
	root, machine := commandFixture(t)
	out := filepath.Join(root, "unpublished")
	for _, args := range [][]string{
		{"run", "--config", machine, "--backend", "live", "--out", out},
		{"run", "--config", machine, "--backend", "automatic", "--out", out},
		{"run", "--config", machine, "--backend", "simulation", "--socket", "", "--out", out},
	} {
		var output bytes.Buffer
		if RunCLI(args, root, &output, &output) == 0 {
			t.Fatalf("accepted %q", args)
		}
		if _, err := os.Stat(out); !os.IsNotExist(err) {
			t.Fatalf("refused launch published output: %v", err)
		}
	}
	writeTestFile(t, machine, `{}`, 0o644)
	var output bytes.Buffer
	if RunCLI([]string{"run", "--config", machine, "--backend", "simulation", "--out", out}, root, &output, &output) != 1 {
		t.Fatal("invalid machine accepted")
	}
	if _, err := os.Stat(out); !os.IsNotExist(err) {
		t.Fatalf("invalid machine published output: %v", err)
	}
}
