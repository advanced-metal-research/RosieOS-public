package rtctl

import (
	"bytes"
	"encoding/json"
	"os"
	"path/filepath"
	"reflect"
	"strconv"
	"strings"
	"testing"
)

func writeTestFile(t *testing.T, path, text string, mode os.FileMode) {
	t.Helper()
	if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte(text), mode); err != nil {
		t.Fatal(err)
	}
}

func commandFixture(t *testing.T) (root, machine string) {
	t.Helper()
	root = buildTemp(t)
	f := fresh()
	machine = filepath.Join(root, "machine.json")
	writeTestFile(t, machine, canonical(f.get("machine")), 0o644)
	writeTestFile(t, filepath.Join(root, "config/drives", "test.json"), canonical(f.get("profile")), 0o644)
	return root, machine
}

// TestDaemonHelper captures exactly the process argv; the parent tests verify
// validate-only dispatch and returned exit status without contacting hardware.
func TestDaemonHelper(t *testing.T) {
	capture := os.Getenv("RTCTL_TEST_CAPTURE")
	if capture == "" {
		return
	}
	var args []string
	for i, arg := range os.Args {
		if arg == "--" {
			args = os.Args[i+1:]
			break
		}
	}
	data, err := json.Marshal(args)
	if err != nil {
		os.Exit(98)
	}
	if err := os.WriteFile(capture, data, 0o644); err != nil {
		os.Exit(98)
	}
	if path := os.Getenv("RTCTL_TEST_PID_CAPTURE"); path != "" {
		if err := os.WriteFile(path, []byte(strconv.Itoa(os.Getpid())), 0o644); err != nil {
			os.Exit(98)
		}
	}
	status, err := strconv.Atoi(os.Getenv("RTCTL_TEST_STATUS"))
	if err != nil {
		os.Exit(98)
	}
	os.Exit(status)
}

func daemonStub(t *testing.T, path string) {
	t.Helper()
	exe, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	quoted := "'" + strings.ReplaceAll(exe, "'", "'\"'\"'") + "'"
	writeTestFile(t, path, "#!/bin/sh\nexec "+quoted+" -test.run=^TestDaemonHelper$ -- \"$@\"\n", 0o755)
}

func TestValidateDispatchAndExitStatus(t *testing.T) {
	for _, tc := range []struct {
		backend, suffix string
		status          int
		custom          bool
	}{
		{"simulation", "-sim", 0, false}, {"ipc-only", "-ipc", 7, false}, {"live", "", 23, false}, {"simulation", "", 42, true},
	} {
		t.Run(tc.backend+strconv.Itoa(tc.status), func(t *testing.T) {
			root, machine := commandFixture(t)
			binary := filepath.Join(root, "build", "rosie-rt-core"+tc.suffix)
			if tc.custom {
				binary = filepath.Join(root, "custom daemon")
			}
			daemonStub(t, binary)
			capture := filepath.Join(root, "capture.json")
			t.Setenv("RTCTL_TEST_CAPTURE", capture)
			t.Setenv("RTCTL_TEST_STATUS", strconv.Itoa(tc.status))
			args := []string{"validate", "--config", machine, "--backend", tc.backend}
			if tc.custom {
				args = append(args, "--binary", binary)
			}
			var stdout, stderr bytes.Buffer
			status := RunCLI(args, root, &stdout, &stderr)
			if status != tc.status {
				t.Fatalf("status %d != %d: %s", status, tc.status, stderr.String())
			}
			data, err := os.ReadFile(capture)
			if err != nil {
				t.Fatal(err)
			}
			var got []string
			if err := json.Unmarshal(data, &got); err != nil {
				t.Fatal(err)
			}
			f := fresh()
			compiled, err := f.compile()
			if err != nil {
				t.Fatal(err)
			}
			directory := filepath.Join(root, "build", "config", compiled.SHA256)
			want := append(compiled.Argv, "--machine-sha256", compiled.MachineSHA256,
				"--deployment-sha256", compiled.DeploymentSHA256,
				"--axis-profiles", filepath.Join(directory, "axes.conf"), "--configuration-hash", compiled.SHA256, "--validate-config")
			if !reflect.DeepEqual(got, want) {
				t.Fatalf("daemon argv differs\n%q\n%q", got, want)
			}
			if !strings.Contains(stdout.String()+stderr.String(), "daemon exit status: "+strconv.Itoa(tc.status)) {
				t.Fatal("daemon status not reported")
			}
		})
	}
}

func TestCompileCommandAndProfileOverride(t *testing.T) {
	root, machine := commandFixture(t)
	profiles := filepath.Join(root, "alternate profiles")
	if err := os.Rename(filepath.Join(root, "config/drives"), profiles); err != nil {
		t.Fatal(err)
	}
	out := filepath.Join(root, "output space é")
	var stdout, stderr bytes.Buffer
	if status := RunCLI([]string{"compile", "--config", machine, "--profiles", profiles, "--out", out}, root, &stdout, &stderr); status != 0 {
		t.Fatalf("compile exit %d: %s", status, stderr.String())
	}
	f := fresh()
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	directory := filepath.Join(out, compiled.SHA256)
	if stdout.String() != directory+"\n" {
		t.Fatalf("directory output: %q", stdout.String())
	}
	data, err := os.ReadFile(filepath.Join(directory, "argv.json"))
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Contains(data, []byte(`\u00e9`)) {
		t.Fatal("argv path must use Python ensure_ascii")
	}
	if bytes.Contains(data, []byte{'\r'}) {
		t.Fatal("output has non-LF endings")
	}
}

func TestCommandRejectsInvalidArguments(t *testing.T) {
	root, machine := commandFixture(t)
	for _, args := range [][]string{nil, {"run"}, {"compile"}, {"compile", "--config", machine},
		{"compile", "--config", machine, "--out", filepath.Join(root, "out"), "extra"},
		{"validate", "--config", machine}, {"validate", "--config", machine, "--backend", "fallback"},
		{"compile", "--unknown"},
	} {
		var stdout, stderr bytes.Buffer
		if code := RunCLI(args, root, &stdout, &stderr); code != 2 {
			t.Errorf("%q: expected usage status 2, got %d", args, code)
		}
	}
	var stdout, stderr bytes.Buffer
	if code := RunCLI([]string{"validate", "--config", machine, "--backend", "simulation", "--binary", filepath.Join(root, "absent")}, root, &stdout, &stderr); code != 1 {
		t.Fatalf("missing binary: status %d", code)
	}
}

func TestInvalidCompileDoesNotPublish(t *testing.T) {
	root, machine := commandFixture(t)
	f := fresh()
	f.put("machine.axes.0.limits.max_target_lead", "0")
	writeTestFile(t, machine, canonical(f.get("machine")), 0o644)
	out := filepath.Join(root, "unpublished")
	var stdout, stderr bytes.Buffer
	if code := RunCLI([]string{"compile", "--config", machine, "--out", out}, root, &stdout, &stderr); code != 1 {
		t.Fatalf("invalid compile: status %d", code)
	}
	if _, err := os.Stat(out); !os.IsNotExist(err) {
		t.Fatalf("invalid input created output directory: %v", err)
	}
}

func TestWriteConfigResolvesSymlinks(t *testing.T) {
	root, machine := commandFixture(t)
	real := filepath.Join(root, "real")
	if err := os.Mkdir(real, 0o755); err != nil {
		t.Fatal(err)
	}
	alias := filepath.Join(root, "alias")
	if err := os.Symlink(real, alias); err != nil {
		t.Fatal(err)
	}
	dir, argv, err := WriteConfig(machine, filepath.Join(root, "config/drives"), filepath.Join(alias, "nested"))
	if err != nil {
		t.Fatal(err)
	}
	if !strings.HasPrefix(dir, real+string(filepath.Separator)) || argv[len(argv)-3] != filepath.Join(dir, "axes.conf") {
		t.Fatal("output path was not resolved")
	}
}

func TestDefaultRoot(t *testing.T) {
	root, err := DefaultRoot()
	if err != nil {
		t.Fatal(err)
	}
	if root != testRoot(t) {
		t.Fatalf("default root %q != %q", root, testRoot(t))
	}
}
