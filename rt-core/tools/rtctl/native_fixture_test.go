package rtctl

import (
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"sync"
	"testing"

	"rosieos/rt-core/internal/simfixture"
)

// Native C++ fixtures are single translation units compiled from the checked-in
// sources. Measured 2026-10-07 in the Mac's arm64 rosie-rtcore-test:24.04b
// container: bus_bringup_tests.cpp -O0 about 4 s, bus_diagnostics_tests.cpp
// and drive_cap_tests.cpp -O0 about 6 s each, bus_change_daemon.cpp -O2 (all of
// src/main.cpp) about 28 s; example-relay took roughly three times as long. Four
// tests compiled the same bring-up fixture and five ran it on two identical
// configurations. A binary is a pure function of its sources and flags within
// one package run, so each is compiled once and every caller shares it; the
// assertions each test makes on the binary and its output are unchanged.
var nativeFixtures struct {
	sync.Mutex
	dir    string
	builds map[string]*nativeBuild
	runs   map[string]*nativeRun
}

type nativeBuild struct {
	once   sync.Once
	argv   string
	path   string
	output []byte
	err    error
}

type nativeRun struct {
	once   sync.Once
	output []byte
	err    error
}

func TestMain(m *testing.M) {
	restore, err := simfixture.ReserveSimulationCPU()
	if err != nil {
		panic(err)
	}
	code := m.Run()
	restore()
	if nativeFixtures.dir != "" {
		os.RemoveAll(nativeFixtures.dir)
	}
	os.Exit(code)
}

// nativeTestBinary compiles one g++ fixture per package run. args receives the
// output path and returns the complete g++ argument list, run in the rt-core
// root. A second caller naming the same fixture with different arguments fails
// rather than silently testing the first caller's binary.
func nativeTestBinary(t *testing.T, name string, args func(output string) []string) (string, []byte, error) {
	t.Helper()
	root := testRoot(t)
	nativeFixtures.Lock()
	if nativeFixtures.dir == "" {
		parent := filepath.Join(root, "build")
		if err := os.MkdirAll(parent, 0o755); err != nil {
			nativeFixtures.Unlock()
			t.Fatal(err)
		}
		dir, err := os.MkdirTemp(parent, "rtctl-native-")
		if err != nil {
			nativeFixtures.Unlock()
			t.Fatal(err)
		}
		nativeFixtures.dir = dir
	}
	if nativeFixtures.builds == nil {
		nativeFixtures.builds = map[string]*nativeBuild{}
	}
	path := filepath.Join(nativeFixtures.dir, name)
	argv := args(path)
	build := nativeFixtures.builds[name]
	if build == nil {
		build = &nativeBuild{argv: strings.Join(argv, "\x00"), path: path}
		nativeFixtures.builds[name] = build
	}
	nativeFixtures.Unlock()
	if build.argv != strings.Join(argv, "\x00") {
		t.Fatalf("native fixture %s requested with different arguments: %q", name, argv)
	}
	build.once.Do(func() {
		cmd := exec.Command("g++", argv...)
		cmd.Dir = root
		build.output, build.err = cmd.CombinedOutput()
	})
	return build.path, build.output, build.err
}

// nativeRunOnce executes binary with args once per key and returns the shared
// combined output. The key must identify everything the output depends on
// besides the binary (for a compiled configuration, its SHA256), so callers
// sharing a run assert on exactly the bytes their own invocation would see.
func nativeRunOnce(binary, key string, args []string) ([]byte, error) {
	nativeFixtures.Lock()
	if nativeFixtures.runs == nil {
		nativeFixtures.runs = map[string]*nativeRun{}
	}
	id := binary + "\x00" + key
	run := nativeFixtures.runs[id]
	if run == nil {
		run = &nativeRun{}
		nativeFixtures.runs[id] = run
	}
	nativeFixtures.Unlock()
	run.once.Do(func() {
		run.output, run.err = exec.Command(binary, args...).CombinedOutput()
	})
	return run.output, run.err
}
