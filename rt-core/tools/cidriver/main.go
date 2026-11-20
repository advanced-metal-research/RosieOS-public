package main

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/signal"
	"path/filepath"
	"regexp"
	"runtime"
	"strings"
	"syscall"
)

func main() {
	if len(os.Args) == 2 && os.Args[1] == "--check-retired-runtime" {
		if err := checkRetiredRuntime(os.Stdout); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	if len(os.Args) == 3 && os.Args[1] == "--shared-words" {
		if err := atomicInterop(ctx, os.Args[2], os.Stdout); err != nil {
			fmt.Fprintln(os.Stderr, err)
			os.Exit(1)
		}
		return
	}
	root, err := os.Getwd()
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
	d := driver{root: root, path: filepath.Join(root, "build/ci-unknown.json"), logDir: filepath.Join(root, "build/ci"), run: execute, out: os.Stdout,
		record: record{Schema: recordSchema, Status: "running", TSan: qualification{Status: "not_run"}, Atomics: qualification{Status: "not_run", Architecture: runtime.GOARCH}}}
	err = d.finish(func() error {
		if len(os.Args) != 1 {
			return errors.New("cidriver accepts only --shared-words BINARY or no arguments")
		}
		var err error
		d.allow, err = loadAllowlist(allowedSkips)
		if err != nil {
			return err
		}
		d.env = controlledEnvironment(root)
		r, err := d.step(ctx, "source-identity", []string{"bash", "tools/packageinfo/git.sh", "rev-parse", "HEAD"}, nil)
		if err != nil {
			return err
		}
		sha := strings.TrimSpace(r.Stdout)
		if !regexp.MustCompile(`^[0-9a-f]{40}$`).MatchString(sha) {
			return errors.New("missing_git_sha")
		}
		d.record.GitSHA = sha
		d.path = filepath.Join(root, "build/ci-"+sha+".json")
		return d.pipeline(ctx)
	})
	fmt.Printf("CI: %s record=%s tsan=%s atomics=%s\n", d.record.Status, d.path, d.record.TSan.Status, d.record.Atomics.Status)
	if err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

// controlledEnvironment pins the test artifacts and detector settings; inherited dry runs and test filters cannot bypass coverage.
func controlledEnvironment(root string) map[string]string {
	return map[string]string{
		"GOFLAGS": "-mod=mod", "GO_BUILD_FLAGS": "-race", "GOENV": "off", "GOWORK": "off", "GOTOOLCHAIN": "local",
		"GORACE": "halt_on_error=1 exitcode=66 log_path=stderr", "ASAN_OPTIONS": "detect_leaks=1:halt_on_error=1:abort_on_error=1",
		"UBSAN_OPTIONS": "halt_on_error=1:print_stacktrace=1", "TSAN_OPTIONS": "halt_on_error=1", "LSAN_OPTIONS": "",
		"MAKEFLAGS": "", "GNUMAKEFLAGS": "", "MFLAGS": "", "MAKEOVERRIDES": "", "MAKEFILES": "",
		"RTCORE_TEST_BINARY": filepath.Join(root, "build/rosie-rt-core-sim"), "RTCONTROL_TEST_BINARY": filepath.Join(root, "build/rt-control"),
	}
}

func raceDetected(r result) error {
	if r.Code <= 0 || !strings.Contains(r.Stdout+r.Stderr, "WARNING: DATA RACE") {
		return errors.New("race_instrumentation_missing: known data race must fail with detector diagnostic")
	}
	return nil
}
func sanitizerSymbols(r result) error {
	if r.Code != 0 {
		return errors.New("sanitizer_binary_missing_or_unreadable")
	}
	if !regexp.MustCompile(`\b__asan_init\b`).MatchString(r.Stdout) || !strings.Contains(r.Stdout, "__ubsan_handle_") {
		return errors.New("sanitizer_instrumentation_missing: __asan_init and __ubsan_handle_* required")
	}
	return nil
}
func tsanSupported(r result) error {
	if r.Code == 0 {
		return nil
	}
	text := r.Stdout + r.Stderr
	if r.Code > 0 && strings.Contains(text, "FATAL: ThreadSanitizer: unexpected memory mapping") {
		return unsupportedHost{text: text}
	}
	return fmt.Errorf("tsan_probe_failed: exit=%d %s", r.Code, text)
}

func (d *driver) pipeline(ctx context.Context) error {
	run := func(name string, args ...string) error { _, err := d.step(ctx, name, args, nil); return err }
	makeTarget := func(target string) error { return run(target, "make", target, "GO=go") }
	if err := makeTarget("check-python-absent"); err != nil {
		return err
	}
	makefile, err := os.ReadFile(filepath.Join(d.root, "Makefile"))
	if err != nil {
		return err
	}
	hasTarget := func(name string) bool {
		return regexp.MustCompile(`(?m)^` + regexp.QuoteMeta(name) + `\s*:`).Match(makefile)
	}
	if err = makeTarget("check-protocol"); err != nil {
		return err
	}
	if hasTarget("check-api") {
		if err = makeTarget("check-api"); err != nil {
			return err
		}
	} else {
		d.record.OptionalAbsent = append(d.record.OptionalAbsent, "check-api")
	}
	if err = makeTarget("control"); err != nil {
		return err
	}
	if _, err = d.step(ctx, "control-race-instrumentation", []string{"go", "version", "-m", "build/rt-control"}, func(r result) error {
		if r.Code != 0 || !strings.Contains(r.Stdout, "-race=true") {
			return errors.New("control_race_instrumentation_missing")
		}
		return nil
	}); err != nil {
		return err
	}
	if err = makeTarget("sim"); err != nil {
		return err
	}
	packageRoot := filepath.Join(d.root, "build/ci/package")
	if err = os.MkdirAll(filepath.Join(packageRoot, "bin"), 0755); err != nil {
		return err
	}
	if err = run("manifest-artifact", "cp", "build/rosie-rt-core-sim", filepath.Join(packageRoot, "bin/rosie-rt-core")); err != nil {
		return err
	}
	if err = run("package-manifest", "go", "run", "./tools/packageinfo", "manifest", packageRoot, d.root, "simulation"); err != nil {
		return err
	}
	data, err := os.ReadFile(filepath.Join(packageRoot, "rt_package.json"))
	if err != nil {
		return err
	}
	var manifest struct {
		Inputs json.RawMessage `json:"build_inputs"`
	}
	if err = json.Unmarshal(data, &manifest); err != nil {
		return err
	}
	if len(manifest.Inputs) == 0 {
		return errors.New("package_build_inputs_missing")
	}
	d.record.Inputs = manifest.Inputs
	if err = makeTarget("test"); err != nil {
		return err
	}
	if _, err = d.step(ctx, "go-race-probe", []string{"go", "test", "-race", "-count=1", "-v", "./tools/cidriver/testdata/raceprobe"}, raceDetected); err != nil {
		return err
	}
	if err = makeTarget("test-go"); err != nil {
		return err
	}
	if err = run("sanitizer-build", "make", "build/rosie-rt-core-asan", "build/assertions-enabled-asan"); err != nil {
		return err
	}
	if _, err = d.step(ctx, "sanitizer-symbols", []string{"nm", "-D", "build/rosie-rt-core-asan"}, sanitizerSymbols); err != nil {
		return err
	}
	if _, err = d.step(ctx, "sanitizer-assertions", []string{"./build/assertions-enabled-asan"}, func(r result) error {
		if r.Code != 0 || !strings.Contains(r.Stdout, "PASS: assertion setup executes and a false assertion aborts") {
			return errors.New("assertions_disabled_or_probe_failed")
		}
		return nil
	}); err != nil {
		return err
	}
	if err = makeTarget("test-sanitize"); err != nil {
		return err
	}
	d.env["RTCORE_TEST_BINARY"] = filepath.Join(d.root, "build/rosie-rt-core-asan")
	err = run("sanitized-go-race", "make", "test-go", "GO=go")
	d.env["RTCORE_TEST_BINARY"] = filepath.Join(d.root, "build/rosie-rt-core-sim")
	if err != nil {
		return err
	}
	if err = makeTarget("test-faults"); err != nil {
		return err
	}
	if hasTarget("soak-check") {
		if err = makeTarget("soak-check"); err != nil {
			return err
		}
	} else {
		d.record.OptionalAbsent = append(d.record.OptionalAbsent, "soak-check")
	}
	if err = run("tsan-probe-build", "g++", "-std=c++17", "-fsanitize=thread", "-pthread", "tools/cidriver/testdata/tsan_probe.cpp", "-o", "build/ci/tsan-probe"); err != nil {
		return err
	}
	r, err := d.step(ctx, "tsan-probe", []string{"./build/ci/tsan-probe"}, tsanSupported)
	if err != nil {
		return err
	}
	if r.Code == 0 {
		d.record.TSan.Status = "failed"
		if err = makeTarget("test-tsan"); err != nil {
			return err
		}
		d.record.TSan.Status = "passed"
	}
	if runtime.GOARCH == "amd64" || runtime.GOARCH == "arm64" {
		d.record.Atomics.Status = "failed"
		if err = run("atomic-interop-build", "make", "build/shared-words-interop"); err != nil {
			return err
		}
		executable, err := os.Executable()
		if err != nil {
			return err
		}
		if err = run("cross-process-atomics", executable, "--shared-words", filepath.Join(d.root, "build/shared-words-interop")); err != nil {
			return err
		}
		d.record.Atomics.Status = "passed"
	} else {
		d.record.Atomics.Status = "unsupported"
	}
	return nil
}
