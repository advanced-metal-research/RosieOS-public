package main

import (
	"context"
	"encoding/json"
	"errors"
	"io"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

func fixture(t *testing.T, run runner) *driver {
	t.Helper()
	root := t.TempDir()
	allow, err := loadAllowlist(allowedSkips)
	if err != nil {
		t.Fatal(err)
	}
	return &driver{root: root, path: filepath.Join(root, "result.json"), logDir: filepath.Join(root, "logs"), run: run, out: io.Discard, allow: allow,
		record: record{Schema: recordSchema, Status: "running", TSan: qualification{Status: "not_run"}, Atomics: qualification{Status: "not_run"}}}
}
func saved(t *testing.T, d *driver) record {
	t.Helper()
	data, err := os.ReadFile(d.path)
	if err != nil {
		t.Fatal(err)
	}
	var r record
	if err = json.Unmarshal(data, &r); err != nil {
		t.Fatal(err)
	}
	return r
}

func TestStopsAtFirstFailureAndWritesRecord(t *testing.T) {
	calls := 0
	d := fixture(t, func(context.Context, command) result {
		calls++
		return result{Code: 17, Stdout: "partial output", Stderr: "failure"}
	})
	err := d.finish(func() error {
		for _, name := range []string{"test", "test-go", "test-sanitize"} {
			if _, err := d.step(context.Background(), name, []string{"make", name}, nil); err != nil {
				return err
			}
		}
		return nil
	})
	r := saved(t, d)
	if err == nil || calls != 1 || len(r.Steps) != 1 || r.Status != "failed" || r.Steps[0].Code != 17 {
		t.Fatalf("failure lost: calls=%d record=%+v err=%v", calls, r, err)
	}
	if r.Steps[0].WallNS < 0 || r.Steps[0].StdoutDigest != digest("partial output") || r.Steps[0].StderrDigest != digest("failure") {
		t.Fatal("measurements/digests missing")
	}
}

func TestSetupFailureStillWritesRecord(t *testing.T) {
	d := fixture(t, nil)
	err := d.finish(func() error { return errors.New("missing binary") })
	if err == nil || saved(t, d).Status != "failed" {
		t.Fatal("setup failure lost")
	}
}

func TestInstrumentationFailureStopsRun(t *testing.T) {
	for _, test := range []struct {
		name  string
		check func(result) error
		r     result
	}{
		{"race-success", raceDetected, result{}},
		{"race-compile-error", raceDetected, result{Code: 1, Stderr: "compiler missing"}},
		{"asan-missing", sanitizerSymbols, result{Code: 1}},
		{"ubsan-missing", sanitizerSymbols, result{Stdout: "__asan_init"}},
	} {
		t.Run(test.name, func(t *testing.T) {
			calls := 0
			d := fixture(t, func(context.Context, command) result { calls++; return test.r })
			err := d.finish(func() error {
				_, err := d.step(context.Background(), test.name, []string{"probe"}, test.check)
				if err != nil {
					return err
				}
				calls++
				return nil
			})
			if err == nil || calls != 1 || saved(t, d).Status != "failed" {
				t.Fatal("instrumentation failure did not stop gate")
			}
		})
	}
}

func TestSkipPolicyRecordsTrackerAndRejectsUnknown(t *testing.T) {
	for _, test := range []struct {
		name, log string
		allowed   bool
		tracker   string
	}{
		{"unknown-go", "--- SKIP: TestMissingDaemon (0.00s)", false, ""},
		{"unknown-shell", "SKIP required binary missing", false, ""},
		{"not-applicable", "policy: not_applicable missing hardware", false, ""},
		{"reason-mismatch", "--- SKIP: TestMatrix/j_digest (0.00s)", false, ""},
		{"listed-go", "=== RUN TestMatrix/j_digest\nconfiguration_test.go:15: modified-artifact rejection is not enforced\n--- SKIP: TestMatrix/j_digest (0.00s)", true, "P1-02"},
		{"listed-shell", "systemd-analyze verify status=skipped reason=not-installed", true, "P8-01"},
	} {
		t.Run(test.name, func(t *testing.T) {
			d := fixture(t, func(context.Context, command) result { return result{Stdout: test.log} })
			err := d.finish(func() error {
				_, err := d.step(context.Background(), "test", []string{"make", "test"}, nil)
				return err
			})
			r := saved(t, d)
			if (err == nil) != test.allowed {
				t.Fatalf("skip outcome %v: %v", test.allowed, err)
			}
			if test.allowed && (len(r.Steps[0].Skips) != 1 || r.Steps[0].Skips[0].TrackerID != test.tracker) {
				t.Fatal("tracker identity missing")
			}
		})
	}
}

func TestDeferredSkipSummariesKeepTheirOwnReasons(t *testing.T) {
	allow, err := loadAllowlist(allowedSkips)
	if err != nil {
		t.Fatal(err)
	}
	log := "=== RUN TestMatrix/e_statusword_unique_name\nfeedback_test.go:28: exhaustive unique fault classification is not enforced\n"
	log += "=== RUN TestMatrix/another_row\n" + strings.Repeat("unrelated test output\n", 100)
	log += "--- PASS: TestMatrix (0.00s)\n    --- SKIP: TestMatrix/e_statusword_unique_name (0.00s)\n"
	skips, err := scanSkips(log, allow)
	if err != nil || len(skips) != 1 || skips[0].TrackerID != "P4-04" {
		t.Fatalf("deferred skip lost its reason: %v %v", skips, err)
	}
	log = "=== RUN TestMatrix/another_row\nmodified-artifact rejection is not enforced\n=== RUN TestMatrix/j_digest\nmissing daemon\n--- SKIP: TestMatrix/j_digest (0.00s)\n"
	if _, err = scanSkips(log, allow); err == nil {
		t.Fatal("another test's reason permitted a missing-daemon skip")
	}
}

func TestExecutePreservesEmptyStderrOnFailure(t *testing.T) {
	executable, err := os.Executable()
	if err != nil {
		t.Fatal(err)
	}
	for _, test := range []struct {
		name string
		argv []string
		code int
	}{
		{"silent-exit", []string{executable, "-test.run=^TestExecuteHelper$"}, 17},
		{"missing-program", []string{filepath.Join(t.TempDir(), "missing")}, -1},
	} {
		t.Run(test.name, func(t *testing.T) {
			r := execute(context.Background(), command{Argv: test.argv, Env: map[string]string{"CIDRIVER_HELPER": "exit17"}})
			if r.Code != test.code || r.Stdout != "" || r.Stderr != "" || r.Error == "" {
				t.Fatalf("captured streams changed: %+v", r)
			}
		})
	}
}

func TestExecuteHelper(t *testing.T) {
	if os.Getenv("CIDRIVER_HELPER") == "exit17" {
		os.Exit(17)
	}
}

func TestTSanUnsupportedIsNeverPassed(t *testing.T) {
	diagnostic := "FATAL: ThreadSanitizer: unexpected memory mapping 0x1234-0x5678\n"
	d := fixture(t, func(context.Context, command) result { return result{Code: 66, Stderr: diagnostic} })
	err := d.finish(func() error {
		_, err := d.step(context.Background(), "tsan-probe", []string{"probe"}, tsanSupported)
		return err
	})
	r := saved(t, d)
	if err != nil || r.TSan.Status != "unsupported_on_host" || r.TSan.Error != diagnostic || r.Steps[0].Status != "unsupported_on_host" || r.Steps[0].Code != 66 {
		t.Fatalf("unsupported was misrepresented: %+v %v", r, err)
	}
	if err := tsanSupported(result{Code: 1, Stderr: "Segmentation fault"}); err == nil {
		t.Fatal("unclassified probe failure accepted")
	}
}

func TestNegativeRaceProbeRetainsActualFailureCode(t *testing.T) {
	d := fixture(t, func(context.Context, command) result { return result{Code: 1, Stderr: "WARNING: DATA RACE"} })
	err := d.finish(func() error {
		_, err := d.step(context.Background(), "race", []string{"go", "test", "-race"}, raceDetected)
		return err
	})
	r := saved(t, d)
	if err != nil || r.Status != "passed" || r.Steps[0].Code != 1 {
		t.Fatal("negative probe lost actual exit")
	}
}

func TestMalformedAllowlistFailsClosed(t *testing.T) {
	for _, data := range []string{`[{"note":"x"}]`, `[{"test":"[","note":"x","reason":"r","tracker_id":"P8-01"}]`, string(allowedSkips) + `{}`} {
		if _, err := loadAllowlist([]byte(data)); err == nil {
			t.Fatal("bad allowlist accepted")
		}
	}
}

func TestControlledEnvironmentCannotDisableDetectors(t *testing.T) {
	env := controlledEnvironment("/component")
	if env["GO_BUILD_FLAGS"] != "-race" || !strings.Contains(env["ASAN_OPTIONS"], "halt_on_error=1") || env["MAKEFLAGS"] != "" || env["GOFLAGS"] != "-mod=mod" {
		t.Fatal("coverage controls missing")
	}
}

func TestPipelineOrderAndFirstFailure(t *testing.T) {
	for _, failure := range []string{"", "check-python-absent", "test", "go-race-probe", "sanitizer-symbols", "test-faults"} {
		t.Run("failure="+failure, func(t *testing.T) {
			d := fixture(t, nil)
			d.env = controlledEnvironment(d.root)
			if err := os.WriteFile(filepath.Join(d.root, "Makefile"), []byte("check-api:\nsoak-check:\n"), 0644); err != nil {
				t.Fatal(err)
			}
			var names []string
			d.run = func(_ context.Context, c command) result {
				names = append(names, c.Name)
				if c.Name == failure {
					return result{Code: 23, Stderr: "required stage failed"}
				}
				switch c.Name {
				case "control-race-instrumentation":
					return result{Stdout: "build -race=true"}
				case "package-manifest":
					if err := os.WriteFile(filepath.Join(d.root, "build/ci/package/rt_package.json"), []byte(`{"build_inputs":{"git_sha":"fixture"}}`), 0644); err != nil {
						t.Fatal(err)
					}
				case "go-race-probe":
					return result{Code: 1, Stderr: "WARNING: DATA RACE"}
				case "sanitizer-symbols":
					return result{Stdout: "__asan_init\n__ubsan_handle_type_mismatch_v1"}
				case "sanitizer-assertions":
					return result{Stdout: "PASS: assertion setup executes and a false assertion aborts"}
				case "tsan-probe":
					return result{Code: 66, Stderr: "FATAL: ThreadSanitizer: unexpected memory mapping 0xabcd\n"}
				case "source-metadata":
					return result{Stdout: "/metadata/fixture\n" + d.root + "\n"}
				}
				return result{}
			}
			err := d.finish(func() error { return d.pipeline(context.Background()) })
			if failure != "" {
				if err == nil || names[len(names)-1] != failure || saved(t, d).Status != "failed" {
					t.Fatalf("did not stop at %s: %v %v", failure, names, err)
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			var required []string
			for _, name := range names {
				switch name {
				case "check-python-absent", "check-protocol", "check-api", "test", "test-go", "test-sanitize", "test-faults", "soak-check":
					required = append(required, name)
				}
				if name == "test-tsan" {
					t.Fatal("unsupported TSan suite executed")
				}
			}
			want := []string{"check-python-absent", "check-protocol", "check-api", "test", "test-go", "test-sanitize", "test-faults", "soak-check"}
			if !reflect.DeepEqual(required, want) {
				t.Fatalf("required order %v", required)
			}
			if saved(t, d).TSan.Status != "unsupported_on_host" {
				t.Fatal("unsupported qualification lost")
			}
		})
	}
}
