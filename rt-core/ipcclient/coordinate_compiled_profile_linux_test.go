//go:build linux

package ipcclient

import (
	"encoding/json"
	"errors"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"testing"
)

// Read the repository JSON so compiler regressions cannot be hidden by a
// separately maintained native descriptor in this fixture.
func compiledCoordinateJSON(t *testing.T, name string) map[string]any {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join("..", name))
	if err != nil {
		t.Fatal(err)
	}
	var value map[string]any
	if err := json.Unmarshal(raw, &value); err != nil {
		t.Fatal(err)
	}
	return value
}

// Compile JSON through the production compiler and preserve its exact output;
// native policy and source lines must never be patched by the test.
func compileCoordinateJSON(t *testing.T, machine, profile map[string]any) (compiledValidationFixture, string, error) {
	t.Helper()
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "coordinate-compiled-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(dir) })
	for name, value := range map[string]any{"machine.json": machine, "coordinate_sim.json": profile} {
		raw, err := json.Marshal(value)
		if err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(dir, name), raw, 0600); err != nil {
			t.Fatal(err)
		}
	}
	compiled, args, err := simfixture.CompileProfiles(root, filepath.Join(dir, "machine.json"), dir, dir)
	if err != nil {
		return compiledValidationFixture{}, simfixture.ErrorText(err), err
	}
	path := filepath.Join(compiled, "axes.conf")
	profileText, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	binary := simfixture.Locate(root, "rosie-rt-core-sim")
	return compiledValidationFixture{binary, args, string(profileText), path}, filepath.Base(compiled), nil
}

// Model the A6 continuous-source contract with deterministic simulator objects. Home checks
// the independent source without enabling, preserving the one-shot displacement
// injection for the subsequent CSP disable instead of consuming it during Home.
func compiledCoordinateInputs(t *testing.T) (map[string]any, map[string]any) {
	t.Helper()
	machine := compiledCoordinateJSON(t, "config/machines/simulation/simulation-home.json")
	machine["pair"] = map[string]any{"id": "coordinate-test", "revision": 1}
	identityAxis := machine["axes"].([]any)[0].(map[string]any)
	identityAxis["motor_encoder_counts_per_rev"] = 131072
	identityAxis["gear_ratio"] = map[string]any{"numerator": 1, "denominator": 1}
	identityAxis["position_tracking_mode"] = "continuous"
	identityAxis["coordinate_evidence_mode"] = "multi_turn"
	machine["axes"].([]any)[0].(map[string]any)["profile"] = "coordinate_sim"
	limits := machine["axes"].([]any)[0].(map[string]any)["limits"].(map[string]any)
	limits["min"], limits["max"] = -10, 10
	profile := compiledCoordinateJSON(t, "config/drives/cia402_sim_home.json")
	a6 := compiledCoordinateJSON(t, "config/drives/a6ec.json")
	policy := a6["coordinate_evidence"].(map[string]any)
	// Unverified synthetic physical bound: 2,000,000 counts/s and 1,000,000 ns
	// acquisition age accept 2,000 counts even at closely spaced receipts.
	// With the fixture's 20 ms lateness limit, 60,000 counts cannot fit.
	policy["maximum_physical_speed_counts_per_second"] = 2000000
	policy["maximum_acquisition_age_ns"] = 10000000
	policy["sample_interval_ns"] = 100000
	// Preserve the synthetic PDO oracle's one-count bounds independently of
	// the measured mailbox tolerance in the live A6 profile.
	policy["maximum_feedback_error_counts"], policy["maximum_anchor_error_counts"] = 1, 1
	profile["coordinate_evidence"] = policy
	home := profile["native_home"].(map[string]any)
	var transaction []any
	for _, step := range home["transaction"].([]any) {
		op := step.(map[string]any)["op"]
		if op != "controlword_sequence" && op != "wait_statusword" {
			transaction = append(transaction, step)
		}
	}
	home["transaction"] = transaction
	return machine, profile
}

func TestA6CoordinateCompiledProfile(t *testing.T) {
	for _, machine := range []string{"a6ec-bench.example.json", "machines/bench/rosie1420-six-motors.json"} {
		t.Run(machine, func(t *testing.T) {
			f := compiledForValidation(t, machine)
			// Declared unverified inhibited acquisition bounds in counts/s.
			maxSpeed := "5000000"
			if machine == "a6ec-bench.example.json" {
				maxSpeed = "262144"
			}
			for _, want := range []string{"tp_status|24761|0|16", "tp_pos1|24762|0|32", "tp_pos2|24764|0|32", "coordinate_evidence=" + maxSpeed + "|3000000000|16|16|a6ec_sdo|1500000000\n"} {
				if !strings.Contains(f.profile, want) {
					t.Fatalf("A6 compiler output missing %q: %s", want, f.profile)
				}
			}
			if strings.Contains(f.profile, "continuous_pos|") || strings.Contains(f.profile, "continuous_valid|") {
				t.Fatal("A6 fixed mapping contains a nonexistent continuous PDO")
			}
			if output, err := f.validate(t, f.profile); err != nil {
				t.Fatalf("native loader rejected compiled A6: %v: %s", err, output)
			}
		})
	}
}

func TestSampledPDOHistoryNativeLoaderBound(t *testing.T) {
	f := compiledForValidation(t, "a6ec-bench.example.json")
	for _, tc := range []struct {
		age      string
		rejected bool
	}{
		{"131070000000", false}, {"131070000001", true}, {"18446744073709551615", true},
	} {
		t.Run(tc.age, func(t *testing.T) {
			profile := strings.ReplaceAll(f.profile, "262144|3000000000|16|16|a6ec_sdo", "262144|"+tc.age+"|16|16|a6ec_sdo")
			if profile == f.profile {
				t.Fatal("missing acquisition fixture mutation")
			}
			output, err := f.validate(t, profile)
			if tc.rejected {
				if err == nil || !strings.Contains(output, "sampled PDO history exceeds 131072 entries") {
					t.Fatalf("native bound refusal: %v: %s", err, output)
				}
			} else if err != nil {
				t.Fatalf("native boundary rejected: %v: %s", err, output)
			}
		})
	}
}

func TestCompiledCoordinateProfileRejectsInvalidPolicy(t *testing.T) {
	for _, test := range []struct {
		name   string
		mutate func(map[string]any)
	}{
		{"retired_feedback_wrap", func(p map[string]any) { p["feedback_counts_wrap"] = false }},
		{"retired_command_wrap", func(p map[string]any) { p["command_counts_wrap"] = true }},
		{"missing_anchor_source", func(p map[string]any) {
			delete(p["coordinate_evidence"].(map[string]any), "anchor_source")
		}},
		{"zero_speed", func(p map[string]any) {
			p["coordinate_evidence"].(map[string]any)["maximum_physical_speed_counts_per_second"] = 0
		}},
		{"zero_age", func(p map[string]any) { p["coordinate_evidence"].(map[string]any)["maximum_acquisition_age_ns"] = 0 }},
		{"negative_error", func(p map[string]any) {
			p["coordinate_evidence"].(map[string]any)["maximum_feedback_error_counts"] = -1
		}},
	} {
		t.Run(test.name, func(t *testing.T) {
			machine, profile := compiledCoordinateInputs(t)
			test.mutate(profile)
			// RunCLI reports configuration rejection with exit 1; launch failures cannot pass.
			var exit *exec.ExitError
			if _, output, err := compileCoordinateJSON(t, machine, profile); !errors.As(err, &exit) || exit.ExitCode() != 1 {
				t.Fatalf("invalid coordinate JSON was not rejected by the compiler: %v: %s", err, output)
			}
		})
	}
}
