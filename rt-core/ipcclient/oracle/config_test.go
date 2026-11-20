//go:build linux

package oracle

import (
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"testing"

	"rosieos/rt-core/tools/rtctl"
)

func machine(t *testing.T, name string) map[string]any {
	t.Helper()
	directory := "../../config/machines"
	if strings.HasPrefix(name, "simulation") {
		directory = filepath.Join(directory, "simulation")
	}
	return machineAt(t, filepath.Join(directory, name))
}

func machineAt(t *testing.T, path string) map[string]any {
	t.Helper()
	raw, err := os.ReadFile(path)
	must(t, err)
	var m map[string]any
	must(t, json.Unmarshal(raw, &m))
	return m
}
func axis(m map[string]any, i int) map[string]any { return m["axes"].([]any)[i].(map[string]any) }
func compile(t *testing.T, m map[string]any) (rtctl.Compiled, error) {
	t.Helper()
	raw, err := json.Marshal(m)
	must(t, err)
	return simfixture.CompileRaw(raw, "../../config/drives", rtctl.Compile)
}

// Origin: test_config.ConfigurationTests.test_mixed_profiles_compile_independently.
func TestMixedProfilesCompileIndependently(t *testing.T) {
	m := machine(t, "simulation.json")
	a, err := compile(t, m)
	must(t, err)
	for _, s := range []string{"vendor_id=4294967294", "vendor_id=4294967293", "rx_pdo=5633"} {
		if !strings.Contains(a.AxisProfiles, s) {
			t.Fatal(s)
		}
	}
	b, err := compile(t, machine(t, "simulation.json"))
	must(t, err)
	if a.SHA256 != b.SHA256 {
		t.Fatal("copy changed identity")
	}
	axis(m, 0)["sign"] = -1
	b, err = compile(t, m)
	must(t, err)
	if a.SHA256 == b.SHA256 {
		t.Fatal("sign did not change identity")
	}
}

// Origin: test_config.ConfigurationTests.test_no_live_use_of_simulated_identity.
func TestNoLiveUseOfSimulatedIdentity(t *testing.T) {
	m := machine(t, "simulation.json")
	m["backend"] = "live"
	if _, err := compile(t, m); err == nil {
		t.Fatal("live simulated identity accepted")
	}
}

// Origin: test_config.ConfigurationTests.test_reject_invalid_machine_before_runtime.
func TestRejectInvalidMachineBeforeRuntime(t *testing.T) {
	mutations := []func(map[string]any){
		func(m map[string]any) { axis(m, 1)["slave_position"] = 0 },
		func(m map[string]any) { axis(m, 0)["sign"] = 0 },
		func(m map[string]any) { axis(m, 0)["wire_revs_per_axis_rev"] = "NaN" },
		func(m map[string]any) { v := axis(m, 0)["limits"].(map[string]any); v["min"] = 1; v["max"] = 0 },
		func(m map[string]any) { axis(m, 0)["limits"].(map[string]any)["completion_tolerance"] = 0 },
		func(m map[string]any) { delete(axis(m, 0)["limits"].(map[string]any), "completion_tolerance") },
		func(m map[string]any) { m["cycle_ns"] = 333333 },
		func(m map[string]any) { m["max_cycle_lateness_ns"] = 0 },
		func(m map[string]any) { m["max_cycle_lateness_ns"] = 250000000 },
		func(m map[string]any) { axis(m, 0)["profile"] = "../cia402_sim" },
		func(m map[string]any) { axis(m, 0)["require_home"] = true },
	}
	for i, mutate := range mutations {
		m := machine(t, "simulation.json")
		mutate(m)
		if _, err := compile(t, m); err == nil {
			t.Fatalf("mutation %d accepted", i)
		}
	}
	// JSON cannot encode NaN: submit its literal token to the compiler boundary.
	raw, err := os.ReadFile("../../config/machines/simulation/simulation.json")
	must(t, err)
	raw = []byte(strings.Replace(string(raw), `"wire_revs_per_axis_rev": 1`, `"wire_revs_per_axis_rev": NaN`, 1))
	if _, err = simfixture.CompileRaw(raw, "../../config/drives", rtctl.Compile); err == nil {
		t.Fatal("nonfinite ratio accepted")
	}
}

// Origin: test_config.ConfigurationTests.test_daemon_rejects_invalid_compiled_profile.
func TestDaemonRejectsInvalidCompiledProfile(t *testing.T) {
	c, err := compile(t, machine(t, "simulation.json"))
	must(t, err)
	root, err := filepath.Abs("../..")
	must(t, err)
	dir, err := os.MkdirTemp(filepath.Join(root, "build"), "invalid-")
	must(t, err)
	defer os.RemoveAll(dir)
	for _, r := range [][2]string{{"sw|24641|0|16", "sw|24641|0|32"}, {"vendor_id=4294967293", "vendor_id=0"}, {"axis=1", "axis=0"}, {"limits=-0.5|0.5|0.1|0", "limits=nan|0.5|0.1|0"}} {
		s := strings.ReplaceAll(c.AxisProfiles, r[0], r[1])
		if s == c.AxisProfiles {
			t.Fatal("fixture mutation missing")
		}
		path := filepath.Join(dir, "axes.conf")
		must(t, os.WriteFile(path, []byte(s), 0600))
		run := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), append(append([]string{}, c.Argv...), "--axis-profiles", path, "--validate-config")...)
		raw, err := run.CombinedOutput()
		if err == nil || run.ProcessState.ExitCode() != 2 {
			t.Fatalf("want exit 2: %v %s", err, raw)
		}
	}
}

// A6 linear mode keeps the standard 1/1 drive factor on every axis and applies
// the mechanical ratio only in the native software conversion.
func TestA6LinearSettingsAreExplicitAndScoped(t *testing.T) {
	for _, name := range []string{"../../config/machines/a6ec-bench.example.json", "../../config/machines/example-arm-1420.json"} {
		t.Run(name, func(t *testing.T) {
			m := machineAt(t, name)
			c, err := compile(t, m)
			must(t, err)
			// Cited ratios: example 1/1; Rosie 1420 robot definition J1..J6.
			ratios := []int{1}
			if filepath.Base(name) == "example-arm-1420.json" {
				ratios = []int{100, 100, 100, 18, 100, 10}
			}
			sections := strings.Split(c.AxisProfiles, "axis=")[1:]
			if len(sections) != len(ratios) {
				t.Fatalf("axis count=%d, want %d", len(sections), len(ratios))
			}
			for i, ratio := range ratios {
				for _, want := range []string{
					"a6ec_encoder_position_tracking_mode|u16|8192|8|1|power_cycle",
					"cia402_position_factor_numerator|u32|24721|1|1",
					"cia402_position_factor_denominator|u32|24721|2|1",
				} {
					if !strings.Contains(sections[i], want) {
						t.Fatalf("axis %d missing linear setting %q", i, want)
					}
				}
				if !strings.Contains(c.MachineIdentity, fmt.Sprintf("axis.%d.gear_ratio=%d\n", i, ratio)) ||
					!strings.Contains(c.MachineIdentity, fmt.Sprintf("axis.%d.counts_per_rev=131072\n", i)) {
					t.Fatalf("axis %d software ratio/count frame missing", i)
				}
				axis(m, i)["startup"] = map[string]any{"a6ec_rotation_mode_gear_ratio_numerator": ratio}
				if _, err = compile(t, m); err == nil {
					t.Fatalf("axis %d retired rotation-mode override accepted", i)
				}
				delete(axis(m, i), "startup")
			}
		})
	}
}

// Origin: test_config.ConfigurationTests.test_startup_scalar_overflow_is_rejected_before_runtime.
func TestStartupScalarOverflowIsRejectedBeforeRuntime(t *testing.T) {
	for k, v := range map[string]int64{"u8": -1, "i8": 128, "u16": 65536, "i16": -32769, "u32": 4294967296, "i32": -2147483649} {
		m := machine(t, "simulation-scalars.json")
		axis(m, 0)["startup"] = map[string]any{k: v}
		if _, err := compile(t, m); err == nil {
			t.Fatalf("%s overflow accepted", k)
		}
	}
}
