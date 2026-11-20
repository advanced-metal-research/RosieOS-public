//go:build linux

package simfixture

import (
	"bytes"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"strconv"
	"strings"
	"testing"

	"golang.org/x/sys/unix"
)

// Both raw and file compilation must observe selection before the compiler
// derives identity; the private input must be removed on success and refusal.
func TestCompileSelectsCPUBeforeCompiler(t *testing.T) {
	runtime.LockOSThread()
	defer runtime.UnlockOSThread()
	var original, allowed unix.CPUSet
	if err := unix.SchedGetaffinity(0, &original); err != nil {
		t.Fatal(err)
	}
	selected := 0
	for cpu := 0; cpu < 1024 && allowed.Count() < 2; cpu++ {
		if original.IsSet(cpu) {
			allowed.Set(cpu)
			selected = cpu
		}
	}
	if allowed.Count() != 2 {
		t.Skip("simulation fixture requires at least two allowed CPUs")
	}
	if err := unix.SchedSetaffinity(0, &allowed); err != nil {
		t.Fatal(err)
	}
	defer func() {
		if err := unix.SchedSetaffinity(0, &original); err != nil {
			t.Fatal(err)
		}
	}()
	excluded := 0
	for allowed.IsSet(excluded) {
		excluded++
	}
	raw, err := os.ReadFile("../../config/machines/simulation/simulation.json")
	Must(t, err)
	raw = bytes.Replace(raw, []byte(`"cpu": 2`), []byte(`"cpu": `+strconv.Itoa(excluded)), 1)
	calls := 0
	got, err := CompileRaw(raw, "config/drives", func(input []byte, profiles string) (int, error) {
		calls++
		if profiles != "config/drives" {
			t.Fatal("profiles changed")
		}
		var machine struct{ Runtime struct{ CPU int } }
		err := json.Unmarshal(input, &machine)
		return machine.Runtime.CPU, err
	})
	if err != nil || calls != 1 || got != selected {
		t.Fatalf("compiler observed CPU=%d calls=%d: %v", got, calls, err)
	}
	root, err := filepath.Abs("../..")
	Must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "compile-cpu-")
	Must(t, err)
	defer os.RemoveAll(build)
	machine := filepath.Join(build, "machine.json")
	for _, valid := range []bool{true, false} {
		input := raw
		if !valid {
			input = []byte(`{"backend":"simulation","runtime":{"cpu":` + strconv.Itoa(excluded) + `}}`)
		}
		Must(t, os.WriteFile(machine, input, 0600))
		_, args, err := Compile(root, machine, filepath.Join(build, "config"))
		if valid != (err == nil) {
			t.Fatalf("valid=%t: %v", valid, err)
		}
		if valid && !strings.Contains(strings.Join(args, " "), "--rt-cpu "+strconv.Itoa(selected)+" ") {
			t.Fatalf("file compiler did not select CPU %d: %v", selected, args)
		}
		leftovers, err := filepath.Glob(filepath.Join(build, "config", "simulation-machine-*.json"))
		if err != nil || len(leftovers) != 0 {
			t.Fatalf("private inputs remain: %v: %v", leftovers, err)
		}
		after, err := os.ReadFile(machine)
		if err != nil || !bytes.Equal(after, input) {
			t.Fatalf("source input changed: %v", err)
		}
	}
}

// CPU substitution must preserve every other byte, including indentation,
// escaped property names, HTML, and numbers beyond binary64 integer precision.
func TestSimulationCPUUntouchedBytes(t *testing.T) {
	var allowed unix.CPUSet
	allowed.Set(0)
	allowed.Set(1)
	for _, raw := range []string{
		`{"backend":"simulation","runtime":{"cpu":2,"priority":90,"annotation":"<unverified>"},"axes":[ { "exact":9007199254740993 } ]}`,
		"{\n  \"backend\": \"simulation\",\n  \"runtime\": {\n    \"cpu\" : 2,\n    \"priority\": 90,\n    \"annotation\": \"<unverified>&\"\n  },\n  \"axes\": [ { \"exact\": 9007199254740993, \"ratio\": 1.00e+02 } ]\n}\n",
		`{"backend":"simulation","runtime":{"cpu":0,"annotation":{"cpu":2},"c\u0070u":1023},"axes": [ { "cpu": 2 } ]}`,
	} {
		got, err := simulationCPUConfig([]byte(raw), allowed)
		if err != nil {
			t.Fatal(err)
		}
		want := strings.Replace(raw, `"cpu":2`, `"cpu":1`, 1)
		want = strings.Replace(want, `"cpu" : 2`, `"cpu" : 1`, 1)
		if strings.Contains(raw, `"c\u0070u":1023`) {
			want = strings.Replace(raw, `"c\u0070u":1023`, `"c\u0070u":1`, 1)
		}
		if string(got) != want {
			t.Errorf("bytes outside selected CPU changed:\nwant %s\ngot  %s", want, got)
		}
		before, after := bytes.Split([]byte(raw), []byte("\n")), bytes.Split(got, []byte("\n"))
		if len(before) != len(after) {
			t.Fatalf("line count changed: %d to %d", len(before), len(after))
		}
		for i := range before {
			if !bytes.Contains(before[i], []byte(`"cpu"`)) && !bytes.Equal(before[i], after[i]) {
				t.Errorf("untouched line %d changed: %q to %q", i+1, before[i], after[i])
			}
		}
	}
}

func TestSimulationCPUConfig(t *testing.T) {
	for _, test := range []struct {
		name, backend string
		allowed       []int
		configured    int
		want          int
		wantError     bool
	}{
		{"two_cpus", "simulation", []int{0, 1}, 2, 1, false},
		{"allowed_cpu", "simulation", []int{0, 1}, 0, 0, false},
		{"sparse_set", "simulation", []int{3, 7}, 2, 7, false},
		{"live_unchanged", "live", []int{0, 1}, 2, 2, false},
		{"empty_set", "simulation", nil, 2, 0, true},
	} {
		t.Run(test.name, func(t *testing.T) {
			var allowed unix.CPUSet
			for _, cpu := range test.allowed {
				allowed.Set(cpu)
			}
			raw, err := json.Marshal(map[string]any{
				"backend": test.backend,
				"runtime": map[string]any{"cpu": test.configured, "priority": 90},
			})
			if err != nil {
				t.Fatal(err)
			}
			// Exact integer preservation prevents an unrelated machine identity change.
			raw = []byte(strings.TrimSuffix(string(raw), "}") + `,"exact":9007199254740993}`)
			got, err := simulationCPUConfig(raw, allowed)
			if test.wantError {
				if err == nil {
					t.Fatal("empty affinity accepted")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			var machine struct {
				Runtime struct{ CPU, Priority int }
				Exact   uint64
			}
			if err := json.Unmarshal(got, &machine); err != nil {
				t.Fatal(err)
			}
			if machine.Runtime.CPU != test.want || machine.Runtime.Priority != 90 || machine.Exact != 9007199254740993 {
				t.Fatalf("unexpected fixture configuration: %s", got)
			}
			if test.configured == test.want && string(got) != string(raw) {
				t.Fatal("unchanged CPU rewrote configuration")
			}
		})
	}
}

func TestSimulationCPUConfigPreservesInvalidCPU(t *testing.T) {
	var allowed unix.CPUSet
	allowed.Set(1)
	for _, cpu := range []string{"null", "-1", "1024", "2.5", `"2"`} {
		raw := []byte(`{"backend":"simulation","runtime":{"cpu":` + cpu + `}}`)
		got, err := simulationCPUConfig(raw, allowed)
		if err != nil || string(got) != string(raw) {
			t.Fatalf("fixture hid invalid CPU %s from compiler: %s: %v", cpu, got, err)
		}
	}
}

// Single-CPU simulation must stop before compilation even when the configured
// CPU is already allowed; otherwise every daemon fails housekeeping admission.
func TestSimulationCPURequiresHousekeeping(t *testing.T) {
	var allowed unix.CPUSet
	allowed.Set(0)
	for _, configured := range []string{"0", "2"} {
		raw := []byte(`{"backend":"simulation","runtime":{"cpu":` + configured + `}}`)
		_, err := simulationCPUConfig(raw, allowed)
		var requirement *CPURequirementError
		if !errors.As(err, &requirement) || requirement.Allowed != 1 || !strings.Contains(err.Error(), "at least two allowed CPUs") {
			t.Fatalf("single CPU configuration %s did not report housekeeping requirement: %v", configured, err)
		}
	}
}

func TestMustSkipsCPURequirement(t *testing.T) {
	ranAfter := false
	if !t.Run("single_cpu", func(t *testing.T) {
		Must(t, fmt.Errorf("compile fixture: %w", &CPURequirementError{Allowed: 1}))
		ranAfter = true
	}) || ranAfter {
		t.Fatal("typed CPU requirement did not skip fixture setup")
	}
}
