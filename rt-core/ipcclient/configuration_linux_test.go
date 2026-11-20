//go:build linux

package ipcclient

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"testing"
	"time"

	"golang.org/x/sys/unix"
)

type compiledValidationFixture struct {
	binary  string
	args    []string
	profile string
	path    string
}

// Use the Go compiler pinned against the original P2 oracle. The daemon
// validates its output without creating sockets, requesting a master or moving.
func compiledForValidation(t *testing.T, machine string) compiledValidationFixture {
	t.Helper()
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	binary := simfixture.Locate(root, "rosie-rt-core-sim")
	directory := t.TempDir()
	machinePath := simfixture.MachineConfigPath(root, machine)
	compiled, args, err := simfixture.Compile(root, machinePath, directory)
	simfixture.Must(t, err)
	path := filepath.Join(compiled, "axes.conf")
	profile, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	return compiledValidationFixture{binary, legacyIdentityArgs(args), string(profile), path}
}

func (f compiledValidationFixture) validate(t *testing.T, profile string, extraArgs ...string) (string, error) {
	t.Helper()
	if err := os.WriteFile(f.path, []byte(profile), 0600); err != nil {
		t.Fatal(err)
	}
	args := append(append([]string(nil), f.args...), "--validate-config")
	args = append(args, extraArgs...)
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	output, err := exec.CommandContext(ctx, f.binary, args...).CombinedOutput()
	if ctx.Err() != nil {
		t.Fatalf("configuration-only validation hung: %v: %s", ctx.Err(), output)
	}
	return string(output), err
}

func editProfileField(t *testing.T, profile string, wantedAxis int, key string, replacement *string) string {
	t.Helper()
	axis, found := -1, false
	lines := strings.Split(profile, "\n")
	result := make([]string, 0, len(lines))
	for _, line := range lines {
		if strings.HasPrefix(line, "axis=") {
			if _, err := fmt.Sscanf(line, "axis=%d", &axis); err != nil {
				t.Fatal(err)
			}
		}
		if axis == wantedAxis && strings.HasPrefix(line, key+"=") {
			found = true
			if replacement != nil {
				result = append(result, key+"="+*replacement)
			}
			continue
		}
		result = append(result, line)
	}
	if !found {
		t.Fatalf("test fixture has no axis%d field %s", wantedAxis, key)
	}
	return strings.Join(result, "\n")
}

func TestNativeCompiledProfileCorpusStillValidates(t *testing.T) {
	for _, machine := range []string{"simulation.json", "simulation-scalars.json", "simulation-home.json", "simulation-linear.json", "simulation-program.json", "a6ec-bench.example.json", "machines/bench/rosie1420-six-motors.json"} {
		t.Run(machine, func(t *testing.T) {
			f := compiledForValidation(t, machine)
			if output, err := f.validate(t, f.profile); err != nil {
				t.Fatalf("compiled profile rejected: %v: %s", err, output)
			}
		})
	}
}

// Fixed A6 mapping has no continuous-valid object. Native parsing must reject
// an injected PDO even when a caller bypasses the Go compiler's source checks.
func TestNativeFixedPDORejectsContinuousValidity(t *testing.T) {
	f := compiledForValidation(t, "a6ec-bench.example.json")
	var layout string
	for _, line := range strings.Split(f.profile, "\n") {
		if strings.HasPrefix(line, "tx_layout=") {
			layout = strings.TrimPrefix(line, "tx_layout=")
			break
		}
	}
	if layout == "" {
		t.Fatal("missing fixed layout")
	}
	layout += ";continuous_valid|8256|67|16"
	profile := editProfileField(t, f.profile, 0, "tx_layout", &layout)
	output, err := f.validate(t, profile)
	if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 {
		t.Fatalf("fixed PDO accepted nonexistent validity object: %v: %s", err, output)
	}
	if ReasonPdoMappingMismatch != 7 {
		t.Fatal("mapping mismatch native reason changed")
	}
}

// The sampled descriptor must participate in the native identity digest, so a
// compiled fixed-PDO machine validates with all production identity arguments.
func TestNativeFixedPDOSampledIdentity(t *testing.T) {
	root, err := filepath.Abs("..")
	simfixture.Must(t, err)
	_, args, err := simfixture.Compile(root, filepath.Join(root, "config", "machines", "bench", "rosie1420-six-motors.json"), t.TempDir())
	simfixture.Must(t, err)
	// Unverified validation watchdog, matching the existing configuration fixture.
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	output, err := exec.CommandContext(ctx, simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--validate-config")...).CombinedOutput()
	if err != nil {
		t.Fatalf("sampled fixed PDO identity rejected: %v: %s", err, output)
	}
}

func TestNativeCompiledProfilesRequireEveryField(t *testing.T) {
	f := compiledForValidation(t, "simulation.json")
	// Both an intermediate and final section must be checked. In particular,
	// empty descriptors are explicit disabled features, not optional omissions.
	for _, axis := range []int{0, 1} {
		for _, key := range []string{"name", "vendor_id", "product_code", "revision_no", "rx_pdo", "tx_pdo", "rx_sync", "tx_sync", "dc_quantum_ns", "dc_assign_activate", "rx_layout", "tx_layout", "home_truth_sign", "native_home", "absolute_feedback", "absolute_pair_field", "startup_sdos", "limits", "tracking"} {
			t.Run(fmt.Sprintf("axis%d/%s", axis, key), func(t *testing.T) {
				profile := editProfileField(t, f.profile, axis, key, nil)
				output, err := f.validate(t, profile)
				if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 {
					t.Fatalf("missing field was not rejected as invalid configuration: %v: %s", err, output)
				}
			})
		}
	}
}

func TestNativeHomeTruthSignAcceptsOnlyUnitSigns(t *testing.T) {
	f := compiledForValidation(t, "simulation.json")
	// Change only an explicitly present native descriptor, after compilation,
	// so compiler validation cannot conceal a missing native sign check. Both
	// intermediate and final axis sections must reject invalid values.
	for _, axis := range []int{0, 1} {
		for _, test := range []struct {
			value string
			valid bool
		}{
			{"0", false}, {"2", false}, {"-2", false},
			{"1", true}, {"-1", true},
		} {
			t.Run(fmt.Sprintf("axis%d/sign%s", axis, test.value), func(t *testing.T) {
				profile := editProfileField(t, f.profile, axis, "home_truth_sign", &test.value)
				output, err := f.validate(t, profile)
				if test.valid {
					if err != nil {
						t.Fatalf("valid unit home sign rejected: %v: %s", err, output)
					}
				} else if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 {
					t.Fatalf("invalid present home sign accepted: %v: %s", err, output)
				}
			})
		}
	}
}

func TestNativeCompiledProfileRejectsDanglingCoordinatePolicy(t *testing.T) {
	f := compiledForValidation(t, "simulation.json")
	for _, change := range []struct{ key, value string }{
		{"absolute_pair_field", "unconfigured_turn_counter"},
		{"limits", "-0.5|0.5|0.1|1"}, // No home procedure in this simulation profile.
		{"name", "axis_1"},           // IDs must identify one axis.
	} {
		t.Run(change.key, func(t *testing.T) {
			profile := editProfileField(t, f.profile, 0, change.key, &change.value)
			output, err := f.validate(t, profile)
			if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 {
				t.Fatalf("invalid coordinate policy accepted: %v: %s", err, output)
			}
		})
	}
}

func TestRetiredGlobalDriveOptionsAreRejected(t *testing.T) {
	f := compiledForValidation(t, "simulation.json")
	for _, option := range []struct{ key, value string }{
		{"--drive-profile", "a6ec"}, {"--startup-sdo-config", ""},
		{"--absolute-feedback-config", ""}, {"--native-home-config", ""},
		{"--slave-vendor-id", "1"}, {"--slave-product-code", "2"},
		{"--slave-revision-no", "0"}, {"--rx-sync-index", "2"},
		{"--tx-sync-index", "3"}, {"--dc-cycle-multiple-ns", "0"},
		{"--rx-pdo", "5632"}, {"--tx-pdo", "6656"},
		{"--rx-pdo-layout", "cw|24640|0|16"}, {"--tx-pdo-layout", "sw|24641|0|16"},
	} {
		t.Run(option.key, func(t *testing.T) {
			output, err := f.validate(t, f.profile, option.key, option.value)
			if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 || !strings.Contains(output, "unknown arg: "+option.key) {
				t.Fatalf("retired override was not explicitly rejected: %v: %s", err, output)
			}
		})
	}
}

func TestRetiredSetpointRegionAbsent(t *testing.T) {
	f := startCore(t, "simulation.json")
	for _, region := range []ring{f.client.commands, f.client.statuses} {
		if wire.Uint32(region.data[8:]) != 128 || wire.Uint32(region.data[40:]) != 128 || wire.Uint32(region.data[52:]) != 0 {
			t.Fatal("protocol 1.4 must retain a 128-byte header and no legacy setpoint region")
		}
	}
}

func TestRetiredSetpointHeaderRejected(t *testing.T) {
	f := startCore(t, "simulation.json")
	data := append([]byte(nil), f.client.commands.data...)
	// Corrupt only the retired field in an otherwise valid, daemon-produced
	// header. No second production encoder is introduced by this negative test.
	wire.PutUint32(data[52:], 256)
	fd, err := unix.MemfdCreate("rtcore-invalid-retired-region", unix.MFD_CLOEXEC)
	if err != nil {
		t.Fatal(err)
	}
	defer unix.Close(fd)
	if err := unix.Ftruncate(fd, int64(len(data))); err != nil {
		t.Fatal(err)
	}
	if n, err := unix.Write(fd, data); err != nil || n != len(data) {
		t.Fatalf("write negative fixture: %d, %v", n, err)
	}
	region, err := mapRing(fd, 1, uint32(f.client.NumAxes()))
	if err == nil {
		_ = unix.Munmap(region.data)
		t.Fatal("client accepted the retired setpoint region without a protocol transition")
	}
}

func TestCompilerCompletionTimeoutIsExplicit(t *testing.T) {
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "config/machines/simulation/simulation.json"))
	if err != nil {
		t.Fatal(err)
	}
	for _, axis := range []int{0, 1} {
		for _, test := range []struct {
			name  string
			value any
		}{
			{"missing", nil}, {"zero", 0}, {"negative", -1}, {"fraction", 0.5},
			{"boolean", true}, {"string", "500"}, {"overflow", uint64(1) << 32},
		} {
			t.Run(fmt.Sprintf("axis%d/%s", axis, test.name), func(t *testing.T) {
				var machine map[string]any
				if err := json.Unmarshal(raw, &machine); err != nil {
					t.Fatal(err)
				}
				limits := machine["axes"].([]any)[axis].(map[string]any)["limits"].(map[string]any)
				if test.value == nil {
					delete(limits, "completion_timeout_ms")
				} else {
					limits["completion_timeout_ms"] = test.value
				}
				mutated, err := json.Marshal(machine)
				if err != nil {
					t.Fatal(err)
				}
				directory := t.TempDir()
				path := filepath.Join(directory, "machine.json")
				if err := os.WriteFile(path, mutated, 0600); err != nil {
					t.Fatal(err)
				}
				_, _, err = simfixture.Compile(root, path, directory)
				output := simfixture.ErrorText(err)
				diagnostic := "completion timeout must be an integer"
				if test.value == nil {
					// The Go compiler reports a missing field by name; the retired Python compiler said
					// "completion timeout must be an integer" for the same input. Rejection is the contract.
					diagnostic = "missing field completion_timeout_ms"
				}
				if err == nil || !strings.Contains(string(output), diagnostic) {
					t.Fatalf("invalid completion timeout not rejected by compiler: %v: %s", err, output)
				}
			})
		}
	}
}

func TestNativeCompletionTimeoutIsExplicit(t *testing.T) {
	f := compiledForValidation(t, "simulation.json")
	var tracking []string
	for _, line := range strings.Split(f.profile, "\n") {
		if strings.HasPrefix(line, "tracking=") {
			tracking = strings.Split(strings.TrimPrefix(line, "tracking="), "|")
			break
		}
	}
	if len(tracking) != 6 || tracking[4] != "100" || tracking[5] != "500" {
		t.Fatalf("compiler did not preserve distinct following/completion timeouts: %v", tracking)
	}
	base := strings.Join(tracking[:5], "|")
	for _, test := range []struct{ name, value string }{
		{"missing", base}, {"empty", base + "|"}, {"zero", base + "|0"},
		{"negative", base + "|-1"}, {"fraction", base + "|0.5"},
		{"nan", base + "|nan"}, {"infinity", base + "|inf"},
		{"overflow", base + "|4294967296"}, {"extra", base + "|500|600"},
	} {
		t.Run(test.name, func(t *testing.T) {
			profile := editProfileField(t, f.profile, 0, "tracking", &test.value)
			output, err := f.validate(t, profile)
			if exit, ok := err.(*exec.ExitError); !ok || exit.ExitCode() != 2 {
				t.Fatalf("invalid native completion timeout accepted: %v: %s", err, output)
			}
		})
	}
	for _, value := range []string{"1", "4294967295"} {
		t.Run("valid/"+value, func(t *testing.T) {
			replacement := base + "|" + value
			profile := editProfileField(t, f.profile, 0, "tracking", &replacement)
			if output, err := f.validate(t, profile); err != nil {
				t.Fatalf("positive uint32 completion timeout rejected: %v: %s", err, output)
			}
		})
	}
}

func TestDescribeReportsSeparateSettlingAndFollowingTimeouts(t *testing.T) {
	f := startCore(t, "simulation.json")
	description, err := f.client.Describe()
	if err != nil {
		t.Fatal(err)
	}
	for _, axis := range description.Axes {
		if axis.CompletionTimeoutNS != 500_000_000 || axis.FollowingErrorTimeoutNS != 100_000_000 {
			t.Fatalf("axis %s: completion=%d, following=%d ns", axis.ID, axis.CompletionTimeoutNS, axis.FollowingErrorTimeoutNS)
		}
	}
}
