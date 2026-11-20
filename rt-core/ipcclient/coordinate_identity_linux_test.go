//go:build linux

package ipcclient

import (
	"context"
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"rosieos/rt-core/adapters/rosie/control/anchors"
	"rosieos/rt-core/internal/simfixture"
)

// editCoordinateField keeps native-only scalar probes explicit without dropping
// the admission table or deriving their expected semantics from daemon output.
func editCoordinateField(t *testing.T, profile string, axis int, key, value string) string {
	t.Helper()
	current := -1
	for _, line := range strings.Split(profile, "\n") {
		if strings.HasPrefix(line, "axis=") {
			if _, err := fmt.Sscanf(line, "axis=%d", &current); err != nil {
				t.Fatal(err)
			}
		}
		if current != axis || !strings.HasPrefix(line, "coordinate_identity=") {
			continue
		}
		raw, err := hex.DecodeString(strings.TrimPrefix(line, "coordinate_identity="))
		if err != nil {
			t.Fatal(err)
		}
		fields := strings.Split(string(raw), "\n")
		for i, field := range fields {
			if strings.HasPrefix(field, key+"=") {
				fields[i] = key + "=" + value
				encoded := hex.EncodeToString([]byte(strings.Join(fields, "\n")))
				return editProfileField(t, profile, axis, "coordinate_identity", &encoded)
			}
		}
	}
	t.Fatalf("axis %d missing coordinate field %s", axis, key)
	return ""
}

// A missing whole table must refuse every axis before native admission, even
// when legacy argv omit the machine digest and restoration is never requested.
func TestCoordinateIdentityNativeMissingTables(t *testing.T) {
	// Measured fixture geometry: one simulation-home axis and nine axes in each live cell.
	for _, tc := range []struct {
		name string
		axes int
	}{{"simulation-home.json", 1}, {"machines/bench/nine-motors-1to1.json", 9}, {"machines/bench/rosie1420-six-motors.json", 6}} {
		t.Run(tc.name, func(t *testing.T) {
			f := compiledForValidation(t, tc.name)
			if output, err := f.validate(t, f.profile); err != nil {
				t.Fatal(err, output)
			}
			if count := strings.Count(f.profile, "\ncoordinate_identity="); count != tc.axes {
				t.Fatalf("coordinate tables: %d, want %d", count, tc.axes)
			}
			for axis := 0; axis < tc.axes; axis++ {
				for _, form := range []string{"missing", "empty"} {
					t.Run(fmt.Sprintf("axis%d/%s", axis, form), func(t *testing.T) {
						var replacement *string
						if form == "empty" {
							empty := ""
							replacement = &empty
						}
						profile := editProfileField(t, f.profile, axis, "coordinate_identity", replacement)
						output, err := f.validate(t, profile)
						name := fmt.Sprintf("J%d", axis+1)
						if tc.axes == 1 {
							name = "axis_0"
						}
						if err == nil || !strings.Contains(output, fmt.Sprintf("missing coordinate_identity axis=%d name=%s", axis, name)) {
							t.Fatalf("whole-table refusal: %v %s", err, output)
						}
					})
				}
			}
		})
	}
}

// Native admission must reject each missing compiled component on the unchanged
// bench and Jetson cells before creating sockets or permitting bus operations.
func TestCoordinateIdentityNativeComponentRefusalsBenchAndJetson(t *testing.T) {
	for _, name := range []string{"machines/bench/nine-motors-1to1.json", "machines/bench/rosie1420-six-motors.json"} {
		t.Run(name, func(t *testing.T) {
			f := compiledForValidation(t, name)
			if output, err := f.validate(t, f.profile); err != nil {
				t.Fatal(err, output)
			}
			_, suffix, found := strings.Cut(f.profile, "coordinate_identity=")
			if !found {
				t.Fatal("unchanged configuration omitted identity")
			}
			encoded, _, _ := strings.Cut(suffix, "\n")
			fields, err := hex.DecodeString(encoded)
			if err != nil {
				t.Fatal(err)
			}
			lines := strings.Split(strings.TrimSuffix(string(fields), "\n"), "\n")
			if len(lines) != 13 {
				t.Fatalf("component count: %d", len(lines))
			}
			for i, line := range lines {
				key, _, _ := strings.Cut(line, "=")
				t.Run(key, func(t *testing.T) {
					remaining := append(append([]string(nil), lines[:i]...), lines[i+1:]...)
					replacement := hex.EncodeToString([]byte(strings.Join(remaining, "\n") + "\n"))
					profile := editProfileField(t, f.profile, 0, "coordinate_identity", &replacement)
					output, err := f.validate(t, profile)
					if err == nil || !strings.Contains(output, "incomplete coordinate identity") {
						t.Fatalf("missing %s: %v %s", key, err, output)
					}
				})
			}
		})
	}
}

// Observe every axis from unchanged committed inputs. Runtime scheduling is
// selected by the shared simulator harness; coordinate semantics stay compiled.
func TestCoordinateIdentityStatusForExistingConfigurations(t *testing.T) {
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	files, err := filepath.Glob(filepath.Join(root, "config", "machines", "simulation", "simulation*.json"))
	if err != nil {
		t.Fatal(err)
	}
	bench, err := filepath.Glob(filepath.Join(root, "config", "machines", "bench", "*.json"))
	if err != nil {
		t.Fatal(err)
	}
	for _, file := range append(files, bench...) {
		t.Run(filepath.Base(file), func(t *testing.T) {
			dir, err := os.MkdirTemp(filepath.Join(root, "build"), "identity-status-")
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() { _ = os.RemoveAll(dir) })
			compiled, args, err := simfixture.Compile(root, file, dir)
			simfixture.Must(t, err)
			var expected struct {
				SHA256 []string `json:"coordinate_identity_sha256"`
				Fields []string `json:"coordinate_identity"`
			}
			raw, err := os.ReadFile(filepath.Join(compiled, "coordinate.identity.json"))
			if err != nil {
				t.Fatal(err)
			}
			if err := json.Unmarshal(raw, &expected); err != nil {
				t.Fatal(err)
			}
			socketDir, err := os.MkdirTemp("", "ci-")
			if err != nil {
				t.Fatal(err)
			}
			t.Cleanup(func() { _ = os.RemoveAll(socketDir) })
			socket := filepath.Join(socketDir, "ipc.sock")
			// Unverified observation/shutdown bound: 8000 ms; success requires axis status.
			ctx, cancel := context.WithTimeout(context.Background(), 8*time.Second)
			defer cancel()
			process := exec.Command(simfixture.Locate(root, "rosie-rt-core-sim"), append(args, "--socket-path", socket)...)
			simfixture.SpawnLogged(t, process, filepath.Join(dir, "daemon.log"), 8*time.Second, 0, nil)
			// Unverified polling cadence: 10 ms; elapsed time never establishes success.
			err = simfixture.Poll(ctx, 10*time.Millisecond, func() (bool, error) {
				raw, err := os.ReadFile(filepath.Join(filepath.Dir(socket), "metrics.json"))
				if err != nil {
					return false, nil
				}
				var s Status
				if json.Unmarshal(raw, &s) != nil || len(s.Axes) != len(expected.SHA256) {
					return false, nil
				}
				for i, axis := range s.Axes {
					if len(axis.CoordinateIdentitySHA256) != 64 || axis.CoordinateIdentitySHA256 != expected.SHA256[i] || axis.CoordinateIdentity != expected.Fields[i] || strings.Contains(axis.CoordinateIdentity, "pair.") {
						t.Fatalf("axis %d identity mismatch: %+v", i, axis)
					}
				}
				return true, nil
			})
			if err != nil {
				t.Fatal(err)
			}
		})
	}
}

// Restart the real daemon with compiler inputs changed independently of the
// saved Home. Positive assertions require native receipts and status, not sleep.
func TestCoordinateIdentityPresetRestartAndComponentRefusals(t *testing.T) {
	t.Setenv("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "0")
	machine := compiledCoordinateJSON(t, "config/machines/a6ec-bench.example.json")
	machine["backend"] = "simulation"
	if _, exists := machine["pair"]; exists {
		t.Fatal("configuration must not contain pair metadata")
	}
	machine["runtime"] = map[string]any{"cpu": 0, "priority": 1}
	// Unverified simulation scheduling budget: 20000000 ns.
	machine["max_cycle_lateness_ns"] = 20000000
	axis := machine["axes"].([]any)[0].(map[string]any)
	axis["motor_encoder_counts_per_rev"], axis["profile"] = 131072, "coordinate_sim"
	axis["coordinate_evidence_mode"] = "multi_turn"
	profile := compiledCoordinateJSON(t, "config/drives/a6ec_1b02.json")
	profile["brake"] = map[string]any{"present": false}
	policy := profile["coordinate_evidence"].(map[string]any)
	// Unverified stationary fixture: 1000 counts/s, 200000000 ns age, 20000000 ns cadence.
	policy["maximum_physical_speed_counts_per_second"], policy["maximum_acquisition_age_ns"], policy["sample_interval_ns"] = 1000, 200000000, 20000000
	compiled, digest, err := compileCoordinateJSON(t, machine, profile)
	if err != nil {
		t.Fatal(err, digest)
	}
	store := anchors.Store{Directory: filepath.Join(filepath.Dir(compiled.path), "anchors")}
	t.Setenv("ROSIE_RT_ANCHOR_DIR", store.Directory)
	f, stop := coordinateModelDaemon(t, compiled, digest)
	coordinateHome(t, f, 1)
	homed := wait(t, f.client, func(s Status) bool {
		return s.HomeValidMask == 1 && s.CommissioningPhase == 0 && s.Axes[0].CoordinateSourceValid == 1
	})
	saved := coordinateModelRecord(homed)
	if len(saved.CoordinateIdentitySHA256) != 64 {
		t.Fatal("missing coordinate identity")
	}
	if err := store.Save(saved); err != nil {
		t.Fatal(err)
	}
	stop()
	// Keep a real captured record's drive/count evidence and valid checksum,
	// changing only the source contract under which it was captured. Both a
	// different source and a different version must refuse before restoration.
	for _, source := range []string{"rotation_mode_encoder|1", "single_turn_encoder|1", "raw_multi_turn_encoder|2"} {
		t.Run("anchor_source/"+source, func(t *testing.T) {
			other := saved
			other.CoordinateIdentity = strings.Replace(saved.CoordinateIdentity,
				"anchor.source=raw_multi_turn_encoder|3\n", "anchor.source="+source+"\n", 1)
			other.CoordinateIdentitySHA256 = fmt.Sprintf("%x", sha256.Sum256([]byte(other.CoordinateIdentity)))
			if other.CoordinateIdentitySHA256 == saved.CoordinateIdentitySHA256 {
				t.Fatal("source/version did not change coordinate digest")
			}
			if err := store.Save(other); err != nil {
				t.Fatal(err)
			}
			f, stop := coordinateModelDaemon(t, compiled, digest)
			defer stop()
			s := wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == "anchor_identity_mismatch" })
			coordinateModelRestore(t, f, false)
			// Native CoordinateReason::IdentityChanged is 3.
			if s.Axes[0].AnchorIdentityChangedField != "anchor.source" || s.Axes[0].CoordinateReason != 3 ||
				s.HomeValidMask != 0 || s.Armed != 0 || s.AxisEnableMask != 0 || s.Axes[0].PositionCounts != 0 {
				t.Fatal("other source restored or failed without naming anchor.source", s)
			}
		})
	}
	if err := store.Save(saved); err != nil {
		t.Fatal(err)
	}
	clone := func(value map[string]any) map[string]any {
		raw, _ := json.Marshal(value)
		var copy map[string]any
		if err := json.Unmarshal(raw, &copy); err != nil {
			t.Fatal(err)
		}
		return copy
	}
	for _, change := range []struct {
		name, field string
		edit        func(map[string]any, map[string]any, map[string]any)
	}{
		{"preset_caps_limits", "", func(m, a, p map[string]any) {
			rich := compiledCoordinateJSON(t, "config/drives/a6ec_bench_1to1.json")
			if p["ethercat"].(map[string]any)["tx_pdo"] == rich["ethercat"].(map[string]any)["tx_pdo"] {
				t.Fatal("anchor fixture must switch between distinct PDO presets")
			}
			p["ethercat"] = rich["ethercat"]
			// Unverified stationary fixture: 20 motor rpm, independent of the coordinate frame.
			m["drive_speed_limit_motor_rpm"] = 20
			a["limits"].(map[string]any)["max"] = 0.2
			a["limits"].(map[string]any)["max_velocity"] = 0.02
		}},
		{"gear", "gear.numerator", func(m, a, p map[string]any) {
			a["gear_ratio"] = map[string]any{"numerator": 2, "denominator": 1}
		}},
		{"encoder", "encoder.counts_per_motor_rev", func(m, a, p map[string]any) { a["motor_encoder_counts_per_rev"] = 262144 }},
		{"sign", "direction.sign", func(m, a, p map[string]any) { a["sign"] = -1 }},
		{"home", "home.policy", func(m, a, p map[string]any) { p["home_truth_sign"] = 1 }},
		{"pair_metadata_ignored", "", func(m, a, p map[string]any) { m["pair"] = map[string]any{"id": "other", "revision": 2} }},
	} {
		t.Run(change.name, func(t *testing.T) {
			m, p := clone(machine), clone(profile)
			a := m["axes"].([]any)[0].(map[string]any)
			change.edit(m, a, p)
			revised, hash, err := compileCoordinateJSON(t, m, p)
			if err != nil {
				t.Fatal(err, hash)
			}
			if hash == digest {
				t.Fatal("configuration change not bound")
			}
			f, stop := coordinateModelDaemon(t, revised, hash)
			defer stop()
			want := "anchor_identity_mismatch"
			if change.field == "" {
				want = "anchor_ready"
			}
			s := wait(t, f.client, func(s Status) bool { return s.Axes[0].AnchorReason == want })
			if s.HomeValidMask != 0 {
				t.Fatal("startup restored Home implicitly")
			}
			coordinateModelRestore(t, f, change.field == "")
			if change.field == "" {
				s = wait(t, f.client, func(s Status) bool { return s.HomeValidMask == 1 && s.CommissioningPhase == 0 })
				if s.Axes[0].CoordinateIdentitySHA256 != saved.CoordinateIdentitySHA256 || s.Axes[0].RestoredAnchorHomeEpoch != saved.HomeEpoch {
					t.Fatal(s)
				}
			} else {
				// Measured against the native CoordinateReason enum: IdentityChanged = 3.
				if s.Axes[0].AnchorIdentityChangedField != change.field || s.Axes[0].CoordinateReason != 3 {
					t.Fatal(s.Axes[0])
				}
				files, _ := filepath.Glob(filepath.Join(filepath.Dir(revised.path), "anchor-daemon-*.log"))
				var logs string
				for _, file := range files {
					raw, _ := os.ReadFile(file)
					logs += string(raw)
				}
				if !strings.Contains(logs, "anchor_identity_mismatch axis=0 field="+change.field+" Home required") {
					t.Fatal(logs)
				}
			}
			if s.Armed != 0 || s.AxisEnableMask != 0 || s.Axes[0].PositionCounts != 0 || s.Axes[0].CoordinateCounts != 0 {
				t.Fatal("restore moved or enabled axis", s)
			}
		})
	}
}
