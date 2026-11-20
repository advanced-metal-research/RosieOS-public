package rtctl

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// Unverified test fixture: one rotary axis, 1 ms cycle, 1000 counts/rev and explicit radian limits.
const minimalJSON = `{
 "machine":{"schema_version":1,"backend":"simulation","cycle_ns":1000000,"max_cycle_lateness_ns":20000000,
 "max_motor_rpm":100,"runtime":{"cpu":0,"priority":1},"axes":[{"name":"axis_0","profile":"test",
 "slave_position":0,"type":"rotary","wire_counts_per_rev":1000,"wire_revs_per_axis_rev":1,"sign":1,
 "motor_encoder_counts_per_rev":1000,"coordinate_evidence_mode":"multi_turn",
 "require_home":false,"limits":{"min":-1,"max":1,"max_velocity":1,"jog_acceleration":1,
 "max_target_lead":0.1,"following_error":0.2,"completion_tolerance":0.01,
 "following_error_timeout_ms":100,"completion_timeout_ms":500}}]},
 "profile":{"ethercat":{"vendor_id":1,"product_code":1,"revision_no":0,"rx_pdo":5632,"tx_pdo":6656,
 "rx_sync":2,"tx_sync":3,"dc_quantum_ns":250000,"dc_assign_activate":768,
 "rx_layout":[{"semantic":"cw","index":24640,"subindex":0,"bits":16},
 {"semantic":"target_pos","index":24698,"subindex":0,"bits":32},{"semantic":"mode","index":24672,"subindex":0,"bits":8}],
 "tx_layout":[{"semantic":"sw","index":24641,"subindex":0,"bits":16},
 {"semantic":"pos","index":24676,"subindex":0,"bits":32},{"semantic":"mode_disp","index":24673,"subindex":0,"bits":8}]},
 "home_truth_sign":1}}
`

// Unverified home fixture uses CiA402 mode values and a signed 32-bit truth object.
const homeJSON = `{"steady_state_mode":8,"commissioning_mode":6,
 "truth_source":{"index":24700,"subindex":0,"type":"i32"},"transaction":[]}`

// Unverified coordinate fixture: counts/s, ns, count errors and validity bits are explicit.
const coordinateJSON = `{"maximum_physical_speed_counts_per_second":1000,"maximum_acquisition_age_ns":1000000,
 "maximum_feedback_error_counts":1,"maximum_anchor_error_counts":1,"validity_mask":3,"validity_value":1}`

// Test the bench descriptor independently of generated artifacts so incorrect
// continuous PDO semantics or unsupported remapping cannot enter compilation.
// The surveyed presets are complete fixed maps. Reject fabricated, reordered or
// width-mismatched entries before a live configuration can claim their signals.
func TestA6DriveTelemetryPresets(t *testing.T) {
	for _, profile := range []string{"a6ec", "a6ec_bench_1to1", "a6ec_1b02"} {
		t.Run(profile, func(t *testing.T) {
			f := fresh()
			f.put("profile", string(readTestFile(t, "config/drives/"+profile+".json")))
			f.put("machine.axes.0.wire_counts_per_rev", "131072")
			f.put("machine.axes.0.motor_encoder_counts_per_rev", "131072")
			result, err := f.compile()
			if err != nil {
				t.Fatal(err)
			}
			rich := profile != "a6ec_1b02"
			if strings.Contains(result.AxisProfiles, "velocity_actual|24684|0|32") != rich ||
				strings.Contains(result.AxisProfiles, "following_error|24820|0|32") != rich ||
				strings.Contains(result.AxisProfiles, "di|24829|0|32") == rich {
				t.Fatal(result.AxisProfiles)
			}
			if !rich {
				return
			}
		})
	}
}

func TestA6FixedPDOSampledEvidence(t *testing.T) {
	f := fresh()
	// Unverified fixture cap: 30 wire rpm * 100/1 = rated 3000 motor rpm.
	f.put("machine.max_motor_rpm", "30")
	f.put("profile", string(readTestFile(t, "config/drives/a6ec.json")))
	f.put("machine.axes.0.wire_counts_per_rev", "131072")
	f.put("machine.axes.0.motor_encoder_counts_per_rev", "131072")
	f.put("machine.axes.0.wire_revs_per_axis_rev", "100")
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(compiled.AxisProfiles, "coordinate_evidence=5000000|3000000000|16|16|a6ec_sdo|1500000000\n") ||
		!strings.Contains(compiled.AxisProfiles, "tp_status|24761|0|16") || strings.Contains(compiled.AxisProfiles, "continuous_valid") {
		t.Fatalf("fixed layout or SDO policy missing: %s", compiled.AxisProfiles)
	}
	for _, change := range []struct{ path, value string }{
		{"profile.coordinate_evidence.validity_mask", "65535"},
		{"profile.coordinate_evidence.source", `"pdo"`},
		{"profile.coordinate_evidence.sample_interval_ns", "3000000000"},
		{"machine.bus_bringup", `{"explicit_pdo_config":true}`},
	} {
		t.Run(change.path, func(t *testing.T) {
			clone := &fixture{asObject(decode([]byte(canonical(f.data))))}
			clone.put(change.path, change.value)
			if _, err := clone.compile(); err == nil {
				t.Fatal("unsupported fixed PDO evidence accepted")
			}
		})
	}
}

// The exact memory admission boundary fails at one ns beyond capacity,
// preventing a compiled sampled axis whose native history is silently empty.
func TestSampledPDOHistoryCompilerBound(t *testing.T) {
	for _, tc := range []struct {
		age      string
		rejected bool
	}{
		{"131070000000", false}, {"131070000001", true}, {"18446744073709551615", true},
	} {
		t.Run(tc.age, func(t *testing.T) {
			f := fresh()
			// Unverified fixture cap: 30 wire rpm * 100/1 = rated 3000 motor rpm.
			f.put("machine.max_motor_rpm", "30")
			f.put("profile", string(readTestFile(t, "config/drives/a6ec.json")))
			f.put("machine.axes.0.wire_counts_per_rev", "131072")
			f.put("machine.axes.0.motor_encoder_counts_per_rev", "131072")
			f.put("machine.axes.0.wire_revs_per_axis_rev", "100")
			f.put("profile.coordinate_evidence.maximum_acquisition_age_ns", tc.age)
			_, err := f.compile()
			if tc.rejected {
				if err == nil || !strings.Contains(err.Error(), "sampled PDO history exceeds 131072 entries") {
					t.Fatalf("bound refusal: %v", err)
				}
			} else if err != nil {
				t.Fatal(err)
			}
		})
	}
}

type fixture struct{ data object }

func fresh() *fixture { return &fixture{asObject(decode([]byte(minimalJSON)))} }

func replace(value any, path []string, replacement any, remove bool) any {
	if len(path) == 0 {
		return replacement
	}
	if items, ok := value.([]any); ok {
		i, err := strconv.Atoi(path[0])
		if err != nil {
			panic(err)
		}
		items[i] = replace(items[i], path[1:], replacement, remove)
		return items
	}
	o := asObject(value)
	if len(path) == 1 && remove {
		for i, m := range o {
			if m.key == path[0] {
				return append(o[:i], o[i+1:]...)
			}
		}
		return o
	}
	var nested any
	if len(path) > 1 {
		nested = o.get(path[0])
	}
	o.set(path[0], replace(nested, path[1:], replacement, remove))
	return o
}

func (f *fixture) put(path, value string) {
	f.data = asObject(replace(f.data, strings.Split(path, "."), decode([]byte(value)), false))
}
func (f *fixture) remove(path string) {
	f.data = asObject(replace(f.data, strings.Split(path, "."), nil, true))
}
func (f *fixture) get(path string) any {
	var value any = f.data
	for _, p := range strings.Split(path, ".") {
		if a, ok := value.([]any); ok {
			i, _ := strconv.Atoi(p)
			value = a[i]
		} else {
			value = asObject(value).get(p)
		}
	}
	return value
}
func (f *fixture) compile() (result Compiled, err error) {
	defer validationBoundary(&err)
	machine := asObject(f.data.get("machine"))
	profiles := make([]any, len(asArray(machine.get("axes"))))
	for i := range profiles {
		profiles[i] = f.data.get("profile")
	}
	return compile(machine, profiles, loadTuningCatalogs("../../config/drives")), nil
}
func (f *fixture) coordinate() {
	f.put("profile.coordinate_evidence", coordinateJSON)
	tx := asArray(f.get("profile.ethercat.tx_layout"))
	tx = append(tx, decode([]byte(`{"semantic":"continuous_pos","index":8256,"subindex":33,"bits":32}`)),
		decode([]byte(`{"semantic":"continuous_valid","index":8256,"subindex":67,"bits":16}`)))
	f.put("profile.ethercat.tx_layout", canonical(tx))
}

func testRoot(t *testing.T) string {
	t.Helper()
	root, err := filepath.Abs(filepath.Join("..", ".."))
	if err != nil {
		t.Fatal(err)
	}
	return root
}

func buildTemp(t *testing.T) string {
	t.Helper()
	parent := filepath.Join(testRoot(t), "build")
	if err := os.MkdirAll(parent, 0o755); err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(parent, "rtctl-test-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() {
		if err := os.RemoveAll(dir); err != nil {
			t.Error(err)
		}
	})
	return dir
}

func TestIntegerRules(t *testing.T) {
	// Unverified oracle ranges in ns, ms, counts, PDO indices, sync indices, CPU indices and FIFO priority.
	rules := []struct {
		path       string
		min        int64
		max        uint64
		coordinate bool
	}{
		{"machine.cycle_ns", 250000, 10000000, false}, {"machine.max_cycle_lateness_ns", 1, 249999999, false},
		{"machine.axes.0.slave_position", 0, 65535, false}, {"machine.axes.0.wire_counts_per_rev", 1, 2147483647, false},
		{"machine.axes.0.sign", -1, 1, false}, {"machine.runtime.cpu", 0, 1023, false}, {"machine.runtime.priority", 1, 99, false},
		{"machine.axes.0.limits.following_error_timeout_ms", 1, 1000, false}, {"machine.axes.0.limits.completion_timeout_ms", 1, 4294967295, false},
		{"profile.home_truth_sign", -1, 1, false}, {"profile.ethercat.vendor_id", 1, 4294967295, false},
		{"profile.ethercat.product_code", 1, 4294967295, false}, {"profile.ethercat.revision_no", 0, 4294967295, false},
		{"profile.ethercat.rx_pdo", 1, 65535, false}, {"profile.ethercat.tx_pdo", 1, 65535, false},
		{"profile.ethercat.rx_sync", 0, 15, false}, {"profile.ethercat.tx_sync", 0, 15, false},
		{"profile.ethercat.dc_quantum_ns", 0, 4294967295, false}, {"profile.ethercat.dc_assign_activate", 0, 65535, false},
		{"profile.ethercat.rx_layout.0.index", 1, 65535, false}, {"profile.ethercat.rx_layout.0.subindex", 0, 255, false},
		{"profile.ethercat.tx_layout.0.index", 1, 65535, false}, {"profile.ethercat.tx_layout.0.subindex", 0, 255, false},
		{"profile.coordinate_evidence.maximum_physical_speed_counts_per_second", 1, 18446744073709551615, true},
		{"profile.coordinate_evidence.maximum_acquisition_age_ns", 1, 18446744073709551615, true},
		{"profile.coordinate_evidence.maximum_feedback_error_counts", 0, 18446744073709551615, true},
		{"profile.coordinate_evidence.maximum_anchor_error_counts", 0, 18446744073709551615, true},
		{"profile.coordinate_evidence.validity_mask", 1, 65535, true}, {"profile.coordinate_evidence.validity_value", 0, 65535, true},
	}
	for _, rule := range rules {
		upper := strconv.FormatUint(rule.max+1, 10)
		if rule.max == ^uint64(0) {
			upper = "18446744073709551616"
		}
		for _, bad := range []string{strconv.FormatInt(rule.min-1, 10), upper, "true", "1.0", "null", `"1"`, "missing"} {
			t.Run(rule.path+"/"+bad, func(t *testing.T) {
				f := fresh()
				if rule.coordinate {
					f.coordinate()
				}
				if _, err := f.compile(); err != nil {
					t.Fatalf("invalid control fixture: %v", err)
				}
				if bad == "missing" {
					f.remove(rule.path)
				} else {
					f.put(rule.path, bad)
				}
				if _, err := f.compile(); err == nil {
					t.Fatal("accepted invalid integer")
				}
			})
		}
	}
}

func TestFiniteRules(t *testing.T) {
	for _, path := range []string{"machine.max_motor_rpm", "machine.axes.0.wire_revs_per_axis_rev", "machine.axes.0.lead_m_per_rev",
		"machine.axes.0.limits.min", "machine.axes.0.limits.max", "machine.axes.0.limits.max_velocity",
		"machine.axes.0.limits.jog_acceleration", "machine.axes.0.limits.max_target_lead",
		"machine.axes.0.limits.following_error", "machine.axes.0.limits.completion_tolerance"} {
		for _, bad := range []string{"true", "null", `"1"`, "1e400", "-1e400"} {
			t.Run(path+"/"+bad, func(t *testing.T) {
				f := fresh()
				f.put(path, bad)
				if _, err := f.compile(); err == nil {
					t.Fatal("accepted nonfinite/non-numeric quantity")
				}
			})
		}
	}
}

func TestRejectionRules(t *testing.T) {
	cases := []struct {
		name string
		edit func(*fixture)
	}{
		{"schema", func(f *fixture) { f.put("machine.schema_version", "2") }},
		{"no_axes", func(f *fixture) { f.put("machine.axes", "[]") }},
		{"axes_type", func(f *fixture) { f.put("machine.axes", "{}") }},
		{"too_many_axes", func(f *fixture) {
			a := asArray(f.get("machine.axes"))
			for len(a) < 17 {
				a = append(a, a[0])
			}
			f.put("machine.axes", canonical(a))
		}},
		{"duplicate_name", func(f *fixture) {
			a := asArray(f.get("machine.axes"))
			f.put("machine.axes", canonical(append(a, decode([]byte(canonical(a[0]))))))
			f.put("machine.axes.1.slave_position", "1")
		}},
		{"duplicate_position", func(f *fixture) {
			a := asArray(f.get("machine.axes"))
			f.put("machine.axes", canonical(append(a, decode([]byte(canonical(a[0]))))))
			f.put("machine.axes.1.name", `"other"`)
		}},
		{"simulation_live", func(f *fixture) { f.put("profile.simulation_only", "true"); f.put("machine.backend", `"live"`) }},
		{"simulation_ipc", func(f *fixture) { f.put("profile.simulation_only", "true"); f.put("machine.backend", `"ipc-only"`) }},
		{"simulation_unspecified", func(f *fixture) { f.put("profile.simulation_only", "true"); f.remove("machine.backend") }},
		{"cycle_quantum", func(f *fixture) { f.put("machine.cycle_ns", "1000001") }},
		{"same_sync", func(f *fixture) { f.put("profile.ethercat.tx_sync", "2") }},
		{"too_many_pdos", func(f *fixture) {
			rx := asArray(f.get("profile.ethercat.rx_layout"))
			for len(rx) < 30 {
				rx = append(rx, rx[0])
			}
			f.put("profile.ethercat.rx_layout", canonical(rx))
		}},
		{"zero_home_sign", func(f *fixture) { f.put("profile.home_truth_sign", "0") }},
		{"zero_axis_sign", func(f *fixture) { f.put("machine.axes.0.sign", "0") }},
		{"equal_limits", func(f *fixture) { f.put("machine.axes.0.limits.min", "1") }},
		{"reversed_limits", func(f *fixture) { f.put("machine.axes.0.limits.min", "2") }},
		{"home_type", func(f *fixture) { f.put("machine.axes.0.require_home", "1") }},
		{"home_missing", func(f *fixture) { f.put("machine.axes.0.require_home", "true") }},
		{"unknown_kind", func(f *fixture) { f.put("machine.axes.0.type", `"helical"`) }},
		{"linear_without_lead", func(f *fixture) { f.put("machine.axes.0.type", `"linear"`) }},
		{"linear_negative_lead", func(f *fixture) {
			f.put("machine.axes.0.type", `"linear"`)
			f.put("machine.axes.0.lead_m_per_rev", "-1")
		}},
		{"linear_wrapped_feedback", func(f *fixture) {
			f.put("machine.axes.0.type", `"linear"`)
			f.put("machine.axes.0.lead_m_per_rev", "0.01")
			f.put("profile.feedback_counts_wrap", "true")
		}},
		{"linear_wrapped_command", func(f *fixture) {
			f.put("machine.axes.0.type", `"linear"`)
			f.put("machine.axes.0.lead_m_per_rev", "0.01")
			f.put("profile.command_counts_wrap", "true")
		}},
		{"coordinate_feedback", func(f *fixture) { f.coordinate(); f.put("profile.feedback_counts_wrap", "false") }},
		{"coordinate_command", func(f *fixture) { f.coordinate(); f.put("profile.command_counts_wrap", "true") }},
		{"coordinate_no_pdos", func(f *fixture) {
			f.coordinate()
			f.put("profile.ethercat.tx_layout", canonical(asArray(f.get("profile.ethercat.tx_layout"))[:3]))
		}},
		{"coordinate_no_validity", func(f *fixture) {
			f.coordinate()
			f.put("profile.ethercat.tx_layout", canonical(asArray(f.get("profile.ethercat.tx_layout"))[:4]))
		}},
		{"coordinate_no_position", func(f *fixture) {
			f.coordinate()
			tx := asArray(f.get("profile.ethercat.tx_layout"))
			f.put("profile.ethercat.tx_layout", canonical(append(tx[:3], tx[4])))
		}},
		{"coordinate_no_policy", func(f *fixture) { f.coordinate(); f.remove("profile.coordinate_evidence") }},
		{"coordinate_null_policy", func(f *fixture) { f.coordinate(); f.put("profile.coordinate_evidence", "null") }},
		{"coordinate_single_pdo_no_policy", func(f *fixture) {
			f.coordinate()
			f.remove("profile.coordinate_evidence")
			f.put("profile.ethercat.tx_layout", canonical(asArray(f.get("profile.ethercat.tx_layout"))[:4]))
		}},
		{"coordinate_mask", func(f *fixture) { f.coordinate(); f.put("profile.coordinate_evidence.validity_value", "4") }},
	}
	for _, path := range []string{"machine.max_motor_rpm", "machine.axes.0.wire_revs_per_axis_rev", "machine.axes.0.limits.max_velocity",
		"machine.axes.0.limits.jog_acceleration", "machine.axes.0.limits.max_target_lead", "machine.axes.0.limits.following_error", "machine.axes.0.limits.completion_tolerance"} {
		for _, value := range []string{"0", "-1"} {
			cases = append(cases, struct {
				name string
				edit func(*fixture)
			}{path + "/" + value, func(f *fixture) { f.put(path, value) }})
		}
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			f := fresh()
			tc.edit(f)
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted invalid configuration")
			}
		})
	}
}

func TestTokens(t *testing.T) {
	for _, path := range []string{"machine.axes.0.name", "machine.axes.0.profile", "profile.ethercat.rx_layout.0.semantic", "profile.ethercat.tx_layout.0.semantic"} {
		for _, bad := range []string{`""`, `"../a6ec"`, `"x/y"`, `"x\\y"`, `"a-b"`, `"a.b"`, `"x\naxis=1"`, `"x|y"`, `"x;y"`, `"é"`, `null`, `7`} {
			t.Run(path+"/"+bad, func(t *testing.T) {
				f := fresh()
				f.put(path, bad)
				if path == "machine.axes.0.profile" {
					if _, err := Compile([]byte(canonical(f.get("machine"))), buildTemp(t)); err == nil {
						t.Fatal("accepted profile traversal")
					}
				} else if _, err := f.compile(); err == nil {
					t.Fatal("accepted invalid identifier")
				}
			})
		}
	}
	for _, good := range []string{"A_z09", "0", "_"} {
		if token(good) != good {
			t.Fatal("valid token changed")
		}
	}
}

func TestPDORules(t *testing.T) {
	// Unverified oracle widths in bits, specified independently of the implementation tables.
	for _, layout := range []struct {
		direction string
		widths    map[string]int
		required  []string
	}{
		{"rx", map[string]int{"cw": 16, "target_pos": 32, "target_vel": 32, "target_torque": 16, "mode": 8, "tp_func": 16, "max_profile_vel": 32}, []string{"cw", "target_pos", "mode"}},
		{"tx", map[string]int{"sw": 16, "pos": 32, "mode_disp": 8, "err": 16, "manufacturer_err": 32, "torque": 16, "di": 32, "tp_status": 16, "tp_pos1": 32, "tp_pos2": 32, "bus_voltage": 16, "load_rate": 16, "position_error": 32, "multi_turn_lo": 32, "multi_turn_hi": 32, "igbt_temp": 16, "motor_temp": 16, "drive_not_ready": 16, "motor_not_rotating": 16, "continuous_pos": 32, "continuous_valid": 16}, []string{"sw", "pos", "mode_disp"}},
	} {
		path := "profile.ethercat." + layout.direction + "_layout"
		for semantic, width := range layout.widths {
			t.Run(layout.direction+"/"+semantic, func(t *testing.T) {
				f := fresh()
				f.coordinate()
				entries := asArray(f.get(path))
				found := -1
				for i, e := range entries {
					if asObject(e).get("semantic") == semantic {
						found = i
					}
				}
				if found < 0 {
					found = len(entries)
					entries = append(entries, decode([]byte(fmt.Sprintf(`{"semantic":%q,"index":16000,"subindex":0,"bits":%d}`, semantic, width))))
				}
				f.put(path, canonical(entries))
				if _, err := f.compile(); err != nil {
					t.Fatalf("valid width rejected: %v", err)
				}
				f.put(fmt.Sprintf("%s.%d.bits", path, found), strconv.Itoa(width+1))
				if _, err := f.compile(); err == nil {
					t.Fatal("accepted wrong width")
				}
			})
		}
		for _, required := range layout.required {
			t.Run(layout.direction+"/missing_"+required, func(t *testing.T) {
				f := fresh()
				var entries []any
				for _, e := range asArray(f.get(path)) {
					if asObject(e).get("semantic") != required {
						entries = append(entries, e)
					}
				}
				f.put(path, canonical(entries))
				if _, err := f.compile(); err == nil {
					t.Fatal("accepted missing required PDO")
				}
			})
		}
		for _, bad := range []string{"unknown", "duplicate_semantic", "duplicate_object"} {
			t.Run(layout.direction+"/"+bad, func(t *testing.T) {
				f := fresh()
				switch bad {
				case "unknown":
					f.put(path+".0.semantic", `"unknown"`)
				case "duplicate_semantic":
					f.put(path+".1.semantic", canonical(f.get(path+".0.semantic")))
					f.put(path+".1.bits", canonical(f.get(path+".0.bits")))
				case "duplicate_object":
					f.put(path+".1.index", canonical(f.get(path+".0.index")))
				}
				if _, err := f.compile(); err == nil {
					t.Fatal("accepted invalid PDO")
				}
			})
		}
	}
}

func TestProfileTraversalWithExistingTarget(t *testing.T) {
	root := buildTemp(t)
	profiles := filepath.Join(root, "config/drives")
	f := fresh()
	writeTestFile(t, filepath.Join(root, "escaped.json"), canonical(f.get("profile")), 0o644)
	writeTestFile(t, filepath.Join(profiles, "nested", "escaped.json"), canonical(f.get("profile")), 0o644)
	for _, name := range []string{"../escaped", "nested/escaped", filepath.Join(root, "escaped")} {
		f.put("machine.axes.0.profile", asciiQuote(name))
		if _, err := Compile([]byte(canonical(f.get("machine"))), profiles); err == nil {
			t.Fatalf("accepted existing profile through forbidden path %q", name)
		}
	}
}

func TestNonfiniteMetadataRejected(t *testing.T) {
	for _, path := range []string{"machine.metadata", "profile.metadata"} {
		f := fresh()
		f.put(path, `{"overflow":1e400}`)
		if _, err := f.compile(); err == nil {
			t.Fatal("accepted nonfinite metadata in configuration identity")
		}
	}
}

func TestMalformedJSON(t *testing.T) {
	for _, input := range []string{"", "[]", "null", "{", `{ "axes": [] } {}`, `{"x":NaN}`, `{"x":Infinity}`} {
		if _, err := Compile([]byte(input), buildTemp(t)); err == nil {
			t.Errorf("accepted %q", input)
		}
	}
}

func TestCoordinateUint64AndMissingProfile(t *testing.T) {
	f := fresh()
	f.coordinate()
	f.put("profile.coordinate_evidence.maximum_acquisition_age_ns", "18446744073709551615")
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(compiled.AxisProfiles, "coordinate_evidence=1000|18446744073709551615|1|1|3|1\n") {
		t.Fatal("uint64 policy lost precision")
	}
	if _, err := Compile([]byte(canonical(f.get("machine"))), buildTemp(t)); err == nil {
		t.Fatal("accepted missing profile")
	}
}

func TestPythonFloat(t *testing.T) {
	// Unverified compatibility examples include radian limits, lead, ratios and decimal exponent boundaries.
	cases := map[string]string{"1": "1.0", "-0.0": "-0.0", "0.0001": "0.0001", "0.00001": "1e-05", "1e15": "1000000000000000.0",
		"1e16": "1e+16", "0.020000000000000004": "0.020000000000000004", "0.04000000000000001": "0.04000000000000001",
		"0.0002": "0.0002", "0.005": "0.005", "6.283185307179586": "6.283185307179586", "5e-324": "5e-324"}
	for input, want := range cases {
		if got := pythonFloat(finite(json.Number(input), "test")); got != want {
			t.Errorf("%s: %s != %s", input, got, want)
		}
	}
}
