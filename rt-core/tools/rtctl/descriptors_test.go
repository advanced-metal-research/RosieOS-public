package rtctl

import (
	"fmt"
	"strconv"
	"strings"
	"testing"
)

func (f *fixture) home(transaction string) {
	f.put("profile.native_home", homeJSON)
	f.put("profile.native_home.transaction", transaction)
}

func TestHomeRules(t *testing.T) {
	for _, path := range []string{"steady_state_mode", "commissioning_mode", "truth_source.index", "truth_source.subindex"} {
		// Unverified home bounds are DS402 modes, 16-bit object indices and 8-bit subindices.
		bad := []string{"null", "true", "1.0", "missing", "-1"}
		switch path {
		case "steady_state_mode", "commissioning_mode":
			bad = append(bad, "0", "128")
		case "truth_source.index":
			bad = append(bad, "0", "65536")
		case "truth_source.subindex":
			bad = append(bad, "256")
		}
		for _, value := range bad {
			t.Run(path+"/"+value, func(t *testing.T) {
				f := fresh()
				f.home("[]")
				if value == "missing" {
					f.remove("profile.native_home." + path)
				} else {
					f.put("profile.native_home."+path, value)
				}
				if _, err := f.compile(); err == nil {
					t.Fatal("accepted invalid home descriptor")
				}
			})
		}
	}
	for _, op := range []string{"set_mode", "restore_mode"} {
		for _, value := range []string{"0", "128", "true", "1.0", "null"} {
			t.Run(op+"/"+value, func(t *testing.T) {
				f := fresh()
				f.home(fmt.Sprintf(`[{"op":%q,"value":%s}]`, op, value))
				if _, err := f.compile(); err == nil {
					t.Fatal("accepted invalid home mode")
				}
			})
		}
	}
	cases := []string{
		`[{"op":"unknown"}]`, `[{"op":"set_mode;op|release_service_override"}]`,
		`[{"op":"write_sdo","index":1,"subindex":0,"type":"u64","value":0}]`,
		`[{"op":"write_sdo_wrap_fraction","index":24700,"subindex":0,"type":"i32","fraction_numerator":1,"fraction_denominator":2}]`,
		`[{"op":"write_sdo_wrap_fraction","index":1,"subindex":0,"type":"i32","fraction_numerator":-1,"fraction_denominator":2}]`,
		`[{"op":"write_sdo_wrap_fraction","index":1,"subindex":0,"type":"i32","fraction_numerator":65536,"fraction_denominator":2}]`,
		`[{"op":"write_sdo_wrap_fraction","index":1,"subindex":0,"type":"i32","fraction_numerator":1,"fraction_denominator":0}]`,
		`[{"op":"write_sdo_wrap_fraction","index":1,"subindex":0,"type":"i32","fraction_numerator":1,"fraction_denominator":65536}]`,
		`[{"op":"controlword_sequence","values":[]}]`, `[{"op":"controlword_sequence","values":[1,2,3,4,5,6,7,8,9]}]`,
		`[{"op":"controlword_sequence","values":[-1]}]`, `[{"op":"controlword_sequence","values":[65536]}]`,
		`[{"op":"controlword_sequence","values":[true]}]`,
		`[{"op":"wait_statusword","all_set_mask":-1,"all_clear_mask":0}]`,
		`[{"op":"wait_statusword","all_set_mask":65536,"all_clear_mask":0}]`,
		`[{"op":"wait_statusword","all_set_mask":0,"all_clear_mask":-1}]`,
		`[{"op":"wait_statusword","all_set_mask":0,"all_clear_mask":65536}]`,
	}
	for i, steps := range cases {
		t.Run(strconv.Itoa(i), func(t *testing.T) {
			f := fresh()
			f.home(steps)
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted invalid home operation")
			}
		})
	}
	f := fresh()
	f.home(`[{"op":"set_mode","value":6},{"op":"restore_mode","value":8},
	 {"op":"write_sdo","index":1,"subindex":0,"type":"i8","value":-128},
	 {"op":"wait_sdo","index":1,"subindex":0,"type":"u8","value":255},
	 {"op":"controlword_sequence","values":[6,7,15]}, {"op":"wait_statusword","all_set_mask":1,"all_clear_mask":2},
	 {"op":"refresh_truth"},{"op":"release_service_override"}]`)
	f.put("machine.axes.0.require_home", "true")
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	want := "native_home=steady_state_mode|8;commissioning_mode|6;truth_source|24700|0|i32;op|set_mode|6;op|restore_mode|8;op|write_sdo|1|0|i8|-128;op|wait_sdo|1|0|u8|255;op|controlword_sequence|6,7,15;op|wait_statusword|1|2;op|refresh_truth;op|release_service_override\n"
	if !strings.Contains(compiled.AxisProfiles, want) {
		t.Fatalf("home transaction differs: %s", compiled.AxisProfiles)
	}
}

func TestScalarRanges(t *testing.T) {
	// Unverified scalar bounds are exact signed/unsigned SDO values at 8, 16 and 32 bits.
	for _, scalar := range []struct {
		name   string
		lo, hi int64
	}{
		{"u8", 0, 255}, {"i8", -128, 127}, {"u16", 0, 65535}, {"i16", -32768, 32767}, {"u32", 0, 4294967295}, {"i32", -2147483648, 2147483647},
	} {
		for _, value := range []int64{scalar.lo - 1, scalar.lo, scalar.hi, scalar.hi + 1} {
			for _, mode := range []string{"startup", "write_sdo", "wait_sdo"} {
				t.Run(fmt.Sprintf("%s/%s/%d", scalar.name, mode, value), func(t *testing.T) {
					f := fresh()
					if mode == "startup" {
						f.put("profile.startup_defaults", fmt.Sprintf(`{"setting":%d}`, value))
						f.put("profile.startup_schema", fmt.Sprintf(`{"setting":{"type":%q,"min":-4294967296,"max":4294967296,"object":{"index":1,"subindex":0}}}`, scalar.name))
					} else {
						f.home(fmt.Sprintf(`[{"op":%q,"index":1,"subindex":0,"type":%q,"value":%d}]`, mode, scalar.name, value))
					}
					_, err := f.compile()
					valid := value >= scalar.lo && value <= scalar.hi
					if valid && err != nil {
						t.Fatalf("valid scalar rejected: %v", err)
					}
					if !valid && err == nil {
						t.Fatal("accepted scalar overflow")
					}
				})
			}
		}
	}
}

func (f *fixture) startup() {
	f.put("profile.startup_defaults", `{"setting":7}`)
	f.put("profile.startup_schema", `{"setting":{"type":"u16","min":5,"max":10,"object":{"index":1,"subindex":0}}}`)
}

func TestStartupRules(t *testing.T) {
	for _, value := range []string{"null", "true", "1.0", `"7"`, "4", "11"} {
		t.Run("value/"+value, func(t *testing.T) {
			f := fresh()
			f.startup()
			f.put("profile.startup_defaults.setting", value)
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted invalid startup setting")
			}
		})
	}
	for _, tc := range []struct{ path, value string }{
		{"profile.startup_schema.setting.type", `"u64"`}, {"profile.startup_schema.setting.object.index", "0"},
		{"profile.startup_schema.setting.object.index", "65536"}, {"profile.startup_schema.setting.object.subindex", "-1"},
		{"profile.startup_schema.setting.object.subindex", "256"}, {"machine.axes.0.startup", `{"unknown":1}`},
		{"machine.axes.0.startup", `{"setting":null}`}, {"profile.startup_defaults", `{"unknown":1}`},
	} {
		t.Run(tc.path+tc.value, func(t *testing.T) {
			f := fresh()
			f.startup()
			f.put(tc.path, tc.value)
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted invalid startup schema or override")
			}
		})
	}
	f := fresh()
	f.startup()
	var defaults, schema object
	for i := 0; i <= startupWritesPerAxisMaximum; i++ {
		key := fmt.Sprintf("setting%d", i)
		defaults.set(key, decode([]byte("7")))
		schema.set(key, f.get("profile.startup_schema.setting"))
	}
	f.put("profile.startup_defaults", canonical(defaults))
	f.put("profile.startup_schema", canonical(schema))
	if _, err := f.compile(); err == nil {
		t.Fatal("accepted more startup settings than the native per-axis capacity")
	}
	for _, bad := range []string{"bad|key", "bad;key", "bad\nkey"} {
		f = fresh()
		f.startup()
		f.put("profile.startup_defaults", "{"+asciiQuote(bad)+":7}")
		f.put("profile.startup_schema", "{"+asciiQuote(bad)+":"+canonical(f.get("profile.startup_schema.setting"))+"}")
		if _, err := f.compile(); err == nil {
			t.Fatal("accepted startup key injection")
		}
	}
}

func TestStartupOrderingAndOverrides(t *testing.T) {
	f := fresh()
	f.startup()
	f.put("profile.startup_defaults", `{"z":7,"a":null}`)
	item := canonical(f.get("profile.startup_schema.setting"))
	f.put("profile.startup_schema", `{"z":`+item+`,"a":`+item+`,"b":`+item+`}`)
	f.put("machine.axes.0.startup", `{"b":6,"a":8,"z":9}`)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(compiled.AxisProfiles, "startup_sdos=z|u16|1|0|9;a|u16|1|0|8;b|u16|1|0|6\n") {
		t.Fatal("startup defaults/overrides lost insertion order")
	}
}

func TestAbsoluteFeedbackRules(t *testing.T) {
	for _, tc := range []struct{ path, value string }{
		{"profile.absolute_feedback.0.index", "0"}, {"profile.absolute_feedback.0.index", "65536"},
		{"profile.absolute_feedback.0.subindex", "-1"}, {"profile.absolute_feedback.0.subindex", "256"},
		{"profile.absolute_feedback.0.type", `"f32"`}, {"profile.absolute_feedback.0.key", `"x|y"`},
		{"profile.absolute_feedback.0.key", `"x\naxis=1"`}, {"profile.absolute_pair_field", `"unknown"`},
	} {
		t.Run(tc.path+tc.value, func(t *testing.T) {
			f := fresh()
			f.put("profile.absolute_feedback", `[{"key":"truth","index":1,"subindex":0,"type":"i32"}]`)
			f.put(tc.path, tc.value)
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted invalid feedback descriptor")
			}
		})
	}
	for _, count := range []int{2, 17} {
		t.Run(strconv.Itoa(count), func(t *testing.T) {
			f := fresh()
			var entries []any
			for i := 0; i < count; i++ {
				key := fmt.Sprintf("field%d", i)
				if count == 2 {
					key = "duplicate"
				}
				entries = append(entries, decode([]byte(fmt.Sprintf(`{"key":%q,"index":1,"subindex":0,"type":"i32"}`, key))))
			}
			f.put("profile.absolute_feedback", canonical(entries))
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted duplicate or excess feedback")
			}
		})
	}
	f := fresh()
	f.home("[]")
	f.put("profile.native_home.truth_source.type", `"u64"`)
	if _, err := f.compile(); err == nil {
		t.Fatal("accepted unsupported truth scalar")
	}
}
