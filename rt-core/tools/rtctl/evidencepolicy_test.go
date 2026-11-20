package rtctl

import (
	"strings"
	"testing"
)

func TestA6LinearEvidenceProfile(t *testing.T) {
	f := fresh()
	f.put("profile", string(readTestFile(t, "config/drives/a6ec.json")))
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	for _, want := range []string{
		"coordinate_anchor_source=8256|33|35|a6ec_raw_multi_turn|raw_multi_turn_encoder|3\n",
		"coordinate_evidence=5000000|3000000000|16|16|a6ec_sdo|1500000000\n",
		"a6ec_encoder_position_tracking_mode|u16|8192|8|1|power_cycle",
		"cia402_position_factor_numerator|u32|24721|1|1",
		"cia402_position_factor_denominator|u32|24721|2|1",
		"op|write_sdo|24700|0|i32|0",
	} {
		if !strings.Contains(compiled.AxisProfiles, want) {
			t.Fatalf("missing %q in compiled profile", want)
		}
	}
	if !strings.Contains(compiled.CoordinateIdentity[0], "anchor.source=raw_multi_turn_encoder|3\n") ||
		!strings.Contains(compiled.CoordinateIdentity[0], "tracking.mode=1\n") {
		t.Fatalf("linear coordinate identity:\n%s", compiled.CoordinateIdentity[0])
	}
}

func TestRetiredCoordinateProfileFieldsRefused(t *testing.T) {
	for _, field := range []string{"feedback_counts_wrap", "command_counts_wrap", "position_semantics"} {
		t.Run(field, func(t *testing.T) {
			f := fresh()
			switch field {
			case "position_semantics":
				f.put("profile."+field, `{"drive_native_ratio_enabled":true}`)
			default:
				f.put("profile."+field, "false")
			}
			if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "retired field "+field) {
				t.Fatalf("accepted retired %s: %v", field, err)
			}
		})
	}
}

func TestRetiredA6RotationStartupKeysRefused(t *testing.T) {
	for _, field := range []string{
		"a6ec_rotation_mode_gear_ratio_numerator",
		"a6ec_rotation_mode_gear_ratio_denominator",
		"a6ec_rotation_mode_reference_running_direction",
	} {
		t.Run(field, func(t *testing.T) {
			f := fresh()
			f.put("profile", string(readTestFile(t, "config/drives/a6ec.json")))
			f.put("profile.startup_defaults."+field, "1")
			if _, err := f.compile(); err == nil ||
				!strings.Contains(err.Error(), "retired startup key "+field) {
				t.Fatalf("accepted retired startup key %s: %v", field, err)
			}
		})
	}
}

func TestContinuousCountSpanRefused(t *testing.T) {
	f := fresh()
	f.put("machine.axes.0.limits.max", "1000000000")
	if _, err := f.compile(); err == nil ||
		!strings.Contains(err.Error(), "signed 32-bit continuous count span") {
		t.Fatalf("accepted unrepresentable continuous span: %v", err)
	}
}
