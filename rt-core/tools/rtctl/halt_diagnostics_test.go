package rtctl

import (
	"strings"
	"testing"
)

// Unverified test assignment: bit 16 of the 32-bit digital-input PDO, active high.
const externalEnableJSON = `{"bit_index":16,"bit_index_source":"unverified test wiring","polarity":"active_high","polarity_source":"unverified test polarity"}`

// The reviewer's S-ON startup descriptor must not silently create a diagnostic
// mapping without a dedicated assignment and its two source annotations.
func TestReviewHaltMappingSurface(t *testing.T) {
	f := fresh()
	f.put("profile", string(readTestFile(t, "config/drives/a6ec.json")))
	f.put("machine.axes.0.wire_counts_per_rev", "131072")
	f.put("machine.axes.0.motor_encoder_counts_per_rev", "131072")
	f.put("machine.axes.0.startup", `{"review_s_on":1}`)
	f.put("profile.startup_schema.review_s_on", `{"type":"u16","min":0,"max":32,"object":{"index":8196,"subindex":1}}`)
	if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "external_enable_input") {
		t.Fatalf("S-ON inferred without explicit diagnostic assignment: %v", err)
	}
	f.put("machine.axes.0.diagnostics", `{"external_enable_input":`+externalEnableJSON+`}`)
	if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "requires a mapped digital-input PDO") {
		t.Fatalf("B04 must refuse a cyclic S-ON observation: %v", err)
	}
	// The explicit B02 alternative retains the wired cyclic-input contract.
	f.put("profile", string(readTestFile(t, "config/drives/a6ec_1b02.json")))
	f.put("profile.startup_schema.review_s_on", `{"type":"u16","min":0,"max":32,"object":{"index":8196,"subindex":1}}`)
	result, err := f.compile()
	if err != nil || !strings.Contains(result.AxisProfiles, "external_enable_input=16|active_high\n") {
		t.Fatalf("explicit S-ON observation missing: %v\n%s", err, result.AxisProfiles)
	}
}

// Assignments are per axis and optional. Source annotations are mandatory;
// absent PDOs, malformed polarity and out-of-width bits cannot claim validity.
func TestExternalEnableInputCompiler(t *testing.T) {
	base := fresh()
	tx := asArray(base.get("profile.ethercat.tx_layout"))
	base.put("profile.ethercat.tx_layout", canonical(append(tx, decode([]byte(`{"semantic":"di","index":24829,"subindex":0,"bits":32}`)))))
	unset, err := base.compile()
	if err != nil || strings.Contains(unset.AxisProfiles, "external_enable_input=") {
		t.Fatalf("default must remain unassigned: %v", err)
	}
	for _, tc := range []struct{ bit, polarity string }{{"0", "active_low"}, {"16", "active_high"}, {"31", "active_low"}} {
		f := &fixture{asObject(decode([]byte(canonical(base.data))))}
		f.put("machine.axes.0.diagnostics", `{"external_enable_input":`+externalEnableJSON+`}`)
		f.put("machine.axes.0.diagnostics.external_enable_input.bit_index", tc.bit)
		f.put("machine.axes.0.diagnostics.external_enable_input.polarity", canonical(tc.polarity))
		result, err := f.compile()
		want := "external_enable_input=" + tc.bit + "|" + tc.polarity + "\n"
		if err != nil || !strings.Contains(result.AxisProfiles, want) || !strings.Contains(result.MachineIdentity, "axis.0."+want) {
			t.Fatalf("bit %s polarity %s: %v\n%s", tc.bit, tc.polarity, err, result.AxisProfiles)
		}
	}
	for _, tc := range []struct{ field, value string }{
		{"bit_index", "-1"}, {"bit_index", "32"}, {"bit_index", "4294967295"}, {"bit_index", "1.5"},
		{"polarity", `"unknown"`}, {"polarity", "null"}, {"polarity", "1"},
		{"bit_index_source", ""}, {"polarity_source", ""},
		{"bit_index_source", `"  "`}, {"polarity_source", "null"},
	} {
		t.Run(tc.field+"/"+tc.value, func(t *testing.T) {
			f := &fixture{asObject(decode([]byte(canonical(base.data))))}
			f.put("machine.axes.0.diagnostics", `{"external_enable_input":`+externalEnableJSON+`}`)
			path := "machine.axes.0.diagnostics.external_enable_input." + tc.field
			if tc.value == "" {
				f.remove(path)
			} else {
				f.put(path, tc.value)
			}
			if _, err := f.compile(); err == nil {
				t.Fatal("accepted invalid external-enable observation")
			}
		})
	}
	absent := fresh()
	absent.put("machine.axes.0.diagnostics", `{"external_enable_input":`+externalEnableJSON+`}`)
	if _, err := absent.compile(); err == nil {
		t.Fatal("accepted an external-enable bit without a digital-input PDO")
	}
}
