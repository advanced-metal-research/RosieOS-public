package rtctl

import (
	"reflect"
	"strings"
	"testing"
)

// TestControlLinkTiming checks absolute ceilings and default parity so a preset
// cannot silently change simulation behavior or bypass native identity fencing.
func TestControlLinkTiming(t *testing.T) {
	base, err := fresh().compile()
	if err != nil {
		t.Fatal(err)
	}
	for _, block := range []string{`{}`, `{"link_profile":"lan"}`, `{"max_grant_lease_ns":500000000,"max_jog_input_age_ns":250000000}`} {
		f := fresh()
		f.put("machine.control", block)
		got, err := f.compile()
		if err != nil {
			t.Fatal(err)
		}
		if !reflect.DeepEqual(base.Argv, got.Argv) || base.MachineSHA256 != got.MachineSHA256 {
			t.Fatal("LAN behavior moved")
		}
		if base.SHA256 == got.SHA256 {
			t.Fatal("explicit control block absent from digest")
		}
	}
	for _, tc := range []struct{ block, lease, age string }{
		{`{"link_profile":"internet"}`, "3000000000", "750000000"},
		{`{"link_profile":"internet","max_grant_lease_ns":10000000000,"max_jog_input_age_ns":2000000000}`, "10000000000", "2000000000"},
		{`{"link_profile":"internet","max_grant_lease_ns":100000000,"max_jog_input_age_ns":120000000}`, "100000000", "120000000"},
	} {
		f := fresh()
		f.put("machine.control", tc.block)
		got, err := f.compile()
		if err != nil {
			t.Fatal(err)
		}
		for _, field := range []string{"control_max_grant_lease_ns=" + tc.lease + "\n", "control_max_jog_input_age_ns=" + tc.age + "\n"} {
			if !strings.Contains(got.MachineIdentity, field) {
				t.Fatalf("missing %s", field)
			}
		}
	}
	for _, block := range []string{`null`, `[]`, `{"link_profile":"wifi"}`, `{"max_grant_lease_ns":10000000001}`, `{"max_grant_lease_ns":1000001}`, `{"max_jog_input_age_ns":2000000001}`, `{"max_jog_input_age_ns":0}`, `{"max_jog_input_age_ns":1.5}`, `{"max_grant_lease_ns":"3000000000"}`, `{"max_grant_lease_ns":null}`, `{"link_profiel":"lan"}`} {
		f := fresh()
		f.put("machine.control", block)
		if _, err := f.compile(); err == nil {
			t.Fatalf("accepted %s", block)
		}
	}
}
