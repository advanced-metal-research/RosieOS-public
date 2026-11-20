//go:build linux

package ipcclient

import (
	"encoding/json"
	"strings"
	"testing"
)

// Native's recovery publication carries the drive-alarm sentence; decoding it
// keeps every public field, and marshalling never adds it to the public form.
func TestRecoveryStatusNativeMessageStaysOutOfThePublicForm(t *testing.T) {
	native := `{"home_valid_mask":63,"faults":[{"bit":7,"name":"drive_readiness","axis_mask":16,"recovery":"reset_after_condition_clears","outcome":"persists","rehome_axis_mask":0}],` +
		`"reset_sequence":42,"reason":"fault_persists","outcomes":[],"drive_alarm_resets":[{"axis":4,"result":"persists_after_reset"}],` +
		`"message":"J5 drive alarm Er41.1 persists after reset: Motor over-temperature due to locked- rotor"}`
	var recovery RecoveryStatus
	if err := json.Unmarshal([]byte(native), &recovery); err != nil {
		t.Fatal(err)
	}
	if recovery.HomeValidMask != 63 || recovery.ResetSequence != 42 || recovery.Reason != "fault_persists" ||
		len(recovery.Faults) != 1 || recovery.Faults[0].AxisMask != 16 || recovery.Outcomes == nil ||
		recovery.Message != "J5 drive alarm Er41.1 persists after reset: Motor over-temperature due to locked- rotor" {
		t.Fatalf("decoded %+v", recovery)
	}
	public, err := json.Marshal(recovery)
	if err != nil {
		t.Fatal(err)
	}
	if strings.Contains(string(public), "message") || strings.Contains(string(public), "drive_alarm_resets") {
		t.Fatalf("public form gained native-only fields: %s", public)
	}
	var again RecoveryStatus
	if err := json.Unmarshal(public, &again); err != nil || again.Message != "" || again.ResetSequence != 42 {
		t.Fatalf("public round trip %+v %v", again, err)
	}
	if err := json.Unmarshal([]byte(`{"reset_sequence":"x"}`), &again); err == nil {
		t.Fatal("malformed recovery accepted")
	}
}

func TestRecoveryStatusCurrentHomePreservationCandidate(t *testing.T) {
	for _, field := range []string{"", `,"reset_preserves_home":false`, `,"reset_preserves_home":true`} {
		native := `{"home_valid_mask":63,"faults":[{"bit":6,"name":"coordinate_reference","axis_mask":4,"recovery":"rehome_required","outcome":"persists","rehome_axis_mask":0` + field + `}],` +
			`"reset_sequence":42,"reason":"","outcomes":[{"bit":6,"name":"coordinate_reference","axis_mask":4,"recovery":"rehome_required","outcome":"cleared","rehome_axis_mask":0}]}`
		var recovery RecoveryStatus
		if err := json.Unmarshal([]byte(native), &recovery); err != nil {
			t.Fatal(err)
		}
		want := strings.Contains(field, "true")
		if recovery.Faults[0].ResetPreservesHome != want || recovery.Outcomes[0].ResetPreservesHome {
			t.Fatalf("candidate or historical outcome: %+v", recovery)
		}
		public, err := json.Marshal(recovery)
		if err != nil {
			t.Fatal(err)
		}
		var again RecoveryStatus
		if err := json.Unmarshal(public, &again); err != nil || again.Faults[0].ResetPreservesHome != want || again.Outcomes[0].ResetPreservesHome {
			t.Fatalf("public round trip %+v %v", again, err)
		}
	}
}
