package main

import (
	"bytes"
	"os"
	"strings"
	"testing"
)

// Compare every emitted field against the committed source; pinned counts and
// the measured ALF1.0 pair prevent an empty or incomplete generator passing.
func TestManualTableMatchesJSON(t *testing.T) {
	body, err := os.ReadFile("../../docs/drives/a6ec/a6ec_manual_codes.json")
	if err != nil {
		t.Fatal(err)
	}
	entries, err := rows(body)
	if err != nil {
		t.Fatal(err)
	}
	if len(entries) != 109 {
		t.Fatalf("rows=%d, want 109", len(entries))
	}
	want, err := generate(body)
	if err != nil {
		t.Fatal(err)
	}
	got, err := os.ReadFile("../../include/a6ec_alarm_codes_generated.hpp")
	if err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(got, want) {
		t.Fatal("manual/generated table mismatch; run go run ./tools/a6ecgen")
	}
	if !strings.Contains(string(got), `{"ALF1.0", "Re-power-on required for parameter settings to take effect", 0x6320, 0x0f10, true, "", true}`) {
		t.Fatal("missing measured 0x6320/0x0f10 pair")
	}
}

func TestManualTableRejectsMalformedCode(t *testing.T) {
	_, err := generate([]byte(`{"tables":{"row":{"code":"ALF1.0","name":"bad","bus_fault_code_603f":"0x10000","alarm_code_203f":"0xf10"}}}`))
	if err == nil {
		t.Fatal("accepted overflowing bus code")
	}
}
