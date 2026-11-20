package planning

// Regression coverage for the P2.11 row-cap removal (ledger L27): the shared
// Robot V4 program bound is a 65536-row memory-safety backstop (sized for 10 ms
// rows on a slow cell, 2026-09-07; 4096 before that), not the old
// 64-row Steam Deck demo constant. The exact Dog Box needs 730 rows at
// 0.1 mm weld tolerance and must decode.

import (
	"encoding/json"
	"fmt"
	"strings"
	"testing"
)

func boundProbeDocumentWithRows(rowCount int) json.RawMessage {
	var rows strings.Builder
	for index := 0; index < rowCount; index++ {
		if index > 0 {
			rows.WriteString(",")
		}
		fmt.Fprintf(
			&rows,
			`{"row_id":"row-%d","revision":1,"enabled":true,"row_type":"comment"}`,
			index+1,
		)
	}
	document := fmt.Sprintf(`{
		"schema_version": 1,
		"program_id": "probe-program",
		"program_revision": 1,
		"base_program_revision": 0,
		"robot_model": "rosie_1400_v3",
		"robotic-cell-type": "phobos_cell",
		"execution_mode": "simulation",
		"rows": [%s]
	}`, rows.String())
	return json.RawMessage(document)
}

func TestRobotV4ProgramRowBoundIsTheContractBackstop(t *testing.T) {
	for _, accepted := range []int{64, 730, maxRobotV4ProgramRows} {
		if _, err := decodeRobotV4ProgramDocument(boundProbeDocumentWithRows(accepted)); err != nil {
			t.Fatalf("%d rows must decode: %v", accepted, err)
		}
	}
	if _, err := decodeRobotV4ProgramDocument(
		boundProbeDocumentWithRows(maxRobotV4ProgramRows + 1),
	); err == nil {
		t.Fatalf("%d rows must be rejected", maxRobotV4ProgramRows+1)
	}
}

func TestRobotV4ProgramByteBoundStillRefusesOversizedDocuments(t *testing.T) {
	document := boundProbeDocumentWithRows(1)
	padded := append(json.RawMessage(nil), document...)
	padded = append(padded, strings.Repeat(" ", maxRobotV4ProgramBytes)...)
	if _, err := decodeRobotV4ProgramDocument(padded); err == nil {
		t.Fatal("document above the byte bound must be rejected")
	}
}
