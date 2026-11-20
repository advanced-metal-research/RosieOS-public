package programs

// Regression coverage for the P2.11 row-cap removal (ledger L27): the
// robot.v4.program.v1 rows bound is a 4096-row memory-safety backstop, not
// the old 64-row Steam Deck demo constant. The exact Dog Box needs 730 rows
// at 0.1 mm weld tolerance and must validate.

import (
	"encoding/json"
	"fmt"
	"os"
	"testing"
)

func TestProgramRowBoundIsTheContractBackstop(t *testing.T) {
	raw, err := os.ReadFile("fixtures/complete-program.json")
	if err != nil {
		t.Fatalf("read fixture: %v", err)
	}
	var program map[string]any
	if err := json.Unmarshal(raw, &program); err != nil {
		t.Fatalf("decode fixture: %v", err)
	}
	rows := program["rows"].([]any)
	var template map[string]any
	for _, candidate := range rows {
		row := candidate.(map[string]any)
		if row["row_type"] == "motion" {
			template = row
			break
		}
	}
	expand := func(rowCount int) []any {
		expanded := make([]any, 0, rowCount)
		for index := 0; index < rowCount; index++ {
			row := make(map[string]any, len(template))
			for key, value := range template {
				row[key] = value
			}
			row["row_id"] = fmt.Sprintf("row-%d", index+1)
			row["row_index"] = float64(index)
			row["row_number"] = float64(index + 1)
			row["row_hash"] = fmt.Sprintf("rowhash-%d", index+1)
			expanded = append(expanded, row)
		}
		return expanded
	}
	for _, accepted := range []int{64, 730, 4096} {
		program["rows"] = expand(accepted)
		if err := validateProgram(program); err != nil {
			t.Fatalf("%d rows must validate: %v", accepted, err)
		}
	}
	program["rows"] = expand(4097)
	if err := validateProgram(program); err == nil {
		t.Fatal("4097 rows must be rejected")
	}
}
