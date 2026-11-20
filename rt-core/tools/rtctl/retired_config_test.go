package rtctl

import (
	"bytes"
	"encoding/json"
	"path/filepath"
	"testing"
)

// readInheritedRobotTestFile adapts retired installation inputs to today's robot
// inheritance contract without changing the archived evidence bytes.
func readInheritedRobotTestFile(t *testing.T, path string) []byte {
	t.Helper()
	data := readTestFile(t, path)
	if filepath.ToSlash(path) != "tools/rtctl/testdata/retired/bench-ninemotor-a6ec.json" {
		return data
	}
	machine := stripRobotTestTravel(asObject(decode(data)))
	var formatted bytes.Buffer
	if err := json.Indent(&formatted, []byte(canonical(machine)), "", "  "); err != nil {
		t.Fatal(err)
	}
	return formatted.Bytes()
}
