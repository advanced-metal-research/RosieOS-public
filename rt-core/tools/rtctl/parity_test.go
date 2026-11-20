package rtctl

import (
	"bytes"
	"encoding/json"
	"io/fs"
	"os"
	"path/filepath"
	"sort"
	"testing"
)

// frozen returns captured oracle bytes only for the identical input, preventing
// new cases from silently acquiring expectations from the compiler under test.
func frozen(t *testing.T, name string, input []byte) []byte {
	t.Helper()
	raw, err := os.ReadFile(filepath.Join(testRoot(t), "tools/rtctl/testdata/parity", name))
	if err != nil {
		t.Fatal(err)
	}
	var fixture struct {
		Input  []byte
		Output []byte
	}
	if err := json.Unmarshal(raw, &fixture); err != nil {
		t.Fatal(err)
	}
	if !bytes.Equal(input, fixture.Input) {
		t.Fatal("oracle input changed; the captured expectation does not cover this case")
	}
	return fixture.Output
}

// oracleMachine removes only the explicit synthetic declarations added after
// capture; equality checks prevent changed count semantics from using old bytes.
func oracleMachine(t *testing.T, value any) object {
	t.Helper()
	machine := append(object(nil), asObject(value)...)
	axes := append([]any(nil), asArray(machine.get("axes"))...)
	machine.set("axes", axes)
	for i, value := range axes {
		axis := append(object(nil), asObject(value)...)
		if axis.get("coordinate_evidence_mode") != "multi_turn" || canonical(axis.get("motor_encoder_counts_per_rev")) != canonical(axis.get("wire_counts_per_rev")) {
			t.Fatal("synthetic declaration differs from captured count semantics")
		}
		for _, key := range []string{"motor_encoder_counts_per_rev", "motor_encoder_counts_per_rev_unverified", "coordinate_evidence_mode"} {
			axis = asObject(replace(axis, []string{key}, nil, true))
		}
		axes[i] = axis
	}
	return machine
}

// declareOracleCoordinates supplies the captured count semantics in memory so
// later validation remains covered without changing frozen fixture bytes.
func declareOracleCoordinates(t *testing.T, data []byte) []byte {
	t.Helper()
	machine := asObject(decode(data))
	axes := asArray(machine.get("axes"))
	for i, value := range axes {
		axis := asObject(value)
		axis.set("motor_encoder_counts_per_rev", axis.get("wire_counts_per_rev"))
		axis.set("motor_encoder_counts_per_rev_unverified", "unverified synthetic encoder matching the declared wire revolution")
		axis.set("coordinate_evidence_mode", "multi_turn")
		axes[i] = axis
	}
	if canonical(oracleMachine(t, machine)) != canonical(decode(data)) {
		t.Fatal("coordinate declarations changed historical machine inputs")
	}
	return []byte(canonical(machine))
}

func readTree(t *testing.T, root string) map[string]string {
	t.Helper()
	tree := map[string]string{}
	err := filepath.WalkDir(root, func(path string, entry fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		rel, err := filepath.Rel(root, path)
		if err != nil {
			return err
		}
		if entry.IsDir() {
			tree[rel+"/"] = ""
			return nil
		}
		data, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		tree[rel] = string(data)
		return nil
	})
	if err != nil {
		t.Fatal(err)
	}
	return tree
}

// TestParity compares every published byte with the final frozen compiler
// outputs. Only the publication directory is substituted; identity text, hashes,
// formatting, file membership and argument ordering remain pinned independently.
func sortedKeys(tree map[string]string) []string {
	keys := make([]string, 0, len(tree))
	for key := range tree {
		keys = append(keys, key)
	}
	sort.Strings(keys)
	return keys
}
