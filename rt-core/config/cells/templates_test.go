package cells

import (
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
	"reflect"
	"testing"
)

// TestCellConfigTemplate validates the annotated binding with the production
// reader; every typed binding field must be present to prevent template drift.
func TestCellConfigTemplate(t *testing.T) {
	root, err := filepath.Abs("../../..")
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(filepath.Join(root, "rt-core/config/templates/cell.json"))
	if err != nil {
		t.Fatal(err)
	}
	var value struct {
		Nodes []struct {
			Core map[string]json.RawMessage `json:"rt_core"`
		} `json:"nodes"`
	}
	if err := json.Unmarshal(raw, &value); err != nil {
		t.Fatal(err)
	}
	if len(value.Nodes) != 1 {
		t.Fatal("template must bind exactly one node")
	}
	typ := reflect.TypeOf(Binding{})
	for i := 0; i < typ.NumField(); i++ {
		name := typ.Field(i).Tag.Get("json")
		if len(value.Nodes[0].Core[name]) == 0 || len(value.Nodes[0].Core[name+"_doc"]) == 0 {
			t.Errorf("undocumented binding field %s", name)
		}
	}
	results, err := CheckManifest(root, raw)
	if err != nil && !errors.Is(err, ErrCompilerWarnings) {
		t.Fatal(err)
	}
	if len(results) != 1 || results[0].AxisCount != 2 || results[0].SHA256 != "b01819bab64e8751b2af2ee56a00b52eb88c210218ae7f12e2e9349ca051c852" {
		t.Fatalf("template binding: %+v", results)
	}
}
