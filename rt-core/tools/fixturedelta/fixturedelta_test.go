package fixturedelta

import (
	"encoding/json"
	"os"
	"reflect"
	"testing"
)

// Every template rule has one case here: default scalar, explicit scalar,
// null-default object, all-defaults object, optional key, default array,
// explicit array with element deltas, and a delta key the template lacks.
func TestExpandRules(t *testing.T) {
	compact := `{
	  "schema": "rosie.fixture-delta.v1",
	  "template": {
	    "keys": ["name", "count", "ratio", "rejection", "axes", "stale", "tags"],
	    "object": {
	      "name": {"default": ""},
	      "count": {"default": 0},
	      "ratio": {"default": 0.5},
	      "rejection": {"keys": ["code", "mask"], "object": {"code": {"default": ""}, "mask": {"default": 1}}, "nullDefault": true},
	      "axes": {"array": {"keys": ["id", "pos"], "object": {"id": {"default": "J1"}, "pos": {"default": 0}}}, "default": [{"id": "J1", "pos": 0}]},
	      "stale": {"default": false, "optional": true},
	      "tags": {"array": {"default": ""}, "default": []}
	    }
	  },
	  "cases": [
	    {"name": "defaults"},
	    {"name": "explicit", "count": 7, "ratio": 1e-07, "rejection": {"code": "not_ready"}, "axes": [{}, {"pos": 3}], "stale": true, "tags": ["a", "b"]},
	    {"name": "empty rejection", "rejection": {}, "axes": []}
	  ]
	}`
	expanded, err := Expand([]byte(compact))
	if err != nil {
		t.Fatal(err)
	}
	var got []map[string]any
	if err := json.Unmarshal(expanded, &got); err != nil {
		t.Fatalf("%v\n%s", err, expanded)
	}
	want := []map[string]any{
		{"name": "defaults", "count": 0.0, "ratio": 0.5, "rejection": nil, "axes": []any{map[string]any{"id": "J1", "pos": 0.0}}, "tags": []any{}},
		{"name": "explicit", "count": 7.0, "ratio": 1e-07, "rejection": map[string]any{"code": "not_ready", "mask": 1.0},
			"axes": []any{map[string]any{"id": "J1", "pos": 0.0}, map[string]any{"id": "J1", "pos": 3.0}}, "stale": true, "tags": []any{"a", "b"}},
		{"name": "empty rejection", "count": 0.0, "ratio": 0.5, "rejection": map[string]any{"code": "", "mask": 1.0}, "axes": []any{}, "tags": []any{}},
	}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("got  %v\nwant %v\n%s", got, want, expanded)
	}
	if _, err := Expand([]byte(`{"schema":"rosie.fixture-delta.v1","template":{"keys":["a"],"object":{"a":{"default":0}}},"cases":[{"b":1}]}`)); err == nil {
		t.Fatal("a delta key outside the template must be refused")
	}
	if _, err := Expand([]byte(`{"schema":"other","template":{"keys":[],"object":{}},"cases":[]}`)); err == nil {
		t.Fatal("another schema must be refused")
	}
}

// One-off equivalence check used before an original fixture is deleted:
// FIXTURE_DELTA_ORIGINAL and FIXTURE_DELTA_COMPACT name the two files.
func TestExpandMatchesOriginal(t *testing.T) {
	original, compact := os.Getenv("FIXTURE_DELTA_ORIGINAL"), os.Getenv("FIXTURE_DELTA_COMPACT")
	if original == "" || compact == "" {
		t.Skip("set FIXTURE_DELTA_ORIGINAL and FIXTURE_DELTA_COMPACT to compare")
	}
	expanded, err := ExpandFile(compact)
	if err != nil {
		t.Fatal(err)
	}
	raw, err := os.ReadFile(original)
	if err != nil {
		t.Fatal(err)
	}
	var got, want []any
	if err := json.Unmarshal(expanded, &got); err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(raw, &want); err != nil {
		t.Fatal(err)
	}
	if len(got) != len(want) {
		t.Fatalf("%d cases, want %d", len(got), len(want))
	}
	for i := range want {
		if !reflect.DeepEqual(got[i], want[i]) {
			g, _ := json.Marshal(got[i])
			w, _ := json.Marshal(want[i])
			t.Fatalf("case %d differs\ngot  %s\nwant %s", i, g, w)
		}
	}
	t.Logf("%d cases identical after expansion; expanded %d bytes from %d", len(want), len(expanded), len(compactSize(compact)))
}

func compactSize(path string) []byte {
	raw, _ := os.ReadFile(path)
	return raw
}
