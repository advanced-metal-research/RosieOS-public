package control

import (
	"encoding/json"
	"os"
	"strings"
	"testing"

	"rosieos/rt-core/adapters/rosie/control/anchors"
)

// TestReviewRound3AnchorIOPublicContract requires exactly one public registration
// for the storage refusal, preventing an adapter-only reason from escaping clients.
func TestReviewRound3AnchorIOPublicContract(t *testing.T) {
	raw, err := os.ReadFile("../../../protocol/application-v1.schema.json")
	if err != nil {
		t.Fatal(err)
	}
	var schema struct{ Reasons []struct{ Name string } }
	if err := json.Unmarshal(raw, &schema); err != nil {
		t.Fatal(err)
	}
	count := 0
	for _, reason := range schema.Reasons {
		if reason.Name == anchors.ErrIO.Error() {
			count++
		}
	}
	if count != 1 {
		t.Fatalf("anchor_store_io public schema count=%d, want 1", count)
	}
	if ReasonAnchorStoreIo != anchors.ErrIO.Error() {
		t.Fatalf("generated anchor reason = %q, want %q", ReasonAnchorStoreIo, anchors.ErrIO.Error())
	}
}

// TestAnchorStoreIOOperatorAction keeps the recovery action in the canonical
// reason and every published copy, preventing an explanation without next steps.
func TestAnchorStoreIOOperatorAction(t *testing.T) {
	raw, err := os.ReadFile("../../../protocol/application-v1.schema.json")
	if err != nil {
		t.Fatal(err)
	}
	var schema struct {
		Reasons []struct{ Name, Meaning string }
	}
	if err := json.Unmarshal(raw, &schema); err != nil {
		t.Fatal(err)
	}
	meaning := ""
	for _, reason := range schema.Reasons {
		if reason.Name == "anchor_store_io" {
			meaning = reason.Meaning
		}
	}
	for _, required := range []string{
		"Retry after checking the anchor directory and disk.",
		"On restore refusal, the anchor store is not modified.",
	} {
		if !strings.Contains(meaning, required) {
			t.Errorf("anchor_store_io meaning lacks %q: %s", required, meaning)
		}
	}
	for _, path := range []string{
		"adapters/rosie/control/api_generated.go",
		"clients/cpp/include/rosie/rt_control_api_generated.hpp",
		"clients/ts/rt_control_api_generated.ts",
		"docs/APPLICATION_API.md",
	} {
		body, err := os.ReadFile("../../../" + path)
		if err != nil {
			t.Fatal(err)
		}
		if meaning == "" || !strings.Contains(string(body), meaning) {
			t.Errorf("%s omits canonical anchor_store_io meaning", path)
		}
	}
}
