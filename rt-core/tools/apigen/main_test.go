package main

import (
	"bytes"
	"encoding/json"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

func schemaBytes(t *testing.T) []byte {
	t.Helper()
	body, err := os.ReadFile("../../protocol/application-v1.schema.json")
	if err != nil {
		t.Fatal(err)
	}
	return body
}

func TestGoldenGeneratedOutputs(t *testing.T) {
	outputs, err := generate(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	if len(outputs) != 3 {
		t.Fatalf("generated %d artifacts, want 3", len(outputs))
	}
	if err := writeOutputs("../..", outputs, true); err != nil {
		t.Fatal(err)
	}
	second, err := generate(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	for path, body := range outputs {
		if !bytes.Equal(body, second[path]) {
			t.Fatalf("nondeterministic output: %s", path)
		}
	}
}

func TestRetiredCoordinateFieldsAreNotGenerated(t *testing.T) {
	outputs, err := generate(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	for path, body := range outputs {
		for _, field := range []string{"feedback_wrap", "command_wrap", "anchor_period_counts"} {
			if bytes.Contains(body, []byte(field)) {
				t.Errorf("generated %s still exposes %s", path, field)
			}
		}
	}
}

func TestRetiredCoordinateFieldsAreNotPublishedByNative(t *testing.T) {
	sources := map[string][]string{
		"../../include/description.hpp":     {"feedback_wrap", "command_wrap"},
		"../../engine/status_publisher.hpp": {"anchor_period_counts"},
	}
	for path, fields := range sources {
		body, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		for _, field := range fields {
			if bytes.Contains(body, []byte(`"`+field+`"`)) {
				t.Errorf("%s still publishes %s", path, field)
			}
		}
	}
}

// TestRegenerationResolvesAllAPIConflicts runs the actual make target over all
// three conflicted artifacts and a changed schema, preventing branch output or
// a stale Makefile recipe from dropping fields during integration.
func TestRegenerationResolvesAllAPIConflicts(t *testing.T) {
	body := mutateSchema(t, func(s *Schema) {
		s.Reasons = append(s.Reasons, Reason{Name: "integration_regeneration_probe", Meaning: "Unverified integration fixture reason."})
	})
	outputs, err := generate(body)
	if err != nil {
		t.Fatal(err)
	}
	if err := os.MkdirAll("../../build", 0755); err != nil {
		t.Fatal(err)
	}
	root, err := os.MkdirTemp("../../build", "apigen-merge-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = os.RemoveAll(root) })
	paths := []string{
		"adapters/rosie/control/api_generated.go",
		"clients/cpp/include/rosie/rt_control_api_generated.hpp",
		"clients/ts/rt_control_api_generated.ts",
	}
	if len(outputs) != 3 {
		t.Fatalf("generated %d artifacts, want 3", len(outputs))
	}
	for _, path := range paths {
		if err := os.MkdirAll(filepath.Dir(filepath.Join(root, path)), 0755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(filepath.Join(root, path), []byte("<<<<<<< integration\nold\n=======\nbranch\n>>>>>>> drivetelemetry\n"), 0644); err != nil {
			t.Fatal(err)
		}
	}
	if err := writeOutputs(root, outputs, true); err == nil {
		t.Fatal("conflicted API artifacts passed check")
	}
	makefile, err := filepath.Abs("../../Makefile")
	if err != nil {
		t.Fatal(err)
	}
	inputs := map[string]string{
		"go.mod":                        "../../go.mod",
		"tools/apigen/main.go":          "main.go",
		"tools/apigen/typescript.go":    "typescript.go",
		"tools/apigen/documentation.go": "documentation.go",
		"tools/apigen/json.hpp.tmpl":    "json.hpp.tmpl",
		"docs/APPLICATION_API.md":       "../../docs/APPLICATION_API.md",
	}
	for destination, source := range inputs {
		input, err := os.ReadFile(source)
		if err != nil {
			t.Fatal(err)
		}
		path := filepath.Join(root, destination)
		if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, input, 0600); err != nil {
			t.Fatal(err)
		}
	}
	if err := os.MkdirAll(filepath.Join(root, "protocol"), 0755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(root, "protocol/application-v1.schema.json"), body, 0600); err != nil {
		t.Fatal(err)
	}
	command := exec.Command("make", "--no-print-directory", "-f", makefile, "generate-api")
	command.Dir = root
	if output, err := command.CombinedOutput(); err != nil {
		t.Fatalf("make generate-api did not resolve all conflicts: %v\n%s", err, output)
	}
	if err := writeOutputs(root, outputs, true); err != nil {
		t.Fatal(err)
	}
	for _, path := range paths {
		got, err := os.ReadFile(filepath.Join(root, path))
		if err != nil || !bytes.Equal(got, outputs[path]) {
			t.Fatalf("regeneration did not replace %s: %v", path, err)
		}
		fields := []string{"integration_regeneration_probe"}
		if strings.HasSuffix(path, ".go") {
			// Go reuses native aliases; C++ and TypeScript emit their JSON fields.
			fields = append(fields, "ipcclient.LogicalAxisStatus")
		} else {
			fields = append(fields, "velocity_actual_counts_per_s", "following_error_counts", "interpolation")
		}
		for _, field := range fields {
			if !bytes.Contains(got, []byte(field)) {
				t.Errorf("regenerated %s omitted %s", path, field)
			}
		}
	}
}

func TestDriftDetection(t *testing.T) {
	root := t.TempDir()
	outputs := map[string][]byte{"generated.go": []byte("canonical\n")}
	if err := writeOutputs(root, outputs, false); err != nil {
		t.Fatal(err)
	}
	if err := writeOutputs(root, outputs, true); err != nil {
		t.Fatal(err)
	}
	path := filepath.Join(root, "generated.go")
	if err := os.WriteFile(path, []byte("changed\n"), 0600); err != nil {
		t.Fatal(err)
	}
	if err := writeOutputs(root, outputs, true); err == nil || !strings.Contains(err.Error(), "drift") {
		t.Fatalf("drift result: %v", err)
	}
	got, err := os.ReadFile(path)
	if err != nil || string(got) != "changed\n" {
		t.Fatalf("check rewrote artifact: %q %v", got, err)
	}
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	if err := writeOutputs(root, outputs, true); err == nil {
		t.Fatal("missing artifact accepted")
	}
}

func mutateSchema(t *testing.T, mutate func(*Schema)) []byte {
	t.Helper()
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	mutate(&s)
	body, err := json.MarshalIndent(s, "", "  ")
	if err != nil {
		t.Fatal(err)
	}
	return append(body, '\n')
}

func TestDuplicateReasonRejection(t *testing.T) {
	body := mutateSchema(t, func(s *Schema) { s.Reasons = append(s.Reasons, s.Reasons[0]) })
	if _, err := generate(body); err == nil || !strings.Contains(err.Error(), "reason") {
		t.Fatalf("duplicate result: %v", err)
	}
}

func TestUnknownTypeRejection(t *testing.T) {
	body := mutateSchema(t, func(s *Schema) { s.Types[0].Fields[0].Type = "[]UnspecifiedPayload" })
	if _, err := generate(body); err == nil || !strings.Contains(err.Error(), "unknown type") {
		t.Fatalf("unknown type result: %v", err)
	}
}

func TestCapabilityStateContract(t *testing.T) {
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	for _, capability := range s.Capabilities {
		if capability.Name == "jog" && (capability.State != "test_only" ||
			!strings.Contains(capability.Description, "begin_jog/update_jog/end_jog")) {
			t.Fatalf("legacy jog contract: %+v", capability)
		}
	}
	body := mutateSchema(t, func(s *Schema) { s.Capabilities[0].State = "unknown" })
	if _, err := generate(body); err == nil || !strings.Contains(err.Error(), "capability state") {
		t.Fatalf("unknown capability state accepted: %v", err)
	}
}

func TestCanonicalBytesAffectDigest(t *testing.T) {
	body := schemaBytes(t)
	first, err := generate(body)
	if err != nil {
		t.Fatal(err)
	}
	second, err := generate(append([]byte(" "), body...))
	if err != nil {
		t.Fatal(err)
	}
	if bytes.Equal(first["adapters/rosie/control/api_generated.go"], second["adapters/rosie/control/api_generated.go"]) {
		t.Fatal("schema-byte change did not change digest")
	}
}

// Scalar map goldens pin both languages' field types and object codec calls,
// preventing support for one scalar from masking omissions in another.
func TestScalarMapGoldenOutputs(t *testing.T) {
	body := mutateSchema(t, func(s *Schema) {
		s.Types = append(s.Types, Type{Name: "ScalarMaps", Fields: []Field{
			{Name: "Flags", JSON: "flags", Type: "map[string]bool", JSONType: "object", Required: true, Constraints: "Boolean values keyed by strings."},
			{Name: "Labels", JSON: "labels", Type: "map[string]string", JSONType: "object", OmitEmpty: true, Constraints: "String values keyed by strings."},
			{Name: "Counts", JSON: "counts", Type: "map[string]uint64", JSONType: "object", Required: true, Constraints: "Exact uint64 values keyed by strings."},
		}})
	})
	outputs, err := generate(body)
	if err != nil {
		t.Fatal(err)
	}
	for _, fixture := range []struct{ output, start, end, golden string }{
		{"adapters/rosie/control/api_generated.go", "type ScalarMaps struct {", "\n// Capability state labels", "scalar_maps.go.golden"},
		{"clients/cpp/include/rosie/rt_control_api_generated.hpp", "struct ScalarMaps {", "// encode/decode", "scalar_maps.hpp.golden"},
	} {
		text := string(outputs[fixture.output])
		start := strings.Index(text, fixture.start)
		if start < 0 {
			t.Fatalf("missing map declaration in %s", fixture.output)
		}
		end := strings.Index(text[start:], fixture.end)
		if end < 0 {
			t.Fatalf("missing map declaration boundary in %s", fixture.output)
		}
		want, err := os.ReadFile(filepath.Join("testdata", fixture.golden))
		if err != nil {
			t.Fatal(err)
		}
		if got := text[start : start+end]; got != string(want) {
			t.Errorf("%s drift\ngot:\n%s\nwant:\n%s", fixture.golden, got, want)
		}
	}
}

func TestUnsupportedMapsFailClosed(t *testing.T) {
	for _, typ := range []string{"map[uint64]bool", "map[string][]bool", "map[string]map[string]bool", "map[string]Program", "map[string]json", "map[string]rawjson", "map[string]UnspecifiedPayload"} {
		t.Run(typ, func(t *testing.T) {
			body := mutateSchema(t, func(s *Schema) { s.Types[0].Fields[0].Type = typ })
			if _, err := generate(body); err == nil || !strings.Contains(err.Error(), "unknown type") {
				t.Fatalf("unsupported map result: %v", err)
			}
		})
	}
}
