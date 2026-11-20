package main

import (
	"bytes"
	"os"
	"path/filepath"
	"testing"
)

// TestSchemaDocumentationDrift checks omissions through the same output checker
// as check-api, preventing new operations, fields or reasons disappearing in docs.
func TestSchemaDocumentationDrift(t *testing.T) {
	body := schemaBytes(t)
	doc, err := os.ReadFile("../../docs/APPLICATION_API.md")
	if err != nil {
		t.Fatal(err)
	}
	want, err := documentation(body, doc)
	if err != nil || !bytes.Equal(want, doc) {
		t.Fatalf("schema documentation drift: %v", err)
	}
	for _, row := range []string{"| `resource` |", "### RobotDescription", "### DriveDescription", "| `resources` |", "| resource_unknown |", "| resource_artifact_invalid |"} {
		t.Run(row, func(t *testing.T) {
			start := bytes.Index(doc, []byte(row))
			if start < 0 {
				t.Fatal("schema entry absent")
			}
			end := start + bytes.IndexByte(doc[start:], '\n') + 1
			mutated := append(append([]byte{}, doc[:start]...), doc[end:]...)
			repaired, err := documentation(body, mutated)
			if err != nil || !bytes.Equal(repaired, want) {
				t.Fatalf("omission not repaired: %v", err)
			}
			root := t.TempDir()
			if err := os.WriteFile(filepath.Join(root, "api.md"), mutated, 0600); err != nil {
				t.Fatal(err)
			}
			if err := writeOutputs(root, map[string][]byte{"api.md": repaired}, true); err == nil {
				t.Fatal("omission passed check-api output check")
			}
		})
	}
}
