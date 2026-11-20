//go:build linux

package control_test

import (
	"go/ast"
	"go/parser"
	"go/token"
	"os"
	"reflect"
	"regexp"
	"strconv"
	"strings"
	"testing"
)

// profileCatalogue reads generated constants without importing adapter code into
// the consumer example, preventing a second hand-maintained reason vocabulary.
func profileCatalogue(t *testing.T) map[string]string {
	t.Helper()
	file, err := parser.ParseFile(token.NewFileSet(), "../../adapters/rosie/control/api_generated.go", nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	values := map[string]string{}
	ast.Inspect(file, func(node ast.Node) bool {
		value, ok := node.(*ast.ValueSpec)
		if !ok {
			return true
		}
		for i, name := range value.Names {
			if i >= len(value.Values) {
				continue
			}
			literal, ok := value.Values[i].(*ast.BasicLit)
			if !ok || literal.Kind != token.STRING {
				continue
			}
			label, err := strconv.Unquote(literal.Value)
			if err != nil {
				t.Fatal(err)
			}
			values[name.Name] = label
		}
		return true
	})
	return values
}

// profileSection isolates exactly the added section; other API tables cannot
// accidentally satisfy the profile's operation or reason coverage.
func profileSection(t *testing.T) string {
	t.Helper()
	raw, err := os.ReadFile("../../docs/APPLICATION_API.md")
	if err != nil {
		t.Fatal(err)
	}
	_, section, ok := strings.Cut(string(raw), "\n## Integration profile\n")
	if !ok {
		t.Fatal("missing Integration profile section")
	}
	section, _, _ = strings.Cut(section, "\n## ")
	return section
}

func profileDocumentOperations(t *testing.T) map[string]bool {
	t.Helper()
	operations := map[string]bool{}
	inTable := false
	for _, line := range strings.Split(profileSection(t), "\n") {
		if strings.HasPrefix(line, "| Operation |") {
			inTable = true
			continue
		}
		if !strings.HasPrefix(line, "|") {
			inTable = false
			continue
		}
		if !inTable || strings.HasPrefix(line, "|---") {
			continue
		}
		column := strings.TrimSpace(strings.Split(line, "|")[1])
		if len(column) < 3 || column[0] != '`' || column[len(column)-1] != '`' {
			t.Fatalf("operation column must contain one operation: %s", line)
		}
		operation := strings.Trim(column, "`")
		if operations[operation] {
			t.Fatalf("duplicate profile operation: %s", operation)
		}
		operations[operation] = true
	}
	if len(operations) == 0 {
		t.Fatal("empty profile operation table")
	}
	return operations
}

// TestIntegrationProfileDocument inspects actual public SDK calls and literal
// Request operations, preventing a manually maintained list from hiding drift.
func TestIntegrationProfileDocument(t *testing.T) {
	file, err := parser.ParseFile(token.NewFileSet(), "example_motion_server_test.go", nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	methods := map[string]string{
		"Describe": "describe", "Status": "status", "Acquire": "acquire", "Renew": "renew",
		"StartRenewal": "renew", "Enable": "enable", "Arm": "arm", "Home": "home",
		"PrepareProgram": "prepare_program", "StartProgram": "start_program",
		"TelemetryBatches": "telemetry", "TelemetryStream": "telemetry",
		"EventsContext": "subscribe_events", "StreamEvents": "subscribe_events", "Stop": "stop", "Release": "release",
	}
	local := map[string]bool{"Command": true, "StopRenewal": true, "Close": true, "Fence": true, "Grant": true, "SetFence": true}
	invoked := map[string]bool{}
	ast.Inspect(file, func(node ast.Node) bool {
		if field, ok := node.(*ast.KeyValueExpr); ok {
			if key, ok := field.Key.(*ast.Ident); ok && key.Name == "Operation" {
				literal, ok := field.Value.(*ast.BasicLit)
				if !ok || literal.Kind != token.STRING {
					t.Fatal("profile Request operation must be a literal for drift inspection")
				}
				operation, err := strconv.Unquote(literal.Value)
				if err != nil {
					t.Fatal(err)
				}
				invoked[operation] = true
			}
		}
		call, ok := node.(*ast.CallExpr)
		if !ok {
			return true
		}
		selector, ok := call.Fun.(*ast.SelectorExpr)
		if !ok {
			return true
		}
		receiver, ok := selector.X.(*ast.Ident)
		if !ok || receiver.Name != "c" {
			return true
		}
		if operation, ok := methods[selector.Sel.Name]; ok {
			invoked[operation] = true
		} else if !local[selector.Sel.Name] {
			t.Fatalf("unmapped SDK method: %s", selector.Sel.Name)
		}
		return true
	})
	if documented := profileDocumentOperations(t); !reflect.DeepEqual(documented, invoked) {
		t.Fatalf("profile operations %v; SDK invokes %v", documented, invoked)
	}
	reasons := map[string]bool{}
	for symbol, label := range profileCatalogue(t) {
		if strings.HasPrefix(symbol, "Reason") {
			reasons[label] = true
		}
	}
	mentions := regexp.MustCompile(`\[reason:([^\]]+)\]`).FindAllStringSubmatch(profileSection(t), -1)
	if len(mentions) == 0 {
		t.Fatal("profile has no catalogue reason references")
	}
	for _, mention := range mentions {
		if !reasons[mention[1]] {
			t.Errorf("reason absent from generated catalogue: %s", mention[1])
		}
	}
}
