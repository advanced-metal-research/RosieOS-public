package main

import (
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

// Execute the shipped declarations through Node's type stripper and compare
// capability/reason values to the schema, preventing a stale third client.
func TestTypeScriptApplicationContract(t *testing.T) {
	// Unverified conformance runtime location, shared with protocol golden tests.
	node := nodeBinary(t)
	if _, err := os.Stat(node); err != nil {
		t.Fatalf("required TypeScript runtime %s absent: %v", node, err)
	}
	path, err := filepath.Abs("../../clients/ts/rt_control_api_generated.ts")
	if err != nil {
		t.Fatal(err)
	}
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	generated, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(generated), "interface Request extends Partial<Fence>") || !strings.Contains(string(generated), "interface Description extends NativeDescription") {
		t.Fatal("embedded field optionality changed")
	}
	script := fmt.Sprintf(`import * as api from %q; console.log(JSON.stringify({capabilities:api.Capabilities,reasons:Object.fromEntries(Object.entries(api).filter(([k])=>k.startsWith('Reason')))}));`, "file://"+path)
	raw, err := exec.Command(node, "--experimental-strip-types", "--input-type=module", "-e", script).Output()
	if err != nil {
		t.Fatalf("TypeScript contract: %v", err)
	}
	var got struct {
		Capabilities []struct{ Name, State, Transport string }
		Reasons      map[string]string
	}
	if err = json.Unmarshal(raw, &got); err != nil {
		t.Fatal(err)
	}
	if len(got.Capabilities) != len(s.Capabilities) {
		t.Fatal("missing TypeScript capabilities")
	}
	for i, c := range s.Capabilities {
		v := got.Capabilities[i]
		if v.Name != c.Name || v.State != c.State || v.Transport != c.Transport {
			t.Fatalf("capability %d differs", i)
		}
	}
	reasons := map[string]string{}
	for _, r := range s.Reasons {
		reasons["Reason"+identifier(r.Name)] = r.Name
	}
	if !reflect.DeepEqual(got.Reasons, reasons) {
		t.Fatal("TypeScript reason catalogue differs")
	}
}

// nodeBinary resolves the pinned Node runtime under the current user's rosie
// toolchain (or NODE when set) and fails, never skips, when it is absent: the
// TypeScript golden comparison is mandatory coverage.
func nodeBinary(t *testing.T) string {
	t.Helper()
	if node := os.Getenv("NODE"); node != "" {
		return node
	}
	home, err := os.UserHomeDir()
	if err != nil {
		t.Fatalf("node runtime: home directory unknown: %v", err)
	}
	node := filepath.Join(home, ".rosie", "node-22.13.1", "bin", "node")
	if _, err := os.Stat(node); err != nil {
		t.Fatalf("node runtime %s is required for the TypeScript golden comparison: %v", node, err)
	}
	return node
}
