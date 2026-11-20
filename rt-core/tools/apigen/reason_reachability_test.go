package main

import (
	"io/fs"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// productionReasonFixture preserves adapter imports and source locations, so a
// mutation tests the real public path rather than an abbreviated call graph.
func productionReasonFixture(t *testing.T) map[string]string {
	t.Helper()
	fixture := map[string]string{}
	err := filepath.WalkDir("../../adapters/rosie/control", func(path string, entry fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if entry.IsDir() || !strings.HasSuffix(path, ".go") || strings.HasSuffix(path, "_test.go") {
			return nil
		}
		body, err := os.ReadFile(path)
		fixture[strings.TrimPrefix(filepath.ToSlash(path), "../../")] = string(body)
		return err
	})
	if err != nil {
		t.Fatal(err)
	}
	body, err := os.ReadFile("../../cmd/rt-control/main.go")
	if err != nil {
		t.Fatal(err)
	}
	fixture["cmd/rt-control/main.go"] = string(body)
	return fixture
}

// The review6 reachability probe must fail check-api at the mutated sentinel;
// a documented log with the old label cannot mask Store.Load's HTTP error.
func TestAdapterReasonInventoryAnchorSentinelMutation(t *testing.T) {
	fixture := productionReasonFixture(t)
	path := "adapters/rosie/control/anchors/store.go"
	before := `errors.New("anchor_store_io")`
	if strings.Count(fixture[path], before) != 1 {
		t.Fatal("anchor sentinel fixture drift")
	}
	fixture[path] = strings.Replace(fixture[path], before, `errors.New("review6_public_anchor_io")`, 1)
	output, err := reasonFixtureGate(t, fixture)
	if err == nil || !strings.Contains(output, "anchors/store.go:") || !strings.Contains(output, `undocumented reason "review6_public_anchor_io"`) {
		t.Fatalf("anchor sentinel escaped check-api: %v\n%s", err, output)
	}
}

// Concrete receivers select their own package's methods, while an unresolved
// interface conservatively includes implementations instead of losing refusals.
func TestAdapterReasonInventoryCrossPackageMethods(t *testing.T) {
	for _, pkg := range []string{"anchors", "jog", "remote"} {
		t.Run(pkg, func(t *testing.T) {
			root := t.TempDir()
			fixture := map[string]string{
				"go.mod": "module inventory.test/control\n\ngo 1.24\n",
				"control.go": `package control
import "net/http"
import target "inventory.test/control/` + pkg + `"
type Controller struct { store *target.Store }
func (c *Controller) reply() { _, err := c.store.Load(); http.Error(nil, err.Error(), 409) }
func event(store target.Store) { _, err := store.Load(); _ = struct { Reason string }{err.Error()} }
func wire() { store := target.New(); _, err := store.Load(); _ = Response{Error: err.Error()} }
`,
				pkg + "/store.go": `package ` + pkg + `
import "errors"
type Store struct {}
func New() *Store { return &Store{} }
func (*Store) Load() (int, error) { return 0, errors.New("public_method") }
`,
				"decoy/store.go": `package decoy
import "errors"
type Store struct {}
func (Store) Load() (int, error) { return 0, errors.New("private_namesake") }
`,
			}
			writeReasonFixture(t, root, fixture)
			reasons, err := adapterReasonLiterals(root)
			if err != nil || len(reasons) != 1 || reasons[0].name != "public_method" || filepath.Base(filepath.Dir(reasons[0].position.Filename)) != pkg {
				t.Fatalf("cross-package method inventory: %+v, %v", reasons, err)
			}
		})
	}
}
