package buildcontract

import (
	"go/ast"
	"go/parser"
	"go/token"
	"io/fs"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// compileCalls finds compiler entry points and subprocess compile arguments,
// including import aliases and locally assembled argv, so new fixtures cannot
// quietly depend on the daemon's simulation affinity fallback.
func compileCalls(name string, source any) ([]token.Position, error) {
	positions := token.NewFileSet()
	file, err := parser.ParseFile(positions, name, source, 0)
	if err != nil {
		return nil, err
	}
	imports := map[string]string{}
	for _, spec := range file.Imports {
		path, _ := strconv.Unquote(spec.Path.Value)
		alias := filepath.Base(path)
		if spec.Name != nil {
			alias = spec.Name.Name
		}
		imports[alias] = path
	}
	var containsCompile func(ast.Node, map[ast.Node]bool) bool
	containsCompile = func(node ast.Node, seen map[ast.Node]bool) bool {
		if node == nil || seen[node] {
			return false
		}
		seen[node] = true
		found := false
		ast.Inspect(node, func(child ast.Node) bool {
			switch value := child.(type) {
			case *ast.BasicLit:
				text, _ := strconv.Unquote(value.Value)
				for _, word := range strings.Fields(text) {
					if word == "compile" {
						found = true
					}
				}
			case *ast.Ident:
				if value.Obj != nil {
					switch declaration := value.Obj.Decl.(type) {
					case *ast.AssignStmt:
						for _, expression := range declaration.Rhs {
							found = found || containsCompile(expression, seen)
						}
					case *ast.ValueSpec:
						for _, expression := range declaration.Values {
							found = found || containsCompile(expression, seen)
						}
					}
				}
			}
			return !found
		})
		return found
	}
	var calls []token.Position
	ast.Inspect(file, func(node ast.Node) bool {
		call, ok := node.(*ast.CallExpr)
		if !ok {
			return true
		}
		selector, ok := call.Fun.(*ast.SelectorExpr)
		if !ok {
			return true
		}
		owner, ok := selector.X.(*ast.Ident)
		if !ok {
			return true
		}
		path := imports[owner.Name]
		direct := path == "rosieos/rt-core/tools/rtctl" &&
			(selector.Sel.Name == "Compile" || selector.Sel.Name == "WriteConfig" || selector.Sel.Name == "RunCLI")
		if path == "os/exec" && (selector.Sel.Name == "Command" || selector.Sel.Name == "CommandContext") {
			for _, arg := range call.Args {
				direct = direct || containsCompile(arg, map[ast.Node]bool{})
			}
		}
		if direct {
			calls = append(calls, positions.Position(call.Pos()))
		}
		return true
	})
	return calls, nil
}

func TestSimulationCompileScanner(t *testing.T) {
	for _, body := range []string{
		`exec.Command("rtctl", "compile", "--config", "machine.json")`,
		`args := []string{"compile", "--config", "machine.json"}; exec.CommandContext(ctx, binary, args...)`,
		`exec.Command("sh", "-c", "rtctl compile --config machine.json")`,
		`compiler.Compile(raw, profiles)`,
		`compiler.WriteConfig(machine, profiles, output)`,
		`compiler.RunCLI(args, root, stdout, stderr)`,
	} {
		source := "package probe\nimport (\"os/exec\"; compiler \"rosieos/rt-core/tools/rtctl\")\nfunc fixture() { " + body + " }"
		calls, err := compileCalls("probe.go", source)
		if err != nil || len(calls) != 1 {
			t.Errorf("missed direct compiler invocation %q: %v %v", body, calls, err)
		}
	}
}

func TestSimulationFixturesSelectCPUBeforeCompile(t *testing.T) {
	root, err := filepath.Abs("../../..")
	if err != nil {
		t.Fatal(err)
	}
	// These are the sole subprocess boundaries with selection or intentional
	// fallback coverage. The compiler's own unit tests retain their input oracle.
	allowed := map[string]string{
		"rt-core/internal/simfixture/fixture_linux.go":   "shared selecting compiler",
		"rt-core/ipcclient/simulation_cpu_linux_test.go": "intentional daemon fallback regression",
		"rt-core/tools/reviewsim/probes_linux.go":        "in-process CLI uses SelectMachine before recording compile arguments",
		"rt-core/tools/reviewsim/composed_linux.go":      "in-process CLI uses SelectMachine before compiling the composed fixture",
	}
	allowed["offline-programming/v1/internal/targets/drive_profiles_test.go"] = "compile-only identity test; starts no simulator"
	allowed["offline-programming/v1/internal/targets/cell1430_opening_trial_test.go"] = "compile-only contract test; starts no simulator"
	allowed["offline-programming/v1/tools/cartesianjog-harness/main.go"] = "reads compiled limits for an external controller; starts no simulator"
	allowed["rt-core/tools/tuningreport/config.go"] = "reads compiled limits and identity for an offline report; starts no simulator"
	err = filepath.WalkDir(root, func(path string, entry fs.DirEntry, walkErr error) error {
		if walkErr != nil {
			return walkErr
		}
		if entry.IsDir() {
			switch entry.Name() {
			case ".git", "build", "node_modules", "vendor", ".venv", "target", ".pixi":
				return filepath.SkipDir
			}
			// Tool-owned hidden directories (.claude/worktrees holds stale agent
			// checkouts of this repository) are not repository sources: scanning
			// them reports every allowed compile site a second time under a
			// path the allow list cannot name.
			if path != root && strings.HasPrefix(entry.Name(), ".") {
				return filepath.SkipDir
			}
			return nil
		}
		relative, err := filepath.Rel(root, path)
		if err != nil {
			return err
		}
		relative = filepath.ToSlash(relative)
		if !strings.HasSuffix(path, ".go") || (!strings.HasSuffix(path, "_test.go") && !strings.Contains(relative, "/tools/") && !strings.Contains(relative, "/tests/") && !strings.Contains(relative, "/simfixture/")) {
			return nil
		}
		calls, err := compileCalls(path, nil)
		if err != nil {
			return err
		}
		for _, call := range calls {
			reason := allowed[relative]
			if strings.HasPrefix(relative, "rt-core/tools/rtctl/") {
				reason = "compiler implementation and input oracle"
			}
			if reason == "" {
				t.Errorf("%s:%d: direct rtctl compile bypasses CPU selection; use simfixture.Compile, CompileProfiles or CompileRaw", relative, call.Line)
			} else {
				t.Logf("%s:%d: allowed: %s", relative, call.Line, reason)
			}
		}
		return nil
	})
	if err != nil {
		t.Fatal(err)
	}
}
