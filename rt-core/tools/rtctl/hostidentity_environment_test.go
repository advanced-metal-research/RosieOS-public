package rtctl

import (
	"go/ast"
	"go/parser"
	gotoken "go/token"
	"os"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"testing"
)

// TestHostIdentityRuntimeEnvironment fences deployment and process bindings
// separately, including an unexpected binding absent from the release manifest.
func TestHostIdentityRuntimeEnvironment(t *testing.T) {
	for i, service := range hostIdentityServices[:2] {
		for _, key := range []string{"ROSIE_RT_COMPILED_CONFIG", "ROSIE_RT_ANCHOR_DIR", "GRADIENT_RT_METRICS_STARTUP_READBACK_ENABLED", "GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", "GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED"} {
			for _, source := range []string{"deployment", "process", "both"} {
				for _, unexpected := range []bool{false, true} {
					t.Run(service+"/"+key+"/"+source+"/unexpected="+strconv.FormatBool(unexpected), func(t *testing.T) {
						root, config, manifest, release := identityFixture(t)
						if unexpected {
							delete(release.Services[service].Environment, key)
							identityFixtureJSON(t, root, "release.json", release)
						}
						value := "0"
						if key == "ROSIE_RT_ANCHOR_DIR" {
							value = "/unexpected/anchors"
						}
						for _, side := range []string{"deployment", "process"} {
							path, nul := "etc/rosie-rt-core/control.env", false
							if side == "process" {
								path, nul = "proc/"+strconv.Itoa(101+i)+"/environ", true
							}
							env := map[string]string{}
							for k, v := range release.Services[service].Environment {
								env[k] = v
							}
							if side == source || source == "both" {
								env[key] = value
							}
							hostcheckWrite(t, root, path, identityFixtureEnv(env, nul))
						}
						r, _, code := identityRun(t, config, manifest)
						if code != 2 || r.QualifiedMatches != 0 {
							t.Fatalf("runtime binding qualified: code=%d %+v", code, r)
						}
						for side, layer := range map[string]string{"deployment": "effective_environment", "process": "process_environment"} {
							want := "match"
							if side == source || source == "both" {
								want = "conflict"
								if !strings.Contains(r.Services[i].Layers[layer].Observed, "changed_bindings="+key) {
									t.Fatalf("missing binding reason: %+v", r.Services[i].Layers[layer])
								}
							}
							if r.Services[i].Layers[layer].Status != want {
								t.Fatalf("%s: want=%s got=%+v", layer, want, r.Services[i].Layers[layer])
							}
						}
					})
				}
			}
		}
	}
}

// TestHostIdentityRuntimeEnvironmentConsumers audits owned runtime sources;
// added environment reads must be explicitly covered before a release qualifies.
func TestHostIdentityRuntimeEnvironmentConsumers(t *testing.T) {
	covered := func(service, key, site string) {
		t.Helper()
		for _, allowed := range identityEnvironmentKeys(service) {
			if key == allowed {
				return
			}
		}
		t.Errorf("%s consumes %s at %s without a manifest binding", service, key, site)
	}
	// Pin indirect native reads as well as literal calls. A new getenv or helper
	// call requires review even when it reuses an existing variable name.
	nativeSites := map[string][]string{
		"engine/commissioning_worker.hpp": {`const char* directory = std::getenv("ROSIE_RT_ANCHOR_DIR");`},
		"include/daemon_options.hpp":      {`bool parse_env_flag(const char* name, bool default_value) {`, `const char* raw = std::getenv(name);`},
		"engine/cycle_machine.hpp": {
			`metrics_startup_readback_enabled(parse_env_flag("GRADIENT_RT_METRICS_STARTUP_READBACK_ENABLED", true)),`,
			`metrics_native_home_refresh_enabled(parse_env_flag("GRADIENT_RT_METRICS_NATIVE_HOME_REFRESH_ENABLED", true)),`,
			`metrics_absolute_feedback_poll_enabled(parse_env_flag("GRADIENT_RT_METRICS_ABSOLUTE_FEEDBACK_POLL_ENABLED", true)) {`,
		},
	}
	nativeRead := regexp.MustCompile(`\b(getenv|secure_getenv|parse_env_flag)\b`)
	literal := regexp.MustCompile(`(?:getenv|parse_env_flag)\("([A-Z0-9_]+)"`)
	for _, dir := range []string{"src", "engine", "include"} {
		err := filepath.WalkDir(filepath.Join("../..", dir), func(path string, entry os.DirEntry, err error) error {
			if err != nil || entry.IsDir() {
				return err
			}
			switch filepath.Ext(path) {
			case ".cpp", ".hpp", ".h", ".cc", ".c":
			default:
				return nil
			}
			raw, err := os.ReadFile(path)
			if err != nil {
				return err
			}
			rel, err := filepath.Rel("../..", path)
			if err != nil {
				return err
			}
			for _, line := range strings.Split(string(raw), "\n") {
				if !nativeRead.MatchString(line) {
					continue
				}
				sites := nativeSites[rel]
				if len(sites) == 0 || strings.TrimSpace(line) != sites[0] {
					t.Errorf("review new native environment consumer at %s: %s", rel, line)
					continue
				}
				nativeSites[rel] = sites[1:]
				if match := literal.FindStringSubmatch(line); match != nil {
					covered("rosie-rt-core", match[1], rel)
				}
			}
			return nil
		})
		if err != nil {
			t.Fatal(err)
		}
	}
	for path, sites := range nativeSites {
		if len(sites) != 0 {
			t.Errorf("native environment consumer changed at %s: %v", path, sites)
		}
	}
	for service, root := range map[string]string{"rosie-rt-control": "cmd/rt-control", "rosie-rt-natspublisher": "tools/natspublisher"} {
		seen := map[string]bool{}
		var scan func(string)
		scan = func(dir string) {
			if seen[dir] {
				return
			}
			seen[dir] = true
			fset := gotoken.NewFileSet()
			packages, err := parser.ParseDir(fset, filepath.Join("../..", dir), func(info os.FileInfo) bool { return !strings.HasSuffix(info.Name(), "_test.go") }, 0)
			if err != nil {
				t.Fatal(err)
			}
			for _, pkg := range packages {
				for path, file := range pkg.Files {
					osName := ""
					for _, imp := range file.Imports {
						importPath, err := strconv.Unquote(imp.Path.Value)
						if err != nil {
							t.Fatal(err)
						}
						if strings.HasPrefix(importPath, "rosieos/rt-core/") {
							scan(strings.TrimPrefix(importPath, "rosieos/rt-core/"))
						}
						if importPath == "os" {
							osName = "os"
							if imp.Name != nil {
								osName = imp.Name.Name
							}
							if osName == "." {
								t.Errorf("cannot audit dot-imported os in %s", path)
							}
						}
					}
					ast.Inspect(file, func(node ast.Node) bool {
						call, ok := node.(*ast.CallExpr)
						if !ok {
							return true
						}
						sel, ok := call.Fun.(*ast.SelectorExpr)
						if !ok {
							return true
						}
						pkg, ok := sel.X.(*ast.Ident)
						if !ok || pkg.Name != osName || (sel.Sel.Name != "Getenv" && sel.Sel.Name != "LookupEnv" && sel.Sel.Name != "Environ") {
							return true
						}
						if len(call.Args) != 1 {
							t.Errorf("unbounded environment read at %s", fset.Position(call.Pos()))
							return true
						}
						arg, ok := call.Args[0].(*ast.BasicLit)
						if !ok || arg.Kind != gotoken.STRING {
							t.Errorf("nonliteral environment read at %s", fset.Position(call.Pos()))
							return true
						}
						key, err := strconv.Unquote(arg.Value)
						if err != nil {
							t.Fatal(err)
						}
						covered(service, key, fset.Position(call.Pos()).String())
						return true
					})
				}
			}
		}
		scan(root)
	}
}
