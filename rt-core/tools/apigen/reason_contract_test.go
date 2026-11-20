package main

import (
	"go/ast"
	"go/parser"
	"go/token"
	"io/fs"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"strconv"
	"strings"
	"testing"
)

type reasonLiteral struct {
	name     string
	position token.Position
	constant string
	format   string
}

// adapterReasonLiterals inventories literal public errors and reasons, including
// field writes and reason-returning helpers, so check-api rejects uncatalogued labels.
func adapterReasonLiterals(root string) ([]reasonLiteral, error) {
	var reasons []reasonLiteral
	files := token.NewFileSet()
	trees := map[string]*ast.File{}
	methods := map[string][]*ast.FuncDecl{}
	sources := reasonSources{files: files, trees: trees, packages: map[string]*ast.Scope{}, values: map[*ast.Object]ast.Expr{}}
	seen := map[token.Pos]map[string]bool{}
	record := func(name string, position token.Pos) {
		relative, err := filepath.Rel(root, files.Position(position).Filename)
		if err != nil || strings.HasPrefix(relative, ".."+string(filepath.Separator)) {
			return // The command supplies sinks, not adapter-owned labels.
		}
		if name != "" && !seen[position][name] {
			if seen[position] == nil {
				seen[position] = map[string]bool{}
			}
			seen[position][name] = true
			reasons = append(reasons, reasonLiteral{name: name, position: files.Position(position)})
		}
	}
	readSource := func(path string, entry fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if entry.IsDir() {
			if entry.Name() == "testdata" {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(path, ".go") || strings.HasSuffix(path, "_test.go") || strings.HasSuffix(path, "_generated.go") {
			return nil
		}
		body, err := os.ReadFile(path)
		if err != nil {
			return err
		}
		file, err := parser.ParseFile(files, path, body, 0)
		if err != nil {
			return err
		}
		trees[path] = file
		for _, declaration := range file.Decls {
			if function, ok := declaration.(*ast.FuncDecl); ok && function.Recv != nil {
				methods[function.Name.Name] = append(methods[function.Name.Name], function)
			}
		}
		return nil
	}
	err := filepath.WalkDir(root, readSource)
	if err != nil {
		return nil, err
	}
	// Startup errors escape through cmd/rt-control's stderr. Include its real
	// callers so private argument branches can still be proven unreachable.
	startup := filepath.Clean(filepath.Join(root, "../../../cmd/rt-control"))
	if _, err := os.Stat(startup); err == nil {
		if err := filepath.WalkDir(startup, readSource); err != nil {
			return nil, err
		}
	} else if !os.IsNotExist(err) {
		return nil, err
	}
	sources.index()
	// Track values backwards from public sinks, including shared parameters and
	// tuple results; merely constructing a private error is not a public emission.
	calls := newReasonCalls(&sources, methods)
	bindings, closures, called := calls.bindings, calls.closures, calls.called
	visited := map[*ast.FuncDecl]map[int]bool{}
	visitedValues := map[reasonValue]bool{}
	var stringsFor func(ast.Expr, map[ast.Expr]bool) []string
	stringsFor = func(expression ast.Expr, visiting map[ast.Expr]bool) []string {
		if visiting[expression] {
			return nil
		}
		visiting[expression] = true
		defer delete(visiting, expression)
		if text, _, complete, _ := sources.stringPrefix(expression, map[*ast.Object]bool{}); complete {
			return []string{text}
		}
		var found []string
		for _, input := range bindings[sources.object(expression)] {
			values := stringsFor(input.expression, visiting)
			if len(values) == 0 {
				return nil // Unknown inputs cannot justify removing a public path.
			}
			found = append(found, values...)
		}
		return found
	}
	var add func(ast.Expr, string)
	var addResult func(ast.Expr, int)
	var returns func(*ast.FuncDecl, int, string)
	returns = func(function *ast.FuncDecl, index int, directory string) {
		if function.Body == nil || visited[function][index] {
			return
		}
		if visited[function] == nil {
			visited[function] = map[int]bool{}
		}
		visited[function][index] = true
		unreachable := map[ast.Node]bool{}
		ast.Inspect(function.Body, func(node ast.Node) bool {
			if branch, ok := node.(*ast.IfStmt); ok && reasonConditionAlwaysFalse(branch.Cond, stringsFor, sources.object) {
				unreachable[branch.Body] = true
			}
			return true
		})
		resultName := ""
		if function.Type.Results != nil {
			resultIndex := 0
			for _, result := range function.Type.Results.List {
				for _, name := range result.Names {
					if resultIndex == index {
						resultName = name.Name
					}
					resultIndex++
				}
				if len(result.Names) == 0 {
					resultIndex++
				}
			}
		}
		ast.Inspect(function.Body, func(node ast.Node) bool {
			if unreachable[node] {
				return false
			}
			if _, nested := node.(*ast.FuncLit); nested {
				return false
			}
			if statement, ok := node.(*ast.ReturnStmt); ok && index < len(statement.Results) {
				add(statement.Results[index], directory)
			} else if ok && len(statement.Results) == 1 {
				addResult(statement.Results[0], index)
			}
			if statement, ok := node.(*ast.AssignStmt); ok && resultName != "" {
				for i, target := range statement.Lhs {
					if name, ok := target.(*ast.Ident); ok && name.Name == resultName && i < len(statement.Rhs) {
						add(statement.Rhs[i], directory)
					}
				}
			}
			return true
		})
	}
	add = func(expression ast.Expr, _ string) { addResult(expression, 0) }
	addResult = func(expression ast.Expr, index int) {
		input := reasonValue{expression, index}
		if visitedValues[input] {
			return
		}
		visitedValues[input] = true
		directory := filepath.Dir(files.Position(expression.Pos()).Filename)
		if prefix, position, _, constant := sources.stringPrefix(expression, map[*ast.Object]bool{}); position.IsValid() {
			// A suffix after ':' is an open diagnostic, not another stable label.
			name, _, _ := strings.Cut(prefix, ":")
			if !strings.Contains(name, "%") {
				record(name, position)
				if constant != "" {
					for i := range reasons {
						if reasons[i].position == files.Position(position) {
							reasons[i].constant = constant
						}
					}
				}
			}
			return
		}
		if object := sources.object(expression); object != nil {
			for _, input := range bindings[object] {
				addResult(input.expression, input.index)
			}
		}
		switch expression := expression.(type) {
		case *ast.ParenExpr:
			addResult(expression.X, index)
		case *ast.BinaryExpr:
			if expression.Op == token.ADD {
				// A helper parameter can supply the stable prefix even when
				// static constant folding cannot evaluate the whole format.
				addResult(expression.X, index)
			}
		case *ast.UnaryExpr:
			addResult(expression.X, index)
		case *ast.CompositeLit:
			// Error wrappers can embed the cause positionally or by field name.
			for _, element := range expression.Elts {
				if field, ok := element.(*ast.KeyValueExpr); ok {
					add(field.Value, directory)
				} else {
					add(element, directory)
				}
			}
		case *ast.CallExpr:
			if function, ok := expression.Fun.(*ast.SelectorExpr); ok {
				name := function.Sel.Name
				if name == "Error" && len(expression.Args) == 0 {
					add(function.X, directory)
					if _, field := sources.underlyingType(function, map[ast.Expr]bool{}).(*ast.FuncType); !field {
						return
					}
				}
				if pkg, ok := function.X.(*ast.Ident); ok && len(expression.Args) > 0 &&
					((pkg.Name == "fmt" && (name == "Sprintf" || name == "Errorf")) || (pkg.Name == "errors" && name == "New")) {
					format, _, _, _ := sources.stringPrefix(expression.Args[0], map[*ast.Object]bool{})
					if strings.HasPrefix(format, "%s:") && len(expression.Args) > 1 {
						add(expression.Args[1], directory)
					} else {
						add(expression.Args[0], directory)
					}
					_, position, complete, _ := sources.stringPrefix(expression.Args[0], map[*ast.Object]bool{})
					if complete {
						for i := range reasons {
							if reasons[i].position == files.Position(position) {
								reasons[i].format = format
							}
						}
					}
					return
				}
			}
			for _, function := range called(expression) {
				returns(function, index, directory)
			}
		}
	}
	for path, file := range trees {
		directory := filepath.Dir(path)
		ast.Inspect(file, func(node ast.Node) bool {
			switch node := node.(type) {
			case *ast.AssignStmt:
				for i, target := range node.Lhs {
					if selector, ok := target.(*ast.SelectorExpr); ok && documentedTLSCallback(selector.Sel.Name) && i < len(node.Rhs) {
						if closure, ok := node.Rhs[i].(*ast.FuncLit); ok {
							returns(closures[closure], tlsCallbackErrorSlot(selector.Sel.Name), directory)
						}
					}
					if selector, ok := target.(*ast.SelectorExpr); ok && publicReasonField(selector.Sel.Name) && i < len(node.Rhs) {
						add(node.Rhs[i], directory)
					}
				}
			case *ast.KeyValueExpr:
				if key, ok := node.Key.(*ast.Ident); ok && documentedTLSCallback(key.Name) {
					if closure, ok := node.Value.(*ast.FuncLit); ok {
						returns(closures[closure], tlsCallbackErrorSlot(key.Name), directory)
					}
				}
				if key, ok := node.Key.(*ast.Ident); ok && publicReasonField(key.Name) {
					add(node.Value, directory)
				}
			case *ast.FuncDecl:
				if node.Type.Results == nil {
					break
				}
				index := 0
				for _, result := range node.Type.Results.List {
					typ, typed := result.Type.(*ast.Ident)
					isReason := typed && (strings.HasSuffix(typ.Name, "Reason") ||
						(typ.Name == "string" && (strings.HasSuffix(node.Name.Name, "Reason") || node.Name.Name == "Error")))
					count := len(result.Names)
					if count == 0 {
						count = 1
					}
					for offset := 0; offset < count; offset++ {
						if isReason || (len(result.Names) > 0 && publicReasonField(result.Names[offset].Name)) {
							returns(node, index+offset, directory)
						}
					}
					index += count
				}
			case *ast.CallExpr:
				selector, ok := node.Fun.(*ast.SelectorExpr)
				if !ok {
					break
				}
				if pkg, ok := selector.X.(*ast.Ident); ok {
					if pkg.Name == "fmt" && (selector.Sel.Name == "Fprintln" || selector.Sel.Name == "Fprintf") && len(node.Args) > 1 {
						if output, ok := node.Args[0].(*ast.SelectorExpr); ok {
							if pkg, ok := output.X.(*ast.Ident); ok && pkg.Name == "os" && output.Sel.Name == "Stderr" {
								for _, argument := range node.Args[1:] {
									add(argument, directory)
								}
							}
						}
					}
					if pkg.Name == "http" && selector.Sel.Name == "Error" && len(node.Args) >= 2 {
						add(node.Args[1], directory)
					}
					if pkg.Name == "log" && selector.Sel.Name == "Printf" && len(node.Args) > 0 {
						prefix, position, _, _ := sources.stringPrefix(node.Args[0], map[*ast.Object]bool{})
						// The anchor_store_io contract explicitly documents this log prefix.
						if strings.HasPrefix(prefix, "anchor_store_io ") {
							record("anchor_store_io", position)
						}
					}
				}
			case *ast.CompositeLit:
				if typ, ok := node.Type.(*ast.StructType); ok {
					index := 0
					for _, field := range typ.Fields.List {
						for _, name := range field.Names {
							if publicReasonField(name.Name) && index < len(node.Elts) {
								add(node.Elts[index], directory)
							}
							index++
						}
					}
				}
				// websocketError is the typed wire-close reason, including its private field.
				if typ, ok := node.Type.(*ast.Ident); ok && typ.Name == "websocketError" && len(node.Elts) == 2 {
					add(node.Elts[1], directory)
				}
			case *ast.ValueSpec:
				typ, typed := node.Type.(*ast.Ident)
				for i, name := range node.Names {
					if i < len(node.Values) && (strings.HasPrefix(name.Name, "Reason") || (typed && strings.HasSuffix(typ.Name, "Reason"))) {
						add(node.Values[i], directory)
					}
				}
			}
			return true
		})
	}
	return reasons, nil
}

func publicReasonField(name string) bool { return name == "Error" || name == "Reason" }

// Public rewrites must retain an unregistered label and its source declaration,
// preventing the review's named-format, struct, helper and literal probes passing.
func TestAdapterReasonInventoryNegativeProbes(t *testing.T) {
	for name, body := range map[string]string{
		"format": `import "fmt"
// unverified negative probe constant.
const reviewProbePrefix = "review_probe_format"
func probe(detail string) Response { return Response{Error: fmt.Sprintf(reviewProbePrefix + ": %s", detail)} }
`,
		"struct": `var probe = Response{Error: "review_probe_struct"}`,
		"helper": `func helper() string { return "review_probe_helper" }
func first() Response { return Response{Error: helper()} }
func second() Response { return Response{Error: helper()} }
`,
		"format_literal": `import "fmt"
func probe(detail string) Response { return Response{Error: fmt.Sprintf("review_probe_format_literal: %s", detail)} }
`,
		"typed": `type StoreReason string
// unverified negative probe constant.
const Unavailable StoreReason = "review_probe_typed"
`,
	} {
		t.Run(name, func(t *testing.T) {
			root := t.TempDir()
			path := filepath.Join(root, "review_probe.go")
			if err := os.WriteFile(path, []byte("package control\n"+body), 0600); err != nil {
				t.Fatal(err)
			}
			reasons, err := adapterReasonLiterals(root)
			if err != nil || len(reasons) != 1 || reasons[0].name != "review_probe_"+name {
				t.Fatalf("negative probe escaped inventory: %+v, %v", reasons, err)
			}
			schema, err := parse(schemaBytes(t))
			if err != nil {
				t.Fatal(err)
			}
			for _, registered := range schema.Reasons {
				if registered.Name == reasons[0].name {
					t.Fatal("negative probe must remain unregistered")
				}
			}
			if reasons[0].position.Filename != path || reasons[0].position.Line < 2 {
				t.Fatalf("missing probe source location: %+v", reasons[0])
			}
			if name == "format" && (reasons[0].constant != "reviewProbePrefix" || reasons[0].position.Line != 4) {
				t.Fatalf("missing constant declaration: %+v", reasons[0])
			}
			output, err := reasonInventoryGate(t, "package control\n"+body)
			if err == nil || !strings.Contains(output, "undocumented reason") || !strings.Contains(output, "review_probe.go:") || !strings.Contains(output, reasons[0].name) {
				t.Fatalf("check-api completeness probe must fail with source and reason: %v\n%s", err, output)
			}
			if name == "format" && !strings.Contains(output, `constant "reviewProbePrefix"`) {
				t.Fatalf("check-api omitted constant name:\n%s", output)
			}
		})
	}
}

// Run the actual completeness test in a source fixture, preventing a regression
// that inventories a probe but accidentally lets the check-api gate accept it.
func reasonInventoryGate(t *testing.T, body string) (string, error) {
	t.Helper()
	return reasonFixtureGate(t, map[string]string{"adapters/rosie/control/review_probe.go": body})
}

// reasonFixtureGate runs check-api's completeness check on an isolated source
// tree, so negative probes never mutate the production adapter or its schema.
func reasonFixtureGate(t *testing.T, fixture map[string]string) (string, error) {
	t.Helper()
	root := t.TempDir()
	fixture["protocol/application-v1.schema.json"] = string(schemaBytes(t))
	fixture["go.mod"] = "module rosieos/rt-core\n\ngo 1.24\n"
	writeReasonFixture(t, root, fixture)
	directory := filepath.Join(root, "tools/apigen")
	if err := os.MkdirAll(directory, 0700); err != nil {
		t.Fatal(err)
	}
	command := exec.Command(os.Args[0], "-test.run=^TestEveryEmittedReasonIsInTheSchema$", "-test.count=1")
	command.Dir = directory
	output, err := command.CombinedOutput()
	return string(output), err
}

func writeReasonFixture(t *testing.T, root string, fixture map[string]string) {
	t.Helper()
	for path, contents := range fixture {
		path = filepath.Join(root, path)
		if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, []byte(contents), 0600); err != nil {
			t.Fatal(err)
		}
	}
}

// Constant resolution crosses files and adapter imports without treating a
// shadowed local variable or an unrelated package's constant as the public value.
func TestAdapterReasonLiteralsResolveNamedConstants(t *testing.T) {
	root := t.TempDir()
	for path, body := range map[string]string{
		"go.mod": "module inventory.test/control\n\ngo 1.24\n",
		"constants.go": `package control
// unverified constant-resolution fixtures.
const ( prefix = "same_"; suffix = "package"; combined = prefix + suffix )
`,
		"control.go": `package control
import "fmt"
import renamed "inventory.test/control/sub"
import . "inventory.test/control/dot"
func probe(detail string) Response {
 // unverified local fixture constant.
 const local = "local_constant"
 return Response{Error: fmt.Sprintf(combined + ": %s", detail), Reason: renamed.Prefix + ": " + detail}
 _ = Response{Error: DotPrefix, Reason: local}
}
func shadow(combined string) Response { return Response{Error: combined} }
`,
		"sub/constants.go": `package differentlyNamed
// unverified imported fixture constants.
const Prefix = "qualified_constant"
`,
		"dot/constants.go": `package dot
// unverified inherited fixture constants.
const ( first = "dot_constant"; DotPrefix )
`,
		"default.go": `package control
import "inventory.test/control/sub"
var response = Response{Error: differentlyNamed.Prefix}
`,
	} {
		path = filepath.Join(root, path)
		if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, []byte(body), 0600); err != nil {
			t.Fatal(err)
		}
	}
	reasons, err := adapterReasonLiterals(root)
	if err != nil {
		t.Fatal(err)
	}
	got := map[string]string{}
	for _, reason := range reasons {
		got[reason.name] = reason.constant
	}
	want := map[string]string{"same_package": "combined", "qualified_constant": "Prefix", "dot_constant": "DotPrefix", "local_constant": "local"}
	if !reflect.DeepEqual(got, want) || len(reasons) != 4 {
		t.Fatalf("constant inventory: %+v, want %v", reasons, want)
	}
}

type reasonSources struct {
	files    *token.FileSet
	trees    map[string]*ast.File
	packages map[string]*ast.Scope
	values   map[*ast.Object]ast.Expr
	imports  map[string]map[string][]*ast.Scope
}

// index retains lexical objects and package declarations so a shadowed name or
// an import alias cannot substitute an unrelated constant into a public reason.
func (s *reasonSources) index() {
	for path, file := range s.trees {
		directory := filepath.Dir(path)
		if s.packages[directory] == nil {
			s.packages[directory] = ast.NewScope(nil)
		}
		for _, object := range file.Scope.Objects {
			s.packages[directory].Insert(object)
		}
		ast.Inspect(file, func(node ast.Node) bool {
			declaration, ok := node.(*ast.GenDecl)
			if !ok || declaration.Tok != token.CONST {
				return true
			}
			var values []ast.Expr
			for _, spec := range declaration.Specs {
				value := spec.(*ast.ValueSpec)
				if len(value.Values) > 0 {
					values = value.Values
				}
				for i, name := range value.Names {
					if i < len(values) {
						s.values[name.Obj] = values[i]
					}
				}
			}
			return true
		})
	}
	s.imports = map[string]map[string][]*ast.Scope{}
	for path, file := range s.trees {
		s.imports[path] = map[string][]*ast.Scope{}
		for _, spec := range file.Imports {
			directory := s.importDirectory(path, spec)
			if scope := s.packages[directory]; scope != nil {
				alias := ""
				if spec.Name != nil {
					alias = spec.Name.Name
				} else {
					for importedPath, imported := range s.trees {
						if filepath.Dir(importedPath) == directory {
							alias = imported.Name.Name
							break
						}
					}
				}
				s.imports[path][alias] = append(s.imports[path][alias], scope)
			}
		}
	}
}

// importDirectory resolves only packages inside the inventoried module tree;
// external packages and generated contracts cannot invent source occurrences.
func (s *reasonSources) importDirectory(file string, spec *ast.ImportSpec) string {
	path, err := strconv.Unquote(spec.Path.Value)
	if err != nil {
		return ""
	}
	for directory := filepath.Dir(file); ; directory = filepath.Dir(directory) {
		if body, err := os.ReadFile(filepath.Join(directory, "go.mod")); err == nil {
			fields := strings.Fields(string(body))
			for i := 0; i+1 < len(fields); i++ {
				if fields[i] == "module" && strings.HasPrefix(path, fields[i+1]+"/") {
					return filepath.Join(directory, filepath.FromSlash(strings.TrimPrefix(path, fields[i+1]+"/")))
				}
			}
			return ""
		}
		if filepath.Dir(directory) == directory {
			return ""
		}
	}
}

func (s *reasonSources) object(expression ast.Expr) *ast.Object {
	path := s.files.Position(expression.Pos()).Filename
	file := s.trees[path]
	if file == nil {
		return nil
	}
	name, qualifier := "", "."
	switch expression := expression.(type) {
	case *ast.Ident:
		if expression.Obj != nil {
			return expression.Obj
		}
		name = expression.Name
		if object := s.packages[filepath.Dir(path)].Lookup(name); object != nil {
			return object
		}
	case *ast.SelectorExpr:
		pkg, ok := expression.X.(*ast.Ident)
		if !ok || pkg.Obj != nil {
			return nil
		}
		name, qualifier = expression.Sel.Name, pkg.Name
	default:
		return nil
	}
	for _, scope := range s.imports[path][qualifier] {
		if object := scope.Lookup(name); object != nil {
			return object
		}
	}
	return nil
}

// declaredType follows receiver, field, constructor and local declarations across
// imports so a method's public result cannot disappear at a package boundary.
func (s *reasonSources) declaredType(expression ast.Expr, visiting map[ast.Expr]bool) *ast.Object {
	if expression == nil || visiting[expression] {
		return nil
	}
	visiting[expression] = true
	defer delete(visiting, expression)
	if object := s.object(expression); object != nil {
		switch declaration := object.Decl.(type) {
		case *ast.TypeSpec:
			if _, ok := declaration.Type.(*ast.InterfaceType); ok {
				return nil
			}
			if declaration.Assign.IsValid() {
				return s.declaredType(declaration.Type, visiting)
			}
			return object
		case *ast.Field:
			return s.declaredType(declaration.Type, visiting)
		case *ast.ValueSpec:
			if declaration.Type != nil {
				return s.declaredType(declaration.Type, visiting)
			}
			for i, name := range declaration.Names {
				if name.Obj == object && i < len(declaration.Values) {
					return s.declaredType(declaration.Values[i], visiting)
				}
			}
		case *ast.AssignStmt:
			for i, target := range declaration.Lhs {
				if s.object(target) == object && i < len(declaration.Rhs) {
					return s.declaredType(declaration.Rhs[i], visiting)
				}
			}
		}
	}
	switch expression := expression.(type) {
	case *ast.StarExpr:
		return s.declaredType(expression.X, visiting)
	case *ast.UnaryExpr:
		return s.declaredType(expression.X, visiting)
	case *ast.ParenExpr:
		return s.declaredType(expression.X, visiting)
	case *ast.CompositeLit:
		return s.declaredType(expression.Type, visiting)
	case *ast.CallExpr:
		if object := s.object(expression.Fun); object != nil {
			if function, ok := object.Decl.(*ast.FuncDecl); ok && function.Type.Results != nil {
				return s.declaredType(function.Type.Results.List[0].Type, visiting)
			}
		}
		return s.declaredType(expression.Fun, visiting)
	case *ast.SelectorExpr:
		if receiver := s.declaredType(expression.X, visiting); receiver != nil {
			if structure, ok := receiver.Decl.(*ast.TypeSpec).Type.(*ast.StructType); ok {
				for _, field := range structure.Fields.List {
					for _, name := range field.Names {
						if name.Name == expression.Sel.Name {
							return s.declaredType(field.Type, visiting)
						}
					}
				}
			}
		}
	}
	return nil
}

// stringPrefix folds literal and named-constant concatenation before dynamic
// diagnostics, preserving the constant's declaration for actionable failures.
func (s *reasonSources) stringPrefix(expression ast.Expr, visiting map[*ast.Object]bool) (string, token.Pos, bool, string) {
	if object := s.object(expression); object != nil && object.Kind == ast.Con && !visiting[object] {
		if value := s.values[object]; value != nil {
			visiting[object] = true
			prefix, position, complete, _ := s.stringPrefix(value, visiting)
			delete(visiting, object)
			if position.IsValid() {
				return prefix, object.Pos(), complete, object.Name
			}
		}
	}
	switch expression := expression.(type) {
	case *ast.BasicLit:
		if expression.Kind == token.STRING {
			value, err := strconv.Unquote(expression.Value)
			return value, expression.Pos(), err == nil, ""
		}
	case *ast.ParenExpr:
		return s.stringPrefix(expression.X, visiting)
	case *ast.BinaryExpr:
		if expression.Op == token.ADD {
			left, position, complete, constant := s.stringPrefix(expression.X, visiting)
			if !complete {
				return left, position, false, constant
			}
			right, _, complete, _ := s.stringPrefix(expression.Y, visiting)
			return left + right, position, complete, constant
		}
	}
	return "", token.NoPos, false, ""
}

// Nested sentinel errors and typed reason literals must reach the same inventory
// as top-level errors; otherwise a newly returned reason could pass check-api.
func TestAdapterReasonLiteralsIncludeSubpackagesAndTypedConstants(t *testing.T) {
	root := t.TempDir()
	for path, body := range map[string]string{
		"go.mod":                "module inventory.test/control\n\ngo 1.24\n",
		"control.go":            "package control\nimport \"errors\"\nimport \"inventory.test/control/anchors\"\nvar err = errors.New(\"registered\")\nconst ReasonNext string = \"unregistered_constant\"\nvar response = Response{Error: err.Error(), Reason: anchors.ErrIO.Error()}\n",
		"anchors/store.go":      "package anchors\nimport \"errors\"\nvar ErrIO = errors.New(\n`unregistered_store`,\n)\ntype StoreReason string\nconst Unavailable StoreReason = \"unregistered_typed\"\n",
		"anchors/store_test.go": "package anchors\nvar err = errors.New(\"test_only\")\n",
		"api_generated.go":      "package control\nconst ReasonGenerated = \"generated_only\"\n",
		"testdata/probe.go":     "package probe\nvar err = errors.New(\"fixture_only\")\n",
	} {
		path = filepath.Join(root, path)
		if err := os.MkdirAll(filepath.Dir(path), 0700); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, []byte(body), 0600); err != nil {
			t.Fatal(err)
		}
	}
	reasons, err := adapterReasonLiterals(root)
	if err != nil {
		t.Fatal(err)
	}
	got := map[string]int{}
	for _, reason := range reasons {
		got[reason.name]++
		if !reason.position.IsValid() {
			t.Errorf("missing source location for %q", reason.name)
		}
	}
	want := map[string]int{"registered": 1, "unregistered_constant": 1, "unregistered_store": 1, "unregistered_typed": 1}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("reason inventory = %v, want %v", got, want)
	}
}

// Public field writes, composite values and helper returns must all reach the
// catalogue, preventing syntax-only rewrites from hiding new refusal labels.
func TestAdapterReasonLiteralsIncludePublicFieldsAndReturns(t *testing.T) {
	root := t.TempDir()
	sources := map[string]string{
		"control.go": `package control
import "fmt"
type Response struct { Error, Reason, State string }
func reply(detail string) {
 var response Response
 response.Error = "assignment"
 response.Reason = ("composed_" + "reason") + ": " + detail
 response.Error = fmt.Sprintf("formatted: %s", detail)
 response.Reason = helper()
 response.Error, response.Reason = "multiple_error", "multiple_reason"
 _ = Response{Error: "struct_error", Reason: "struct_reason", State: "not_a_reason"}
 response.Error = ""
 _ = func() string { return "unrelated_closure" }
}

func stateReason() string { return "returned_reason" }
func (Response) Error() string { return "returned_error" + ": details" }
type PublicReason string
func typed() PublicReason { return "typed_return" }
func namedReason() (reason string) { reason = "named_return"; return }
func result() (count int, Reason string) { return 1, "named_reason" }
func unrelated() string { return "not_a_reason" }
`,
		"helpers.go": `package control
func helper() string { return nested() }
func nested() string { return "helper_return" }
`,
	}
	for path, body := range sources {
		if err := os.WriteFile(filepath.Join(root, path), []byte(body), 0600); err != nil {
			t.Fatal(err)
		}
	}
	reasons, err := adapterReasonLiterals(root)
	if err != nil {
		t.Fatal(err)
	}
	got := map[string]int{}
	for _, reason := range reasons {
		got[reason.name]++
		if !reason.position.IsValid() {
			t.Errorf("missing source location for %q", reason.name)
		}
	}
	want := map[string]int{
		"assignment": 1, "composed_reason": 1, "formatted": 1,
		"multiple_error": 1, "multiple_reason": 1, "struct_error": 1,
		"struct_reason": 1, "returned_reason": 1, "returned_error": 1,
		"typed_return": 1, "named_return": 1, "named_reason": 1, "helper_return": 1,
	}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("reason inventory = %v, want %v", got, want)
	}
}

// Private diagnostics remain outside the catalogue; flowing the same error
// through a local, shared helper, tuple return or wire reply must expose it.
func TestAdapterReasonLiteralsExcludePrivateDiagnostics(t *testing.T) {
	root := t.TempDir()
	body := `package control
import "errors"
import "fmt"
import "net/http"
// unverified private fixture constant.
const privatePrefix = "review_private_format"
// unverified shared public prefix fixture.
const publicPrefix = "public_"
var privateError = errors.New("review private global diagnostic")
func reviewProbeInternalDiagnostic() error { return errors.New("review private setup diagnostic") }
func privateFormat(detail string) error { return fmt.Errorf(privatePrefix + ": %s", detail) }
func privateLiteral(detail string) error { return fmt.Errorf("review_private_literal: %s", detail) }
func setup() { _ = reviewProbeInternalDiagnostic(); _ = privateError; _ = privateFormat("detail"); _ = privateLiteral("detail") }
func reply(err error) { _ = Response{Error: err.Error()} }
func pair() (int, error) { return 1, errors.New("public_tuple") }
func named() (err error) { err = errors.New("public_named"); return }
func public() {
 _ = Response{Error: publicPrefix + "first", Reason: publicPrefix + "second"}
 err := errors.New("public_local")
 reply(err)
 _, err = pair()
 reply(err)
 reply(named())
 wire := func(err error) { http.Error(nil, err.Error(), 400) }
 wire(errors.New("public_wire"))
 send := func(err error) { _ = struct { Reason string }{err.Error()} }
 send(errors.New("public_positional"))
}
`
	path := filepath.Join(root, "private_probe.go")
	if err := os.WriteFile(path, []byte(body), 0600); err != nil {
		t.Fatal(err)
	}
	reasons, err := adapterReasonLiterals(root)
	if err != nil {
		t.Fatal(err)
	}
	got := map[string]int{}
	for _, reason := range reasons {
		got[reason.name]++
	}
	want := map[string]int{"public_local": 1, "public_tuple": 1, "public_named": 1, "public_wire": 1, "public_positional": 1, "public_first": 1, "public_second": 1}
	if !reflect.DeepEqual(got, want) {
		t.Fatalf("public/private inventory: %v, want %v", got, want)
	}
	output, err := reasonInventoryGate(t, `package control
import "errors"
func reviewProbeInternalDiagnostic() error { return errors.New("review private setup diagnostic") }
var response = Response{Error: "native_execution_failed"}
`)
	if err != nil {
		t.Fatalf("private diagnostic failed check-api completeness gate: %v\n%s", err, output)
	}
}

// TestReviewErrorFieldLiteralsAreCatalogued pins the production execution failure
// to its assignment and schema entry so neither an inventory gap nor drift passes.
func TestReviewErrorFieldLiteralsAreCatalogued(t *testing.T) {
	emitted, err := adapterReasonLiterals("../../adapters/rosie/control")
	if err != nil {
		t.Fatal(err)
	}
	schema, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	inventoried, registered := false, false
	unique := map[string]bool{}
	for _, reason := range emitted {
		unique[reason.name] = true
		if reason.name == "native_execution_failed" && filepath.Base(reason.position.Filename) == "controller.go" {
			inventoried = true
			t.Logf("execution failure assignment: %s", reason.position)
		}
	}
	for _, reason := range schema.Reasons {
		if reason.Name == "native_execution_failed" {
			registered = true
		}
	}
	t.Logf("adapter reason inventory: %d literals, %d unique labels; schema: %d reasons", len(emitted), len(unique), len(schema.Reasons))
	if !inventoried || !registered {
		t.Fatalf("controller.go native_execution_failed: inventoried=%t registered=%t", inventoried, registered)
	}
}
