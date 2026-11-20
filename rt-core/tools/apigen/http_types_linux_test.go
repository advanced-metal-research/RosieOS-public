package main

import (
	"bytes"
	"encoding/json"
	"fmt"
	"go/ast"
	"go/importer"
	"go/parser"
	"go/token"
	"go/types"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
)

// responsePayloadTypes discovers concrete Data values before interface erasure,
// preventing a new HTTP payload from bypassing the reflected wire inventory.
func responsePayloadTypes(files []*ast.File, info *types.Info) []types.Type {
	isResponse := func(typ types.Type) bool {
		if pointer, ok := typ.(*types.Pointer); ok {
			typ = pointer.Elem()
		}
		named, ok := types.Unalias(typ).(*types.Named)
		return ok && named.Obj().Name() == "Response"
	}
	var payloads []types.Type
	for _, file := range files {
		ast.Inspect(file, func(node ast.Node) bool {
			switch node := node.(type) {
			case *ast.AssignStmt:
				for i, lhs := range node.Lhs {
					field, ok := lhs.(*ast.SelectorExpr)
					if !ok || field.Sel.Name != "Data" || !isResponse(info.TypeOf(field.X)) {
						continue
					}
					if len(node.Rhs) == len(node.Lhs) {
						payloads = append(payloads, info.TypeOf(node.Rhs[i]))
					} else if tuple, ok := info.TypeOf(node.Rhs[0]).(*types.Tuple); ok {
						payloads = append(payloads, tuple.At(i).Type())
					}
				}
			case *ast.CompositeLit:
				if isResponse(info.TypeOf(node)) {
					for i, element := range node.Elts {
						if field, ok := element.(*ast.KeyValueExpr); ok {
							if name, ok := field.Key.(*ast.Ident); ok && name.Name == "Data" {
								payloads = append(payloads, info.TypeOf(field.Value))
							}
						} else if info.TypeOf(node).Underlying().(*types.Struct).Field(i).Name() == "Data" {
							payloads = append(payloads, info.TypeOf(element))
						}
					}
				}
			}
			return true
		})
	}
	return payloads
}

// TestHTTPResponseTypesAreReflected checks production handler assignments as well
// as schema roots; adding a payload requires a schema and a reflected Go fixture.
func TestHTTPResponseTypesAreReflected(t *testing.T) {
	command := exec.Command("go", "list", "-export", "-deps", "-json", "./adapters/rosie/control")
	command.Dir = "../.."
	var stderr bytes.Buffer
	command.Stderr = &stderr
	body, err := command.Output()
	if err != nil {
		t.Fatalf("load handler types: %v\n%s", err, stderr.Bytes())
	}
	decoder := json.NewDecoder(bytes.NewReader(body))
	exports := map[string]string{}
	fset := token.NewFileSet()
	var files []*ast.File
	for {
		var pkg struct {
			ImportPath, Export, Dir string
			GoFiles                 []string
		}
		if err := decoder.Decode(&pkg); err == io.EOF {
			break
		} else if err != nil {
			t.Fatal(err)
		}
		exports[pkg.ImportPath] = pkg.Export
		if pkg.ImportPath == "rosieos/rt-core/adapters/rosie/control" {
			for _, name := range pkg.GoFiles {
				file, err := parser.ParseFile(fset, filepath.Join(pkg.Dir, name), nil, 0)
				if err != nil {
					t.Fatal(err)
				}
				files = append(files, file)
			}
		}
	}
	info := &types.Info{Types: map[ast.Expr]types.TypeAndValue{}}
	config := types.Config{Importer: importer.ForCompiler(fset, "gc", func(path string) (io.ReadCloser, error) {
		return os.Open(exports[path])
	})}
	if _, err := config.Check("rosieos/rt-core/adapters/rosie/control", fset, files, info); err != nil {
		t.Fatal(err)
	}
	payloads := responsePayloadTypes(files, info)
	if len(payloads) == 0 {
		t.Fatal("no HTTP payloads discovered")
	}
	reflected := reflectedWireTypes()
	schema, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	declared := map[string]bool{}
	for _, spec := range schema.Types {
		declared[spec.Name] = true
	}
	for _, payload := range payloads {
		for {
			payload = types.Unalias(payload)
			switch container := payload.(type) {
			case *types.Pointer:
				payload = container.Elem()
			case *types.Slice:
				payload = container.Elem()
			case *types.Array:
				payload = container.Elem()
			case *types.Map:
				payload = container.Elem()
			default:
				goto element
			}
		}
	element:
		if _, opaque := payload.Underlying().(*types.Interface); opaque {
			t.Errorf("HTTP payload loses its concrete wire type: %s", payload)
			continue
		}
		if _, structured := payload.Underlying().(*types.Struct); !structured {
			continue
		}
		named, ok := payload.(*types.Named)
		if !ok {
			t.Errorf("anonymous HTTP payload needs a named schema type: %s", payload)
			continue
		}
		name := named.Obj().Name()
		if named.Obj().Pkg().Path() == "rosieos/rt-core/ipcclient" && (name == "Status" || name == "Description") {
			name = "Native" + name
		}
		if !declared[name] || reflected[name] == nil {
			t.Errorf("HTTP payload needs a schema and reflected wire fixture: %s", name)
		}
	}
}

func TestHTTPPayloadDiscoveryIncludesNewHandlerStructs(t *testing.T) {
	fset := token.NewFileSet()
	file, err := parser.ParseFile(fset, "handler.go", `package handler
type Response struct { Data any }
type Added struct { Generation uint64 }
func result() (uint64, Added, error) { return 0, Added{}, nil }
func handler() {
    var response Response
    var sequence uint64
    var err error
    sequence, response.Data, err = result()
    _ = sequence; _ = err
    response.Data = &Added{}
    _ = Response{Data: Added{}}
    _ = Response{Added{}}
    response.Data = struct { Unexpected uint64 }{}
}`, 0)
	if err != nil {
		t.Fatal(err)
	}
	info := &types.Info{Types: map[ast.Expr]types.TypeAndValue{}}
	if _, err := new(types.Config).Check("handler", fset, []*ast.File{file}, info); err != nil {
		t.Fatal(err)
	}
	payloads := responsePayloadTypes([]*ast.File{file}, info)
	if got := fmt.Sprint(payloads); got != "[handler.Added *handler.Added handler.Added handler.Added struct{Unexpected uint64}]" {
		t.Fatalf("missing concrete handler payload: %s", got)
	}
}
