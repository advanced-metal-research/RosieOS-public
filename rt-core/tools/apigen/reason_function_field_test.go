package main

import (
	"os/exec"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

// The review7 probes must emit their exact HTTP labels in compiled Go and fail
// the inventory gate at the imported constant, preventing dispatch blind spots.
func TestAdapterReasonInventoryReview7Dispatch(t *testing.T) {
	for _, kind := range []string{"interface", "function-field"} {
		t.Run(kind, func(t *testing.T) {
			constant, label := "FunctionLabel", "review7_function_field_public"
			provider := `func Fetch() string { return FunctionLabel }`
			dispatch := `type review7Callbacks struct { fetch func() string }
func review7Public() Response {
 callbacks := review7Callbacks{fetch: review7sub.Fetch}
 return Response{Error: callbacks.fetch()}
}
`
			if kind == "interface" {
				constant, label = "InterfaceLabel", "review7_interface_public"
				provider = `type Provider struct{}
func (Provider) Fetch() string { return InterfaceLabel }`
				dispatch = `type review7Provider interface { Fetch() string }
func review7Dispatch(source review7Provider) Response { return Response{Error: source.Fetch()} }
func review7Public() Response { return review7Dispatch(review7sub.Provider{}) }
`
			}
			fixture := map[string]string{
				"go.mod": "module rosieos/rt-core\n\ngo 1.24\n",
				"adapters/rosie/control/review7sub/provider.go": "package review7sub\n// unverified negative review constant.\nconst " + constant + " = " + `"` + label + `"` + "\n" + provider + "\n",
				"adapters/rosie/control/review7_dispatch.go": `package control
import "rosieos/rt-core/adapters/rosie/control/review7sub"
` + dispatch,
				"adapters/rosie/control/response.go": `package control
type Response struct { Error string }
`,
				"adapters/rosie/control/review7_dispatch_test.go": `package control
import (
 "encoding/json"
 "net/http"
 "net/http/httptest"
 "testing"
)
// HTTP serialization must preserve the callback result to prove public exposure.
func TestReview7DispatchHTTP(t *testing.T) {
 handler := http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) { json.NewEncoder(w).Encode(review7Public()) })
 recorder := httptest.NewRecorder()
 handler.ServeHTTP(recorder, httptest.NewRequest("GET", "/review7", nil))
 var response Response
 if err := json.Unmarshal(recorder.Body.Bytes(), &response); err != nil { t.Fatal(err) }
 if recorder.Code != 200 || response.Error != "` + label + `" { t.Fatalf("HTTP %d: %s", recorder.Code, recorder.Body.String()) }
}
`,
			}
			root := t.TempDir()
			writeReasonFixture(t, root, fixture)
			command := exec.Command("go", "test", "-race", "-count=1", "./adapters/rosie/control", "-run", "^TestReview7DispatchHTTP$")
			command.Dir = root
			if output, err := command.CombinedOutput(); err != nil {
				t.Fatalf("compiled review probe: %v\n%s", err, output)
			}
			output, err := reasonFixtureGate(t, fixture)
			for _, expected := range []string{"review7sub/provider.go:3:7", `undocumented reason "` + label + `"`, `constant "` + constant + `"`} {
				if err == nil || !strings.Contains(output, expected) {
					t.Fatalf("review probe escaped check-api (%s): %v\n%s", expected, err, output)
				}
			}
			t.Logf("HTTP 200 emitted %s; check-api rejected %s at review7sub/provider.go:3:7", label, constant)
		})
	}
}

// Every binding form must reach its imported declaration while a concrete
// receiver excludes private namesakes, preventing syntax rewrites from passing.
func TestAdapterReasonInventoryFunctionFieldBindings(t *testing.T) {
	for name, body := range map[string]string{
		"keyed":                  `callbacks := target.Callbacks{Fetch: target.Fetch}; return callbacks.Fetch()`,
		"positional":             `callbacks := target.Callbacks{target.Fetch}; return callbacks.Fetch()`,
		"assignment":             `var callbacks target.Callbacks; callbacks.Fetch = target.Fetch; return callbacks.Fetch()`,
		"pointer_alias":          `callbacks := &target.Callbacks{}; alias := callbacks; alias.Fetch = target.Fetch; return callbacks.Fetch()`,
		"function_alias":         `fetch := target.Fetch; alias := fetch; callbacks := target.Callbacks{Fetch: alias}; return callbacks.Fetch()`,
		"closure":                `callbacks := target.Callbacks{Fetch: func() string { return target.Fetch() }}; return callbacks.Fetch()`,
		"constructor":            `callbacks := target.New(target.Fetch); return callbacks.Fetch()`,
		"constructor_assignment": `callbacks := target.Assigned(target.Fetch); return callbacks.Fetch()`,
		"factory":                `callbacks := target.Callbacks{Fetch: target.Factory(target.Fetch)}; return callbacks.Fetch()`,
		"named_factory":          `callbacks := target.Callbacks{Fetch: target.NamedFactory(target.Fetch)}; return callbacks.Fetch()`,
		"tuple_factory":          `_, fetch := target.TupleFactory(target.Fetch); callbacks := target.Callbacks{Fetch: fetch}; return callbacks.Fetch()`,
		"field_alias":            `callbacks := target.New(target.Fetch); fetch := callbacks.Fetch; return fetch()`,
		"field_copy":             `first := target.New(target.Fetch); callbacks := target.Callbacks{Fetch: first.Fetch}; return callbacks.Fetch()`,
		"nested":                 `callbacks := struct { Inner *target.Callbacks }{Inner: target.New(target.Fetch)}; return callbacks.Inner.Fetch()`,
		"anonymous":              `callbacks := struct { Fetch func() string }{Fetch: target.Fetch}; return callbacks.Fetch()`,
		"all_writes":             `callbacks := target.Callbacks{Fetch: target.Fetch}; callbacks.Fetch = target.Second; return callbacks.Fetch()`,
		"elided_writes":          `callbacks := target.New(target.Fetch); _ = []target.Callbacks{{Fetch: target.Second}}; return callbacks.Fetch()`,
		"slice":                  `callbacks := []target.Callbacks{{Fetch: target.Fetch}}; return callbacks[0].Fetch()`,
		"map":                    `callbacks := map[int]*target.Callbacks{0: {Fetch: target.Fetch}}; return callbacks[0].Fetch()`,
		"promoted":               `callbacks := struct { *target.Callbacks }{target.New(target.Fetch)}; return callbacks.Fetch()`,
		"error_field":            `callbacks := struct { Error func() string }{Error: target.Fetch}; return callbacks.Error()`,
		"parameter":              `callbacks := struct { Fetch func(string) string }{Fetch: func(prefix string) string { return prefix }}; return callbacks.Fetch(target.PublicLabel)`,
	} {
		t.Run(name, func(t *testing.T) {
			fixture := functionFieldFixture(body)
			root := t.TempDir()
			writeReasonFixture(t, root, fixture)
			reasons, err := adapterReasonLiterals(filepath.Join(root, "adapters/rosie/control"))
			if err != nil {
				t.Fatal(err)
			}
			got := map[string]string{}
			for _, reason := range reasons {
				got[reason.name] = reason.constant
				if !strings.HasSuffix(filepath.ToSlash(reason.position.Filename), "target/provider.go") {
					t.Fatalf("lost declaration: %+v", reason)
				}
			}
			want := map[string]string{"field_public": "PublicLabel"}
			if name == "all_writes" || name == "elided_writes" {
				want["field_second"] = "SecondLabel"
			}
			if !reflect.DeepEqual(got, want) || len(reasons) != len(want) {
				t.Fatalf("field binding inventory: %+v, want %v", reasons, want)
			}
		})
	}
}

func functionFieldFixture(body string) map[string]string {
	fixture := map[string]string{
		"go.mod": "module rosieos/rt-core\n\ngo 1.24\n",
		"adapters/rosie/control/control.go": `package control
import "rosieos/rt-core/adapters/rosie/control/target"
type Response struct { Error string }
func public() Response { return Response{Error: dispatch()} }
func dispatch() string { ` + body + ` }
`,
		"adapters/rosie/control/target/provider.go": `package target
// unverified field-binding fixture labels.
const ( PublicLabel = "field_public"; SecondLabel = "field_second" )
type Callback func() string
type Callbacks struct { Fetch Callback }
func Fetch() string { return PublicLabel }
func Second() string { return SecondLabel }
func New(fetch Callback) *Callbacks { return &Callbacks{Fetch: fetch} }
func Assigned(fetch Callback) *Callbacks { callbacks := &Callbacks{}; callbacks.Fetch = fetch; return callbacks }
func Factory(fetch Callback) Callback { return forward(fetch) }
func forward(fetch Callback) Callback { return fetch }
func NamedFactory(fetch Callback) (result Callback) { result = fetch; return }
func TupleFactory(fetch Callback) (int, Callback) { return 0, fetch }
`,
		"adapters/rosie/control/decoy/provider.go": `package decoy
type Callbacks struct { Fetch func() string }
func Fetch() string { return "field_private_namesake" }
var callbacks = Callbacks{Fetch: Fetch}
`,
	}
	path := "adapters/rosie/control/target/provider.go"
	for _, constructor := range []string{"New", "Assigned"} {
		if !strings.Contains(body, "target."+constructor+"(") {
			var kept []string
			for _, line := range strings.Split(fixture[path], "\n") {
				if !strings.HasPrefix(line, "func "+constructor+"(") {
					kept = append(kept, line)
				}
			}
			fixture[path] = strings.Join(kept, "\n")
		}
	}
	return fixture
}

// An unknown assignment cannot prove compatible functions private, even when
// another assignment resolves; keep their labels and terminate cyclic aliases.
func TestAdapterReasonInventoryFunctionFieldUnresolved(t *testing.T) {
	for _, body := range []string{
		`var callbacks target.Callbacks; return callbacks.Fetch()`,
		`callbacks := target.Callbacks{Fetch: unknown}; return callbacks.Fetch()`,
		`callbacks := target.New(target.Fetch); callbacks.Fetch = unknown; return callbacks.Fetch()`,
		`callbacks := target.Callbacks{}; callbacks.Fetch = callbacks.Fetch; return callbacks.Fetch()`,
	} {
		root := t.TempDir()
		fixture := functionFieldFixture(body)
		writeReasonFixture(t, root, fixture)
		reasons, err := adapterReasonLiterals(filepath.Join(root, "adapters/rosie/control"))
		if err != nil {
			t.Fatal(err)
		}
		got := map[string]bool{}
		for _, reason := range reasons {
			got[reason.name] = true
		}
		for _, label := range []string{"field_public", "field_second", "field_private_namesake"} {
			if !got[label] {
				t.Fatalf("unresolved binding discarded %s: %v", label, got)
			}
		}
	}
}
