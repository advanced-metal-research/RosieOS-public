//go:build linux

package control

import (
	"bytes"
	"context"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"math"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"reflect"
	"strconv"
	"strings"
	"testing"

	"rosieos/rt-core/adapters/rosie/densejointtraj"
	"rosieos/rt-core/ipcclient"
)

// programReasonLabels reads the authoritative catalogue so newly emitted
// diagnostics cannot pass by being added only to a test's parallel allowlist.
func programReasonLabels(t *testing.T) map[string]bool {
	t.Helper()
	raw, err := os.ReadFile("../../../protocol/application-v1.schema.json")
	if err != nil {
		t.Fatal(err)
	}
	var schema struct {
		Reasons []struct{ Name, Meaning string }
	}
	if err := json.Unmarshal(raw, &schema); err != nil {
		t.Fatal(err)
	}
	labels := make(map[string]bool)
	for _, reason := range schema.Reasons {
		if reason.Name == "" || reason.Meaning == "" || strings.ContainsAny(reason.Meaning, "\r\n") {
			t.Fatalf("invalid reason: %+v", reason)
		}
		labels[reason.Name] = true
	}
	return labels
}

// Static enumeration covers diagnostics outside the committed corpus, including
// internal encoder failures that must retain a catalogue prefix if exposed.
func TestProgramDiagnosticCatalogue(t *testing.T) {
	labels := programReasonLabels(t)
	paths, err := filepath.Glob("../densejointtraj/*.go")
	if err != nil {
		t.Fatal(err)
	}
	for _, path := range paths {
		if strings.HasSuffix(path, "_test.go") {
			continue
		}
		file, err := parser.ParseFile(token.NewFileSet(), path, nil, 0)
		if err != nil {
			t.Fatal(err)
		}
		ast.Inspect(file, func(node ast.Node) bool {
			call, ok := node.(*ast.CallExpr)
			if !ok {
				return true
			}
			if fn, ok := call.Fun.(*ast.SelectorExpr); ok && fn.Sel.Name == "Errorf" {
				t.Errorf("uncatalogued free-text error in %s", path)
			}
			fn, ok := call.Fun.(*ast.Ident)
			if !ok || fn.Name != "reject" {
				return true
			}
			literal, ok := call.Args[0].(*ast.BasicLit)
			if !ok {
				t.Fatalf("nonliteral dense diagnostic in %s", path)
			}
			label, err := strconv.Unquote(literal.Value)
			if err != nil || !labels[label] {
				t.Errorf("diagnostic %q absent from reason catalogue", label)
			}
			if label == ReasonSchemaMismatch {
				t.Error("dense schema diagnostic collides with control envelope")
			}
			return true
		})
	}
	generated, err := parser.ParseFile(token.NewFileSet(), "api_generated.go", nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	constants := make(map[string]string)
	ast.Inspect(generated, func(node ast.Node) bool {
		value, ok := node.(*ast.ValueSpec)
		if !ok || len(value.Names) != 1 || len(value.Values) != 1 {
			return true
		}
		literal, ok := value.Values[0].(*ast.BasicLit)
		if ok && strings.HasPrefix(value.Names[0].Name, "Reason") {
			constants[value.Names[0].Name], _ = strconv.Unquote(literal.Value)
		}
		return true
	})
	program, err := parser.ParseFile(token.NewFileSet(), "program.go", nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	ast.Inspect(program, func(node ast.Node) bool {
		call, ok := node.(*ast.CallExpr)
		if !ok {
			return true
		}
		fn, ok := call.Fun.(*ast.SelectorExpr)
		if !ok {
			return true
		}
		pkg, ok := fn.X.(*ast.Ident)
		if !ok {
			return true
		}
		argument := -1
		if pkg.Name == "errors" && fn.Sel.Name == "New" {
			argument = 0
		}
		if pkg.Name == "fmt" && (fn.Sel.Name == "Errorf" || fn.Sel.Name == "Sprintf") {
			argument = 1
		}
		if argument < 0 {
			return true
		}
		if len(call.Args) <= argument {
			t.Fatal("program diagnostic lacks catalogue prefix")
		}
		name, ok := call.Args[argument].(*ast.Ident)
		if !ok || !labels[constants[name.Name]] {
			t.Fatalf("program diagnostic must use a generated reason: %v", call.Args[argument])
		}
		return true
	})
}

func programHTTPResponse(t *testing.T, c *Controller, f Fence, blob []byte) (int, Response) {
	t.Helper()
	r := httptest.NewRequest(http.MethodPost, "/v1/program", bytes.NewReader(blob))
	r.Header.Set("X-Control-Session", f.Session)
	r.Header.Set("X-Control-Generation", fmt.Sprint(f.Generation))
	w := httptest.NewRecorder()
	NewHandler(c).ServeHTTP(w, r)
	var response Response
	if err := json.Unmarshal(w.Body.Bytes(), &response); err != nil {
		t.Fatal(err)
	}
	return w.Code, response
}

// Every corpus/example admission response, plus damaged envelopes and payloads,
// must resolve through the schema table before a consumer can act on its error.
func TestProgramCorpusAdmissionReasons(t *testing.T) {
	labels := programReasonLabels(t)
	var paths []string
	for _, pattern := range []string{"../densejointtraj/testdata/corpus/*.rdt", "../../../clients/cpp/examples/testdata/*.rdt"} {
		matches, err := filepath.Glob(pattern)
		if err != nil || len(matches) == 0 {
			t.Fatalf("corpus %s: %v", pattern, err)
		}
		paths = append(paths, matches...)
	}
	for _, path := range paths {
		raw, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		cases := map[string][]byte{"original": raw, "short_prefix": raw[:3], "header_length": raw[:4], "truncated_body": raw[:len(raw)-1], "extra_body": append(append([]byte(nil), raw...), 0)}
		badJSON := append([]byte(nil), raw...)
		badJSON[4] = '!'
		cases["json"] = badJSON
		badBlock := append([]byte(nil), raw...)
		badBlock[len(badBlock)-1] ^= 1
		cases["hash"] = badBlock
		oversize := make([]byte, 4)
		binary.BigEndian.PutUint32(oversize, 1048577)
		cases["size"] = oversize
		plan, err := densejointtraj.Decode(raw)
		if err != nil {
			t.Fatal(err)
		}
		for name, mutate := range map[string]func(*densejointtraj.Header){
			"schema":   func(h *densejointtraj.Header) { h.Schema = "invalid" },
			"axes":     func(h *densejointtraj.Header) { h.Axes.AxisCount = 8 },
			"encoding": func(h *densejointtraj.Header) { h.SampleEncoding.RecordBytes = 152 },
			"limits":   func(h *densejointtraj.Header) { h.Limits.MaxAbsQdRadS = nil },
			"time":     func(h *densejointtraj.Header) { h.Sampling.DtS = 0 },
			"identity": func(h *densejointtraj.Header) { h.ProgramID = "" },
			"revision": func(h *densejointtraj.Header) { h.ManifestRevision = 8 },
			"count":    func(h *densejointtraj.Header) { h.Sampling.TotalSampleCount++ },
		} {
			header := plan.Header
			mutate(&header)
			blob, err := densejointtraj.Encode(header, plan.Segments)
			if err != nil {
				t.Fatal(err)
			}
			cases[name] = blob
		}
		for name, blob := range cases {
			t.Run(filepath.Base(path)+"/"+name, func(t *testing.T) {
				c, f, _ := admissionController()
				c.description, c.axes = programDescription()
				c.binding.Revision = 7
				for i := range c.description.Axes {
					limit := 1.0
					c.description.Axes[i].MaxVelocity = .1
					c.description.Axes[i].MaxAcceleration = &limit
				}
				status, response := programHTTPResponse(t, c, f, blob)
				if response.Error == "" {
					if name != "original" || status != http.StatusOK {
						t.Fatalf("invalid admission accepted: %d %+v", status, response)
					}
					return
				}
				label, _, _ := strings.Cut(response.Error, ":")
				if !labels[label] {
					t.Fatalf("uncatalogued admission diagnostic: %s", response.Error)
				}
				if name == "schema" && label != ReasonDenseSchemaMismatch {
					t.Fatalf("schema collision: %s", label)
				}
				if status != http.StatusConflict && status != http.StatusRequestEntityTooLarge {
					t.Fatalf("status %d", status)
				}
			})
		}
	}
}

func TestProgramAccelerationAdmissionPreservesHandles(t *testing.T) {
	c, f, native := admissionController()
	c.description, c.axes = programDescription()
	c.binding.Revision = 7
	for i := range c.description.Axes {
		limit := 1.0
		c.description.Axes[i].MaxVelocity = .1
		c.description.Axes[i].MaxAcceleration = &limit
	}
	read := func(name string) []byte {
		raw, err := os.ReadFile("../../../clients/cpp/examples/testdata/" + name + ".rdt")
		if err != nil {
			t.Fatal(err)
		}
		return raw
	}
	program, err := c.PrepareProgram(f, read("valid"))
	if err != nil {
		t.Fatal(err)
	}
	before := c.execution
	for _, name := range []string{"acceleration", "velocity"} {
		status, response := programHTTPResponse(t, c, f, read(name))
		if status != 409 || response.Error != ReasonNativeLimitExceeded+": segment=0 sample=1 axis=J1" || response.NativeResult != nil {
			t.Fatalf("limit refusal: %d %+v", status, response)
		}
		if native.prepares.Load() != 1 || len(c.handles) != 1 || c.handles[program.handle].State != HandlePrepared || !reflect.DeepEqual(c.execution, before) || c.DescribeProgram().Identity != program.Identity {
			t.Fatalf("precheck changed prior admission: %+v", c.execution)
		}
	}
	identity := program.Identity
	identity.SourceDigest = "wrong"
	_, err = c.StartProgram(f, identity)
	if err == nil || err.Error() != ReasonProgramIdentityMismatch || native.prepares.Load() != 1 {
		t.Fatalf("identity refusal: %v", err)
	}
}

// Positive, negative, exact-bound and absent-bound cases fix the derivative
// contract in absolute rad/s^2; explicit remapping must identify the native axis.
func TestProgramAccelerationDeclaredBounds(t *testing.T) {
	for _, test := range []struct {
		name     string
		velocity float64
		limit    *float64
		reject   bool
	}{
		{"positive", .02, floatPointer(1), true}, {"negative", -.02, floatPointer(1), true},
		{"at_bound", .01, floatPointer(1), false}, {"below_bound", .005, floatPointer(1), false},
		{"absent", .02, nil, false}, {"invalid_bound", .005, floatPointer(math.NaN()), true},
	} {
		t.Run(test.name, func(t *testing.T) {
			plan := seamProgramTest(t, .01)
			plan.Segments[1].Samples[1].QdRadS[0] = test.velocity
			description, ids := programDescription()
			ids[0], ids[1] = ids[1], ids[0]
			description.Axes[1].MaxAcceleration = test.limit
			_, err := DecodeProgram(encodeProgramTest(t, plan), description, ids)
			if test.reject {
				segment := 1
				if test.name == "invalid_bound" {
					segment = 0
				}
				if err == nil || err.Error() != fmt.Sprintf("%s: segment=%d sample=1 axis=J2", ReasonNativeLimitExceeded, segment) {
					t.Fatalf("derivative refusal: %v", err)
				}
			} else if err != nil {
				t.Fatal(err)
			}
		})
	}
}

func floatPointer(value float64) *float64 { return &value }

// Clock conversion failures keep a catalogue label instead of leaking the IPC
// client's free-text point error after nanosecond rounding or signed overflow.
func TestProgramNativeClockReasons(t *testing.T) {
	description, ids := programDescription()
	for _, test := range []struct {
		dt              float64
		segment, sample int
	}{
		{1e-12, 0, 1}, {1e10, 0, 1}, {3e9, 1, 2},
	} {
		_, err := DecodeProgram(encodeProgramTest(t, seamProgramTest(t, test.dt)), description, ids)
		want := fmt.Sprintf("%s: segment=%d sample=%d", ReasonTimeGridInvalid, test.segment, test.sample)
		if err == nil || err.Error() != want {
			t.Fatalf("dt=%g: %v, want %s", test.dt, err, want)
		}
	}
}

func TestProgramEncoderDiagnosticReasons(t *testing.T) {
	labels := programReasonLabels(t)
	for _, header := range []densejointtraj.Header{
		{Segments: []densejointtraj.SegmentMeta{{}}},
		{Sampling: densejointtraj.Sampling{DtS: math.Inf(1)}},
	} {
		_, err := densejointtraj.Encode(header, nil)
		if err == nil {
			t.Fatal("invalid encoding accepted")
		}
		label, _, _ := strings.Cut(err.Error(), ":")
		if !labels[label] {
			t.Fatalf("uncatalogued encoder error: %v", err)
		}
	}
}

type programRejectedNative struct {
	*admissionNative
	reason uint32
}

func (n *programRejectedNative) PrepareTrajectory(context.Context, uint64, uint32, []ipcclient.Point) (uint64, error) {
	return 0, &ipcclient.RejectedError{Result: ipcclient.CommandResult{Reason: n.reason}}
}

func TestProgramNativeCapacityClassification(t *testing.T) {
	for _, reason := range []uint32{ipcclient.ReasonCapacity, ipcclient.ReasonNotReady} {
		c, f, native := admissionController()
		c.description, c.axes = programDescription()
		c.binding.Revision = 7
		c.core = &programRejectedNative{native, reason}
		_, err := c.PrepareProgram(f, stationaryProgram(t, "native-reason"))
		var rejected *ipcclient.RejectedError
		if !errors.As(err, &rejected) || rejected.Result.Reason != reason {
			t.Fatalf("native detail lost: %v", err)
		}
		outcome := encodeResponse(Response{Schema: ResponseSchema}, err)
		var response Response
		if err := json.Unmarshal(outcome.body, &response); err != nil {
			t.Fatal(err)
		}
		want := ReasonNativeRejected
		if reason == ipcclient.ReasonCapacity {
			want = ReasonBusy
		}
		label, _, _ := strings.Cut(response.Error, ":")
		if label != want || response.NativeResult == nil || response.NativeResult.Reason != reason {
			t.Fatalf("classification: %+v", response)
		}
	}
}

// Both bounded upload admission and native storage exhaustion expose busy;
// only an actual native refusal carries a native receipt for consumer recovery.
func TestProgramBusyRefusalSources(t *testing.T) {
	for _, source := range []string{"preparation_slot", "native_capacity"} {
		t.Run(source, func(t *testing.T) {
			c, f, native := admissionController()
			c.description, c.axes = programDescription()
			c.binding.Revision = 7
			if source == "preparation_slot" {
				release, err := c.admitPreparation(f, true)
				if err != nil {
					t.Fatal(err)
				}
				defer release()
			} else {
				c.core = &programRejectedNative{native, ipcclient.ReasonCapacity}
			}
			status, response := programHTTPResponse(t, c, f, stationaryProgram(t, source))
			label, _, _ := strings.Cut(response.Error, ":")
			if status != http.StatusConflict || label != ReasonBusy {
				t.Fatalf("%s: HTTP %d %+v", source, status, response)
			}
			if source == "preparation_slot" {
				if response.NativeResult != nil || native.prepares.Load() != 0 {
					t.Fatalf("slot refusal reached native: %+v", response)
				}
			} else if response.NativeResult == nil || response.NativeResult.Reason != ipcclient.ReasonCapacity {
				t.Fatalf("capacity receipt lost: %+v", response)
			}
		})
	}
}
