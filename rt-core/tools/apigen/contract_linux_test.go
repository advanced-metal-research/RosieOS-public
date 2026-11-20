package main

import (
	"bytes"
	"context"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"go/ast"
	"go/parser"
	"go/token"
	"io"
	"math"
	"net/http"
	"net/http/httptest"
	"os"
	"os/exec"
	"reflect"
	"regexp"
	"rosieos/rt-core/internal/controltest"
	"strconv"
	"strings"
	"testing"

	adapter "rosieos/rt-core/adapters/rosie/control"
	"rosieos/rt-core/ipcclient"
)

// TestEveryEmittedReasonIsInTheSchema includes production subpackages and
// declared reason constants, preventing stable refusals from escaping the catalogue.
func TestEveryEmittedReasonIsInTheSchema(t *testing.T) {
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	reasons := map[string]bool{}
	for _, reason := range s.Reasons {
		reasons[reason.Name] = true
	}
	var rules struct {
		ErrorPatterns []string `json:"error_patterns"`
	}
	if err := json.Unmarshal(s.Rules, &rules); err != nil {
		t.Fatal(err)
	}
	emitted, err := adapterReasonLiterals("../../adapters/rosie/control")
	if err != nil || len(emitted) == 0 {
		t.Fatalf("adapter reason inventory: %v (%d literals)", err, len(emitted))
	}
	for _, reason := range emitted {
		// A wrapped cause is an explicitly documented open diagnostic only
		// when its entire format matches the schema, never just its prefix.
		documentedPattern := false
		for _, pattern := range rules.ErrorPatterns {
			if strings.Contains(reason.format, "%w") && strings.ReplaceAll(reason.format, "%w", "<cause>") == pattern {
				documentedPattern = true
			}
		}
		if !reasons[reason.name] && !documentedPattern {
			t.Errorf("%s emits undocumented reason %q (constant %q)", reason.position, reason.name, reason.constant)
		}
	}
}

// TestEveryEmittedBusDispositionAndReasonIsInTheSchema retains review2-contract:
// native topology strings must be documented even though bus remains raw JSON.
func TestEveryEmittedBusDispositionAndReasonIsInTheSchema(t *testing.T) {
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	var constraints string
	for _, typ := range s.Types {
		if typ.Name != "NativeStatus" {
			continue
		}
		for _, field := range typ.Fields {
			if field.JSON == "bus" {
				if field.Type != "rawjson" || !field.Required {
					t.Fatalf("bus changed its opaque publication contract: %+v", field)
				}
				constraints = field.Constraints
			}
		}
	}
	if constraints == "" {
		t.Fatal("NativeStatus.bus contract absent")
	}
	doc, err := os.ReadFile("../../docs/APPLICATION_API.md")
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(doc), "| `bus` | object / rawjson | true | false | "+constraints+" |") {
		t.Error("documented bus row differs from schema prose")
	}
	source, err := os.ReadFile("../../include/bus_topology.hpp")
	if err != nil {
		t.Fatal(err)
	}
	emissions := regexp.MustCompile(`case (BusSlaveDisposition|BusAdmissionReason)::([A-Za-z]+): return "([a-z_]+)";`).FindAllSubmatch(source, -1)
	if len(emissions) < 16 {
		t.Fatalf("native emission inventory incomplete: %d, want at least 9 dispositions and 7 reasons", len(emissions))
	}
	for _, entry := range emissions {
		t.Run(string(entry[1])+"/"+string(entry[2]), func(t *testing.T) {
			name := string(entry[3])
			if !regexp.MustCompile(`\b` + regexp.QuoteMeta(name) + `\b`).MatchString(constraints) {
				t.Errorf("native topology emits %q absent from NativeStatus.bus schema prose", name)
			}
		})
	}
}

func TestUnimplementedCapabilityIsRejectedExplicitly(t *testing.T) {
	server := httptest.NewServer(adapter.NewHandler(&adapter.Controller{}))
	defer server.Close()
	for _, operation := range []string{"abort", "readiness", "not_a_contract_operation"} {
		t.Run(operation, func(t *testing.T) {
			request := adapter.Request{Schema: adapter.RequestSchema, Operation: operation}
			body, err := json.Marshal(request)
			if err != nil {
				t.Fatal(err)
			}
			var result adapter.RawResponse
			code, err := (controltest.Client{HTTP: server.Client(), Base: server.URL}).Decode(context.Background(), http.MethodPost, "/v1/control", body, http.Header{"Content-Type": {"application/json"}}, &result)
			if err != nil {
				t.Fatal(err)
			}
			if code != http.StatusConflict {
				t.Fatalf("HTTP %d", code)
			}
			if operation == "not_a_contract_operation" {
				if result.Error != "unknown_operation" {
					t.Fatalf("unknown error = %q", result.Error)
				}
				return
			}
			if result.Error != "capability_unimplemented" || result.Operation != operation {
				t.Fatalf("response: %+v", result)
			}
			var capability adapter.CapabilityInfo
			if err := json.Unmarshal(result.Data, &capability); err != nil {
				t.Fatal(err)
			}
			if capability.Name != operation || capability.State != "unimplemented" {
				t.Fatalf("capability: %+v", capability)
			}
		})
	}
}

func TestDescribeReportsContractVersionAndDigest(t *testing.T) {
	server := httptest.NewServer(adapter.NewHandler(&adapter.Controller{}))
	defer server.Close()
	var envelope adapter.RawResponse
	code, err := (controltest.Client{HTTP: server.Client(), Base: server.URL}).Decode(context.Background(), http.MethodGet, "/v1/describe", nil, nil, &envelope)
	if err != nil {
		t.Fatal(err)
	}
	var description adapter.Description
	if err := json.Unmarshal(envelope.Data, &description); err != nil {
		t.Fatal(err)
	}
	if code != 200 || envelope.Schema != "rosie.rt-control.response.v1" || envelope.Operation != "describe" || description.ContractVersion != 2 {
		t.Fatalf("describe: %+v %+v", envelope, description)
	}
	want := fmt.Sprintf("%x", sha256.Sum256(schemaBytes(t)))
	if description.CapabilitiesDigest != want {
		t.Fatalf("digest %q, want %q", description.CapabilitiesDigest, want)
	}
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	if len(description.Capabilities) != len(s.Capabilities) {
		t.Fatal("incomplete capabilities")
	}
	for i, capability := range description.Capabilities {
		want := s.Capabilities[i]
		if capability.Name != want.Name || capability.State != want.State || capability.Transport != want.Transport {
			t.Fatalf("capability mismatch: %+v", capability)
		}
		lookup, ok := adapter.Capability(capability.Name)
		if !ok || !adapter.KnownOperation(capability.Name) || lookup != capability {
			t.Fatal("lookup differs from describe")
		}
	}
	if _, ok := adapter.Capability("not_a_contract_operation"); ok {
		t.Fatal("unknown capability accepted")
	}
	copy := adapter.Capabilities()
	copy[0].Name = "mutated"
	if adapter.Capabilities()[0].Name == "mutated" {
		t.Fatal("caller modified inventory")
	}
}

// Verify externally owned Go aliases against schema fields as well as generated
// declarations; otherwise a native or retained adapter type could drift unnoticed.
func reflectedWireTypes() map[string]reflect.Type {
	values := []any{adapter.EventBatch{}, adapter.Request{}, adapter.Response{}, adapter.RawResponse{}, adapter.Fence{}, adapter.Binding{}, adapter.Grant{}, adapter.Execution{}, adapter.HandleRecord{}, adapter.ProcessStatus{}, adapter.Description{}, adapter.CapabilityInfo{}, adapter.Program{}, adapter.JogIngressObservation{}, adapter.LocalJogClock{}, ipcclient.Status{}}
	values = append(values, adapter.ProgramLimitData{}, adapter.VerifiedDeltaRestoreReceipt{}, adapter.PathJogStatus{})
	values = append(values, adapter.MaintenanceObservation{})
	types := map[string]reflect.Type{}
	types["TelemetryBatch"] = reflect.TypeOf(adapter.TelemetryBatch{})
	var visit func(reflect.Type)
	visit = func(typ reflect.Type) {
		for typ.Kind() == reflect.Pointer || typ.Kind() == reflect.Slice || typ.Kind() == reflect.Array || typ.Kind() == reflect.Map {
			typ = typ.Elem()
		}
		if typ.Kind() != reflect.Struct {
			return
		}
		name := typ.Name()
		if strings.HasSuffix(typ.PkgPath(), "ipcclient") && (name == "Status" || name == "Description") {
			name = "Native" + name
		}
		if types[name] != nil {
			return
		}
		types[name] = typ
		for i := 0; i < typ.NumField(); i++ {
			if typ.Field(i).IsExported() && typ.Field(i).Tag.Get("json") != "-" {
				visit(typ.Field(i).Type)
			}
		}
	}
	for _, value := range values {
		visit(reflect.TypeOf(value))
	}
	return types
}

func TestSchemaMatchesGoWireTypes(t *testing.T) {
	types := reflectedWireTypes()
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	declared := map[string]bool{}
	for _, spec := range s.Types {
		declared[spec.Name] = true
	}
	for name := range types {
		if !declared[name] {
			t.Errorf("Go wire type absent from schema: %s", name)
		}
	}
	for _, spec := range s.Types {
		typ := types[spec.Name]
		if typ == nil {
			t.Errorf("missing Go wire type %s", spec.Name)
			continue
		}
		exported := 0
		for i := 0; i < typ.NumField(); i++ {
			if typ.Field(i).IsExported() && typ.Field(i).Tag.Get("json") != "-" {
				exported++
			}
		}
		if exported != len(spec.Fields) {
			t.Errorf("field count for %s: %d != %d", spec.Name, exported, len(spec.Fields))
		}
		for _, field := range spec.Fields {
			name := field.Name
			if field.Embedded && spec.Name == "Description" {
				name = "NativeDescription"
			}
			actual, ok := typ.FieldByName(name)
			if !ok {
				t.Errorf("missing %s.%s", spec.Name, name)
				continue
			}
			if field.Embedded {
				continue
			}
			tag := strings.Split(actual.Tag.Get("json"), ",")
			if tag[0] == "" {
				tag[0] = actual.Name
			}
			if tag[0] != field.JSON || (len(tag) > 1 && tag[1] == "omitempty") != field.OmitEmpty {
				t.Errorf("JSON tag drift %s.%s", spec.Name, name)
			}
			if actualType := wireType(actual.Type); actualType != field.Type {
				t.Errorf("type drift %s.%s: %s != %s", spec.Name, name, actualType, field.Type)
			}
		}
	}
}

func TestRetiredCoordinateFieldsAreAbsentFromGoWire(t *testing.T) {
	values := []any{
		adapter.Description{NativeDescription: ipcclient.Description{Axes: []ipcclient.AxisDescription{{}}}},
		adapter.ProcessStatus{Core: ipcclient.Status{Axes: []ipcclient.AxisStatus{{}}}},
	}
	for _, value := range values {
		body, err := json.Marshal(value)
		if err != nil {
			t.Fatal(err)
		}
		for _, field := range []string{"feedback_wrap", "command_wrap", "anchor_period_counts"} {
			if bytes.Contains(body, []byte(field)) {
				t.Errorf("%T still publishes %s", value, field)
			}
		}
	}
}

func wireType(typ reflect.Type) string {
	if typ == reflect.TypeOf(json.RawMessage{}) {
		return "rawjson"
	}
	switch typ.Kind() {
	case reflect.Pointer:
		return "*" + wireType(typ.Elem())
	case reflect.Slice:
		return "[]" + wireType(typ.Elem())
	case reflect.Array:
		return fmt.Sprintf("[%d]%s", typ.Len(), wireType(typ.Elem()))
	case reflect.Map:
		return "map[" + wireType(typ.Key()) + "]" + wireType(typ.Elem())
	case reflect.Struct:
		name := typ.Name()
		if strings.HasSuffix(typ.PkgPath(), "ipcclient") && (name == "Status" || name == "Description") {
			return "Native" + name
		}
		return name
	case reflect.Interface:
		return "json"
	default:
		if typ.Name() == "HandleState" {
			return "HandleState"
		}
		return typ.Kind().String()
	}
}

func fillWire(value reflect.Value) {
	if value.Type() == reflect.TypeOf(json.RawMessage{}) {
		value.SetBytes([]byte(`{"receipt":18446744073709551615}`))
		return
	}
	switch value.Kind() {
	case reflect.Struct:
		for i := 0; i < value.NumField(); i++ {
			if value.Field(i).CanSet() {
				fillWire(value.Field(i))
			}
		}
	case reflect.Pointer:
		value.Set(reflect.New(value.Type().Elem()))
		fillWire(value.Elem())
	case reflect.Slice:
		value.Set(reflect.MakeSlice(value.Type(), 2, 2))
		for i := 0; i < 2; i++ {
			fillWire(value.Index(i))
		}
	case reflect.Map:
		value.Set(reflect.MakeMap(value.Type()))
		key := reflect.New(value.Type().Key()).Elem()
		item := reflect.New(value.Type().Elem()).Elem()
		fillWire(key)
		fillWire(item)
		value.SetMapIndex(key, item)
	case reflect.Array:
		for i := 0; i < value.Len(); i++ {
			fillWire(value.Index(i))
		}
	case reflect.String:
		value.SetString("wire\"\\\n\u03bb\U0001f680")
	case reflect.Bool:
		value.SetBool(true)
	case reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64:
		value.SetUint(math.MaxUint64 >> (64 - value.Type().Bits()))
	case reflect.Int, reflect.Int32, reflect.Int64:
		value.SetInt(-1 << (value.Type().Bits() - 1))
	case reflect.Float64:
		value.SetFloat(1.2345678901234567)
	case reflect.Interface:
		value.Set(reflect.ValueOf(map[string]uint64{"receipt": math.MaxUint64}))
	}
}

// Populated Go fixtures are independent of the C++ generator. Comparing parsed
// JSON with UseNumber catches casing, widths, arrays, Unicode and nested data drift.
func TestEveryGeneratedTypeInteroperatesWithCPP(t *testing.T) {
	command := exec.Command("make", "build/rt-control-api-tests")
	command.Dir = "../.."
	if output, err := command.CombinedOutput(); err != nil {
		t.Fatalf("C++ fixture build: %v\n%s", err, output)
	}
	values := []any{adapter.Fence{}, adapter.Binding{}, adapter.Point{}, adapter.Identity{}, adapter.Request{}, adapter.Response{}, adapter.RawResponse{}, adapter.Grant{}, adapter.HandleRecord{}, adapter.Execution{}, adapter.ProcessStatus{}, adapter.JogObservation{}, adapter.CommandResult{}, adapter.GrantObservation{}, adapter.FaultRecovery{}, adapter.RecoveryStatus{}, adapter.AxisStatus{}, adapter.NativeStatus{}, adapter.MotionState{}, adapter.ProcessMarker{}, adapter.Program{}, adapter.JogIngressObservation{}, adapter.AxisDescription{}, adapter.NativeDescription{}, adapter.LocalJogClock{}, adapter.Description{}, adapter.CapabilityInfo{}}
	values = append(values, adapter.LogicalAxisStatus{}, adapter.StatusGenerations{}, adapter.PlanCursor{}, adapter.BufferHealth{}, adapter.AdapterStatus{}, adapter.GrantEvent{}, adapter.HandleTransitionEvent{}, adapter.ExecutionEvent{}, adapter.JogEvent{}, adapter.FaultEvent{}, adapter.EnableEvent{}, adapter.EpochEvent{}, adapter.IncarnationEvent{}, adapter.PublisherOverflowEvent{}, adapter.EventsDropped{}, adapter.Event{}, adapter.EventBatch{})
	values = append(values, adapter.TelemetryMarkEvent{}, adapter.JogIngressRefusal{})
	values = append(values, adapter.CollisionWatchdogStatus{}, adapter.BrakeHoldDiagnostic{}, adapter.BrakeWarningEvent{})
	values = append(values, adapter.DriveSupervisorDiagnostic{}, adapter.DriveSupervisorEvent{})
	values = append(values, adapter.IoInput{}, adapter.IoOutput{}, adapter.IoState{})
	values = append(values, adapter.ResourceInfo{}, adapter.RobotDescription{}, adapter.DriveIdentity{}, adapter.DriveDescription{})
	values = append(values, adapter.ProgramLimitData{}, adapter.ProgramLimitViolation{})
	values = append(values, adapter.IndependentAnchorSource{}, adapter.ReferenceVerifiedDeltaRestorePolicy{}, adapter.AnchorDeltaRestoreStatus{}, adapter.VerifiedDeltaRestoreReceipt{})
	values = append(values, adapter.PathJogCapability{}, adapter.PathObservation{}, adapter.PathJogStatus{})
	values = append(values, adapter.MaintenanceObservation{}, adapter.MaintenanceHolder{})
	values = append(values, adapter.AxisLoad{}, adapter.AxisMonitor{})
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	binaryTypes := 0
	for _, typ := range s.Types {
		if typ.Binary {
			binaryTypes++
		}
	}
	if len(values)+binaryTypes != len(s.Types) {
		t.Fatalf("interop fixtures cover %d types; schema has %d", len(values), len(s.Types))
	}
	for _, sample := range values {
		name := wireType(reflect.TypeOf(sample))
		t.Run(name, func(t *testing.T) {
			value := reflect.New(reflect.TypeOf(sample)).Elem()
			fillWire(value)
			if name == "Request" {
				request := value.Addr().Interface().(*adapter.Request)
				id := "request-1"
				request.RequestID = &id
				request.Schema = adapter.RequestSchema
				request.Operation = adapter.OperationPrepareTrajectory
			}
			body, err := json.Marshal(value.Interface())
			if err != nil {
				t.Fatal(err)
			}
			command := exec.Command("../../build/rt-control-api-tests", name)
			command.Stdin = bytes.NewReader(body)
			output, err := command.CombinedOutput()
			if err != nil {
				t.Fatalf("C++ decode/encode: %v\n%s", err, output)
			}
			decode := func(body []byte) any {
				var result any
				decoder := json.NewDecoder(bytes.NewReader(body))
				decoder.UseNumber()
				if err := decoder.Decode(&result); err != nil {
					t.Fatal(err)
				}
				return result
			}
			if !reflect.DeepEqual(decode(body), decode(output)) {
				t.Fatalf("wire mismatch\nGo: %s\nC++: %s", body, output)
			}
		})
	}
}

// Dispatch and GET operation inventory is independently extracted from source;
// every route must be described even when its implementation uses a constant.
func TestSchemaCoversHandlerOperations(t *testing.T) {
	s, err := parse(schemaBytes(t))
	if err != nil {
		t.Fatal(err)
	}
	names := map[string]bool{}
	for _, capability := range s.Capabilities {
		if capability.Transport == "POST /v1/control" {
			names["Operation"+identifier(capability.Name)] = false
		}
	}
	path := "../../adapters/rosie/control/http.go"
	file, err := parser.ParseFile(token.NewFileSet(), path, nil, 0)
	if err != nil {
		t.Fatal(err)
	}
	ast.Inspect(file, func(node ast.Node) bool {
		if clause, ok := node.(*ast.CaseClause); ok {
			for _, expression := range clause.List {
				if name, ok := expression.(*ast.Ident); ok && strings.HasPrefix(name.Name, "Operation") {
					if _, declared := names[name.Name]; !declared {
						t.Errorf("dispatch operation absent from control schema: %s", name.Name)
					}
					names[name.Name] = true
				}
			}
		}
		literal, ok := node.(*ast.BasicLit)
		if !ok || literal.Kind != token.STRING {
			return true
		}
		value, _ := strconv.Unquote(literal.Value)
		if strings.HasPrefix(value, "/v1/") && value != "/v1/control" {
			found := false
			for _, capability := range s.Capabilities {
				for _, endpoint := range strings.Split(capability.Transport, "; ") {
					fields := strings.Fields(endpoint)
					if len(fields) >= 2 && strings.TrimSuffix(strings.SplitN(fields[1], "?", 2)[0], "<sha256>") == value {
						found = true
					}
				}
			}
			if !found {
				t.Errorf("endpoint absent from schema: %s", value)
			}
		}
		return true
	})
	for name, found := range names {
		if !found {
			t.Errorf("schema operation absent from dispatch: %s", name)
		}
	}
	// Exercise the real decoder's ID rules without changing the frozen oracle.
	for _, id := range []string{"null", "0", `""`, `"\u001f"`, `"` + strings.Repeat("a", 65) + `"`} {
		server := httptest.NewServer(adapter.NewHandler(&adapter.Controller{}))
		body := `{"schema":"rosie.rt-control.request.v1","operation":"halt","request_id":` + id + `}`
		response, err := server.Client().Post(server.URL+"/v1/control", "application/json", strings.NewReader(body))
		if err != nil {
			server.Close()
			t.Fatal(err)
		}
		data, err := io.ReadAll(response.Body)
		response.Body.Close()
		server.Close()
		if err != nil || !bytes.Contains(data, []byte(`"error":"invalid_request_id"`)) {
			t.Errorf("ID %s: %s %v", id, data, err)
		}
	}
}
