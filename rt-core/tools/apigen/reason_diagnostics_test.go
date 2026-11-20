package main

import (
	"go/ast"
	"go/token"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// TLS callback errors are rendered by net/http's handshake logger, including
// reload failures; excluding them would contradict APPLICATION_API.md's sinks.
func documentedTLSCallback(name string) bool {
	switch name {
	case "VerifyPeerCertificate", "VerifyConnection", "GetConfigForClient", "GetCertificate":
		return true
	}
	return false
}

func tlsCallbackErrorSlot(name string) int {
	if name == "GetConfigForClient" || name == "GetCertificate" {
		return 1
	}
	return 0
}

// A branch is excluded only if every known caller argument makes its predicate
// false. Unknown values or predicates retain the branch to fail closed.
func reasonConditionAlwaysFalse(condition ast.Expr, stringsFor func(ast.Expr, map[ast.Expr]bool) []string, objectFor func(ast.Expr) *ast.Object) bool {
	inputs := map[*ast.Object][]string{}
	ast.Inspect(condition, func(node ast.Node) bool {
		if expression, ok := node.(*ast.Ident); ok {
			if object := objectFor(expression); object != nil && object.Kind == ast.Var {
				inputs[object] = stringsFor(expression, map[ast.Expr]bool{})
			}
		}
		return true
	})
	var variables []*ast.Object
	for object, values := range inputs {
		if len(values) == 0 {
			return false
		}
		variables = append(variables, object)
	}
	if len(variables) == 0 {
		return false
	}
	assignment := map[*ast.Object]string{}
	stringValue := func(expression ast.Expr) (string, bool) {
		if text, ok := expression.(*ast.BasicLit); ok && text.Kind == token.STRING {
			value, err := strconv.Unquote(text.Value)
			return value, err == nil
		}
		value, ok := assignment[objectFor(expression)]
		return value, ok
	}
	var evaluate func(ast.Expr) (bool, bool)
	evaluate = func(expression ast.Expr) (bool, bool) {
		if parentheses, ok := expression.(*ast.ParenExpr); ok {
			return evaluate(parentheses.X)
		}
		binary, ok := expression.(*ast.BinaryExpr)
		if !ok {
			return false, false
		}
		if binary.Op == token.LAND || binary.Op == token.LOR {
			left, leftKnown := evaluate(binary.X)
			right, rightKnown := evaluate(binary.Y)
			if binary.Op == token.LAND {
				return left && right, leftKnown && rightKnown
			}
			return left || right, leftKnown && rightKnown
		}
		left, leftKnown := stringValue(binary.X)
		right, rightKnown := stringValue(binary.Y)
		if binary.Op == token.EQL {
			return left == right, leftKnown && rightKnown
		}
		return left != right, binary.Op == token.NEQ && leftKnown && rightKnown
	}
	var allFalse func(int) bool
	allFalse = func(index int) bool {
		if index == len(variables) {
			value, known := evaluate(condition)
			return known && !value
		}
		object := variables[index]
		for _, value := range inputs[object] {
			assignment[object] = value
			if !allFalse(index + 1) {
				return false
			}
		}
		return true
	}
	return allFalse(0)
}

// Each documented startup/TLS occurrence must survive the inventory, including
// repeated labels; a set of label names alone would hide a dropped public path.
func TestAdapterReasonInventoryDocumentedDiagnostics(t *testing.T) {
	emitted, err := adapterReasonLiterals("../../adapters/rosie/control")
	if err != nil {
		t.Fatal(err)
	}
	want := map[string][]int{
		"remote_tls.go":   {28, 33, 38, 44, 54, 59, 68, 75, 106, 113, 117, 123, 137, 145},
		"socket_owner.go": {19, 59, 72, 84, 88, 118, 149},
	}
	count := 0
	for file, lines := range want {
		for _, line := range lines {
			found := false
			for _, reason := range emitted {
				if filepath.Base(reason.position.Filename) == file && reason.position.Line == line {
					found = true
					count++
				}
			}
			if !found {
				t.Errorf("documented diagnostic missing at %s:%d", file, line)
			}
		}
	}
	// The 21st occurrence is the documented CRL-signature wrapper at line 113.
	if count != 21 {
		t.Fatalf("documented diagnostic occurrences = %d, want 21", count)
	}
}

// Startup and handshake mutations must fail the actual check-api completeness
// test with their source and value, even when no HTTP response carries them.
func TestAdapterReasonInventoryDiagnosticMutations(t *testing.T) {
	for _, label := range []string{"remote pair-id required", "remote client revoked", "socket lock path changed"} {
		t.Run(label, func(t *testing.T) {
			fixture := productionReasonFixture(t)
			path := "adapters/rosie/control/remote_tls.go"
			if strings.HasPrefix(label, "socket") {
				path = "adapters/rosie/control/socket_owner.go"
			}
			if strings.Count(fixture[path], strconv.Quote(label)) != 1 {
				t.Fatal("diagnostic mutation fixture drift")
			}
			fixture[path] = strings.Replace(fixture[path], strconv.Quote(label), `"review6_startup_diagnostic"`, 1)
			output, err := reasonFixtureGate(t, fixture)
			if err == nil || !strings.Contains(output, filepath.Base(path)+":") || !strings.Contains(output, `undocumented reason "review6_startup_diagnostic"`) {
				t.Fatalf("diagnostic escaped check-api: %v\n%s", err, output)
			}
		})
	}
}

// The three updateJog strings are converted to typed ingress reasons by the
// production caller. Expose that same error and check-api must reject each one.
func TestAdapterReasonInventoryPrivateJogConversions(t *testing.T) {
	for _, label := range []string{"independent_jog_unavailable", "jog_publisher_busy", "jog_session_or_sequence_stale"} {
		t.Run(label, func(t *testing.T) {
			fixture := productionReasonFixture(t)
			path := "adapters/rosie/control/jog_linux.go"
			start := strings.Index(fixture[path], "func (c *Controller) updateJog(")
			end := strings.Index(fixture[path][start:], "// currentJogSessionLocked") + start
			body := fixture[path][start:end]
			if strings.Count(body, strconv.Quote(label)) != 1 {
				t.Fatal("private jog fixture drift")
			}
			fixture[path] = fixture[path][:start] + strings.Replace(body, strconv.Quote(label), `"review7_private_jog"`, 1) + fixture[path][end:]
			output, err := reasonFixtureGate(t, fixture)
			if err != nil {
				t.Fatalf("converted private jog string became public: %v\n%s", err, output)
			}
			conversion := "reason = jogIngressReason(err)"
			if strings.Count(fixture[path], conversion) != 1 {
				t.Fatal("production ingress must convert updateJog's error")
			}
			fixture[path] = strings.Replace(fixture[path], conversion, `_ = Response{Error: err.Error()}; `+conversion, 1)
			output, err = reasonFixtureGate(t, fixture)
			if err == nil || !strings.Contains(output, "jog_linux.go:") || !strings.Contains(output, `undocumented reason "review7_private_jog"`) {
				t.Fatalf("unconverted jog string escaped check-api: %v\n%s", err, output)
			}
		})
	}
}

// The unsupported transport branch is private only while all production calls
// supply unix or unixgram. A new transport must restore it to the inventory.
func TestAdapterReasonInventoryPrivateSocketTransport(t *testing.T) {
	fixture := productionReasonFixture(t)
	path := "adapters/rosie/control/socket_owner.go"
	label := `"unsupported socket ownership transport"`
	if strings.Count(fixture[path], label) != 1 {
		t.Fatal("private transport fixture drift")
	}
	fixture[path] = strings.Replace(fixture[path], label, `"review7_private_transport"`, 1)
	output, err := reasonFixtureGate(t, fixture)
	if err != nil {
		t.Fatalf("unused transport diagnostic became public: %v\n%s", err, output)
	}
	path = "cmd/rt-control/main.go"
	caller := `control.ClaimSocket(jogSocket, "unixgram")`
	if strings.Count(fixture[path], caller) != 1 {
		t.Fatal("production transport caller drift")
	}
	fixture[path] = strings.Replace(fixture[path], caller, `control.ClaimSocket(jogSocket, "tcp")`, 1)
	output, err = reasonFixtureGate(t, fixture)
	if err == nil || !strings.Contains(output, "socket_owner.go:") || !strings.Contains(output, `undocumented reason "review7_private_transport"`) {
		t.Fatalf("new transport diagnostic escaped check-api: %v\n%s", err, output)
	}
}

// The schema and generated document explicitly include startup/TLS diagnostics;
// the CRL wrapper is accepted only with the documented entire format.
func TestAdapterReasonInventoryDocumentedSinkContract(t *testing.T) {
	body, err := os.ReadFile("../../docs/APPLICATION_API.md")
	if err != nil || !strings.Contains(string(body), "including startup/TLS failures") || !strings.Contains(string(body), "remote CRL signature: <cause>") {
		t.Fatalf("documented diagnostic sinks drifted: %v", err)
	}
	for _, format := range []string{"remote CRL signature: %w", "review7_CRL_signature: %w", "remote CRL signature: changed %w"} {
		output, err := reasonInventoryGate(t, `package control
import "fmt"
func diagnostic(err error) Response { return Response{Error: fmt.Errorf(`+strconv.Quote(format)+`, err).Error()} }
`)
		if format == "remote CRL signature: %w" {
			if err != nil {
				t.Fatalf("documented CRL diagnostic rejected: %v\n%s", err, output)
			}
		} else if err == nil || !strings.Contains(output, "undocumented reason") {
			t.Fatalf("changed CRL diagnostic escaped check-api: %v\n%s", err, output)
		}
	}
}
