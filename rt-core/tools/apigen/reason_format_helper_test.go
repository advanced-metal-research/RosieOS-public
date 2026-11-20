package main

import (
	"os/exec"
	"strings"
	"testing"
)

// The review's cross-package constant reaches Response.Error through a helper
// parameter and format concatenation; fixtures retain its declaration location.
func formatHelperFixture() map[string]string {
	return map[string]string{
		"adapters/rosie/control/review6sub/constants.go": `package review6sub
// unverified review fixture.
const Prefix = "review6_cross_format_helper"
`,
		"adapters/rosie/control/review6_probe.go": `package control
import "fmt"
import "rosieos/rt-core/adapters/rosie/control/review6sub"
type Response struct { Error string }
func review6Helper(prefix, detail string) string { return fmt.Sprintf(prefix + ": %s", detail) }
func review6Public() Response { return Response{Error: review6Helper(review6sub.Prefix, "details")} }
`,
	}
}

func requireFormatHelperRejected(t *testing.T, fixture map[string]string) {
	t.Helper()
	output, err := reasonFixtureGate(t, fixture)
	for _, expected := range []string{"review6sub/constants.go:3:", `undocumented reason "review6_cross_format_helper"`, `constant "Prefix"`} {
		if err == nil || !strings.Contains(output, expected) {
			t.Fatalf("format helper escaped check-api (%s): %v\n%s", expected, err, output)
		}
	}
}

// format_helper.sh stays a permanent negative, including the equivalent
// assignment rewrite, so moving the format through a local cannot hide it.
func TestAdapterReasonInventoryFormatHelperNegative(t *testing.T) {
	for _, assigned := range []bool{false, true} {
		fixture := formatHelperFixture()
		if assigned {
			path := "adapters/rosie/control/review6_probe.go"
			fixture[path] = strings.Replace(fixture[path], `return fmt.Sprintf(prefix + ": %s", detail)`, `format := prefix + ": %s"; return fmt.Sprintf(format, detail)`, 1)
		}
		requireFormatHelperRejected(t, fixture)
	}
}

// format_helper_runtime.sh must first demonstrate the publicly emitted string
// in compiled Go, then fail the same check-api gate on that exact source tree.
func TestAdapterReasonInventoryFormatHelperRuntime(t *testing.T) {
	fixture := formatHelperFixture()
	fixture["go.mod"] = "module rosieos/rt-core\n\ngo 1.24\n"
	fixture["adapters/rosie/control/review6_runtime_test.go"] = `package control
import "testing"
// Formatting must preserve the constant in the public response error.
func TestReview6FormatHelperPublic(t *testing.T) {
 response := review6Public()
 if response.Error != "review6_cross_format_helper: details" { t.Fatal(response.Error) }
}
`
	root := t.TempDir()
	writeReasonFixture(t, root, fixture)
	command := exec.Command("go", "test", "-race", "-count=1", "./adapters/rosie/control", "-run", "^TestReview6FormatHelperPublic$")
	command.Dir = root
	if output, err := command.CombinedOutput(); err != nil {
		t.Fatalf("runtime format helper: %v\n%s", err, output)
	}
	requireFormatHelperRejected(t, fixture)
}
