package main

import (
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"regexp"
	"strconv"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
)

// TestTypeScriptTelemetryInstructions binds framing instructions to the exported
// byte-size constant, preventing an obsolete literal from truncating records.
func TestTypeScriptTelemetryInstructions(t *testing.T) {
	readme, err := os.ReadFile("../../clients/ts/README.md")
	if err != nil {
		t.Fatal(err)
	}
	_, instructions, ok := strings.Cut(string(readme), "For telemetry,")
	if !ok || strings.Count(instructions, "`CycleCaptureRecordV2Bytes`") != 2 ||
		regexp.MustCompile(`record size [0-9]|[0-9]+-byte\s+record`).MatchString(instructions) {
		t.Fatalf("telemetry instructions must use the generated record size: %s", instructions)
	}
	generated, err := os.ReadFile("../../clients/ts/rt_protocol_generated.ts")
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(generated), "export const CycleCaptureRecordV2Bytes = 5392;") {
		t.Fatal("documented size constant must export the measured 5392-byte wire layout")
	}
}

// The appended diagnostic layout must match the pinned digest byte-for-byte in all three
// languages, while reserved-field failures retain one unambiguous label.
func TestTelemetryDigestAndReservedMessage(t *testing.T) {
	data, err := os.ReadFile("../../protocol/control.json")
	if err != nil {
		t.Fatal(err)
	}
	schema, err := parseSchema(data)
	if err != nil {
		t.Fatal(err)
	}
	outputs, err := generate(schema)
	if err != nil {
		t.Fatal(err)
	}
	checked := 0
	for _, output := range outputs {
		switch output.path {
		case "clients/cpp/include/rosie/protocol_generated.hpp", "ipcclient/protocol_generated.go", "clients/ts/rt_protocol_generated.ts":
			if !strings.Contains(string(output.data), `"1173439681672348f35f9652f2a4821d251638c70b60362ff59b37032a794b6b"`) {
				t.Fatalf("telemetry digest changed in %s", output.path)
			}
			checked++
		}
		if output.path == "clients/ts/rt_protocol_generated.ts" {
			if strings.Contains(string(output.data), "reserved reserved") || strings.Count(string(output.data), `"nonzero reserved field: reserved"`) != 2 {
				t.Fatal("reserved-field diagnostics changed")
			}
		}
	}
	if checked != 3 {
		t.Fatalf("language digests checked=%d", checked)
	}
}

// jsonFields preserves uint64 as decimal strings and arrays as arrays, allowing
// field-by-field comparison without JSON number rounding or byte-array base64.
func jsonFields(v reflect.Value) any {
	switch v.Kind() {
	case reflect.Struct:
		m := map[string]any{}
		for i := 0; i < v.NumField(); i++ {
			m[v.Type().Field(i).Name] = jsonFields(v.Field(i))
		}
		return m
	case reflect.Array, reflect.Slice:
		a := make([]any, v.Len())
		for i := range a {
			a[i] = jsonFields(v.Index(i))
		}
		return a
	case reflect.Uint64:
		return strconv.FormatUint(v.Uint(), 10)
	case reflect.Int64:
		return strconv.FormatInt(v.Int(), 10)
	default:
		return v.Interface()
	}
}

// TestTypeScriptGoldenDecoders executes the shipped TypeScript with Node's type
// stripping; every nested field must equal Go, including cursors above 2^53.
func TestTypeScriptGoldenDecoders(t *testing.T) {
	// Unverified tooling location required by the conformance environment.
	node := nodeBinary(t)
	if _, err := os.Stat(node); err != nil {
		t.Fatalf("required TypeScript decoder runtime %s absent: %v", node, err)
	}
	module, err := filepath.Abs("../../clients/ts/rt_protocol_generated.ts")
	if err != nil {
		t.Fatal(err)
	}
	for _, tc := range telemetryGoldenCases() {
		if tc.name == "TelemetryMarkV1" {
			continue
		}
		t.Run(tc.name, func(t *testing.T) {
			fixture, err := filepath.Abs(filepath.Join("testdata/golden", tc.name+".bin"))
			if err != nil {
				t.Fatal(err)
			}
			program := fmt.Sprintf(`import {readFileSync} from 'node:fs'; import {decode%s} from %q;
const bytes=new Uint8Array(readFileSync(%q));
for(const bad of [bytes.subarray(1),new Uint8Array(bytes.length+1)]) {let rejected=false;try{decode%s(bad)}catch{rejected=true}if(!rejected)throw Error('malformed size accepted');}
console.log(JSON.stringify(decode%s(bytes),(_,v)=>typeof v==='bigint'?v.toString():v));`, tc.name, "file://"+module, fixture, tc.name, tc.name)
			if tc.name == "TelemetryBatchHeaderV1" {
				program += fmt.Sprintf(`
const incompatible=bytes.slice();
incompatible[%d]='0'.charCodeAt(0);
let mismatch;
try { decodeTelemetryBatchHeaderV1(incompatible); } catch (e) { mismatch=e; }
if(mismatch?.name!=='TelemetryLayoutMismatchError' || mismatch.storedDigest.length!==64 || mismatch.expectedDigest!==%q ||
   !mismatch.message.includes(mismatch.storedDigest) || !mismatch.message.includes(mismatch.expectedDigest)) throw Error('layout mismatch accepted or unnamed');
`, ipcclient.TelemetryBatchHeaderV1RecordLayoutDigestOffset, ipcclient.CycleCaptureRecordV2LayoutDigest)
			}
			output, err := exec.Command(node, "--experimental-strip-types", "--input-type=module", "-e", program).Output()
			if err != nil {
				t.Fatalf("Node %s: %v stderr=%s", node, err, func() []byte {
					if e, ok := err.(*exec.ExitError); ok {
						return e.Stderr
					}
					return nil
				}())
			}
			bytes, err := os.ReadFile(fixture)
			if err != nil {
				t.Fatal(err)
			}
			decoded, err := tc.decode(bytes)
			if err != nil {
				t.Fatal(err)
			}
			expected, err := json.Marshal(jsonFields(reflect.ValueOf(decoded)))
			if err != nil {
				t.Fatal(err)
			}
			var got, want any
			if json.Unmarshal(output, &got) != nil || json.Unmarshal(expected, &want) != nil || !reflect.DeepEqual(got, want) {
				t.Fatalf("TypeScript/Go fields differ\ngot %s\nwant %s", output, expected)
			}
		})
	}
}

// nodeBinary resolves the pinned Node runtime under the current user's rosie
// toolchain (or NODE when set) and fails, never skips, when it is absent: the
// TypeScript golden comparison is mandatory coverage.
func nodeBinary(t *testing.T) string {
	t.Helper()
	if node := os.Getenv("NODE"); node != "" {
		return node
	}
	home, err := os.UserHomeDir()
	if err != nil {
		t.Fatalf("node runtime: home directory unknown: %v", err)
	}
	node := filepath.Join(home, ".rosie", "node-22.13.1", "bin", "node")
	if _, err := os.Stat(node); err != nil {
		t.Fatalf("node runtime %s is required for the TypeScript golden comparison: %v", node, err)
	}
	return node
}
