package main

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"runtime"
	"strings"
	"testing"

	wire "rosieos/rt-core/ipcclient"
)

type goldenCase struct {
	name     string
	size     int
	expected any
	decode   func([]byte) (any, error)
	encode   func(any) []byte
}

func TestPinnedNative13Compatibility(t *testing.T) {
	// The two inputs are the only survivors of the frozen P2 baseline package removed on
	// 2026-09-17 (bloat review item 6): the retained 1.3 schema and the Go codec generated from it then.
	baseline := "testdata/pinned13"
	data, err := os.ReadFile(filepath.Join(baseline, "protocol-1.3.json"))
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
	pinned, err := os.ReadFile(filepath.Join(baseline, "protocol_generated.go.txt"))
	if err != nil {
		t.Fatal(err)
	}
	for _, output := range outputs {
		if output.path == "ipcclient/protocol_generated.go" {
			// Historical generator provenance differs; every codec declaration
			// after the generated header must still match the frozen source.
			_, pinnedBody, _ := bytes.Cut(pinned, []byte{'\n'})
			_, currentBody, _ := bytes.Cut(output.data, []byte{'\n'})
			if !equivalent(output.path, pinnedBody, currentBody) {
				t.Fatal("retained 1.3 schema no longer reproduces its pinned Go codec")
			}
		}
	}
	// Execute the unchanged historical Go decoder against today's C++ golden.
	// Start's retired 16-byte envelope is deliberately not a compatible payload.
	dir := t.TempDir()
	golden, err := os.ReadFile("testdata/golden/CommandResult.bin")
	if err != nil {
		t.Fatal(err)
	}
	files := map[string][]byte{
		"go.mod":                []byte("module pinnedfixture\n\ngo 1.24\n"),
		"protocol_generated.go": pinned,
		"result.bin":            golden,
		"compatibility_test.go": []byte(`package ipcclient
import ("os"; "testing")
func TestPinnedResult(t *testing.T) {
 p,err:=os.ReadFile("result.bin"); if err!=nil { t.Fatal(err) }
 want:=CommandResult{Sequence:0x0908070605040302,Handle:0x11100f0e0d0c0b0a,Generation:0x1918171615141312,Operation:0x1d1c1b1a,Result:0x21201f1e,Reason:0x25242322,AxisMask:0x29282726}
 if len(p)!=40 || decodeCommandResult(p)!=want || VersionMajor!=1 || VersionMinor!=3 {t.Fatal("pinned 1.3 result contract changed")}
}`),
	}
	for name, contents := range files {
		if err := os.WriteFile(filepath.Join(dir, name), contents, 0644); err != nil {
			t.Fatal(err)
		}
	}
	command := exec.Command(filepath.Join(runtime.GOROOT(), "bin", "go"), "test", "-race", "-count=1", "./...")
	command.Dir = dir
	command.Env = append(os.Environ(), "GOWORK=off", "GOTOOLCHAIN=local")
	if output, err := command.CombinedOutput(); err != nil {
		t.Fatalf("pinned 1.3 codec: %v\n%s", err, output)
	}
	for _, tc := range nativeGoldenCases() {
		if tc.name != "HelloV1" && tc.name != "WelcomeV1" {
			continue
		}
		p := tc.encode(tc.expected)
		binary.LittleEndian.PutUint16(p[6:8], 3)
		got, err := tc.decode(p)
		if err != nil || reflect.ValueOf(got).FieldByName("VerMinor").Uint() != 3 {
			t.Fatalf("unchanged 1.3 handshake layout: %+v %v", got, err)
		}
	}
	if _, err := wire.DecodeTrajectoryHandleFrame(make([]byte, 16)); !errors.Is(err, wire.ErrMalformedFrame) {
		t.Fatal("retired 1.3 Start payload accepted as a 1.6 handle")
	}
	header := wire.EncodeShmHeaderV1(wire.ShmHeaderV1{VerMajor: 1, VerMinor: 6, SetpointOffset: 128})
	if _, err := wire.DecodeShmHeaderV1(header[:]); !errors.Is(err, wire.ErrMalformedFrame) {
		t.Fatal("retired setpoint location accepted")
	}
}

func TestGoldenNestedEntriesAreDistinct(t *testing.T) {
	for _, tc := range []struct {
		name           string
		offset, stride int
	}{
		{"StatusSnapshotV1", 40, 28}, {"StatusExtendedSnapshotV1", 16, 24}, {"IoSnapshotV1", 8, 32},
	} {
		p, err := os.ReadFile(filepath.Join("testdata/golden", tc.name+".bin"))
		if err != nil {
			t.Fatal(err)
		}
		if len(p) < tc.offset+2*tc.stride || bytes.Equal(p[tc.offset:tc.offset+tc.stride], p[tc.offset+tc.stride:tc.offset+2*tc.stride]) {
			t.Fatalf("%s golden cannot detect swapped nested entries", tc.name)
		}
	}
}

func TestNativeGoldenBytesAndMalformedFrames(t *testing.T) {
	schema := productionSchema(t)
	records := map[string]wireRecord{}
	for _, r := range schema.wireRecords {
		records[r.Name] = r
	}
	cases := append(nativeGoldenCases(), telemetryGoldenCases()...)
	if len(cases) != len(records) {
		seen := map[string]bool{}
		for _, tc := range cases {
			seen[tc.name] = true
		}
		var missing []string
		for name := range records {
			if !seen[name] {
				missing = append(missing, name)
			}
		}
		t.Fatalf("golden coverage: %d cases for %d records; missing %v", len(cases), len(records), missing)
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			golden, err := os.ReadFile(filepath.Join("testdata", "golden", tc.name+".bin"))
			if err != nil {
				t.Fatal(err)
			}
			if len(golden) != tc.size {
				t.Fatalf("golden has %d bytes; want %d", len(golden), tc.size)
			}
			got, err := tc.decode(golden)
			if err != nil || !reflect.DeepEqual(got, tc.expected) {
				t.Fatalf("field mismatch: got %+v, want %+v, err %v", got, tc.expected, err)
			}
			if encoded := tc.encode(tc.expected); !bytes.Equal(encoded, golden) {
				t.Fatalf("golden changed: %x", encoded)
			}
			reject := func(p []byte, field string) {
				t.Helper()
				got, err := tc.decode(p)
				var typed *wire.MalformedFrameError
				if !errors.Is(err, wire.ErrMalformedFrame) || !errors.As(err, &typed) || typed.Record != tc.name {
					t.Fatalf("%s: expected typed malformed frame, got %+v %v", field, got, err)
				}
				if !reflect.ValueOf(got).IsZero() {
					t.Fatalf("%s: exposed partial fields: %+v", field, got)
				}
			}
			for n := 0; n < tc.size; n++ {
				reject(golden[:n], fmt.Sprintf("short/%d", n))
			}
			reject(append(append([]byte{}, golden...), 0), "oversized")
			r := records[tc.name]
			wireLeaves(r, records, "value", "value", "", func(f wireField, cv, gv, offset string) {
				// Constant expression offsets include nesting; evaluate their integer sum.
				at := 0
				for _, part := range strings.Split(offset, "+") {
					var n int
					if _, err := fmt.Sscan(part, &n); err != nil {
						t.Fatal(err)
					}
					at += n
				}
				width := scalarWidth[f.base]
				if f.Reserved || f.AllowedBits != nil {
					for bit := 0; bit < width*8; bit++ {
						if !f.Reserved && *f.AllowedBits&(uint64(1)<<bit) != 0 {
							continue
						}
						bad := append([]byte{}, golden...)
						bad[at+bit/8] |= 1 << uint(bit%8)
						reject(bad, cv)
					}
				}
			})
			if tc.name == "HelloV1" || tc.name == "WelcomeV1" {
				// Corrupt all other fields too: version must be the first trusted discriminator.
				bad := bytes.Repeat([]byte{0xff}, tc.size)
				binary.LittleEndian.PutUint16(bad[4:6], 2)
				got, err := tc.decode(bad)
				var typed *wire.MalformedFrameError
				if !errors.As(err, &typed) || typed.Field != "ver_major" || !reflect.ValueOf(got).IsZero() {
					t.Fatalf("major mismatch was not rejected first: %+v %v", got, err)
				}
			}
		})
	}
}

func TestNativeSchemaRejectsLayoutDrift(t *testing.T) {
	original, err := os.ReadFile("../../protocol/control.json")
	if err != nil {
		t.Fatal(err)
	}
	for _, mutate := range []func(map[string]any){
		func(s map[string]any) { s["native_records"].([]any)[0].(map[string]any)["size"] = 55 },
		func(s map[string]any) { s["native_records"].([]any)[0].(map[string]any)["alignment"] = 4 },
		func(s map[string]any) {
			s["native_records"].([]any)[0].(map[string]any)["fields"].([]any)[0].(map[string]any)["offset"] = 1
		},
		func(s map[string]any) {
			s["native_records"].([]any)[0].(map[string]any)["fields"].([]any)[0].(map[string]any)["size"] = 8
		},
		func(s map[string]any) {
			s["native_records"].([]any)[0].(map[string]any)["fields"].([]any)[6].(map[string]any)["reserved"] = false
		},
		func(s map[string]any) {
			s["native_records"].([]any)[0].(map[string]any)["fields"].([]any)[0].(map[string]any)["typo"] = true
		},
		func(s map[string]any) { s["native_records"].([]any)[0].(map[string]any)["major_field"] = "pid" },
		func(s map[string]any) { s["wire_contract"].(map[string]any)["endianness"] = "big" },
	} {
		var document map[string]any
		if err := json.Unmarshal(original, &document); err != nil {
			t.Fatal(err)
		}
		mutate(document)
		data, _ := json.Marshal(document)
		if _, err := parseSchema(data); err == nil {
			t.Fatal("accepted invalid native layout")
		}
	}
}

func TestNativeCheckDetectsEveryOutputDrift(t *testing.T) {
	root := t.TempDir()
	if err := os.MkdirAll(filepath.Join(root, "protocol"), 0755); err != nil {
		t.Fatal(err)
	}
	data, err := os.ReadFile("../../protocol/control.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(root, "protocol/control.json"), data, 0644); err != nil {
		t.Fatal(err)
	}
	if err := run([]string{"--root", root}, io.Discard); err != nil {
		t.Fatal(err)
	}
	for _, path := range []string{"include/control_protocol.hpp", "include/protocol_generated.hpp", "ipcclient/protocol_generated.go", "clients/cpp/include/rosie/control_protocol.hpp", "clients/cpp/include/rosie/protocol_generated.hpp", "clients/ts/rt_protocol_generated.ts"} {
		absolute := filepath.Join(root, path)
		original, err := os.ReadFile(absolute)
		if err != nil {
			t.Fatal(err)
		}
		corrupt := append(append([]byte{}, original...), []byte("\nDRIFT\n")...)
		if err := os.WriteFile(absolute, corrupt, 0644); err != nil {
			t.Fatal(err)
		}
		if err := run([]string{"--root", root, "--check"}, io.Discard); err == nil {
			t.Fatalf("undetected drift: %s", path)
		}
		after, err := os.ReadFile(absolute)
		if err != nil || !bytes.Equal(after, corrupt) {
			t.Fatal("check rewrote output", err)
		}
		if err := os.WriteFile(absolute, original, 0644); err != nil {
			t.Fatal(err)
		}
	}
}

// Each leaf is addressed using the schema offset, including nested arrays.
// Byte loads and bit copies avoid unaligned access and signed-shift undefined behavior.
func wireLeaves(r wireRecord, known map[string]wireRecord, cp, gp, offset string, visit func(wireField, string, string, string)) {
	for _, f := range r.Fields {
		count := f.count
		if count == 0 {
			count = 1
		}
		width := f.Size / count
		for i := 0; i < count; i++ {
			cfield, gfield := cp+"."+f.Name, gp+"."+camel(f.Name)
			if f.count != 0 {
				suffix := fmt.Sprintf("[%d]", i)
				cfield += suffix
				gfield += suffix
			}
			at := fmt.Sprintf("%s%d", offset, f.Offset+i*width)
			if child, ok := known[f.base]; ok {
				wireLeaves(child, known, cfield, gfield, at+"+", visit)
			} else {
				visit(f, cfield, gfield, at)
			}
		}
	}
}
