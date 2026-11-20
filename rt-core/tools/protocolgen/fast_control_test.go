package main

import (
	"bytes"
	"os"
	"strings"
	"testing"
)

const fastControlFixture = `{
  "magic": 1179931203,
  "region_bytes": 4096,
  "grant_offset": 128,
  "observation_offset": 256,
  "max_grant_lease_ns": 500000000
}`

func productionSchema(t *testing.T) protocolSchema {
	t.Helper()
	encoded, err := os.ReadFile("../../protocol/control.json")
	if err != nil {
		t.Fatal(err)
	}
	schema, err := parseSchema(encoded)
	if err != nil {
		t.Fatal(err)
	}
	return schema
}

func TestFastControlRejectsInvalidGeometryAndPolicy(t *testing.T) {
	records := productionSchema(t).records
	for _, invalid := range []string{`null`, `[]`, `{}`} {
		if _, err := parseFastControl([]byte(invalid), records); err == nil {
			t.Fatalf("invalid fast_control accepted: %s", invalid)
		}
	}
	for _, tc := range []struct{ name, from, to string }{
		{"missing", `"magic": 1179931203,`, ``},
		{"unknown", `"magic": 1179931203,`, `"typo": 1, "magic": 1179931203,`},
		{"duplicate", `"magic": 1179931203,`, `"magic": 1, "magic": 1179931203,`},
		{"zero magic", `1179931203`, `0`},
		{"negative", `1179931203`, `-1`},
		{"fractional", `1179931203`, `1.5`},
		{"null", `1179931203`, `null`},
		{"overflow", `1179931203`, `18446744073709551616`},
		{"zero lease", `500000000`, `0`},
		{"negative lease", `500000000`, `-1`},
		{"null lease", `500000000`, `null`},
		{"unbounded region", `4096`, `4294967296`},
		{"region alignment", `4096`, `4095`},
		{"grant alignment", `"grant_offset": 128`, `"grant_offset": 129`},
		{"observation alignment", `"observation_offset": 256`, `"observation_offset": 255`},
		{"header overlap", `"grant_offset": 128`, `"grant_offset": 88`},
		{"request sequence overlap", `"observation_offset": 256`, `"observation_offset": 208`},
		{"reversed slots", `"grant_offset": 128`, `"grant_offset": 264`},
		{"observation sequence outside region", `4096`, `384`},
		{"slot outside region", `"observation_offset": 256`, `"observation_offset": 4104`},
	} {
		t.Run(tc.name, func(t *testing.T) {
			invalid := strings.Replace(fastControlFixture, tc.from, tc.to, 1)
			if invalid == fastControlFixture {
				t.Fatal("fixture mutation made no change")
			}
			if _, err := parseFastControl([]byte(invalid), records); err == nil {
				t.Fatalf("invalid geometry/policy accepted: %s", invalid)
			}
		})
	}
	for _, name := range []string{"fast_header", "grant_request", "grant_observation"} {
		t.Run("missing record/"+name, func(t *testing.T) {
			var incomplete []record
			for _, record := range records {
				if record.name != name {
					incomplete = append(incomplete, record)
				}
			}
			if _, err := parseFastControl([]byte(fastControlFixture), incomplete); err == nil {
				t.Fatal("geometry without its complete record set accepted")
			}
		})
	}
	// Check derived record widths, including the publication word, rather than
	// silently assuming that a request always has the historical 80-byte payload.
	for _, width := range []uint32{81, 128} {
		changed := append([]record(nil), records...)
		for i := range changed {
			if changed[i].name == "grant_request" {
				changed[i].width = width
			}
		}
		if _, err := parseFastControl([]byte(fastControlFixture), changed); err == nil {
			t.Fatalf("unaligned/overlapping derived request width %d accepted", width)
		}
	}
}

func TestFastControlGenerationKeepsLegacyPythonUnchanged(t *testing.T) {
	schema := productionSchema(t)
	if len(schema.sharedConstants) != 12 {
		t.Fatal("production fast-control contract is missing")
	}
	withShared, err := generate(schema)
	if err != nil {
		t.Fatal(err)
	}
	schema.sharedConstants = nil
	withoutShared, err := generate(schema)
	if err != nil {
		t.Fatal(err)
	}
	for i, output := range withShared {
		if output.path == "clients/ts/rt_protocol_generated.ts" {
			if !bytes.Equal(output.data, withoutShared[i].data) {
				t.Fatal("private fast-control constants changed telemetry-only TypeScript")
			}
			continue
		}
		if strings.HasPrefix(output.path, "include/") || output.path == "clients/cpp/include/rosie/protocol_generated.hpp" {
			continue // Shared constants remain in its generated control_protocol.hpp include.
		}
		if bytes.Equal(output.data, withoutShared[i].data) {
			t.Fatalf("new shared constants were omitted from %s", output.path)
		}
	}
}

func TestFastControlUses64BitMagicAndDeadlineConstants(t *testing.T) {
	schema := productionSchema(t)
	encoded := strings.Replace(fastControlFixture, "1179931203", "18446744073709551615", 1)
	encoded = strings.Replace(encoded, "500000000", "4294967296", 1)
	var err error
	schema.sharedConstants, err = parseFastControl([]byte(encoded), schema.records)
	if err != nil {
		t.Fatal(err)
	}
	outputs, err := generate(schema)
	if err != nil {
		t.Fatal(err)
	}
	for _, output := range outputs {
		if output.path == "clients/cpp/include/rosie/control_protocol.hpp" &&
			(!bytes.Contains(output.data, []byte("uint64_t fast_control_magic = UINT64_C(18446744073709551615)")) ||
				!bytes.Contains(output.data, []byte("uint64_t fast_control_max_grant_lease_ns = UINT64_C(4294967296)"))) {
			t.Fatal("shared 64-bit wire quantities were truncated in C++")
		}
		if output.path == "ipcclient/protocol_generated.go" &&
			(!bytes.Contains(output.data, []byte("const FastControlMagic = 18446744073709551615")) ||
				!bytes.Contains(output.data, []byte("const FastControlMaxGrantLeaseNs = 4294967296"))) {
			t.Fatal("shared 64-bit wire quantities were truncated in Go")
		}
	}
}

// The lossless path sample slot and its observation must stay after the jog
// slots inside the region; an overlap would let a sample chunk tear a jog
// observation or a latest-value jog update.
func TestFastControlPathSlotsRejectOverlap(t *testing.T) {
	encoded, err := os.ReadFile("../../protocol/control.json")
	if err != nil {
		t.Fatal(err)
	}
	members, err := object(encoded)
	if err != nil {
		t.Fatal(err)
	}
	var production string
	for _, member := range members {
		if member.name == "fast_control" {
			production = string(member.value)
		}
	}
	records := productionSchema(t).records
	if _, err := parseFastControl([]byte(production), records); err != nil {
		t.Fatalf("production geometry refused: %v", err)
	}
	for _, tc := range []struct{ name, from, to string }{
		{"samples overlap jog observation", `"path_samples_offset": 1280`, `"path_samples_offset": 1184`},
		{"observation overlaps samples", `"path_observation_offset": 3584`, `"path_observation_offset": 3576`},
		{"observation outside region", `"path_observation_offset": 3584`, `"path_observation_offset": 3912`},
		{"misaligned samples", `"path_samples_offset": 1280`, `"path_samples_offset": 1284`},
		{"unknown observation key", `"path_observation_offset"`, `"path_observation_typo"`},
	} {
		t.Run(tc.name, func(t *testing.T) {
			invalid := strings.Replace(production, tc.from, tc.to, 1)
			if invalid == production {
				t.Fatal("fixture mutation made no change")
			}
			if _, err := parseFastControl([]byte(invalid), records); err == nil {
				t.Fatalf("invalid path geometry accepted: %s", invalid)
			}
		})
	}
	if _, err := parseFastControl([]byte(strings.Replace(fastControlFixture, `"magic"`, `"path_samples_offset": 1280, "path_observation_offset": 3584, "magic"`, 1)), records); err == nil {
		t.Fatal("path slots accepted without jog slots")
	}
}
