package main

import (
	"encoding/json"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"runtime"
	"strings"
	"testing"
)

func recordFixture(records string) string {
	base := strings.TrimSuffix(fixture, "}")
	return base + `, "records": ` + records + `}`
}

func TestRejectsInvalidRecords(t *testing.T) {
	for _, records := range []string{
		`null`,
		`{"trajectory_handle": []}`,
		`{"trajectory_handle": [["handle", "i64"]]}`,
		`{"trajectory_handle": [["handle", "u64"], ["handle", "u64"]]}`,
		`{"trajectory_handle": [["size", "u32"], ["handle", "u64"]]}`,
		`{"trajectory_handle": [["handle", "u64"]], "trajectory_handle": [["handle", "u32"]]}`,
		`{"command_result": [["handle", "u64"]]}`,
		`{"trajectory-handle": [["handle", "u64"]]}`,
	} {
		if _, err := parseSchema([]byte(recordFixture(records))); err == nil {
			t.Errorf("invalid records accepted: %s", records)
		}
	}
}

// Compile and execute independent byte fixtures against generated artifacts.
// A generator string comparison alone cannot establish that its codecs work.
func TestGeneratedHandleCodecs(t *testing.T) {
	schema, err := parseSchema([]byte(recordFixture(`{"trajectory_handle": [["handle", "u64"]]}`)))
	if err != nil {
		t.Fatal(err)
	}
	runGeneratedCodecs(t, schema, "trajectory_handle")
}

func runGeneratedCodecs(t *testing.T, schema protocolSchema, name string, extraFiles ...string) {
	t.Helper()
	outputs, err := generate(schema)
	if err != nil {
		t.Fatal(err)
	}
	dir := t.TempDir()
	for _, output := range outputs {
		path := filepath.Join(dir, output.path)
		if err := os.MkdirAll(filepath.Dir(path), 0o755); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, output.data, 0o644); err != nil {
			t.Fatal(err)
		}
	}
	fixtureGo, err := os.ReadFile("testdata/" + name + "_test.go")
	if err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(dir, "ipcclient", name+"_test.go"), fixtureGo, 0o644); err != nil {
		t.Fatal(err)
	}
	var cppArgs []string
	for _, name := range extraFiles {
		data, err := os.ReadFile(filepath.Join("testdata", name))
		if err != nil {
			t.Fatal(err)
		}
		path := filepath.Join(dir, "ipcclient", name)
		if err := os.WriteFile(path, data, 0o644); err != nil {
			t.Fatal(err)
		}
		cppArgs = append(cppArgs, path)
	}
	if err := os.WriteFile(filepath.Join(dir, "go.mod"), []byte("module codecfixture\n\ngo 1.24\n"), 0o644); err != nil {
		t.Fatal(err)
	}
	goTool := filepath.Join(runtime.GOROOT(), "bin", "go")
	if runtime.GOOS == "windows" {
		goTool += ".exe"
	}
	goTest := exec.Command(goTool, "test", "./ipcclient")
	goTest.Dir = dir
	goTest.Env = append(os.Environ(), "GOWORK=off", "GOTOOLCHAIN=local")
	if output, err := goTest.CombinedOutput(); err != nil {
		t.Fatalf("generated Go golden-byte fixtures failed: %v\n%s", err, output)
	}

	cppFixture, err := filepath.Abs("testdata/" + name + "_contract.cpp")
	if err != nil {
		t.Fatal(err)
	}
	binary := filepath.Join(dir, name+"-contract")
	if runtime.GOOS == "windows" {
		binary += ".exe"
	}
	compile := exec.Command("g++", "-std=c++17", "-O2", "-fstrict-aliasing", "-Wall", "-Wextra", "-Werror", "-I"+filepath.Join(dir, "include"), cppFixture, "-o", binary)
	if output, err := compile.CombinedOutput(); err != nil {
		t.Fatalf("generated C++ fixture needs the RT-core C++ compiler: %v\n%s", err, output)
	}
	if output, err := exec.Command(binary, cppArgs...).CombinedOutput(); err != nil {
		t.Fatalf("generated C++ golden-byte fixtures failed: %v\n%s", err, output)
	}
}

func TestRejectsInvalidArrayLayouts(t *testing.T) {
	for _, kind := range []string{"u8", "u16", "f32", "i64", "u64[16]", "u32[16]", "f64[]", "f64[0]", "u8[17]", "f64[-1]", "u8[01]", "f64[1.5]", "f64[16][16]", "f64[999999999999999999999999999999999]"} {
		t.Run(kind, func(t *testing.T) {
			fields, _ := json.Marshal([][]string{{"value", kind}})
			if _, err := parseRecord("invalid", fields); err == nil {
				t.Fatalf("unsupported/unbounded type accepted: %s", kind)
			}
		})
	}
	for _, fields := range []string{
		`[["prefix", "u8[1]"], ["velocity", "f64"]]`,
		`[["velocity", "f64"], ["tail", "u8[1]"]]`,
	} {
		if _, err := parseRecord("padding", []byte(fields)); err == nil {
			t.Fatalf("implicit layout padding accepted: %s", fields)
		}
	}
	var fields [][]string
	for i := 0; i < maxRecordBytes/(16*8); i++ {
		fields = append(fields, []string{fmt.Sprintf("values_%d", i), "f64[16]"})
	}
	encoded, _ := json.Marshal(fields)
	if record, err := parseRecord("bounded", encoded); err != nil || record.width != maxRecordBytes {
		t.Fatalf("exact bounded layout rejected: width=%d err=%v", record.width, err)
	}
	fields = append(fields, []string{"overflow", "f64[16]"})
	encoded, _ = json.Marshal(fields)
	if _, err := parseRecord("overflow", encoded); err == nil {
		t.Fatal("record layout exceeded its bounded byte size")
	}
	if _, err := parseSchema([]byte(strings.Replace(fixture, `["sequence", "u64"]`, `["sequence", "f64"]`, 1))); err == nil {
		t.Fatal("new record types silently changed the transitional CommandResult codec")
	}
}

func TestGeneratedJogCodecs(t *testing.T) {
	const records = `{
		"jog_update": [
			["conn_id", "u64"], ["native_generation", "u64"], ["capability_id", "u8[16]"],
			["app_grant_generation", "u64"], ["config_epoch", "u64"], ["home_epoch", "u64"],
			["jog_generation", "u64"], ["source_sequence", "u64"], ["earliest_origin_host_ns", "u64"],
			["immutable_deadline_host_ns", "u64"], ["clock_mapping_generation", "u64"],
			["axis_mask", "u32"], ["flags", "u32"], ["velocity", "f64[16]"], ["reserved", "u64"]
		],
		"scalar_sample": [["value", "f64"], ["marker", "u32"], ["reserved", "u32"]]
	}`
	schema, err := parseSchema([]byte(recordFixture(records)))
	if err != nil {
		t.Fatal(err)
	}
	if schema.records[0].width != 240 || schema.records[1].width != 16 {
		t.Fatalf("unexpected fixed-record widths: %+v", schema.records)
	}
	runGeneratedCodecs(t, schema, "jog_update", "jog_update_golden.hex")
}

func TestGeneratedByteOnlyCodec(t *testing.T) {
	schema, err := parseSchema([]byte(recordFixture(`{"byte_marker": [["value", "u8[1]"]]}`)))
	if err != nil {
		t.Fatal(err)
	}
	// This schema contains no f64: compile the byte-array branch without the
	// float-only includes/imports supplied by the larger jog fixture.
	runGeneratedCodecs(t, schema, "byte_marker")
}

func TestProductionLocalJogCodec(t *testing.T) {
	encoded, err := os.ReadFile("../../protocol/control.json")
	if err != nil {
		t.Fatal(err)
	}
	schema, err := parseSchema(encoded)
	if err != nil {
		t.Fatal(err)
	}
	runGeneratedCodecs(t, schema, "local_jog_update", "local_jog_update_golden.hex")
}

func TestGeneratedShortFloatArrays(t *testing.T) {
	schema, err := parseSchema([]byte(recordFixture(`{
		"short_floats": [["single", "f64[1]"], ["pair", "f64[2]"], ["tag", "u8[8]"]]
	}`)))
	if err != nil {
		t.Fatal(err)
	}
	runGeneratedCodecs(t, schema, "short_floats")
}

func TestProductionGrantLaneCodecs(t *testing.T) {
	encoded, err := os.ReadFile("../../protocol/control.json")
	if err != nil {
		t.Fatal(err)
	}
	schema, err := parseSchema(encoded)
	if err != nil {
		t.Fatal(err)
	}
	// Read the production schema so a field/layout change there must also pass
	// these independent fixed 96/80/128-byte fixtures in both languages.
	runGeneratedCodecs(t, schema, "grant_lane")
}

func TestRejectsInvalidOptionalGrantEnums(t *testing.T) {
	for _, group := range []string{"grant_operations", "grant_reasons"} {
		for _, invalid := range []string{
			`null`, `[]`, `{}`, `{"bad-name": 1}`, `{"bad": -1}`, `{"bad": 0.5}`,
			`{"bad": 4294967296}`, `{"bad": null}`, `{"same": 1, "same": 2}`,
			`{"axis2d": 1, "axis2_d": 2}`,
		} {
			t.Run(group+"/"+invalid, func(t *testing.T) {
				encoded := strings.TrimSuffix(fixture, "}") + `, "` + group + `": ` + invalid + `}`
				if _, err := parseSchema([]byte(encoded)); err == nil {
					t.Fatalf("invalid optional enum accepted: %s", encoded)
				}
			})
		}
	}
}
