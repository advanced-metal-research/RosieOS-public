package main

import (
	"bytes"
	"io"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// Independent v1.3 fixture: changing the live schema must not accidentally
// change the established 40-byte CommandResult offsets during this port.
const fixture = `{
  "version_major": 1, "version_minor": 3, "max_axes": 16, "max_trajectory_points": 250000,
  "messages": {"trajectory_start": 292, "command_result": 519},
  "results": {"accepted": 0, "rejected": 1, "prepared": 2},
  "reasons": {"none": 0, "invalid_command": 1, "not_ready": 2, "mode_conflict": 3, "unknown_handle": 4, "capacity": 5, "invalid_trajectory": 6},
  "command_result": [["sequence", "u64"], ["handle", "u64"], ["generation", "u64"], ["operation", "u32"], ["result", "u32"], ["reason", "u32"], ["axis_mask", "u32"]]
}`

func TestRejectsInvalidSchema(t *testing.T) {
	for _, tc := range []struct{ name, from, to string }{
		{"negative constant", `"version_minor": 3`, `"version_minor": -1`},
		{"fractional constant", `"version_minor": 3`, `"version_minor": 3.5`},
		{"overflow constant", `"version_minor": 3`, `"version_minor": 4294967296`},
		{"null constant", `"version_minor": 3`, `"version_minor": null`},
		{"missing member", `"version_minor": 3,`, ``},
		{"unknown member", `"version_minor": 3,`, `"version_minor": 3, "typo": 0,`},
		{"duplicate root key", `"version_minor": 3,`, `"version_minor": 3, "version_minor": 4,`},
		{"duplicate message key", `"trajectory_start": 292`, `"trajectory_start": 292, "trajectory_start": 293`},
		{"colliding Go names", `"trajectory_start": 292`, `"axis2d": 292, "axis2_d": 293`},
		{"invalid identifier", `"trajectory_start": 292`, `"trajectory-start": 292`},
		{"zero axes", `"max_axes": 16`, `"max_axes": 0`},
		{"axis mask overflow", `"max_axes": 16`, `"max_axes": 33`},
		{"empty groups", `"messages": {"trajectory_start": 292, "command_result": 519}`, `"messages": {}`},
		{"wrong width", `["sequence", "u64"]`, `["sequence", "u16"]`},
		{"extra field member", `["sequence", "u64"]`, `["sequence", "u64", "ignored"]`},
		{"duplicate field", `["handle", "u64"]`, `["sequence", "u64"]`},
		{"reserved C++ field", `["handle", "u64"]`, `["class", "u64"]`},
		{"interior padding", `["sequence", "u64"]`, `["sequence", "u32"]`},
		{"tail padding", `, ["axis_mask", "u32"]`, ``},
	} {
		t.Run(tc.name, func(t *testing.T) {
			invalid := strings.Replace(fixture, tc.from, tc.to, 1)
			if invalid == fixture {
				t.Fatal("test did not change its schema fixture")
			}
			if _, err := parseSchema([]byte(invalid)); err == nil {
				t.Fatal("invalid schema was accepted")
			}
		})
	}
	if _, err := parseSchema([]byte(fixture + " {}")); err == nil {
		t.Fatal("accepted trailing JSON document")
	}
}

func TestGenerationIsDeterministicAndPreservesWireContract(t *testing.T) {
	schema, err := parseSchema([]byte(fixture))
	if err != nil {
		t.Fatal(err)
	}
	first, err := generate(schema)
	if err != nil {
		t.Fatal(err)
	}
	for attempt := 0; attempt < 20; attempt++ {
		next, err := generate(schema)
		if err != nil || len(next) != len(first) {
			t.Fatal("generation changed", err)
		}
		for i := range first {
			if next[i].path != first[i].path || !bytes.Equal(next[i].data, first[i].data) {
				t.Fatalf("nondeterministic output: %s", first[i].path)
			}
		}
	}
	generated := make(map[string]string)
	for _, output := range first {
		generated[output.path] = string(output.data)
	}
	if !strings.Contains(generated["clients/cpp/include/rosie/control_protocol.hpp"], "inline constexpr uint32_t command_result_bytes = 40;") || len(generated) != 3 {
		t.Fatal("v1.3 result width or field encodings changed")
	}
	goOutput := generated["ipcclient/protocol_generated.go"]
	for _, declaration := range []string{
		"Sequence:   binary.LittleEndian.Uint64(p[0:]),",
		"Handle:     binary.LittleEndian.Uint64(p[8:]),",
		"Generation: binary.LittleEndian.Uint64(p[16:]),",
		"Operation:  binary.LittleEndian.Uint32(p[24:]),",
		"Result:     binary.LittleEndian.Uint32(p[28:]),",
		"Reason:     binary.LittleEndian.Uint32(p[32:]),",
		"AxisMask:   binary.LittleEndian.Uint32(p[36:]),",
	} {
		if !strings.Contains(goOutput, declaration) {
			t.Fatalf("changed command-result field offset: %s", declaration)
		}
	}
	if strings.Index(goOutput, "const MsgTrajectoryStart") > strings.Index(goOutput, "const MsgCommandResult") {
		t.Fatal("schema declaration order changed")
	}
}

func TestCheckDetectsDriftWithoutRewriting(t *testing.T) {
	root := t.TempDir()
	if err := os.Mkdir(filepath.Join(root, "protocol"), 0o755); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(filepath.Join(root, "protocol/control.json"), []byte(fixture), 0o644); err != nil {
		t.Fatal(err)
	}
	outputRoot := filepath.Join(root, "isolated")
	args := []string{"--root", root, "--output-root", outputRoot}
	if err := run(args, io.Discard); err != nil {
		t.Fatal(err)
	}
	if _, err := os.Stat(filepath.Join(root, "clients/cpp/include/rosie/control_protocol.hpp")); !os.IsNotExist(err) {
		t.Fatal("--output-root wrote into the schema root")
	}
	checkArgs := append(append([]string{}, args...), "--check")
	if err := run(checkArgs, io.Discard); err != nil {
		t.Fatal(err)
	}
	cppPath := filepath.Join(outputRoot, "clients/cpp/include/rosie/control_protocol.hpp")
	original, err := os.ReadFile(cppPath)
	if err != nil {
		t.Fatal(err)
	}
	legacy := bytes.ReplaceAll(original, []byte("\n"), []byte("\r\n"))
	if err := os.WriteFile(cppPath, legacy, 0o644); err != nil {
		t.Fatal(err)
	}
	if err := run(checkArgs, io.Discard); err != nil {
		t.Fatalf("line endings should pass parity: %v", err)
	}
	corrupt := bytes.Replace(legacy, []byte("msg_trajectory_start = 292"), []byte("msg_trajectory_start = 293"), 1)
	if err := os.WriteFile(cppPath, corrupt, 0o644); err != nil {
		t.Fatal(err)
	}
	if err := run(checkArgs, io.Discard); err == nil || !strings.Contains(err.Error(), "generated protocol drift") {
		t.Fatalf("--check failed to report ABI drift: %v", err)
	}
	after, err := os.ReadFile(cppPath)
	if err != nil || !bytes.Equal(after, corrupt) {
		t.Fatal("--check rewrote the mismatching file", err)
	}
}

func TestCheckedInArtifactsMatchCurrentSchema(t *testing.T) {
	root := filepath.Join("..", "..")
	if err := run([]string{"--root", root, "--check"}, io.Discard); err != nil {
		t.Fatal(err)
	}
}
