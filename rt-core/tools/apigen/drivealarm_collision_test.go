package main

import (
	"crypto/sha256"
	"fmt"
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"testing"
)

// TestDriveAlarmAndCollisionWatchdogContract requires these status contracts,
// including the brake-hold warning, and their shared digest, preventing a merge
// from silently discarding any of them.
func TestDriveAlarmAndCollisionWatchdogContract(t *testing.T) {
	body := schemaBytes(t)
	schema, err := parse(body)
	if err != nil {
		t.Fatal(err)
	}
	types := map[string]Type{}
	for _, typ := range schema.Types {
		types[typ.Name] = typ
	}
	axisFields := []struct{ name, json, typ string }{
		{"DriveAlarm", "drive_alarm", "string"},
		{"DriveAlarmText", "drive_alarm_text", "string"},
		{"DriveAlarmAuxCode", "drive_alarm_aux_code", "uint32"},
		{"DriveAlarmAuxValid", "drive_alarm_aux_valid", "bool"},
		{"TorqueRaw", "torque_raw", "int32"},
		{"CollisionWatchdog", "collision_watchdog", "CollisionWatchdogStatus"},
		{"BrakeHoldDiagnostic", "brake_hold_diagnostic", "BrakeHoldDiagnostic"},
		{"DriveSupervisor", "drive_supervisor", "DriveSupervisorDiagnostic"},
	}
	for _, name := range []string{"AxisStatus", "LogicalAxisStatus"} {
		t.Run(name, func(t *testing.T) {
			typ, ok := types[name]
			if !ok {
				t.Fatalf("schema omits %s", name)
			}
			for _, want := range axisFields {
				found := false
				for _, field := range typ.Fields {
					if field.Name == want.name {
						found = true
						if field.JSON != want.json || field.Type != want.typ || !field.Required || field.OmitEmpty {
							t.Errorf("%s.%s lost its required wire contract: %+v", name, want.name, field)
						}
					}
				}
				if !found {
					t.Errorf("schema omits %s.%s", name, want.name)
				}
			}
		})
	}
	watchdog, ok := types["CollisionWatchdogStatus"]
	if !ok || len(watchdog.Fields) != 11 {
		t.Fatalf("collision watchdog has %d fields, want 11", len(watchdog.Fields))
	}
	wantDigest := fmt.Sprintf("%x", sha256.Sum256(body))
	digestDeclaration := regexp.MustCompile(`CapabilitiesDigest\s*=\s*"([0-9a-f]{64})"`)
	for _, path := range []string{
		"adapters/rosie/control/api_generated.go",
		"clients/cpp/include/rosie/rt_control_api_generated.hpp",
		"clients/ts/rt_control_api_generated.ts",
	} {
		t.Run(path, func(t *testing.T) {
			client, err := os.ReadFile(filepath.Join("../..", path))
			if err != nil {
				t.Fatal(err)
			}
			matches := digestDeclaration.FindAllSubmatch(client, -1)
			if len(matches) != 1 || string(matches[0][1]) != wantDigest {
				t.Fatalf("capability digest declarations %q, want exactly one matching schema SHA-256 %s", matches, wantDigest)
			}
		})
	}

	t.Run("documentation", func(t *testing.T) {
		doc, err := os.ReadFile("../../docs/APPLICATION_API.md")
		if err != nil {
			t.Fatal(err)
		}
		if !strings.Contains(string(doc), "SHA-256\n`"+wantDigest+"`") {
			t.Error("documented schema digest differs from canonical bytes")
		}
		for _, name := range []string{"AxisStatus", "LogicalAxisStatus"} {
			_, section, ok := strings.Cut(string(doc), "### "+name+"\n")
			if !ok {
				t.Errorf("documentation omits %s", name)
				continue
			}
			section, _, _ = strings.Cut(section, "\n### ")
			for _, field := range axisFields {
				if !strings.Contains(section, "\n| `"+field.json+"` |") {
					t.Errorf("documentation omits %s.%s field row", name, field.json)
				}
			}
		}
	})
}
