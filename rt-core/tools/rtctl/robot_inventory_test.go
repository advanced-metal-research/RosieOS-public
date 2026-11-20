package rtctl

import (
	"bytes"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// Unverified fake robot drive values: linear tracking mode 1, standard 1/1
// position factor and 131072 motor counts/rev. No fake command writes any drive.
const robotInventoryFake = `#!/bin/sh
printf '%s\n' "$*" >> "$INVENTORY_CALLS"
case "$*" in
 'slaves --verbose') printf '=== Master 0, Slave 0 ===\nIdentity:\nVendor Id: 0x00400000\nProduct code: 0x00000715\nRevision number: 0x00002ef8\n'; exit 0 ;;
 'upload --position 0 --type uint32 0x1018 1') echo 4194304; exit 0 ;;
 'upload --position 0 --type uint32 0x1018 2') echo 1813; exit 0 ;;
 'upload --position 0 --type uint32 0x1018 3') echo 12024; exit 0 ;;
 'upload --position 0 --type uint16 0x6041 0') echo 39; exit 0 ;;
 'upload --position 0 --type int8 0x6061 0') echo 8; exit 0 ;;
 'upload --position 0 --type int32 0x6064 0') echo 0; exit 0 ;;
 'upload --position 0 --type uint16 0x2000 8') field=tracking; value=1 ;;
 'upload --position 0 --type uint32 0x6091 1') field=numerator; value=1 ;;
 'upload --position 0 --type uint32 0x6091 2') field=denominator; value=1 ;;
 'upload --position 0 --type uint32 0x608f 1') field=encoder; value=131072 ;;
 'upload --position 0 --type uint32 0x608f 2') field=revolutions; value=1 ;;
 *) echo 'unexpected command' >&2; exit 99 ;;
esac
if [ "$ROBOT_MISMATCH" = "$field" ]; then value=$((value + 1)); fi
if [ "$ROBOT_MISMATCH" = encoder-unavailable ] && [ "$field" = encoder ]; then echo 'SDO abort 0x06020000' >&2; exit 1; fi
if [ "$ROBOT_MISMATCH" = encoder-malformed ] && [ "$field" = encoder ]; then echo '131072 junk'; exit 0; fi
if [ "$ROBOT_MISMATCH" = encoder-equivalent ]; then
 if [ "$field" = encoder ]; then value=262144; fi
 if [ "$field" = revolutions ]; then value=2; fi
fi
if [ "$ROBOT_MISMATCH" = encoder-zero ] && [ "$field" = revolutions ]; then value=0; fi
echo "$value"
`

// TestRobotInventoryCrossChecks exercises the CLI against exact independent
// readbacks; every mismatch and missing encoder object must refuse qualification.
func TestRobotInventoryCrossChecks(t *testing.T) {
	for _, scenario := range []string{"match", "tracking", "numerator", "denominator", "encoder", "revolutions", "encoder-unavailable", "encoder-malformed", "encoder-equivalent", "encoder-zero"} {
		t.Run(scenario, func(t *testing.T) {
			f := newRobotFixture(t, true)
			f.machine.set("backend", "live")
			config := filepath.Join(f.root, "cell.json")
			writeTestFile(t, config, canonical(f.machine), 0o644)
			fake := filepath.Join(f.root, "ethercat")
			writeTestFile(t, fake, robotInventoryFake, 0o755)
			calls := filepath.Join(f.root, "calls")
			t.Setenv("INVENTORY_CALLS", calls)
			t.Setenv("ROBOT_MISMATCH", scenario)
			var stdout, stderr bytes.Buffer
			code := RunInventoryCLI([]string{"--config", config, "--ethercat", fake, "--json"}, f.root, &stdout, &stderr)
			wantCode, wantStatus := 2, "mismatch"
			if scenario == "match" || scenario == "encoder-equivalent" {
				wantCode, wantStatus = 0, "match"
			}
			if scenario == "encoder-unavailable" || scenario == "encoder-malformed" {
				wantCode, wantStatus = 3, "unreadable"
			}
			if code != wantCode || stderr.Len() != 0 {
				t.Fatalf("code=%d stderr=%s report=%s", code, &stderr, &stdout)
			}
			var report inventoryReport
			if err := json.Unmarshal(stdout.Bytes(), &report); err != nil {
				t.Fatal(err)
			}
			if len(report.Axes) != 1 || report.Axes[0].Status != wantStatus || len(report.Axes[0].DriveChecks) != 5 || len(report.Axes[0].Reads) != 11 {
				t.Fatalf("%+v", report)
			}
			found := false
			for _, check := range report.Axes[0].DriveChecks {
				if check.Reason != "" {
					found = true
					if check.Reason != "drive_parameter_mismatch" && check.Reason != "drive_parameter_unreadable" {
						t.Fatal(check)
					}
				}
			}
			if found != (wantCode != 0) {
				t.Fatal("missing typed refusal or false disagreement")
			}
			data, err := os.ReadFile(calls)
			if err != nil {
				t.Fatal(err)
			}
			lines := strings.Split(strings.TrimSpace(string(data)), "\n")
			if len(lines) != 12 || lines[0] != "slaves --verbose" {
				t.Fatal(lines)
			}
			for _, line := range lines[1:] {
				if !strings.HasPrefix(line, "upload --position 0 --type ") {
					t.Fatal("non-read command: " + line)
				}
			}
		})
	}
}

func TestRobotInventoryBindsEachAxis(t *testing.T) {
	data := readInheritedRobotTestFile(t, "tools/rtctl/testdata/retired/bench-ninemotor-a6ec.json")
	c, err := Compile(data, filepath.Join(testRoot(t), "config/drives"))
	if err != nil {
		t.Fatal(err)
	}
	axes, err := compiledInventoryAxes(asObject(decode(data)), c)
	if err != nil {
		t.Fatal(err)
	}
	if len(axes) != 9 {
		t.Fatal(len(axes))
	}
	for i, axis := range axes {
		if axis.Name != fmt.Sprintf("J%d", i+1) || axis.SlavePosition != uint32(i) || axis.Profile != "a6ec" || len(axis.DriveChecks) != 5 {
			t.Fatal(axis)
		}
		if axis.DriveChecks[0].Expected != 1 || axis.DriveChecks[1].Expected != 1 ||
			axis.DriveChecks[2].Expected != 1 || axis.DriveChecks[3].Expected != 131072 {
			t.Fatal(axis.DriveChecks)
		}
	}
}
