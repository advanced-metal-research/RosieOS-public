package rtctl

import (
	"bytes"
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

// Unverified fake drive tuning values: stiffness level 12 (0x2000:06) and load
// inertia ratio 250 % (0x2000:07); every other command reaches inventoryFake.
const inventoryObserveFake = `#!/bin/sh
case "$*" in
  'upload --position 3 --type uint16 0x2000 6')
    printf '%s\n' "$*" >> "$INVENTORY_CALLS"; printf '0x000c 12\n' ;;
  'upload --position 3 --type uint16 0x2000 7')
    printf '%s\n' "$*" >> "$INVENTORY_CALLS"
    if [ "$INVENTORY_OBSERVE" = unreadable ]; then printf 'SDO abort\n' >&2; exit 7; fi
    printf '250\n' ;;
  *) exec "$(dirname "$0")/ethercat-base" "$@" ;;
esac
`

// TestInventoryObservesCompiledTuning requires inventory to upload exactly the
// tuning objects the compiled configuration has rt-core observe at startup, in
// the compiled order, preventing a second hand-kept list from drifting from the
// startup journal. Like rt-core, a failed observe read is reported but never
// changes the identity verdict or exit status.
func TestInventoryObservesCompiledTuning(t *testing.T) {
	root, machine, fake, f := inventoryFixture(t)
	writeTestFile(t, filepath.Join(filepath.Dir(fake), "ethercat-base"), inventoryFake, 0o755)
	writeTestFile(t, fake, inventoryObserveFake, 0o755)
	writeTestFile(t, filepath.Join(root, "config/drives/a6ec.tuning.json"), string(readTestFile(t, "config/drives/a6ec.tuning.json")), 0o644)
	f.put("machine.observe", `["a6ec_stiffness_level","a6ec_load_inertia_ratio"]`)
	writeTestFile(t, machine, canonical(f.get("machine")), 0o644)
	data, err := os.ReadFile(machine)
	if err != nil {
		t.Fatal(err)
	}
	compiled, err := Compile(data, filepath.Join(root, "config/drives"))
	if err != nil {
		t.Fatal(err)
	}
	// The catalog places both keys on 0x2000 (C00.05 at :06, C00.06 at :07) as u16.
	if !strings.Contains(compiled.AxisProfiles, "\nstartup_observe=a6ec_stiffness_level|u16|8192|6;a6ec_load_inertia_ratio|u16|8192|7\n") {
		t.Fatalf("compiled observe list changed: %s", compiled.AxisProfiles)
	}
	identityCalls := []string{
		"slaves --verbose",
		"upload --position 3 --type uint32 0x1018 1",
		"upload --position 3 --type uint32 0x1018 2",
		"upload --position 3 --type uint32 0x1018 3",
		"upload --position 3 --type uint16 0x6041 0",
		"upload --position 3 --type int8 0x6061 0",
		"upload --position 3 --type int32 0x6064 0",
	}
	observeCalls := []string{"upload --position 3 --type uint16 0x2000 6", "upload --position 3 --type uint16 0x2000 7"}

	var stdout, stderr bytes.Buffer
	if code := RunInventoryCLI([]string{"--config", machine, "--json"}, root, &stdout, &stderr); code != 0 || stderr.Len() != 0 {
		t.Fatalf("exit=%d stderr=%s", code, &stderr)
	}
	report := decodeInventory(t, stdout.Bytes())
	if report.ConfigurationSHA256 != compiled.SHA256 {
		t.Fatal("inventory compiled a different configuration")
	}
	axis := report.Axes[0]
	stiffness, inertia := int64(12), int64(250)
	want := []inventoryObservation{
		{Key: "a6ec_stiffness_level", Index: "0x2000", Subindex: "6", Type: "uint16", Observed: &stiffness},
		{Key: "a6ec_load_inertia_ratio", Index: "0x2000", Subindex: "7", Type: "uint16", Observed: &inertia},
	}
	if axis.Status != "match" || len(axis.Reads) != 8 || len(axis.Errors) != 0 || !reflect.DeepEqual(axis.Observe, want) {
		t.Fatalf("observed tuning: %+v", axis)
	}
	if calls := inventoryCalls(t); !reflect.DeepEqual(calls, append(append([]string{}, identityCalls...), observeCalls...)) {
		t.Fatalf("read-only command sequence: %q", calls)
	}

	if err := os.Remove(os.Getenv("INVENTORY_CALLS")); err != nil {
		t.Fatal(err)
	}
	t.Setenv("INVENTORY_OBSERVE", "unreadable")
	stdout.Reset()
	if code := RunInventoryCLI([]string{"--config", machine, "--json"}, root, &stdout, &stderr); code != 0 {
		t.Fatalf("an unreadable observation changed the exit status: %d %s", code, &stderr)
	}
	axis = decodeInventory(t, stdout.Bytes()).Axes[0]
	failed := axis.Observe[1]
	if axis.Status != "match" || len(axis.Errors) != 0 || failed.Reason != "observe_unreadable" || failed.Observed != nil ||
		axis.Observe[0].Observed == nil || *axis.Observe[0].Observed != 12 {
		t.Fatalf("unreadable observation: %+v", axis)
	}
	if read := axis.Reads[7]; read.ExitCode != 7 || read.Stderr != "SDO abort\n" {
		t.Fatalf("failed observe read not retained: %+v", read)
	}
	stdout.Reset()
	if code := RunInventoryCLI([]string{"--config", machine}, root, &stdout, &stderr); code != 0 {
		t.Fatalf("human summary exit=%d: %s", code, &stderr)
	}
	lines := strings.Split(strings.TrimSuffix(stdout.String(), "\n"), "\n")
	if len(lines) != 4 || strings.Join(strings.Fields(lines[2]), " ") != "axis_0 3 test match observe_unreadable: a6ec_load_inertia_ratio 0x2000:7" ||
		lines[3] != "observe axis_0: a6ec_stiffness_level=12 a6ec_load_inertia_ratio=unreadable" {
		t.Fatalf("human summary: %q", stdout.String())
	}
}
