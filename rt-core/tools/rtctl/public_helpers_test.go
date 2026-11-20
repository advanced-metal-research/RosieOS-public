package rtctl

// Helpers shared by public and private tests; the private tests that also
// use them read site configuration and are left out of the public mirror.

import (
	"bytes"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"rosieos/robotdesc"
	"sort"
	"strconv"
	"strings"
	"testing"
)

// Unverified fixture rationale; no test claims the physical brake was inspected.
const brakeOverrideJSON = `{"present":false,"reason":"unverified bench wiring"}`

// a6Axes returns an A6-EC fixture with n axes on the linear profile and 1/1
// software gearing, so tuning keys can be exercised per axis.
func a6Axes(t *testing.T, n int) *fixture {
	t.Helper()
	f := fresh()
	f.put("profile", string(readTestFile(t, "config/drives/a6ec.json")))
	f.put("machine.axes.0.wire_counts_per_rev", "131072")
	f.put("machine.axes.0.motor_encoder_counts_per_rev", "131072")
	f.put("machine.axes.0.startup", `{}`)
	axis := canonical(f.get("machine.axes.0"))
	axes := []any{}
	for i := 0; i < n; i++ {
		a := asObject(decode([]byte(axis)))
		a.set("name", fmt.Sprintf("axis_%d", i))
		a.set("slave_position", json.Number(strconv.Itoa(i)))
		axes = append(axes, a)
	}
	f.data = asObject(replace(f.data, []string{"machine", "axes"}, axes, false))
	return f
}

// Unverified fake drive: IDs 4194304/1813/12024, statusword 39, mode 8 and -123 counts.
// The exhaustive argument cases reject any command outside the seven read operations.
const inventoryFake = `#!/bin/sh
printf '%s\n' "$*" >> "$INVENTORY_CALLS"
# Cited nine-motor bench values: readback.txt slaves 0..8 SII/CoE/DC;
# v4-status.json axis_statusword, axis_mode_display and axis_pos_counts.
if [ "$INVENTORY_SCENARIO" = ninemotor ]; then
  case "$*" in
    'slaves --verbose')
      for position in 0 1 2 3 4 5 6 7 8; do
        printf '=== Master 0, Slave %s ===\nDevice: Main\nState: OP\nFlag: +\nIdentity:\n  Vendor Id:       0x00400000\n  Product code:    0x00000715\n  Revision number: 0x00002ef8\n  Serial number:   0x00000000\nDL information:\n  Distributed clocks: yes, 64 bit\n' "$position"
      done
      ;;
    'upload --position '[0-8]' --type uint32 0x1018 1') printf '0x00400000 4194304\n' ;;
    'upload --position '[0-8]' --type uint32 0x1018 2') printf '0x00000715 1813\n' ;;
    'upload --position '[0-8]' --type uint32 0x1018 3')
      case "$3" in
        0|1|2|6|7) printf '0x00005611 22033\n' ;;
        3|4|5|8) printf '0x00005612 22034\n' ;;
      esac
      ;;
    'upload --position '[0-8]' --type uint16 0x6041 0')
      case "$3" in
        [0-5]) printf '0x9650 38480\n' ;;
        [6-8]) printf '0x1650 5712\n' ;;
      esac
      ;;
    'upload --position '[0-8]' --type int8 0x6061 0') printf '0x08 8\n' ;;
    'upload --position '[0-8]' --type int32 0x6064 0')
      case "$3" in
        0) printf '6509175\n' ;; 1) printf '6553611\n' ;; 2) printf '6553590\n' ;;
        3) printf '1179628\n' ;; 4) printf '6553604\n' ;; 5) printf '655362\n' ;;
        6) printf '%s\n' '-2' ;; 7) printf '30\n' ;; 8) printf '%s\n' '-32' ;;
      esac
      ;;
    'upload --position '[0-8]' --type uint16 0x2000 8') printf '1\n' ;;
    'upload --position '[0-8]' --type uint32 0x6091 1') printf '1\n' ;;
    'upload --position '[0-8]' --type uint32 0x6091 2') printf '1\n' ;;
    'upload --position '[0-8]' --type uint32 0x608f 1') printf '131072\n' ;;
    'upload --position '[0-8]' --type uint32 0x608f 2') printf '1\n' ;;
    *) printf 'unexpected command: %s\n' "$*" >&2; exit 99 ;;
  esac
  exit 0
fi
case "$*" in
  'slaves --verbose')
    case "$INVENTORY_SCENARIO" in
      topology-failed) printf 'master unavailable\n' >&2; exit 9 ;;
      topology-malformed) printf 'unrecognized topology\n'; exit 0 ;;
      topology-empty) exit 0 ;;
      topology-position) printf '=== Master 0, Slave 4 ===\n'; exit 0 ;;
      topology-duplicate) printf '=== Master 0, Slave 3 ===\n=== Master 0, Slave 3 ===\n'; exit 0 ;;
      topology-masters) printf '=== Master 0, Slave 3 ===\n=== Master 1, Slave 3 ===\n'; exit 0 ;;
    esac
    vendor=0x00400000; product=0x00000715; revision=0x00002ef8
    case "$INVENTORY_SCENARIO" in
      vendor) vendor=0x00400001 ;;
      product) product=0x00000716 ;;
      revision|precedence) revision=0x00002ef9 ;;
    esac
    printf '=== Master 0, Slave 3 ===\nDevice: Main\nState: PREOP\nFlag: +\nIdentity:\n  Vendor Id:       %s\n  Product code:    %s\n  Revision number: %s\n' "$vendor" "$product" "$revision"
    if [ "$INVENTORY_SCENARIO" = multi ]; then printf '=== Master 0, Slave 7 ===\nDevice: Main\nState: PREOP\nFlag: +\nIdentity:\n  Vendor Id:       0x00400000\n  Product code:    0x00000715\n  Revision number: 0x00002ef8\n'; fi
    ;;
  'upload --position 3 --type uint32 0x1018 1')
    case "$INVENTORY_SCENARIO" in
      vendor) printf '4194305\n'; exit 0 ;;
      malformed) printf '0x00400000 noise\n'; exit 0 ;;
      overflow) printf '4294967296\n'; exit 0 ;;
    esac
    printf '0x00400000 4194304\n'
    ;;
  'upload --position 3 --type uint32 0x1018 2')
    if [ "$INVENTORY_SCENARIO" = product ]; then printf '1814\n'; else printf '1813\n'; fi
    ;;
  'upload --position 3 --type uint32 0x1018 3')
    case "$INVENTORY_SCENARIO" in
      revision|precedence) printf '12025\n' ;;
      coe-revision) printf '22033\n' ;;
      unreadable) printf 'partial\n'; printf 'SDO abort\n' >&2; exit 7 ;;
      *) printf '0x00002ef8\n' ;;
    esac
    ;;
  'upload --position 3 --type uint16 0x6041 0')
    if [ "$INVENTORY_SCENARIO" = statusword ]; then printf '65536\n'; else printf '0x0027 39\n'; fi
    ;;
  'upload --position 3 --type int8 0x6061 0')
    if [ "$INVENTORY_SCENARIO" = mode ]; then printf '128\n'; else printf '8\n'; fi
    ;;
  'upload --position 3 --type int32 0x6064 0')
    case "$INVENTORY_SCENARIO" in
      position) printf '2147483648\n' ;;
      precedence) printf 'position read failed\n' >&2; exit 8 ;;
      *) printf '%s\n' '-123'; printf 'read-only diagnostic\n' >&2 ;;
    esac
    ;;
  'upload --position 7 --type uint32 0x1018 1') printf '4194304\n' ;;
  'upload --position 7 --type uint32 0x1018 2') printf '1813\n' ;;
  'upload --position 7 --type uint32 0x1018 3') printf '12024\n' ;;
  'upload --position 7 --type uint16 0x6041 0') printf '39\n' ;;
  'upload --position 7 --type int8 0x6061 0') printf '8\n' ;;
  'upload --position 7 --type int32 0x6064 0') printf '0\n' ;;
  *) printf 'unexpected command: %s\n' "$*" >&2; exit 99 ;;
esac
`

func inventoryFixture(t *testing.T) (string, string, string, *fixture) {
	t.Helper()
	root := buildTemp(t)
	f := fresh()
	f.put("machine.backend", `"live"`)
	f.put("machine.axes.0.slave_position", "3")
	// Inventory uses the surveyed fixed map; fabricated A6 layouts now refuse compilation.
	a6 := asObject(decode(readTestFile(t, "config/drives/a6ec.json")))
	drive := asObject(a6.get("ethercat"))
	for _, key := range []string{"rx_pdo", "tx_pdo", "rx_layout", "tx_layout"} {
		f.put("profile.ethercat."+key, canonical(drive.get(key)))
	}
	f.put("profile.ethercat.vendor_id", "4194304")
	f.put("profile.ethercat.product_code", "1813")
	f.put("profile.ethercat.revision_no", "12024")
	// Unverified inventory fixture: declare a 1000-count encoder and 1/1 gearing
	// so strict compilation reaches the independent read-only hardware assertions.
	f.put("machine.axes.0.motor_encoder_counts_per_rev", "1000")
	f.put("profile.startup_defaults", canonical(a6.get("startup_defaults")))
	f.put("profile.startup_schema", canonical(a6.get("startup_schema")))
	machine := filepath.Join(root, "machine.json")
	writeTestFile(t, machine, canonical(f.get("machine")), 0o644)
	writeTestFile(t, filepath.Join(root, "config/drives", "test.json"), canonical(f.get("profile")), 0o644)
	fake := filepath.Join(root, "fake tools", "ethercat")
	writeTestFile(t, fake, inventoryFake, 0o755)
	t.Setenv("PATH", filepath.Dir(fake)+string(os.PathListSeparator)+os.Getenv("PATH"))
	t.Setenv("INVENTORY_CALLS", filepath.Join(root, "calls"))
	t.Setenv("INVENTORY_SCENARIO", "match")
	return root, machine, fake, f
}

func decodeInventory(t *testing.T, output []byte) inventoryReport {
	t.Helper()
	var report inventoryReport
	decoder := json.NewDecoder(bytes.NewReader(output))
	decoder.DisallowUnknownFields()
	if err := decoder.Decode(&report); err != nil {
		t.Fatalf("inventory JSON: %v\n%s", err, output)
	}
	if err := decoder.Decode(new(any)); err != io.EOF {
		t.Fatalf("expected exactly one JSON document, got %v", err)
	}
	if report.Schema != "rosie.rt-core.inventory.v1" {
		t.Fatalf("schema: %q", report.Schema)
	}
	return report
}

func inventoryCalls(t *testing.T) []string {
	t.Helper()
	data, err := os.ReadFile(os.Getenv("INVENTORY_CALLS"))
	if err != nil {
		t.Fatal(err)
	}
	return strings.Split(strings.TrimSuffix(string(data), "\n"), "\n")
}

// Unverified synthetic model: 18/1 reduction, 131072 motor counts/rev, +/-2 rad
// and 0.5 rad/s URDF bounds. Values are independent of the compiler outputs.
const robotFixtureJSON = `{
 "schema":"rosie.robot-definition.v1","robot_id":"fixture","name":"Fixture","revision":1,"revision_source":"unverified test revision",
 "defaults":{"drive_profile":"test","home_policy":"optional","coordinate_evidence_mode":"multi_turn"},
 "joints":[{"name":"axis_0","urdf_joint":"J1","kind":"arm","gear_ratio":{"numerator":18,"denominator":1,"source":"unverified test reduction","numerator_source":"unverified test reduction","denominator_source":"unverified test reduction"},"encoder_battery":"present","encoder_battery_source":"unverified synthetic battery","motor_encoder_counts_per_rev":131072,"motor_encoder_counts_per_rev_source":"unverified test resolution","sign":-1,"sign_source":"unverified test direction","cartesian_participates":true}],
 "calibration":{"identity":"home_calibration","note":"Offsets live in a separate Home record with its own revision; this file carries none."}}
`

// Unverified synthetic geometry: one independent revolute joint, in radians.
const robotFixtureURDF = `<robot name="fixture"><joint name="base" type="fixed"/><joint name="J1" type="revolute"><limit lower="-2" upper="2" velocity="0.5"/></joint><transmission><joint name="J1"/></transmission></robot>`

type robotFixture struct {
	root, profiles, urdf string
	machine, definition  object
	// dir is the fixture's description directory, <repository>/robot_description/robots/fixture.
	dir string
}

func newRobotFixture(t *testing.T, native bool) robotFixture {
	t.Helper()
	repository := buildTemp(t)
	root := filepath.Join(repository, "rt-core")
	f := fresh()
	profile := asObject(f.get("profile"))
	if native {
		profile = asObject(decode(readTestFile(t, "config/drives/a6ec.json")))
	}
	writeTestFile(t, filepath.Join(root, "config/drives/test.json"), canonical(profile), 0o644)
	machine := asObject(f.get("machine"))
	machine = asObject(replace(replace(machine, []string{"max_motor_rpm"}, nil, true), []string{"drive_speed_limit_motor_rpm"}, nil, true))
	limits := asObject(asObject(asArray(machine.get("axes"))[0]).get("limits"))
	limits = asObject(replace(replace(limits, []string{"min"}, nil, true), []string{"max"}, nil, true))
	limits = asObject(replace(replace(limits, []string{"max_velocity"}, nil, true), []string{"jog_acceleration"}, nil, true))
	machine.set("axes", []any{object{{"robot_joint", "axis_0"}, {"slave_position", json.Number("0")}, {"limits", limits}}})
	definition := asObject(decode([]byte(robotFixtureJSON)))
	dir := filepath.Join(repository, "robot_description", "robots", "fixture")
	fixture := robotFixture{root, filepath.Join(root, "config/drives"), filepath.Join(dir, "robot.urdf"), machine, definition, dir}
	writeTestFile(t, fixture.urdf, robotFixtureURDF, 0o644)
	writeTestFile(t, filepath.Join(dir, "config.json"), `{"schema":"rosie.robot-config.v1","model_id":"fixture","planning":{"joints":{"J1":{"velocity_rad_s":0.25,"acceleration_rad_s2":1}}}}`, 0o644)
	fixture.bind(t)
	return fixture
}

// bind writes the definition into the description and pins it.
func (f *robotFixture) bind(t *testing.T) {
	t.Helper()
	writeTestFile(t, filepath.Join(f.dir, robotdesc.Definition), canonical(f.definition), 0o644)
	f.pin(t)
}

// pin registers every file in the description directory in its manifest,
// computes the identity, and points the machine at it -- what a person does
// with robot_description/tools/manifest.py and the identity tool.
func (f *robotFixture) pin(t *testing.T) {
	t.Helper()
	var files []string
	err := filepath.WalkDir(f.dir, func(path string, entry os.DirEntry, err error) error {
		if err != nil || entry.IsDir() {
			return err
		}
		rel, _ := filepath.Rel(f.dir, path)
		if rel != robotdesc.Manifest {
			files = append(files, filepath.ToSlash(rel))
		}
		return nil
	})
	if err != nil {
		t.Fatal(err)
	}
	sort.Strings(files)
	quoted := make([]string, len(files))
	for i, rel := range files {
		quoted[i] = strconv.Quote(rel)
	}
	writeTestFile(t, filepath.Join(f.dir, robotdesc.Manifest), `{"schema":"`+robotdesc.ManifestSchema+`","model_id":"fixture","files":[`+strings.Join(quoted, ",")+`]}`, 0o644)
	identity, err := robotdesc.RobotDescriptionSHA256(f.dir)
	if err != nil {
		t.Fatal(err)
	}
	ref, _ := f.machine.optional("robot", nil).(object)
	if ref == nil {
		ref = object{}
	}
	ref.set("robot_description", object{{"path", "robot_description/robots/fixture"}, {"sha256", identity}})
	f.machine.set("robot", ref)
}

// stripRobotTestTravel migrates a historical assembled-robot test input to
// authoritative URDF travel. Do not use for standalone null-URDF motor models.
func stripRobotTestTravel(machine object) object {
	machine = asObject(replace(replace(machine, []string{"max_motor_rpm"}, nil, true), []string{"drive_speed_limit_motor_rpm"}, nil, true))
	axes := asArray(machine.get("axes"))
	for i, value := range axes {
		axis := asObject(replace(value, []string{"coordinate_evidence_mode"}, nil, true))
		limits := replace(replace(axis.get("limits"), []string{"min"}, nil, true), []string{"max"}, nil, true)
		limits = replace(replace(limits, []string{"max_velocity"}, nil, true), []string{"jog_acceleration"}, nil, true)
		axis.set("limits", limits)
		axes[i] = axis
	}
	machine.set("axes", axes)
	return machine
}

func (f robotFixture) compile() (Compiled, error) {
	return Compile([]byte(canonical(f.machine)), f.profiles)
}

func (f robotFixture) joint() object { return asObject(asArray(f.definition.get("joints"))[0]) }

func (f robotFixture) axis() object { return asObject(asArray(f.machine.get("axes"))[0]) }

func (f robotFixture) setAxis(key string, value any) {
	a := f.axis()
	a.set(key, value)
	f.machine.set("axes", []any{a})
}

func (f robotFixture) setJoint(key string, value any) {
	j := f.joint()
	j.set(key, value)
	f.definition.set("joints", []any{j})
}

// setRobotDescriptionPin changes one field of a robot block's robot_description pin in place.
func setRobotDescriptionPin(robot object, key string, value any) {
	pin := asObject(robot.get("robot_description"))
	pin.set(key, value)
	robot.set("robot_description", pin)
}
