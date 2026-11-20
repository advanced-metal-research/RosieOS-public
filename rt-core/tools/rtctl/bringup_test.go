package rtctl

import (
	"os"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

// Unverified policy fixtures pin native defaults independently of the compiler.
const defaultBusBringup = `{"startup_passive_ms":0,"explicit_pdo_config":false,
"disable_output_watchdog":false,"no_dc":false,"wait_before_safeop_ms":250,
"preop_safeop_timeout_ms":5000,"safeop_op_timeout_ms":5000}`

// Unverified nondefault fixtures use explicit milliseconds and boolean policies.
var bringupCases = []struct {
	key, value, flag, field, before, after string
}{
	{"startup_passive_ms", "17", "--startup-passive-ms", "startup_passive_ms", "0", "17"},
	{"explicit_pdo_config", "true", "--explicit-pdo-config", "explicit_pdo_config", "0", "1"},
	{"disable_output_watchdog", "true", "--disable-output-watchdog", "disable_output_watchdog", "0", "1"},
	{"no_dc", "true", "--no-dc", "use_dc", "1", "0"},
	{"wait_before_safeop_ms", "300", "--wait-before-safeop-ms", "wait_before_safeop_ms", "250", "300"},
	{"preop_safeop_timeout_ms", "6000", "--preop-safeop-timeout-ms", "preop_to_safeop_timeout_ms", "5000", "6000"},
	{"safeop_op_timeout_ms", "7000", "--safeop-op-timeout-ms", "safeop_to_op_timeout_ms", "5000", "7000"},
}

func readTestFile(t *testing.T, path string) []byte {
	t.Helper()
	data, err := os.ReadFile(filepath.Join(testRoot(t), path))
	if err != nil {
		t.Fatal(err)
	}
	return data
}

// TestBusBringupDefaults keeps explicit defaults and measurement annotations
// byte-compatible with absent policy, preventing unnecessary evidence invalidation.
func TestBusBringupDefaults(t *testing.T) {
	baseline, err := fresh().compile()
	if err != nil {
		t.Fatal(err)
	}
	for _, block := range []string{"{}", defaultBusBringup} {
		f := fresh()
		f.put("machine.bus_bringup", block)
		for _, tc := range bringupCases {
			f.put("machine.bus_bringup."+tc.key+"_unverified", `"unverified bench measurement"`)
		}
		before := canonical(f.get("machine"))
		compiled, err := f.compile()
		if err != nil {
			t.Fatal(err)
		}
		if !reflect.DeepEqual(compiled, baseline) {
			t.Fatalf("default policy changed argv or digests: %+v", compiled)
		}
		if canonical(f.get("machine")) != before {
			t.Fatal("compilation mutated the input")
		}
	}
}

// TestBusBringupMachineIdentity binds each effective policy to machine identity
// and exact argv while keeping host identity independent of firmware choices.
func TestBusBringupMachineIdentity(t *testing.T) {
	baseline, err := fresh().compile()
	if err != nil {
		t.Fatal(err)
	}
	for _, tc := range bringupCases {
		t.Run(tc.key, func(t *testing.T) {
			f := fresh()
			f.put("machine.bus_bringup", `{"`+tc.key+`":`+tc.value+`}`)
			compiled, err := f.compile()
			if err != nil {
				t.Fatal(err)
			}
			wantArgs := append(append([]string{}, baseline.Argv...), tc.flag)
			if tc.value != "true" {
				wantArgs = append(wantArgs, tc.value)
			}
			if !reflect.DeepEqual(compiled.Argv, wantArgs) {
				t.Fatalf("argv: %q want %q", compiled.Argv, wantArgs)
			}
			wantIdentity := strings.Replace(goldenMachineText, tc.field+"="+tc.before+"\n", tc.field+"="+tc.after+"\n", 1)
			if compiled.MachineIdentity != wantIdentity || compiled.MachineSHA256 != identityDigest(wantIdentity) {
				t.Fatalf("machine policy missing or encoded incorrectly: %s", compiled.MachineIdentity)
			}
			if compiled.DeploymentSHA256 != goldenDeploymentSHA256 || compiled.SHA256 == goldenConfigurationSHA256 {
				t.Fatal("machine/configuration/deployment digest partition violated")
			}
			f.put("machine.bus_bringup."+tc.key+"_unverified", `"unverified measurement to take"`)
			noted, err := f.compile()
			if err != nil || !reflect.DeepEqual(noted, compiled) {
				t.Fatalf("measurement note changed effective identity: %v", err)
			}
		})
	}
}

// TestBusBringupValidation rejects malformed overrides before publication and
// pins integer units and native API boundaries independently of compiler tables.
func TestBusBringupValidation(t *testing.T) {
	for _, block := range []string{"null", "[]", "true", "0", `"default"`, `{"startup_pasive_ms":1}`, `{"startup_passive_ms_unverified":false}`, `{"unknown_unverified":"note"}`} {
		f := fresh()
		f.put("machine.bus_bringup", block)
		if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "bus_bringup") {
			t.Fatalf("block %s: expected labeled rejection, got %v", block, err)
		}
	}
	for _, tc := range bringupCases {
		badValues := []string{"null", `"1"`, "1.0", "[]", "{}"}
		validValues := []string{"false", "true"}
		if strings.HasSuffix(tc.key, "_ms") {
			badValues = append(badValues, "-1", "true", "1e3", "4294967296")
			validValues = []string{"0", "1", "4294967295"}
			if tc.key == "wait_before_safeop_ms" {
				badValues = append(badValues, "2147483648")
				validValues = []string{"0", "1", "2147483647"}
			}
		} else {
			badValues = append(badValues, "0", "1", `"false"`)
		}
		for _, value := range badValues {
			t.Run(tc.key+"/reject/"+value, func(t *testing.T) {
				f := fresh()
				f.put("machine.bus_bringup", `{"`+tc.key+`":`+value+`}`)
				if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "bus_bringup."+tc.key) {
					t.Fatalf("expected labeled rejection, got %v", err)
				}
			})
		}
		for _, value := range validValues {
			t.Run(tc.key+"/accept/"+value, func(t *testing.T) {
				f := fresh()
				f.put("machine.bus_bringup", `{"`+tc.key+`":`+value+`}`)
				if _, err := f.compile(); err != nil {
					t.Fatal(err)
				}
			})
		}
	}
}

// TestBusBringupNoDC admits nonquantized periods only when DC is explicitly
// disabled, matching native admission instead of hiding a firmware constraint.
func TestBusBringupNoDC(t *testing.T) {
	f := fresh()
	f.put("machine.cycle_ns", "1000001")
	if _, err := f.compile(); err == nil || !strings.Contains(err.Error(), "cycle incompatible") {
		t.Fatalf("DC accepted nonquantized period: %v", err)
	}
	f.put("machine.bus_bringup", `{"no_dc":true}`)
	if _, err := f.compile(); err != nil {
		t.Fatal(err)
	}
}

// TestBusBringupBenchNotes requires every example value to retain its native
// default and a measurement note, preventing placeholders from claiming qualification.
func TestBusBringupBenchNotes(t *testing.T) {
	for _, name := range []string{"config/machines/a6ec-bench.example.json", "tools/rtctl/testdata/retired/bench-ninemotor-a6ec.json"} {
		t.Run(name, func(t *testing.T) {
			checkBusBringupBenchNotes(t, name)
		})
	}
}

func checkBusBringupBenchNotes(t *testing.T, name string) {
	t.Helper()
	data := readInheritedRobotTestFile(t, name)
	block := asObject(asObject(decode(data)).get("bus_bringup"))
	defaults := asObject(decode([]byte(defaultBusBringup)))
	if len(block) != 14 {
		t.Fatalf("want seven values and seven notes, got %d fields", len(block))
	}
	for _, field := range defaults {
		if block.get(field.key) != field.value {
			t.Errorf("%s: default changed", field.key)
		}
		note, ok := block.get(field.key + "_unverified").(string)
		if field.key == "explicit_pdo_config" {
			if !ok || !strings.Contains(note, "0x06010002") || !strings.Contains(note, "2026-09-13") {
				t.Error("fixed PDO selection requires the read-only abort bench citation")
			}
			continue
		}
		if !ok || !strings.Contains(note, "unverified") || !strings.Contains(note, "bench") || !strings.Contains(note, "machine semantics") {
			t.Errorf("%s: missing machine classification or bench measurement note", field.key)
		}
	}
}

func TestNineMotorBenchNoteScope(t *testing.T) {
	machine := asObject(decode(readInheritedRobotTestFile(t, "tools/rtctl/testdata/retired/bench-ninemotor-a6ec.json")))
	profiles := filepath.Join(testRoot(t), "config/drives")
	noted, err := Compile([]byte(canonical(machine)), profiles)
	if err != nil {
		t.Fatal(err)
	}
	var strip func(any) any
	strip = func(value any) any {
		switch v := value.(type) {
		case object:
			out := object{}
			for _, field := range v {
				if !strings.HasSuffix(field.key, "_source") && !strings.HasSuffix(field.key, "_unverified") {
					out = append(out, member{field.key, strip(field.value)})
				}
			}
			return out
		case []any:
			out := make([]any, len(v))
			for i, item := range v {
				out[i] = strip(item)
			}
			return out
		default:
			return value
		}
	}
	plain, err := Compile([]byte(canonical(strip(machine))), profiles)
	if err != nil || !reflect.DeepEqual(noted.Argv, plain.Argv) || noted.AxisProfiles != plain.AxisProfiles ||
		noted.MachineSHA256 != plain.MachineSHA256 {
		t.Fatalf("notes changed effective machine inputs: %v", err)
	}
	// Cited compiler.go identity contract: non-bus metadata belongs to the frozen
	// configuration digest; preserving it prevents silently changing existing identities.
	if noted.SHA256 == plain.SHA256 {
		t.Fatal("configuration identity unexpectedly discarded non-bus metadata")
	}
	// Startup descriptors remain strict on the compatibility path; robot cells
	// reject the entire startup override block before it can reach that schema.
	machine = asObject(decode(readTestFile(t, "tools/rtctl/testdata/robotdef/legacy-rosie_1400_v3.json")))
	a := asObject(asArray(machine.get("axes"))[0])
	startup := asObject(a.get("startup"))
	startup.set("a6ec_encoder_position_tracking_mode_source", "unverified annotation")
	a.set("startup", startup)
	if _, err := Compile([]byte(canonical(machine)), profiles); err == nil {
		t.Fatal("unknown startup note was silently accepted as an SDO")
	}
}
