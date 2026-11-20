package rtctl

import (
	"fmt"
	"path/filepath"
	"reflect"
	"strings"
	"testing"
)

// Build the live constructor against the recording simulator so normal
// simulator setup cannot hide physical assignment or scanned-cache differences.
func buildA6BringupFixture(t *testing.T) string {
	t.Helper()
	binary, output, err := nativeTestBinary(t, "bus-bringup-tests", func(binary string) []string {
		return []string{"-std=c++17", "-O0", "-UNDEBUG", "-Iinclude", "-I.",
			"tests/bus_bringup_tests.cpp", "-o", binary, "-pthread"}
	})
	if err != nil {
		t.Fatalf("build live bring-up fixture: %v\n%s", err, output)
	}
	return binary
}

// Compile each real profile and require every constructor scenario to exit
// successfully; assertions in the recording master enforce mailbox ownership.
// Tests asserting on the same profile share one run of the fixture.
func runA6BringupFixture(t *testing.T, binary, profile string) string {
	t.Helper()
	f := fresh()
	f.put("profile", string(readTestFile(t, "config/drives/"+profile+".json")))
	f.put("profile.ethercat.restore_assignment_on_exit", "true")
	f.put("machine.axes.0.wire_counts_per_rev", "131072")
	f.put("machine.axes.0.motor_encoder_counts_per_rev", "131072")
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	_, args, err := writeCompiled(compiled, buildTemp(t))
	if err != nil {
		t.Fatal(err)
	}
	output, err := nativeRunOnce(binary, compiled.SHA256, args)
	if err != nil {
		t.Fatalf("live bring-up: %v\n%s", err, output)
	}
	return string(output)
}

// TestA6LiveBringupConfigurationCalls executes the live constructor with a
// recording master, preventing simulator setup from hiding robot remapping.
func TestA6LiveBringupConfigurationCalls(t *testing.T) {
	t.Parallel()
	binary := buildA6BringupFixture(t)
	for _, profile := range []string{"a6ec", "a6ec_bench_1to1", "a6ec_1b02"} {
		t.Run(profile, func(t *testing.T) {
			t.Parallel()
			log := runA6BringupFixture(t, binary, profile)
			if profile == "a6ec_1b02" {
				// Measured legacy B02 log contract from 372e9b25, retained by the alternative.
				for axis := 0; axis < 9; axis++ {
					want := fmt.Sprintf("[rosie-rt-core] EtherCAT config phase=slave_config_pdos axis=%d slave_pos=%d profile=0x1702/0x1b02 mode=fixed_default readback=pending\n", axis, axis)
					if !strings.Contains(log, want) {
						t.Fatalf("robot log parity: missing %q", want)
					}
				}
				const notice = "[rosie-rt-core] EtherCAT fixed_mapping kernel=IgH-1.6.9 mapping_write=kernel_unavoidable registration=scanned_mapping userspace_mapping_writes=0 bench_abort=0x06010002 bench=2026-09-13 kernel_clear_attempts=1_per_PDO_per_configuration readback=required\n"
				if !strings.Contains(log, notice) || strings.Contains(log, "profile=0x1702/0x1b02 mode=fixed_default readback=PREOP_required") {
					t.Fatal("robot mapping notice or pending readback changed")
				}
			} else {
				for axis := 0; axis < 9; axis++ {
					want := "profile=0x1702/0x1b04 mode=fixed_default readback=PREOP_required\n"
					prefix := fmt.Sprintf("phase=slave_config_pdos axis=%d slave_pos=%d ", axis, axis)
					if !strings.Contains(log, prefix+want) {
						t.Fatalf("B04 PREOP contract: missing %q", prefix+want)
					}
				}
			}
			for _, want := range []string{
				"PASS: TestPREOPAlreadySelected", "PASS: TestPREOPAssignmentWriteOrdering",
				"PASS: TestExplicitAssignmentAtActivation", "PASS: TestFixedPresetRefusesActivationOnlyAssignment",
				"PASS: TestPREOPIgnoredAssignment", "PASS: TestPREOPLateAssignment",
				"PASS: TestPREOPMailboxRefused", "PASS: TestPREOPDownloadRefused",
				"PASS: TestPREOPRestoreAssignment", "PASS: TestPREOPAutomaticRescan",
				"PASS: TestPREOPRescanBoundedRefusal", "PASS: TestPREOPRescanReverifyRefusal",
				"phase=PREOP result=verified", "assignment=unchanged phase=PREOP",
				"assignment=written phase=PREOP", "rescan_required=1 activation=blocked",
			} {
				if !strings.Contains(log, want) {
					t.Fatalf("missing %q: %s", want, log)
				}
			}
		})
	}
}

// Both operator entry points must describe the mailbox exclusion and automatic
// hand-off, preventing a stale manual-rescan procedure from undoing a preset.
func TestA6BringupProcedureMailboxContract(t *testing.T) {
	t.Parallel()
	for _, path := range []string{"docs/drives/a6ec/README.md", "docs/BENCH_TESTING.md"} {
		text := string(readTestFile(t, path))
		for _, required := range []string{"inhibit motion", "INIT", "PREOP", "automatic rescan",
			"operational=9/9", "domain_wc=2 wkc=18/18", "startup_ready", "500,000,000 ns",
			"restore_assignment_on_exit=false", "state=reacquired", "activation=blocked"} {
			if !strings.Contains(text, required) {
				t.Errorf("%s omits bring-up contract %q", path, required)
			}
		}
		if strings.Contains(text, "run `ethercat rescan` and restart") {
			t.Errorf("%s still requires manual preset hand-off", path)
		}
	}
	log := runA6BringupFixture(t, buildA6BringupFixture(t), "a6ec_bench_1to1")
	for _, name := range []string{"TestPREOPRescanFastCompletion", "TestPREOPRescanReverifyRefusal"} {
		for _, axis := range []int{0, 8} {
			t.Run(fmt.Sprintf("%s/axis%d", name, axis), func(t *testing.T) {
				_, section, ok := strings.Cut(log, fmt.Sprintf("BEGIN: %s axis=%d\n", name, axis))
				if !ok {
					t.Fatal("missing scenario start")
				}
				section, _, ok = strings.Cut(section, "PASS: "+name+"\n")
				if !ok {
					t.Fatal("missing scenario completion")
				}
				// Unverified fixture sequence: nine PREOP slaves, with the busy
				// pulse entirely between polls and one terminal mapping proof.
				want := []string{
					"EtherCAT phase=pdo_rescan state=released master=0 restoration=off",
					"EtherCAT phase=pdo_rescan state=queued master=0",
				}
				for _, mapping := range []string{"pending", "verified"} {
					want = append(want, "EtherCAT phase=pdo_rescan state=observed slaves=9/9 al_states=0x2 scan_busy=0 mapping="+mapping)
					for i := 0; i < 9; i++ {
						want = append(want, fmt.Sprintf("EtherCAT phase=pdo_rescan axis=%d slave_pos=%d al_state=0x2 rc=0", i, i))
					}
				}
				want = append(want, "EtherCAT phase=pdo_rescan state=reacquired master=0 rebuild=required readback=PREOP_required")
				var got []string
				for _, line := range strings.Split(section, "\n") {
					if strings.Contains(line, "EtherCAT phase=pdo_rescan ") {
						got = append(got, strings.TrimPrefix(line, "[rosie-rt-core] "))
					}
				}
				if !reflect.DeepEqual(got, want) {
					t.Fatalf("rescan sequence:\ngot %q\nwant %q", got, want)
				}
				_, after, _ := strings.Cut(section, want[len(want)-1]+"\n")
				if name == "TestPREOPRescanReverifyRefusal" {
					refused := fmt.Sprintf("ERROR: mapping_readback axis=%d phase=PREOP result=refused attempts=1 rescan_required=1 activation=blocked\n", axis)
					if strings.Count(after, refused) != 1 || strings.Contains(after, "IgH libecrt active") {
						t.Fatalf("post-rescan refusal must retain the documented failure: %s", after)
					}
				} else {
					for i := 0; i < 9; i++ {
						proof := fmt.Sprintf("EtherCAT mapping_readback axis=%d phase=PREOP result=verified attempts=1", i)
						if strings.Count(after, proof) != 1 {
							t.Fatalf("missing fresh physical proof: %s", after)
						}
					}
					if strings.Count(after, "IgH libecrt active (num_axes=9, expected_wkc=18)") != 1 {
						t.Fatalf("missing successful activation after reverification: %s", after)
					}
				}
			})
		}
	}
}

// Inventory bypasses daemon admission; require the bench procedure to exclude
// a daemon owner so a second mailbox user cannot race runtime requests.
func TestInventoryProcedureMailboxOwnership(t *testing.T) {
	text := string(readTestFile(t, "docs/BENCH_TESTING.md"))
	if !strings.Contains(text, "`rtctl inventory` uploads through the CLI outside the daemon's mailbox admission and runs only while no daemon owns the master.") {
		t.Fatal("inventory procedure must require an unowned master for CLI uploads")
	}
}

// Preserve the default artifact while accepting only an explicit boolean opt-in;
// arbitrary truthy JSON must not silently enable shutdown mailbox writes.
func TestRestoreAssignmentProfileField(t *testing.T) {
	for _, value := range []string{"true", "false", "1", `"false"`, "null"} {
		t.Run(value, func(t *testing.T) {
			f := fresh()
			f.put("profile.ethercat.restore_assignment_on_exit", value)
			compiled, err := f.compile()
			if value != "true" && value != "false" {
				if err == nil {
					t.Fatal("non-boolean restore policy accepted")
				}
				return
			}
			if err != nil {
				t.Fatal(err)
			}
			if strings.Contains(compiled.AxisProfiles, "restore_assignment_on_exit=1\n") != (value == "true") {
				t.Fatalf("restore policy not preserved: %s", compiled.AxisProfiles)
			}
		})
	}
	f := fresh()
	f.put("profile.ethercat.restore_assignment_on_exit", "true")
	f.put("machine.bus_bringup", `{"explicit_pdo_config":true}`)
	if _, err := f.compile(); err == nil {
		t.Fatal("restore policy accepted with explicit PDO configuration")
	}
}

// Pin the migrated A6 fixed-PDO artifact independently of historical rotation-mode goldens.
func TestA6FixedPDOArtifacts(t *testing.T) {
	compiled, err := Compile(readTestFile(t, "config/machines/a6ec-bench.example.json"), filepath.Join(testRoot(t), "config/drives"))
	if err != nil {
		t.Fatal(err)
	}
	for _, want := range []string{
		"tx_layout=" + a6ecB04Layout + "\n",
		"a6ec_encoder_position_tracking_mode|u16|8192|8|1|power_cycle",
		"cia402_position_factor_numerator|u32|24721|1|1",
		"coordinate_anchor_source=8256|33|35|a6ec_raw_multi_turn|raw_multi_turn_encoder|3",
	} {
		if !strings.Contains(compiled.AxisProfiles, want) {
			t.Fatalf("migrated A6 artifact missing %q", want)
		}
	}
}
