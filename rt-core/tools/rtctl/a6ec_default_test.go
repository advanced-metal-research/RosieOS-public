package rtctl

import (
	"bytes"
	"crypto/sha256"
	"encoding/json"
	"fmt"
	"reflect"
	"strings"
	"testing"
)

// Measured raw tx_layout bytes at 7f834034, including entry provenance and LF whitespace.
const a6ecLegacyLayoutSHA256 = "1cd88d11f3514a6f9d90baa2d62e5c8536c55a95b18790b80d38fe8a8e96360b"

// Measured fixed 0x1B04 survey: the nine-motor bench's servo host 2026-09-14; 0x60F4 and 0x606C are I32.
const a6ecB04Layout = "err|24639|0|16;sw|24641|0|16;pos|24676|0|32;torque|24695|0|16;mode_disp|24673|0|8;following_error|24820|0|32;tp_status|24761|0|16;tp_pos1|24762|0|32;tp_pos2|24764|0|32;velocity_actual|24684|0|32"

// Measured fixed 0x1B02 survey: the nine-motor bench's servo host 2026-09-13; 0x60FD is U32.
const a6ecB02Layout = "err|24639|0|16;sw|24641|0|16;pos|24676|0|32;torque|24695|0|16;mode_disp|24673|0|8;tp_status|24761|0|16;tp_pos1|24762|0|32;tp_pos2|24764|0|32;di|24829|0|32"

// Measured the nine-motor bench's servo host mapping hand-off: nine first-attempt physical verifications,
// nine stale-cache misses, then mapping=verified at the cited rescan observation.
const a6ecB04HandoffSource = "On the nine-motor bench's servo host on 2026-09-15, physical PREOP readback verified the 0x1B04 assignment for all nine AS715N axes on the first attempt; the nine scanned-cache misses were resolved by the daemon's automatic rescan, with no mailbox retry during the mapping hand-off (docs/evidence/bench-ninemotor-2026-09-15/daemon-8deed804-b04.log:234-347,584-592); the kernel log contained neither 'wrong SDO' nor 'SDO configuration failed' (docs/evidence/bench-ninemotor-2026-09-15/dmesg-ethercat-8deed804-b04.txt:1-60)."

// TestA6B04HandoffProvenance keeps physical readback distinct from stale cache
// observations and preserves every profile byte outside the corrected sentence.
func TestA6B04HandoffProvenance(t *testing.T) {
	// Measured original sentence and raw profile digests at ab129cdc.
	const previous = "0x1B04 assignment was accepted on nine AS715N drives on the nine-motor bench's servo host on 2026-09-15 with the daemon-owned PREOP write and automatic IgH rescan; all nine reached OP with zero unavailable readbacks (docs/evidence/bench-ninemotor-2026-09-15/)."
	for _, tc := range []struct{ name, originalSHA256 string }{
		// Updated after removing the drive's duplicate robot acceleration cap, then
		// after moving the former native 50-count Start floor into the profile.
		// a6ec updated 2026-10-11 for its fault_reset classification block, and
		// 2026-10-31 for the linear-mode migration (absolute pair field,
		// battery-absent tolerance, retired wrap keys), main's monitor list and
		// the bench profiles' 50 rad/s^2 acceleration.
		{"a6ec", "4bbb7d08475761882975af17bd97a45bd55737673bb9bb9780c5dc26bc11d7bb"},
		{"a6ec_bench_1to1", "b380d257be68f5daa4e6ff3332834d154dd2deb1362030fa59990fc103c0c989"},
		{"a6ec_bench_1to1_fast", "ae4768997716b048bb142a306c7736b22cb5aa14f63f434ef18f3069ce073fe6"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			data := readTestFile(t, "config/drives/"+tc.name+".json")
			var profile struct {
				EtherCAT map[string]json.RawMessage `json:"ethercat"`
			}
			if err := json.Unmarshal(data, &profile); err != nil {
				t.Fatal(err)
			}
			for _, field := range []string{"_tx_layout_source", "_fixed_mapping_kernel", "_tx_preset_source"} {
				var source string
				if err := json.Unmarshal(profile.EtherCAT[field], &source); err != nil {
					t.Fatal(err)
				}
				if strings.Count(source, a6ecB04HandoffSource) != 1 || strings.Contains(source, "zero unavailable readbacks") {
					t.Errorf("%s must distinguish physical verification, scanned cache and mailbox retries", field)
				}
			}
			restored := bytes.ReplaceAll(data, []byte(a6ecB04HandoffSource), []byte(previous))
			if got := fmt.Sprintf("%x", sha256.Sum256(restored)); got != tc.originalSHA256 {
				t.Fatalf("bytes outside hand-off sentence changed: %s", got)
			}
		})
	}
}

// TestA6RobotDefaultLayout preserves the surveyed entry order and every unrelated
// robot policy; the explicit DI alternative cannot silently acquire a new layout.
func TestA6RobotDefaultLayout(t *testing.T) {
	read := func(name string) map[string]json.RawMessage {
		t.Helper()
		var p struct {
			EtherCAT map[string]json.RawMessage `json:"ethercat"`
		}
		if err := json.Unmarshal(readTestFile(t, "config/drives/"+name+".json"), &p); err != nil {
			t.Fatal(err)
		}
		return p.EtherCAT
	}
	robot, bench, alternative := read("a6ec"), read("a6ec_bench_1to1"), read("a6ec_1b02")
	if string(robot["tx_pdo"]) != "6916" || string(bench["tx_pdo"]) != "6916" || string(alternative["tx_pdo"]) != "6914" {
		t.Fatal("incorrect default or explicit alternative assignment")
	}
	if !bytes.Equal(robot["tx_layout"], bench["tx_layout"]) {
		t.Fatal("robot and bench layouts differ in entry bytes, order, widths or signed semantic names")
	}
	if got := fmt.Sprintf("%x", sha256.Sum256(alternative["tx_layout"])); got != a6ecLegacyLayoutSHA256 {
		t.Fatalf("pre-change DI layout bytes changed: %s", got)
	}
	var original, current map[string]any
	if err := json.Unmarshal(readTestFile(t, "config/drives/a6ec_1b02.json"), &original); err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(readTestFile(t, "config/drives/a6ec.json"), &current); err != nil {
		t.Fatal(err)
	}
	for _, p := range []map[string]any{original, current} {
		delete(p, "id")
		delete(p, "label")
		delete(p, "max_acceleration")
		delete(p, "_max_acceleration_unverified")
		e := p["ethercat"].(map[string]any)
		for _, field := range []string{"tx_pdo", "tx_layout", "_tx_layout_source", "_fixed_mapping_kernel", "_tx_preset_source"} {
			delete(e, field)
		}
	}
	if !reflect.DeepEqual(original, current) {
		t.Fatal("preset switch changed unrelated robot values")
	}
	for _, tc := range []struct{ profile, layout string }{{"a6ec", a6ecB04Layout}, {"a6ec_bench_1to1", a6ecB04Layout}, {"a6ec_1b02", a6ecB02Layout}} {
		f := fresh()
		f.put("profile", string(readTestFile(t, "config/drives/"+tc.profile+".json")))
		// Unverified 100:1 software-ratio fixture with standard motor counts.
		f.put("machine.axes.0.wire_counts_per_rev", "131072")
		f.put("machine.axes.0.motor_encoder_counts_per_rev", "131072")
		f.put("machine.axes.0.wire_revs_per_axis_rev", "100")
		c, err := f.compile()
		if err != nil {
			t.Fatal(err)
		}
		if !strings.Contains(c.AxisProfiles, "tx_layout="+tc.layout+"\n") {
			t.Fatal(c.AxisProfiles)
		}
	}
}

// TestA6BenchExampleAcquisitionAllowance keeps the 1:1 example compilable with
// the declared continuous evidence.
func TestA6BenchExampleAcquisitionAllowance(t *testing.T) {
	data := readTestFile(t, "config/machines/a6ec-bench.example.json")
	c, err := Compile(data, "../../config/drives")
	if err != nil {
		t.Fatal(err)
	}
	m := asObject(decode(data))
	a := asObject(asArray(m.get("axes"))[0])
	if a.get("profile") != "a6ec_bench_1to1" || !numericEqual(a.get("wire_counts_per_rev"), 131072) {
		t.Fatal(a)
	}
	p := asObject(asObject(decode(readTestFile(t, "config/drives/a6ec_bench_1to1.json"))).get("coordinate_evidence"))
	// Declared unverified bench bounds: ceil(262144 * 128000000 / 1e9) + 16 = 33571 counts.
	if !numericEqual(p.get("maximum_physical_speed_counts_per_second"), 262144) ||
		!numericEqual(p.get("maximum_acquisition_duration_ns"), 128000000) ||
		!numericEqual(p.get("maximum_feedback_error_counts"), 16) {
		t.Fatal("bench evidence changed; requalify the 33571-count allowance")
	}
	if !strings.Contains(c.AxisProfiles, "coordinate_evidence=262144|3000000000|16|16|a6ec_sdo|1500000000\n") {
		t.Fatal(c.AxisProfiles)
	}
}
