package rtctl

import (
	"regexp"
	"strings"
	"testing"
)

// vendorHM35Transaction is the GradientOS two-stage HM35 transaction for the
// A6-EC profile as compiled from config/drives/a6ec.json on origin/main
// (014b12c2). Vendor-confirmed sequence; do not modify (see rt-core/AGENTS.md).
// A different transaction requires a vendor-confirmed change to a6ec.json,
// never a dwell, preconditioning stage, reordered write or per-drive variant.
const vendorHM35Transaction = "steady_state_mode|8;commissioning_mode|6;truth_source|24700|0|i32;" +
	"op|set_mode|6;op|write_sdo|24806|0|u8|0;op|write_sdo|24700|0|i32|0;" +
	"op|write_sdo|24728|0|i8|35;op|controlword_sequence|6,7,15;op|wait_statusword|551|8264;" +
	"op|controlword_sequence|31;op|wait_statusword|36864|8192;op|refresh_truth;" +
	"op|restore_mode|8;op|release_service_override"

// TestA6ecHomeTransactionMatchesVendorGolden pins the compiled Home transaction
// and the native worker's golden test to the same vendor-confirmed sequence:
// set_mode 6; 60E6=0; 607C=0; 6098=35; CW 6,7,15; wait set 0x0227 clear
// 0x2048; CW31; wait set 0x9000 clear 0x2000; refresh; restore_mode 8; release.
func TestA6ecHomeTransactionMatchesVendorGolden(t *testing.T) {
	profile := asObject(decode(readTestFile(t, "config/drives/a6ec.json")))
	if got := renderHome(profile.get("native_home")); got != vendorHM35Transaction {
		t.Fatalf("a6ec.json Home transaction differs from the vendor-confirmed golden:\n got %s\nwant %s", got, vendorHM35Transaction)
	}
	source := string(readTestFile(t, "tests/commissioning_worker_tests.cpp"))
	declaration := regexp.MustCompile(`(?s)constexpr const char kVendorHm35Transaction\[\] =(.*?);\n`).FindStringSubmatch(source)
	if declaration == nil {
		t.Fatal("native worker golden kVendorHm35Transaction is missing")
	}
	var native strings.Builder
	for _, piece := range regexp.MustCompile(`"([^"]*)"`).FindAllStringSubmatch(declaration[1], -1) {
		native.WriteString(piece[1])
	}
	if native.String() != vendorHM35Transaction {
		t.Fatalf("native worker golden differs:\n got %s\nwant %s", native.String(), vendorHM35Transaction)
	}
}
