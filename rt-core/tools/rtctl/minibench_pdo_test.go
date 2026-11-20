package rtctl

import (
	"crypto/sha256"
	"fmt"
	"regexp"
	"strings"
	"testing"
)

// TestMiniBenchSelectedRobotLogParity pins every constructor log byte for nine
// already-selected drives apart from the explicit WKC convention labels,
// preventing Rx ownership from changing configuration or counter log bytes.
func TestMiniBenchSelectedRobotLogParity(t *testing.T) {
	t.Parallel()
	log := runA6BringupFixture(t, buildA6BringupFixture(t), "a6ec")
	for _, scenario := range []string{"AlreadySelected", "AssignmentWriteOrdering", "RestoreAssignment",
		"RestoreAlreadySelectedNoWrite", "RestoreAfterRefusedBringup", "StartupAssignmentConflict",
		"IgnoredAssignment", "LateAssignment", "MailboxRefused", "DownloadRefused", "TransientRead",
		"EntryMismatch", "CountMismatch", "AssignmentCountMismatch", "WrongState", "AutomaticRescan",
		"RescanFastCompletion", "RescanBoundedRefusal", "RescanReverifyRefusal"} {
		if !strings.Contains(log, "PASS: TestPREOP"+scenario+"Rx\n") {
			t.Fatalf("missing RxPDO regression: %s", scenario)
		}
	}
	_, section, ok := strings.Cut(log, "BEGIN: TestPREOPAlreadySelected axis=0\n")
	if !ok {
		t.Fatal("missing selected scenario")
	}
	section, _, ok = strings.Cut(section, "PASS: TestPREOPAlreadySelected\n")
	if !ok {
		t.Fatal("missing selected completion")
	}
	// Linear tracking is already active after the simulated power cycle; the
	// two standard position-factor objects are written on change.
	if !strings.Contains(section, "[rosie-rt-core] EtherCAT startup_sdo_summary axis=0 slave_pos=0 written=2 skipped=1 persisted=2 ram_only=0 unread=2 storage_window=0\n") ||
		strings.Contains(section, "startup_sdo_summary axis=1 ") {
		t.Fatalf("write-on-change summary changed:\n%s", section)
	}
	// Measured pre-Rx-ownership constructor bytes at e0c155eb, apart from the
	// write-on-change journal fields, plus main's DC records: the fixture now
	// exercises DC and logs its half-cycle phase. Removing only slave_config_dc /
	// after_slave_config_dc records reproduces the linear-mode branch checksum
	// 2b3483db exactly (checked at the 2026-10-31 merge of main).
	const want = "f72073f241b81c0b0b1c0d07066d78cb4387f83bdf9f4823f69d1b8e61b78b3f"
	digest := func(log string) string {
		return fmt.Sprintf("%x", sha256.Sum256([]byte(normalizeMiniBenchWKCRecords(normalizeStartupWriteOnChange(log)))))
	}
	if got := digest(section); got != want {
		t.Fatalf("robot log SHA256=%s want=%s\n%s", got, want, normalizeMiniBenchWKCRecords(section))
	}
	t.Run("misplaced_label", func(t *testing.T) {
		if !strings.Contains(section, "slave_pos=0") {
			t.Fatal("review mutation target absent")
		}
		mutated := strings.Replace(section, "slave_pos=0", "slave_pos=0: two directions per axis, held devices zero", 1)
		if digest(mutated) == want {
			t.Fatal("WKC label appended to unrelated slave_pos escaped the checksum")
		}
	})
}

// normalizeStartupWriteOnChange removes the fields write-on-change added to the
// startup write records of an unread (fresh) drive and its summary line, so
// every other constructor byte stays pinned.
func normalizeStartupWriteOnChange(log string) string {
	written := regexp.MustCompile(`(?m)^(\[rosie-rt-core\] EtherCAT config phase=slave_config_sdo [^\n]* ok) prior=unread storage=stored$`)
	summary := regexp.MustCompile(`(?m)^\[rosie-rt-core\] EtherCAT startup_sdo_summary [^\n]*\n`)
	return summary.ReplaceAllString(written.ReplaceAllString(log, "${1}"), "")
}

// normalizeMiniBenchWKCRecords removes only labels on the constructor's WKC
// records, preserving counts and unrelated fields so misplaced text fails parity.
func normalizeMiniBenchWKCRecords(log string) string {
	wkc := regexp.MustCompile(`(?m)^(\[rosie-rt-core\] EtherCAT phase=[^\n]* domain_wc=[0-9]+ )wkc simulator ([0-9]+/[0-9]+): two directions per axis, held devices zero$`)
	active := regexp.MustCompile(`(?m)^(\[rosie-rt-core\] IgH libecrt active \(num_axes=[0-9]+, expected_wkc=[0-9]+\)) backend=simulator$`)
	return active.ReplaceAllString(wkc.ReplaceAllString(log, "${1}wkc=${2}"), "${1}")
}
