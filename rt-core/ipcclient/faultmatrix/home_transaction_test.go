//go:build linux

package faultmatrix

import (
	"bytes"
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"regexp"
	"testing"

	ipc "rosieos/rt-core/ipcclient"
)

// The simulated A6 completes HM35 on its observed CW31 rising edge. Observe the
// plant's full cycle trace and the submitted-wire audit: Home submits the
// vendor-confirmed origin/main transaction, with CW6/CW7/CW15 only in homing
// mode (no steady-mode Shutdown dwell) and the trigger following Operation
// Enabled without a stationary pretrigger dwell.
func TestHomeSubmitsVendorTransactionWithoutDwells(t *testing.T) {
	f := compileFixture(t, "simulation-rosie1420.json", nil)
	f.launch(true, "--capture-cycles", "10000")
	f.command(f.c.Home(context.Background(), 1))
	completed := ipc.HostMonotonicNS()
	f.until(completed, publicationNS, func(s ipc.Status) bool { return s.HomeValidMask == 1 && s.TimeNS > completed+f.cycle })
	raw, err := os.ReadFile(filepath.Join(f.socketDir, "cycle-capture.jsonl"))
	must(t, err)
	var firstEnabled, firstAttained uint64
	lines := bytes.Split(raw, []byte{'\n'})
	for _, line := range lines[:len(lines)-1] {
		var sample struct {
			Time uint64 `json:"t_ns"`
			Axes []struct {
				Status uint16 `json:"sw"`
			} `json:"ax"`
		}
		must(t, json.Unmarshal(line, &sample))
		if len(sample.Axes) == 0 {
			continue
		}
		sw := sample.Axes[0].Status
		if firstEnabled == 0 && sw&0x6f == 0x27 {
			firstEnabled = sample.Time
		}
		if firstAttained == 0 && sw&0x9000 == 0x9000 {
			firstAttained = sample.Time
		}
	}
	// Unverified simulator bound: 50 ms step settle, 20 ms statusword poll and the
	// profile's brake release delay fit well inside a removed 500 ms dwell.
	if firstEnabled == 0 || firstAttained < firstEnabled || firstAttained-firstEnabled >= 500000000 {
		t.Fatalf("Home trigger delayed after Operation Enabled: enabled=%d attained=%d", firstEnabled, firstAttained)
	}
	log, err := os.ReadFile(f.logPath)
	must(t, err)
	wire := regexp.MustCompile(`Native-home wire audit axis=0 available=1 .*submitted=[1-9][0-9]* failures=0 .*triggered=[1-9][0-9]* violations=0 missing_cycles=0 nonmonotonic=0 `)
	samples := regexp.MustCompile(`Native-home wire samples axis=0 token=[1-9][0-9]* .*first_cw=0x001f first_mode=6 .*last_cw=0x001f last_mode=6 `)
	prefix := regexp.MustCompile(`Native-home wire prefix axis=0 token=[1-9][0-9]* cycles=[1-9][0-9]* submitted=[1-9][0-9]* failures=0 missing_cycles=0 nonmonotonic=0 `)
	if !prefix.Match(log) {
		t.Fatalf("missing completed pre-trigger submitted-wire audit: %s", log)
	}
	for _, bucket := range []string{"cw6", "cw7", "cw15"} {
		pattern := regexp.MustCompile(`Native-home wire prefix bucket axis=0 token=[1-9][0-9]* bucket=` + bucket + ` submitted=[1-9][0-9]* mode6=[1-9][0-9]* mode8=0 other_mode=0 `)
		if !pattern.Match(log) {
			t.Fatalf("%s was not submitted only in homing mode: %s", bucket, log)
		}
	}
	if !wire.Match(log) || !samples.Match(log) {
		t.Fatalf("missing completed submitted-wire audit: %s", log)
	}
	for _, removed := range []string{"Shutdown dwell", "pretrigger settle", "CSP precondition", "Native-home diagnostic"} {
		if bytes.Contains(log, []byte(removed)) {
			t.Fatalf("Home ran a removed %q stage: %s", removed, log)
		}
	}
}
