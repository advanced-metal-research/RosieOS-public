//go:build linux

package control

import (
	"os"
	"testing"
)

// The remote jog stream (jog_ws.go) begins a jog by calling BeginJog or
// BeginPathJog directly, not through the control dispatcher: a notice must
// refuse those calls themselves, or a lease holder could start new motion
// over the stream while an update runs.
func TestMaintenanceNoticeRefusesJogBeginsOutsideTheDispatcher(t *testing.T) {
	c, path := maintenanceController(t, maintenanceActiveNotice)
	want := ReasonBusy + ": rt-core full deploy on cell"
	if _, err := c.BeginJog(Fence{}, 1, 0, 0); err == nil || err.Error() != want {
		t.Fatalf("BeginJog during maintenance: %v", err)
	}
	if _, err := c.BeginPathJog(Fence{}, 1, 0, 0); err == nil || err.Error() != want {
		t.Fatalf("BeginPathJog during maintenance: %v", err)
	}
	if err := os.Remove(path); err != nil {
		t.Fatal(err)
	}
	// Without the notice the calls reach their own checks (this controller has
	// no local jog lane and no path jog).
	if _, err := c.BeginJog(Fence{}, 1, 0, 0); err == nil || err.Error() != "independent_jog_unavailable" {
		t.Fatalf("BeginJog without a notice: %v", err)
	}
	if _, err := c.BeginPathJog(Fence{}, 1, 0, 0); err == nil || err.Error() != ReasonPathJogUnavailable {
		t.Fatalf("BeginPathJog without a notice: %v", err)
	}
}
