//go:build linux

package faultmatrix

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	ipc "rosieos/rt-core/ipcclient"
)

// Exercise the real process and native Home worker with the shipped Rosie1430
// the 1430 cell A6 configuration. Diagnostic caps are explicit; simulated success does
// not qualify a physical brake. Repeated Home covers frame reconciliation after
// ReleaseServiceOverride as well as ordinary stationary Arm/Stop.
func TestArmJ4BrakeTransitionHomeArmStop(t *testing.T) {
	f := compileFixture(t, "simulation-rosie1420.json", func(machine map[string]any) {
		// One machine-level joint-unit guard for every axis, as in example-arm-1430.json.
		machine["brake_transition_guard"] = map[string]any{
			"max_displacement_rad": 0.0008726646259971648, "max_velocity_rad_per_s": 0.05235987755982989,
			"target_stationary_ms": 100, "settling_ms": 100, "timeout_ms": 500,
		}
	})
	f.launch(true)
	const mask uint32 = 8
	const brakeReleased, brakeApplied = 1, 2
	var previousHome uint64
	for repeat := 0; repeat < 2; repeat++ {
		f.command(f.c.Home(context.Background(), mask))
		homeDone := ipc.HostMonotonicNS()
		home := f.until(homeDone, publicationNS, func(s ipc.Status) bool {
			return s.HomeValidMask == mask && s.CommissioningPhase == 0 && s.TimeNS > homeDone+f.cycle
		})
		if home.ExecutionFaultReasons != 0 || home.HomeEpoch <= previousHome {
			t.Fatalf("Home frame failed: %+v", home)
		}
		previousHome = home.HomeEpoch
		f.command(f.c.Enable(context.Background(), mask))
		f.command(f.c.Arm(context.Background()))
		armed := f.until(ipc.HostMonotonicNS(), 2_000_000_000, func(s ipc.Status) bool {
			return s.Armed == 1 && s.AxisEnableMask == mask && s.Axes[3].Statusword&0x6f == 0x27 && s.Axes[3].BrakeState == brakeReleased
		})
		f.until(armed.TimeNS+500_000_000, publicationNS, func(s ipc.Status) bool {
			if s.ExecutionFaultReasons != 0 || s.Armed != 1 || s.AxisEnableMask != mask {
				t.Fatalf("stationary hold failed: %+v", s)
			}
			return true
		})
		stoppedAt := ipc.HostMonotonicNS()
		_, stopErr := f.c.Stop(context.Background())
		must(t, stopErr)
		stopped := f.until(stoppedAt, publicationNS+500_000_000, func(s ipc.Status) bool {
			if s.ExecutionFaultReasons != 0 {
				t.Fatalf("stop guard fault: %+v", s)
			}
			return s.Armed == 0 && s.AxisEnableMask == 0 && s.Axes[3].Statusword&0x6f != 0x27 && s.Axes[3].BrakeState == brakeApplied
		})
		if stopped.TimeNS < stoppedAt+200_000_000 || stopped.HomeEpoch != home.HomeEpoch || stopped.HomeValidMask != mask {
			t.Fatalf("closing/settling prematurely qualified or Home changed: %+v", stopped)
		}
		raw, err := os.ReadFile(filepath.Join(f.socketDir, "metrics.json"))
		must(t, err)
		var metrics struct {
			Axes []struct {
				Trip struct{ Cause, Phase string } `json:"brake_hold_diagnostic"`
			} `json:"axes"`
		}
		must(t, json.Unmarshal(raw, &metrics))
		if metrics.Axes[3].Trip.Cause != "none" {
			t.Fatalf("unexpected diagnostic: %+v", metrics.Axes[3].Trip)
		}
	}
}
