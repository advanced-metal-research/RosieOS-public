//go:build linux

package denseexec

import (
	"bytes"
	"context"
	"errors"
	"log"
	"slices"
	"strings"
	"testing"

	"rosieos/rt-core/ipcclient"
)

func captureJournal(t *testing.T) *bytes.Buffer {
	t.Helper()
	var buffer bytes.Buffer
	previous, flags := log.Writer(), log.Flags()
	log.SetOutput(&buffer)
	log.SetFlags(0)
	t.Cleanup(func() { log.SetOutput(previous); log.SetFlags(flags) })
	return &buffer
}

func deckPress() context.Context {
	return WithClient(context.Background(), Client{ID: "deck-0b7e44d1", Label: "rosie-pendant-v5 (steamdeck-pendant)"})
}

func armSteps(result *ArmResult) []string {
	var steps []string
	for _, step := range result.Steps {
		steps = append(steps, step.Step+":"+step.Outcome)
	}
	return steps
}

// The owner's rule (2026-10-11, "re-arming the robot should clear faults too
// if they exist") in the 1430 cell's 12:19 state, J5 cooled: one Arm confirms the
// latched Stop from a fresh status, resets J5's drive alarm, then enables and
// arms, and the journal names the client, the joint, the fault and the alarm.
func TestArmConfirmsStopResetsDriveAlarmThenArms(t *testing.T) {
	r, c := recoverFixture(t)
	journal := captureJournal(t)
	result, err := r.Arm(deckPress(), true)
	if err != nil || !result.Accepted {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := recoverCalls(c, "stop", "reset_fault", "enable", "arm", "home"); !slices.Equal(got, []string{"reset_fault", "enable", "arm"}) {
		t.Fatal("want reset, enable, arm and nothing else:", got)
	}
	if got := armSteps(result); !slices.Equal(got, []string{"confirm_stop:done", "reset:done"}) {
		t.Fatal(got)
	}
	c.mu.Lock()
	armed, faults := c.status.Core.Armed, c.status.Core.SafetyFaultMask
	c.mu.Unlock()
	if armed != 1 || faults != 0 {
		t.Fatalf("armed=%d faults=%d", armed, faults)
	}
	for _, line := range []string{
		"dense-execution: arm client=rosie-pendant-v5 (steamdeck-pendant) step=confirm_stop outcome=done",
		"dense-execution: arm client=rosie-pendant-v5 (steamdeck-pendant) step=reset outcome=done detail=axes=J5 faults=drive_readiness(reset_after_condition_clears) alarms=J5 Er41.1 (Motor over-temperature due to locked- rotor); native faults cleared",
	} {
		if !strings.Contains(journal.String(), line) {
			t.Fatalf("journal lacks %q:\n%s", line, journal.String())
		}
	}
}

// J5 still hot: Arm sends one reset, native refuses it, and Arm fails with the
// drive's own words and what to do; nothing is enabled or armed, no Stop latch
// is left behind, and the journal keeps the attempt and its result.
func TestArmRefusesPersistingDriveAlarmWithItsReason(t *testing.T) {
	r, c := recoverFixture(t)
	c.persists = true
	journal := captureJournal(t)
	result, err := r.Arm(deckPress(), true)
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "fault_persists" || result.Accepted {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if failure.Detail != "J5 drive alarm Er41.1 (Motor over-temperature due to locked- rotor) still active — let it cool, then Arm again" {
		t.Fatal(failure.Detail)
	}
	if got := recoverCalls(c, motionOps...); len(got) != 0 {
		t.Fatal("motion after a refused reset:", got)
	}
	if got := recoverCalls(c, "reset_fault"); len(got) != 1 {
		t.Fatal("reset retried in one Arm:", got)
	}
	if got := armSteps(result); !slices.Equal(got, []string{"confirm_stop:done", "reset:persists", "reset:refused"}) {
		t.Fatal(got)
	}
	if body := r.Status(context.Background()); body.RTCore.StopUncertain || body.RTCore.Inhibited {
		t.Fatalf("refused reset latched recovery: %+v", body.RTCore)
	}
	if !strings.Contains(journal.String(), "step=reset outcome=persists detail=axes=J5 faults=drive_readiness(reset_after_condition_clears) alarms=J5 Er41.1") {
		t.Fatal(journal.String())
	}
}

// A fault whose recovery is a new Home, a reset that would leave a joint's
// Home invalid (untrusted evidence) or a restart: Arm refuses naming it and
// sends no reset, no enable and no Home.
func TestArmNeverResetsWhatNeedsHome(t *testing.T) {
	for _, tc := range []struct {
		name, words string
		fault       ipcclient.FaultRecovery
	}{
		{"coordinate_reference", "J1 coordinate_reference needs a new Home at the mechanical home pose (Arm never re-homes)", ipcclient.FaultRecovery{Bit: 5, Name: "coordinate_reference", AxisMask: 1, Recovery: "rehome_required", RehomeAxisMask: 1}},
		{"untrusted evidence", "J3 bus_transport lost the position reference on J3: it needs a new Home at the mechanical home pose (Arm never re-homes)", ipcclient.FaultRecovery{Bit: 8, Name: "bus_transport", AxisMask: 4, Recovery: "reset_after_condition_clears", RehomeAxisMask: 4}},
		{"restart", "J4 pdo_mapping_mismatch needs a controller restart", ipcclient.FaultRecovery{Bit: 11, Name: "pdo_mapping_mismatch", AxisMask: 8, Recovery: "restart_required"}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			r, c := recoverFixture(t)
			c.mu.Lock()
			c.status.Core.Recovery.Faults = append(c.status.Core.Recovery.Faults, tc.fault)
			c.mu.Unlock()
			_, err := r.Arm(deckPress(), true)
			var failure *Error
			if !errors.As(err, &failure) || failure.Code != "fault_not_resettable" || !strings.Contains(failure.Detail, tc.words) {
				t.Fatalf("err=%v", err)
			}
			if got := recoverCalls(c, append(motionOps, "reset_fault")...); len(got) != 0 {
				t.Fatal(got)
			}
		})
	}
}

// An unconfirmed Stop and nothing latched: Arm confirms the Stop from a
// fresh status (it sends nothing for that) and arms; no reset is sent. A Stop
// the servo host cannot confirm (another lease) keeps Arm's rt_core_inhibited
// refusal, with no reset and no enable.
func TestArmConfirmsUnconfirmedStop(t *testing.T) {
	clear := func(c *recoverClient) {
		c.mu.Lock()
		defer c.mu.Unlock()
		c.status.Core.SafetyFaultMask, c.status.Core.ExecutionFaultReasons, c.status.Core.Recovery = 0, 0, nil
		c.status.Axes[4].Readiness, c.status.Axes[4].DriveAlarm, c.status.Axes[4].DriveAlarmText = "ready", "", ""
	}
	r, c := recoverFixture(t)
	clear(c)
	result, err := r.Arm(deckPress(), true)
	if err != nil || !result.Accepted || !slices.Equal(armSteps(result), []string{"confirm_stop:done"}) {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := recoverCalls(c, "stop", "reset_fault", "enable", "arm"); !slices.Equal(got, []string{"enable", "arm"}) {
		t.Fatal(got)
	}

	r, c = recoverFixture(t)
	clear(c)
	c.foreign = true
	_, err = r.Arm(deckPress(), true)
	var failure *Error
	if !errors.As(err, &failure) || failure.Code != "rt_core_inhibited" || !strings.Contains(failure.Detail, "still shows a lease or grant") {
		t.Fatalf("err=%v", err)
	}
	if got := recoverCalls(c, append(motionOps, "reset_fault")...); len(got) != 0 {
		t.Fatal(got)
	}
}

// the second 1430 cell 2026-10-11: a disarmed cell showed J2 Er41.1 on the axis with both
// fault masks 0 and nothing in the recovery table. Arm sends the reset
// (native resets the drive only when the drive profile classes the alarm
// resettable) and arms once the drive leaves Fault; a clean robot gets no
// reset and no steps.
func TestArmResetsDriveFaultWithNothingLatched(t *testing.T) {
	r, c := recoverFixture(t)
	c.mu.Lock()
	c.status.Core.SafetyFaultMask, c.status.Core.ExecutionFaultReasons, c.status.Core.Recovery = 0, 0, nil
	c.mu.Unlock()
	r.mu.Lock()
	r.stopUncertain, r.reason = false, ""
	r.mu.Unlock()
	result, err := r.Arm(deckPress(), true)
	if err != nil || !slices.Equal(armSteps(result), []string{"reset:done"}) {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := recoverCalls(c, "reset_fault", "enable", "arm"); !slices.Equal(got, []string{"reset_fault", "enable", "arm"}) {
		t.Fatal(got)
	}

	clean, z := recoverFixture(t)
	z.mu.Lock()
	z.status.Core.SafetyFaultMask, z.status.Core.ExecutionFaultReasons, z.status.Core.Recovery = 0, 0, nil
	z.status.Axes[4].Readiness, z.status.Axes[4].DriveAlarm = "ready", ""
	z.mu.Unlock()
	clean.mu.Lock()
	clean.stopUncertain, clean.reason = false, ""
	clean.mu.Unlock()
	result, err = clean.Arm(deckPress(), true)
	if err != nil || len(result.Steps) != 0 {
		t.Fatalf("result=%+v err=%v", result, err)
	}
	if got := recoverCalls(z, "reset_fault"); len(got) != 0 {
		t.Fatal(got)
	}
}
