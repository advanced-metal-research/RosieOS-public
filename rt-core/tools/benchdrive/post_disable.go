//go:build linux

package main

import (
	"context"
	"errors"
	"fmt"
	"strings"

	"rosieos/rt-core/ipcclient"
	"rosieos/rt-core/sdk/control"
)

// postDisable observes the reported drive/brake transition after authority has
// been released. A timer-estimated brake state is not physical brake proof.
// Two advancing publications prevent a single boundary sample or frozen status
// from declaring completion. The existing observation deadline remains the cap.
// A brake warning or drive-supervisor event published during the run keeps the
// verdict from a clean pass (post_disable_warning_free NotMet) even though
// neither is a native fault.
func (r *runner) postDisable(ctx context.Context) error {
	completed := ipcclient.HostMonotonicNS()
	var terminalTime uint64
	terminalAxes := make([]uint64, len(r.description.Axes))
	final, err := r.observe(ctx, func(s control.Status) (bool, error) {
		if err := sessionFault(s); err != nil {
			return false, err
		}
		if !r.inhibited(s) || s.Core.LeaseValid != 0 || s.Grant == nil || s.Grant.GrantActive != 0 {
			return false, errors.New("post-disable output inhibition or lease retirement not observed")
		}
		if len(s.Core.Axes) != len(r.description.Axes) || len(s.Core.Axes) == 0 {
			return false, errors.New("post-disable axis feedback missing")
		}
		if s.Core.TimeNS < completed {
			return false, nil
		}
		for i, a := range s.Core.Axes {
			if r.enabledMask()&(1<<i) == 0 {
				continue
			}
			if a.ErrorCode != 0 || a.Statusword&0x0008 != 0 {
				return false, fmt.Errorf("post-disable axis %d drive fault", i)
			}
			if a.PDOFresh != 1 || a.TimeNS < completed || a.TimeNS > s.Core.TimeNS {
				return false, fmt.Errorf("post-disable axis %d feedback unavailable", i)
			}
			inactive := a.Statusword&0x004f == 0x0040 || a.Statusword&0x006f == 0x0021 || a.Statusword&0x006f == 0x0023
			// RELEASED is also the native state of axes with no configured brake.
			// UNKNOWN and both delay states cannot establish completion.
			if !inactive || (a.BrakeState != 1 && a.BrakeState != 2) {
				terminalTime = 0
				return false, nil
			}
		}
		if terminalTime == 0 {
			terminalTime = s.Core.TimeNS
			for i, a := range s.Core.Axes {
				terminalAxes[i] = a.TimeNS
			}
			return false, nil
		}
		if s.Core.TimeNS <= terminalTime {
			return false, nil
		}
		for i, a := range s.Core.Axes {
			if r.enabledMask()&(1<<i) == 0 {
				continue
			}
			if a.TimeNS < terminalAxes[i] {
				return false, fmt.Errorf("post-disable axis %d feedback time regressed", i)
			}
			if a.TimeNS == terminalAxes[i] {
				return false, nil
			}
		}
		return true, nil
	})
	err = errors.Join(err, r.check("post_disable_observed", err == nil))
	if final.Core.TimeNS != 0 {
		warnings := r.runWarnings(final)
		err = errors.Join(err, r.check("post_disable_warning_free", len(warnings) == 0))
		if len(warnings) != 0 {
			err = errors.Join(err, fmt.Errorf("run warned: %s", strings.Join(warnings, "; ")))
		}
	}
	return err
}

// runWarnings lists selected-axis brake warnings and supervisor events first
// published after the run's first status observation.
func (r *runner) runWarnings(s control.Status) []string {
	var warnings []string
	for i, a := range s.Core.Axes {
		if r.enabledMask()&(1<<i) == 0 {
			continue
		}
		var base warningMark
		if i < len(r.warningBaseline) {
			base = r.warningBaseline[i]
		}
		if d := a.BrakeHoldDiagnostic; d.WarningSequence > base.brake {
			warnings = append(warnings, fmt.Sprintf("axis %d brake warning %d (%s during %s): %s", i, d.WarningSequence, d.Cause, d.Phase, d.Message))
		}
		if d := a.DriveSupervisor; d.EventSequence > base.supervisor {
			warnings = append(warnings, fmt.Sprintf("axis %d drive supervisor %s: %s", i, d.Kind, d.Message))
		}
	}
	return warnings
}
